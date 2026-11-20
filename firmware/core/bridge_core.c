/*
 * Portable protocol core, milestone 1 ("keep-alive"):
 *   - power the T30 and tell the MCU (like the stock firmware)
 *   - answer the T30 hello like the stock XR871 (0x85 / 0x40 / 0x80)
 *   - track the T30 heartbeat, but never power-cycle it: a stuck T30 is
 *     only reported (debug friendly; the stock firmware cycles after 6 s)
 *   - answer the MCU heartbeat and acknowledge MCU events with {0x0e, val}
 *   - log everything on the debug UART
 * plus sleep/wake: the T30 can ask to be switched off and is powered up
 * again on PIR, keys, Wake-on-LAN or a timer (see bridge_core.h),
 * plus the service extensions (0xd0..0xdc): status, MCU passthrough, GPIO
 * test, log/MCU-event forwarding and the application update.
 */
#include <string.h>
#include "bridge_core.h"
#include "fw_update.h"
#include "spi_frame.h"

#define CORE_VERSION "abus-xr871 core 4"

static const struct bridge_hal *H;
static struct bridge_stats S;
static uint32_t wake_src;
static uint32_t last_status_ms;

enum { ST_ON, ST_GRACE, ST_OFF };
static int st;
static uint32_t grace_until, wake_at, pending_wake;
static uint32_t reset_until, last_reset_ms, stall_ref_ms, stall_frames;
static int auto_reset = 1;
static uint32_t probe_until;
static int probe_pin = -1;
static uint8_t t30_mac[6];
static int t30_mac_known;
static uint8_t t30_ip[4];	/* learnt from the T30's own IPv4 frames */
static int t30_ip_known;

static void wq_clear(void);
static void gpio_release(void);
static uint8_t fwd;		/* BR_FWD_* enabled by the T30 */
static int rx_have = -1;	/* T30 frame bytes collected, -1 = hunting magic */

/* ---- T30 side --------------------------------------------------------- */

static void t30_send(uint8_t cmd, uint16_t param, const void *payload, uint16_t len)
{
	/* static: all core entry points run under the glue's core lock */
	static uint8_t f[BR_HDR_LEN + BR_MAX_PAYLOAD];

	if (len > sizeof(f) - BR_HDR_LEN)
		len = sizeof(f) - BR_HDR_LEN;
	memset(f, 0, sizeof(f));
	f[0] = 0xaa; f[1] = 0x55; f[2] = 0xaa; f[3] = 0x55;
	f[4] = len & 0xff; f[5] = len >> 8;
	f[6] = cmd;
	f[7] = 0x01;
	f[10] = param & 0xff; f[11] = param >> 8;
	if (len)
		memcpy(f + BR_HDR_LEN, payload, len);
	H->t30_write(f, BR_HDR_LEN + len);
}

void bridge_t30_send(uint8_t cmd, uint16_t param, const void *payload, uint16_t len)
{
	t30_send(cmd, param, payload, len);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static void mcu_send(uint8_t cmd, uint8_t val);

/* ---- extensions -------------------------------------------------------- */

static void send_status(void)
{
	uint8_t p[BR_STATUS_LEN];
	uint32_t now = H->millis();
	int pct = -1, mv = -1, linked = 0;
	int8_t rssi = 0;
	uint8_t mac[6], st, run, ota;
	const uint32_t c[] = {
		S.t30_frames, S.t30_bad, S.t30_hb, S.mcu_rx, S.mcu_tx, S.mcu_tx_err,
		S.spi_frames, S.spi_bad, S.spi_eth_up, S.spi_eth_down, S.wifi_drop,
		S.log_dropped, S.sleeps, S.wakes, S.log_lines, S.mcu_events_fwd
	};
	unsigned i;

	memset(p, 0, sizeof(p));
	if (H->battery && H->battery(&pct, &mv) < 0)
		pct = mv = -1;
	if (H->wifi_status)
		linked = H->wifi_status(mac, &rssi) > 0;
	fwu_summary(&st, &run, &ota);
	p[0] = BR_STATUS_VERSION;
	p[1] = (H->wifi_status ? 0x01 : 0) | (linked ? 0x02 : 0) |
	       (pct >= 0 ? 0x04 : 0) | (S.t30_on ? 0x08 : 0);
	p[2] = pct >= 0 && pct <= 100 ? pct : 0xff;
	p[3] = (uint8_t)rssi;
	p[4] = mv > 0 ? mv & 0xff : 0;
	p[5] = mv > 0 ? (mv >> 8) & 0xff : 0;
	p[6] = S.mcu_version;
	p[7] = st;
	put_le32(p + 8, now);
	put_le32(p + 12, S.last_wake);
	for (i = 0; i < sizeof(c) / sizeof(c[0]); i++)
		put_le32(p + 16 + 4 * i, c[i]);
	/* 16 + 64 = 80 */
	p[80] = S.mcu_state1;
	p[81] = S.mcu_state2;
	p[82] = S.mcu_usb;
	p[83] = S.mcu_mainbat;
	p[84] = S.mcu_seen;
	p[85] = run;
	p[86] = ota;
	p[87] = fwd;
	/* 88..151: version "<core> <glue>", NUL padded */
	{
		const char *v = H->version ? H->version : "";
		size_t n = strlen(CORE_VERSION), m = strlen(v);

		memcpy(p + 88, CORE_VERSION, n);
		if (m) {
			if (m > BR_STATUS_LEN - 88 - n - 2)
				m = BR_STATUS_LEN - 88 - n - 2;
			p[88 + n] = ' ';
			memcpy(p + 89 + n, v, m);
		}
	}
	t30_send(T30_STATUS, BR_STATUS_VERSION, p, sizeof(p));
}

/* MCU messages the T30 may only send with the force flag: they would
 * switch the T30 itself (or the whole board) off */
static int mcu_dangerous(uint8_t cmd, uint8_t val)
{
	return (cmd == MCU_DSP_POWER && val != 0x07) || cmd == MCU_LOW_POWER;
}

static void mcu_passthrough(uint16_t param, const uint8_t *p, uint16_t plen)
{
	uint8_t cmd = param & 0xff, val = param >> 8, out[2] = { cmd, val };
	int force = plen >= 1 && (p[0] & 1);
	uint16_t r;

	if (mcu_dangerous(cmd, val) && !force) {
		H->log("mcu: passthrough {0x%02x,0x%02x} refused (needs force)\n", cmd, val);
		r = BR_E_DENIED;
	} else {
		uint32_t err = S.mcu_tx_err;

		H->log("mcu: passthrough {0x%02x,0x%02x}%s\n", cmd, val, force ? " (forced)" : "");
		mcu_send(cmd, val);
		r = S.mcu_tx_err != err ? BR_E_FAIL : 0;
	}
	t30_send(T30_MCU, r, out, 2);
}

/* ---- GPIO test ---- */

static struct {
	int active, port, pin, blink, level;
	uint32_t until, next;
	uint16_t half;
} G;

/* never driven, whatever the T30 asks: UART0 (console) PB0/1, flash PB2-7,
 * I2C0 PA4/5 (stock pinmux), MCU IRQ PA7, I2C1 PA8/9 (MCU + PMIC),
 * T30 power PA16, UART1 (T30) PA17/18, SPI1 PA19-22; XR871 has PA0-22,
 * PB0-7 */
int bridge_gpio_protected(int port, int pin)
{
	if (port != 0 || pin < 0 || pin > 22)
		return 1;
	switch (pin) {
	case 4: case 5: case 7: case 8: case 9:
	case 16: case 17: case 18: case 19: case 20: case 21: case 22:
		return 1;
	}
	return 0;
}

static void gpio_release(void)
{
	if (!G.active)
		return;
	G.active = 0;
	if (H->gpio_test)
		H->gpio_test(G.port, G.pin, BR_GPIO_RELEASE);
	H->log("gpio: P%c%d released\n", 'A' + G.port, G.pin);
}

static void gpio_cmd(const uint8_t *p, uint16_t plen)
{
	int port, pin, mode, r = -1;
	uint32_t ms;
	uint16_t half;
	uint8_t echo[3];

	if (plen < 3) {
		t30_send(T30_GPIO, BR_E_FAIL, NULL, 0);
		return;
	}
	port = p[0];
	pin = p[1];
	mode = p[2];
	ms = plen >= 5 ? (uint32_t)(p[3] | p[4] << 8) : 0;
	half = plen >= 7 ? p[5] | p[6] << 8 : 0;
	if (!ms)
		ms = 30000;
	if (half < 50)
		half = 250;
	memcpy(echo, p, 3);
	if (bridge_gpio_protected(port, pin) || !H->gpio_test || mode > BR_GPIO_RELEASE) {
		H->log("gpio: P%c%d mode %d refused\n", 'A' + (port & 1), pin, mode);
		t30_send(T30_GPIO, BR_E_DENIED, echo, 3);
		return;
	}
	if (G.active && (G.port != port || G.pin != pin))
		gpio_release();		/* one test pin at a time */
	switch (mode) {
	case BR_GPIO_RELEASE:
		if (G.active)
			gpio_release();
		else
			H->gpio_test(port, pin, BR_GPIO_RELEASE);
		r = 0;
		break;
	case BR_GPIO_READ:
		G.active = 0;
		r = H->gpio_test(port, pin, BR_GPIO_READ);
		break;
	case BR_GPIO_LOW:
	case BR_GPIO_HIGH:
	case BR_GPIO_BLINK:
		r = H->gpio_test(port, pin, mode == BR_GPIO_BLINK ? BR_GPIO_HIGH : mode);
		if (r >= 0) {
			G.active = 1;
			G.port = port;
			G.pin = pin;
			G.blink = mode == BR_GPIO_BLINK;
			G.level = mode != BR_GPIO_LOW;
			G.half = half;
			G.until = H->millis() + ms;
			G.next = H->millis() + half;
			r = 0;
		}
		break;
	}
	H->log("gpio: P%c%d mode %d -> %d\n", 'A' + port, pin, mode, r);
	t30_send(T30_GPIO, r < 0 ? BR_E_FAIL : r, echo, 3);
}

static void gpio_poll(uint32_t now)
{
	if (!G.active)
		return;
	if ((int32_t)(now - G.until) >= 0) {
		gpio_release();		/* never leave a test level behind */
		return;
	}
	if (G.blink && (int32_t)(now - G.next) >= 0) {
		G.level = !G.level;
		G.next = now + G.half;
		H->gpio_test(G.port, G.pin, G.level ? BR_GPIO_HIGH : BR_GPIO_LOW);
	}
}

/* ---- log forwarding ---- */

#define LOGQ_SIZE 2048
static uint8_t logq[LOGQ_SIZE];
static unsigned logq_head, logq_used;	/* records: u8 len, bytes */
static uint32_t logq_dropped, log_tokens, log_refill_ms;
static uint16_t log_seq;

static int ci_has(const char *s, int n, const char *w)
{
	int i, j;

	for (i = 0; i < n; i++) {
		for (j = 0; w[j] && i + j < n; j++) {
			char c = s[i + j];

			if (c >= 'A' && c <= 'Z')
				c += 'a' - 'A';
			if (c != w[j])
				break;
		}
		if (!w[j])
			return 1;
	}
	return 0;
}

void bridge_log_capture(const char *line, int len)
{
	static const char withheld[] = "[log line withheld: credential keyword]";
	unsigned i, t;

	while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
		len--;
	if (len <= 0)
		return;
	/* the core never logs secrets; this guards against future mistakes */
	/* also WPA handshake debug (hexdumps, key/nonce material) and network
	 * names: the owner's rule is no SSIDs or keys in any log */
	if (ci_has(line, len, "psk") || ci_has(line, len, "passw") ||
	    ci_has(line, len, "passphr") || ci_has(line, len, "pmk") ||
	    ci_has(line, len, "secret") || ci_has(line, len, "ssid") ||
	    ci_has(line, len, "hexdump") || ci_has(line, len, "ptk") ||
	    ci_has(line, len, "gtk") || ci_has(line, len, "nonce") ||
	    ci_has(line, len, "kck") || ci_has(line, len, "kek") ||
	    ci_has(line, len, "key")) {
		line = withheld;
		len = sizeof(withheld) - 1;
	}
	if (len > BR_LOG_MAX)
		len = BR_LOG_MAX;
	if (H && H->logq_lock)
		H->logq_lock(1);
	if (logq_used + 1 + len > LOGQ_SIZE) {
		logq_dropped++;		/* drop the newest: keeps the boot log */
	} else {
		t = (logq_head + logq_used) % LOGQ_SIZE;
		logq[t] = len;
		for (i = 0; i < (unsigned)len; i++)
			logq[(t + 1 + i) % LOGQ_SIZE] = line[i];
		logq_used += 1 + len;
	}
	if (H && H->logq_lock)
		H->logq_lock(0);
}

/*
 * At most 10 lines/s (burst 20) and one line per poll. The glue only has
 * the UART receive interrupt enabled while it waits in its receive call,
 * so while we transmit, T30 bytes pile up in the hardware FIFO: keep each
 * transmit short and never start one while a T30 frame is half received
 * or an update transfer is running (lines wait in the queue meanwhile).
 */
static void log_drain(uint32_t now)
{
	char line[BR_LOG_MAX + 40];
	int n, i, k;
	uint8_t fst, frun, fota;

	if (now - log_refill_ms >= 100) {
		log_tokens += (now - log_refill_ms) / 100;
		log_refill_ms = now;
		if (log_tokens > 20)
			log_tokens = 20;
	}
	fwu_summary(&fst, &frun, &fota);
	if (rx_have >= 0 || fst == FWU_RECEIVING)
		return;
	for (k = 0; k < 1 && log_tokens && st == ST_ON && (fwd & BR_FWD_LOG); k++) {
		uint32_t dropped;

		if (H->logq_lock)
			H->logq_lock(1);
		dropped = 0;
		n = 0;
		if (logq_used) {
			n = logq[logq_head];
			for (i = 0; i < n; i++)
				line[i] = logq[(logq_head + 1 + i) % LOGQ_SIZE];
			logq_head = (logq_head + 1 + n) % LOGQ_SIZE;
			logq_used -= 1 + n;
		} else {
			/* dropped lines came after everything that was queued */
			dropped = logq_dropped;
			logq_dropped = 0;
		}
		if (H->logq_lock)
			H->logq_lock(0);
		if (dropped) {
			S.log_dropped += dropped;
			memcpy(line, "[xr871: ", 8);
			n = 8;
			/* small itoa, no printf in the core */
			{
				char d[11];
				int m = 0;

				do {
					d[m++] = '0' + dropped % 10;
					dropped /= 10;
				} while (dropped);
				while (m)
					line[n++] = d[--m];
			}
			memcpy(line + n, " log lines dropped]", 19);
			n += 19;
		}
		if (!n)
			break;
		t30_send(XR_LOG, log_seq++, line, n);
		S.log_lines++;
		log_tokens--;
	}
}

static void mcu_forward(uint8_t cmd, uint8_t val)
{
	static uint32_t win_ms, win_n;
	uint32_t now = H->millis();
	uint8_t d = 0;

	if (!(fwd & BR_FWD_MCU) || st != ST_ON)
		return;
	if (now - win_ms >= 1000) {
		win_ms = now;
		win_n = 0;
	}
	if (++win_n > 20)
		return;		/* storm: at most 20 events/s */
	S.mcu_events_fwd++;
	t30_send(XR_MCU_EVENT, cmd | val << 8, &d, 1);
}

static void t30_handle(const uint8_t *b, int len)
{
	uint8_t cmd = b[6];
	uint16_t plen = b[4] | (b[5] << 8);
	uint16_t param = len >= BR_HDR_LEN ? (b[10] | (b[11] << 8)) : 0;
	uint32_t now = H->millis();
	uint8_t p[8];

	S.t30_frames++;
	switch (cmd) {
	case T30_HELLO:
		S.t30_hello++;
		fwd = 0;	/* a (re)started T30 application opts in again */
		H->log("t30: hello (+%lu ms after power-on)\n",
		       (unsigned long)(now - S.t30_power_on_ms));
		put_le32(p, wake_src);
		t30_send(XR_WAKE_SRC, 0, p, 4);
		/* no RTC/NTP yet: time 0, the T30 ignores values before 2020 */
		memset(p, 0, 8);
		t30_send(XR_TIME, 0, p, 8);
		put_le32(p, 0);
		t30_send(XR_INFO, 0, p, 4);
		break;
	case T30_HEARTBEAT:
		if (!S.t30_hb) {
			S.t30_first_hb_ms = now;
			H->log("t30: first heartbeat after %lu ms\n",
			       (unsigned long)(now - S.t30_power_on_ms));
		}
		S.t30_hb++;
		S.t30_last_hb_ms = now;
		break;
	case T30_POWER:
		if (param == 0 && st == ST_ON) {
			uint32_t secs = 0;

			if (plen >= 4 && len >= BR_HDR_LEN + 4)
				secs = b[12] | b[13] << 8 | b[14] << 16 | (uint32_t)b[15] << 24;
			wake_at = secs ? now + secs * 1000 : 0;
			if (wake_at == 0 && secs)
				wake_at = 1;
			st = ST_GRACE;
			grace_until = now + BR_SLEEP_GRACE_MS;
			pending_wake = 0;
			t30_send(XR_POWER, BR_SLEEP_GRACE_MS, NULL, 0);
			H->log("t30: sleep request, timer %lu s, power off in %u ms\n",
			       (unsigned long)secs, BR_SLEEP_GRACE_MS);
		} else if (param == 1 && st == ST_GRACE) {
			st = ST_ON;
			H->log("t30: sleep request withdrawn\n");
		} else {
			H->log("t30: power request %u in state %d ignored\n", param, st);
		}
		break;
	case T30_WIFI_CFG: {
		int r = -1;

		if (plen >= 1 + 32 + 1 && b[12] <= 32 && b[12] > 0 && H->wifi_config) {
			uint8_t psk_len = b[12 + 33];

			if (psk_len <= 64 && plen >= 1 + 32 + 1 + psk_len)
				r = H->wifi_config(b + 13, b[12], b + 12 + 34, psk_len);
		}
		/* never log the credentials themselves */
		H->log("t30: wifi config (ssid %u bytes) -> %d\n", b[12], r);
		t30_send(T30_WIFI_CFG, (uint16_t)(r < 0 ? 0xffff : r), NULL, 0);
		break;
	}
	case T30_WIFI_AP:
	case T30_WIFI_STA:
	case T30_WIFI_SCAN: {
		int op = cmd == T30_WIFI_AP ? BR_WIFI_OP_AP :
			 cmd == T30_WIFI_STA ? BR_WIFI_OP_STA : BR_WIFI_OP_SCAN;
		const uint8_t *ssid = NULL, *psk = NULL;
		int ssid_len = 0, psk_len = 0, r = -1;

		if (op == BR_WIFI_OP_AP) {
			if (plen >= 1 + 32 + 1 && b[12] > 0 && b[12] <= 32) {
				ssid = b + 13;
				ssid_len = b[12];
				psk_len = b[12 + 33];
				psk = b + 12 + 34;
				if (psk_len > 64 || plen < 1 + 32 + 1 + psk_len ||
				    (psk_len && psk_len < 8))
					ssid_len = 0;
			}
			if (!ssid_len)
				op = -1;
		}
		if (op >= 0 && H->wifi_mode)
			r = H->wifi_mode(op, ssid, ssid_len, psk, psk_len);
		H->log("t30: wifi op %d -> %d\n", op, r);
		t30_send(cmd, (uint16_t)(r < 0 ? 0xffff : 0), NULL, 0);
		break;
	}
	case T30_PROBE_PIN: {
		int pin = param & 0xff, lvl = (param >> 8) & 1, r = -1;

		if (probe_pin >= 0 && probe_pin != pin)
			H->probe_pin(probe_pin, 2);	/* previous candidate back */
		probe_pin = -1;
		if (H->probe_pin)
			r = H->probe_pin(pin, lvl);
		if (r >= 0) {
			/* hold the level (also low: the pin's normal function might
			 * drive it high) until the next probe, 0xc5 or the timeout */
			probe_pin = pin;
			probe_until = now + BR_PROBE_MS;
		}
		t30_send(T30_PROBE_PIN, (uint16_t)(r < 0 ? 0xffff : param), NULL, 0);
		break;
	}
	case T30_READY_PIN: {
		int r;

		if (probe_pin >= 0) {
			H->probe_pin(probe_pin, 2);
			probe_pin = -1;
		}
		r = H->set_ready_pin ? H->set_ready_pin(param) : -1;

		H->log("t30: ready line = PA%u -> %d\n", param, r);
		t30_send(T30_READY_PIN, (uint16_t)(r < 0 ? 0xffff : param), NULL, 0);
		break;
	}
	case T30_STATUS:
		send_status();
		break;
	case T30_MCU:
		mcu_passthrough(param, b + BR_HDR_LEN, plen);
		break;
	case T30_GPIO:
		gpio_cmd(b + BR_HDR_LEN, plen);
		break;
	case T30_FWD:
		fwd = param & (BR_FWD_LOG | BR_FWD_MCU);
		H->log("t30: forwarding log %s, mcu events %s\n",
		       fwd & BR_FWD_LOG ? "on" : "off", fwd & BR_FWD_MCU ? "on" : "off");
		t30_send(T30_FWD, fwd, NULL, 0);
		break;
	case FWU_CMD_BEGIN:
	case FWU_CMD_DATA:
	case FWU_CMD_VERIFY:
	case FWU_CMD_COMMIT:
	case FWU_CMD_INFO: {
		uint8_t o[32];
		uint16_t olen, r;

		r = fwu_cmd(cmd, param, b + BR_HDR_LEN, plen, o, &olen);
		t30_send(cmd, r, o, olen);
		if (fwu_take_reboot() && H->reboot) {
			H->log("fwu: reboot requested by the T30\n");
			H->reboot();
		}
		break;
	}
	case T30_MARKER:
		/* tell xr871d it talks to this firmware, not the stock one */
		t30_send(T30_MARKER, 2, "abus-xr871", 10);
		S.t30_marker_seen = 1;
		H->log("t30: marker \"%.*s\"\n", plen > 32 ? 32 : plen,
		       (const char *)b + BR_HDR_LEN);
		break;
	default:
		H->log("t30: cmd 0x%02x param %u len %u\n", cmd, param, plen);
		break;
	}
}

void bridge_t30_rx(const uint8_t *in, int n)
{
	static uint8_t buf[BR_HDR_LEN + BR_MAX_PAYLOAD];
	static uint32_t acc, last_ms;
	uint32_t now = H->millis();
	int i;

	/* a frame arrives in one burst at 115200 baud (<= 23 ms); a gap
	 * means bytes were lost: drop the partial frame */
	if (rx_have >= 0 && now - last_ms > 200)
		rx_have = -1;
	last_ms = now;
	for (i = 0; i < n; i++) {
		uint8_t c = in[i];

		acc = (acc >> 8) | ((uint32_t)c << 24);
		/* the magic restarts a frame only outside the payload: binary
		 * payloads (firmware update data) may contain aa 55 aa 55 */
		if (acc == 0x55aa55aaU && rx_have < BR_HDR_LEN) {
			buf[0] = 0xaa; buf[1] = 0x55; buf[2] = 0xaa; buf[3] = 0x55;
			rx_have = 4;
			continue;
		}
		if (rx_have < 0)
			continue;
		buf[rx_have++] = c;
		if (rx_have >= 6) {
			uint16_t plen = buf[4] | (buf[5] << 8);
			if (plen > BR_MAX_PAYLOAD) {
				S.t30_bad++;
				rx_have = -1;
			} else if (rx_have == BR_HDR_LEN + plen) {
				t30_handle(buf, rx_have);
				rx_have = -1;
			}
		}
	}
}

/* ---- MCU side --------------------------------------------------------- */

static void mcu_send(uint8_t cmd, uint8_t val)
{
	if (H->mcu_send(cmd, val) == 0)
		S.mcu_tx++;
	else {
		S.mcu_tx_err++;
		H->log("mcu: send {0x%02x,0x%02x} failed\n", cmd, val);
	}
}

void bridge_mcu_rx(uint8_t cmd, uint8_t val)
{
	S.mcu_rx++;
	switch (cmd) {
	case 0x00:
		return;			/* empty read */
	case MCU_HEARTBEAT:
		S.mcu_hb++;
		mcu_send(MCU_HEARTBEAT, val);
		mcu_forward(cmd, val);
		return;
	case MCU_Q_VERSION:
		S.mcu_version = val;
		H->log("mcu: software version %u\n", val);
		mcu_forward(cmd, val);
		return;
	case MCU_Q_STATE1:
	case MCU_Q_STATE2:
		if (cmd == MCU_Q_STATE1) {
			/* answer to the boot query {0x1e}: USB state (0 = no
			 * USB), the same meaning as event 0x0c (stock
			 * FUN_00013d6c) - known at boot, not only after the
			 * first plug change */
			S.usb = val ? 1 : 0;
			S.mcu_usb = val;
			S.mcu_state1 = val;
			S.mcu_seen |= 1 | 4;
		} else {
			S.mcu_state2 = val;
			S.mcu_seen |= 2;
		}
		H->log("mcu: state 0x%02x = 0x%02x\n", cmd, val);
		mcu_forward(cmd, val);
		return;
	default:
		if (cmd == MCU_EV_USB) {
			S.mcu_usb = val;
			S.mcu_seen |= 4;
		} else if (cmd == MCU_EV_MAINBAT) {
			S.mcu_mainbat = val;
			S.mcu_seen |= 8;
		}
		mcu_forward(cmd, val);
		/* events: 0x0d PIR, 0x12/0x13 keys, 0x0c USB, 0x2d battery,
		 * 0x08 tamper ... - acknowledged like the stock firmware */
		H->log("mcu: event 0x%02x val 0x%02x\n", cmd, val);
		mcu_send(MCU_HEARTBEAT, val);
		if (cmd == MCU_EV_USB)
			S.usb = val ? 1 : 0;
		if (cmd == MCU_EV_PIR) {
			/* tell a running T30 (night light); a sleeping one is woken
			 * with wake source 0x101 and handles that itself */
			if (st == ST_ON && !reset_until)
				t30_send(XR_PIR, val, NULL, 0);
			bridge_wake(BR_WAKE_PIR);
		}
		else if (cmd == MCU_EV_KEY1 || cmd == MCU_EV_KEY2)
			bridge_wake(BR_WAKE_KEY);
		return;
	}
}

/* ---- lifecycle -------------------------------------------------------- */

void bridge_init(const struct bridge_hal *hal, uint32_t wake_source)
{
	H = hal;
	memset(&S, 0, sizeof(S));
	S.usb = -1;
	wake_src = wake_source;
	fwu_init(hal);
	H->log("abus-xr871 keepalive firmware, milestone 1 (%s%s%s)\n", CORE_VERSION,
	       H->version ? ", " : "", H->version ? H->version : "");

	mcu_send(MCU_Q_STATE1, 0);
	mcu_send(MCU_Q_STATE2, 0);
	mcu_send(MCU_Q_VERSION, 0);

	H->t30_power(1);
	S.t30_power_on_ms = H->millis();
	S.t30_on = 1;
	S.last_wake = wake_source;
	st = ST_ON;
	mcu_send(MCU_DSP_POWER, 0x07);
	H->log("t30: powered on\n");
	last_status_ms = S.t30_power_on_ms;
}

static void t30_off(void)
{
	gpio_release();
	H->t30_power(0);
	mcu_send(MCU_DSP_POWER, 0x09);
	st = ST_OFF;
	fwd = 0;
	S.t30_on = 0;
	S.sleeps++;
	wq_clear();
	H->log("t30: powered off (sleep #%lu)\n", (unsigned long)S.sleeps);
	if (pending_wake) {
		/* an event arrived during the grace time: clean reboot */
		wake_at = H->millis() + 500;
		if (!wake_at)
			wake_at = 1;
	} else if (H->low_power) {
		H->low_power(1);
	}
}

void bridge_wake(uint32_t reason)
{
	if (st == ST_ON)
		return;
	if (st == ST_GRACE) {
		if (!pending_wake)
			pending_wake = reason;
		return;
	}
	if (H->low_power)
		H->low_power(0);
	if (pending_wake) {
		reason = pending_wake;
		pending_wake = 0;
	}
	wake_at = 0;
	wake_src = reason;
	fwd = 0;
	S.wakes++;
	S.last_wake = reason;
	S.t30_hb = 0;
	S.t30_power_on_ms = H->millis();
	H->t30_power(1);
	S.t30_on = 1;
	st = ST_ON;
	mcu_send(MCU_DSP_POWER, 0x07);
	H->log("t30: woken, reason 0x%03lx\n", (unsigned long)reason);
}

void bridge_t30_auto(int on)
{
	auto_reset = on;
	H->log("t30: automatic reset %s\n", on ? "on" : "off");
}

void bridge_t30_reset(const char *why)
{
	uint32_t now = H->millis();

	if (st != ST_ON || reset_until)
		return;
	H->log("t30: power-cycle (%s)\n", why);
	H->t30_power(0);
	reset_until = now + BR_RESET_OFF_MS;
	if (!reset_until)
		reset_until = 1;
	last_reset_ms = now;
	S.t30_resets++;
	wq_clear();
}

/* T30 on and past boot, but no SPI traffic or no heartbeat for too long */
static void t30_watch(uint32_t now)
{
	if (st != ST_ON || reset_until) {
		stall_ref_ms = now;
		stall_frames = S.spi_frames;
		return;
	}
	if (S.spi_frames != stall_frames) {
		stall_frames = S.spi_frames;
		stall_ref_ms = now;
	}
	if (!auto_reset || now - S.t30_power_on_ms < BR_STALL_MS)
		return;
	if (last_reset_ms && now - last_reset_ms < BR_RESET_HOLDOFF_MS)
		return;
	if (now - stall_ref_ms >= BR_STALL_MS)
		bridge_t30_reset("no SPI frames");
	else if (S.t30_hb && now - S.t30_last_hb_ms >= BR_STALL_MS)
		bridge_t30_reset("no heartbeat");
	else if (!S.t30_hb && S.t30_frames == 0)
		bridge_t30_reset("T30 never talked");
}

void bridge_poll(void)
{
	uint32_t now = H->millis();

	if (reset_until && (int32_t)(now - reset_until) >= 0) {
		reset_until = 0;
		S.t30_hb = 0;
		S.t30_power_on_ms = now;
		H->t30_power(1);
		H->log("t30: powered on again after reset\n");
	}
	t30_watch(now);

	if (probe_pin >= 0 && (int32_t)(now - probe_until) >= 0) {
		H->probe_pin(probe_pin, 2);	/* safety: release, never leave a pin high */
		probe_pin = -1;
	}
	if (st == ST_GRACE && (int32_t)(now - grace_until) >= 0)
		t30_off();
	if (st == ST_OFF && wake_at && (int32_t)(now - wake_at) >= 0)
		bridge_wake(BR_WAKE_TIMER);
	gpio_poll(now);
	log_drain(now);
	if (now - last_status_ms < (st == ST_OFF ? 60000u : 10000u))
		return;
	last_status_ms = now;
	H->log("status: t30 frames %lu hb %lu (last %lu ms ago) hello %lu bad %lu marker %d | "
	       "mcu rx %lu tx %lu err %lu hb %lu ver %u\n",
	       (unsigned long)S.t30_frames, (unsigned long)S.t30_hb,
	       (unsigned long)(S.t30_hb ? now - S.t30_last_hb_ms : 0),
	       (unsigned long)S.t30_hello, (unsigned long)S.t30_bad, S.t30_marker_seen,
	       (unsigned long)S.mcu_rx, (unsigned long)S.mcu_tx,
	       (unsigned long)S.mcu_tx_err, (unsigned long)S.mcu_hb, S.mcu_version);
	H->log("status: spi frames %lu bad %lu eth up %lu down %lu wifi-drop %lu\n",
	       (unsigned long)S.spi_frames, (unsigned long)S.spi_bad,
	       (unsigned long)S.spi_eth_up, (unsigned long)S.spi_eth_down,
	       (unsigned long)S.wifi_drop);
	if (st == ST_ON && S.t30_hb && now - S.t30_last_hb_ms > 6000)
		H->log("WARN: no T30 heartbeat for %lu ms (stock would power-cycle; we don't)\n",
		       (unsigned long)(now - S.t30_last_hb_ms));
}

/* ---- SPI data path ---------------------------------------------------- */

#define WQ_LEN 8
static uint8_t wq[WQ_LEN][SPIF_MAX_PAYLOAD];
static uint16_t wq_len[WQ_LEN];
static unsigned wq_head, wq_count;
static uint16_t spi_seq;

static void wq_clear(void)
{
	wq_head = wq_count = 0;
}

/* Wake-on-LAN magic packet: 6 x 0xff followed by 16 x the target MAC,
 * anywhere in the frame (UDP port 9/7 or raw ethertype 0x0842) */
static int is_magic(const uint8_t *p, int len)
{
	int i, j, k;

	if (!t30_mac_known)
		return 0;
	for (i = 0; i + 102 <= len; i++) {
		for (j = 0; j < 6 && p[i + j] == 0xff; j++)
			;
		if (j < 6)
			continue;
		for (k = 0; k < 16; k++)
			if (memcmp(p + i + 6 + k * 6, t30_mac, 6))
				break;
		if (k == 16)
			return 1;
	}
	return 0;
}

/*
 * Standby helpers (T30 off): answer ARP for the T30's IP so it stays
 * reachable, and wake the T30 on a TCP connection attempt to one of its
 * services. The client retransmits its SYN and gets in once the T30 runs.
 */
static void standby_arp_reply(const uint8_t *req)
{
	uint8_t r[42];

	memcpy(r, req + 6, 6);			/* to the asker */
	memcpy(r + 6, t30_mac, 6);
	r[12] = 0x08; r[13] = 0x06;
	r[14] = 0; r[15] = 1; r[16] = 0x08; r[17] = 0; r[18] = 6; r[19] = 4;
	r[20] = 0; r[21] = 2;			/* reply */
	memcpy(r + 22, t30_mac, 6);
	memcpy(r + 28, t30_ip, 4);
	memcpy(r + 32, req + 22, 6);		/* asker's MAC + IP */
	memcpy(r + 38, req + 28, 4);
	if (H->wifi_tx)
		H->wifi_tx(r, sizeof(r));
}

static int standby_wake_port(unsigned port)
{
	return port == 22 || port == 80 || port == 443 || port == 554 || port == 8880;
}

static void standby_rx(const uint8_t *eth, int len)
{
	unsigned ihl, port;

	if (is_magic(eth, len)) {
		H->log("wifi: wake-on-lan packet\n");
		bridge_wake(BR_WAKE_WLAN);
		return;
	}
	if (!t30_mac_known || !t30_ip_known || len < 42)
		return;
	if (eth[12] == 0x08 && eth[13] == 0x06 && eth[21] == 1 &&
	    !memcmp(eth + 38, t30_ip, 4)) {
		standby_arp_reply(eth);
		return;
	}
	if (eth[12] != 0x08 || eth[13] != 0x00 || eth[23] != 6 ||
	    memcmp(eth + 30, t30_ip, 4))
		return;
	ihl = (eth[14] & 0x0f) * 4;
	if (len < (int)(14 + ihl + 14))
		return;
	port = eth[14 + ihl + 2] << 8 | eth[14 + ihl + 3];
	/* SYN without ACK = a new connection */
	if ((eth[14 + ihl + 13] & 0x12) == 0x02 && standby_wake_port(port)) {
		H->log("wifi: connection to port %u, waking the T30\n", port);
		bridge_wake(BR_WAKE_WLAN);
	}
}

void bridge_wifi_rx(const uint8_t *eth, int len)
{
	unsigned t;

	if (st != ST_ON) {
		if (st == ST_OFF)
			standby_rx(eth, len);
		else if (is_magic(eth, len))
			bridge_wake(BR_WAKE_WLAN);
		return;		/* nobody to deliver to */
	}

	if (len <= 0 || len > SPIF_MAX_PAYLOAD || wq_count == WQ_LEN) {
		S.wifi_drop++;
		return;
	}
	t = (wq_head + wq_count) % WQ_LEN;
	memcpy(wq[t], eth, len);
	wq_len[t] = len;
	wq_count++;
}

void bridge_spi_rx(const uint8_t *frame, int len)
{
	uint8_t type, flags;
	int n = spif_parse(frame, len, &type, &flags);

	S.spi_frames++;
	if (n < 0) {
		S.spi_bad++;
		return;
	}
	if (type == SPIF_ETH && n >= 14 && memcmp(t30_mac, frame + SPIF_HDR + 6, 6)) {
		memcpy(t30_mac, frame + SPIF_HDR + 6, 6);
		t30_mac_known = 1;
		H->log("spi: T30 wlan0 MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
		       t30_mac[0], t30_mac[1], t30_mac[2], t30_mac[3], t30_mac[4], t30_mac[5]);
	}
	if (type == SPIF_ETH && n >= 34 && frame[SPIF_HDR + 12] == 0x08 &&
	    frame[SPIF_HDR + 13] == 0x00 && frame[SPIF_HDR + 26] != 0 &&
	    memcmp(t30_ip, frame + SPIF_HDR + 26, 4)) {
		memcpy(t30_ip, frame + SPIF_HDR + 26, 4);
		t30_ip_known = 1;
		H->log("spi: T30 IP %u.%u.%u.%u\n", t30_ip[0], t30_ip[1], t30_ip[2], t30_ip[3]);
	}
	if (type == SPIF_ETH && n > 0 && H->wifi_tx) {
		S.spi_eth_up++;
		H->wifi_tx(frame + SPIF_HDR, n);
	}
}

int bridge_spi_next(uint8_t *frame)
{
	if (!wq_count) {
		uint8_t info[SPIF_INFO_LEN];
		int8_t rssi = 0;

		if (H->wifi_status) {
			memset(info, 0, sizeof(info));
			info[6] = H->wifi_status(info, &rssi) ? 1 : 0;
			info[7] = (uint8_t)rssi;
			spif_build(frame, SPIF_IDLE, 0, spi_seq++, info, sizeof(info));
		} else {
			spif_build(frame, SPIF_IDLE, 0, spi_seq++, NULL, 0);
		}
		return 0;
	}
	spif_build(frame, SPIF_ETH, wq_count > 1 ? SPIF_F_MORE : 0, spi_seq++,
		   wq[wq_head], wq_len[wq_head]);
	wq_head = (wq_head + 1) % WQ_LEN;
	wq_count--;
	S.spi_eth_down++;
	return 1;
}

const struct bridge_stats *bridge_stats(void)
{
	return &S;
}
