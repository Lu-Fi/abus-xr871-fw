/*
 * Host emulation of the XR871 firmware: runs the unmodified protocol core
 * (core/bridge_core.c) on the PC.
 *
 *   T30 UART: UART0 of the Ingenic T30 QEMU (unix socket, -serial unix:...)
 *   T30 SPI:  optional, the spi-frame-peer device on the QEMU ssi0 bus
 *             (chardev socket, u16 length + frame in both directions)
 *   WiFi:     Ethernet frames to/from tools/netsim.py over UDP 127.0.0.1:47871
 *   MCU:      simulated in-process - heartbeat {0x0e,..} every 2 s, a PIR
 *             event {0x0d,1} at 15 s, checks the answers
 *   Power:    optional QMP socket - T30 rail off = "stop", back on =
 *             "system_reset" + "cont", so sleep/wake runs the real reboot
 *   WiFi STA: fixed station MAC 02:87:1e:00:00:01, always associated
 *   Ready:    the simulated board wires the SPI ready line to PA14, while
 *             the firmware starts with its PA15 guess: the line stays low
 *             until xr871d's pin probe finds PA14 and sends 0xc5 (the QEMU
 *             device takes the level as a u16 0xffff + u8 message)
 *   Extensions: battery 77 % / 3950 mV, USB event {0x0c,1} at 1 s, the
 *             simulated MCU answers 0x1e (USB state 1) / 0x1f / 0x23, stays
 *             silent on 0x26/0x29/0x2b/0x2c/0x2f like MCU sw 150 and echoes
 *             any other message 100 ms later (passthrough test), status LED
 *             on PA11
 *   Flash:    XR_FLASH=<2 MiB file> = the W25Q16 (NOR semantics: writes
 *             only clear bits, erase sets 0xff; every change goes straight
 *             to the file). XR_RAM_APP=<addr> = flash address of the APP
 *             section header whose code counts as "running in RAM"
 *             ("other" = code that matches no slot). XR_FLASH_SLOW=1 adds
 *             typical W25Q16 erase/program times.
 *
 * usage: xr871-host <qemu-uart0.sock> [seconds] [qemu-spi.sock] [qmp.sock]
 */
#include <fcntl.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include "../core/bridge_core.h"
#include "../core/spi_frame.h"

#define NETSIM_PORT 47871

static int sock = -1, spi = -1, udp = -1;
static struct sockaddr_in netsim;
static struct timespec t0;
static int t30_powered, dsp_on_msg;
static unsigned mcu_hb_sent, mcu_hb_answered;
static int armed;
static int qmp = -1, t30_was_off;
static const uint8_t sta_mac[6] = { 0x02, 0x87, 0x1e, 0x00, 0x00, 0x01 };
#define PHYS_READY 14
static int cur_ready = 15, probe_p = -1, probe_l, line = -1, wifi_cfg_ok;
static void ready_update(void);
static void qmp_cmd(const char *c);

/* ---- extension simulation ---- */
#define FLASH_SIZE 0x200000
static int flash_fd = -1;
static uint8_t *flash;
static uint8_t ram_app[0x1000];
static uint32_t ram_load, ram_len;
static struct { uint32_t at; uint8_t cmd, val; } mcu_q[16];
static int mcu_qn;
static int gpio_level[32], gpio_out[32];
static int reboots;

static uint32_t h_millis(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (t.tv_sec - t0.tv_sec) * 1000 + (t.tv_nsec - t0.tv_nsec) / 1000000;
}

static void h_log(const char *fmt, ...)
{
	char b[256];
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(b, sizeof(b), fmt, ap);
	va_end(ap);
	if (n >= (int)sizeof(b))
		n = sizeof(b) - 1;
	printf("[%7lu] %s", (unsigned long)h_millis(), b);
	fflush(stdout);
	bridge_log_capture(b, n);
}

static void mcu_queue(uint8_t cmd, uint8_t val, uint32_t delay)
{
	if (mcu_qn < 16) {
		mcu_q[mcu_qn].at = h_millis() + delay;
		mcu_q[mcu_qn].cmd = cmd;
		mcu_q[mcu_qn].val = val;
		mcu_qn++;
	}
}

static void mcu_queue_run(void)
{
	int i;

	for (i = 0; i < mcu_qn; i++) {
		if ((int32_t)(h_millis() - mcu_q[i].at) >= 0) {
			uint8_t c = mcu_q[i].cmd, v = mcu_q[i].val;

			memmove(mcu_q + i, mcu_q + i + 1, (mcu_qn - i - 1) * sizeof(mcu_q[0]));
			mcu_qn--;
			i--;
			h_log("sim-mcu -> {0x%02x,0x%02x}\n", c, v);
			bridge_mcu_rx(c, v);
		}
	}
}

static int h_battery(int *pct, int *mv)
{
	*pct = 77;
	*mv = 3950;
	return 0;
}

static int h_gpio_test(int port, int pin, int mode)
{
	if (port != 0 || pin < 0 || pin > 31)
		return -1;
	switch (mode) {
	case BR_GPIO_LOW:
	case BR_GPIO_HIGH:
		gpio_out[pin] = 1;
		if (gpio_level[pin] != mode && pin == 11)
			h_log("host: status LED (PA11) %s\n", mode ? "ON" : "off");
		gpio_level[pin] = mode;
		return 0;
	case BR_GPIO_READ:
		gpio_out[pin] = 0;
		return pin == 12;	/* simulated board: PA12 pulled high */
	case BR_GPIO_RELEASE:
		if (gpio_level[pin] && pin == 11)
			h_log("host: status LED (PA11) off\n");
		gpio_out[pin] = gpio_level[pin] = 0;
		h_log("host: PA%d released\n", pin);
		return 0;
	}
	return -1;
}

static int flash_sync(uint32_t a, uint32_t n)
{
	return pwrite(flash_fd, flash + a, n, a) == (ssize_t)n ? 0 : -1;
}

static int h_flash_read(uint32_t a, void *b, uint32_t n)
{
	if (!flash || a >= FLASH_SIZE || n > FLASH_SIZE - a)
		return -1;
	memcpy(b, flash + a, n);
	return 0;
}

/* XR_FLASH_SLOW: W25Q16 typical timings (sector erase 45 ms, page program
 * 0.7 ms per 256 bytes); the XR871 core blocks just like this */
static void flash_delay(unsigned us)
{
	if (getenv("XR_FLASH_SLOW"))
		usleep(us);
}

static int h_flash_erase4k(uint32_t a)
{
	if (!flash || (a & 0xfff) || a >= FLASH_SIZE)
		return -1;
	flash_delay(45000);
	memset(flash + a, 0xff, 0x1000);
	return flash_sync(a, 0x1000);
}

static int h_flash_write(uint32_t a, const void *b, uint32_t n)
{
	const uint8_t *p = b;
	uint32_t i;

	if (!flash || a >= FLASH_SIZE || n > FLASH_SIZE - a)
		return -1;
	flash_delay(700 * ((n + 255) / 256));
	for (i = 0; i < n; i++)
		flash[a + i] &= p[i];	/* NOR: programming only clears bits */
	return flash_sync(a, n);
}

static int h_ram_compare(uint32_t addr, const void *buf, uint32_t len)
{
	if (!ram_len || addr < ram_load || addr + len > ram_load + ram_len)
		return -1;
	return memcmp(ram_app + (addr - ram_load), buf, len) ? 1 : 0;
}

static void h_reboot(void)
{
	reboots++;
	h_log("host: XR871 reboot requested (#%d)\n", reboots);
}

static void flash_setup(void)
{
	const char *f = getenv("XR_FLASH"), *r = getenv("XR_RAM_APP");

	if (!f)
		return;
	flash_fd = open(f, O_RDWR);
	flash = malloc(FLASH_SIZE);
	if (flash_fd < 0 || !flash || pread(flash_fd, flash, FLASH_SIZE, 0) != FLASH_SIZE) {
		fprintf(stderr, "XR_FLASH %s: need a 2 MiB file\n", f);
		exit(2);
	}
	if (r && !strcmp(r, "other")) {
		/* code that matches neither slot (running-slot detection test) */
		ram_load = 0x10000;
		ram_len = sizeof(ram_app);
		memset(ram_app, 0x5a, ram_len);
	} else if (r) {
		uint32_t a = strtoul(r, NULL, 0), body = a + 64;

		/* the "RAM" copy: code of that APP section (body offset 0..4 KiB) */
		ram_load = flash[a + 16] | flash[a + 17] << 8 | flash[a + 18] << 16 |
			   (uint32_t)flash[a + 19] << 24;
		ram_len = sizeof(ram_app);
		memcpy(ram_app, flash + body, ram_len);
	}
}

static void h_t30_write(const uint8_t *buf, int len)
{
	if (send(sock, buf, len, 0) != len)
		h_log("host: t30 write failed: %s\n", strerror(errno));
}

static int h_mcu_send(uint8_t cmd, uint8_t val)
{
	if (cmd == 0x05 && val == 0x07)
		dsp_on_msg = 1;
	if (cmd == 0x0e)
		mcu_hb_answered++;
	else
		h_log("sim-mcu <- {0x%02x,0x%02x}\n", cmd, val);
	/* like MCU sw 150 on the camera: {0x1e} = USB state, no answer at
	 * all to 0x26/0x29/0x2b/0x2c/0x2f */
	if (cmd == 0x1e)
		mcu_queue(cmd, 0x01, 50);
	else if (cmd == 0x1f)
		mcu_queue(cmd, 0xa5, 50);
	else if (cmd == 0x23)
		mcu_queue(cmd, 150, 50);
	else if (cmd == 0x26 || cmd == 0x29 || cmd == 0x2b || cmd == 0x2c || cmd == 0x2f)
		;
	else if (cmd != 0x0e && cmd != 0x05)
		mcu_queue(cmd, val, 100);	/* echo: passthrough test */
	return 0;
}

static void h_t30_power(int on)
{
	t30_powered = on;
	h_log("host: PA16 (T30 power) = %d\n", on);
	if (!on) {
		qmp_cmd("stop");
		t30_was_off = 1;
	} else if (t30_was_off) {
		qmp_cmd("system_reset");
		qmp_cmd("cont");
	}
}

static int h_wifi_status(uint8_t *mac, int8_t *rssi)
{
	memcpy(mac, sta_mac, 6);
	*rssi = -48;
	return 1;
}

static int allowed(int pin)
{
	return pin == 6 || pin == 10 || pin == 11 || pin == 13 || pin == 14 || pin == 15;
}

static int h_probe_pin(int pin, int level)
{
	if (!allowed(pin))
		return -1;
	if (level == 2) {
		if (probe_p == pin)
			probe_p = -1;
	} else {
		probe_p = pin;
		probe_l = level;
	}
	ready_update();
	return 0;
}

static int h_set_ready_pin(int pin)
{
	if (!allowed(pin))
		return -1;
	cur_ready = pin;
	ready_update();
	return 0;
}

static int h_wifi_config(const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len)
{
	(void)ssid;
	(void)psk;
	h_log("host: wifi config ssid %d bytes, psk %d bytes\n", ssid_len, psk_len);
	wifi_cfg_ok = ssid_len > 0 && psk_len >= 8;
	return wifi_cfg_ok ? 1 : -1;
}

static void h_low_power(int enter)
{
	h_log("host: XR871 %s low power\n", enter ? "enters" : "leaves");
}

static void h_wifi_tx(const uint8_t *frame, int len)
{
	if (udp >= 0)
		sendto(udp, frame, len, 0, (struct sockaddr *)&netsim, sizeof(netsim));
}

static const struct bridge_hal hal = {
	.t30_write = h_t30_write, .mcu_send = h_mcu_send, .t30_power = h_t30_power,
	.millis = h_millis, .log = h_log, .wifi_tx = h_wifi_tx,
	.wifi_status = h_wifi_status, .low_power = h_low_power,
	.wifi_config = h_wifi_config, .probe_pin = h_probe_pin,
	.set_ready_pin = h_set_ready_pin,
	.version = "host", .battery = h_battery, .gpio_test = h_gpio_test,
	.flash_read = h_flash_read, .flash_erase4k = h_flash_erase4k,
	.flash_write = h_flash_write, .ram_compare = h_ram_compare,
	.reboot = h_reboot,
};

static int connect_unix(const char *path)
{
	struct sockaddr_un a;
	int i, s;

	memset(&a, 0, sizeof(a));
	a.sun_family = AF_UNIX;
	strncpy(a.sun_path, path, sizeof(a.sun_path) - 1);
	for (i = 0; i < 300; i++) {
		s = socket(AF_UNIX, SOCK_STREAM, 0);
		if (connect(s, (struct sockaddr *)&a, sizeof(a)) == 0)
			return s;
		close(s);
		usleep(100000);
	}
	return -1;
}

static void qmp_drain(void)
{
	char b[4096];
	fd_set rf;
	struct timeval tv = { 0, 200000 };

	FD_ZERO(&rf);
	FD_SET(qmp, &rf);
	while (select(qmp + 1, &rf, NULL, NULL, &tv) > 0 && recv(qmp, b, sizeof(b), 0) > 0) {
		tv.tv_sec = 0;
		tv.tv_usec = 20000;
		FD_ZERO(&rf);
		FD_SET(qmp, &rf);
	}
}

static void qmp_cmd(const char *c)
{
	char b[128];
	int n;

	if (qmp < 0)
		return;
	n = snprintf(b, sizeof(b), "{\"execute\":\"%s\"}\n", c);
	if (send(qmp, b, n, 0) != n)
		h_log("host: qmp %s failed\n", c);
	qmp_drain();
	h_log("host: qmp %s\n", c);
}

/* level of the physical ready line (PA14 on the simulated board) */
static void ready_update(void)
{
	int l;
	uint8_t m[3] = { 0xff, 0xff, 0 };

	if (probe_p == PHYS_READY)
		l = probe_l;
	else if (probe_p >= 0 && probe_p == cur_ready)
		l = 0;			/* firmware ready logic suspended */
	else
		l = cur_ready == PHYS_READY;
	if (l == line || spi < 0)
		return;
	line = l;
	m[2] = l;
	if (send(spi, m, 3, 0) == 3)
		h_log("host: ready line (PA%d) %s\n", PHYS_READY, l ? "high" : "low");
}

/* arm the slave's next transfer (what the T30 will clock out of us) */
static void spi_arm(void)
{
	static uint8_t f[2 + SPIF_SIZE];

	bridge_spi_next(f + 2);
	f[0] = SPIF_SIZE & 0xff;
	f[1] = SPIF_SIZE >> 8;
	if (send(spi, f, sizeof(f), 0) == (ssize_t)sizeof(f))
		armed++;
}

/* frames from the spi-frame-peer: u16 length + data */
static void spi_input(void)
{
	static uint8_t in[2 + 8192];
	static int have;
	ssize_t n = recv(spi, in + have, sizeof(in) - have, 0);

	if (n <= 0) {
		h_log("host: QEMU closed the SPI link\n");
		close(spi);
		spi = -1;
		return;
	}
	have += n;
	while (have >= 2) {
		int len = in[0] | in[1] << 8;

		if (len > (int)sizeof(in) - 2) {
			have = 0;
			return;
		}
		if (have < 2 + len)
			return;
		bridge_spi_rx(in + 2, len);
		if (armed > 0)
			armed--;
		if (!armed)
			spi_arm();
		memmove(in, in + 2 + len, have - 2 - len);
		have -= 2 + len;
	}
}

int main(int argc, char **argv)
{
	int secs = argc > 2 ? atoi(argv[2]) : 60;
	uint32_t next_hb = 2000, pir_at = 15000;
	uint32_t secret_at = getenv("XR_TEST_SECRET_LOG") ? 500 : 0;
	int pir_done = 0;

	if (argc < 2) {
		fprintf(stderr, "usage: %s <qemu-uart0.sock> [seconds] [qemu-spi.sock]\n", argv[0]);
		return 2;
	}
	sock = connect_unix(argv[1]);
	if (sock < 0) {
		perror("connect uart");
		return 2;
	}
	if (argc > 3) {
		spi = connect_unix(argv[3]);
		if (spi < 0) {
			perror("connect spi");
			return 2;
		}
		udp = socket(AF_INET, SOCK_DGRAM, 0);
		memset(&netsim, 0, sizeof(netsim));
		netsim.sin_family = AF_INET;
		netsim.sin_port = htons(NETSIM_PORT);
		netsim.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	}
	if (argc > 4) {
		qmp = connect_unix(argv[4]);
		if (qmp < 0) {
			perror("connect qmp");
			return 2;
		}
		qmp_drain();
		qmp_cmd("qmp_capabilities");
	}
	clock_gettime(CLOCK_MONOTONIC, &t0);
	flash_setup();
	bridge_init(&hal, 4);
	mcu_queue(0x0c, 0x01, 1000);	/* USB power present */
	if (spi >= 0) {
		spi_arm();
		ready_update();
	}

	while (h_millis() < (uint32_t)secs * 1000) {
		fd_set rf;
		struct timeval tv = { 0, 20000 };
		uint8_t buf[2048];
		int maxfd = sock;

		FD_ZERO(&rf);
		FD_SET(sock, &rf);
		if (spi >= 0) {
			FD_SET(spi, &rf);
			if (spi > maxfd)
				maxfd = spi;
		}
		if (udp >= 0) {
			FD_SET(udp, &rf);
			if (udp > maxfd)
				maxfd = udp;
		}
		if (select(maxfd + 1, &rf, NULL, NULL, &tv) > 0) {
			if (FD_ISSET(sock, &rf)) {
				ssize_t n = recv(sock, buf, sizeof(buf), 0);
				if (n <= 0) {
					h_log("host: QEMU closed the UART\n");
					break;
				}
				bridge_t30_rx(buf, n);
			}
			if (spi >= 0 && FD_ISSET(spi, &rf))
				spi_input();
			if (udp >= 0 && FD_ISSET(udp, &rf)) {
				ssize_t n = recv(udp, buf, sizeof(buf), 0);
				if (n > 0)
					bridge_wifi_rx(buf, n);
			}
		}
		if (h_millis() >= next_hb) {
			mcu_hb_sent++;
			bridge_mcu_rx(0x0e, 0x55);
			next_hb += 2000;
		}
		mcu_queue_run();
		if (secret_at && h_millis() >= secret_at) {
			/* log-filter test: must never reach the T30 */
			h_log("test: wifi psk=S3cr3tValue\n");
			secret_at = 0;
		}
		if (!pir_done && getenv("XR_NO_PIR"))
			pir_done = 1;
		if (!pir_done && h_millis() >= pir_at) {
			bridge_mcu_rx(0x0d, 0x01);
			pir_done = 1;
		}
		bridge_poll();
	}

	{
		const struct bridge_stats *s = bridge_stats();
		int ok = s->t30_hb > 0 && s->t30_marker_seen && dsp_on_msg && t30_powered &&
			 mcu_hb_answered >= mcu_hb_sent;
		printf("---- verdict ----\n");
		printf("t30: hello %lu, heartbeats %lu, first after %lu ms, marker %d, bad frames %lu\n",
		       (unsigned long)s->t30_hello, (unsigned long)s->t30_hb,
		       (unsigned long)(s->t30_first_hb_ms - s->t30_power_on_ms),
		       s->t30_marker_seen, (unsigned long)s->t30_bad);
		printf("mcu: heartbeats sent %u answered %u, dsp-on msg %d, PA16 %d\n",
		       mcu_hb_sent, mcu_hb_answered, dsp_on_msg, t30_powered);
		if (argc > 3)
			printf("spi: frames %lu bad %lu, eth T30->wifi %lu, wifi->T30 %lu, dropped %lu\n",
			       (unsigned long)s->spi_frames, (unsigned long)s->spi_bad,
			       (unsigned long)s->spi_eth_up, (unsigned long)s->spi_eth_down,
			       (unsigned long)s->wifi_drop);
		printf("power: sleeps %lu wakes %lu last wake 0x%03lx\n",
			       (unsigned long)s->sleeps, (unsigned long)s->wakes,
			       (unsigned long)s->last_wake);
		if (argc > 3)
			printf("ready: line PA%d, firmware uses PA%d, wifi config %s\n",
			       PHYS_READY, cur_ready, wifi_cfg_ok ? "received" : "missing");
		printf("%s\n", ok ? "PASS" : "FAIL");
		return ok ? 0 : 1;
	}
}
