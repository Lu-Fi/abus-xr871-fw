/*
 * Unit test for the T30 frame receiver (bridge_t30_rx) with lost bytes.
 *
 * 2026-11-21, camera 2: a WiFi config frame (0xc0) that lost bytes on the
 * UART was filled up with the following frame and applied, the XR871
 * rejoined with a wrong key until reboot, and the next frame (ready pin
 * 0xc5) was eaten. Expected now:
 *   - CRC frames (flag 0x02): a damaged frame is never handled, and the
 *     frame after it is still handled (resync from the next magic);
 *   - vendor frames (flag 0x01): resync on the next magic as before.
 *
 * Build: cc -Wall -Wextra -I.. -o t30_rx_loss t30_rx_loss.c ../core/bridge_core.c ../core/fw_update.c
 * Dummy credentials only.
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "core/bridge_core.h"

static uint32_t ms = 1000;
static int ncfg, ncfg_bad, nready;
static uint8_t ref[64];

static void w(const uint8_t *b, int l) { (void)b; (void)l; }
static int mcu(uint8_t c, uint8_t v) { (void)c; (void)v; return 0; }
static void pw(int on) { (void)on; }
static uint32_t mil(void) { return ms; }
static void lg(const char *f, ...) { (void)f; }

static int cfg(const uint8_t *s, int sl, const uint8_t *p, int pl)
{
	ncfg++;
	if (pl != 64 || memcmp(p, ref, 64) || sl != 8 || memcmp(s, "TESTNETW", 8))
		ncfg_bad++;
	return 0;
}

static int rdy(int pin) { (void)pin; nready++; return 0; }

static uint16_t crc16(const uint8_t *f, int plen)
{
	uint16_t crc = 0xffff;
	int i, j;

	for (i = 4; i < 12 + plen; i++) {
		if (i == 8 || i == 9)
			continue;
		crc ^= (uint16_t)f[i] << 8;
		for (j = 0; j < 8; j++)
			crc = crc & 0x8000 ? (uint16_t)(crc << 1) ^ 0x1021 : (uint16_t)(crc << 1);
	}
	return crc;
}

static int mk(uint8_t *o, uint8_t cmd, uint16_t param, const uint8_t *p, int pl, int withcrc)
{
	memset(o, 0, 12);
	o[0] = 0xaa; o[1] = 0x55; o[2] = 0xaa; o[3] = 0x55;
	o[4] = pl; o[5] = pl >> 8; o[6] = cmd; o[7] = withcrc ? 0x02 : 0x01;
	o[10] = param; o[11] = param >> 8;
	if (pl)
		memcpy(o + 12, p, pl);
	if (withcrc) {
		uint16_t c = crc16(o, pl);

		o[8] = c; o[9] = c >> 8;
	}
	return 12 + pl;
}

static int fails;

static void expect(const char *what, int ok)
{
	if (!ok)
		fails++;
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}

int main(void)
{
	static struct bridge_hal h;
	uint8_t pay[98] = { 0 };
	int withcrc, drop, i;

	h.t30_write = w; h.mcu_send = mcu; h.t30_power = pw; h.millis = mil;
	h.log = lg; h.wifi_config = cfg; h.set_ready_pin = rdy;
	bridge_init(&h, 0);
	for (i = 0; i < 64; i++)
		ref[i] = "0123456789abcdef"[(i * 7 + 3) & 15];
	pay[0] = 8; memcpy(pay + 1, "TESTNETW", 8); pay[33] = 64; memcpy(pay + 34, ref, 64);

	for (withcrc = 1; withcrc >= 0; withcrc--) {
		int bad_applied = 0, ready_lost = 0, good_ok = 1;

		for (drop = 0; drop <= 14; drop++) {
			uint8_t s[400], f[200];
			int n = 0, l;

			l = mk(f, 0xc0, 0, pay, 98, withcrc);	/* lose bytes from frame offset 70 */
			memcpy(s + n, f, 70); n += 70;
			memcpy(s + n, f + 70 + drop, l - 70 - drop); n += l - 70 - drop;
			l = mk(f, 0xc5, 10, NULL, 0, withcrc);
			memcpy(s + n, f, l); n += l;
			ncfg = ncfg_bad = nready = 0;
			ms += 5000;
			bridge_t30_rx(s, n);
			ms += 300;		/* idle poll after the burst */
			bridge_t30_rx(NULL, 0);
			if (drop == 0 && (ncfg != 1 || ncfg_bad || nready != 1))
				good_ok = 0;
			if (ncfg_bad)
				bad_applied++;
			if (nready != 1)
				ready_lost++;
		}
		printf("%s frames:\n", withcrc ? "CRC" : "vendor");
		expect("intact WiFi config and ready frame handled", good_ok);
		if (withcrc)
			expect("damaged WiFi config never applied with wrong data", bad_applied == 0);
		else	/* vendor format, no CRC: 1-3 lost bytes are filled from the next
			 * frame; the XR871 glue's credential check is the safety net */
			printf("  [INFO] vendor frames: %d damaged configs reached the hook\n", bad_applied);
		expect("ready frame after a damaged frame still handled", ready_lost == 0);
	}
	printf(fails ? "T30 RX LOSS: %d FAIL\n" : "T30 RX LOSS PASS\n", fails);
	return fails != 0;
}
