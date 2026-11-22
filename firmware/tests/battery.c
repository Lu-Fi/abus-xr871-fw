/*
 * Unit test for the battery policy in the core (bridge_battery):
 *   - charge restart {0x21,0},{0x21,1} on USB when SOC drops from 100 %;
 *   - low-battery shutdown {0x24,0} without USB after BR_LOWBAT_READS low
 *     readings, once, re-armed by USB power;
 *   - no shutdown with USB unknown, on USB, or with a healthy VBAT;
 *   - the status record (0xd0) carries T2E/T2F, restarts and the low count;
 *   - LED request 0xd5: stored, answered, cleared by a T30 hello.
 *
 * Build: cc -Wall -Wextra -I.. -o battery battery.c ../core/bridge_core.c ../core/fw_update.c
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "core/bridge_core.h"

static uint32_t ms = 1000;
static uint8_t sent[64][2];
static int nsent;
static uint8_t st[BR_STATUS_LEN];
static int st_len;

static int led_reply = -1;

static void w(const uint8_t *b, int l)
{
	int plen = b[4] | b[5] << 8;

	if (l >= BR_HDR_LEN && b[6] == T30_LED)
		led_reply = b[10] | b[11] << 8;

	if (l >= BR_HDR_LEN && b[6] == T30_STATUS) {
		st_len = plen;
		memcpy(st, b + BR_HDR_LEN, plen > (int)sizeof(st) ? (int)sizeof(st) : plen);
	}
}

static int mcu(uint8_t c, uint8_t v)
{
	if (nsent < 64) {
		sent[nsent][0] = c;
		sent[nsent][1] = v;
	}
	nsent++;
	return 0;
}

static void pw(int on) { (void)on; }
static uint32_t mil(void) { return ms; }
static void lg(const char *f, ...) { (void)f; }

static int count(uint8_t c, uint8_t v)
{
	int i, n = 0;

	for (i = 0; i < nsent && i < 64; i++)
		if (sent[i][0] == c && sent[i][1] == v)
			n++;
	return n;
}

static int fails;

static void expect(const char *what, int ok)
{
	if (!ok)
		fails++;
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}

static void status(void)
{
	uint8_t f[12] = { 0xaa, 0x55, 0xaa, 0x55, 0, 0, T30_STATUS, 0x01, 0, 0, 0, 0 };

	st_len = 0;
	bridge_t30_rx(f, sizeof(f));
}

static void t30_cmd(uint8_t cmd, uint16_t param)
{
	uint8_t f[12] = { 0xaa, 0x55, 0xaa, 0x55, 0, 0, cmd, 0x01, 0, 0,
			  param & 0xff, param >> 8 };

	bridge_t30_rx(f, sizeof(f));
}

static void reads(int n, int soc, int mv)
{
	while (n--) {
		ms += 30000;
		bridge_battery(soc, mv, 600, 0xffff);
	}
}

int main(void)
{
	static struct bridge_hal h;
	int i;

	h.t30_write = w; h.mcu_send = mcu; h.t30_power = pw; h.millis = mil;
	h.log = lg;
	bridge_init(&h, 0);

	printf("battery policy:\n");
	/* USB state unknown (no 0x1e answer yet): never a shutdown */
	nsent = 0;
	reads(5, 0, 3300);
	expect("USB unknown: no shutdown", count(MCU_LOW_POWER, 0) == 0);

	/* USB in, full, then below 100 */
	bridge_mcu_rx(MCU_EV_USB, 1);
	nsent = 0;
	reads(1, 100, 4180);
	reads(1, 100, 4180);
	expect("USB, 100 %: no charge restart", count(MCU_CHARGE, 0) == 0);
	reads(1, 99, 4150);
	expect("USB, 100 -> 99 %: {0x21,0} then {0x21,1}",
	       count(MCU_CHARGE, 0) == 1 && count(MCU_CHARGE, 1) == 1 &&
	       sent[0][0] == MCU_CHARGE && sent[0][1] == 0 && sent[1][1] == 1);
	reads(3, 98, 4140);
	expect("USB, staying below 100 %: no repeat", count(MCU_CHARGE, 0) == 1);
	reads(1, 100, 4180);
	reads(1, 99, 4150);
	expect("USB, full again then 99 %: second restart", count(MCU_CHARGE, 0) == 2);
	reads(5, 1, 3300);
	expect("USB, SOC 1 %: no shutdown on USB", count(MCU_LOW_POWER, 0) == 0);

	/* battery only */
	bridge_mcu_rx(MCU_EV_USB, 0);
	nsent = 0;
	reads(1, 100, 4100);
	reads(1, 99, 4090);
	expect("no USB, 100 -> 99 %: no charge restart", count(MCU_CHARGE, 0) == 0);
	reads(5, 0, 3900);
	expect("no USB, 0 % with VBAT 3900 mV (gauge glitch): no shutdown",
	       count(MCU_LOW_POWER, 0) == 0);
	reads(BR_LOWBAT_READS - 1, 1, 3400);
	expect("no USB, 1 %: no shutdown before 3 readings", count(MCU_LOW_POWER, 0) == 0);
	reads(1, 5, 3550);
	reads(BR_LOWBAT_READS - 1, 1, 3400);
	expect("a good reading in between restarts the count", count(MCU_LOW_POWER, 0) == 0);
	reads(1, 1, 3400);
	expect("no USB, 3 readings < 2 %: {0x24,0} sent", count(MCU_LOW_POWER, 0) == 1);
	reads(10, 0, 3300);
	expect("shutdown sent only once", count(MCU_LOW_POWER, 0) == 1);
	reads(1, 3, 3500);
	reads(3, 1, 3400);
	expect("still only once after a recovery reading (no USB)", count(MCU_LOW_POWER, 0) == 1);
	status();
	expect("status: 160 bytes", st_len == BR_STATUS_LEN && BR_STATUS_LEN == 160);
	expect("status: low battery 0xff (shutdown sent)", st[158] == 0xff);
	expect("status: T2E 600 min, T2F unknown",
	       (st[152] | st[153] << 8) == 600 && (st[154] | st[155] << 8) == 0xffff);
	expect("status: 2 charge restarts", (st[156] | st[157] << 8) == 2);
	expect("status: version string still at 88",
	       !memcmp(st + 88, "abus-xr871 core", 15) && st[151] == 0);

	/* USB back: re-armed */
	bridge_mcu_rx(MCU_EV_USB, 1);
	reads(1, 1, 3600);
	bridge_mcu_rx(MCU_EV_USB, 0);
	reads(BR_LOWBAT_READS, 1, 3400);
	expect("USB in and out again: shutdown re-armed", count(MCU_LOW_POWER, 0) == 2);

	/* failed reads are ignored */
	i = nsent;
	bridge_battery(-1, -1, -1, -1);
	bridge_battery(101, 4000, -1, -1);
	expect("invalid readings ignored", nsent == i);

	/* LED request from the T30 */
	led_reply = -1;
	t30_cmd(T30_LED, 0x0101);
	expect("LED 0xd5: blue stored, other bits dropped, answered",
	       bridge_stats()->led_req == BR_LED_BLUE && led_reply == BR_LED_BLUE);
	t30_cmd(T30_LED, 0);
	expect("LED 0xd5: blue off", bridge_stats()->led_req == 0 && led_reply == 0);
	t30_cmd(T30_LED, 1);
	t30_cmd(T30_HELLO, 0);
	expect("LED request cleared by a T30 hello", bridge_stats()->led_req == 0);

	printf(fails ? "BATTERY: %d FAIL\n" : "BATTERY PASS\n", fails);
	return fails != 0;
}
