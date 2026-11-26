/*
 * Unit test for the T30 wake sequence and the boot check (2026-11-26,
 * camera 3: a woken T30 stayed dead until the 180 s stall reset):
 *   - wake: {0x05,0x07} goes to the MCU first, the power GPIO stays low for
 *     BR_WAKE_RAIL_MS, then goes high;
 *   - a T30 that says hello after power-on is left alone;
 *   - no hello within BR_BOOT_MS: power-cycle, even inside the stall
 *     holdoff, with the off times of BR_BOOT_OFF_MS, at most
 *     BR_BOOT_RETRIES times in a row;
 *   - a hello resets the retry count;
 *   - a key press (MCU event 0x06) wakes a sleeping T30 with the key reason;
 *   - a sleep request during the rail wait leaves the T30 off.
 *
 * Build: cc -Wall -Wextra -I.. -o wake wake.c ../core/bridge_core.c ../core/fw_update.c
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "core/bridge_core.h"

static uint32_t ms = 1000;
static int pwr = -1, n_pwr_on, n_pwr_off;
static int mcu_last_cmd = -1, mcu_last_val = -1;
static int pwr_at_rail = -1;	/* GPIO state when {0x05,0x07} went out */

static void w(const uint8_t *b, int l) { (void)b; (void)l; }
static int mcu(uint8_t c, uint8_t v)
{
	mcu_last_cmd = c;
	mcu_last_val = v;
	if (c == MCU_DSP_POWER && v == 0x07)
		pwr_at_rail = pwr;
	return 0;
}
static void pw(int on)
{
	if (on && pwr != 1)
		n_pwr_on++;
	if (!on && pwr != 0)
		n_pwr_off++;
	pwr = on;
}
static uint32_t mil(void) { return ms; }
static void lg(const char *f, ...) { (void)f; }

static int fails;

static void expect(const char *what, int ok)
{
	if (!ok)
		fails++;
	printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
}

static void t30_cmd(uint8_t cmd, uint16_t param)
{
	uint8_t f[12] = { 0xaa, 0x55, 0xaa, 0x55, 0, 0, cmd, 0x01, 0, 0,
			  param & 0xff, param >> 8 };

	bridge_t30_rx(f, sizeof(f));
}

static void advance(uint32_t step_ms, uint32_t total_ms, int hb)
{
	uint32_t end = ms + total_ms;

	while ((int32_t)(ms - end) < 0) {
		ms += step_ms;
		if (hb)
			t30_cmd(T30_HEARTBEAT, 0);
		bridge_poll();
	}
}

static void sleep_t30(void)
{
	t30_cmd(T30_POWER, 0);
	advance(100, BR_SLEEP_GRACE_MS + 200, 0);
}

int main(void)
{
	static struct bridge_hal h;
	const struct bridge_stats *S;
	uint32_t r0;
	int on0;

	h.t30_write = w; h.mcu_send = mcu; h.t30_power = pw; h.millis = mil;
	h.log = lg;
	bridge_init(&h, 0);
	S = bridge_stats();

	printf("T30 wake sequence and boot check:\n");
	t30_cmd(T30_HELLO, 0);
	advance(1000, 30 * 1000, 1);
	expect("booted T30 with hello: no reset", S->t30_resets == 0 && pwr == 1);

	sleep_t30();
	expect("asleep: power GPIO low", pwr == 0);
	advance(1000, 5 * 60 * 1000, 0);

	on0 = n_pwr_on;
	bridge_wake(BR_WAKE_PIR);
	expect("wake: MCU DSP rail {0x05,0x07} sent with the GPIO still low",
	       pwr_at_rail == 0 && mcu_last_cmd == MCU_DSP_POWER && mcu_last_val == 0x07);
	expect("wake: GPIO not high at once", pwr == 0);
	advance(50, BR_WAKE_RAIL_MS - 100, 0);
	expect("wake: still low just before BR_WAKE_RAIL_MS", pwr == 0);
	advance(50, 200, 0);
	expect("wake: GPIO high after BR_WAKE_RAIL_MS", pwr == 1 && n_pwr_on == on0 + 1);

	/* the T30 says nothing: boot check, longer off time per retry */
	{
		static const uint32_t off[BR_BOOT_RETRIES] = BR_BOOT_OFF_MS;
		char what[96];
		int i;

		r0 = S->t30_resets;
		advance(100, BR_BOOT_MS - 1000, 0);
		expect("no reset before BR_BOOT_MS", S->t30_resets == r0);
		for (i = 0; i < BR_BOOT_RETRIES; i++) {
			uint32_t lim = ms + BR_BOOT_MS + 2000;

			while (S->t30_resets == r0 + i && (int32_t)(ms - lim) < 0)
				advance(100, 100, 0);	/* stop right at the cycle */
			snprintf(what, sizeof(what), "retry %d: power-cycle, T30 off", i + 1);
			expect(what, S->t30_resets == r0 + i + 1 && pwr == 0);
			advance(100, off[i] - 1000, 0);
			snprintf(what, sizeof(what), "retry %d: still off before %lu ms",
				 i + 1, (unsigned long)off[i]);
			expect(what, pwr == 0);
			advance(100, 1000, 0);	/* power-on at off[i] after the cycle */
			snprintf(what, sizeof(what), "retry %d: on again after %lu ms",
				 i + 1, (unsigned long)off[i]);
			expect(what, pwr == 1);
		}
		advance(100, BR_BOOT_MS + 1000, 0);
		expect("gives up after BR_BOOT_RETRIES", S->t30_resets == r0 + BR_BOOT_RETRIES);
		advance(1000, 60 * 1000, 0);
		expect("no further quick retry", S->t30_resets == r0 + BR_BOOT_RETRIES && pwr == 1);
	}

	/* a hello resets the count: next wake gets its retries again */
	t30_cmd(T30_HELLO, 0);
	advance(1000, 10 * 1000, 1);
	sleep_t30();
	advance(1000, 60 * 1000, 0);
	bridge_wake(BR_WAKE_KEY);
	advance(100, BR_WAKE_RAIL_MS + 200, 0);
	r0 = S->t30_resets;
	advance(500, BR_BOOT_MS + 1000, 0);
	expect("after a hello the boot check is armed again", S->t30_resets == r0 + 1);
	advance(500, BR_RESET_OFF_MS + 1000, 0);
	t30_cmd(T30_HELLO, 0);
	advance(1000, BR_BOOT_MS + 5000, 1);
	expect("hello after the retry: no further reset", S->t30_resets == r0 + 1 && pwr == 1);

	/* key press 0x06 from standby wakes with the key reason */
	sleep_t30();
	advance(1000, 10 * 1000, 0);
	expect("asleep before the key press", pwr == 0);
	bridge_mcu_rx(MCU_EV_KEYPRESS, 0x96);
	advance(100, BR_WAKE_RAIL_MS + 200, 0);
	expect("key press 0x06 wakes the T30, reason key",
	       pwr == 1 && S->last_wake == BR_WAKE_KEY);
	t30_cmd(T30_HELLO, 0);
	advance(1000, 10 * 1000, 1);

		/* sleep during the rail wait cancels the power-on */
	sleep_t30();
	advance(1000, 30 * 1000, 0);
	bridge_wake(BR_WAKE_TIMER);
	t30_cmd(T30_POWER, 0);	/* a T30 cannot ask this early; model a fast re-sleep */
	advance(100, BR_SLEEP_GRACE_MS + BR_WAKE_RAIL_MS + 500, 0);
	expect("re-sleep during the rail wait: T30 ends up off", pwr == 0);

	printf("%s\n", fails ? "FAILURES" : "all passed");
	return fails ? 1 : 0;
}
