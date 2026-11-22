/*
 * XR871 glue for the ABUS PPIC90520 bridge firmware.
 *   milestone 1 (default): keep-alive, T30 power, MCU link
 *   milestone 2 (-DBR_WIFI, project abus_xr871_net): + WiFi station and
 *   the SPI1 slave data link to the T30 (spi_slave.c, wifi.c)
 *
 * Pin map (from the stock firmware, see NOTES.md):
 *   UART0 PB0/PB1  debug console (front pads)        - SDK console
 *   UART1 PA17/18  T30 link, 115200 8N1
 *   I2C1  PA8/PA9  ST power MCU at 0x0A, 100 kHz
 *   PA7            MCU wake (out pulse) / IRQ (falling edge, EINT)
 *   PA16           T30 power enable (1 = on)
 *   PA14/PA15      held low as in the stock SPI init (role TBD)
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_i2c.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_global.h"
#include "core/bridge_core.h"	/* project dir is on the include path */
#include "hal_ext.h"		/* service extensions (status, GPIO test, update) */
#ifdef BR_WIFI
#include "common/framework/platform_init.h"
#include "spi_slave.h"
#include "wifi.h"
#endif

#define T30_UART	UART1_ID
#define MCU_I2C		I2C1_ID
#define MCU_ADDR	0x0A
#define MCU_IRQ_PORT	GPIO_PORT_A
#define MCU_IRQ_PIN	GPIO_PIN_7
#define T30_PWR_PIN	GPIO_PIN_16

static OS_Mutex_t i2c_lock;
static OS_Mutex_t core_lock;	/* the core is not thread safe */
static OS_Mutex_t log_lock;	/* h_log's line buffer */
static OS_Semaphore_t mcu_irq_sem;
static OS_Thread_t t30_thread, mcu_thread;

static void out_pin(GPIO_Pin pin, GPIO_PinState v)
{
	GPIO_InitParam p = { GPIOx_Pn_F1_OUTPUT, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE };

	HAL_GPIO_Init(GPIO_PORT_A, pin, &p);
	HAL_GPIO_WritePin(GPIO_PORT_A, pin, v);
}

static void mcu_irq_cb(void *arg)
{
	(void)arg;
	OS_SemaphoreRelease(&mcu_irq_sem);
}

static void mcu_irq_enable(void)
{
	GPIO_InitParam p = { GPIOA_P7_F6_EINTA7, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE };
	GPIO_IrqParam irq = { GPIO_IRQ_EVT_FALLING_EDGE, mcu_irq_cb, NULL };

	HAL_GPIO_Init(MCU_IRQ_PORT, MCU_IRQ_PIN, &p);
	HAL_GPIO_EnableIRQ(MCU_IRQ_PORT, MCU_IRQ_PIN, &irq);
}

/* CMU_WakeUpMcu: 2 ms high pulse on PA7, 20 ms settle, back to IRQ input */
static void mcu_wake(void)
{
	HAL_GPIO_DisableIRQ(MCU_IRQ_PORT, MCU_IRQ_PIN);
	HAL_GPIO_DeInit(MCU_IRQ_PORT, MCU_IRQ_PIN);
	out_pin(MCU_IRQ_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(MCU_IRQ_PORT, MCU_IRQ_PIN, GPIO_PIN_HIGH);
	OS_MSleep(2);
	HAL_GPIO_WritePin(MCU_IRQ_PORT, MCU_IRQ_PIN, GPIO_PIN_LOW);
	OS_MSleep(20);
	HAL_GPIO_DeInit(MCU_IRQ_PORT, MCU_IRQ_PIN);
	mcu_irq_enable();
}

/* ---- HAL for the portable core ---------------------------------------- */

static void h_t30_write(const uint8_t *buf, int len)
{
	HAL_UART_Transmit_IT(T30_UART, buf, len);
}

static int h_mcu_send(uint8_t cmd, uint8_t val)
{
	int32_t r;

	OS_MutexLock(&i2c_lock, OS_WAIT_FOREVER);
	mcu_wake();
	r = HAL_I2C_Master_Transmit_Mem_IT(MCU_I2C, MCU_ADDR, cmd,
					   I2C_MEMADDR_SIZE_8BIT, &val, 1);
	OS_MutexUnlock(&i2c_lock);
	return r == 1 ? 0 : -1;
}

static void h_t30_power(int on)
{
	HAL_GPIO_WritePin(GPIO_PORT_A, T30_PWR_PIN, on ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

static uint32_t h_millis(void)
{
	return OS_TicksToMSecs(OS_GetTicks());
}

/* console + queue for the T30 (log forwarding, bridge_log_capture). The
 * line buffer is static: the core also logs from the SPI thread (2 KiB
 * stack) and from the SDK's WLAN receive path, whose stacks we do not own. */
static void h_log(const char *fmt, ...)
{
	static char b[200];
	va_list ap;
	int n;

	OS_MutexLock(&log_lock, OS_WAIT_FOREVER);
	va_start(ap, fmt);
	n = vsnprintf(b, sizeof(b), fmt, ap);
	va_end(ap);
	if (n >= (int)sizeof(b))
		n = sizeof(b) - 1;
	if (n > 0) {
		printf("[%7lu] %s", (unsigned long)h_millis(), b);
		bridge_log_capture(b, n);
	}
	OS_MutexUnlock(&log_lock);
}

#ifdef BR_WIFI
/* The MCU/T30 side now starts before platform_init (see main), so the T30
 * could hand over WiFi credentials before the WLAN stack exists. Keep the
 * last ones and apply them once the stack is up. */
static volatile int net_ready;
static uint8_t pend_ssid[32], pend_psk[64];
static int pend_ssid_len = -1, pend_psk_len;

static int wifi_config_deferred(const uint8_t *ssid, int ssid_len,
				const uint8_t *psk, int psk_len)
{
	if (net_ready)
		return wifi_config(ssid, ssid_len, psk, psk_len);
	if (ssid_len < 0 || ssid_len > (int)sizeof(pend_ssid) ||
	    psk_len < 0 || psk_len > (int)sizeof(pend_psk))
		return -1;
	memcpy(pend_ssid, ssid, ssid_len);
	memcpy(pend_psk, psk, psk_len);
	pend_ssid_len = ssid_len;
	pend_psk_len = psk_len;
	return 0;
}
#endif

/* not const: hal_ext_fill() adds the optional hooks */
#ifdef BR_WIFI
static struct bridge_hal hal = {
	h_t30_write, h_mcu_send, h_t30_power, h_millis, h_log, wifi_tx,
	wifi_status, wifi_low_power, wifi_config_deferred,
	spis_probe_pin, spis_set_ready_pin, wifi_mode
};
#else
static struct bridge_hal hal = {
	h_t30_write, h_mcu_send, h_t30_power, h_millis, h_log, NULL
};
#endif

/* ---- threads ---------------------------------------------------------- */

/*
 * T30 UART receive: interrupt -> ring -> task. HAL_UART_Receive_IT windows
 * left only the 64-byte hardware FIFO between two calls; while the task
 * waited for the core lock (frame handling, flash writes) a burst from the
 * T30 overran it. A frame with lost bytes was then filled up with the
 * next frame: a corrupted WiFi config made the XR871 rejoin with a wrong
 * key until reboot (2026-11-21). The ring is drained under the core lock.
 */
static uint8_t t30_ring[4096];
static volatile uint32_t t30_w, t30_r, t30_ovr;
static OS_Semaphore_t t30_sem;

static void t30_rx_cb(void *arg)
{
	UART_T *u = HAL_UART_GetInstance(T30_UART);

	(void)arg;
	while (HAL_UART_IsRxReady(u)) {
		uint8_t c = HAL_UART_GetRxData(u);
		uint32_t n = (t30_w + 1) % sizeof(t30_ring);

		if (n != t30_r) {
			t30_ring[t30_w] = c;
			t30_w = n;
		} else {
			t30_ovr++;	/* ring full: counted, shown in the status line */
		}
	}
	OS_SemaphoreRelease(&t30_sem);
}

uint32_t t30_rx_overruns(void)
{
	return t30_ovr;
}

static void t30_task(void *arg)
{
	uint8_t buf[256];
	int n;

	(void)arg;
	for (;;) {
		(void)OS_SemaphoreWait(&t30_sem, 50);
		OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
		do {
			for (n = 0; n < (int)sizeof(buf) && t30_r != t30_w; n++) {
				buf[n] = t30_ring[t30_r];
				t30_r = (t30_r + 1) % sizeof(t30_ring);
			}
			bridge_t30_rx(buf, n);	/* n == 0: lets the core see gaps */
		} while (n == (int)sizeof(buf));
		bridge_poll();
		OS_MutexUnlock(&core_lock);
	}
}

static void mcu_task(void *arg)
{
	uint8_t m[2];
	int32_t r;

	(void)arg;
	for (;;) {
		if (OS_SemaphoreWait(&mcu_irq_sem, 1000) != OS_OK)
			continue;
		OS_MutexLock(&i2c_lock, OS_WAIT_FOREVER);
		r = HAL_I2C_Master_Receive_Mem_IT(MCU_I2C, MCU_ADDR, 0,
						  I2C_MEMADDR_SIZE_8BIT, m, 2);
		OS_MutexUnlock(&i2c_lock);
		if (r == 2) {
			OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
			bridge_mcu_rx(m[0], m[1]);
			OS_MutexUnlock(&core_lock);
		} else
			h_log("mcu: read failed (%ld)\n", (long)r);
	}
}

#ifdef BR_WIFI
/* ---- boot diagnostics -------------------------------------------------
 * Strong copies of the SDK's weak platform_init_level0/1 (project/common/
 * framework/platform_init.c, SDK 1.4.0) with a flushed marker after every
 * step: stdout is a 64-byte polled FIFO, so without the flush the last
 * lines before a reset never leave the chip. "boot: <step>" = step begins.
 */
#include "common/board/board.h"
#include "common/framework/sysinfo.h"
#include "common/framework/net_ctrl.h"
#include "common/framework/sys_ctrl/sys_ctrl.h"
#include "image/image.h"
#include "pm/pm.h"
#include "efpg/efpg.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_efuse.h"
#include "driver/chip/hal_wdg.h"
#include "driver/chip/hal_adc.h"
#include "driver/chip/hal_crypto.h"
#include <stddef.h>

/* ---- crash record (svc3) ----------------------------------------------
 * Breadcrumbs in RAM that survive a watchdog/fault reset (not a power cut).
 * 0x66f00 is below the bootloader's RAM (0x67000) and far above the app's
 * early heap. A checksum tells a valid record from power-on garbage. The
 * hardware watchdog (4 s, fed only by crumb()) turns a hang, even one with
 * IRQs off, into a reset; the next boot prints the record and stays in the
 * service console. */
#define CR_MAGIC	0x41425533U	/* "ABU3" */
#define CR_HIST		8
struct crec {
	uint32_t magic;
	uint32_t state;		/* 1 = booting, 2 = init done */
	uint32_t boots;
	uint32_t step;
	uint32_t ms;
	const char *hist[CR_HIST];
	uint32_t hist_ms[CR_HIST];
	uint32_t fault_no, pc, lr, xpsr, cfsr, hfsr, bfar, mmfar;
	uint32_t sum;
};
#define CR ((volatile struct crec *)0x00066F00U)

static struct crec cr_last;	/* copy of the previous run's record */
static int cr_crashed;		/* previous run died during boot */
static int wdg_on;
static int wdg_skip;		/* "nowdg" in the console */

static uint32_t cr_sum(volatile struct crec *c)
{
	volatile uint32_t *w = (volatile uint32_t *)c;
	uint32_t s = 0x5a5a1234U, i;

	for (i = 0; i < offsetof(struct crec, sum) / 4; i++)
		s = (s << 5 | s >> 27) ^ w[i];
	return s;
}

static void crumb(const char *what)
{
	uint32_t n = CR->step;

	CR->hist[n % CR_HIST] = what;
	CR->hist_ms[n % CR_HIST] = OS_TicksToMSecs(OS_GetTicks());
	CR->step = n + 1;
	CR->sum = cr_sum(CR);
	if (wdg_on)
		HAL_WDG_Feed();
}

void abus_fault_hook(uint32_t *pstack, uint32_t no)
{
	CR->fault_no = no;
	CR->lr = pstack[13];
	CR->pc = pstack[14];
	CR->xpsr = pstack[15];
	CR->cfsr = *(volatile uint32_t *)0xE000ED28;
	CR->hfsr = *(volatile uint32_t *)0xE000ED2C;
	CR->mmfar = *(volatile uint32_t *)0xE000ED34;
	CR->bfar = *(volatile uint32_t *)0xE000ED38;
	CR->sum = cr_sum(CR);
}

static void cr_boot(void)
{
	uint32_t boots = 0;

	if (CR->magic == CR_MAGIC && CR->sum == cr_sum(CR)) {
		memcpy(&cr_last, (const void *)CR, sizeof(cr_last));
		cr_crashed = (cr_last.state == 1);
		boots = cr_last.boots;
	}
	memset((void *)CR, 0, sizeof(*CR));
	CR->magic = CR_MAGIC;
	CR->state = 1;
	CR->boots = boots + 1;
	CR->sum = cr_sum(CR);
}

static int cr_ptr_ok(const char *p)
{
	extern uint8_t __end__[];

	return (uint32_t)p >= 0x10000U && (uint8_t *)p < __end__;
}

static void cr_report(void)
{
	uint32_t i, n = cr_last.step;

	if (cr_last.magic != CR_MAGIC) {
		printf("crash: no record from the previous run (power-on)\n");
		return;
	}
	printf("crash: previous run %s, boot #%lu, %lu crumbs\n",
	       cr_crashed ? "DIED during boot" : "reached init done",
	       (unsigned long)cr_last.boots, (unsigned long)n);
	for (i = n > CR_HIST ? n - CR_HIST : 0; i < n; i++) {
		const char *s = cr_last.hist[i % CR_HIST];

		printf("crash: #%lu @%lu ms %s\n", (unsigned long)i,
		       (unsigned long)cr_last.hist_ms[i % CR_HIST],
		       cr_ptr_ok(s) ? s : "?");
	}
	if (cr_last.fault_no)
		printf("crash: FAULT exc %lu pc %08lx lr %08lx xpsr %08lx cfsr %08lx"
		       " hfsr %08lx bfar %08lx mmfar %08lx\n",
		       (unsigned long)cr_last.fault_no, (unsigned long)cr_last.pc,
		       (unsigned long)cr_last.lr, (unsigned long)cr_last.xpsr,
		       (unsigned long)cr_last.cfsr, (unsigned long)cr_last.hfsr,
		       (unsigned long)cr_last.bfar, (unsigned long)cr_last.mmfar);
	else
		printf("crash: no CPU fault recorded (hang -> watchdog, or reset)\n");
}

static void wdg_arm(void)
{
	WDG_InitParam p;

	if (wdg_skip)
		return;
	memset(&p, 0, sizeof(p));
	p.event = WDG_EVT_RESET;
	p.timeout = WDG_TIMEOUT_4SEC;
	p.resetCycle = WDG_DEFAULT_RESET_CYCLE;
	if (HAL_WDG_Init(&p) != HAL_OK) {
		printf("boot: wdg init failed\n");
		return;
	}
	HAL_WDG_Start();
	wdg_on = 1;
	printf("boot: watchdog armed (4 s, fed by boot steps)\n");
}

static void wdg_feed_timer(void *arg)
{
	(void)arg;
	HAL_WDG_Feed();
}

/* after init: hand the watchdog to a periodic timer, mark the run good */
static void wdg_handover(void)
{
	static OS_Timer_t t;

	CR->state = 2;
	crumb("init done");
	if (!wdg_on)
		return;
	OS_TimerSetInvalid(&t);
	if (OS_TimerCreate(&t, OS_TIMER_PERIODIC, wdg_feed_timer, NULL, 1000) == OS_OK)
		OS_TimerStart(&t);
	else
		HAL_WDG_Stop();
}

static void boot_mark(const char *step)
{
	UART_T *u = HAL_UART_GetInstance(BOARD_MAIN_UART_ID);

	crumb(step);
	printf("boot: %s\n", step);
	while (!HAL_UART_IsTxEmpty(u))
		;
}

/* eFuse read probe: on hardware the first eFuse read in level1 (PRNG seed,
 * chip ID) resets the chip, although the stock boot reads eFuses fine.
 * Probing between the steps shows which step makes it fatal. */
static void efuse_probe(const char *where)
{
	uint8_t id[16];
	int r;

	printf("boot: efuse probe %s ...", where);
	while (!HAL_UART_IsTxEmpty(HAL_UART_GetInstance(BOARD_MAIN_UART_ID)))
		;
	r = efpg_read(EFPG_FIELD_CHIPID, id);
	printf(" %s\n", r == 0 ? "ok" : "err");
	while (!HAL_UART_IsTxEmpty(HAL_UART_GetInstance(BOARD_MAIN_UART_ID)))
		;
}

/* ---- service console on UART0 (debug pads TX/RX) ------------------------
 * Entered by sending 's' within 3 s after "boot: level0 done", i.e. before
 * the WLAN/net init that may be broken. Lets the PC (tools/xr871-svc.py via
 * the ESP32 monitor) inspect registers/eFuse and rewrite the XR871 flash
 * without the clip. The boot area 0x0-0x7fff is never written. Replies:
 * one or more lines, then "OK ..." or "ERR ...".
 */
#define SVC_FLASH_MIN	0x8000U
#define SVC_FLASH_END	0x200000U

/* one flash driver state for the console, the SDK init and the update */
static void flash_init_once(void)
{
	hal_ext_flash_init();
}

static UART_T *svc_uart(void)
{
	return HAL_UART_GetInstance(BOARD_MAIN_UART_ID);
}

static void svc_flush(void)
{
	while (!HAL_UART_IsTxEmpty(svc_uart()))
		;
}

static int svc_getc(uint32_t timeout_ms)
{
	uint32_t t0 = OS_GetTicks();

	for (;;) {
		if (HAL_UART_IsRxReady(svc_uart()))
			return HAL_UART_GetRxData(svc_uart());
		/* busy poll: the RX FIFO is 64 bytes (~5 ms at 115200), an
		 * OS sleep of one tick could overflow it */
		if (OS_TicksToMSecs(OS_GetTicks() - t0) >= timeout_ms)
			return -1;
		OS_ThreadYield();	/* let the mcu/t30 threads (same prio) run */
	}
}

static uint32_t svc_crc32(uint32_t crc, const uint8_t *p, uint32_t n)
{
	int k;

	crc = ~crc;
	while (n--) {
		crc ^= *p++;
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1));
	}
	return ~crc;
}

static int svc_hex(int c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static int svc_range_ok(uint32_t off, uint32_t len)
{
	return off >= SVC_FLASH_MIN && len > 0 && off + len <= SVC_FLASH_END &&
	       off + len > off;
}

/* flash CRC32 of [off, off+len) */
static int svc_fcrc(uint32_t off, uint32_t len, uint32_t *crc)
{
	static uint8_t buf[256];
	uint32_t n;

	*crc = 0;
	if ((flash_init_once(), HAL_Flash_Open(PRJCONF_IMG_FLASH, 5000) != HAL_OK))
		return -1;
	while (len) {
		n = len > sizeof(buf) ? sizeof(buf) : len;
		if (HAL_Flash_Read(PRJCONF_IMG_FLASH, off, buf, n) != HAL_OK) {
			HAL_Flash_Close(PRJCONF_IMG_FLASH);
			return -1;
		}
		*crc = svc_crc32(*crc, buf, n);
		off += n;
		len -= n;
	}
	HAL_Flash_Close(PRJCONF_IMG_FLASH);
	return 0;
}

/* ---- hardware diagnostics (svc4d) ---------------------------------------
 * i2c: port 0 = PA4/PA5 (AXP2601 fuel gauge, initialised on first use),
 * port 1 = PA8/PA9 (power MCU, already up). Both share i2c_lock.
 * gpio: pins that would break the console, the flash, the T30 link or
 * power are refused. */
#define AXP_ADDR 0x62	/* AXP2601 7-bit address (to be confirmed by "i2c scan 0") */

static int diag_i2c_port(uint32_t p)
{
	static int up0;

	if (p > 1)
		return -1;
	if (p == 0 && !up0) {
		I2C_InitParam ip = { I2C_ADDR_MODE_7BIT, 100000 };

		if (HAL_I2C_Init(I2C0_ID, &ip) != HAL_OK)
			return -1;
		up0 = 1;
	}
	return 0;
}

static int32_t diag_i2c_rd(uint32_t p, uint32_t addr, uint32_t reg, uint8_t *b, int n)
{
	int32_t r;

	if (diag_i2c_port(p))
		return -1;
	OS_MutexLock(&i2c_lock, OS_WAIT_FOREVER);
	if (p == 1)
		mcu_wake();
	r = HAL_I2C_Master_Receive_Mem_IT(p ? I2C1_ID : I2C0_ID, (uint16_t)addr, reg,
					  I2C_MEMADDR_SIZE_8BIT, b, n);
	OS_MutexUnlock(&i2c_lock);
	return r;
}

static int32_t diag_i2c_wr(uint32_t p, uint32_t addr, uint32_t reg, uint8_t v)
{
	int32_t r;

	if (diag_i2c_port(p))
		return -1;
	OS_MutexLock(&i2c_lock, OS_WAIT_FOREVER);
	if (p == 1)
		mcu_wake();
	r = HAL_I2C_Master_Transmit_Mem_IT(p ? I2C1_ID : I2C0_ID, (uint16_t)addr, reg,
					   I2C_MEMADDR_SIZE_8BIT, &v, 1);
	OS_MutexUnlock(&i2c_lock);
	return r;
}

static int diag_pin_ok(int port, int pin)
{
	/* PA4/5 i2c0, PA7 MCU IRQ, PA8/9 i2c1, PA15 SPI ready, PA16 T30 power,
	 * PA17/18 T30 UART, PA19-22 SPI; PB0/1 console, PB2-7 flash */
	static const uint32_t bad_a = (1U << 4) | (1U << 5) | (1U << 7) | (1U << 8) |
		(1U << 9) | (1U << 15) | (1U << 16) | (1U << 17) | (1U << 18) |
		(0xfU << 19);

	if (pin < 0 || pin > 31)
		return 0;
	return port == 0 ? !(bad_a & (1U << pin)) : pin > 7;
}

static void svc_cmd(char *l)
{
	static uint8_t buf[256];
	char *a[7] = { 0 };
	int n = 0;
	uint32_t x, y, i;

	while (*l && n < 7) {
		while (*l == ' ')
			*l++ = 0;
		if (!*l)
			break;
		a[n++] = l;
		while (*l && *l != ' ')
			l++;
	}
	if (!n)
		return;
	x = a[1] ? strtoul(a[1], NULL, 0) : 0;
	y = a[2] ? strtoul(a[2], NULL, 0) : 0;

	if (!strcmp(a[0], "help")) {
		printf("peek A [N] | poke A V | efuse | efusehw W | fcrc OFF LEN | "
		       "fread OFF LEN | ferase OFF LEN | fwrite OFF HEX | spi | crash | nowdg | boot | reboot\n");
		printf("i2c scan P | i2c r P ADDR REG [N] | i2c w P ADDR REG VAL | "
		       "gpio a|b PIN [0|1|in] | mcu CMD VAL | bat\n");
		printf("OK\n");
	} else if (!strcmp(a[0], "i2c") && n >= 3) {
		uint32_t p = y;	/* a[2] = port */

		if (!strcmp(a[1], "scan")) {
			int found = 0;

			for (i = 0x08; i < 0x78; i++) {
				if (diag_i2c_rd(p, i, 0, buf, 1) == 1) {
					printf("i2c%lu: 0x%02lx\n", (unsigned long)p, (unsigned long)i);
					found++;
				}
			}
			printf("OK %d devices\n", found);
		} else if (!strcmp(a[1], "r") && n >= 5) {
			uint32_t addr = strtoul(a[3], NULL, 0), reg = strtoul(a[4], NULL, 0);
			int cnt = a[5] ? (int)strtoul(a[5], NULL, 0) : 1;
			int32_t r;

			if (cnt < 1 || cnt > 64)
				cnt = 1;
			r = diag_i2c_rd(p, addr, reg, buf, cnt);
			for (i = 0; r == cnt && i < (uint32_t)cnt; i++)
				printf("%02x%s", buf[i], (i & 15) == 15 ? "\n" : " ");
			printf(r == cnt ? "\nOK\n" : "ERR i2c read %ld\n", (long)r);
		} else if (!strcmp(a[1], "w") && n >= 6) {
			int32_t r = diag_i2c_wr(p, strtoul(a[3], NULL, 0), strtoul(a[4], NULL, 0),
						(uint8_t)strtoul(a[5], NULL, 0));

			printf(r == 1 ? "OK\n" : "ERR i2c write %ld\n", (long)r);
		} else {
			printf("ERR i2c usage\n");
		}
	} else if (!strcmp(a[0], "gpio") && n >= 3) {
		int port = (a[1][0] == 'b' || a[1][0] == 'B') ? 1 : 0;
		int pin = (int)y;
		GPIO_Port gp = port ? GPIO_PORT_B : GPIO_PORT_A;

		if (!diag_pin_ok(port, pin)) {
			printf("ERR P%c%d is protected\n", port ? 'B' : 'A', pin);
		} else if (n >= 4) {
			GPIO_InitParam g = { GPIOx_Pn_F1_OUTPUT, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE };

			if (!strcmp(a[3], "in")) {
				g.mode = GPIOx_Pn_F0_INPUT;
				HAL_GPIO_Init(gp, (GPIO_Pin)pin, &g);
			} else {
				HAL_GPIO_Init(gp, (GPIO_Pin)pin, &g);
				HAL_GPIO_WritePin(gp, (GPIO_Pin)pin, strtoul(a[3], NULL, 0) ?
						  GPIO_PIN_HIGH : GPIO_PIN_LOW);
			}
			printf("OK P%c%d %s\n", port ? 'B' : 'A', pin, a[3]);
		} else {
			printf("OK P%c%d = %d\n", port ? 'B' : 'A', pin,
			       HAL_GPIO_ReadPin(gp, (GPIO_Pin)pin) == GPIO_PIN_HIGH);
		}
	} else if (!strcmp(a[0], "t30") && n >= 2) {
		/* t30 reset | t30 auto 0|1 | t30 status (runtime only) */
		if (!OS_MutexIsValid(&core_lock)) {
			printf("ERR bridge not running\n");
		} else if (!strcmp(a[1], "reset")) {
			OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
			bridge_t30_reset("console");
			OS_MutexUnlock(&core_lock);
			printf("OK\n");
		} else if (!strcmp(a[1], "auto") && n == 3) {
			OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
			bridge_t30_auto(y != 0);
			OS_MutexUnlock(&core_lock);
			printf("OK\n");
		} else {
			const struct bridge_stats *st = bridge_stats();

			printf("OK t30 on %d resets %lu hb %lu spi %lu\n", st->t30_on,
			       (unsigned long)st->t30_resets, (unsigned long)st->t30_hb,
			       (unsigned long)st->spi_frames);
		}
	} else if (!strcmp(a[0], "mcu") && n == 3) {
		int r = h_mcu_send((uint8_t)x, (uint8_t)y);

		printf(r == 0 ? "OK\n" : "ERR mcu send\n");
	} else if (!strcmp(a[0], "bat")) {
		/* raw AXP2601 register dump until the register map is decoded */
		int32_t r = diag_i2c_rd(0, AXP_ADDR, 0, buf, 64);

		for (i = 0; r == 64 && i < 64; i++)
			printf("%02x%s", buf[i], (i & 15) == 15 ? "\n" : " ");
		printf(r == 64 ? "OK axp 0x%02x regs 0x00-0x3f\n" : "ERR axp 0x%02x read %ld\n",
		       AXP_ADDR, (long)r);
	} else if (!strcmp(a[0], "peek")) {
		if (!y)
			y = 1;
		for (i = 0; i < y && i < 64; i++)
			printf("%08lx: %08lx\n", (unsigned long)(x + 4 * i),
			       (unsigned long)*(volatile uint32_t *)(x + 4 * i));
		printf("OK\n");
	} else if (!strcmp(a[0], "poke") && n == 3) {
		*(volatile uint32_t *)x = y;
		printf("OK\n");
	} else if (!strcmp(a[0], "efuse")) {
		extern uint32_t abus_efuse_cache[64];
		extern uint32_t abus_efuse_cached;

		printf("cached %lu\n", (unsigned long)abus_efuse_cached);
		for (i = 0; i < 64; i++)
			printf("%02lu: %08lx\n", (unsigned long)i, (unsigned long)abus_efuse_cache[i]);
		printf("OK\n");
	} else if (!strcmp(a[0], "efusehw") && n == 2) {
		/* direct hardware read of one word, bypassing the cache: the
		 * test that used to reset the chip */
		volatile uint32_t *ctrl = (volatile uint32_t *)0x40043C40;
		uint32_t v;

		printf("efusehw %lu ...", (unsigned long)x);
		svc_flush();
		*ctrl |= 1U << 28;
		*ctrl = (*ctrl & ~0xFF0000U) | (((x & 63) << 2) << 16);
		*ctrl = (*ctrl & ~0xFF02U) | (0xACU << 8) | 2U;
		while (*ctrl & 2U)
			;
		while (*ctrl & 4U)
			;
		v = *(volatile uint32_t *)0x40043C60;
		*ctrl &= ~0xFF00FCU;
		*ctrl &= ~(1U << 28);
		printf(" %08lx\nOK\n", (unsigned long)v);
	} else if (!strcmp(a[0], "fcrc") && n == 3) {
		uint32_t c;

		if (x + y < x || x + y > SVC_FLASH_END || svc_fcrc(x, y, &c))
			printf("ERR fcrc\n");
		else
			printf("OK %08lx\n", (unsigned long)c);
	} else if (!strcmp(a[0], "fread") && n == 3) {
		if (y > sizeof(buf) || x + y > SVC_FLASH_END ||
		    (flash_init_once(), HAL_Flash_Open(PRJCONF_IMG_FLASH, 5000) != HAL_OK)) {
			printf("ERR fread\n");
		} else {
			HAL_Status st = HAL_Flash_Read(PRJCONF_IMG_FLASH, x, buf, y);

			HAL_Flash_Close(PRJCONF_IMG_FLASH);
			for (i = 0; st == HAL_OK && i < y; i++)
				printf("%02x", buf[i]);
			printf(st == HAL_OK ? "\nOK\n" : "ERR read\n");
		}
	} else if (!strcmp(a[0], "ferase") && n == 3) {
		if (!svc_range_ok(x, y) || (x & 0xFFF) || (y & 0xFFF) ||
		    (flash_init_once(), HAL_Flash_Open(PRJCONF_IMG_FLASH, 5000) != HAL_OK)) {
			printf("ERR ferase\n");
		} else {
			HAL_Status st = HAL_Flash_Erase(PRJCONF_IMG_FLASH, FLASH_ERASE_4KB,
							x, y >> 12);

			HAL_Flash_Close(PRJCONF_IMG_FLASH);
			printf(st == HAL_OK ? "OK\n" : "ERR erase\n");
		}
	} else if (!strcmp(a[0], "fwrite") && n == 3) {
		uint32_t len = strlen(a[2]) / 2;
		int bad = (strlen(a[2]) & 1) || len > sizeof(buf);

		for (i = 0; !bad && i < len; i++) {
			int h = svc_hex(a[2][2 * i]), lo = svc_hex(a[2][2 * i + 1]);

			if (h < 0 || lo < 0)
				bad = 1;
			else
				buf[i] = (uint8_t)(h << 4 | lo);
		}
		if (bad || !svc_range_ok(x, len) ||
		    (flash_init_once(), HAL_Flash_Open(PRJCONF_IMG_FLASH, 5000) != HAL_OK)) {
			printf("ERR fwrite\n");
		} else {
			HAL_Status st = HAL_Flash_Write(PRJCONF_IMG_FLASH, x, buf, len);

			HAL_Flash_Close(PRJCONF_IMG_FLASH);
			if (st == HAL_OK)
				printf("OK %08lx\n", (unsigned long)svc_crc32(0, buf, len));
			else
				printf("ERR write\n");
		}
	} else if (!strcmp(a[0], "spi")) {
		spis_diag();
		printf("OK\n");
	} else if (!strcmp(a[0], "crash")) {
		cr_report();
		printf("OK\n");
	} else if (!strcmp(a[0], "nowdg")) {
		wdg_skip = 1;
		printf("OK watchdog stays off for this boot\n");
	} else if (!strcmp(a[0], "reboot")) {
		printf("OK\n");
		svc_flush();
		HAL_WDG_Reboot();
	} else {
		printf("ERR unknown (help)\n");
	}
}

static void svc_window(void)
{
	static char line[640];
	int c, len = 0;
	uint32_t t0 = OS_GetTicks();

	/* drop anything that arrived before */
	while (HAL_UART_IsRxReady(svc_uart()))
		(void)HAL_UART_GetRxData(svc_uart());
	cr_report();
	svc_flush();
	if (cr_crashed) {
		printf("boot: previous run died, service console stays open\n");
		goto console;
	}
	printf("boot: send 's' within 3 s for the service console\n");
	svc_flush();
	for (;;) {
		c = svc_getc(50);
		if (c == 's')
			break;
		if (OS_TicksToMSecs(OS_GetTicks() - t0) >= 3000) {
			boot_mark("no service request, normal boot");
			return;
		}
	}
	while (HAL_UART_IsRxReady(svc_uart()))
		(void)HAL_UART_GetRxData(svc_uart());
console:
	printf("svc: service console, 'help'; 'boot' continues the normal boot\nOK\n");
	svc_flush();
	for (;;) {
		c = svc_getc(1000);
		if (c < 0)
			continue;
		if (c == '\r' || c == '\n') {
			line[len] = 0;
			if (len && !strcmp(line, "boot")) {
				printf("OK\n");
				boot_mark("service: continuing normal boot");
				return;
			}
			if (len)
				svc_cmd(line);
			svc_flush();
			len = 0;
		} else if (len < (int)sizeof(line) - 1) {
			line[len++] = (char)c;
		}
	}
}

/* ---- runtime console (svc4) --------------------------------------------
 * The same commands as the boot-time console, but while the bridge runs:
 * UART0 RX interrupt -> ring buffer -> low-priority thread. Lets the PC
 * reboot or reflash the XR871 without power-cycling the camera. Every
 * 10 s it also prints the SPI link diagnostics. */
static uint8_t rt_buf[2048];
static volatile uint32_t rt_w, rt_r;
static OS_Semaphore_t rt_sem;
static OS_Thread_t rt_thread;

static void rt_rx_cb(void *arg)
{
	UART_T *u = svc_uart();

	(void)arg;
	while (HAL_UART_IsRxReady(u)) {
		uint8_t c = HAL_UART_GetRxData(u);
		uint32_t n = (rt_w + 1) % sizeof(rt_buf);

		if (n != rt_r) {
			rt_buf[rt_w] = c;
			rt_w = n;
		}
		if (c == '\r' || c == '\n')
			OS_SemaphoreRelease(&rt_sem);
	}
}

/* AXP2601: wake it if asleep (reg 0x02 bit0), read VBAT (0x04/05, mV, big
 * endian), temperature (0x06, signed) and SOC (0x08). 0 = ok. */
/*
 * Battery model for the AXP2601, copied from the stock 2022 XR871 app
 * (0x30da0, the table it selects with "charge mode 1" as logged by camera 2;
 * the other table at 0x30d48 is for charge mode 0). The gauge forgets it
 * when it loses power; without it SOC follows a generic curve.
 */
static const uint8_t axp_model[80] = {
	0x01, 0xf5, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x1e, 0x32, 0x01,
	0x14, 0x05, 0x0a, 0x04, 0x74, 0xff, 0xe2, 0x0c, 0xdf, 0x10, 0x36, 0xfb,
	0x80, 0x01, 0x54, 0x75, 0xc7, 0x06, 0x36, 0x06, 0x0c, 0x0a, 0xc2, 0x0f,
	0x38, 0x0e, 0xc0, 0x09, 0x4a, 0x0e, 0x09, 0x0e, 0x00, 0x03, 0xed, 0x03,
	0xd7, 0x08, 0xb6, 0x0d, 0x83, 0x0d, 0x62, 0x08, 0x49, 0x0d, 0x0d, 0x0c,
	0x3b, 0x02, 0x09, 0x01, 0xd7, 0x06, 0xa5, 0x0b, 0x6e, 0x0b, 0x41, 0x06,
	0x0f, 0x80, 0x2b, 0x1c, 0x10, 0xff, 0x04, 0xfa,
};
static int axp_model_tries;

/* stock sequence: cfg(0x03) bit0 = write enable, model bytes via 0x01,
 * re-arm, read back and compare, then cfg bit4 = loaded and pulse 0x02 bit5 */
static int axp_model_load(void)
{
	uint8_t cfg, v;
	int i, ok = 1;

	if (diag_i2c_rd(1, AXP_ADDR, 0x03, &cfg, 1) != 1)
		return -1;
	if (diag_i2c_wr(1, AXP_ADDR, 0x03, cfg | 1) != 1)
		return -1;
	for (i = 0; i < (int)sizeof(axp_model); i++)
		if (diag_i2c_wr(1, AXP_ADDR, 0x01, axp_model[i]) != 1)
			return -1;
	diag_i2c_wr(1, AXP_ADDR, 0x03, cfg & ~1);
	diag_i2c_wr(1, AXP_ADDR, 0x03, cfg | 1);
	for (i = 0; i < (int)sizeof(axp_model); i++)
		if (diag_i2c_rd(1, AXP_ADDR, 0x01, &v, 1) != 1 || v != axp_model[i]) {
			ok = 0;
			break;
		}
	diag_i2c_wr(1, AXP_ADDR, 0x03, ok ? ((cfg & ~1) | 0x10) : (cfg & ~1));
	if (diag_i2c_rd(1, AXP_ADDR, 0x02, &v, 1) == 1) {	/* restart the gauge */
		diag_i2c_wr(1, AXP_ADDR, 0x02, v | 0x20);
		diag_i2c_wr(1, AXP_ADDR, 0x02, v & ~0x20);
	}
	if (diag_i2c_rd(1, AXP_ADDR, 0x21, &v, 1) == 1)
		diag_i2c_wr(1, AXP_ADDR, 0x21, v | 0x0f);
	printf("bat: model %s (byte %d)\n", ok ? "loaded" : "verify FAILED", i);
	return ok ? 0 : -1;
}

/* AXP2601 0x0a:0x0b time to empty, 0x0c:0x0d time to full (big endian,
 * minutes, inferred; see work/cam2/fable-stock-re.md). -1 = read failed. */
static void bat_times(int *t2e, int *t2f)
{
	uint8_t b[4];

	*t2e = *t2f = -1;
	if (diag_i2c_rd(1, AXP_ADDR, 0x0a, b, 4) != 4)
		return;
	*t2e = b[0] << 8 | b[1];
	*t2f = b[2] << 8 | b[3];
}

static int bat_read(uint16_t *mv, uint8_t *soc, int8_t *t, uint8_t *flags)
{
	uint8_t b[2];

	*flags = 0;
	if (diag_i2c_rd(1, AXP_ADDR, 0x02, b, 1) != 1)
		return -1;
	if (b[0] & 1) {
		diag_i2c_wr(1, AXP_ADDR, 0x02, b[0] & ~1);
		*flags |= 1;
	}
	if (diag_i2c_rd(1, AXP_ADDR, 0x03, b, 1) == 1 && !(b[0] & 0x10) &&
	    axp_model_tries < 3) {
		axp_model_tries++;
		if (!axp_model_load())
			*flags |= 2;
	}
	if (diag_i2c_rd(1, AXP_ADDR, 0x04, b, 2) != 2)
		return -1;
	*mv = (uint16_t)(b[0] << 8 | b[1]);
	if (diag_i2c_rd(1, AXP_ADDR, 0x06, b, 1) != 1)
		return -1;
	*t = (int8_t)b[0];
	if (diag_i2c_rd(1, AXP_ADDR, 0x08, b, 1) != 1)
		return -1;
	*soc = b[0] > 100 ? 100 : b[0];
	return 0;
}

/*
 * Status LEDs, updated about once per second:
 *   red (PA11):  charge indicator like the stock firmware's charge LED -
 *                slow blink while USB power is in and the battery is not
 *                full, on when full on USB, off on battery;
 *   blue (PA10): on while the T30 asks for it (T30_LED, xr871d sets it
 *                while a video stream runs), off when the T30 is off.
 * A pin held by a T30 GPIO test is left alone.
 */
static int led_soc = -1;

static void led_update(void)
{
	static int phase;
	const struct bridge_stats *st = bridge_stats();
	int red = 0, blue;

	phase ^= 1;
	if (st->usb == 1 && led_soc >= 0)
		red = led_soc >= 100 ? 1 : phase;
	blue = st->t30_on && (st->led_req & BR_LED_BLUE);
	if (!bridge_gpio_busy(0, 11))
		HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_11, red ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
	if (!bridge_gpio_busy(0, 10))
		HAL_GPIO_WritePin(GPIO_PORT_A, GPIO_PIN_10, blue ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

static void bat_report(void)
{
	uint16_t mv;
	uint8_t soc, flags, p[9];
	int8_t t;
	int t2e, t2f;

	if (bat_read(&mv, &soc, &t, &flags))
		return;
	bat_times(&t2e, &t2f);
	p[0] = mv & 0xff;
	p[1] = mv >> 8;
	p[2] = soc;
	p[3] = (uint8_t)t;
	{
		const struct bridge_stats *st = bridge_stats();

		if (st->usb == 1)
			flags |= 4;	/* USB power present */
		if (st->usb == 1 && soc < 100)
			flags |= 8;	/* charging (inferred: USB in, not full) */
	}
	led_soc = soc;
	hal_ext_battery_set(soc, mv);	/* for the status record (0xd0) */
	p[4] = flags;
	p[5] = t2e >= 0 ? t2e & 0xff : 0xff;
	p[6] = t2e >= 0 ? t2e >> 8 : 0xff;
	p[7] = t2f >= 0 ? t2f & 0xff : 0xff;
	p[8] = t2f >= 0 ? t2f >> 8 : 0xff;
	OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
	bridge_battery(soc, mv, t2e, t2f);	/* charge restart, low-battery shutdown */
	bridge_t30_send(XR_BATTERY, 0, p, sizeof(p));
	OS_MutexUnlock(&core_lock);
	printf("bat: %u mV %u%% %d C t2e %d t2f %d%s%s\n", mv, soc, t, t2e, t2f,
	       (flags & 1) ? " (gauge woken)" : "",
	       (flags & 8) ? " charging" : (flags & 4) ? " usb" : "");
}

static void rt_task(void *arg)
{
	static char line[640];
	int len = 0;
	uint32_t last = OS_GetTicks(), led_last = last;

	(void)arg;
	for (;;) {
		OS_SemaphoreWait(&rt_sem, 1000);
		while (rt_r != rt_w) {
			char c = (char)rt_buf[rt_r];

			rt_r = (rt_r + 1) % sizeof(rt_buf);
			if (c == '\r' || c == '\n') {
				line[len] = 0;
				if (len)
					svc_cmd(line);
				len = 0;
			} else if (len < (int)sizeof(line) - 1) {
				line[len++] = c;
			}
		}
		if (OS_TicksToMSecs(OS_GetTicks() - led_last) >= 1000) {
			led_last = OS_GetTicks();
			led_update();
		}
		if (OS_TicksToMSecs(OS_GetTicks() - last) >= 10000) {
			static int n;

			last = OS_GetTicks();
			spis_diag();
			if (n++ % 3 == 0)	/* every 30 s */
				bat_report();
		}
	}
}

static void rt_console_start(void)
{
	if (OS_SemaphoreCreate(&rt_sem, 0, 1000) != OS_OK ||
	    HAL_UART_EnableRxCallback(BOARD_MAIN_UART_ID, rt_rx_cb, NULL) != HAL_OK ||
	    OS_ThreadCreate(&rt_thread, "rtcon", rt_task, NULL,
			    OS_THREAD_PRIO_APP, 3 * 1024) != OS_OK) {
		printf("boot: runtime console failed\n");
		return;
	}
	printf("boot: runtime console on UART0 ('help')\n");
}

void platform_init_level0(void)
{
	{
		uint8_t b = 0xee;
		HAL_Status st;
		UART_T *u = HAL_UART_GetInstance(BOARD_MAIN_UART_ID);

		crumb("l0: entered");
		printf("boot: efuse read 608/2 with IRQs off ...");
		crumb("l0: printf returned");
		while (!HAL_UART_IsTxEmpty(u))
			;
		crumb("l0: uart tx empty");
		__disable_irq();
		crumb("l0: irqs off");
		st = HAL_EFUSE_Read(608, 2, &b);
		crumb("l0: efuse read returned");
		__enable_irq();
		crumb("l0: irqs on");
		printf(" survived, status %d value 0x%02x\n", (int)st, b);
		while (!HAL_UART_IsTxEmpty(u))
			;
	}
	efuse_probe("before flash init");
	boot_mark("flash init");
	flash_init_once();
	boot_mark("image init");
	image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR, PRJCONF_IMG_MAX_SIZE);
	boot_mark("level0 done");
}

#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
#define RAND_SYS_TICK() ((SysTick->VAL & 0xffffff) | (OS_GetTicks() << 24))
static void diag_prng_init_seed(void)
{
	uint32_t seed[6];
	HAL_Status status;
	ADC_InitParam initParam;
	uint32_t chksum;

	initParam.delay = 0;
	initParam.freq = 1000000;
	initParam.mode = ADC_CONTI_CONV;
	boot_mark("prng: adc vbat");
	status = HAL_ADC_Init(&initParam);
	if (status == HAL_OK) {
		boot_mark("prng: adc conv");
		status = HAL_ADC_Conv_Polling(ADC_CHANNEL_VBAT, &seed[0], 1000);
		boot_mark("prng: adc deinit");
		HAL_ADC_DeInit();
	}
	if (status != HAL_OK)
		seed[0] = RAND_SYS_TICK();
	boot_mark("prng: image read");
	if (image_read(IMAGE_APP_ID, IMAGE_SEG_HEADER,
		       offsetof(section_header_t, header_chksum),
		       &chksum, sizeof(chksum)) == sizeof(chksum))
		seed[0] = (seed[0] << 24) ^ (seed[0] << 12) ^ (seed[0]) ^ chksum;
	boot_mark("prng: efuse chipid");
	efpg_read(EFPG_FIELD_CHIPID, (uint8_t *)&seed[1]);
	seed[5] = RAND_SYS_TICK();
	boot_mark("prng: set seed");
	HAL_PRNG_SetSeed(seed);
}
#endif

#if 0	/* svc3: own watchdog, see wdg_arm() */
static void diag_wdg_feed(void *arg)
{
	(void)arg;
	HAL_WDG_Feed();
}

static void diag_wdg_start(void)
{
	static OS_Timer_t timer;
	WDG_InitParam param;

	param.event = WDG_EVT_RESET;
	param.timeout = PRJCONF_WDG_TIMEOUT;
	param.resetCycle = WDG_DEFAULT_RESET_CYCLE;
	HAL_WDG_Init(&param);
	OS_TimerSetInvalid(&timer);
	if (OS_TimerCreate(&timer, OS_TIMER_PERIODIC, diag_wdg_feed, NULL,
			   PRJCONF_WDG_FEED_PERIOD) != OS_OK) {
		HAL_WDG_DeInit();
		return;
	}
	HAL_WDG_Start();
	OS_TimerStart(&timer);
}
#endif

void platform_init_level1(void)
{
#if PRJCONF_CE_EN
	boot_mark("ce init");
	HAL_CE_Init();
#endif
	boot_mark("sys_ctrl");
	sys_ctrl_create();
	boot_mark("net_ctrl");
	net_ctrl_init();
#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
	diag_prng_init_seed();
#endif
	boot_mark("sysinfo init");
	sysinfo_init();
#if PRJCONF_PM_EN
	boot_mark("pm select");
	pm_mode_platform_select(PRJCONF_PM_MODE);
#endif
	boot_mark("net_sys init");
	net_sys_init();
	boot_mark("net_sys start (wlan core)");
	{
		int r = net_sys_start(sysinfo_get()->wlan_mode);

		printf("boot: net_sys_start rc %d\n", r);
	}
#if PRJCONF_NET_PM_EN
	pm_register_wlan_power_onoff(net_sys_onoff, PRJCONF_NET_PM_MODE);
#endif
	boot_mark("level1 done");
}
#endif /* BR_WIFI */

int main(void)
{
	UART_InitParam up = { 115200, UART_DATA_BITS_8, UART_STOP_BITS_1,
			      UART_PARITY_NONE, 0 };
	I2C_InitParam ip = { I2C_ADDR_MODE_7BIT, 100000 };

#ifdef BR_WIFI
	cr_boot();
	boot_mark("main");
#else
	printf("\n\nabus-xr871 fw " __DATE__ " " __TIME__ "\n");
#endif

	/* MCU, T30 power and their threads FIRST, like m1: on the hardware the
	 * board resets the XR871 ~240 ms after boot unless the app talks to the
	 * MCU early (every m2 build died at that fixed time, m1 never did). */
	/* outputs to a known state before anything else */
	out_pin(T30_PWR_PIN, GPIO_PIN_LOW);
	out_pin(GPIO_PIN_14, GPIO_PIN_LOW);
	out_pin(GPIO_PIN_15, GPIO_PIN_LOW);
	out_pin(GPIO_PIN_10, GPIO_PIN_LOW);	/* blue LED: video stream */
	out_pin(GPIO_PIN_11, GPIO_PIN_LOW);	/* red LED: charge indicator */

	OS_MutexCreate(&i2c_lock);
	OS_MutexCreate(&core_lock);
	OS_MutexCreate(&log_lock);
	OS_SemaphoreCreate(&mcu_irq_sem, 0, 8);

	if (HAL_UART_Init(T30_UART, &up) != HAL_OK)
		printf("T30 uart init failed\n");
	if (HAL_I2C_Init(MCU_I2C, &ip) != HAL_OK)
		printf("MCU i2c init failed\n");
	mcu_irq_enable();

	hal_ext_fill(&hal);
	/* wake source 4 = what the stock firmware reported in the sim tests */
	OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
	bridge_init(&hal, BR_WAKE_COLD);
	OS_MutexUnlock(&core_lock);
	/* MCU and T30 threads first: if WiFi or the SPI slave hang on this
	 * hardware, the MCU still gets its answers and the T30 keeps power */
	/* 4 KiB: the service extensions (status record, update path in
	 * fw_update.c) go deeper than the 2 KiB the plain bridge got by with */
	OS_ThreadCreate(&mcu_thread, "mcu", mcu_task, NULL,
			OS_THREAD_PRIO_APP, 4 * 1024);
	if (OS_SemaphoreCreate(&t30_sem, 0, 1000) != OS_OK ||
	    HAL_UART_EnableRxCallback(T30_UART, t30_rx_cb, NULL) != HAL_OK)
		printf("boot: t30 rx callback failed\n");
	OS_ThreadCreate(&t30_thread, "t30", t30_task, NULL,
			OS_THREAD_PRIO_APP, 4 * 1024);
#ifdef BR_WIFI
	{
		extern uint32_t abus_prcm_dbg[4];
		extern uint32_t abus_efuse_early;

		extern uint32_t abus_efuse_cached;

		printf("boot: efuse cache %lu words\n", (unsigned long)abus_efuse_cached);
		printf("boot: efuse read in SystemInit: %s, status %lu value 0x%02lx\n",
		       abus_efuse_early == 0xdeadbeef ? "NOT REACHED" : "survived",
		       (unsigned long)(abus_efuse_early >> 8) & 0xff,
		       (unsigned long)abus_efuse_early & 0xff);
		printf("boot: prcm dig_ldo 0x%08lx->0x%08lx dcdc 0x%08lx->0x%08lx"
		       " efuse shadow[0] 0x%02x [9] 0x%02x chipver 0x%x\n",
		       (unsigned long)abus_prcm_dbg[0], (unsigned long)abus_prcm_dbg[2],
		       (unsigned long)abus_prcm_dbg[1], (unsigned long)abus_prcm_dbg[3],
		       *(volatile uint8_t *)0x40043D00, *(volatile uint8_t *)0x40043D09,
		       (unsigned)HAL_GlobalGetChipVer());
	}
	boot_mark("mcu/t30 up");
	svc_window();	/* before platform_init: works even if that dies */
	wdg_arm();
	boot_mark("platform init");
	platform_init();	/* sysinfo, WLAN + lwIP netif (no DHCP, see wifi.c) */
	printf("\n\nabus-xr871 fw (wifi) " HAL_EXT_BUILD " " __DATE__ " " __TIME__ "\n");
	OS_MutexLock(&core_lock, OS_WAIT_FOREVER);
	net_ready = 1;
	if (pend_ssid_len >= 0) {
		boot_mark("applying WiFi config received during init");
		wifi_config(pend_ssid, pend_ssid_len, pend_psk, pend_psk_len);
		memset(pend_psk, 0, sizeof(pend_psk));
	}
	OS_MutexUnlock(&core_lock);
#endif

#ifdef BR_WIFI
	{
		struct sysinfo *si = sysinfo_get();

		printf("boot: wlan mac %02x:%02x:%02x:%02x:%02x:%02x\n",
		       si->mac_addr[0], si->mac_addr[1], si->mac_addr[2],
		       si->mac_addr[3], si->mac_addr[4], si->mac_addr[5]);
	}
	boot_mark("wifi start");
	if (wifi_start(&core_lock) != 0)
		printf("wifi start failed\n");
	boot_mark("spi slave start");
	if (spis_start(&core_lock) != 0)
		printf("spi slave start failed\n");
	boot_mark("init done");
	wdg_handover();
	rt_console_start();
#endif

	for (;;)
		OS_MSleep(10000);
	return 0;
}
