/*
 * XR871 glue for the service extensions of the bridge core:
 *
 *   battery   the last AXP2601 reading taken by main.c (bat_report, every
 *             30 s, I2C1 0x62 with the stock battery model). The status
 *             command only reports that copy: no second fuel gauge driver
 *             and no I2C traffic under the core lock.
 *   GPIO      test outputs on port A; the core already refuses the
 *             protected pins, this refuses them again and also the SPI
 *             ready pin in use and the charge LED (PA10, main.c drives it
 *             every second).
 *   log       main.c's h_log() queues every core log line with
 *             bridge_log_capture(); the lock for that queue lives here.
 *   flash     HAL_Flash_* for the application update. The driver state is
 *             shared with main.c (hal_ext_flash_init); in the keep-alive
 *             build (no BR_WIFI) it is only initialised on the first
 *             update request, never at boot. Sector 0 (bootloader) is
 *             refused here as well as in fw_update.c.
 *   reboot    watchdog reset after a committed update (T30 request)
 */
#include <string.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_wdg.h"
#include "core/bridge_core.h"
#ifdef BR_WIFI
#include "spi_slave.h"
#endif
#include "hal_ext.h"

#define IMG_FLASH	0	/* = PRJCONF_IMG_FLASH */
#define BOOT_AREA_END	0x8000U	/* bootloader: never written at runtime */
#define LED_PIN		10	/* blue charge LED, see main.c led_update() */
#define BAT_MAX_AGE_MS	120000U

#ifdef BR_WIFI
#define EXT_VERSION HAL_EXT_BUILD " m2 " __DATE__ " " __TIME__
#else
#define EXT_VERSION HAL_EXT_BUILD " m1 " __DATE__ " " __TIME__
#endif

static OS_Mutex_t log_lock;
static int flash_ready;
static volatile int bat_pct = -1, bat_mv = -1;
static volatile uint32_t bat_ms;

static void ext_logq_lock(int lock)
{
	if (lock)
		OS_MutexLock(&log_lock, OS_WAIT_FOREVER);
	else
		OS_MutexUnlock(&log_lock);
}

void hal_ext_battery_set(int percent, int mv)
{
	bat_pct = percent;
	bat_mv = mv;
	bat_ms = OS_TicksToMSecs(OS_GetTicks());
}

static int ext_battery(int *pct, int *mv)
{
	if (bat_pct < 0 || OS_TicksToMSecs(OS_GetTicks()) - bat_ms > BAT_MAX_AGE_MS)
		return -1;	/* no (recent) reading */
	*pct = bat_pct;
	*mv = bat_mv;
	return 0;
}

static int ext_gpio(int port, int pin, int mode)
{
	GPIO_InitParam p = { GPIOx_Pn_F1_OUTPUT, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE };

	if (port != 0 || bridge_gpio_protected(port, pin) || pin == LED_PIN)
		return -1;
#ifdef BR_WIFI
	if (pin == spis_ready_pin())
		return -1;
#endif
	switch (mode) {
	case BR_GPIO_LOW:
	case BR_GPIO_HIGH:
		HAL_GPIO_Init(GPIO_PORT_A, (GPIO_Pin)pin, &p);
		HAL_GPIO_WritePin(GPIO_PORT_A, (GPIO_Pin)pin,
				  mode == BR_GPIO_HIGH ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
		return 0;
	case BR_GPIO_READ:
		p.mode = GPIOx_Pn_F0_INPUT;
		HAL_GPIO_Init(GPIO_PORT_A, (GPIO_Pin)pin, &p);
		return HAL_GPIO_ReadPin(GPIO_PORT_A, (GPIO_Pin)pin) == GPIO_PIN_HIGH;
	case BR_GPIO_RELEASE:
		if (pin == 14 || pin == 15) {	/* main.c holds these low */
			HAL_GPIO_Init(GPIO_PORT_A, (GPIO_Pin)pin, &p);
			HAL_GPIO_WritePin(GPIO_PORT_A, (GPIO_Pin)pin, GPIO_PIN_LOW);
		} else {
			HAL_GPIO_DeInit(GPIO_PORT_A, (GPIO_Pin)pin);
		}
		return 0;
	}
	return -1;
}

/* ---- flash (update only) ---- */

int hal_ext_flash_init(void)
{
	if (!flash_ready) {
		if (HAL_Flash_Init(IMG_FLASH) != HAL_OK)
			return -1;
		flash_ready = 1;
	}
	return 0;
}

static int flash_open(void)
{
	if (hal_ext_flash_init())
		return -1;
	return HAL_Flash_Open(IMG_FLASH, 5000) == HAL_OK ? 0 : -1;
}

static int ext_flash_read(uint32_t a, void *b, uint32_t n)
{
	HAL_Status r;

	if (flash_open())
		return -1;
	r = HAL_Flash_Read(IMG_FLASH, a, b, n);
	HAL_Flash_Close(IMG_FLASH);
	return r == HAL_OK ? 0 : -1;
}

static int ext_flash_erase4k(uint32_t a)
{
	HAL_Status r;

	if (a < BOOT_AREA_END || flash_open())
		return -1;
	r = HAL_Flash_Erase(IMG_FLASH, FLASH_ERASE_4KB, a, 1);
	HAL_Flash_Close(IMG_FLASH);
	return r == HAL_OK ? 0 : -1;
}

static int ext_flash_write(uint32_t a, const void *b, uint32_t n)
{
	HAL_Status r;

	if (a < BOOT_AREA_END || flash_open())
		return -1;
	r = HAL_Flash_Write(IMG_FLASH, a, b, n);
	HAL_Flash_Close(IMG_FLASH);
	return r == HAL_OK ? 0 : -1;
}

/* compare with the running image's code (SRAM, .text only) */
static int ext_ram_compare(uint32_t addr, const void *buf, uint32_t len)
{
	extern uint8_t __text_start__[], __text_end__[];

	if (addr < (uint32_t)__text_start__ || addr + len > (uint32_t)__text_end__)
		return -1;
	return memcmp((const void *)addr, buf, len) ? 1 : 0;
}

static void ext_reboot(void)
{
	OS_MSleep(200);		/* let the reply leave the UART */
	HAL_WDG_Reboot();
}

void hal_ext_fill(struct bridge_hal *hal)
{
	OS_MutexCreate(&log_lock);
	hal->version = EXT_VERSION;
	hal->battery = ext_battery;
	hal->gpio_test = ext_gpio;
	hal->logq_lock = ext_logq_lock;
	hal->flash_read = ext_flash_read;
	hal->flash_erase4k = ext_flash_erase4k;
	hal->flash_write = ext_flash_write;
	hal->ram_compare = ext_ram_compare;
	hal->reboot = ext_reboot;
}
