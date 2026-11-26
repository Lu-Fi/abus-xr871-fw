/*
 * SPI1 slave for the T30 data link (milestone 2), register level.
 *
 * The public SDK HAL is master-only, so this follows the stock firmware's
 * own slave init (CMU_SPIInterfaceInit, see NOTES.md):
 *   SPI1 @ 0x40006000, module clock 48 MHz, pins PA19..PA22 function 5
 *   (MOSI, MISO, CLK, CS0), CTRL.MODE = slave, TCTRL CPOL 0 / CPHA 1
 *   (SPI mode 1), full duplex, CS owned by the controller,
 *   DMA RX from RXD (0x300) / TX to TXD (0x200), FCTL |= TF_DRQ | RF_DRQ.
 * One transfer = SPIF_SIZE bytes. The TX frame is armed in DMA before the
 * ready line goes high; the T30 clocks the next frame only when it sees it.
 *
 * NOT verified on hardware. The ready pin is a hypothesis (PA15) until the
 * T30 side's probe (cmd 0xc4/0xc5) has confirmed or replaced it.
 */
#include <stdio.h>
#include <string.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_clock.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_spi.h"
#include "core/bridge_core.h"
#include "core/spi_frame.h"
#include "spi_slave.h"

#define SPIS		((SPI_T *)SPI1_BASE)
#define SPIS_MCLK	48000000U
#define ABORT_MS	2000	/* partial frame (CS dropped mid-way): re-arm */

static uint8_t tx_buf[SPIF_SIZE] __attribute__((aligned(4)));
static uint8_t rx_buf[SPIF_SIZE] __attribute__((aligned(4)));
static DMA_Channel dma_tx = DMA_CHANNEL_INVALID, dma_rx = DMA_CHANNEL_INVALID;
static OS_Semaphore_t rx_done;
static OS_Thread_t spi_thread;
static OS_Mutex_t *core_lock;
static volatile int ready_pin = 15;
static volatile int ready_suspended = -1;	/* pin under probe, or -1 */
static uint32_t n_xfer, n_abort;

/* per-stage timing with the Cortex-M4 cycle counter (svc4 diag) */
#define DWT_CTRL	(*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT	(*(volatile uint32_t *)0xE0001004)
#define DEMCR		(*(volatile uint32_t *)0xE000EDFC)
static uint32_t cyc_wait, cyc_rx, cyc_next, cyc_arm, cyc_n;

/* microseconds from the 1 ms OS tick plus the SysTick down-counter (the
 * DWT cycle counter does not run on this chip) */
static uint32_t cyc(void)
{
	uint32_t t, v, load = SysTick->LOAD + 1;

	do {
		t = OS_GetTicks();
		v = SysTick->VAL;
	} while (t != OS_GetTicks());
	return t * 1000 + (load - v) * 1000 / load;
}

/* Only PA15 for the first hardware rounds: stock drives PA15 as the ready
 * line (0x1423a: PA15=1/PA14=0); PA14 is never high in stock and PA6/10/11/
 * 13 have unknown roles (T30 power/reset?), so the probe must not touch them
 * until confirmed. Was { 6, 10, 11, 13, 14, 15 }. */
static const uint8_t allowed_pins[] = { 15 };

int spis_pin_allowed(int pin)
{
	unsigned i;

	for (i = 0; i < sizeof(allowed_pins); i++)
		if (allowed_pins[i] == pin)
			return 1;
	return 0;
}

static void pin_out(int pin, int level)
{
	GPIO_InitParam p = { GPIOx_Pn_F1_OUTPUT, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE };

	HAL_GPIO_Init(GPIO_PORT_A, (GPIO_Pin)pin, &p);
	HAL_GPIO_WritePin(GPIO_PORT_A, (GPIO_Pin)pin, level ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

static volatile int ready_level;	/* what the SPI thread wants on the line */

static void ready(int level)
{
	int pin = ready_pin;

	ready_level = level;
	if (pin >= 0 && ready_suspended != pin)
		HAL_GPIO_WritePin(GPIO_PORT_A, (GPIO_Pin)pin, level ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
}

/* ---- probe / ready pin selection (called from the core, any thread) -- */

int spis_probe_pin(int pin, int level)
{
	if (!spis_pin_allowed(pin))
		return -1;
	if (level == 2) {
		if (ready_suspended == pin)
			ready_suspended = -1;
		/* the ready pin itself goes back to the SPI thread's level, or a
		 * frame armed meanwhile would never be announced (deadlock) */
		pin_out(pin, pin == ready_pin ? ready_level : 0);
		return 0;
	}
	ready_suspended = pin;
	pin_out(pin, level);
	return 0;
}

int spis_ready_pin(void)
{
	return ready_pin;
}

int spis_set_ready_pin(int pin)
{
	if (!spis_pin_allowed(pin))
		return -1;
	if (ready_pin >= 0 && ready_pin != pin)
		pin_out(ready_pin, 0);
	/* keep the current state: xr871d sends 0xc5 on every (re)start, also
	 * while a frame is armed with the line high. Forcing it low here hung
	 * the link for good (T30 waits for high, XR871 waits for a transfer). */
	pin_out(pin, ready_level);
	ready_pin = pin;
	return 0;
}

/* ---- controller -------------------------------------------------------- */

static void dma_end_rx(void *arg)
{
	(void)arg;
	OS_SemaphoreRelease(&rx_done);
}

static void dma_end_tx(void *arg)
{
	(void)arg;
}

static void fifo_reset(void)
{
	uint32_t n = 1000000;

	HAL_SET_BIT(SPIS->FCTL, SPI_FCTL_TF_RST_MASK | SPI_FCTL_RF_RST_MASK);
	while (HAL_GET_BIT(SPIS->FCTL, SPI_FCTL_TF_RST_MASK | SPI_FCTL_RF_RST_MASK) && --n)
		;
	if (!n)
		printf("spis: fifo reset timeout, FCTL 0x%08lx\n", (unsigned long)SPIS->FCTL);
}

static void clock_init(void)
{
	uint32_t src_hz, div;
	CCM_AHBPeriphClkSrc src;

	if (SPIS_MCLK > HAL_GetHFClock()) {
		src_hz = HAL_GetDevClock();
		src = CCM_AHB_PERIPH_CLK_SRC_DEVCLK;
	} else {
		src_hz = HAL_GetHFClock();
		src = CCM_AHB_PERIPH_CLK_SRC_HFCLK;
	}
	div = (src_hz + SPIS_MCLK - 1) / SPIS_MCLK;
	if (div < 1)
		div = 1;
	if (div > 16)
		div = 16;
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_SPI1);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_SPI1);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_SPI1);
	HAL_CCM_SPI1_SetMClock(src, CCM_PERIPH_CLK_DIV_N_1, (CCM_PeriphClkDivM)(div - 1));
	HAL_CCM_SPI1_EnableMClock();
}

static void pins_init(void)
{
	static const struct { GPIO_Pin pin; uint8_t mode; GPIO_PullType pull; } p[] = {
		{ GPIO_PIN_19, GPIOA_P19_F5_SPI1_MOSI, GPIO_PULL_NONE },
		{ GPIO_PIN_20, GPIOA_P20_F5_SPI1_MISO, GPIO_PULL_NONE },
		{ GPIO_PIN_21, GPIOA_P21_F5_SPI1_CLK, GPIO_PULL_NONE },
		{ GPIO_PIN_22, GPIOA_P22_F5_SPI1_CS0, GPIO_PULL_UP },
	};
	unsigned i;

	for (i = 0; i < sizeof(p) / sizeof(p[0]); i++) {
		GPIO_InitParam g = { p[i].mode, GPIO_DRIVING_LEVEL_1, p[i].pull };
		HAL_GPIO_Init(GPIO_PORT_A, p[i].pin, &g);
	}
}

/* T30 off: no XR871 pin may drive (or pull up) a line into the unpowered
 * T30, or it is back-powered through its protection diodes and does not
 * reset on the next power-on (2026-11-26 camera 3: the ready line PA15 was
 * high in standby; a T30 re-powered within ~6 min of a standby never
 * booted). park = 1: SPI pins and the ready line to inputs with pull-down,
 * park = 0: SPI function back, ready line at the SPI thread's level. */
void spis_park(int park)
{
	static const GPIO_Pin sp[] = { GPIO_PIN_19, GPIO_PIN_20, GPIO_PIN_21, GPIO_PIN_22 };
	GPIO_InitParam in = { GPIOx_Pn_F0_INPUT, GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN };
	int pin = ready_pin;
	unsigned i;

	if (park) {
		for (i = 0; i < sizeof(sp) / sizeof(sp[0]); i++)
			HAL_GPIO_Init(GPIO_PORT_A, sp[i], &in);
		if (pin >= 0) {
			ready_suspended = pin;
			HAL_GPIO_Init(GPIO_PORT_A, (GPIO_Pin)pin, &in);
		}
		return;
	}
	pins_init();
	if (pin >= 0) {
		if (ready_suspended == pin)
			ready_suspended = -1;
		pin_out(pin, ready_level);
	}
}

static int dma_init(void)
{
	DMA_ChannelInitParam t, r;

	dma_tx = HAL_DMA_Request();
	dma_rx = HAL_DMA_Request();
	if (dma_tx == DMA_CHANNEL_INVALID || dma_rx == DMA_CHANNEL_INVALID)
		return -1;
	memset(&t, 0, sizeof(t));
	memset(&r, 0, sizeof(r));
	/* same channel setup as the SDK master driver, for SPI1 */
	/* 8 bit / burst 1 on both sides, exactly like the stock firmware
	 * (TX cfg 0x04228000, RX cfg 0x04008022): 32-bit FIFO accesses with
	 * the default 1-byte RX trigger read the FIFO empty/twice */
	t.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE, DMA_WAIT_CYCLE_2,
					   DMA_BYTE_CNT_MODE_REMAIN,
					   DMA_DATA_WIDTH_8BIT, DMA_BURST_LEN_1,
					   DMA_ADDR_MODE_FIXED, DMA_PERIPH_SPI1,
					   DMA_DATA_WIDTH_8BIT, DMA_BURST_LEN_1,
					   DMA_ADDR_MODE_INC, DMA_PERIPH_SRAM);
	t.irqType = DMA_IRQ_TYPE_END;
	t.endCallback = dma_end_tx;
	r.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE, DMA_WAIT_CYCLE_2,
					   DMA_BYTE_CNT_MODE_REMAIN,
					   DMA_DATA_WIDTH_8BIT, DMA_BURST_LEN_1,
					   DMA_ADDR_MODE_INC, DMA_PERIPH_SRAM,
					   DMA_DATA_WIDTH_8BIT, DMA_BURST_LEN_1,
					   DMA_ADDR_MODE_FIXED, DMA_PERIPH_SPI1);
	r.irqType = DMA_IRQ_TYPE_END;
	r.endCallback = dma_end_rx;
	HAL_DMA_Init(dma_rx, &r);
	HAL_DMA_Init(dma_tx, &t);
	return 0;
}

static void ctrl_init(void)
{
	uint32_t n = 1000000;

	HAL_SET_BIT(SPIS->CTRL, SPI_CTRL_RST_MASK);
	while (HAL_GET_BIT(SPIS->CTRL, SPI_CTRL_RST_MASK) && --n)
		;
	if (!n)
		printf("spis: ctrl reset timeout, CTRL 0x%08lx\n", (unsigned long)SPIS->CTRL);
	HAL_MODIFY_REG(SPIS->CTRL, SPI_CTRL_MODE_MASK, SPI_CTRL_MODE_SLAVE);
	HAL_MODIFY_REG(SPIS->TCTRL, SPI_TCTRL_CPOL_MASK | SPI_TCTRL_CPHA_MASK, SPI_SCLK_Mode1);
	HAL_MODIFY_REG(SPIS->TCTRL, SPI_TCTRL_FBS_MASK, SPI_TCTRL_FBS_MSB);
	HAL_MODIFY_REG(SPIS->TCTRL, SPI_TCTRL_DHB_MASK, SPI_TCTRL_DHB_FULL_DUPLEX);
	HAL_CLR_BIT(SPIS->TCTRL, SPI_TCTRL_SS_OWNER_MASK);	/* CS from the pin */
	HAL_MODIFY_REG(SPIS->TCTRL, SPI_TCTRL_SS_SEL_MASK, SPI_TCTRL_SS_SEL_SS0);
	fifo_reset();
	HAL_SET_BIT(SPIS->CTRL, SPI_CTRL_EN_MASK);
}

/* stock sequence (FUN_0001c02c): burst counters = frame length, TX+RX FIFO
 * reset, TCTRL |= SS_OWNER (0x40), DMA TX+RX, FCTL |= DRQs, TCTRL |= XCH */
static void arm(void)
{
	HAL_CLR_BIT(SPIS->FCTL, SPI_FCTL_TF_DRQ_EN_MASK | SPI_FCTL_RF_DRQ_EN_MASK);
	SPIS->BC = (SPIS->BC & 0xff000000U) | SPIF_SIZE;
	SPIS->TC = (SPIS->TC & 0xff000000U) | SPIF_SIZE;
	SPIS->BCC = (SPIS->BCC & 0xff000000U) | SPIF_SIZE;
	fifo_reset();
	HAL_SET_BIT(SPIS->TCTRL, SPI_TCTRL_SS_OWNER_MASK);
	HAL_DMA_Start(dma_tx, (uint32_t)tx_buf, (uint32_t)&SPIS->TXD, SPIF_SIZE);
	HAL_DMA_Start(dma_rx, (uint32_t)&SPIS->RXD, (uint32_t)rx_buf, SPIF_SIZE);
	HAL_SET_BIT(SPIS->FCTL, SPI_FCTL_TF_DRQ_EN_MASK | SPI_FCTL_RF_DRQ_EN_MASK);
	HAL_SET_BIT(SPIS->TCTRL, SPI_TCTRL_XCH_MASK);
	/* first bytes must sit in the TX FIFO before the master may clock */
	{
		int n = 100000;

		while (((SPIS->FST & SPI_FST_TF_CNT_MASK) >> SPI_FST_TF_CNT_SHIFT) == 0 && --n)
			;
	}
}

static void disarm(void)
{
	HAL_CLR_BIT(SPIS->FCTL, SPI_FCTL_TF_DRQ_EN_MASK | SPI_FCTL_RF_DRQ_EN_MASK);
	HAL_DMA_Stop(dma_rx);
	HAL_DMA_Stop(dma_tx);
}

static void spi_task(void *arg)
{
	(void)arg;
	for (;;) {
		uint32_t waited = 0, t0, t1, t2;

		t0 = cyc();
		OS_MutexLock(core_lock, OS_WAIT_FOREVER);
		bridge_spi_next(tx_buf);
		OS_MutexUnlock(core_lock);
		t1 = cyc();
		arm();
		ready(1);
		t2 = cyc();
		cyc_next += t1 - t0;
		cyc_arm += t2 - t1;
		while (OS_SemaphoreWait(&rx_done, 500) != OS_OK) {
			waited += 500;
			/* nothing clocked: fine, keep the frame armed. Part of a
			 * frame clocked and then nothing: the master gave up. */
			if (waited >= ABORT_MS &&
			    HAL_DMA_GetByteCount(dma_rx) != SPIF_SIZE &&
			    HAL_DMA_GetByteCount(dma_rx) != 0)
				break;
		}
		ready(0);
		disarm();
		if (waited >= ABORT_MS) {
			n_abort++;
			continue;
		}
		n_xfer++;
		t0 = cyc();
		cyc_wait += t0 - t2;
		OS_MutexLock(core_lock, OS_WAIT_FOREVER);
		bridge_spi_rx(rx_buf, SPIF_SIZE);
		OS_MutexUnlock(core_lock);
		cyc_rx += cyc() - t0;
		cyc_n++;
	}
}

int spis_start(OS_Mutex_t *lock)
{
	core_lock = lock;
	if (OS_SemaphoreCreate(&rx_done, 0, 1) != OS_OK)
		return -1;
	pin_out(ready_pin, 0);
	pins_init();
	clock_init();
	if (dma_init() < 0)
		return -1;
	ctrl_init();
	return OS_ThreadCreate(&spi_thread, "spis", spi_task, NULL,
			       OS_THREAD_PRIO_APP, 2 * 1024) == OS_OK ? 0 : -1;
}

void spis_stats(uint32_t *xfer, uint32_t *aborted)
{
	*xfer = n_xfer;
	*aborted = n_abort;
}

/* svc4: one line of SPI link diagnostics (runtime console "spi" and the
 * periodic status) */
void spis_diag(void)
{
	uint32_t pins = 0;
	int i;

	for (i = 19; i <= 22; i++)
		pins |= (HAL_GPIO_ReadPin(GPIO_PORT_A, (GPIO_Pin)i) == GPIO_PIN_HIGH) << (i - 19);
	{
		uint32_t n = cyc_n ? cyc_n : 1, mhz = 1;	/* cyc() is in us */

		printf("spis: per xfer us: wait %lu rx %lu next %lu arm %lu (n %lu)\n",
		       (unsigned long)(cyc_wait / n / mhz), (unsigned long)(cyc_rx / n / mhz),
		       (unsigned long)(cyc_next / n / mhz), (unsigned long)(cyc_arm / n / mhz),
		       (unsigned long)cyc_n);
		cyc_wait = cyc_rx = cyc_next = cyc_arm = cyc_n = 0;
	}
	{
		extern void wifi_stats(uint32_t *rx, uint32_t *tx, uint32_t *tx_err);
		uint32_t wrx, wtx, werr;

		wifi_stats(&wrx, &wtx, &werr);
		extern int wifi_link_info(int *rssi, int *level, int *chan);
		int rssi = 0, level = 0, chan = 0;

		wifi_link_info(&rssi, &level, &chan);
		extern uint32_t wifi_rx_busy(void);
		extern uint32_t t30_rx_overruns(void);

		printf("wifi: air->t30 %lu  t30->air %lu  tx_err %lu  rx_busy %lu  t30 rx_ovr %lu  rssi %d level %d ch %d\n",
		       (unsigned long)wrx, (unsigned long)wtx, (unsigned long)werr,
		       (unsigned long)wifi_rx_busy(), (unsigned long)t30_rx_overruns(),
		       rssi, level, chan);
	}
	printf("spis: xfer %lu abort %lu dma rx left %lu tx left %lu | CTRL %08lx TCTRL %08lx"
	       " STA %08lx FCTL %08lx FST %08lx | ready PA%d=%d pins(mosi,miso,clk,cs) %lx\n",
	       (unsigned long)n_xfer, (unsigned long)n_abort,
	       (unsigned long)(dma_rx != DMA_CHANNEL_INVALID ? HAL_DMA_GetByteCount(dma_rx) : 0),
	       (unsigned long)(dma_tx != DMA_CHANNEL_INVALID ? HAL_DMA_GetByteCount(dma_tx) : 0),
	       (unsigned long)SPIS->CTRL, (unsigned long)SPIS->TCTRL,
	       (unsigned long)SPIS->STA, (unsigned long)SPIS->FCTL, (unsigned long)SPIS->FST,
	       ready_pin, ready_pin >= 0 ?
	       HAL_GPIO_ReadPin(GPIO_PORT_A, (GPIO_Pin)ready_pin) == GPIO_PIN_HIGH : -1,
	       (unsigned long)pins);
}
