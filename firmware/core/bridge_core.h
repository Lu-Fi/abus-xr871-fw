/*
 * Portable protocol core of the ABUS PPIC90520 XR871 firmware.
 *
 * Talks to the T30 (UART frames, see NOTES.md) and to the ST power MCU
 * (2-byte I2C messages). No SDK dependency: the same file is built for the
 * XR871 (hal_xr871.c) and for the host emulation (host/hal_host.c), where
 * it is wired to the T30 QEMU and a simulated MCU.
 */
#ifndef BRIDGE_CORE_H
#define BRIDGE_CORE_H

#include <stdint.h>

struct bridge_hal {
	void (*t30_write)(const uint8_t *buf, int len);	/* UART to T30 */
	int (*mcu_send)(uint8_t cmd, uint8_t val);	/* I2C to MCU, 0 = ok */
	void (*t30_power)(int on);			/* PA16 */
	uint32_t (*millis)(void);
	void (*log)(const char *fmt, ...);
	void (*wifi_tx)(const uint8_t *frame, int len);	/* Ethernet frame out */
	/* station MAC + link state (1 = associated); NULL = no WiFi */
	int (*wifi_status)(uint8_t *mac, int8_t *rssi);
	/* T30 is off and nothing is pending: the glue may enter low power
	 * (XR871 standby with WiFi kept); 0 = back to full power */
	void (*low_power)(int enter);
	/* store + apply STA credentials; returns 0 unchanged, 1 changed, <0 error */
	int (*wifi_config)(const uint8_t *ssid, int ssid_len, const uint8_t *psk, int psk_len);
	/* ready-line discovery: drive candidate pin PAn to level 0/1, 2 = give it
	 * back to its normal function; <0 = not allowed */
	int (*probe_pin)(int pin, int level);
	/* use PAn as the SPI ready line from now on; <0 = not allowed */
	int (*set_ready_pin)(int pin);
	/* WiFi role / scan, runs asynchronously in the glue: op BR_WIFI_OP_*;
	 * the glue reports the result with bridge_t30_send() (under the core
	 * lock). Returns 0 = accepted, <0 = not supported / busy. */
	int (*wifi_mode)(int op, const uint8_t *ssid, int ssid_len,
			 const uint8_t *psk, int psk_len);

	/* ---- optional extensions (NULL = not available) ------------------
	 * Appended so the positional initialisers of older glue still work. */
	const char *version;			/* build string of the glue */
	/* battery: percent 0..100 and mV; <0 = unknown. Read-only. */
	int (*battery)(int *percent, int *mv);
	/* GPIO test: port 0 = A, 1 = B; mode BR_GPIO_*; returns the level for
	 * BR_GPIO_READ, 0 for the others, <0 = refused / failed. The core has
	 * already refused the protected pins; the glue may refuse more. */
	int (*gpio_test)(int port, int pin, int mode);
	/* serialise bridge_log_capture() against the core draining the log
	 * queue (multi-threaded glue); lock = 1 take, 0 release */
	void (*logq_lock)(int lock);
	/* raw flash access for the application update (fw_update.c), 0 = ok.
	 * Only called on explicit T30 requests, never at boot. */
	int (*flash_read)(uint32_t addr, void *buf, uint32_t len);
	int (*flash_erase4k)(uint32_t addr);
	int (*flash_write)(uint32_t addr, const void *buf, uint32_t len);
	/* compare buf with the running application in RAM at addr:
	 * 0 = equal, 1 = different, <0 = cannot tell */
	int (*ram_compare)(uint32_t addr, const void *buf, uint32_t len);
	void (*reboot)(void);			/* reset the XR871 (and so the T30) */
};

#define BR_GPIO_LOW	0
#define BR_GPIO_HIGH	1
#define BR_GPIO_READ	2	/* switch to input, return the level */
#define BR_GPIO_BLINK	3	/* core only: toggles via LOW/HIGH */
#define BR_GPIO_RELEASE	4	/* back to what the firmware does with it */

#define BR_WIFI_OP_STA 0
#define BR_WIFI_OP_AP 1
#define BR_WIFI_OP_SCAN 2

/* frame layout shared with the T30 side (xr871d) */
#define BR_MAGIC0 0xaa
#define BR_MAGIC1 0x55
#define BR_HDR_LEN 12
#define BR_MAX_PAYLOAD 0xf4
#define BR_PROBE_MS 300	/* a probed pin falls back to low after this */

/* T30 -> XR871 */
#define T30_HELLO 0x00
#define T30_POWER 0x83
#define T30_HEARTBEAT 0xa9
#define T30_MARKER 0xf1
#define T30_WIFI_CFG 0xc0	/* u8 ssid_len, ssid[32], u8 psk_len, psk[64] */
#define T30_PROBE_PIN 0xc4	/* param = pin | level << 8 */
#define T30_READY_PIN 0xc5	/* param = pin */
/* WiFi setup (portal): same payload as 0xc0, psk_len 0 = open AP.
 * Ack: same cmd, param 0 = accepted / 0xffff = refused; when done the
 * XR871 sends XR_WIFI_DONE. */
#define T30_WIFI_AP 0xc6
#define T30_WIFI_STA 0xc7	/* back to station mode (stored credentials) */
#define T30_WIFI_SCAN 0xc8	/* scan (station mode only) */

/*
 * Extensions (T30 -> XR871 requests, answered with the same command; only
 * xr871d sends them, and only after our 0xf1 marker was answered):
 *   0xd0 status        reply payload struct below (BR_STATUS_*)
 *   0xd1 MCU message   param = cmd | val << 8, payload u8 flags (bit0 force)
 *                      reply param 0 ok, BR_E_DENIED, BR_E_FAIL
 *   0xd3 GPIO test     payload u8 port (0 A, 1 B), u8 pin, u8 mode
 *                      (BR_GPIO_*), u16 ms (hold / blink time, 0 = 30 s),
 *                      u16 blink half period ms (0 = 250)
 *                      reply param = level (READ) / 0, BR_E_DENIED, BR_E_FAIL
 *   0xd4 forwarding    param bit0 = XR871 log lines, bit1 = MCU events
 *   0xd8..0xdc         application update, see fw_update.h
 * XR871 -> T30, only after the T30 enabled them with 0xd4 (the stock T30
 * application never does, so it never sees these frames):
 *   0xd2 MCU event     param = cmd | val << 8, payload u8 dir (0 = from MCU)
 *   0xd4 log line      param = sequence, payload = text (no newline)
 */
#define T30_STATUS 0xd0
#define T30_MCU 0xd1
#define XR_MCU_EVENT 0xd2
#define T30_GPIO 0xd3
#define T30_FWD 0xd4
#define XR_LOG 0xd4
#define BR_FWD_LOG 0x01
#define BR_FWD_MCU 0x02
#define BR_E_DENIED 0xfffe
#define BR_E_FAIL 0xffff
#define BR_STATUS_VERSION 1
#define BR_STATUS_LEN 152
#define BR_LOG_MAX 200		/* longest forwarded line */

/* XR871 -> T30 */
#define XR_WAKE_SRC 0x85	/* payload u32: BR_WAKE_* */
#define XR_POWER 0x83		/* sleep request acknowledged, param = grace ms */
/* 0xc0 / 0xc4 / 0xc5 are acked with the same command, param = result */
#define XR_TIME 0x40
#define XR_INFO 0x80
/* param = BR_WIFI_OP_* | result << 8 (0 ok, 0xff error); for a scan the
 * entries come first as XR_SCAN_ENTRY frames, param = index, payload:
 * bssid[6], s8 rssi, u8 channel, u8 security (0 open, 1 WEP, 2 WPA,
 * 3 WPA2, 4 WPA/WPA2), u8 ssid_len, ssid[ssid_len] */
#define XR_WIFI_DONE 0xca
#define XR_SCAN_ENTRY 0xcb
/* battery (AXP2601 fuel gauge), every 30 s: payload u16 vbat_mv (LE),
 * u8 soc %, s8 temperature deg C, u8 flags (bit0 = gauge was asleep and
 * was woken) */
#define XR_BATTERY 0xcc
#define XR_PIR 0xcd		/* XR871->T30: PIR motion (param = MCU event value) */

/* MCU <-> XR871 */
#define MCU_HEARTBEAT 0x0e
#define MCU_Q_STATE1 0x1e
#define MCU_Q_STATE2 0x1f
#define MCU_Q_VERSION 0x23
#define MCU_DSP_POWER 0x05	/* value 0x07 = on, 0x09 = off */
#define MCU_EV_PIR 0x0d
#define MCU_EV_KEY1 0x12
#define MCU_EV_KEY2 0x13
#define MCU_EV_USB 0x0c		/* val != 0: USB power in (stock "USB INSERT") */
#define MCU_EV_MAINBAT 0x2d	/* main battery in/out */
#define MCU_LOW_POWER 0x24	/* XR871 -> MCU low-power notice */

/*
 * Sleep: the T30 sends T30_POWER with param 0 (optional payload u32 =
 * wake-up timer in seconds, 0 = none). The core acks with XR_POWER, waits
 * BR_SLEEP_GRACE_MS, cuts the T30 rail and tells the MCU. It wakes the T30
 * again on PIR, a key, a Wake-on-LAN magic packet for the T30 MAC from the
 * WiFi side, or the timer, and reports why in XR_WAKE_SRC.
 * 4 is what the stock firmware reports for a cold power-on.
 */
#define BR_WAKE_COLD 0x004
#define BR_WAKE_PIR 0x101
#define BR_WAKE_KEY 0x102
#define BR_WAKE_WLAN 0x103
#define BR_WAKE_TIMER 0x104
#define BR_SLEEP_GRACE_MS 1500

struct bridge_stats {
	uint32_t t30_frames, t30_bad, t30_hb, t30_hello;
	uint32_t t30_last_hb_ms, t30_power_on_ms, t30_first_hb_ms;
	uint32_t mcu_rx, mcu_tx, mcu_tx_err, mcu_hb;
	uint8_t mcu_version;
	int t30_marker_seen;
	uint32_t spi_frames, spi_bad, spi_eth_up, spi_eth_down, wifi_drop;
	uint32_t sleeps, wakes, last_wake;
	int t30_on;
	uint32_t t30_resets;
	int usb;	/* MCU event 0x0c: 1 = USB power in, 0 = out, -1 = unknown */
	uint8_t mcu_state1, mcu_state2, mcu_usb, mcu_mainbat;
	uint8_t mcu_seen;	/* bit0 state1, bit1 state2, bit2 usb, bit3 mainbat */
	uint32_t log_lines, log_dropped, mcu_events_fwd;
};

void bridge_init(const struct bridge_hal *hal, uint32_t wake_source);
/* send one frame to the T30 (caller holds the core lock) */
void bridge_t30_send(uint8_t cmd, uint16_t param, const void *payload, uint16_t len);
void bridge_t30_rx(const uint8_t *buf, int len);
void bridge_mcu_rx(uint8_t cmd, uint8_t val);
void bridge_poll(void);		/* call about every 100 ms */
void bridge_wake(uint32_t reason);	/* power the T30 up (no-op if on) */
/*
 * Power-cycle a running T30 (3 s off). Also done automatically when the T30
 * is on but its network path is dead (no SPI frame for BR_STALL_MS while it
 * should be talking) or its heartbeat is gone; at most once per
 * BR_RESET_HOLDOFF_MS. bridge_t30_auto(0) disables the automatic reset.
 */
#define BR_RESET_OFF_MS 3000
#define BR_STALL_MS 180000
#define BR_RESET_HOLDOFF_MS 600000
void bridge_t30_reset(const char *why);
void bridge_t30_auto(int on);

/* SPI data path (frame format in spi_frame.h) */
void bridge_spi_rx(const uint8_t *frame, int len);	/* frame the T30 clocked in */
int bridge_spi_next(uint8_t *frame);	/* build the frame to arm next; 1 = carries data */
void bridge_wifi_rx(const uint8_t *eth, int len);	/* frame from WiFi for the T30 */
const struct bridge_stats *bridge_stats(void);

/* glue log hook: queue one formatted log line for forwarding to the T30.
 * Never blocks; drops when the queue is full. Lines that might carry
 * credentials are withheld. */
void bridge_log_capture(const char *line, int len);
/* 1 = pin must never be driven (see bridge_core.c) */
int bridge_gpio_protected(int port, int pin);

/*
 * Status reply (0xd0, param = BR_STATUS_VERSION), little endian:
 *   0 u8 version   1 u8 flags (bit0 wifi present, bit1 associated,
 *   bit2 battery valid, bit3 T30 powered)   2 u8 battery %   3 s8 rssi dBm
 *   4 u16 vbat mV  6 u8 MCU sw version      7 u8 update state (fw_update.h)
 *   8 u32 uptime ms   12 u32 wake reason (BR_WAKE_*)
 *  16 u32 x 16 counters: t30 frames, t30 bad, t30 hb, mcu rx, mcu tx,
 *     mcu tx err, spi frames, spi bad, eth up, eth down, wifi drop,
 *     log dropped, sleeps, wakes, log lines, mcu events forwarded
 *  80 u8 MCU state 0x1e   81 u8 state 0x1f   82 u8 last USB event value
 *  83 u8 last main battery event   84 u8 seen bits (1 state1, 2 state2,
 *     4 usb, 8 main battery)   85 u8 running image (0/1, 0xff unknown)
 *  86 u8 OTA enabled (0xff unknown)   87 u8 forwarding mask
 *  88 char[64] version, NUL padded
 */

#endif
