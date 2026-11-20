/*
 * XR871 application update over the T30 UART link (portable, no SDK).
 *
 * The T30 streams a new image chain (everything after the bootloader, i.e.
 * image offsets 0x8000..end) into the INACTIVE image slot. Nothing here
 * runs at boot: the flash is only touched after an explicit request.
 *
 *   0xd8 BEGIN   T30 -> XR  payload struct fwu_begin, reply struct fwu_begin_reply
 *   0xd9 DATA    T30 -> XR  payload u32 off, u32 crc32, data[<=232]
 *                           param bit0 = fill: payload u32 off, u32 len,
 *                           range must read back as 0xff (gaps in the image)
 *                           reply param = status, payload u32 next_off
 *   0xda VERIFY  T30 -> XR  read back the whole span (crc32) and walk the
 *                           AWIH chain in the target slot (header + data
 *                           checksums, same rules as the bootloader)
 *                           reply param = status, payload struct fwu_verify_reply
 *   0xdb COMMIT  T30 -> XR  param = FWU_COMMIT_MAGIC, payload u32 span crc32;
 *                           only after VERIFY ok and with OTA enabled in the
 *                           boot header: appends {target seq, verified} to
 *                           the OTA config (fdcm) - data slot first, bitmap
 *                           bit last, so a power cut keeps the old config
 *   0xdc INFO    T30 -> XR  param 0 = info, 1 = abort, 2 = reboot (only in
 *                           state COMMITTED); reply struct fwu_info_reply
 *
 * Every reply carries the same command, param = FWU_E_* status.
 */
#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include <stdint.h>

#define FWU_MAGIC		0x31555746u	/* "FWU1" */
#define FWU_COMMIT_MAGIC	0xc0ffu
#define FWU_CHUNK_MAX		232
#define FWU_MAX_SECTIONS	16

/* flash map facts (W25Q16, see NOTES.md / README.md) */
#define FWU_FLASH_SIZE		0x200000u
#define FWU_SLOT_LIMIT		0xfb000u	/* image offsets end here: sysinfo 0xfb000 */
#define FWU_SYSINFO_ADDR	0xfb000u
#define FWU_SYSINFO_SIZE	0x1000u
#define FWU_DEVINFO_ADDR	0x1fe000u	/* stock device info, never touched */

#define FWU_CMD_BEGIN	0xd8
#define FWU_CMD_DATA	0xd9
#define FWU_CMD_VERIFY	0xda
#define FWU_CMD_COMMIT	0xdb
#define FWU_CMD_INFO	0xdc

#define FWU_F_FORCE_STAGE	0x01	/* BEGIN: stage although OTA is disabled */
#define FWU_F_RESUME		0x02	/* BEGIN: continue an identical transfer */
#define FWU_DATA_FILL		0x0001	/* DATA param: 0xff range */

enum fwu_err {
	FWU_OK = 0,
	FWU_E_STATE,		/* command not valid in this state */
	FWU_E_ARG,		/* malformed request */
	FWU_E_OTA_DISABLED,	/* boot header has no OTA area: no A/B switch */
	FWU_E_TOO_BIG,		/* span does not fit the slot */
	FWU_E_CRC,		/* chunk crc mismatch */
	FWU_E_OFFSET,		/* out-of-order chunk */
	FWU_E_FLASH,		/* erase/write/read failed */
	FWU_E_READBACK,		/* written data reads back different */
	FWU_E_IMAGE,		/* AWIH chain / checksum / span crc bad */
	FWU_E_NOT_VERIFIED,	/* commit before a good verify */
	FWU_E_CFG,		/* OTA config unreadable / full / write failed */
	FWU_E_RUNNING,		/* cannot tell which slot is running */
	FWU_E_NO_FLASH,		/* no flash access in this build */
	FWU_E_DENIED,		/* address outside the writable window */
	FWU_E_LAYOUT,		/* boot header unusable */
};

enum fwu_state { FWU_IDLE, FWU_RECEIVING, FWU_RECEIVED, FWU_VERIFIED, FWU_COMMITTED };

struct fwu_begin {		/* 24 bytes, little endian */
	uint32_t magic;
	uint32_t span_off;	/* image offset of the first byte, must be bl_size */
	uint32_t span_len;
	uint32_t span_crc;	/* crc32 (zlib) of the span */
	uint16_t chunk_max;
	uint8_t flags;
	uint8_t reserved;
	uint32_t reserved2;
};

/* reply payloads (packed by hand, little endian):
 * BEGIN/INFO: u8 state, u8 running, u8 target, u8 ota_enabled,
 *             u32 target_phys (span start), u32 next_off, u32 span_len,
 *             u32 span_crc, u32 slot_limit_phys
 * VERIFY:     u32 crc, u8 sections, u8 reserved[3], u32 bad_phys */

struct bridge_hal;
void fwu_init(const struct bridge_hal *hal);
/* handle one of the FWU_CMD_* frames; returns the status, fills the reply
 * payload (max 32 bytes) */
uint16_t fwu_cmd(uint8_t cmd, uint16_t param, const uint8_t *p, uint16_t plen,
		 uint8_t *out, uint16_t *olen);
/* for the status reply; 0xff = not probed yet (flash untouched) */
/* 1 once after an accepted reboot request (reply already queued) */
int fwu_take_reboot(void);
void fwu_summary(uint8_t *state, uint8_t *running, uint8_t *ota);
uint32_t fwu_crc32(uint32_t crc, const void *buf, uint32_t len);
uint16_t fwu_sum16(const uint8_t *p, uint32_t len, uint16_t s);

#endif
