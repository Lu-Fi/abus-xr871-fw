/*
 * XR871 application update over the T30 UART link - see fw_update.h.
 *
 * Boot selection (stock bootloader @0 = XR871 SDK bl_load_bin, checked in
 * the disassembly, see README):
 *   - the bootloader's own AWIH header carries the OTA area in priv[0]
 *     (flash | size << 8) and priv[1] (address); priv[1] = 0xffffffff means
 *     OTA disabled -> it always loads image 1 and never looks at image 2
 *   - with OTA: the last fdcm record at the OTA area is {raw_seq, state};
 *     0x5555 = image 1, 0xaaaa = image 2, 0x9669 = verified. Not verified /
 *     unreadable -> image 1. If the APP section of the chosen image fails
 *     its checksum it loads the other one and writes that choice back.
 *   - image n starts at addr[n] = image offset + base[n]; base[0] = 0,
 *     base[1] = ota_addr + ota_size - bl_size (0xfc000 on the stock layout).
 *
 * Writes are confined to the target slot window [base + bl_size,
 * base + FWU_SLOT_LIMIT), which must not overlap the bootloader, the
 * running slot, the OTA area, sysinfo or the stock device info. The OTA
 * area itself is only written by fwu_cfg_append (one data slot + one
 * bitmap byte, no erase).
 */
#include <string.h>
#include "bridge_core.h"
#include "fw_update.h"

#define AWIH_MAGIC	0x48495741u
#define BOOT_ID		0xa5ff5a00u
#define APP_ID		0xa5fe5a01u
#define FDCM_ID		0xa55a5aa5u
#define RAW_SEQ0	0x5555
#define RAW_SEQ1	0xaaaa
#define RAW_VERIFIED	0x9669
#define STOCK_SLOT1_BASE 0xfc000u	/* stock OTA area 0x100000 + 0x4000 - 0x8000 */
#define RAM_CHECK_OFF	0x400	/* compare 1 KiB of code, past the vector table */
#define RAM_CHECK_LEN	0x400

static const struct bridge_hal *H;
static uint8_t fbuf[256];

static struct {
	int probed;		/* 1 = layout known */
	uint32_t bl_size, ota_addr, ota_size;
	int ota;		/* OTA enabled in the boot header */
	uint32_t base[2];
	int running, target;
	uint32_t lo, hi;	/* writable window (target slot) */
	int state;
	uint32_t span_len, span_crc, next, erased_to;
	int reboot;
} F;

static uint32_t le32(const uint8_t *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

static const uint32_t crc_tab[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4,
	0x4db26158, 0x5005713c, 0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
	0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

/* zlib crc32: fwu_crc32(0, buf, len) == zlib.crc32(buf) */
uint32_t fwu_crc32(uint32_t crc, const void *buf, uint32_t len)
{
	const uint8_t *p = buf;

	crc = ~crc;
	while (len--) {
		crc ^= *p++;
		crc = (crc >> 4) ^ crc_tab[crc & 15];
		crc = (crc >> 4) ^ crc_tab[crc & 15];
	}
	return ~crc;
}

/* XR871 image checksum: 16-bit LE word sum, odd tail byte added as is.
 * Only the last piece of a chunked sum may have an odd length. */
uint16_t fwu_sum16(const uint8_t *p, uint32_t len, uint16_t s)
{
	while (len > 1) {
		s += p[0] | p[1] << 8;
		p += 2;
		len -= 2;
	}
	if (len)
		s += p[0];
	return s;
}

void fwu_init(const struct bridge_hal *hal)
{
	H = hal;
	memset(&F, 0, sizeof(F));
	F.running = F.target = -1;
}

static int frd(uint32_t a, void *b, uint32_t n)
{
	if (a >= FWU_FLASH_SIZE || n > FWU_FLASH_SIZE - a)
		return -1;
	return H->flash_read(a, b, n) ? -1 : 0;
}

static int in_window(uint32_t a, uint32_t n)
{
	return a >= F.lo && a < F.hi && n <= F.hi - a;
}

static int ferase(uint32_t a)
{
	if ((a & 0xfff) || !in_window(a, 0x1000))
		return FWU_E_DENIED;
	return H->flash_erase4k(a) ? FWU_E_FLASH : 0;
}

static int fwrite_(uint32_t a, const void *b, uint32_t n)
{
	if (!in_window(a, n))
		return FWU_E_DENIED;
	return H->flash_write(a, b, n) ? FWU_E_FLASH : 0;
}

static int overlaps(uint32_t a0, uint32_t a1, uint32_t b0, uint32_t b1)
{
	return a0 < b1 && b0 < a1;
}

/* ---- OTA config (SDK fdcm, record = u16 raw_seq, u16 raw_state) ------ */

struct fdcm {
	uint32_t data;		/* address of record 0 */
	uint16_t bitmap_size;
	uint32_t count;		/* records written */
};

static int fdcm_open(struct fdcm *c)
{
	uint8_t h[8];
	uint32_t i, cnt = 0;

	if (frd(F.ota_addr, h, 8))
		return -1;
	c->bitmap_size = h[4] | h[5] << 8;
	if (le32(h) != FDCM_ID || (h[6] | h[7] << 8) != 4 || !c->bitmap_size ||
	    c->bitmap_size + 8u >= F.ota_size)
		return -1;
	/* same counting as the SDK: whole 0x00 bytes, then the cleared low
	 * bits of the first non-zero byte */
	for (i = 0; i < c->bitmap_size; i++) {
		uint8_t b, v;

		if (frd(F.ota_addr + 8 + i, &b, 1))
			return -1;
		if (b == 0) {
			cnt += 8;
			continue;
		}
		for (v = ~b; v; v >>= 1)
			cnt++;
		break;
	}
	c->count = cnt;
	c->data = F.ota_addr + 8 + c->bitmap_size;
	return 0;
}

/* current config: seq 0/1, *verified; -1 = unreadable */
static int cfg_read(int *verified)
{
	struct fdcm c;
	uint8_t r[4];
	uint16_t seq, st;

	*verified = 0;
	if (fdcm_open(&c) || c.count == 0 || frd(c.data + 4 * (c.count - 1), r, 4))
		return -1;
	seq = r[0] | r[1] << 8;
	st = r[2] | r[3] << 8;
	*verified = st == RAW_VERIFIED;
	return seq == RAW_SEQ0 ? 0 : seq == RAW_SEQ1 ? 1 : -1;
}

/* append {seq, verified}: data slot first, then the bitmap bit that makes
 * it current - a power cut in between leaves the previous record active */
static int cfg_append(int seq)
{
	struct fdcm c;
	uint8_t rec[4], chk[4], b, nb;
	uint32_t slot, baddr;
	int v;

	if (fdcm_open(&c))
		return FWU_E_CFG;
	if (c.count >= 8u * c.bitmap_size || 8u + c.bitmap_size + 4 * (c.count + 1) > F.ota_size)
		return FWU_E_CFG;	/* would need an erase of the area: refused */
	slot = c.data + 4 * c.count;
	baddr = F.ota_addr + 8 + c.count / 8;
	if (frd(slot, chk, 4) || le32(chk) != 0xffffffffu || frd(baddr, &b, 1))
		return FWU_E_CFG;
	nb = b & ~(1u << (c.count % 8));
	if ((b & ((1u << (c.count % 8)) - 1)) || nb == b)
		return FWU_E_CFG;	/* bitmap not in the expected shape */
	rec[0] = seq ? RAW_SEQ1 & 0xff : RAW_SEQ0 & 0xff;
	rec[1] = seq ? RAW_SEQ1 >> 8 : RAW_SEQ0 >> 8;
	rec[2] = RAW_VERIFIED & 0xff;
	rec[3] = RAW_VERIFIED >> 8;
	/* the only writes outside the slot window: checked by hand */
	if (slot < c.data || slot + 4 > F.ota_addr + F.ota_size ||
	    baddr < F.ota_addr + 8 || baddr >= c.data)
		return FWU_E_DENIED;
	if (H->flash_write(slot, rec, 4) || frd(slot, chk, 4) || memcmp(chk, rec, 4))
		return FWU_E_CFG;
	if (H->flash_write(baddr, &nb, 1) || frd(baddr, &b, 1) || b != nb)
		return FWU_E_CFG;
	if (cfg_read(&v) != seq || !v)
		return FWU_E_CFG;
	H->log("fwu: boot config -> image %d (verified)\n", seq + 1);
	return 0;
}

/* ---- layout ---------------------------------------------------------- */

/* 0 = slot's APP body matches the running code, 1 = differs, <0 unknown */
static int slot_is_running(int seq)
{
	uint8_t h[64];
	uint32_t a = F.base[seq] + F.bl_size;
	int r;

	if (!H->ram_compare)
		return -1;
	if (frd(a, h, 64) || le32(h) != AWIH_MAGIC || le32(h + 36) != APP_ID ||
	    fwu_sum16(h, 64, 0) != 0xffff || le32(h + 24) < RAM_CHECK_OFF + RAM_CHECK_LEN)
		return 1;
	if (frd(a + 64 + RAM_CHECK_OFF, fbuf, 256))
		return -1;
	/* 4 x 256 bytes */
	for (r = 0; r < RAM_CHECK_LEN / 256; r++) {
		int c;

		if (r && frd(a + 64 + RAM_CHECK_OFF + r * 256, fbuf, 256))
			return -1;
		c = H->ram_compare(le32(h + 16) + RAM_CHECK_OFF + r * 256, fbuf, 256);
		if (c)
			return c;
	}
	return 0;
}

static int layout(void)
{
	uint8_t h[64];
	uint32_t priv0, ver, r0, r1;
	int seq, verified, c[2];

	if (F.probed)
		return 0;
	if (!H->flash_read || !H->flash_write || !H->flash_erase4k)
		return FWU_E_NO_FLASH;
	if (frd(0, h, 64))
		return FWU_E_FLASH;
	if (le32(h) != AWIH_MAGIC || fwu_sum16(h, 64, 0) != 0xffff || le32(h + 36) != BOOT_ID)
		return FWU_E_LAYOUT;
	F.bl_size = le32(h + 32);
	if (F.bl_size < 0x1000 || F.bl_size > 0x20000 || (F.bl_size & 0xfff))
		return FWU_E_LAYOUT;
	ver = le32(h + 4);
	priv0 = le32(h + 40);
	F.ota_addr = le32(h + 44);
	F.ota_size = priv0 >> 8 == 0xffffff ? F.bl_size : priv0 >> 8;
	if (ver < 2) {
		F.ota_addr = 0x100000;
		F.ota_size = F.bl_size;
	}
	if ((priv0 & 0xff) != 0xff && (priv0 & 0xff) != 0)
		return FWU_E_LAYOUT;	/* OTA area on another flash */
	F.ota = F.ota_addr != 0xffffffffu;
	F.base[0] = 0;
	if (F.ota) {
		if (F.ota_addr >= FWU_FLASH_SIZE || F.ota_size < 0x1000 ||
		    F.ota_size > FWU_FLASH_SIZE - F.ota_addr || (F.ota_addr & 0xfff))
			return FWU_E_LAYOUT;
		F.base[1] = F.ota_addr + F.ota_size - F.bl_size;
		seq = cfg_read(&verified);
		F.running = seq >= 0 && verified ? seq : 0;
	} else {
		F.base[1] = STOCK_SLOT1_BASE;	/* only for --force-stage */
		F.running = 0;
	}
	/* cross-check with the code in RAM where the glue can: the config
	 * can lag behind a bootloader fallback whose write-back failed */
	c[0] = slot_is_running(0);
	c[1] = slot_is_running(1);
	if (c[F.running] == 1) {
		if (F.ota && c[!F.running] == 0) {
			H->log("fwu: config says image %d, RAM says image %d\n",
			       F.running + 1, 2 - F.running);
			F.running = !F.running;
		} else {
			H->log("fwu: running code matches no slot (%d %d)\n", c[0], c[1]);
			return FWU_E_RUNNING;
		}
	}
	F.target = !F.running;
	F.lo = F.base[F.target] + F.bl_size;
	F.hi = F.base[F.target] + FWU_SLOT_LIMIT;
	r0 = F.base[F.running] + F.bl_size;
	r1 = F.base[F.running] + FWU_SLOT_LIMIT;
	if ((F.lo & 0xfff) || (F.hi & 0xfff) || F.hi > FWU_FLASH_SIZE || F.hi <= F.lo ||
	    overlaps(F.lo, F.hi, 0, F.bl_size) || overlaps(F.lo, F.hi, r0, r1) ||
	    (F.ota && overlaps(F.lo, F.hi, F.ota_addr, F.ota_addr + F.ota_size)) ||
	    overlaps(F.lo, F.hi, FWU_SYSINFO_ADDR, FWU_SYSINFO_ADDR + FWU_SYSINFO_SIZE) ||
	    overlaps(F.lo, F.hi, FWU_DEVINFO_ADDR, FWU_FLASH_SIZE))
		return FWU_E_LAYOUT;
	F.probed = 1;
	H->log("fwu: boot header OTA %s, running image %d, target window 0x%06lx-0x%06lx\n",
	       F.ota ? "enabled" : "DISABLED", F.running + 1,
	       (unsigned long)F.lo, (unsigned long)F.hi);
	return 0;
}

/* ---- image check ----------------------------------------------------- */

/* read-back crc of the span and the AWIH chain of the target slot */
static int verify(uint32_t *crc_out, uint8_t *nsec, uint32_t *bad)
{
	uint32_t a, end = F.lo + F.span_len, crc = 0, off;
	uint8_t h[64];
	int i;

	*nsec = 0;
	*bad = 0;
	for (a = F.lo; a < end; a += sizeof(fbuf)) {
		uint32_t n = end - a < sizeof(fbuf) ? end - a : sizeof(fbuf);

		if (frd(a, fbuf, n))
			return FWU_E_FLASH;
		crc = fwu_crc32(crc, fbuf, n);
	}
	*crc_out = crc;
	if (crc != F.span_crc)
		return FWU_E_IMAGE;
	off = F.bl_size;
	for (i = 0; i < FWU_MAX_SECTIONS; i++) {
		uint32_t dsize, blen, next, s;
		uint16_t sum;

		a = F.base[F.target] + off;
		*bad = a;
		if (a < F.lo || a + 64 > end || frd(a, h, 64))
			return FWU_E_IMAGE;
		if (le32(h) != AWIH_MAGIC || fwu_sum16(h, 64, 0) != 0xffff)
			return FWU_E_IMAGE;
		if (i == 0 && le32(h + 36) != APP_ID)
			return FWU_E_IMAGE;
		dsize = le32(h + 12);
		blen = le32(h + 24);
		next = le32(h + 32);
		if (dsize > end - a - 64 || blen > end - a - 64)
			return FWU_E_IMAGE;
		sum = h[10] | h[11] << 8;
		for (s = 0; s < dsize; s += sizeof(fbuf)) {
			uint32_t n = dsize - s < sizeof(fbuf) ? dsize - s : sizeof(fbuf);

			if (frd(a + 64 + s, fbuf, n))
				return FWU_E_FLASH;
			sum = fwu_sum16(fbuf, n, sum);
		}
		if (sum != 0xffff)
			return FWU_E_IMAGE;
		(*nsec)++;
		if (next == 0xffffffffu) {
			*bad = 0;
			return 0;
		}
		if (next <= off || next >= FWU_SLOT_LIMIT)
			return FWU_E_IMAGE;	/* forward only: no loops */
		off = next;
	}
	return FWU_E_IMAGE;
}

/* ---- commands -------------------------------------------------------- */

static void info(uint8_t *o, uint16_t *olen)
{
	o[0] = F.state;
	o[1] = F.probed ? F.running : 0xff;
	o[2] = F.probed ? F.target : 0xff;
	o[3] = F.probed ? F.ota : 0xff;
	put32(o + 4, F.lo);
	put32(o + 8, F.next);
	put32(o + 12, F.span_len);
	put32(o + 16, F.span_crc);
	put32(o + 20, F.hi);
	*olen = 24;
}

static uint16_t begin(const uint8_t *p, uint16_t plen)
{
	uint32_t off, len, crc;
	uint8_t flags;
	int r = layout();

	if (r)
		return r;
	if (plen < 20 || le32(p) != FWU_MAGIC)
		return FWU_E_ARG;
	off = le32(p + 4);
	len = le32(p + 8);
	crc = le32(p + 12);
	flags = p[18];
	if (F.state == FWU_COMMITTED)
		return FWU_E_STATE;	/* reboot (or abort = revert) first */
	if (!F.ota && !(flags & FWU_F_FORCE_STAGE))
		return FWU_E_OTA_DISABLED;
	if (off != F.bl_size || len < 64)
		return FWU_E_ARG;
	if (len > FWU_SLOT_LIMIT - off || len > F.hi - F.lo)
		return FWU_E_TOO_BIG;
	if ((flags & FWU_F_RESUME) && F.state != FWU_IDLE &&
	    len == F.span_len && crc == F.span_crc) {
		H->log("fwu: resume at %lu of %lu\n", (unsigned long)F.next, (unsigned long)len);
		return 0;
	}
	F.state = FWU_RECEIVING;
	F.span_len = len;
	F.span_crc = crc;
	F.next = 0;
	F.erased_to = F.lo;
	H->log("fwu: receiving %lu bytes for image %d at 0x%06lx\n",
	       (unsigned long)len, F.target + 1, (unsigned long)F.lo);
	return 0;
}

static uint16_t data(uint16_t param, const uint8_t *p, uint16_t plen)
{
	uint32_t off, n, a, i;
	const uint8_t *d = p + 8;
	int fill = param & FWU_DATA_FILL, r;

	if (F.state != FWU_RECEIVING && F.state != FWU_RECEIVED)
		return FWU_E_STATE;
	if (plen < 8)
		return FWU_E_ARG;
	off = le32(p);
	n = fill ? le32(p + 4) : plen - 8u;
	if (!n || (!fill && n > FWU_CHUNK_MAX) || off > F.span_len || n > F.span_len - off)
		return FWU_E_ARG;
	if (!fill && fwu_crc32(0, d, n) != le32(p + 4))
		return FWU_E_CRC;
	a = F.lo + off;
	if (off < F.next) {
		/* repeat of an acknowledged chunk (reply lost): fine if equal */
		if (n > F.next - off)
			return FWU_E_OFFSET;
		for (i = 0; i < n; i += sizeof(fbuf)) {
			uint32_t k = n - i < sizeof(fbuf) ? n - i : sizeof(fbuf);

			if (frd(a + i, fbuf, k))
				return FWU_E_FLASH;
			if (fill ? fbuf[0] != 0xff || (k > 1 && memcmp(fbuf, fbuf + 1, k - 1))
				 : memcmp(fbuf, d + i, k))
				return FWU_E_OFFSET;
		}
		return 0;
	}
	if (off != F.next)
		return FWU_E_OFFSET;
	while (F.erased_to < a + n) {
		r = ferase(F.erased_to);
		if (r)
			return r;
		F.erased_to += 0x1000;
	}
	if (!fill && (r = fwrite_(a, d, n)))
		return r;
	for (i = 0; i < n; i += sizeof(fbuf)) {
		uint32_t k = n - i < sizeof(fbuf) ? n - i : sizeof(fbuf), j;

		if (frd(a + i, fbuf, k))
			return FWU_E_FLASH;
		if (fill) {
			for (j = 0; j < k; j++)
				if (fbuf[j] != 0xff)
					return FWU_E_READBACK;
		} else if (memcmp(fbuf, d + i, k)) {
			return FWU_E_READBACK;
		}
	}
	F.next += n;
	if (F.next == F.span_len) {
		F.state = FWU_RECEIVED;
		H->log("fwu: all %lu bytes written\n", (unsigned long)F.span_len);
	}
	return 0;
}

uint16_t fwu_cmd(uint8_t cmd, uint16_t param, const uint8_t *p, uint16_t plen,
		 uint8_t *o, uint16_t *olen)
{
	uint32_t crc = 0, bad = 0;
	uint8_t nsec = 0;
	int r;

	*olen = 0;
	if (!H)
		return FWU_E_STATE;
	switch (cmd) {
	case FWU_CMD_BEGIN:
		r = begin(p, plen);
		info(o, olen);
		return r;
	case FWU_CMD_DATA:
		r = data(param, p, plen);
		put32(o, F.next);
		*olen = 4;
		return r;
	case FWU_CMD_VERIFY:
		if (F.state != FWU_RECEIVED && F.state != FWU_VERIFIED && F.state != FWU_COMMITTED)
			return FWU_E_STATE;
		r = verify(&crc, &nsec, &bad);
		put32(o, crc);
		o[4] = nsec;
		o[5] = o[6] = o[7] = 0;
		put32(o + 8, bad);
		*olen = 12;
		if (r) {
			H->log("fwu: verify FAILED (%d) crc %08lx at 0x%06lx\n", r,
			       (unsigned long)crc, (unsigned long)bad);
			if (F.state != FWU_COMMITTED)
				F.state = FWU_IDLE;	/* start over */
			return r;
		}
		if (F.state != FWU_COMMITTED)
			F.state = FWU_VERIFIED;
		H->log("fwu: image %d verified: %u sections, crc %08lx\n",
		       F.target + 1, nsec, (unsigned long)crc);
		return 0;
	case FWU_CMD_COMMIT:
		if (param != FWU_COMMIT_MAGIC || plen < 4)
			return FWU_E_ARG;
		if (F.state != FWU_VERIFIED || le32(p) != F.span_crc)
			return FWU_E_NOT_VERIFIED;
		if (!F.ota)
			return FWU_E_OTA_DISABLED;
		r = verify(&crc, &nsec, &bad);	/* once more, right before */
		if (r) {
			F.state = FWU_IDLE;
			return r;
		}
		r = cfg_append(F.target);
		if (r)
			return r;
		F.state = FWU_COMMITTED;
		info(o, olen);
		return 0;
	case FWU_CMD_INFO:
		if (param == 0) {
			r = layout();
			info(o, olen);
			return r;
		}
		if (param == 1) {	/* abort; after a commit: revert the config */
			if (F.state == FWU_COMMITTED) {
				r = cfg_append(F.running);
				if (r)
					return r;
				F.state = FWU_VERIFIED;
			} else {
				F.state = FWU_IDLE;
			}
			info(o, olen);
			return 0;
		}
		if (param == 2) {
			if (F.state != FWU_COMMITTED || !H->reboot)
				return FWU_E_STATE;
			F.reboot = 1;
			info(o, olen);
			return 0;
		}
		return FWU_E_ARG;
	}
	return FWU_E_ARG;
}

int fwu_take_reboot(void)
{
	int r = F.reboot;

	F.reboot = 0;
	return r;
}

void fwu_summary(uint8_t *state, uint8_t *running, uint8_t *ota)
{
	*state = F.state;
	*running = F.probed ? F.running : 0xff;
	*ota = F.probed ? F.ota : 0xff;
}
