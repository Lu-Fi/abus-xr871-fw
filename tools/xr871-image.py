#!/usr/bin/env python3
"""XR871 flash image tool (AWIH section chain), no vendor binaries needed.

  xr871-image.py check <flash.bin> [start]
      walk the section chain from <start> (default 0) and verify the header
      and data checksums of every section.

  xr871-image.py extract <flash.bin> <outdir> [start]
      write every section body to <outdir>/<id>.bin

  xr871-image.py build <base.bin|-> <out.bin> <id>:<file>:<flash_off>:<load>:<entry>:<attr> ...
      build a chain into a copy of <base.bin> (or an erased 2 MiB image for
      "-"), sections linked in the given order; offsets/addresses accept hex
      and "K" suffix. Use "x" for 0xffffffff.

  xr871-image.py set-ota <in.bin> <out.bin> [ota_addr ota_size]
      rewrite the bootloader header's OTA fields (priv[0] = flash 0xff |
      size << 8, priv[1] = address; default = stock 0x100000 / 0x4000) and
      fix its header checksum. "set-ota in out off" disables OTA (0xff..).
      NOTE: "build" keeps the priv fields of a boot section found in the
      base image (older versions wrote 0xff = OTA disabled).

  xr871-image.py bootsel <flash.bin>
      which image the stock bootloader (= XR871 SDK bl_load_bin) would boot
      from this flash: OTA fields, last OTA config record, APP section check
      with fallback to the other image.

Checksum rules (XR871 SDK src/image/image.c): 16-bit little-endian word sum;
header sum over 64 bytes must be 0xFFFF, data_chksum + sum(body) must be
0xFFFF.
"""
import os
import struct
import sys

MAGIC = 0x48495741  # "AWIH"
HDR = struct.Struct("<IIHHIIIIIII24s")
FLASH = 2 * 1024 * 1024


def csum16(b):
    s = 0
    n = len(b) & ~1
    for (w,) in struct.iter_unpack("<H", b[:n]):
        s = (s + w) & 0xFFFF
    if len(b) & 1:
        s = (s + b[-1]) & 0xFFFF
    return s


def num(v):
    v = v.strip()
    if v == "x":
        return 0xFFFFFFFF
    if v.upper().endswith("K"):
        return int(v[:-1], 0) * 1024
    return int(v, 0)


def parse_hdr(img, off):
    (magic, ver, hsum, dsum, dsize, load, entry, blen, attr, nxt, sid,
     priv) = HDR.unpack_from(img, off)
    return dict(magic=magic, ver=ver, hsum=hsum, dsum=dsum, size=dsize,
                load=load, entry=entry, blen=blen, attr=attr, next=nxt,
                id=sid, priv=priv)


def walk(img, start=0):
    off, seen = start, set()
    while off != 0xFFFFFFFF and off + 64 <= len(img) and off not in seen:
        seen.add(off)
        h = parse_hdr(img, off)
        if h["magic"] != MAGIC:
            yield off, None, False, False
            return
        body = img[off + 64:off + 64 + h["blen"]]
        hok = csum16(img[off:off + 64]) == 0xFFFF
        dok = (h["dsum"] + csum16(body)) & 0xFFFF == 0xFFFF
        yield off, h, hok, dok
        off = h["next"]


def make_header(sid, body, load, entry, attr, nxt, ver=3, priv=b"\xff" * 24):
    dsum = (0xFFFF - csum16(body)) & 0xFFFF
    raw = bytearray(HDR.pack(MAGIC, ver, 0, dsum, len(body), load, entry,
                             len(body), attr, nxt, sid, priv))
    hsum = (0xFFFF - csum16(bytes(raw))) & 0xFFFF
    struct.pack_into("<H", raw, 8, hsum)
    assert csum16(bytes(raw)) == 0xFFFF
    return bytes(raw)


def cmd_check(path, start="0"):
    img = open(path, "rb").read()
    ok_all = True
    for off, h, hok, dok in walk(img, num(start)):
        if h is None:
            print(f"{off:#08x}: no AWIH magic")
            ok_all = False
            break
        ok_all &= hok and dok
        print(f"{off:#08x}: id {h['id']:#010x} size {h['size']:#8x} load {h['load']:#010x} "
              f"entry {h['entry']:#010x} attr {h['attr']:#x} next {h['next']:#010x} "
              f"hdr {'ok' if hok else 'BAD'} data {'ok' if dok else 'BAD'}")
    return 0 if ok_all else 1


def cmd_extract(path, outdir, start="0"):
    img = open(path, "rb").read()
    os.makedirs(outdir, exist_ok=True)
    for off, h, hok, dok in walk(img, num(start)):
        if h is None:
            break
        fn = os.path.join(outdir, f"{h['id']:08x}.bin")
        open(fn, "wb").write(img[off + 64:off + 64 + h["blen"]])
        print(f"{fn} {h['blen']} bytes (load {h['load']:#x}, entry {h['entry']:#x}, attr {h['attr']:#x})")
    return 0


def cmd_build(base, out, *specs):
    img = bytearray(b"\xff" * FLASH) if base == "-" else bytearray(open(base, "rb").read())
    secs = []
    for s in specs:
        sid, fn, off, load, entry, attr = s.split(":")
        secs.append((num(sid), open(fn, "rb").read(), num(off), num(load), num(entry), num(attr)))
    for i, (sid, body, off, load, entry, attr) in enumerate(secs):
        nxt = secs[i + 1][2] if i + 1 < len(secs) else 0xFFFFFFFF
        priv = b"\xff" * 24
        if sid == BOOT_ID and base != "-" and off + 64 <= len(img):
            old = parse_hdr(img, off)
            if old["magic"] == MAGIC and old["id"] == BOOT_ID:
                priv = old["priv"]  # keep the OTA area of the base image
        end = off + 64 + len(body)
        if i + 1 < len(secs) and end > secs[i + 1][2]:
            sys.exit(f"section {sid:#x} ({end:#x}) overlaps next at {secs[i + 1][2]:#x}")
        if end > len(img):
            sys.exit(f"section {sid:#x} exceeds flash size")
        img[off:off + 64] = make_header(sid, body, load, entry, attr, nxt, priv=priv)
        img[off + 64:end] = body
        print(f"{off:#08x}: id {sid:#010x} {len(body)} bytes -> next {nxt:#010x}")
    open(out, "wb").write(img)
    return cmd_check(out)


BOOT_ID = 0xA5FF5A00
APP_ID = 0xA5FE5A01


def set_priv(img, priv):
    h = bytearray(img[0:64])
    h[40:64] = priv
    h[8:10] = b"\0\0"
    struct.pack_into("<H", h, 8, (0xFFFF - csum16(bytes(h))) & 0xFFFF)
    assert csum16(bytes(h)) == 0xFFFF
    img[0:64] = h


def cmd_set_ota(src, out, addr="0x100000", size="0x4000"):
    img = bytearray(open(src, "rb").read())
    h = parse_hdr(img, 0)
    if h["magic"] != MAGIC or h["id"] != BOOT_ID:
        sys.exit("no bootloader header at 0")
    if addr == "off":
        priv = b"\xff" * 24
    else:
        priv = struct.pack("<II", 0xFF | (num(size) << 8), num(addr)) + b"\xff" * 16
    set_priv(img, priv)
    open(out, "wb").write(img)
    return cmd_bootsel(out)


def boot_layout(img):
    """(bl_size, ota_addr or None, ota_size, base[2]) like SDK image_init"""
    h = parse_hdr(img, 0)
    if h["magic"] != MAGIC or csum16(img[0:64]) != 0xFFFF:
        raise ValueError("bad bootloader header")
    bl = h["next"]
    p0, p1 = struct.unpack_from("<II", h["priv"], 0)
    size = bl if (p0 >> 8) == 0xFFFFFF else p0 >> 8
    addr = p1
    if h["ver"] < 2:
        addr, size = 0x100000, bl
    if addr == 0xFFFFFFFF:
        return bl, None, size, [0, None]
    return bl, addr, size, [0, addr + size - bl]


def fdcm_last(img, addr, size):
    """last OTA config record (raw_seq, raw_state) or None, SDK fdcm_read"""
    idc, bms, dsz = struct.unpack_from("<IHH", img, addr)
    if idc != 0xA55A5AA5 or dsz != 4 or bms == 0 or bms + 8 >= size:
        return None
    cnt = 0
    for b in img[addr + 8:addr + 8 + bms]:
        if b == 0:
            cnt += 8
            continue
        v = (~b) & 0xFF
        while v:
            v >>= 1
            cnt += 1
        break
    if cnt == 0:
        return None
    return struct.unpack_from("<HH", img, addr + 8 + bms + 4 * (cnt - 1))


def app_ok(img, bl, base):
    """bootloader check of the APP section of an image (header + data sum)"""
    off = bl
    for _ in range(16):
        a = base + off
        if a + 64 > len(img):
            return False
        h = parse_hdr(img, a)
        if h["magic"] != MAGIC:
            return False
        if h["id"] == APP_ID:
            body = img[a + 64:a + 64 + h["blen"]]
            return (csum16(img[a:a + 64]) == 0xFFFF and
                    (h["dsum"] + csum16(body)) & 0xFFFF == 0xFFFF and
                    h["load"] + h["blen"] <= 0x67000)
        if h["next"] == 0xFFFFFFFF:
            return False
        off = h["next"]
    return False


def bootsel(img):
    """(image index 1/2 or None, text) - model of SDK bl_load_bin"""
    bl, addr, size, base = boot_layout(img)
    if addr is None:
        ok = app_ok(img, bl, 0)
        return (1 if ok else None), "OTA disabled in the boot header -> image 1 only, no fallback" + (
            "" if ok else ", and its APP section is BAD")
    rec = fdcm_last(img, addr, size)
    cfg = None
    if rec and rec[1] == 0x9669 and rec[0] in (0x5555, 0xAAAA):
        cfg = 0 if rec[0] == 0x5555 else 1
    txt = f"OTA area {addr:#x}/{size:#x}, image 2 base {base[1]:#x}, config " + (
        f"{rec[0]:#06x}/{rec[1]:#06x}" if rec else "unreadable")
    seq = 0 if cfg is None else cfg
    for _ in range(2):
        if app_ok(img, bl, base[seq]):
            if seq != cfg:
                txt += f"; would WRITE config -> image {seq + 1}"
            return seq + 1, txt
        txt += f"; image {seq + 1} APP bad"
        seq ^= 1
    return None, txt + "; NOTHING bootable (upgrade mode)"


def cmd_bootsel(path):
    img = open(path, "rb").read()
    n, txt = bootsel(img)
    print(f"{path}: {txt}")
    print(f"boots image {n}" if n else "boots NOTHING")
    return 0 if n else 1


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    cmd = sys.argv[1]
    if cmd == "check":
        return cmd_check(*sys.argv[2:4])
    if cmd == "extract":
        return cmd_extract(*sys.argv[2:5])
    if cmd == "build":
        return cmd_build(*sys.argv[2:])
    if cmd == "set-ota":
        return cmd_set_ota(*sys.argv[2:6])
    if cmd == "bootsel":
        return cmd_bootsel(sys.argv[2])
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
