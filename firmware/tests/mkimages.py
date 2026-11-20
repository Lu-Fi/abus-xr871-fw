#!/usr/bin/env python3
"""Synthetic XR871 flash images for the host tests (random bodies, no data
from the real device).

usage: mkimages.py <outdir>

  flash_ota.bin    2 MiB, boot header with the stock OTA area
                   (0x100000/0x4000), image 1 @0x8000 (running), an older
                   image 2 @0xfc000 base, OTA config ending in image 1,
                   stand-ins for sysinfo 0xfb000 and device info 0x1fe000
  flash_noota.bin  the same with OTA disabled in the boot header (what the
                   m1/m2 builds have)
  new_full.bin     a new firmware as a full 2 MiB flash image (other
                   layout, a 0xff gap between app and net)
  new_chain.bin    the same chain as a file starting at image offset 0x8000
  bad_full.bin     new_full.bin with one flipped byte in the net section
"""
import importlib.util
import os
import random
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "xi", os.environ.get("XR_IMAGE_TOOL") or
    os.path.join(HERE, "..", "..", "tools", "xr871-image.py"))
xi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(xi)

FLASH = 0x200000
BOOT, APP, NET, SDD = 0xA5FF5A00, 0xA5FE5A01, 0xA5FC5A03, 0xA5F85A07
OTA_PRIV = struct.pack("<II", 0xFF | (0x4000 << 8), 0x100000) + b"\xff" * 16


def rnd(n, seed):
    return random.Random(seed).randbytes(n)


def put_chain(img, base, secs):
    """secs: (image offset, id, body, load)"""
    for i, (off, sid, body, load) in enumerate(secs):
        nxt = secs[i + 1][0] if i + 1 < len(secs) else 0xFFFFFFFF
        a = base + off
        entry = load + 0x101 if sid in (BOOT, APP) else 0xFFFFFFFF
        img[a:a + 64] = xi.make_header(sid, body, load, entry, 1, nxt)
        img[a + 64:a + 64 + len(body)] = body


def boot(img, priv):
    put_chain(img, 0, [(0, BOOT, rnd(0x7000, 0), 0x67000)])
    struct.pack_into("<I", img, 32, 0x8000)  # next = bl size
    xi.set_priv(img, priv)


def fdcm(img, records):
    a = 0x100000
    img[a:a + 0x4000] = b"\xff" * 0x4000
    bms = 0x1F1
    struct.pack_into("<IHH", img, a, 0xA55A5AA5, bms, 4)
    n = len(records)
    for i in range(n):
        img[a + 8 + i // 8] &= ~(1 << (i % 8)) & 0xFF
    for i, (seq, st) in enumerate(records):
        struct.pack_into("<HH", img, a + 8 + bms + 4 * i, seq, st)


def main(out):
    os.makedirs(out, exist_ok=True)
    img = bytearray(b"\xff" * FLASH)
    boot(img, OTA_PRIV)
    put_chain(img, 0, [(0x8000, APP, rnd(0x9000, 1), 0x10000),
                       (0x14000, NET, rnd(0x20000, 2), 0x60000000),
                       (0x40000, SDD, rnd(0x2E8, 3), 0xFFFFFFFF)])
    put_chain(img, 0xFC000, [(0x8000, APP, rnd(0xA000, 11), 0x10000),
                             (0x30000, SDD, rnd(0x2E8, 12), 0xFFFFFFFF)])
    fdcm(img, [(0xAAAA, 0x9669), (0x5555, 0x9669)])
    img[0xFB000:0xFB100] = rnd(0x100, 98)    # sysinfo stand-in
    img[0x1FE000:0x1FE238] = rnd(0x238, 99)  # device info stand-in
    open(os.path.join(out, "flash_ota.bin"), "wb").write(img)
    noota = bytearray(img)
    xi.set_priv(noota, b"\xff" * 24)
    open(os.path.join(out, "flash_noota.bin"), "wb").write(noota)

    new = bytearray(b"\xff" * FLASH)
    boot(new, OTA_PRIV)
    # the UART frame magic inside the update data: a real image has it
    # (ap18 at span offset 12852), the receiver must treat it as data
    app = bytearray(rnd(0xA400, 21))
    app[0x1234:0x1238] = b"\xaa\x55\xaa\x55"
    net = bytearray(rnd(0x24000, 22))
    net[0x777:0x77F] = b"\xaa\x55\xaa\x55" * 2
    put_chain(new, 0, [(0x8000, APP, bytes(app), 0x10000),
                       (0x16000, NET, bytes(net), 0x60000000),
                       (0x44000, SDD, rnd(0x2E8, 23), 0xFFFFFFFF)])
    open(os.path.join(out, "new_full.bin"), "wb").write(new)
    end = 0x44000 + 64 + 0x2E8
    open(os.path.join(out, "new_chain.bin"), "wb").write(new[0x8000:end])
    bad = bytearray(new)
    bad[0x16000 + 64 + 0x1234] ^= 0x40
    open(os.path.join(out, "bad_full.bin"), "wb").write(bad)
    for f in ("flash_ota.bin", "flash_noota.bin", "new_full.bin"):
        n, txt = xi.bootsel(open(os.path.join(out, f), "rb").read())
        print(f"{f}: boots image {n} ({txt})")


if __name__ == "__main__":
    main(sys.argv[1])
