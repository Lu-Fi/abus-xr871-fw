#!/usr/bin/env python3
"""Write an XR871 flash image over the svc console (ESP32 monitor telnet).

usage: xr871-svc-flash.py HOST[:PORT] NEW.bin [--old OLD.bin] [--write]
                           [--from 0x8000] [--to 0x200000] [--chunk 256]

Compares NEW with the chip (fcrc per 4 KiB block; with --old only the
blocks that differ between OLD and NEW are checked). Without --write it
only reports which blocks differ (read-only). With --write it erases and
rewrites each differing block, checks every chunk's CRC and the whole
block's CRC, and retries a block up to 3 times. The boot area below
0x8000 is never touched (svc refuses it too).
The svc console must already be open (svc2/svc3 "svc:" prompt).
"""
import argparse, socket, sys, time, zlib

ap = argparse.ArgumentParser()
ap.add_argument("host")
ap.add_argument("new")
ap.add_argument("--old")
ap.add_argument("--write", action="store_true")
ap.add_argument("--from", dest="lo", type=lambda x: int(x, 0), default=0x8000)
ap.add_argument("--to", dest="hi", type=lambda x: int(x, 0), default=0x200000)
ap.add_argument("--chunk", type=int, default=256)
a = ap.parse_args()

host, _, port = a.host.partition(":")
port = int(port or 2321)
new = open(a.new, "rb").read()
old = open(a.old, "rb").read() if a.old else None
lo = max(a.lo, 0x8000)
B = 0x1000

s = socket.create_connection((host, port))
s.settimeout(0.05)
rx = b""


def drain(t):
    global rx
    end = time.time() + t
    while time.time() < end:
        try:
            d = s.recv(4096)
            if not d:
                sys.exit("connection closed")
        except socket.timeout:
            continue
    rx = b""


def cmd(line, timeout=10.0):
    """send one command, return the final OK/ERR line"""
    global rx
    s.sendall(line.encode() + b"\r")
    end = time.time() + timeout
    while time.time() < end:
        try:
            d = s.recv(4096)
            if not d:
                sys.exit("connection closed")
            rx += d.replace(b"\xff", b"")
        except socket.timeout:
            pass
        while b"\n" in rx:
            l, rx = rx.split(b"\n", 1)
            l = l.strip().decode(errors="replace")
            if l.startswith("OK") or l.startswith("ERR"):
                return l
    return "TIMEOUT"


def chip_crc(off, n):
    r = cmd("fcrc 0x%x 0x%x" % (off, n), 20)
    return int(r.split()[1], 16) if r.startswith("OK ") else None


def console_free(ch):
    """no other terminal on this TX pin (web terminal, telnet), per /status"""
    import json, urllib.request
    try:
        st = json.load(urllib.request.urlopen("http://%s/status" % host, timeout=5))
        return st["ch"][ch]["tx_console"] <= 1, st["ch"][ch]["tx_console"]
    except Exception as e:
        return False, str(e)


def wait_quiet(t=1.0, limit=30.0):
    """wait until the channel has been silent for t seconds"""
    end = time.time() + limit
    last = time.time()
    while time.time() < end:
        try:
            if s.recv(4096):
                last = time.time()
        except socket.timeout:
            if time.time() - last >= t:
                return True
    return False


ch = port - 2321
ok, n = console_free(ch)
if not ok:
    sys.exit("console busy: %s other terminal(s) on this channel; close them first" % n)
drain(1.5)  # the telnet port replays history on connect
if not wait_quiet():
    sys.exit("channel never went quiet for 1 s; not sending anything")
for _try in range(3):  # a reply can get lost while the XR871 logs
    _h = cmd("help", 5)
    if _h == "OK":
        break
    drain(1.0)
if _h != "OK":
    print("help answered: %r" % _h)
    sys.exit("svc console not answering (is the 'svc:' console open?)")

blocks = [o for o in range(lo, min(a.hi, len(new)), B)
          if old is None or old[o:o + B] != new[o:o + B]]
todo = []
for o in blocks:
    c = chip_crc(o, B)
    want = zlib.crc32(new[o:o + B])
    if c != want:
        todo.append(o)
print("checked %d blocks, %d differ from NEW" % (len(blocks), len(todo)))
if not a.write or not todo:
    sys.exit(0)

t0 = time.time()
for i, o in enumerate(todo):
    for attempt in range(3):
        ok = cmd("ferase 0x%x 0x%x" % (o, B), 10) == "OK"
        p = o
        while ok and p < o + B:
            chunk = new[p:p + a.chunk]
            if chunk == b"\xff" * len(chunk):
                p += len(chunk)
                continue
            r = cmd("fwrite 0x%x %s" % (p, chunk.hex()), 10)
            ok = r == "OK %08x" % zlib.crc32(chunk)
            p += len(chunk)
        if ok and chip_crc(o, B) == zlib.crc32(new[o:o + B]):
            break
        print("block 0x%x attempt %d failed, retrying" % (o, attempt + 1))
        drain(0.5)
    else:
        sys.exit("block 0x%x FAILED 3 times; chip is NOT consistent" % o)
    print("block 0x%x ok (%d/%d, %.0f s)" % (o, i + 1, len(todo), time.time() - t0))
    sys.stdout.flush()

span = (todo[0], todo[-1] + B)
c = chip_crc(span[0], span[1] - span[0])
print("final crc 0x%x-0x%x: %s" % (span[0], span[1],
      "MATCH" if c == zlib.crc32(new[span[0]:span[1]]) else "MISMATCH"))
