#!/usr/bin/env python3
"""XR871 service console client over the ESP32 UART monitor (no clip needed).

Uses the monitor's telnet console of the channel that receives the XR871
console (--ch, default C = port 2323). The console sends keystrokes on the TX
pin set by "keys to" in the monitor settings (C -> TX B GPIO19 by default) and
drives it only while this tool is connected: TX is released on exit, nothing
is switched on permanently.
Wiring (camera 1 default): ESP32 GPIO21 (ch C RX) <- camera pad "TX",
        ESP32 GPIO19 (TX B) -> camera pad "RX", GND - GND, 115200 8N1.
Camera 2: XR871 console on channel A -> --ch A (keys to TX A, GPIO17).
Only connect while the camera is powered.

usage:
  xr871-svc.py [--host IP] enter            wait for the boot window, send 's'
                                            (power-cycle the camera while it waits)
  xr871-svc.py [--host IP] cmd 'peek 0x40040200 4'
  xr871-svc.py [--host IP] flash IMAGE.bin  rewrite changed 4 KiB sectors in
                                            0x8000..end (boot area untouched),
                                            verify every chunk and sector by CRC32
  xr871-svc.py [--host IP] boot|reboot

The firmware only accepts service commands if 's' arrives within 3 s after
"boot: level0 done" (before the WLAN init). Output of the console is the XR871
debug log: do not paste it anywhere public.
"""
import argparse
import json
import socket
import sys
import time
import urllib.request
import zlib

SECTOR = 0x1000
CHUNK = 128          # bytes per fwrite line (256 hex chars)
BOOT_END = 0x8000


class Svc:
    def __init__(self, host, ch="C"):
        self.host = host
        port = 2321 + "ABC".index(ch)
        st = json.load(urllib.request.urlopen("http://%s/status" % host, timeout=5))
        ctx = st["ch"]["ABC".index(ch)]["ctx"]
        if ctx < 0:
            sys.exit("channel %s is read only (monitor settings: keys to)" % ch)
        print("console %s (port %d), keys go out on TX %s" % (ch, port, "AB"[ctx]))
        # one telnet console socket for both directions; TX armed only while open
        self.rx = self.tx = socket.create_connection((host, port), timeout=5)
        self.rx.settimeout(0.05)
        self.buf = b""
        self.iac = b""
        time.sleep(0.3)
        self.drain()

    def close(self):
        try:
            self.tx.close()     # last console closed -> monitor releases TX
        except OSError:
            pass

    def _strip_telnet(self, d):
        """remove telnet IAC negotiation, un-double 0xff"""
        d = self.iac + d
        self.iac = b""
        out = bytearray()
        i = 0
        while i < len(d):
            b = d[i]
            if b != 0xFF:
                out.append(b)
                i += 1
                continue
            if i + 1 >= len(d):
                self.iac = d[i:]
                break
            c = d[i + 1]
            if c == 0xFF:
                out.append(0xFF)
                i += 2
            elif 0xFB <= c <= 0xFE:
                if i + 2 >= len(d):
                    self.iac = d[i:]
                    break
                i += 3
            else:
                i += 2
        return bytes(out)

    def drain(self):
        try:
            while self.rx.recv(4096):
                pass
        except (socket.timeout, BlockingIOError):
            pass
        self.buf = b""

    def lines(self, timeout):
        end = time.time() + timeout
        while time.time() < end:
            try:
                d = self.rx.recv(4096)
                if d:
                    self.buf += self._strip_telnet(d)
            except socket.timeout:
                pass
            while b"\n" in self.buf:
                ln, self.buf = self.buf.split(b"\n", 1)
                yield ln.rstrip(b"\r").decode("ascii", "replace")
        return

    def send(self, s):
        self.tx.sendall(s.encode().replace(b"\xff", b"\xff\xff"))

    def cmd(self, line, timeout=10.0, show=False):
        self.send(line + "\n")
        out = []
        for ln in self.lines(timeout):
            if ln.startswith("OK") or ln.startswith("ERR"):
                return ln, out
            out.append(ln)
            if show:
                print(ln)
        raise RuntimeError("no answer to %r" % line.split(" ")[0])

    def enter(self, wait=600):
        print("waiting for the boot window, power-cycle the camera now ...")
        end = time.time() + wait
        while time.time() < end:
            self.send("s")
            for ln in self.lines(0.1):
                if "svc: service console" in ln:
                    for _ in self.lines(0.5):
                        pass
                    print("service console active")
                    return True
        print("no service console seen")
        return False


def flash(s, path):
    img = open(path, "rb").read()
    end = len(img)
    while end > BOOT_END and img[end - 1] == 0xFF:
        end -= 1
    end = (end + SECTOR - 1) & ~(SECTOR - 1)
    t0 = time.time()
    changed = 0
    for off in range(BOOT_END, end, SECTOR):
        want = img[off:off + SECTOR]
        st, _ = s.cmd("fcrc 0x%x 0x%x" % (off, SECTOR))
        if st == "OK %08x" % zlib.crc32(want):
            continue
        changed += 1
        st, _ = s.cmd("ferase 0x%x 0x%x" % (off, SECTOR), timeout=15)
        if st != "OK":
            sys.exit("erase 0x%x failed: %s" % (off, st))
        for c in range(0, SECTOR, CHUNK):
            data = want[c:c + CHUNK]
            if data == b"\xff" * len(data):
                continue
            for attempt in range(3):
                st, _ = s.cmd("fwrite 0x%x %s" % (off + c, data.hex()))
                if st == "OK %08x" % zlib.crc32(data):
                    break
                s.drain()
            else:
                sys.exit("write 0x%x failed: %s" % (off + c, st))
        st, _ = s.cmd("fcrc 0x%x 0x%x" % (off, SECTOR))
        if st != "OK %08x" % zlib.crc32(want):
            sys.exit("verify 0x%x failed: %s" % (off, st))
        print("sector 0x%05x written" % off)
    st, _ = s.cmd("fcrc 0x%x 0x%x" % (BOOT_END, end - BOOT_END), timeout=30)
    ok = st == "OK %08x" % zlib.crc32(img[BOOT_END:end])
    print("%d sectors rewritten, full check 0x%x-0x%x %s, %.0f s"
          % (changed, BOOT_END, end, "OK" if ok else "FAILED (" + st + ")", time.time() - t0))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.10.183")
    ap.add_argument("--ch", default="C", choices=["A", "B", "C"],
                    help="monitor channel that receives the XR871 console")
    ap.add_argument("what", choices=["enter", "cmd", "flash", "boot", "reboot"])
    ap.add_argument("arg", nargs="?")
    a = ap.parse_args()
    s = Svc(a.host, a.ch)
    try:
        if a.what == "enter":
            rc = 0 if s.enter() else 1
        elif a.what == "cmd":
            st, _ = s.cmd(a.arg, show=True)
            print(st)
            rc = 0
        elif a.what == "flash":
            rc = 0 if flash(s, a.arg) else 1
        else:
            st, _ = s.cmd(a.what)
            print(st)
            rc = 0
    finally:
        s.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
