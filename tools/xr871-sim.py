#!/usr/bin/env python3
"""Simulated XR871 for testing the T30 side (xr871d) in QEMU or on a pty.

usage: xr871-sim.py <unix-socket-path | /dev/pts/N> [timeout_s]

Behaves like the stock XR871 as far as the T30 can see it on UART0:
  - answers hello (cmd 0x00) with 0x85 (wake source), 0x40 (time in ms),
    0x80
  - logs every frame from the T30 with a timestamp
  - arms the 6 s heartbeat timer like the stock firmware does when it
    powers the T30, re-arms it on every 0xa9, and reports a would-be
    power cycle when it expires
At the end prints a verdict: first heartbeat time, max heartbeat gap,
hello seen, marker 0xf1 seen, number of would-be power cycles.
"""
import os
import socket
import struct
import sys
import time

TIMEOUT = 6.0


def frame(cmd, param=0, payload=b""):
    return (b"\xaa\x55\xaa\x55" + struct.pack("<HBBHH", len(payload), cmd, 1, 0, param)
            + payload)


class Link:
    def __init__(self, path):
        self.sock = None
        self.fd = None
        if path.startswith("/dev/"):
            import tty
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
            tty.setraw(self.fd)
        else:
            for _ in range(600):
                try:
                    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    s.connect(path)
                    self.sock = s
                    break
                except OSError:
                    time.sleep(0.1)
            if not self.sock:
                sys.exit(f"cannot connect to {path}")
            self.sock.setblocking(False)

    def read(self):
        try:
            if self.sock:
                d = self.sock.recv(4096)
                if d == b"":
                    raise EOFError
                return d
            return os.read(self.fd, 4096)
        except (BlockingIOError, InterruptedError):
            return b""

    def write(self, b):
        if self.sock:
            self.sock.sendall(b)
        else:
            os.write(self.fd, b)


def main():
    path = sys.argv[1]
    limit = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    link = Link(path)
    t0 = time.time()
    # the stock XR871 arms the heartbeat timer when it powers the T30 on
    deadline = t0 + TIMEOUT
    buf = b""
    first_hb = None
    last_hb = None
    max_gap = 0.0
    cycles = 0
    hello = marker = False
    stats = {}

    def now():
        return time.time() - t0

    print(f"[sim] connected, T30 'powered on' at 0.000 s, heartbeat timeout {TIMEOUT} s")
    try:
        while now() < limit:
            try:
                buf += link.read()
            except EOFError:
                print(f"[sim] {now():8.3f} link closed")
                break
            while True:
                i = buf.find(b"\xaa\x55\xaa\x55")
                if i < 0:
                    buf = buf[-3:]
                    break
                if len(buf) - i < 12:
                    buf = buf[i:]
                    break
                ln, cmd, flag, _, param = struct.unpack_from("<HBBHH", buf, i + 4)
                if len(buf) - i < 12 + ln:
                    buf = buf[i:]
                    break
                payload = buf[i + 12:i + 12 + ln]
                buf = buf[i + 12 + ln:]
                stats[cmd] = stats.get(cmd, 0) + 1
                t = now()
                if cmd == 0xA9:
                    if first_hb is None:
                        first_hb = t
                        print(f"[sim] {t:8.3f} first heartbeat")
                    if last_hb is not None:
                        max_gap = max(max_gap, t - last_hb)
                    last_hb = t
                    deadline = time.time() + TIMEOUT
                elif cmd == 0x00:
                    print(f"[sim] {t:8.3f} hello -> sending 0x85/0x40/0x80")
                    hello = True
                    link.write(frame(0x85, 0, struct.pack("<I", 4)))
                    link.write(frame(0x40, 0, struct.pack("<Q", int(time.time() * 1000))))
                    link.write(frame(0x80, 0, struct.pack("<I", 0)))
                elif cmd == 0xF1:
                    marker = True
                    print(f"[sim] {t:8.3f} marker: {payload.rstrip(bytes(1)).decode(errors='replace')}")
                else:
                    print(f"[sim] {t:8.3f} cmd {cmd:#04x} param {param} len {ln}")
            if time.time() > deadline:
                cycles += 1
                print(f"[sim] {now():8.3f} HEARTBEAT TIMEOUT -> stock XR871 would power-cycle the T30")
                deadline = time.time() + TIMEOUT
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    print("[sim] ---- verdict ----")
    print(f"[sim] first heartbeat: {first_hb if first_hb is None else round(first_hb, 2)} s"
          f", max gap: {max_gap:.2f} s, hello: {hello}, marker: {marker}"
          f", would-be power cycles: {cycles}")
    print(f"[sim] frames by cmd: { {hex(k): v for k, v in sorted(stats.items())} }")
    return 0 if (first_hb is not None and cycles == 0 and marker) else 1


if __name__ == "__main__":
    sys.exit(main())
