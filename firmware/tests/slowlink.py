#!/usr/bin/env python3
"""UART-speed relay for the host tests: pty (xr871d side) <-> unix socket
(xr871-host connects), each direction limited to 115200 8N1 = 11520 B/s.

usage: slowlink.py <socket path> <pty symlink path> [bytes_per_s]
"""
import os
import select
import socket
import sys
import time
import tty

path, link = sys.argv[1], sys.argv[2]
rate = float(sys.argv[3]) if len(sys.argv) > 3 else 11520.0
m, s = os.openpty()
tty.setraw(s)
tty.setraw(m)
if os.path.lexists(link):
    os.unlink(link)
os.symlink(os.ttyname(s), link)
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(path)
srv.listen(1)
c, _ = srv.accept()
q = {m: bytearray(), c.fileno(): bytearray()}   # pending output per destination
tok = {m: 0.0, c.fileno(): 0.0}
last = time.time()
while True:
    r, _, _ = select.select([m, c], [], [], 0.002)
    for f in r:
        try:
            d = os.read(m, 4096) if f == m else c.recv(4096)
        except OSError:
            d = b""
        if not d:
            sys.exit(0)
        q[c.fileno() if f == m else m] += d
    now = time.time()
    for dst in q:
        tok[dst] = min(tok[dst] + (now - last) * rate, 64.0)
        n = int(tok[dst])
        if n and q[dst]:
            chunk = bytes(q[dst][:n])
            del q[dst][:n]
            tok[dst] -= len(chunk)
            if dst == m:
                os.write(m, chunk)
            else:
                c.sendall(chunk)
    last = now
