#!/usr/bin/env python3
"""Host tests for the service extensions (no hardware, no QEMU, no SDK).

  1. stock compatibility: xr871d against tools/xr871-sim.py (stock XR871
     model): extension commands are refused locally, no 0xd0..0xdc frame
     ever reaches the stock firmware
  2. end to end: real xr871d <-> real XR871 core (xr871-host, flash file
     with OTA enabled) over a pty: status, mcu, gpio, gpio scan, log
     forwarding to the log file, fwupdate stage / commit / abort / reboot,
     corrupt image refused, flash outside the target window unchanged
  3. OTA disabled flash: refused, --force-stage writes image 2 only, commit
     refused, the bootloader model still boots image 1
  4. raw protocol (python plays the T30 against xr871-host): protected
     pins/MCU messages refused by the XR871 itself, credentials never
     forwarded, log rate limit + drop notice, update state machine (bad
     crc, order, resume, duplicate chunk, commit magic, begin after commit,
     revert), running-slot detection from RAM

  5. the same update over a 115200 baud link model with W25Q16 timings

usage: run_tests.py  (exit 0 = all PASS)
Paths (defaults = the real trees; the environment overrides them, e.g. to
test a scratch copy before installing it):
  XR_FW          xr871fw directory            (default: parent of tests/)
  XR_XD          directory with xr871d.c      (default: thingino-aperto package)
  XR_IMAGE_TOOL  xr871-image.py               (default: ABUS/tools)
  XR_TEST_OUT    where the run directory goes (default: ABUS/work/fwtest)
"""
import importlib.util
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ABUS = "/mnt/NVMe/git/ABUS"
FW = os.environ.get("XR_FW") or os.path.dirname(HERE)
XD = os.environ.get("XR_XD") or "/mnt/NVMe/git/thingino-aperto/package/xr871-bridge/files"
TOOL = os.environ.get("XR_IMAGE_TOOL") or os.path.join(ABUS, "tools", "xr871-image.py")
OUT = os.environ.get("XR_TEST_OUT") or os.path.join(ABUS, "work", "fwtest")
SIM = os.path.join(ABUS, "tools", "xr871-sim.py")
CORE = "abus-xr871 core 4"
spec = importlib.util.spec_from_file_location("xi", TOOL)
xi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(xi)

results = []


def check(name, cond, info=""):
    results.append((name, bool(cond)))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f"  ({info})" if info and not cond else ""))
    return cond


def build(w):
    fw = FW
    xd = os.path.join(XD, "xr871d.c")
    subprocess.check_call(["gcc", "-O2", "-Wall", "-Wextra", "-Werror", "-o", f"{w}/xr871d", xd])
    subprocess.check_call(["gcc", "-O2", "-Wall", "-Wno-unused-parameter", "-Werror",
                           "-o", f"{w}/xr871-host", f"{fw}/host/hal_host.c",
                           f"{fw}/core/bridge_core.c", f"{fw}/core/fw_update.c"])
    subprocess.check_call([sys.executable, "-I", os.path.join(HERE, "mkimages.py"), f"{w}/img"],
                          stdout=subprocess.DEVNULL, env=dict(os.environ, XR_IMAGE_TOOL=TOOL))


def wait_for(pred, t=10.0):
    end = time.time() + t
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


def read(p):
    try:
        return open(p, errors="replace").read()
    except OSError:
        return ""


class Rig:
    """socat pty <-> xr871-host, xr871d on the pty"""

    def __init__(self, w, name, flash, ram_app, secs=200, env_extra=None, slow=False):
        self.w, self.ctl = w, f"xr871d-test-{os.getpid()}-{name}"
        self.dir = os.path.join(w, name)
        os.makedirs(self.dir, exist_ok=True)
        self.flash = os.path.join(self.dir, "flash.bin")
        shutil.copy(flash, self.flash)
        sock, pty = os.path.join(self.dir, "xr.sock"), os.path.join(self.dir, "pty")
        if slow:    # 115200 baud both ways
            self.socat = subprocess.Popen([sys.executable, "-I", os.path.join(HERE, "slowlink.py"),
                                           sock, pty])
        else:
            self.socat = subprocess.Popen(["socat", f"UNIX-LISTEN:{sock}", f"PTY,link={pty},raw,echo=0"])
        wait_for(lambda: os.path.exists(sock), 5)
        env = dict(os.environ, XR_FLASH=self.flash, XR_RAM_APP=hex(ram_app), XR_NO_PIR="1")
        env.update(env_extra or {})
        self.hostlog = os.path.join(self.dir, "xr871.log")
        self.host = subprocess.Popen([f"{w}/xr871-host", sock, str(secs)], env=env,
                                     stdout=open(self.hostlog, "w"), stderr=subprocess.STDOUT)
        wait_for(lambda: os.path.exists(pty), 5)
        self.dlog = os.path.join(self.dir, "xr871d.log")
        self.xlog = os.path.join(self.dir, "xlog.txt")
        self.d = subprocess.Popen([f"{w}/xr871d", "-v", "-d", pty, "-l", self.xlog],
                                  env=dict(os.environ, XR871D_CTL=self.ctl),
                                  stderr=open(self.dlog, "w"))
        self.up = wait_for(lambda: "bridge firmware" in read(self.dlog), 10)

    def c(self, cmd, timeout=180):
        r = subprocess.run([f"{self.w}/xr871d", "-c", cmd], capture_output=True, text=True,
                           timeout=timeout, env=dict(os.environ, XR871D_CTL=self.ctl))
        print(f"    $ xr871d -c '{cmd}' -> rc {r.returncode}")
        for line in (r.stdout + r.stderr).splitlines():
            print(f"      {line}")
        return r.returncode, r.stdout + r.stderr

    def stop(self):
        for p in (self.d, self.host, self.socat):
            p.terminate()
        for p in (self.d, self.host, self.socat):
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()


def flash_diff(a, b):
    """list of (start, end) ranges where two images differ"""
    out, i = [], 0
    while i < len(a):
        if a[i] != b[i]:
            j = i
            while j < len(a) and a[j] != b[j]:
                j += 1
            if out and i - out[-1][1] < 0x100:
                out[-1] = (out[-1][0], j)
            else:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def only_in(diffs, ranges):
    return all(any(lo <= s and e <= hi for lo, hi in ranges) for s, e in diffs)


# ---------------------------------------------------------------------------

def test_stock(w):
    print("== 1. stock XR871 firmware model (tools/xr871-sim.py)")
    d = os.path.join(w, "stock")
    os.makedirs(d, exist_ok=True)
    a, b = os.path.join(d, "a"), os.path.join(d, "b")
    socat = subprocess.Popen(["socat", f"PTY,link={a},raw,echo=0", f"PTY,link={b},raw,echo=0"])
    wait_for(lambda: os.path.exists(a) and os.path.exists(b), 5)
    simlog = os.path.join(d, "sim.log")
    sim = subprocess.Popen([sys.executable, "-I", SIM, os.path.realpath(b), "12"],
                           stdout=open(simlog, "w"), stderr=subprocess.STDOUT)
    ctl = f"xr871d-test-{os.getpid()}-stock"
    dlog = os.path.join(d, "xr871d.log")
    xd = subprocess.Popen([f"{w}/xr871d", "-v", "-d", os.path.realpath(a), "-l", os.path.join(d, "x")],
                          env=dict(os.environ, XR871D_CTL=ctl), stderr=open(dlog, "w"))
    wait_for(lambda: "answered" in read(dlog), 8)
    time.sleep(1)
    outs = {}
    for cmd in ("status", "mcu 0x23 0", "gpio PA11 1", "fwupdate /nonexistent", "sleep 0"):
        r = subprocess.run([f"{w}/xr871d", "-c", cmd], capture_output=True, text=True, timeout=20,
                           env=dict(os.environ, XR871D_CTL=ctl))
        outs[cmd] = (r.returncode, r.stdout)
        print(f"    $ xr871d -c '{cmd}' -> rc {r.returncode}: {r.stdout.strip()}")
    sim.wait(20)
    xd.terminate()
    socat.terminate()
    xd.wait(5)
    log = read(simlog)
    check("stock: status refused locally", outs["status"][0] == 1 and "stock firmware" in outs["status"][1])
    check("stock: mcu/gpio/fwupdate refused locally",
          all(outs[c][0] == 1 and "stock firmware" in outs[c][1]
              for c in ("mcu 0x23 0", "gpio PA11 1", "fwupdate /nonexistent")))
    check("stock: old commands still answered (sleep -> ok)", outs["sleep 0"] == (0, "ok\n"))
    bad = [l for l in log.splitlines() if " cmd 0xd" in l]
    check("stock: no 0xd0..0xdc frame sent to the stock firmware", not bad, bad[:3])
    check("stock: heartbeat kept, no would-be power cycle", "would-be power cycles: 0" in log, log[-300:])


def test_e2e(w):
    print("== 2. xr871d <-> XR871 core, flash with OTA enabled")
    img = os.path.join(w, "img")
    r = Rig(w, "e2e", f"{img}/flash_ota.bin", 0x8000,
            env_extra={"XR_TEST_SECRET_LOG": "1"})
    try:
        check("e2e: bridge firmware detected", r.up, read(r.dlog)[-500:])
        time.sleep(1.5)
        rc, o = r.c("status")
        check("e2e status: rc 0", rc == 0, o)
        for want in (f"xr871 firmware: {CORE} host", "mcu firmware:   150",
                     "wake reason:    cold (0x004)", "battery:        77 %, 3950 mV",
                     "usb power:      present", "wifi:           associated, rssi -48 dBm",
                     "mcu state:      0x1e=0x01 0x1f=0xa5", "t30 power:      on, forwarding mask 0x03",
                     "running image ? (flash not probed"):
            check(f"e2e status shows '{want}'", want in o)
        rc, o = r.c("mcu 0x23 0")
        check("e2e mcu: query sent and MCU reply printed", rc == 0 and "mcu -> {0x23,0x96}" in o, o)
        # MCU sw 150 answers 0x15/0x18/0x1c, but not 0x26/0x29/0x2b/0x2c/0x2f
        rc, o = r.c("mcu 0x15 1")
        check("e2e mcu: 0x15 answered by the MCU model", rc == 0 and "mcu -> {0x15,0x01}" in o, o)
        rc, o = r.c("mcu 0x26 0")
        check("e2e mcu: unanswered query (0x26) is sent, reports no reply, rc 0",
              rc == 0 and "sent" in o and "{0x26" not in o.split("sent", 1)[1], o)
        rc, o = r.c("mcu 0x05 0x09")
        check("e2e mcu: T30-off message refused without force", rc == 3 and "force" in o, o)
        rc, o = r.c("mcu 0x05 0x09 force")
        check("e2e mcu: forced T30-off message sent", rc == 0 and "sent" in o, o)
        check("e2e mcu: reached the MCU", "sim-mcu <- {0x05,0x09}" in read(r.hostlog))
        rc, o = r.c("gpio PA11 1 2")
        check("e2e gpio: PA11 high", rc == 0 and "PA11 high for 2 s" in o, o)
        time.sleep(2.6)
        hl = read(r.hostlog)
        check("e2e gpio: LED on, then released by the XR871 after the hold time",
              "status LED (PA11) ON" in hl and "gpio: PA11 released" in hl)
        rc, o = r.c("gpio PA12 in")
        check("e2e gpio: read PA12 = 1", rc == 0 and "PA12 = 1" in o, o)
        for pin in ("PA16", "PA7", "PA17", "PA20", "PB3", "PA4", "PA9"):
            rc, o = r.c(f"gpio {pin} 1")
            check(f"e2e gpio: {pin} refused by the XR871", rc == 1 and "refused" in o, o)
        rc, o = r.c("gpio scan 1")
        check("e2e gpio scan: no default pin list (unknown roles), usage + rc 2",
              rc == 2 and "usage: gpio scan" in o and "blinking" not in o, o)
        rc, o = r.c("gpio scan 1 PA6 PA10 PA11 PA13 PA14 PA15")
        check("e2e gpio scan: 6 named pins blinked one by one",
              rc == 0 and all(f"PA{p:<2d}: blinking now" in o for p in (6, 10, 11, 13, 14, 15)), o)
        for cmd in ("sta", "scan", "stay"):
            rc, o = r.c(cmd)
            check(f"e2e: old command '{cmd}' still answered (ok, rc 0)", (rc, o) == (0, "ok\n"), o)
        rc, o = r.c("gpio scan 1 PA11 PA16")
        check("e2e gpio scan: explicit list, protected pin refused",
              "PA11: blinking now" in o and "PA16: refused" in o, o)
        rc, o = r.c("bogus")
        check("e2e: unknown command answered with rc 2", rc == 2, o)

        # log forwarding
        xl = read(r.xlog)
        check("e2e log: XR871 lines forwarded (boot + passthrough)",
              "abus-xr871 keepalive firmware" in xl and "mcu: passthrough {0x23,0x00}" in xl, xl[:300])
        check("e2e log: credential line withheld", "withheld" in xl and "S3cr3tValue" not in xl
              and "S3cr3tValue" not in read(r.dlog))
        check("e2e: MCU events logged by xr871d", "mcu event {0x0c,0x01}" in read(r.dlog))

        # firmware update
        orig = open(r.flash, "rb").read()
        rc, o = r.c(f"fwupdate {img}/bad_full.bin")
        check("e2e fwupdate: corrupt image refused by xr871d", rc == 1 and "bad data checksum" in o, o)
        check("e2e fwupdate: flash untouched after refusal", open(r.flash, "rb").read() == orig)
        rc, o = r.c("fwupdate --info")
        check("e2e fwupdate --info: running 1, target 2, OTA enabled",
              rc == 0 and "running image 1, target image 2, OTA enabled" in o, o)
        t0 = time.time()
        rc, o = r.c(f"fwupdate {img}/new_full.bin")
        dt = time.time() - t0
        check("e2e fwupdate: staged + verified", rc == 0 and "verified: crc" in o and "staged only" in o, o)
        new = open(f"{img}/new_full.bin", "rb").read()
        span = len(open(f"{img}/new_chain.bin", "rb").read())
        fl = open(r.flash, "rb").read()
        check("e2e fwupdate: image 2 area = new chain", fl[0x104000:0x104000 + span] == new[0x8000:0x8000 + span])
        dif = flash_diff(orig, fl)
        check("e2e fwupdate: nothing written outside image 2 window",
              only_in(dif, [(0x104000, 0x1F7000)]), [(hex(a), hex(b)) for a, b in dif[:4]])
        check("e2e fwupdate: bootloader model still boots image 1", xi.bootsel(fl)[0] == 1)
        print(f"    (transfer of {span} bytes took {dt:.1f} s on the pty)")
        rc, o = r.c("fwupdate --commit")
        fl2 = open(r.flash, "rb").read()
        check("e2e fwupdate --commit: committed", rc == 0 and "committed" in o, o)
        check("e2e commit: only the OTA config changed (4 + 1 bytes)",
              only_in(flash_diff(fl, fl2), [(0x100000, 0x104000)]) and
              sum(e - s for s, e in flash_diff(fl, fl2)) <= 0x200)
        check("e2e commit: bootloader model now boots image 2", xi.bootsel(fl2)[0] == 2, xi.bootsel(fl2))
        rc, o = r.c(f"fwupdate {img}/new_full.bin")
        check("e2e: new transfer refused after commit (reboot or abort first)",
              rc == 1 and "wrong state" in o, o)
        rc, o = r.c("fwupdate --abort")
        fl3 = open(r.flash, "rb").read()
        check("e2e fwupdate --abort after commit: reverted to image 1",
              rc == 0 and xi.bootsel(fl3)[0] == 1, xi.bootsel(fl3))
        rc, o = r.c("fwupdate --commit --reboot")
        fl4 = open(r.flash, "rb").read()
        check("e2e commit again + reboot request", rc == 0 and xi.bootsel(fl4)[0] == 2
              and "XR871 reboot requested" in read(r.hostlog), o)
        rc, o = r.c("status")
        check("e2e status after update: running image 1, OTA enabled, state committed",
              "fw update:      committed, running image 1, OTA enabled" in o, o)
    finally:
        r.stop()
    hl = read(r.hostlog)
    check("e2e: core reported no bad T30 frames", "bad 0 " in hl or "bad frames 0" in hl, hl[-400:])


def test_noota(w):
    print("== 3. flash with OTA disabled in the boot header (like m1/m2)")
    img = os.path.join(w, "img")
    r = Rig(w, "noota", f"{img}/flash_noota.bin", 0x8000)
    try:
        check("noota: bridge firmware detected", r.up)
        orig = open(r.flash, "rb").read()
        rc, o = r.c(f"fwupdate {img}/new_chain.bin")
        check("noota: update refused (OTA disabled)", rc == 1 and "OTA disabled" in o, o)
        check("noota: flash untouched", open(r.flash, "rb").read() == orig)
        rc, o = r.c(f"fwupdate {img}/new_chain.bin --force-stage")
        fl = open(r.flash, "rb").read()
        check("noota: --force-stage writes + verifies image 2", rc == 0 and "verified" in o, o)
        check("noota: only the image 2 window changed",
              only_in(flash_diff(orig, fl), [(0x104000, 0x1F7000)]))
        rc, o = r.c("fwupdate --commit")
        check("noota: commit refused", rc == 1 and "OTA disabled" in o, o)
        check("noota: bootloader model boots image 1", xi.bootsel(open(r.flash, "rb").read())[0] == 1)
    finally:
        r.stop()


def test_boot_usb(w):
    print("== 1b. merge: USB state known from the boot answer {0x1e}")
    img = os.path.join(w, "img")
    t = T30(w, "bootusb", f"{img}/flash_ota.bin", 0x8000)
    try:
        t.send(0x00)
        hello = t.poll(0.3)
        check("boot: hello answered like stock (0x85 wake 4, 0x40, 0x80)",
              [f[0] for f in hello[:3]] == [0x85, 0x40, 0x80] and hello[0][2] == b"\x04\0\0\0", hello)
        f = t.req(0xD0)     # ~0.4 s: before the simulated {0x0c,1} event at 1 s
        p = f[2] if f else bytes(152)
        check("boot: status has USB = present from {0x1e} before any 0x0c event",
              f and p[84] & 5 == 5 and p[82] == 1 and p[80] == 1 and
              "event 0x0c" not in read(t.log), (p[80:88].hex(), read(t.log)[-200:]))
        check("boot: status version string = core + glue", f and
              p[88:].rstrip(b"\0").decode() == CORE + " host", p[88:])
        f = t.req(0xC8)     # current command set still dispatched (no WiFi worker on the host)
        check("boot: WiFi scan request answered (refused: host has no wifi_mode hook)",
              f and f[1] == 0xFFFF, f)
        f = t.req(0xD4, 0)
        check("boot: forwarding stays off unless asked (mask 0)", f and f[1] == 0)
        n = len([x for x in t.poll(1.5) if x[0] in (0xD2, 0xD4)])
        check("boot: no 0xd2/0xd4 frames without opt-in (also not for the USB event)",
              n == 0 and "event 0x0c val 0x01" in read(t.log), n)
    finally:
        t.stop()


# ---- raw protocol ----------------------------------------------------------

def frame(cmd, param=0, payload=b""):
    return b"\xaa\x55\xaa\x55" + struct.pack("<HBBHH", len(payload), cmd, 1, 0, param) + payload


class T30:
    """python as the T30 side, xr871-host connects to our unix socket"""

    def __init__(self, w, name, flash, ram_app, env_extra=None):
        ram_app = ram_app if isinstance(ram_app, str) else hex(ram_app)
        self.dir = os.path.join(w, name)
        os.makedirs(self.dir, exist_ok=True)
        self.flash = os.path.join(self.dir, "flash.bin")
        shutil.copy(flash, self.flash)
        path = os.path.join(self.dir, "t30.sock")
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(path)
        srv.listen(1)
        env = dict(os.environ, XR_FLASH=self.flash, XR_RAM_APP=ram_app, XR_NO_PIR="1")
        env.update(env_extra or {})
        self.log = os.path.join(self.dir, "xr871.log")
        self.host = subprocess.Popen([f"{w}/xr871-host", path, "120"], env=env,
                                     stdout=open(self.log, "w"), stderr=subprocess.STDOUT)
        srv.settimeout(5)
        self.s, _ = srv.accept()
        self.s.settimeout(0.05)
        self.buf = b""
        self.frames = []    # everything received, for the leak checks

    def send(self, cmd, param=0, payload=b""):
        self.s.sendall(frame(cmd, param, payload))

    def poll(self, t):
        end = time.time() + t
        got = []
        while time.time() < end:
            try:
                self.buf += self.s.recv(65536)
            except socket.timeout:
                pass
            while True:
                i = self.buf.find(b"\xaa\x55\xaa\x55")
                if i < 0 or len(self.buf) - i < 12:
                    break
                ln, cmd, _, _, param = struct.unpack_from("<HBBHH", self.buf, i + 4)
                if len(self.buf) - i < 12 + ln:
                    break
                f = (cmd, param, self.buf[i + 12:i + 12 + ln], time.time())
                self.buf = self.buf[i + 12 + ln:]
                got.append(f)
                self.frames.append(f)
        return got

    def req(self, cmd, param=0, payload=b"", t=5.0):
        self.send(cmd, param, payload)
        end = time.time() + t
        while time.time() < end:
            for f in self.poll(0.05):
                if f[0] == cmd and not (cmd == 0xD4 and f[2]):
                    return f
        return None

    def stop(self):
        self.host.terminate()
        self.host.wait(5)


def begin_payload(span_len, crc, flags=0, off=0x8000):
    return struct.pack("<IIIIHBBI", 0x31555746, off, span_len, crc, 232, flags, 0, 0)


def test_raw(w):
    print("== 4. raw protocol against the XR871 core")
    img = os.path.join(w, "img")
    t = T30(w, "raw", f"{img}/flash_ota.bin", 0x8000, {"XR_TEST_SECRET_LOG": "1"})
    try:
        t.send(0x00)
        t.poll(0.3)
        f = t.req(0xD4, 3)
        check("raw: forwarding enabled (ack param 3)", f and f[1] == 3)
        # MCU safety in the firmware itself
        f = t.req(0xD1, 0x05 | 0x09 << 8, b"\x00")
        check("raw mcu: {5,9} refused by the XR871 without force", f and f[1] == 0xFFFE)
        f = t.req(0xD1, 0x24 | 0x01 << 8, b"\x00")
        check("raw mcu: {0x24,..} refused by the XR871 without force", f and f[1] == 0xFFFE)
        f = t.req(0xD1, 0x05 | 0x07 << 8, b"\x00")
        check("raw mcu: {5,7} allowed", f and f[1] == 0)
        # GPIO protection in the firmware itself
        prot = [(0, p) for p in (4, 5, 7, 8, 9, 16, 17, 18, 19, 20, 21, 22, 23, 31)] + \
               [(1, p) for p in range(8)]
        bad = []
        for port, pin in prot:
            for mode in (0, 1, 2, 3, 4):
                f = t.req(0xD3, 0, bytes([port, pin, mode, 0x10, 0x27, 0, 0]))
                if not f or f[1] != 0xFFFE:
                    bad.append((port, pin, mode))
        check("raw gpio: every protected pin refused in every mode", not bad, bad[:5])
        f = t.req(0xD3, 0, bytes([0, 11, 9, 0, 0, 0, 0]))
        check("raw gpio: invalid mode refused", f and f[1] == 0xFFFE)
        f = t.req(0xD3, 0, bytes([0, 6, 3, 0xF4, 0x01, 50, 0]))   # blink 500 ms
        t.poll(0.8)
        check("raw gpio: blink accepted and auto-released",
              f and f[1] == 0 and "PA6 released" in read(t.log))
        # credentials: never in any frame
        ssid, psk = b"TestNet-QX7", b"Pw-Only-For-Test-123"
        p = bytes([len(ssid)]) + ssid.ljust(32, b"\0") + bytes([len(psk)]) + psk.ljust(64, b"\0")
        f = t.req(0xC0, 0, p)
        check("raw wifi config accepted", f and f[1] == 1)
        f = t.req(0xD0)
        check("raw status reply (152 bytes, version 1)", f and len(f[2]) == 152 and f[2][0] == 1)
        # log flood: rate limit + drop notice
        for i in range(150):
            t.send(0x77, i)
        t.poll(1.0)
        lines = [x for x in t.frames if x[0] == 0xD4 and x[2]]
        t_flood = [x for x in lines if b"cmd 0x77" in x[2]]
        t.poll(12.0)
        lines = [x for x in t.frames if x[0] == 0xD4 and x[2]]
        texts = [x[2].decode(errors="replace") for x in lines]
        check("raw log: rate limited (<= 30 lines in the first second of the flood)",
              len(t_flood) <= 30, len(t_flood))
        check("raw log: overflow reported as dropped lines",
              any("log lines dropped" in x for x in texts), texts[-3:])
        check("raw log: credential keyword line withheld",
              any("withheld" in x for x in texts) and not any("S3cr3tValue" in x for x in texts))
        allbytes = b"".join(x[2] for x in t.frames)
        check("raw: SSID / PSK never sent back to the T30", ssid not in allbytes and psk not in allbytes)
        check("raw: SSID / PSK never in the XR871 log",
              ssid.decode() not in read(t.log) and psk.decode() not in read(t.log))
        seqs = [x[1] for x in lines]
        check("raw log: sequence numbers increase by one", all(b == (a + 1) & 0xFFFF for a, b in zip(seqs, seqs[1:])))

        # update state machine
        full = open(f"{img}/new_full.bin", "rb").read()
        chain = open(f"{img}/new_chain.bin", "rb").read()
        crc = zlib.crc32(chain)
        orig = open(t.flash, "rb").read()
        f = t.req(0xD9, 0, struct.pack("<II", 0, 0) + b"x")
        check("raw fwu: DATA before BEGIN -> wrong state", f and f[1] == 1)
        f = t.req(0xD8, 0, begin_payload(len(chain), crc, off=0x9000))
        check("raw fwu: BEGIN with wrong span offset -> bad request", f and f[1] == 2)
        f = t.req(0xD8, 0, begin_payload(0xF3001, crc))
        check("raw fwu: BEGIN too big -> too big", f and f[1] == 4)
        f = t.req(0xD8, 0, begin_payload(len(chain), crc))
        check("raw fwu: BEGIN ok, running 1 -> target 2 at 0x104000",
              f and f[1] == 0 and f[2][1] == 0 and f[2][2] == 1 and struct.unpack_from("<I", f[2], 4)[0] == 0x104000)

        def data(off, n, crc_ok=True):
            d = chain[off:off + n]
            c = zlib.crc32(d) ^ (0 if crc_ok else 1)
            return t.req(0xD9, 0, struct.pack("<II", off, c) + d)
        f = data(0, 232, crc_ok=False)
        check("raw fwu: bad chunk crc -> crc error", f and f[1] == 5)
        f = data(232, 232)
        check("raw fwu: out of order -> offset error", f and f[1] == 6)
        f = t.req(0xD9, 0, struct.pack("<II", 0, 0) + bytes(233))
        check("raw fwu: chunk > 232 bytes -> bad request", f and f[1] == 2)
        off = 0
        while off < len(chain) // 2:
            n = min(232, len(chain) - off)
            f = data(off, n)
            if not f or f[1]:
                break
            off = struct.unpack("<I", f[2])[0]
        check("raw fwu: first half written", off >= len(chain) // 2, f)
        f = data(off - 232, 232)
        check("raw fwu: repeated (already acked) chunk accepted", f and f[1] == 0 and
              struct.unpack("<I", f[2])[0] == off)
        f = t.req(0xD8, 0, begin_payload(len(chain), crc, flags=2))
        check("raw fwu: BEGIN with resume continues at the same offset",
              f and f[1] == 0 and struct.unpack_from("<I", f[2], 8)[0] == off)
        f = t.req(0xDA)
        check("raw fwu: VERIFY before complete -> wrong state", f and f[1] == 1)
        while off < len(chain):
            fill = 0
            while off + fill < len(chain) and chain[off + fill] == 0xFF and fill < 16384:
                fill += 1
            if fill >= 64:
                f = t.req(0xD9, 1, struct.pack("<II", off, fill))
            else:
                f = data(off, min(232, len(chain) - off))
            if not f or f[1]:
                break
            off = struct.unpack("<I", f[2])[0]
        check("raw fwu: second half written (with 0xff fill frames)", off == len(chain), f)
        f = t.req(0xDB, 0xC0FF, struct.pack("<I", crc))
        check("raw fwu: COMMIT before VERIFY -> not verified", f and f[1] == 10)
        f = t.req(0xDA, t=10)
        check("raw fwu: VERIFY ok, 3 sections", f and f[1] == 0 and f[2][4] == 3 and
              struct.unpack_from("<I", f[2])[0] == crc)
        f = t.req(0xDB, 0x1234, struct.pack("<I", crc))
        check("raw fwu: COMMIT without magic refused", f and f[1] == 2)
        f = t.req(0xDB, 0xC0FF, struct.pack("<I", crc ^ 1))
        check("raw fwu: COMMIT with other crc refused", f and f[1] == 10)
        f = t.req(0xDC, 2)
        check("raw fwu: reboot before commit refused", f and f[1] == 1)
        f = t.req(0xDB, 0xC0FF, struct.pack("<I", crc), t=10)
        fl = open(t.flash, "rb").read()
        check("raw fwu: COMMIT ok -> bootloader model boots image 2", f and f[1] == 0 and xi.bootsel(fl)[0] == 2)
        check("raw fwu: image 2 = new chain", fl[0x104000:0x104000 + len(chain)] == chain)
        check("raw fwu: changes only in image 2 window + OTA config",
              only_in(flash_diff(orig, fl), [(0x104000, 0x1F7000), (0x100000, 0x104000)]))
        f = t.req(0xD8, 0, begin_payload(len(chain), crc))
        check("raw fwu: BEGIN after COMMIT refused", f and f[1] == 1)
        f = t.req(0xDC, 1)
        check("raw fwu: abort after commit reverts to image 1",
              f and f[1] == 0 and xi.bootsel(open(t.flash, "rb").read())[0] == 1)
    finally:
        t.stop()

    # a chain whose crc matches but whose section checksum is wrong
    t = T30(w, "raw-badsum", f"{img}/flash_ota.bin", 0x8000)
    try:
        t.send(0x00)
        t.poll(0.2)
        chain = open(f"{img}/new_chain.bin", "rb").read()
        bad = bytearray(open(f"{img}/bad_full.bin", "rb").read()[0x8000:0x8000 + len(chain)])
        crc = zlib.crc32(bad)
        t.req(0xD8, 0, begin_payload(len(bad), crc))
        off = 0
        while off < len(bad):
            d = bytes(bad[off:off + 232])
            f = t.req(0xD9, 0, struct.pack("<II", off, zlib.crc32(d)) + d)
            off = struct.unpack("<I", f[2])[0]
        f = t.req(0xDA, t=10)
        check("raw fwu: section checksum error found by the XR871 verify", f and f[1] == 9)
        f = t.req(0xDB, 0xC0FF, struct.pack("<I", crc))
        check("raw fwu: and commit refused afterwards", f and f[1] == 10)
        check("raw fwu: config untouched", xi.bootsel(open(t.flash, "rb").read())[0] == 1)
    finally:
        t.stop()

    # running slot from RAM: config says image 1, code in RAM is image 2
    t = T30(w, "raw-ram2", f"{img}/flash_ota.bin", 0x104000)
    try:
        t.send(0x00)
        t.poll(0.2)
        f = t.req(0xDC, 0)
        check("raw fwu: RAM says image 2 (bootloader fallback) -> target image 1",
              f and f[1] == 0 and f[2][1] == 1 and f[2][2] == 0, f)
    finally:
        t.stop()
    t = T30(w, "raw-ramx", f"{img}/flash_ota.bin", "other")
    try:
        t.send(0x00)
        t.poll(0.2)
        f = t.req(0xDC, 0)
        check("raw fwu: RAM matches no slot -> refused (cannot tell which image runs)",
              f and f[1] == 12, f)
    finally:
        t.stop()


def test_slow(w):
    print("== 5. 115200 baud link + W25Q16 flash timings")
    img = os.path.join(w, "img")
    r = Rig(w, "slow", f"{img}/flash_ota.bin", 0x8000, slow=True,
            env_extra={"XR_FLASH_SLOW": "1"})
    try:
        check("slow: bridge firmware detected", r.up)
        time.sleep(1)
        rc, o = r.c("status")
        check("slow: status ok", rc == 0 and "battery:        77 %" in o, o)
        t0 = time.time()
        rc, o = r.c(f"fwupdate {img}/new_chain.bin --commit")
        dt = time.time() - t0
        check("slow: fwupdate + commit at UART speed", rc == 0 and "committed" in o, o[-400:])
        check("slow: no retries needed", "retrying" not in o)
        print(f"    (246568 bytes at 115200 baud with flash timings: {dt:.1f} s,"
              f" about {246568 / dt / 1024:.1f} KiB/s)")
        check("slow: bootloader model boots image 2", xi.bootsel(open(r.flash, "rb").read())[0] == 2)
        time.sleep(3)
        rc, o = r.c("status")
        check("slow: link still clean after the transfer (t30 bad 0)", rc == 0 and
              any(l.split()[:2] == ["t30", "frames"] and l.split()[5] == "0" for l in o.splitlines()), o)
        xl = read(r.xlog)
        check("slow: log lines queued during the transfer arrive afterwards", "fwu: image 2 verified" in xl
              or "log lines dropped" in xl, xl[-300:])
    finally:
        r.stop()


def main():
    w = os.path.join(OUT, "t-" + time.strftime("%m%d-%H%M%S"))
    os.makedirs(w)
    print(f"firmware {FW}\nxr871d   {XD}\nimage.py {TOOL}")
    build(w)
    test_stock(w)
    test_boot_usb(w)
    test_e2e(w)
    test_noota(w)
    test_raw(w)
    test_slow(w)
    n_ok = sum(1 for _, ok in results if ok)
    print(f"\n{n_ok}/{len(results)} checks passed, run dir {w}")
    print("ALL PASS" if n_ok == len(results) else "FAILURES: " +
          ", ".join(n for n, ok in results if not ok))
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
