# XR871 bridge: service extensions (status, MCU passthrough, GPIO test, log forwarding, firmware update)

Work copy, 2026-11-13. Nothing here was built with the XR871 SDK, flashed or
run on the camera. Everything was tested on the host only (see "Tested").

## Files

| Path | What |
|---|---|
| `xr871fw/core/bridge_core.[ch]` | core: new commands 0xd0..0xd4, log queue, GPIO test, MCU event forwarding, new optional HAL hooks |
| `xr871fw/core/fw_update.[ch]` | NEW, portable: application update into the inactive image slot, boot-config switch |
| `xr871fw/xr871/hal_ext.[ch]` | NEW, XR871 glue for the hooks: AXP2601 battery read, GPIO, flash, log hook, reboot |
| `xr871fw/xr871/main.c` | 3 small changes (see "Merging") |
| `xr871fw/xr871/spi_slave.[ch]` | `spis_ready_pin()` getter (GPIO test refuses the ready pin) |
| `xr871fw/xr871/wifi.c` | RSSI from `wlan_sta_ap_info()` (`level`, dBm), at most every 2 s |
| `xr871fw/host/hal_host.c` | host model: battery, USB event, MCU echo, LED on PA11, NOR flash file, RAM model, slow flash |
| `xr871-bridge/files/xr871d.c` | v0.4: control socket answers the caller, new commands, syslog |
| `xr871-bridge/files/xr871d_ext.c` | NEW, `#include`d at the end of xr871d.c (same compile line, no .mk change) |
| `tools/xr871-image.py` | `build` now keeps the boot header OTA fields; new `set-ota` and `bootsel` |
| `tests/run_tests.py` | host test suite (107 checks), `tests/mkimages.py` synthetic flash images |
| `tests/slowlink.py` | 115200 baud relay for the timing test |
| `tests/host-sleep-test.sh`, `tests/test_xr871d_pty.py` | the old tests, pointed at these copies |
| `patches/` | `main.c.features.patch` (applies to the current main.c), `xr871fw-rest.patch`, `xr871-bridge.patch` |
| `build/xr871d.mips` | MIPS build of xr871d (Thingino toolchain, `-Os -Wall -Wextra`, no warnings) |

## Protocol additions (UART, T30 <-> XR871)

Frame format unchanged (magic, u16 len @4, cmd @6, 0x01 @7, u16 param @10,
payload @12, max payload 0xf4). Every request is answered with the same
command number. Free numbers 0xd0..0xdc were chosen (not used by the stock
T30 application, the stock XR871 firmware, or our 0xc0/0xc4/0xc5/0xf1).

| cmd | dir | param | payload | reply |
|---|---|---|---|---|
| 0xd0 status | T30->XR | - | - | param 1, 152-byte status record (layout in `bridge_core.h`) |
| 0xd1 MCU message | T30->XR | `cmd \| val<<8` | u8 flags (bit0 force) | param 0 ok, 0xfffe refused, 0xffff I2C error |
| 0xd2 MCU event | XR->T30 | `cmd \| val<<8` | u8 dir (0 = from MCU) | - |
| 0xd3 GPIO test | T30->XR | - | u8 port (0=A,1=B), u8 pin, u8 mode (0 low, 1 high, 2 read, 3 blink, 4 release), u16 hold/blink ms (0 = 30 s), u16 half period ms (0 = 250) | param = level (read) / 0, 0xfffe refused, 0xffff failed |
| 0xd4 forwarding | T30->XR | bit0 log lines, bit1 MCU events | - | param = mask now active |
| 0xd4 log line | XR->T30 | sequence number | text, no newline, <= 200 bytes | - |
| 0xd8 FWU begin | T30->XR | - | u32 "FWU1", u32 span offset (=0x8000), u32 span len, u32 crc32, u16 chunk, u8 flags (bit0 force-stage, bit1 resume), u8 0, u32 0 | status; info record |
| 0xd9 FWU data | T30->XR | bit0 = 0xff fill | u32 offset, u32 crc32, data <= 232 / fill: u32 offset, u32 length | status; u32 next offset |
| 0xda FWU verify | T30->XR | - | - | status; u32 crc, u8 sections, 3x0, u32 bad address |
| 0xdb FWU commit | T30->XR | 0xc0ff | u32 span crc32 | status; info record |
| 0xdc FWU info | T30->XR | 0 info, 1 abort (after commit: revert), 2 reboot (only after commit) | - | status; info record |

Info record (24 bytes): u8 state (0 idle, 1 receiving, 2 received, 3
verified, 4 committed), u8 running image (0/1, 0xff unknown), u8 target, u8
OTA enabled, u32 target window start, u32 next offset, u32 span length, u32
span crc, u32 target window end.
FWU status codes: `fw_update.h` (`FWU_E_*`), printed as text by xr871d.

Stock compatibility: the XR871 only sends 0xd2/0xd4 after the T30 enabled
them with 0xd4 (reset on every hello and every T30 power cycle). The stock T30
application never sends 0xd4, so it never sees these frames. xr871d only sends
0xd0..0xdc after our firmware answered the 0xf1 marker; with the stock XR871
firmware all new `-c` commands answer "stock firmware: command not available"
(tested against `tools/xr871-sim.py`: no 0xd0..0xdc frame reaches it).

## CLI (Thingino)

The `-c` client now waits for the daemon's answer and prints it (exit code =
result; old commands answer `ok`).

```
xr871d -c status
xr871d -c 'mcu 0x23 0'                 # raw {cmd,val} to the STM8, prints replies for 1 s
xr871d -c 'mcu 0x05 0x09 force'        # T30-off ({5,!7}) and 0x24 need "force"
xr871d -c 'gpio PA11 1 10'             # high for 10 s (max 65), then released by the XR871
xr871d -c 'gpio PA11 blink 30' | 'gpio PA12 in' | 'gpio PA11 off'
xr871d -c 'gpio scan'                  # PA6/10/11/13/14/15 (minus ready pin), 3 s blink each
xr871d -c 'gpio scan 5 PA0 PA1 PA2'    # own list / time
xr871d -c 'fwupdate --info'            # probes the XR871 flash layout (read only)
xr871d -c 'fwupdate /tmp/xr871-abus-m2.bin'          # stage + verify only
xr871d -c 'fwupdate --commit'                        # switch boot config to it
xr871d -c 'fwupdate /tmp/new.bin --commit --reboot'  # all in one
xr871d -c 'fwupdate --abort'           # before commit: forget; after commit: revert config
```

XR871 log lines go to syslog with the tag `xr871` (to /dev/kmsg as
`xr871: ...` while /dev/log does not exist yet); `xr871d -l file` writes them
to a file instead (used by the tests). MCU events appear in the xr871d log as
`mcu event {0x0d,0x01}`.

Status example (host model):
```
xr871 firmware: abus-xr871 core 3 host          (on the XR871: "... m2 <date> <time>")
mcu firmware:   150
xr871 uptime:   1 s
wake reason:    cold (0x004)
battery:        77 %, 3950 mV
usb power:      present (MCU event 0x0c val 0x01)
mcu state:      0x1e=0xa5 0x1f=0xa5
wifi:           associated, rssi -48 dBm
t30 power:      on, forwarding mask 0x03
fw update:      idle, running image ? (flash not probed; 'fwupdate --info'), OTA ?
t30 frames 8  t30 bad 0  t30 hb 1  mcu rx 4 ... (16 counters)
xr871d:         v0.4, rx 22 bad 0 hb sent 1, xr871 log lines 15, mcu events 1
```

## Firmware update: how the image is chosen at boot, and what is verified

### Stock bootloader (checked statically)

The bootloader at flash 0 is byte-identical in the stock dump and in m1/m2
(body md5 72c8a9a7...). Its disassembly (scratchpad `sboot.dis`) matches XR871
SDK `project/bootloader/main.c` + `src/image/image.c`:

* `image_init(flash 0, addr 0, max 0xff000)` (call at 0x6726a, code at
  0x6a19c) reads the bootloader's own AWIH header: `next` = bootloader size
  (0x8000), `priv[0]` @0x28 = OTA flash (0xff = same) | OTA size << 8,
  `priv[1]` @0x2c = OTA address. Image 2 base = OTA addr + OTA size - 0x8000.
* 0x67286: OTA address 0xffffffff -> "OTA disabled": always image 1, no
  fallback, config never read.
* Otherwise `image_get_cfg` (0x6a418/0x6a48c): last fdcm record at the OTA area;
  0x5555 = image 1, 0xaaaa = image 2, 0x9669 verified / 0x6996 unverified.
  Not verified or unreadable -> image 1 first.
* It loads the APP section of that image (header + data checksum, load range);
  if that fails it tries the other image and, if that one loads, writes the
  config to it (0x672e6..0x672f6, `image_set_cfg`). Only the APP section is
  checked; a broken net/WLAN section or a firmware that crashes is NOT a reason
  to fall back.

`tools/xr871-image.py bootsel <flash.bin>` models exactly this.

### Finding: m1/m2 have OTA disabled

```
work/xr871.bin (stock):        OTA area 0x100000/0x4000, config 0xaaaa/0x9669 -> boots image 2
flash/xr871-abus-m1.bin, -m2:  OTA disabled in the boot header -> image 1 only, no fallback
```
The old `xr871-image.py build` wrote 0xff into the boot header's `priv`
fields. So on the flash currently on the camera the bootloader never looks at
image 2 (the "stock image 2 fallback" in flash/README.md does not exist) and
the OTA config written for m1 has no effect. Consequences:

* An A/B update is impossible on this flash without rewriting sector 0 (the
  bootloader header), which this code refuses to do at runtime (power cut
  during that erase = hard brick).
* `fwupdate` refuses (`OTA disabled in the XR871 boot header`). With
  `--force-stage` it writes and verifies image 2 anyway (transfer test only;
  overwrites the unused stock image 2 copy, which is also in the PC dumps);
  `--commit` stays refused.
* Fix for the next clip flash: build with this `tools/xr871-image.py` (keeps
  the priv fields of the base image's boot header) on a stock-based base, or
  run `xr871-image.py set-ota <in> <out>` (default 0x100000 / 0x4000, as
  stock) on the finished image. On m2 this changes 9 bytes; `bootsel` then
  says: config 0x5555/0x9669 -> image 1, with the stock image 2 as a real
  (APP-checksum-only) fallback. Not done here (flash/ is off limits).

### Update procedure (XR871 side, `fw_update.c`)

* No flash access at boot. The layout is probed on the first `fwupdate`
  request (`fwupdate --info` or the first transfer). In m1 even
  `HAL_Flash_Init` only happens then.
* Running image = last verified config record; cross-checked against the
  code in RAM (1 KiB at APP body +0x400, `__text_start__..__text_end__`): if
  RAM matches the other slot (fallback whose config write failed) that one
  counts as running; if it matches neither, refused.
* Target = the other slot. Writable window = target slot offsets
  0x8000..0xfb000 (0x104000..0x1f7000 for image 2, 0x8000..0xfb000 for image
  1). Must not overlap bootloader, running slot, OTA area, sysinfo
  (0xfb000), device info (0x1fe000..). Every erase/write is checked against it.
* Chunks are sequential (stop-and-wait), crc32 per chunk, sectors erased on
  first touch, every chunk read back and compared. A repeated chunk (lost
  reply) is accepted if the flash already holds it. 0xff runs are sent as
  "fill" (erase + check).
* Restartable: the state lives in XR871 RAM. BEGIN with the same length +
  crc resumes at the next offset (xr871d always asks to resume, so re-running
  the same command after an interruption continues). XR871 reset -> start over.
* Verify: crc32 of the whole span read back from flash + the AWIH chain of
  the target slot (header sum, data sum, first section = APP, links forward
  only, all inside the span), the bootloader's rules.
* Commit (only with OTA enabled, only after a good verify, verify repeated
  right before): append {target, verified} to the fdcm config: data slot
  first, read back, then the bitmap bit that makes it current, read back,
  re-read the config. A power cut before the bitmap byte keeps the old
  config. A full config area (needs an erase) is refused, not erased.
* `--abort` after commit appends the old image again (revert). New transfers
  after a commit are refused until reboot/abort. Reboot only on request
  (`--reboot`, watchdog reset; this cuts the T30 power).
* xr871d checks the file first (AWIH sums, chain inside 0x8000..0xfb000,
  bootloader size 0x8000; the bootloader in a full image is never sent).

Verified statically: bootloader decision logic and header/config field
offsets (disassembly, see above); record values against the stock dump.
Assumed: stock `fdcm_read` = SDK source (bit counting; only the cfg reader's
call sites were checked, not fdcm_read's body); the app-side `image_init` in
a slot-2 app finds its sections the same way (SDK source); HAL_Flash_* work
from the app on this board (never run on hardware, m1 never initialised the
flash driver); RAM compare symbols. Not verified on hardware: everything.

Transfer time at 115200 baud (host model with W25Q16 timings): 246 KB in
20.6 s (~11.7 KiB/s), so m2 (~500 KB) about 45 s.

## Other features: notes

* Status reads the AXP2601 fuel gauge only when asked: I2C1 (shared with the
  MCU, same lock) address 0x62, SOC register 0x08 (0..100), VBAT 0x04/0x05
  big endian mV (plausible 2000..4500). Derived from the stock app
  (`axp260x_i2c_read` at 0x1156c: I2C1, addr 0x62, 1-byte reg; register
  table at 0x30bb0; `GetBatLevel` uses entry 6 = 0x08, `GetVbatAndTemp`
  entry 4 = 0x04, 2 bytes, median of 3). Read-only; never writes the PMIC
  (the stock firmware does model updates there). Unverified on hardware.
* USB present = last MCU event 0x0c value != 0 (stock `CMU_ProcessUsbEvent`
  prints "USB INSERT" for val != 0, "USB OUT" for 0). Unknown until the MCU
  sends one after boot.
* MCU passthrough: the XR871 refuses {0x05, != 0x07} (T30 off) and 0x24
  (low-power notice) without the force flag; xr871d asks for `force` too.
  Replies come as normal MCU messages via the IRQ line, so everything the
  MCU sends within 1 s is printed (can include a heartbeat).
* GPIO test: the XR871 core refuses port B entirely (PB0/1 console, PB2-7
  flash), PA4/5 (I2C0), PA7 (MCU IRQ), PA8/9 (I2C1: MCU + PMIC), PA16 (T30
  power), PA17/18 (UART1 = T30 link), PA19-22 (SPI1), pins > PA22; the glue
  refuses them again plus the SPI ready pin in use (m2). One test pin at a
  time; levels are released after the hold time and when the T30 is switched
  off. Release = back to low output for PA14/PA15 (as main.c sets them),
  else `HAL_GPIO_DeInit`. The scan default is the pins the stock firmware
  drives as outputs; PA0-3/PA12 are allowed but of unknown role (could be
  inputs driven by another chip: short hold times only).
* Log forwarding: every line through the core's log hook (all `H->log`
  output, incl. the fwu and gpio messages) goes into a 2 KiB queue (newest
  dropped when full, reported as "[xr871: N log lines dropped]"). Sent at
  most 10 lines/s, one per poll, only while the T30 is on and has enabled
  it, never during an update transfer or while a T30 frame is half received.
  Lines containing psk / passw / passphr / pmk / secret are replaced (both
  on the XR871 and in xr871d). The core never logs credentials (only
  lengths). SDK `printf` output (WLAN stack, boot markers) is not forwarded.

## Tested (host only)

`taskset -c 4-15 nice -n 19 python3 -I tests/run_tests.py` -> **107/107 PASS**
(run dir `build/t-182623`, log `build/run_tests.log`), plus `tests/host-sleep-test.sh` (HOST SLEEP PASS),
`tools/xr871d-parser-test.py build/.../xr871d` (PARSER PASS),
`tests/test_xr871d_pty.py` (old pty test, unchanged behaviour), MIPS build
of xr871d without warnings, ARM (`arm-none-eabi-gcc -mcpu=cortex-m4`) compile
of the core, and `-fsyntax-only` of main.c / hal_ext.c / wifi.c (m1 and m2
variants) against the SDK headers (no SDK build, no object files there).

The suite covers: stock-firmware compatibility; status fields; MCU query +
reply, refusal and force; GPIO high/auto-release/read, every protected pin
in every mode refused by the XR871 itself, scan; log forwarding, credential
filter, SSID/PSK never in any frame or log, rate limit, drop notice,
sequence numbers; update: corrupt file refused, stage/verify, nothing
written outside the window, commit (only the config changes, bootloader model
-> image 2), begin-after-commit refused, abort = revert, reboot request,
OTA-disabled refusal and force-stage, bad chunk crc, out of order, oversize,
resume, duplicate chunk, verify before complete, commit before verify / wrong
magic / wrong crc, section checksum error found by the XR871 verify,
running-slot detection from RAM (fallback case and "matches nothing"),
115200-baud link with flash timings (no retries, link clean afterwards).

## Untested / assumed / open risks

* Nothing ran on the XR871. SDK build not done (by rule). The m2 boot
  problem (eFuse / power trim) is unrelated but blocks any hardware test.
* OTA is disabled on the flashed m1/m2 -> no A/B update possible until a
  clip flash with OTA re-enabled (see above).
* The bootloader's fallback only covers a bad APP section checksum, not a
  firmware that boots and then fails (no trial boot in this bootloader).
  Recovery for that case stays the clip.
* UART RX on the XR871 is only interrupt-driven while `t30_task` waits in
  `HAL_UART_Receive_IT`; while the core transmits, T30 bytes pile up in the
  hardware FIFO. The new code keeps transmits short (one log line per poll,
  none during transfers), but a long reply racing a T30 frame could still
  lose bytes (xr871d retries update chunks; other commands just time out).
  A ring-buffered RX in main.c would remove this (not done: main.c is busy).
* Thread stacks: t30/mcu tasks have 2 KiB; the new paths add ~300-400 bytes
  of depth (core frames measured with `-fstack-usage`: bridge_t30_rx 304,
  fwu chain ~380, plus `vsnprintf` in the log hook). Recommended: 4 KiB.
* AXP2601 registers, RSSI field (`level`), lazily initialised flash driver
  in m1, `HAL_WDG_Reboot` from the app: from the stock app / SDK, not seen
  working.
* GPIO: PA0-3/PA12 are allowed but unknown; driving a pin that another chip
  drives can cause contention while the hold time runs.

## Merging

1. `xr871fw/xr871/main.c`, three changes (`patches/main.c.features.patch`,
   `patch --dry-run` OK against the current main.c with the boot diagnostics):
   * `#include "hal_ext.h"` after `#include "core/bridge_core.h"`
   * both `static const struct bridge_hal hal = {` -> `static struct bridge_hal hal = {`
   * `hal_ext_fill(&hal, &i2c_lock);` right before
     `OS_MutexLock(&core_lock, ...); bridge_init(&hal, BR_WAKE_COLD);`
   * recommended, by hand: thread stacks of "mcu" and "t30" `2 * 1024` -> `4 * 1024`
2. New files: `xr871fw/core/fw_update.[ch]` (picked up through the `core`
   symlink), `xr871fw/xr871/hal_ext.[ch]` (needs symlinks
   `project/abus_xr871/hal_ext.c` and `project/abus_xr871_net/hal_ext.[ch]`
   next to the existing main.c symlinks; the m1 Makefile builds `..` and
   `../core`).
3. `patches/xr871fw-rest.patch`: bridge_core.[ch], spi_slave.[ch], wifi.c,
   hal_host.c (+ the new files).
4. `patches/xr871-bridge.patch`: xr871d.c, new xr871d_ext.c (no .mk change).
5. `tools/xr871-image.py` from here (OTA fields kept, `set-ota`, `bootsel`).
6. SDK projects share objects: `make clean` + `rm core/*.o` when switching
   between abus_xr871 and abus_xr871_net (as before).
