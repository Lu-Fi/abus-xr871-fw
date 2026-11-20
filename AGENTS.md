# Instructions for AI agents working on this repository

Read `README.md` first. These rules come from the owner and apply to every
session (Claude Code or other agents).

## Language

- Chat with the owner in **German**.
- Code, comments, commit messages and documentation are in **English**.
- Never name third parties as author or maintainer.

## Privacy (hard rules)

- XR871 flash dumps, readbacks and camera images contain the owner's **WiFi
  credentials** (sysinfo at 0xFB000, `wpa_supplicant.conf` on the T30) and
  vendor cloud IDs.
  - Never print SSIDs, PSKs, keys, UUIDs or passwords. Print lengths only.
  - Filter log output with `ssid|psk|pass|bssid|wpa|key` before showing it.
- Never commit `*.bin`, dumps, readbacks, `private/` or anything copied from a
  camera. Run a secret scan before every push. If in doubt, ask.
- No camera pictures online.
- Work under `/mnt/NVMe/...`, not `/tmp` (shared RAM tmpfs). Original dumps
  stay untouched; work on copies.

## Hardware safety

- **Never flash anything without the owner's explicit OK.**
  - "Flash" includes the clip, the service console writes
    (`xr871-svc-flash.py`, `xr871-deploy-app.sh`) and `xr871d -c fwupdate`.
  - One OK covers one flash.
- Ask before device tests and restarts, and say that the picture will be gone
  briefly. Don't test unfinished components on the device.
- UART monitor (ESP32):
  - only send while the camera is powered (a driven TX pin can back-power it);
  - check that the console is free before writing; another session may be
    capturing.
- Never touch GPIO79/PC15 on the T30: it is the SPI chip select.
- The service console's `poke`, `ferase` and `fwrite` can brick the XR871.
  The boot area (< 0x8000) must never be written at runtime.
- Clip flashing:
  - read twice and compare before writing;
  - erase, write and verify;
  - do an independent readback and compare md5;
  - discard a noisy first read.

## Workstation rules

- Heavy jobs (SDK builds, QEMU, Thingino builds): `taskset -c 4-15 nice -n 19`.
- No wide `find`/`grep` over `/` or `/mnt/NVMe/git`.
- Don't touch other sessions' QEMU instances. Don't rebuild the shared QEMU.
- Don't change system or security settings (firewall, groups, udev); the
  owner does that.

## Build notes

- SDK: XradioTech XR871SDK at commit 5dff623 plus `sdk/xr871sdk.patch`. Use
  `scripts/setup-sdk.sh`.
- Always `make clean` (and remove `core/*.o`) when switching between SDK
  projects. They share objects.
- Builds are named `apNN` (`ap17` = current hardware build). The image is
  assembled by `tools/xr871-image.py build` and must pass `check` (all
  sections "hdr ok data ok").
- Keep the application inside its window. The service console build left only
  a few KB before the net section; check the end address after every build.
- Host tests: `firmware/host` builds the core with a hardware model. Run them
  before any device test.
- Code style: Linux kernel style C, tabs. Small, commented changes. Keep a
  backup of a file before larger edits (`*.pre-<change>`).

## Protocol facts worth knowing

- The T30 must send heartbeat 0xa9 at least every 3 s, otherwise its power
  is cut after 6 s.
- MCU (sw 150):
  - answers the queries 0x15 (arming), 0x18 (sensitivity), 0x1c (delay),
    0x1e (USB), 0x1f (tamper), 0x23 (version);
  - does NOT answer 0x26/0x29/0x2b/0x2c/0x2f;
  - 0x16 = arming on/off.
- Acknowledge every MCU message with `{0x0e, val}` like the vendor firmware.
- AXP2601:
  - SOC 0x08, VBAT 0x04:0x05 (BE, mV), temperature 0x06;
  - time to empty 0x0a:0x0b and time to full 0x0c:0x0d (minutes, inferred);
  - keep the vendor battery model (80 bytes) loaded.
- WiFi credentials arrive as a 64-hex PSK and are accepted directly.

## Where to record things

- Findings and hardware results go into `docs/` (English). A short dated entry
  is enough.
- The camera project notes (`ABUS/work/NOTES.md`, outside this repo) hold the
  full chronology. Keep private data out of both.
