# abus-xr871-fw

Own firmware for the **XRadio XR871** WiFi co-processor of the ABUS PPIC90520
battery camera, written so the camera can run [Thingino](https://github.com/themactep/thingino-firmware)
on its Ingenic T30 instead of the vendor firmware.

**Private repository.** Built images contain parts of the vendor firmware
(bootloader, RF calibration) and the camera stores WiFi credentials in its
XR871 flash. Never commit flash images, dumps or readbacks. See
[Private inputs](#private-inputs).

## The camera

| Part | Role |
|---|---|
| Ingenic T30N + Sony IMX307 | video. Runs Thingino. |
| XRadio XR871 (Cortex-M4 + WiFi) | WiFi, powers the T30, talks to the MCU. Runs **this firmware**. |
| STM8L051 MCU (I2C 0x0A) | power/wake controller: PIR, keys, USB detection, tamper |
| AXP2601 (I2C 0x62) | battery fuel gauge |

The T30 has no WiFi of its own. Everything it sends or receives goes through
the XR871.

## What the firmware does

- Powers the T30 (PA16) and keeps it powered while the T30 sends its heartbeat
  (UART cmd 0xa9 at least every 3 s).
- **UART control link** to the T30 (115200 8N1). These are the vendor frames
  (`aa 55 aa 55`, u16 len, cmd, 0x01, u16 param, payload). They carry:
  - hello, clock, wake source;
  - WiFi credentials (cmd 0xc0, from the T30's `wpa_supplicant.conf`, never logged);
  - the ready-pin probe;
  - sleep requests.
- **SPI data link**: SPI1 slave (PA19..22, mode 1, DMA, 1536-byte frames) plus a
  ready line to the T30 (GPIO77). The T30 side (`xr871net`) turns it into a
  normal `wlan0` network interface. The XR871 is a pure layer-2 bridge (no
  IP stack use, no DHCP on the XR871). The WiFi MAC is the station MAC.
- **MCU protocol** (I2C1, 2-byte messages `{cmd,val}`, IRQ on PA7):
  - answers the heartbeat {0x0e}, acknowledges events;
  - queries USB/tamper/version at boot;
  - PIR (0x0d) and key (0x12/0x13) wake-ups.
- **Sleep / wake**: the T30 asks for sleep with cmd 0x83, and the XR871 cuts the
  T30 power.
  - Wake sources: PIR, key, timer, Wake-on-LAN, and TCP SYN to selected ports
    (an ARP proxy answers for the sleeping T30).
  - The wake reason is reported to the T30 (0x85).
  - WLAN power save is on while the T30 sleeps.
- **Battery**: AXP2601 with the vendor battery model; SOC, voltage and
  temperature are reported to the T30.
- **Service console** on the debug UART (pads "TX"/"RX" on the board):
  - peek/poke, eFuse, flash CRC/read/erase/write, MCU messages, GPIO, I2C,
    battery dump, T30 power reset;
  - a boot-time window (send `s` within 3 s) and a runtime console.

The T30 counterpart is the Thingino package `xr871-bridge` (`xr871d` =
control daemon, `xr871net` = SPI network interface, standby/PIR/day-night
scripts). It lives in the Thingino tree, not here.

## Layout

| Path | What |
|---|---|
| `firmware/core/` | portable protocol and bridge logic (also built on the host for tests) |
| `firmware/xr871/` | XR871 SDK glue: `main.c`, `spi_slave.c`, `wifi.c`, hardware hooks |
| `firmware/host/` | host model of the hardware (tests without a camera) |
| `sdk/xr871sdk.patch` | changes to the XR871 SDK (eFuse, chip init, lwIP, WLAN control, libc wraps) |
| `sdk/project/abus_xr871_net/` | SDK project (Makefile, `localconfig.mk`, `prj_config.h`) |
| `sdk/board/abus_ppic90520/` | board config (pinmux, I2C1 on PA8/PA9, flash mode) |
| `scripts/setup-sdk.sh` | clones the SDK at the pinned commit, applies the patch, links this repo in |
| `tools/` | image builder/checker (`xr871-image.py`), service console clients, deploy script |
| `docs/` | design notes |

## Build

Fedora (other distributions similar): `arm-none-eabi-gcc` (15.x works) and
`arm-none-eabi-newlib`.

```bash
scripts/setup-sdk.sh ../xr871sdk              # clone XR871SDK @5dff623, patch, link
cd ../xr871sdk/project/abus_xr871_net/gcc
make clean && make                            # -> abus_xr871_net.bin (application)
```

- **Always `make clean` after switching SDK projects.** `project/common` and the
  linked `core/` objects are shared.
- The application must end before the next section in the image layout.
  `tools/xr871-image.py check` validates the finished image.

## Private inputs

An XR871 flash image is assembled from sections (`tools/xr871-image.py
build`):

| Section | Source |
|---|---|
| bootloader (0x0) | **your own flash dump** (vendor code) |
| application (0x8000) | this repo |
| net / WLAN bootloader / WLAN firmware | XR871 SDK binaries |
| RF calibration (`sdd`) | **your own flash dump** (per-device calibration) |

Keep the private parts in `private/` (git-ignored). Never add `*.bin` files to
the repository.

## Flashing

- **Clip** (board unpowered): CH341A on the W25Q16 (SOIC-8). Always read twice
  and compare before writing, then do an independent readback after writing.
- **Service console** (no clip): `tools/xr871-svc-flash.py` writes the changed
  sectors over the debug UART, through the
  [Lu-Fi UART Monitor](https://github.com/Lu-Fi/lufi-uart-monitor) ESP32.
  `tools/xr871-deploy-app.sh` wraps this.
- **From Thingino over the internal UART** (`xr871d -c fwupdate`, A/B slots,
  the bootloader is never written): in progress.

Recovery is always possible with the clip and the original dump.

## Tests (host, no camera needed)

```bash
cd firmware && python3 -I tests/run_tests.py     # 119 checks, incl. the update over a 115200 link model
```

`firmware/host` builds the portable core against a hardware model (MCU,
battery, flash, slow UART). `main.c`, `wifi.c`, `spi_slave.c` and `hal_ext.c`
are only covered by the SDK build.

## Status

- Running on camera 2 as build `ap22` (2026-11-21; ap17 before): WiFi STA, SPI bridge, standby with
  wake-ups, battery, LED.
- `ap18-features` (built, host-tested, not on hardware yet) adds the service
  extensions: status 0xd0, MCU passthrough 0xd1/0xd2, GPIO test 0xd3, XR871 log
  forwarding 0xd4, and the application update 0xd8..0xdc. See
  `docs/service-extensions.md` and `docs/merge-2026-11-20.md`.
- The current flash images have **OTA disabled in the bootloader header**: there
  is no A/B fallback, so `fwupdate --commit` is refused. Enabling it needs
  `tools/xr871-image.py set-ota` and one clip flash of sector 0.
- Camera 1 is paused: hardware damage on the charger, unrelated to this
  firmware.
- See `AGENTS.md` for the working rules and `docs/` for details.
