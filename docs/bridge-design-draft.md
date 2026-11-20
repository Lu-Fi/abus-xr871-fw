> Historical design draft from 2026-11-11, written before the implementation.
> The current behaviour is described in README.md; differences: xr871net is a
> userspace TAP daemon (not a kernel module) and SPI frames are 1536 bytes.

# XR871 WiFi bridge for Thingino (ABUS PPIC90520) - design draft

Status: draft, nothing implemented on the XR871 side yet.

## Goal
Thingino on the T30 gets a normal network interface (`wlan0`) while the XR871
keeps doing what only it can do: WiFi radio, power control of the T30, and
talking to the ST power/wake MCU. Web UI, portal and `/etc/wpa_supplicant.conf`
stay unchanged for the user.

## Split of work
| Part | Runs on | Job |
|---|---|---|
| bridge firmware | XR871 | WiFi STA/AP (SDK wpa_supplicant / hostapd), Ethernet frames <-> SPI, MCU protocol, T30 power |
| `xr871net.ko` | T30 kernel | netdev `wlan0` over SPI0 (spidev replaced by a real driver), ready-line IRQ |
| `xr871d` | T30 userspace | heartbeat, control channel (SSID/PSK, scan, AP mode, status), replaces `S38wpa_supplicant` backend |

Model: Espressif ESP-Hosted "FG" (co-processor runs the supplicant, host
sees an Ethernet-like interface).

## Physical links (from the stock firmware)
- SPI: T30 SSI0 (port C, master, mode 1, up to 23 MHz in stock) <-> XR871
  SPI1 (PA19..PA22, slave, mode 1, DMA, 1024-byte transfers).
- Ready line: XR871 -> T30 GPIO 77 (T30 waits for 1 before each transfer).
- UART: T30 UART0 (`ttyS0`) <-> XR871 UART, 115200 8N1, 64-byte frames
  (see NOTES.md, "Protocol findings").
- I2C: XR871 I2C1 -> ST MCU 0x0A @100 kHz, wake/IRQ line PA7.

## SPI frame (proposal)
Keep the stock 1024-byte fixed transfer (both directions per transfer):
```
0  u32 magic 0x55aa55aa
4  u8  type   (0x01 eth frame, 0x02 control, 0x03 event, 0x00 idle)
5  u8  flags  (bit0: more frames queued on the sender)
6  u16 len    (payload length, <= 1012)
8  u16 seq
10 u16 crc16 of payload
12 payload
```
An Ethernet frame up to 1514 bytes is split into two transfers (flag
"continued"), or the transfer size is raised to 1536 once the slave DMA is
proven at that size. The XR871 raises the ready line when it has a buffer
armed; it keeps it high while it has data to send (the T30 then clocks an
idle frame to fetch it).

## Control channel (UART, reuse the existing framing)
New command IDs above the vendor range (0xc0..0xdf):
- 0xc0 set STA config (SSID, PSK)
- 0xc1 start AP (SSID, PSK, channel) - portal mode
- 0xc2 scan -> 0xc3 scan results (several frames)
- 0xc4 status -> link, RSSI, MAC, channel
- 0xc5 get MAC (used for `wlan0` hw address)
Heartbeat 0xa9, hello 0x00 and power 0x83 keep their vendor meaning, so the
same T30 daemon works with stock and bridge firmware.

## MCU duties the bridge firmware must keep (or the MCU may cut power)
- answer heartbeat {0x0e,x} with {0x0e,..}
- send {0x05,0x07} before powering the T30, {0x05,0x09} before powering off
- read events on PA7 IRQ (PIR 0x0d, keys 0x12/0x13, USB 0x0c, battery 0x2d,
  tamper 0x08) and forward them to the T30 as UART events (new 0xd0..)
- first version: keep the T30 permanently on while USB power is present

## Build / flash
- SDK: XradioTech/XR871SDK (work/xr871sdk), app + app_xip built from source;
  reuse stock `net`, `wlan_fw`, `wlan_sdd` sections (matching radio/RF
  calibration) where the SDK API allows, else SDK binaries.
- Image: AWIH section chain as in the stock flash; flash the W25Q16 with the
  CH341A clip. Keep the stock dump (md5 62ef60fd9d44a86ebe01a7beef641712)
  for recovery.

## Milestones (each verified on the XR871 UART)
1. SDK "hello" firmware boots, prints on UART, answers the MCU heartbeat,
   keeps the T30 powered (stock XR871 behaviour as reference log first).
2. WiFi STA connect with fixed credentials, ping from XR871.
3. SPI slave loopback with a T30 test tool over `spidev`.
4. Ethernet frames over SPI, `wlan0` on the T30 (userspace TAP prototype
   first, kernel driver after).
5. Control channel + Thingino portal/web UI integration.
