# Enabling A/B updates on the XR871 (one clip flash)

Status: host-tested (`run_tests.py` section 3b) and done on camera 2 on 2026-11-21 (boots image 1, `fwupdate --info` reports OTA enabled). A commit + reboot into image 2 is not yet tested on hardware.

## Why

The XR871 bootloader supports two application images:
- image 1 at 0x8000;
- image 2, with the base at 0xFC000 and the application at 0x104000.

The last record of an OTA config at 0x100000 (SDK "fdcm") selects which one
boots. Our flash images, like the first test builds, have the OTA fields in
the bootloader header **disabled** (0xff). Then the bootloader always boots
image 1, `xr871d -c 'fwupdate <file> --commit'` is refused, and only
`--force-stage` (write and verify image 2 without switching) works.

With OTA enabled:
- an update from Thingino writes image 2, verifies it, appends a "verified"
  record for image 2 (commit) and reboots the XR871;
- the next update goes to the other slot again.

The bootloader falls back to the other image only if the selected APP section
fails its checksum, **not** if the new firmware misbehaves.

Enabling OTA means changing the bootloader header in flash sector 0. The
firmware never writes below 0x8000 at runtime, so this needs one clip flash.

## What changes

| Area | Change | Tool |
|---|---|---|
| 0x000000-0x000FFF (sector 0) | boot header priv fields: OTA area 0x100000, size 0x4000; header checksum fixed | `xr871-image.py set-ota` |
| 0x100000-0x103FFF (OTA config) | one more record "image 1, verified" (or a fresh area with that record) | `xr871-image.py set-cfg <in> <out> 1 [fresh]` |

Nothing else may differ. `run_tests.py` checks this on the host model.

Why `set-cfg` is needed:
- A stock-derived flash still has the stock records in the OTA area, and
  image 2 may hold an older staged image (e.g. after a `--force-stage` test).
- Without an explicit "image 1" record, enabling OTA could boot that older
  image 2.
- `set-cfg` refuses to point at an image whose APP section is not valid.
  `bootsel` prints what the bootloader model will boot.

## Procedure

Prerequisites:
- CH341A programmer and an SOIC-8 clip; flashrom.
- **3.3 V** on VCC and on the data lines. The W25Q16JV is a 3.3 V part, and
  the black CH341A boards with 5 V logic need the 3.3 V modification.

1. **Power off completely:** unplug USB and the battery connector. The clip
   supply back-feeds the board rails. Earlier, a shorted charger made the
   CH341A drop off USB.
2. Clip the **W25Q16** (8-pin SOIC on the main board next to the XR871GT).
   - Not the bigger 25Q128 of the T30.
   - Pin 1 = the dot on the chip = the red wire of the clip.
3. Read the flash three times and compare:
   ```bash
   flashrom -p ch341a_spi -r xr871-read1.bin
   flashrom -p ch341a_spi -r xr871-read2.bin
   flashrom -p ch341a_spi -r xr871-read3.bin
   md5sum xr871-read*.bin
   ```
   - A noisy first read (contact settling) is common: discard it.
   - Two identical reads are the backup.
   - The backup contains the WiFi credentials (sysinfo at 0xFB000). Keep it
     private (`private/`, mode 600) and never upload it.
4. Build the new image from that backup and check it:
   ```bash
   tools/xr871-image.py set-ota private/xr871-read2.bin private/xr871-ota.bin
   tools/xr871-image.py set-cfg private/xr871-ota.bin private/xr871-ab.bin 1
   tools/xr871-image.py check private/xr871-ab.bin      # all sections "hdr ok data ok"
   tools/xr871-image.py bootsel private/xr871-ab.bin    # must say "boots image 1"
   cmp -l private/xr871-read2.bin private/xr871-ab.bin | awk '{printf "%x\n", $1-1}' | sort -u
   ```
   The `cmp` offsets must be in 0x0-0xfff and 0x100000-0x103fff only.
5. Write, then do an independent readback:
   ```bash
   flashrom -p ch341a_spi -w private/xr871-ab.bin       # flashrom verifies
   flashrom -p ch341a_spi -r private/xr871-after.bin
   md5sum private/xr871-ab.bin private/xr871-after.bin  # must match
   ```
6. Remove the clip, connect the battery, power on. Then check:
   - the console banner shows the expected build;
   - on the T30, `xr871d -c 'fwupdate --info'` reports
     "running image 1, target image 2, OTA enabled".

Recovery: write the backup from step 3 with the clip.

## After enabling

```bash
xr871d -c 'fwupdate /tmp/new.bin'                    # stage + verify image 2
xr871d -c 'fwupdate --commit --reboot'               # switch and reboot the XR871
xr871d -c 'fwupdate --abort'                         # before the reboot: back to the running image
```

Rebooting the XR871 also power-cycles the T30.
