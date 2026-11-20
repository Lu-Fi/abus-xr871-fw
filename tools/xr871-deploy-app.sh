#!/bin/sh
# Build the XR871 image from the current SDK build output and write the app
# region over the runtime console, then reboot the XR871.
# usage: xr871-deploy-app.sh NAME PREV_NAME   (flash/xr871-abus-m2-NAME.bin)
set -e
N=$1 P=$2
cd /mnt/NVMe/git/ABUS/work/xr871m2
cp ../xr871sdk/project/abus_xr871_net/gcc/abus_xr871_net.bin app-$N.bin
python3 -I ../../tools/xr871-image.py build xr871-abus-m2-$P.bin xr871-abus-m2-$N.bin \
 0xa5ff5a00:svc2-sec/a5ff5a00.bin:0x0:0x67000:0x67101:1 \
 0xa5fe5a01:app-$N.bin:0x8000:0x10000:0x10101:1 \
 0xa5fc5a03:svc2-sec/a5fc5a03.bin:0x2d000:0x60000000:x:1 \
 0xa5fa5a05:svc2-sec/a5fa5a05.bin:0x65000:x:x:1 \
 0xa5f95a06:svc2-sec/a5f95a06.bin:0x66000:x:x:1 \
 0xa5f85a07:svc2-sec/a5f85a07.bin:0x86000:x:x:1 \
 0xa5fb5a04:net_ap.bin:0x87000:0x60000000:x:1 >/dev/null
[ "$(python3 -I ../../tools/xr871-image.py check xr871-abus-m2-$N.bin | grep -c 'hdr ok data ok')" = 7 ] || { echo "image check failed"; exit 1; }
install -m 600 xr871-abus-m2-$N.bin ../../flash/
md5sum ../../flash/xr871-abus-m2-$N.bin
cd /mnt/NVMe/git/ABUS
python3 -I tools/xr871-svc-flash.py 192.168.10.183:2321 flash/xr871-abus-m2-$N.bin --old flash/xr871-abus-m2-$P.bin --from 0x8000 --to 0xc6000 --write | grep -v -E "^block 0x[0-9a-f]+ ok"
python3 -I -c "
import socket,time
s=socket.create_connection(('192.168.10.183',2321)); time.sleep(1.5); s.settimeout(0.2)
try:
    while s.recv(65536): pass
except Exception: pass
s.sendall(b'reboot\r'); time.sleep(1); s.close()"
echo "rebooted into $N"
