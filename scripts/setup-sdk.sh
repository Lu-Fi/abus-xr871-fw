#!/bin/sh
# Set up the XR871 SDK for this repository:
#   scripts/setup-sdk.sh <sdk-dir>
# clones XradioTech/XR871SDK at the pinned commit (if <sdk-dir> does not exist),
# applies sdk/xr871sdk.patch and links the project, board and firmware sources
# of this repository into the SDK tree.
set -e
SDK_COMMIT=5dff623
REPO=$(cd "$(dirname "$0")/.." && pwd)
SDK=${1:?usage: $0 <sdk-dir>}
if [ ! -d "$SDK/.git" ]; then
	git clone https://github.com/XradioTech/XR871SDK.git "$SDK"
fi
cd "$SDK"
git checkout -q "$SDK_COMMIT"
if git apply --check "$REPO/sdk/xr871sdk.patch" 2>/dev/null; then
	git apply "$REPO/sdk/xr871sdk.patch"
	echo "sdk patch applied"
else
	echo "sdk patch already applied or does not fit - check 'git status' in $SDK"
fi
B=project/common/board/abus_ppic90520
P=project/abus_xr871_net
mkdir -p "$P/gcc"
[ -e "$B" ] || ln -s "$REPO/sdk/board/abus_ppic90520" "$B"
for f in Makefile localconfig.mk; do
	ln -sf "$REPO/sdk/project/abus_xr871_net/gcc/$f" "$P/gcc/$f"
done
ln -sf "$REPO/sdk/project/abus_xr871_net/prj_config.h" "$P/prj_config.h"
ln -sfn "$REPO/firmware/core" "$P/core"
# the project Makefile compiles every .c in the project directory
for f in "$REPO"/firmware/xr871/*.[ch]; do
	ln -sf "$f" "$P/$(basename "$f")"
done
echo "ready: cd $SDK/$P/gcc && make clean && make"
