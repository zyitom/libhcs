#!/usr/bin/env bash
#
# Flash the mc02 application over USB DFU.
#
# Prerequisite: the mc02 DFU bootloader is already on the board (flashed once via
# SWD/J-Link, see README "烧录 Bootloader"). Put the device into DFU mode first:
#   - power up with no valid app in flash (bootloader stays in DFU), or
#   - trigger a DFU reboot from the host while the app runs.
# A running app also exposes the DFU runtime interface, so dfu-util will detach
# and re-enumerate it into DFU mode automatically.
#
# Usage:
#   ./flash-mc02.sh                 # builds the release preset, then flashes
#   PRESET=debug ./flash-mc02.sh    # build/flash a debuggable (-O0) image instead
#   SKIP_BOOTLOADER=1 ./flash-mc02.sh   # app only, do not touch the bootloader target
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DFU_IMAGE="$SCRIPT_DIR/firmware/mc02/build/app/mc02_app.dfu"
DFU_ID="0xa511:0x0723" # VENDOR_ID:PRODUCT_ID baked into the .dfu suffix (alt 0 = Internal Flash)

command -v dfu-util >/dev/null 2>&1 || {
    echo "error: dfu-util not found (need >= 0.11)" >&2
    exit 1
}

# Build the exact image about to be flashed. Flashing is deployment, so this
# defaults to the release preset (optimized + NDEBUG). The presets share one
# build dir, so switching PRESET triggers a full reconfigure.
PRESET="${PRESET:-release}"
echo ">> Building mc02_app (preset: $PRESET)"
cmake --preset "$PRESET" -S "$SCRIPT_DIR/firmware/mc02"
cmake --build "$SCRIPT_DIR/firmware/mc02/build" --target mc02_app

# The bootloader is not flashed here -- it goes over SWD and normally never
# changes. It is still built, for two reasons: the two images share the flash
# layout and the metadata-record format (bootloader/src/flash/validation.hpp),
# so building only the app hides breakage in that shared code until the next
# cold boot rejects the image; and ozone/mc02.jdebug opens
# build/bootloader/mc02_bootloader.elf, which does not exist until something
# builds it.
#
# Deliberately after the app and deliberately non-fatal: this script's job is to
# get an app onto the board, and a bootloader that fails to compile must not
# stand between you and that. Ninja no-ops when nothing changed, so the usual
# cost is zero.
if [[ -z "${SKIP_BOOTLOADER:-}" ]]; then
    echo ">> Building mc02_bootloader (not flashed; SWD only)"
    cmake --build "$SCRIPT_DIR/firmware/mc02/build" --target mc02_bootloader ||
        echo "warning: mc02_bootloader failed to build; flashing the app anyway." >&2
fi

if [[ ! -f "$DFU_IMAGE" ]]; then
    echo "error: $DFU_IMAGE not found after the build." >&2
    exit 1
fi

echo ">> mc02 app image: $DFU_IMAGE"
echo ">> DFU devices currently visible:"
dfu-util -l 2>/dev/null | grep -i "a511:0723" || \
    echo "   (none in DFU mode yet; dfu-util will try to detach a running app)"

# -d filters to the mc02 VID:PID, -a 0 selects the Internal Flash alt setting.
dfu-util -d "$DFU_ID" -a 0 -D "$DFU_IMAGE"

echo ">> Done. The bootloader verifies the image and resets into the app."
echo "   (A 'lost device' / status-read error from dfu-util at the end is normal.)"
