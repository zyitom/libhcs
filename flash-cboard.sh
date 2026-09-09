#!/usr/bin/env bash
#
# Flash the c_board application over USB DFU.
#
# Prerequisite: the c_board DFU bootloader is already on the board (flashed once
# via SWD/J-Link, see README). Put the device into DFU mode first:
#   - power up with no valid app in flash (bootloader stays in DFU), or
#   - trigger a DFU reboot from the host while the app runs.
# A running app also exposes the DFU runtime interface, so dfu-util will detach
# and re-enumerate it into DFU mode automatically.
#
# Usage:
#   ./flash-cboard.sh                 # builds the release preset, then flashes
#   PRESET=debug ./flash-cboard.sh    # build/flash a debuggable (-O0) image instead
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DFU_IMAGE="$SCRIPT_DIR/firmware/c_board/build/app/c_board_app.dfu"
DFU_ID="0xa511:0xf407" # VENDOR_ID:PRODUCT_ID baked into the .dfu suffix (alt 0 = Internal Flash)

command -v dfu-util >/dev/null 2>&1 || {
    echo "error: dfu-util not found (need >= 0.11)" >&2
    exit 1
}

# Build the exact image about to be flashed. Flashing is deployment, so this
# defaults to the release preset (optimized + NDEBUG). The presets share one
# build dir, so switching PRESET triggers a full reconfigure.
PRESET="${PRESET:-release}"
echo ">> Building c_board_app (preset: $PRESET)"
cmake --preset "$PRESET" -S "$SCRIPT_DIR/firmware/c_board"
cmake --build "$SCRIPT_DIR/firmware/c_board/build" --target c_board_app

if [[ ! -f "$DFU_IMAGE" ]]; then
    echo "error: $DFU_IMAGE not found after the build." >&2
    exit 1
fi

echo ">> c_board app image: $DFU_IMAGE"
echo ">> DFU devices currently visible:"
dfu-util -l 2>/dev/null | grep -i "a511:f407" || \
    echo "   (none in DFU mode yet; dfu-util will try to detach a running app)"

# -d filters to the c_board VID:PID, -a 0 selects the Internal Flash alt setting.
dfu-util -d "$DFU_ID" -a 0 -D "$DFU_IMAGE"

echo ">> Done. The bootloader verifies the image and resets into the app."
echo "   (A 'lost device' / status-read error from dfu-util at the end is normal.)"
