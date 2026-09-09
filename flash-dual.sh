#!/usr/bin/env bash
#
# Flash the dual CAN-FD HPM5321 board over USB DFU.
#
# Builds BOARD=hpm5321, whose single image serves both HPM5321 PCBs and picks its
# CAN/LED tables at run time from OTP word 25 (firmware/hpm_board/common/
# board_identity.hpp). The board still enumerates as 0x5322, so the DFU_ID below
# and any udev rule keyed on it are unchanged by the merge.
#
# The HPM5321 board carries the same "HCS DFU Bootloader" as mc02/c_board, so the
# app is flashed with dfu-util -- no debugger (J-Link/OpenOCD) needed. A debugger
# is only required to put the DFU bootloader onto a blank chip the first time
# (see ozone/hpm5321.jdebug, or ./jlink-debug.sh hpm5321 with LOAD=1).
#
# Prerequisite: the DFU bootloader is already on the board. Put the device into
# DFU mode first:
#   - power up with no valid app (bootloader stays in DFU), or
#   - trigger a DFU reboot from the host while the app runs.
# A running app exposes the DFU runtime interface, so dfu-util will detach and
# re-enumerate it into DFU mode automatically.
#
# Usage:
#   ./flash-dual.sh                        # builds the release preset, then flashes
#   PRESET=debug ./flash-dual.sh           # build/flash a debuggable (-O0) image instead
#   BOARD=hpm5321 ./flash-dual.sh          # explicit equivalent of the default
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOARD="${BOARD:-hpm5321}"
if [[ "$BOARD" != hpm5321 ]]; then
    echo "error: BOARD must be hpm5321; the image selects the PCB from OTP word 25" >&2
    exit 2
fi
DFU_IMAGE="$SCRIPT_DIR/firmware/hpm_board/build/app/output/hpm_board_app_${BOARD}.dfu"
# The PID the DEVICE enumerates as, which is what -d matches. Unrelated to the PID
# in the .dfu suffix: BOARD=hpm5321 stamps the wildcard 0xFFFF there so one file
# passes for either PCB.
DFU_ID="0xa511:0x5322" # VENDOR_ID:PRODUCT_ID (alt 0 = Internal Flash)

command -v dfu-util >/dev/null 2>&1 || {
    echo "error: dfu-util not found (need >= 0.11)" >&2
    exit 1
}

# Build the exact image about to be flashed. Flashing is deployment, so this
# defaults to the release preset (optimized + NDEBUG). The presets share one
# build dir, so switching PRESET triggers a full reconfigure.
PRESET="${PRESET:-release}"
: "${GNURISCV_TOOLCHAIN_PATH:?GNURISCV_TOOLCHAIN_PATH must point to the RISC-V toolchain root}"
echo ">> Building hpm_board_app (preset: $PRESET, BOARD=$BOARD)"
cmake --preset "$PRESET" -S "$SCRIPT_DIR/firmware/hpm_board" -DBOARD="$BOARD"
cmake --build "$SCRIPT_DIR/firmware/hpm_board/build" --target hpm_board_app

if [[ ! -f "$DFU_IMAGE" ]]; then
    echo "error: $DFU_IMAGE not found after the build." >&2
    exit 1
fi

echo ">> dual CAN-FD app image ($BOARD): $DFU_IMAGE"
echo ">> DFU devices currently visible:"
dfu-util -l 2>/dev/null | grep -i "a511:5322" || \
    echo "   (none in DFU mode yet; dfu-util will try to detach a running app)"

# -d filters to the dual-can VID:PID, -a 0 selects the Internal Flash alt setting.
dfu-util -d "$DFU_ID" -a 0 -D "$DFU_IMAGE"

echo ">> Done. The bootloader verifies the image and resets into the app."
echo "   (A 'lost device' / status-read error from dfu-util at the end is normal.)"
