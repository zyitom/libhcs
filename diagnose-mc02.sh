#!/usr/bin/env bash
#
# Read-only check of why the mc02 bootloader is not jumping to the application.
#
# Nothing is erased, programmed or reset -- the probe attaches to the running
# target and reads memory, so the board's state is exactly what it was.
#
# It reports the state of the two gates in bootloader/src/main.cpp that stand
# between a valid image and jump_to_app():
#
#   1. the metadata record in flash sector 1, which validate_app_image() needs
#      to read as "ready" (magic "HCS1" + state "IMRD" + image size), and
#   2. the KEY pin, whose force_stay overrides even a fresh APP1 boot request.
#
# KEY is PA15, which is also JTDI. A probe that drives TDI reads as a held key
# and pins the board in DFU forever, so run this with the probe attached to see
# the value the bootloader actually samples.
#
# Usage:
#   ./diagnose-mc02.sh
#
set -euo pipefail

DEVICE="${JLINK_DEVICE:-STM32H723VGTx}"
SPEED="${SPEED:-4000}"

METADATA_ADDRESS=0x08020000
APP_ADDRESS=0x08040000
GPIOA_IDR=0x58020010 # STM32H7 GPIOA base 0x58020000 + IDR offset 0x10

command -v JLinkExe >/dev/null 2>&1 || {
    echo "error: JLinkExe not found. Install the SEGGER J-Link tools." >&2
    exit 1
}

SCRIPT="$(mktemp -t mc02-diag-XXXXXX.jlink)"
trap 'rm -f "$SCRIPT"' EXIT INT TERM

# Attach without reset: the question is what the CURRENT boot decided.
{
    echo "h"
    echo "mem32 $METADATA_ADDRESS, 16"
    echo "mem32 $APP_ADDRESS, 2"
    echo "mem32 $GPIOA_IDR, 1"
    echo "go"
    echo "qc"
} >"$SCRIPT"

echo ">> Attaching to $DEVICE @ ${SPEED}kHz (read-only)"
JLinkExe -device "$DEVICE" -if SWD -speed "$SPEED" -autoconnect 1 \
    -NoGui 1 -CommanderScript "$SCRIPT" | tee /dev/stderr | awk -v meta="$METADATA_ADDRESS" '
    tolower($0) ~ /^[0-9a-f]{8}[ ]*=/ { lines[n++] = $0 }
    END {
        print ""
        print "=================== interpretation ==================="
        print "Metadata slots are 32 bytes: magic | state | size | reserved"
        print "  magic 0x48435331 = HCS1, state 0x494D5244 = IMRD (ready)"
        print "A slot of all-FFFFFFFF is erased; the LAST non-erased slot wins."
        print ""
        print "Application vector table at 0x08040000 must read as"
        print "  [0] initial MSP  in 0x20000000..0x20020000 (DTCM), 8-aligned"
        print "  [1] Reset_Handler odd (thumb) and inside 0x08040000..0x08100000"
        print ""
        print "GPIOA IDR bit 15 is KEY (PA15/JTDI):"
        print "  1 = released -> force_stay false, the bootloader may jump"
        print "  0 = held     -> force_stay TRUE, the bootloader stays in DFU"
        print "     regardless of the APP1 request a successful DFU flash left."
        print "     With no finger on the button, a 0 here means the PROBE is"
        print "     driving TDI. Unplug the ribbon and power-cycle."
    }'
