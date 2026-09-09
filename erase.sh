#!/usr/bin/env bash
#
# One-command flash erase over J-Link, for any board this repo debugs with one.
#
# Ozone can erase too, but only by typing Target.EraseChip() into its Console --
# the manual lists "GUI Access: None" for that command (UM08025 7.9.18.1) -- and
# it always wipes the whole chip. This script is the one-liner equivalent, and it
# defaults to the reversible half of the job.
#
# Usage:
#   ./erase.sh <target>            # erase the application region (default, reversible over DFU)
#   ./erase.sh <target> --all      # full chip erase, DFU bootloader included
#   ./erase.sh <target> --dry-run  # print the J-Link script instead of running it
#   ./erase.sh --list
#
# Target names match ./ozone-debug.sh and ./jlink-debug.sh.
#
# WHAT "APP" MEANS. Every board here boots a DFU bootloader that validates the
# application against a SHA-256 metadata record. The app erase starts at the
# metadata record, not at the application image, so no stale record survives to
# describe an image that is no longer there. The bootloader itself is untouched,
# so the board still enumerates over USB and ./flash-*.sh keeps working with no
# debugger attached.
#
# A full erase takes the bootloader with it. The only way back in after that is
# the debugger: ./ozone-debug.sh <target> opens a CM_DOWNLOAD_RESET project, so
# just opening it reprograms the bootloader.
#
# Environment overrides:
#   SPEED=<kHz>     probe clock (default 4000, same as the ozone/ projects)
#   DEVICE=<name>   J-Link device name, overriding the per-target default
#   JLINK=<path>    JLinkExe to use (default: whatever is on PATH)
#
set -euo pipefail

SPEED="${SPEED:-4000}"
JLINK="${JLINK:-$(command -v JLinkExe || true)}"

# Per-target flash map. Every address below is taken from a file in this repo,
# never from a datasheet: the STM32 regions come from the two linker scripts plus
# kMetadataStartAddress in bootloader/src/flash/metadata.hpp, the HPM ones from
# UF2_BOOTLOADER_RESERVED_LENGTH in the board's app_flash_uf2.ld and the flash
# size in its yaml.
#
# EXT_NOR marks a part whose program lives in external QSPI NOR. J-Link refuses
# `erase` outside internal flash banks unless EnableEraseAllFlashBanks is set --
# and without it, it reports "No Flash bank within given address range ..." and
# then still prints "Erasing done.", so it looks like it worked while nothing was
# touched.
print_targets() {
    echo "Available targets:"
    printf '  %-10s %-18s %-5s %s\n' NAME DEVICE IF "APP ERASE RANGE"
    printf '  %-10s %-18s %-5s %s\n' mc02 STM32H723VGTx SWD "0x08020000 .. 0x080FFFFF"
    printf '  %-10s %-18s %-5s %s\n' c_board STM32F407IGHx SWD "0x0800C000 .. 0x080FFFFF"
    printf '  %-10s %-18s %-5s %s\n' hpm5321 HPM5321xEGx JTAG "0x80020000 .. 0x800FFFFF"
    printf '  %-10s %-18s %-5s %s\n' hpm6e8y HPM6E8YxGNx JTAG "0x80020000 .. 0x803FFFFF"
}

TARGET="${1:-}"
case "$TARGET" in
-h | --help)
    sed -n '2,33p' "${BASH_SOURCE[0]}" | sed 's/^#\s\?//'
    exit 0
    ;;
--list)
    print_targets
    exit 0
    ;;
esac
shift || true

case "$TARGET" in
mc02)
    DEFAULT_DEVICE="STM32H723VGTx"
    IFACE="SWD"
    FLASH_BASE=0x08000000
    APP_BASE=0x08020000 # metadata record (sector 1); the image itself is at 0x08040000
    FLASH_END=0x080FFFFF
    EXT_NOR=0
    ;;
c_board)
    DEFAULT_DEVICE="STM32F407IGHx"
    IFACE="SWD"
    FLASH_BASE=0x08000000
    APP_BASE=0x0800C000 # metadata record; the image itself is at 0x08010000
    FLASH_END=0x080FFFFF
    EXT_NOR=0
    ;;
hpm5321)
    # HPM5321xEGx is the real die. The yaml's HPM5361 is the SDK build target,
    # not a debugger name -- see the long note in jlink-debug.sh.
    DEFAULT_DEVICE="HPM5321xEGx"
    IFACE="JTAG"
    FLASH_BASE=0x80000000
    APP_BASE=0x80020000
    FLASH_END=0x800FFFFF # 1 MB, hpm5321.yaml
    EXT_NOR=1
    ;;
hpm6e8y)
    DEFAULT_DEVICE="HPM6E8YxGNx"
    IFACE="JTAG"
    FLASH_BASE=0x80000000
    APP_BASE=0x80020000
    FLASH_END=0x803FFFFF # 4 MB, hpm6e8y.yaml
    EXT_NOR=1
    ;;
hpm6e8y-core1)
    echo "error: core1 has no image in flash -- core0 copies it out of sec_core_img" >&2
    echo "       at boot. Erase the hpm6e8y target instead." >&2
    exit 2
    ;;
ch32_board)
    echo "error: ch32_board is debugged over WCH-Link, which J-Link does not speak." >&2
    echo "       See firmware/ch32_board/AGENTS.md." >&2
    exit 2
    ;;
"")
    echo "error: no target given" >&2
    echo >&2
    print_targets >&2
    exit 1
    ;;
*)
    echo "error: unknown target '$TARGET'" >&2
    echo >&2
    print_targets >&2
    exit 1
    ;;
esac

DEVICE="${DEVICE:-$DEFAULT_DEVICE}"

MODE=app
DRY_RUN=0
for arg in "$@"; do
    case "$arg" in
    --all) MODE=all ;;
    --app) MODE=app ;;
    --dry-run) DRY_RUN=1 ;;
    *)
        echo "error: unknown argument: $arg" >&2
        exit 2
        ;;
    esac
done

[[ -n "$JLINK" && -x "$JLINK" ]] || {
    echo "error: JLinkExe not found. Set JLINK=/path/to/JLinkExe." >&2
    exit 1
}

if [[ "$MODE" == all ]]; then
    ERASE_CMD="erase"
    VERIFY_ADDR="$FLASH_BASE"
    WHAT="ENTIRE CHIP ($FLASH_BASE .. $FLASH_END) -- DFU bootloader included"
else
    ERASE_CMD="erase $APP_BASE $FLASH_END"
    VERIFY_ADDR="$APP_BASE"
    WHAT="application region only ($APP_BASE .. $FLASH_END) -- bootloader kept"
fi

SCRIPT_FILE="$(mktemp -t "$TARGET-erase.XXXXXX.jlink")"
trap 'rm -f "$SCRIPT_FILE"' EXIT
{
    echo "connect"
    ((EXT_NOR)) && echo "exec EnableEraseAllFlashBanks"
    echo "$ERASE_CMD"
    echo "mem32 $VERIFY_ADDR 0x4"
    echo "exit"
} >"$SCRIPT_FILE"

echo ">> target: $TARGET   device: $DEVICE   interface: $IFACE @ ${SPEED} kHz"
echo ">> erasing: $WHAT"

if ((DRY_RUN)); then
    echo ">> dry run, J-Link script would be:"
    sed 's/^/     /' "$SCRIPT_FILE"
    exit 0
fi

if [[ "$MODE" == all ]]; then
    echo ">> after this, the only way back in is ./ozone-debug.sh $TARGET"
    # `|| reply=` so EOF (Ctrl-D, or stdin redirected) falls through to the abort
    # below instead of tripping set -e and exiting with no explanation.
    read -r -p ">> this removes the DFU bootloader too. Type 'erase' to confirm: " reply || reply=
    [[ "$reply" == erase ]] || {
        echo ">> aborted"
        exit 1
    }
fi

LOG_FILE="$(mktemp -t "$TARGET-erase.XXXXXX.log")"
trap 'rm -f "$SCRIPT_FILE" "$LOG_FILE"' EXIT

# -JTAGConf -1,-1 is auto-detect, matching the ozone/ projects' chain position.
# Without it JLinkExe stops to ask interactively and the script stalls. It is a
# JTAG-only option, so SWD targets must not pass it.
JLINK_ARGS=(-nogui 1 -exitonerror 1 -device "$DEVICE" -if "$IFACE" -speed "$SPEED")
[[ "$IFACE" == JTAG ]] && JLINK_ARGS+=(-JTAGConf -1,-1)

"$JLINK" "${JLINK_ARGS[@]}" -CommanderScript "$SCRIPT_FILE" 2>&1 | tee "$LOG_FILE"

# J-Link prints "Erasing done." even when the range matched no flash bank at all,
# so its exit status and its own summary line both lie. The only honest check is
# reading the first word back and seeing erased flash (0xFFFFFFFF).
if grep -q "No Flash bank within given address range" "$LOG_FILE"; then
    echo >&2
    echo "error: J-Link found no flash bank covering the range -- nothing was erased." >&2
    ((EXT_NOR)) && echo "       This part programs external QSPI NOR; check that
       'exec EnableEraseAllFlashBanks' was accepted above." >&2
    exit 1
fi

if ! grep -qi "$(printf '%08X' "$VERIFY_ADDR").*FFFFFFFF" "$LOG_FILE"; then
    echo >&2
    echo "warning: read-back at $VERIFY_ADDR is not 0xFFFFFFFF -- the erase may not" >&2
    echo "         have taken. See the mem32 output above." >&2
    exit 1
fi

echo
echo ">> verified: $VERIFY_ADDR reads 0xFFFFFFFF"
