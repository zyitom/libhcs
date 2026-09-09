#!/usr/bin/env bash
#
# Flash an application image over USB DFU.
#
# Prerequisite: the DFU bootloader is already on the board (first programming
# is SWD/JTAG; see ozone/*.jdebug or ./tools/jlink-debug.sh <target> with LOAD=1).
# Put the device into DFU mode first:
#   - power up with no valid app (bootloader stays in DFU), or
#   - trigger a DFU reboot from the host while the app runs.
# A running app exposes the DFU runtime interface, so dfu-util will detach and
# re-enumerate it into DFU mode automatically.
#
# Usage:
#   ./tools/flash.sh <target>           # build the release preset, then flash
#   ./tools/flash.sh <target> debug     # build/flash a debuggable image instead
#   ./tools/flash.sh <target> release   # explicit equivalent of the default
#   ./tools/flash.sh --list
#
# Target names match ./tools/erase.sh and ./tools/jlink-debug.sh. Aliases: cboard, 5321.
#
# HPM5321: one image serves both PCBs (OTP word 25 picks CAN/LED tables). The
# .dfu suffix PID is the wildcard 0xFFFF; -d matches whichever of 0x5321 / 0x5322
# is currently enumerated. Both plugged in at once is an error.
#
# Environment overrides:
#   SKIP_BOOTLOADER=1   mc02: do not also build the bootloader target
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

print_targets() {
    echo "Available targets:"
    printf '  %-10s %-18s %s\n' NAME CMAKE DFU
    printf '  %-10s %-18s %s\n' mc02 firmware/mc02 "0xa511:0x0723"
    printf '  %-10s %-18s %s\n' c_board firmware/c_board "0xa511:0xf407"
    printf '  %-10s %-18s %s\n' hpm5321 "firmware/hpm_board -DBOARD=hpm5321" "0xa511:0x5321 or 0x5322"
}

usage() {
    sed -n '2,27p' "${BASH_SOURCE[0]}" | sed 's/^#\s\?//'
    echo
    print_targets
}

TARGET="${1:-}"
case "$TARGET" in
-h | --help)
    usage
    exit 0
    ;;
--list)
    print_targets
    exit 0
    ;;
"")
    usage >&2
    exit 2
    ;;
esac
shift

PRESET=release
if [[ $# -gt 0 ]]; then
    case "$1" in
    debug | release)
        PRESET="$1"
        shift
        ;;
    *)
        echo "error: unexpected argument '$1' (only trailing debug or release is accepted)" >&2
        exit 2
        ;;
    esac
fi
if [[ $# -gt 0 ]]; then
    echo "error: extra arguments: $*" >&2
    exit 2
fi

case "$TARGET" in
cboard) TARGET=c_board ;;
5321) TARGET=hpm5321 ;;
esac

CMAKE_SRC=""
CMAKE_EXTRA=()
BUILD_DIR=""
BUILD_TARGET=""
DFU_IMAGE=""
DFU_ID=""
DFU_GREP=""
NEED_RISCV=0
BUILD_BOOTLOADER=""

case "$TARGET" in
mc02)
    CMAKE_SRC="$ROOT/firmware/mc02"
    BUILD_DIR="$ROOT/firmware/mc02/build"
    BUILD_TARGET=mc02_app
    DFU_IMAGE="$BUILD_DIR/app/mc02_app.dfu"
    DFU_ID="0xa511:0x0723"
    DFU_GREP="a511:0723"
    BUILD_BOOTLOADER=mc02_bootloader
    ;;
c_board)
    CMAKE_SRC="$ROOT/firmware/c_board"
    BUILD_DIR="$ROOT/firmware/c_board/build"
    BUILD_TARGET=c_board_app
    DFU_IMAGE="$BUILD_DIR/app/c_board_app.dfu"
    DFU_ID="0xa511:0xf407"
    DFU_GREP="a511:f407"
    ;;
hpm5321)
    CMAKE_SRC="$ROOT/firmware/hpm_board"
    CMAKE_EXTRA=(-DBOARD=hpm5321)
    BUILD_DIR="$ROOT/firmware/hpm_board/build"
    BUILD_TARGET=hpm_board_app
    DFU_IMAGE="$BUILD_DIR/app/output/hpm_board_app_hpm5321.dfu"
    DFU_GREP='a511:532[12]'
    NEED_RISCV=1
    ;;
*)
    echo "error: unknown target '$TARGET'" >&2
    print_targets >&2
    exit 2
    ;;
esac

command -v dfu-util >/dev/null 2>&1 || {
    echo "error: dfu-util not found (need >= 0.11)" >&2
    exit 1
}

if [[ "$NEED_RISCV" -eq 1 ]]; then
    : "${GNURISCV_TOOLCHAIN_PATH:?GNURISCV_TOOLCHAIN_PATH must point to the RISC-V toolchain root}"
fi

resolve_hpm5321_dfu_id() {
    local list has_5321=0 has_5322=0
    list="$(dfu-util -l 2>/dev/null || true)"
    if grep -qiE '\[0a511:5321\]' <<<"$list"; then
        has_5321=1
    fi
    if grep -qiE '\[0a511:5322\]' <<<"$list"; then
        has_5322=1
    fi
    if [[ "$has_5321" -eq 1 && "$has_5322" -eq 1 ]]; then
        echo "error: both HPM5321 PIDs are enumerated (0x5321 and 0x5322); unplug one" >&2
        echo "$list" | grep -iE 'a511:532[12]' >&2 || true
        exit 2
    fi
    if [[ "$has_5321" -eq 1 ]]; then
        echo "0xa511:0x5321"
        return
    fi
    if [[ "$has_5322" -eq 1 ]]; then
        echo "0xa511:0x5322"
        return
    fi
    echo ">> no 0x5321/0x5322 device listed yet; defaulting -d to 0x5322" >&2
    echo "   (single-CAN 0x5321: plug it in so dfu-util -l can see it, then re-run)" >&2
    echo "0xa511:0x5322"
}

echo ">> Building $BUILD_TARGET (preset: $PRESET, target: $TARGET)"
if [[ ${#CMAKE_EXTRA[@]} -gt 0 ]]; then
    cmake --preset "$PRESET" -S "$CMAKE_SRC" "${CMAKE_EXTRA[@]}"
else
    cmake --preset "$PRESET" -S "$CMAKE_SRC"
fi
cmake --build "$BUILD_DIR" --target "$BUILD_TARGET"

# mc02 bootloader is not flashed here -- it goes over SWD and normally never
# changes. It is still built so shared flash layout / metadata-record code cannot
# rot unseen, and so ozone/mc02.jdebug can open build/bootloader/mc02_bootloader.elf.
# Deliberately after the app and deliberately non-fatal.
if [[ -n "$BUILD_BOOTLOADER" && -z "${SKIP_BOOTLOADER:-}" ]]; then
    echo ">> Building $BUILD_BOOTLOADER (not flashed; SWD only)"
    cmake --build "$BUILD_DIR" --target "$BUILD_BOOTLOADER" ||
        echo "warning: $BUILD_BOOTLOADER failed to build; flashing the app anyway." >&2
fi

if [[ ! -f "$DFU_IMAGE" ]]; then
    echo "error: $DFU_IMAGE not found after the build." >&2
    exit 1
fi

if [[ "$TARGET" == hpm5321 ]]; then
    DFU_ID="$(resolve_hpm5321_dfu_id)"
fi

echo ">> $TARGET app image: $DFU_IMAGE"
echo ">> DFU devices currently visible:"
dfu-util -l 2>/dev/null | grep -iE "$DFU_GREP" ||
    echo "   (none in DFU mode yet; dfu-util will try to detach a running app)"

dfu-util -d "$DFU_ID" -a 0 -D "$DFU_IMAGE"

echo ">> Done. The bootloader verifies the image and resets into the app."
echo "   (A 'lost device' / status-read error from dfu-util at the end is normal.)"
