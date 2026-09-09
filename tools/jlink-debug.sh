#!/usr/bin/env bash
#
# One-key J-Link debug session: starts JLinkGDBServer for the selected board,
# connects the matching cross gdb to the board's ELF, and tears the server down
# when gdb exits. Terminal counterpart of the Ozone projects in ozone/*.jdebug
# and the VS Code configs in .vscode/launch.json.
#
# THESE DEBUG THE BOOTLOADERS. That is what a debug probe is for on these boards:
# applications are deployed over DFU (./tools/flash.sh <target>), while
# the bootloader is what you program and step through over SWD/JTAG. A bootloader
# also hands control to the application within milliseconds, so it is only
# observable from reset -- the bootloader targets therefore reset and break at
# main instead of attaching to whatever happens to be running.
#
# Nothing is programmed unless you pass LOAD=1. Programming a bootloader is safe
# (first flash sector only, metadata record and application untouched), but do
# NOT combine LOAD=1 with ELF=<an app image>: programming an app without
# refreshing its SHA-256 metadata record makes the bootloader reject it on the
# next cold boot (firmware/*/bootloader/src/flash/validation.hpp).
#
# Usage:
#   ./tools/jlink-debug.sh mc02              # mc02 bootloader,    STM32H723VGT6, SWD
#   ./tools/jlink-debug.sh c_board           # c_board bootloader, STM32F407IGH6, SWD
#   ./tools/jlink-debug.sh hpm6e8y           # bootloader,         HPM6E8Y,       JTAG
#   ./tools/jlink-debug.sh hpm5321           # hpm_board bootloader, HPM5361 (hpm5321 board),      JTAG
#   ./tools/jlink-debug.sh --list            # show the target table and exit
#
# Environment overrides (all optional):
#   ELF=<path>            debug a different image than the default
#   PORT=<n>              gdb server port (default 2331)
#   SPEED=<kHz|adaptive>  probe clock (default 4000)
#   JLINK_DEVICE=<name>   override the J-Link device name
#   IFACE=SWD|JTAG        override the probe interface
#   LOAD=1                run `load` after attaching (program via gdb)
#   NO_GDB=1              start only the gdb server (attach your own client/IDE)
#
# RTT: c_board and mc02 both link SEGGER RTT. While this session is up, open a
# second terminal and run `JLinkRTTClient` to see the log stream.
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ARM_GDB="${ARM_GDB:-/opt/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-gdb}"
RISCV_GDB="${RISCV_GDB:-$HOME/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux/bin/riscv32-unknown-elf-gdb}"

usage() {
    sed -n '2,41p' "${BASH_SOURCE[0]}" | sed 's/^#\s\?//'
}

print_targets() {
    printf '%-16s %-18s %-6s %s\n' TARGET DEVICE IFACE ELF
    printf '%-16s %-18s %-6s %s\n' mc02 STM32H723VGTx SWD \
        firmware/mc02/build/bootloader/mc02_bootloader.elf
    printf '%-16s %-18s %-6s %s\n' c_board STM32F407IGHx SWD \
        firmware/c_board/build/bootloader/c_board_bootloader.elf
    printf '%-16s %-18s %-6s %s\n' hpm6e8y HPM6E8YxGNx JTAG \
        firmware/hpm_board/build/bootloader/output/hpm_board_bootloader_hpm6e8y.elf
    printf '%-16s %-18s %-6s %s\n' hpm5321 HPM5321xEGx JTAG \
        firmware/hpm_board/build/bootloader/output/hpm_board_bootloader_hpm5321.elf
}

RESET_TO_MAIN=0
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
esac

# Per-target defaults. Every device name below exists in SEGGER's device database
# (verified against libjlinkarm.so).
case "$TARGET" in
mc02)
    DEFAULT_DEVICE="STM32H723VGTx"
    DEFAULT_IFACE="SWD"
    DEFAULT_ELF="$ROOT/firmware/mc02/build/bootloader/mc02_bootloader.elf"
    GDB="$ARM_GDB"
    RESET_TO_MAIN=1
    ;;
c_board)
    DEFAULT_DEVICE="STM32F407IGHx"
    DEFAULT_IFACE="SWD"
    DEFAULT_ELF="$ROOT/firmware/c_board/build/bootloader/c_board_bootloader.elf"
    GDB="$ARM_GDB"
    RESET_TO_MAIN=1
    ;;
hpm6e8y)
    # The name SEGGER's own tools offer for this part (register set RV32IFD, one
    # QSPI flash bank at 0x80000000). SEGGER also ships HPM6E8YxVMx for the other
    # package, and the HPM SDK's EVK profile is HPM6E80xVMx -- HPM6E80 being
    # register- and pin-identical to what is actually on the board, an HPM6E00
    # (firmware/hpm_board/boards/hpm6e8y/README.md, "HPM6E80 vs HPM6E00"). If a
    # connection fails, the package suffix is the first thing to try.
    DEFAULT_DEVICE="HPM6E8YxGNx"
    DEFAULT_IFACE="JTAG"
    DEFAULT_ELF="$ROOT/firmware/hpm_board/build/bootloader/output/hpm_board_bootloader_hpm6e8y.elf"
    GDB="$RISCV_GDB"
    RESET_TO_MAIN=1
    ;;
hpm5321)
    # hpm5321.yaml declares soc=HPM5361, device=
    # HPM5361xEGx -- that is the SDK's build target (register-compatible
    # superset), same relationship as hpm6e8y building against HPM6E80 while
    # the physical die is an HPM6E00 (see the hpm6e8y case above). The chip
    # actually soldered on these boards is marked HPM5321IEG1, so the J-Link
    # device name needs to match the real silicon: HPM5321xEGx (confirmed
    # against SEGGER's device DB, libjlinkarm). Do not "fix" this back to
    # HPM5361xEGx from the yaml -- that name is for the SDK, not the debugger.
    #
    # RESET_TO_MAIN below does a JTAG-level `monitor reset` + break-at-main. To
    # actually program the bootloader, pass LOAD=1 -- the J-Link now ships an
    # open flash loader for this part's XPI NOR ("HPM XPI Nor Flash XPI0", built
    # by SEGGER from the HPM SDK's hpm_xpi driver). That replaced the old
    # flash-dual-bootloader.sh, which drove OpenOCD because SEGGER used to ship
    # no loader for this part.
    DEFAULT_DEVICE="HPM5321xEGx"
    DEFAULT_IFACE="JTAG"
    DEFAULT_ELF="$ROOT/firmware/hpm_board/build/bootloader/output/hpm_board_bootloader_hpm5321.elf"
    GDB="$RISCV_GDB"
    RESET_TO_MAIN=1
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

DEVICE="${JLINK_DEVICE:-$DEFAULT_DEVICE}"
IFACE="${IFACE:-$DEFAULT_IFACE}"
ELF="${ELF:-$DEFAULT_ELF}"
PORT="${PORT:-2331}"
SPEED="${SPEED:-4000}"

command -v JLinkGDBServerCLExe >/dev/null 2>&1 || {
    echo "error: JLinkGDBServerCLExe not found. Install the SEGGER J-Link tools." >&2
    exit 1
}
[[ -x "$GDB" ]] || {
    echo "error: gdb not found or not executable: $GDB" >&2
    echo "       override with ARM_GDB= / RISCV_GDB=." >&2
    exit 1
}
[[ -f "$ELF" ]] || {
    echo "error: $ELF not found -- build the target first, or pass ELF=<path>." >&2
    exit 1
}

SERVER_PID=""
cleanup() {
    if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
        kill "$SERVER_PID" 2>/dev/null || true
        wait "$SERVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo ">> target : $TARGET"
echo ">> device : $DEVICE  (interface $IFACE @ ${SPEED}kHz)"
echo ">> image  : $ELF"
echo ">> port   : $PORT"

# -singlerun makes the server exit once the gdb client disconnects, so a session
# never leaves a stale server holding the probe.
JLinkGDBServerCLExe \
    -select USB \
    -device "$DEVICE" \
    -endian little \
    -if "$IFACE" \
    -speed "$SPEED" \
    -port "$PORT" \
    -swoport $((PORT + 1)) \
    -telnetport $((PORT + 2)) \
    -nogui \
    -singlerun &
SERVER_PID=$!

# Wait for the port to accept connections rather than sleeping a fixed amount:
# probe enumeration and target detection take a variable amount of time.
for _ in $(seq 1 50); do
    if (exec 3<>"/dev/tcp/127.0.0.1/$PORT") 2>/dev/null; then
        exec 3<&- 3>&-
        break
    fi
    kill -0 "$SERVER_PID" 2>/dev/null || {
        echo "error: gdb server exited during startup (see its output above)." >&2
        exit 1
    }
    sleep 0.2
done

if [[ -n "${NO_GDB:-}" ]]; then
    echo ">> server only (NO_GDB set). Connect with: target extended-remote :$PORT"
    echo ">> Ctrl-C to stop."
    wait "$SERVER_PID"
    exit 0
fi

GDB_ARGS=(-q "$ELF" -ex "target extended-remote 127.0.0.1:$PORT")
if [[ -n "${LOAD:-}" ]]; then
    # NOTE for hpm6e8y: its flash is XPI0/QSPI NOR (README "HPM6E00 的 Flash 对
    # J-Link 来说是 QSPI"), so a gdb `load` needs the J-Link flash algorithm for
    # that profile -- which the J-Link now ships ("HPM XPI Nor Flash XPI0"), so
    # LOAD works where it used to fail.
    GDB_ARGS+=(-ex "load")
fi
if [[ "$RESET_TO_MAIN" == 1 ]]; then
    # A bootloader is gone from the CPU within milliseconds of reset, so plain
    # attach-to-running would land in application code where its symbols are
    # meaningless. Reset and stop at its main instead.
    GDB_ARGS+=(-ex "monitor reset" -ex "tbreak main" -ex "continue")
fi

"$GDB" "${GDB_ARGS[@]}"
