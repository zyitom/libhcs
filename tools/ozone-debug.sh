#!/usr/bin/env bash
#
# Open Ozone on a pre-configured project -- no New Project Wizard, no device
# picking. The projects live in ozone/*.jdebug and already carry the device,
# interface, speed, connect mode and program file.
#
# Usage:
#   ./tools/ozone-debug.sh mc02             # STM32H723VGT6 bootloader, SWD
#   ./tools/ozone-debug.sh mc02 app         # STM32H723VGT6 application, SWD
#   ./tools/ozone-debug.sh c_board          # STM32F407IGH6 bootloader, SWD
#   ./tools/ozone-debug.sh c_board app      # STM32F407IGH6 application, SWD
#   ./tools/ozone-debug.sh hpm6e8y          # HPM6E8Y core0 bootloader, JTAG
#   ./tools/ozone-debug.sh hpm6e8y app      # HPM6E8Y application, JTAG
#   ./tools/ozone-debug.sh hpm5321          # hpm_board hpm5321 bootloader, JTAG (attach)
#   ./tools/ozone-debug.sh hpm5321 app      # hpm_board hpm5321 application, JTAG (attach)
#   ./tools/ozone-debug.sh --list
#
# Omit the image selector to use the project default image. Passing "app" creates
# a temporary attach-mode project for the board application ELF.
#
# Ozone is started detached: the shell returns immediately and Ozone survives
# closing the terminal. Its stdout/stderr go to the log file printed on start.
#
# ch32_board is deliberately absent: it is debugged over WCH-Link, which Ozone
# does not speak. Use the VS Code "ch32 - Debug V3F boot core" configuration, or
# ./tools/jlink-debug.sh for the command-line route on the J-Link boards.
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="$ROOT/ozone"
CACHE_DIR="${TMPDIR:-/tmp}/libhcs-ozone-${UID:-$(id -u)}"

OZONE="${OZONE:-}"
if [[ -z "$OZONE" ]]; then
    # Prefer whatever is on PATH, else the newest versioned install under /opt.
    if command -v Ozone >/dev/null 2>&1; then
        OZONE="$(command -v Ozone)"
    else
        for candidate in $(ls -d /opt/SEGGER/Ozone* 2>/dev/null | sort -r); do
            if [[ -x "$candidate/Ozone" ]]; then
                OZONE="$candidate/Ozone"
                break
            fi
        done
    fi
fi

list_projects() {
    echo "Available projects (ozone/):"
    for f in "$PROJECT_DIR"/*.jdebug; do
        [[ -e "$f" ]] || continue
        name="$(basename "$f" .jdebug)"
        device="$(sed -n 's/.*Project\.SetDevice *("\([^"]*\)").*/\1/p' "$f" | head -1)"
        iface="$(sed -n 's/.*Project\.SetTargetIF *("\([^"]*\)").*/\1/p' "$f" | head -1)"
        printf '  %-16s %-18s %s\n' "$name" "$device" "$iface"
    done
}

usage() {
    sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^#\s\?//'
}

app_elf_for_project() {
    case "$1" in
    mc02)
        printf '%s\n' "$ROOT/firmware/mc02/build/app/mc02_app.elf"
        ;;
    c_board)
        printf '%s\n' "$ROOT/firmware/c_board/build/app/c_board_app.elf"
        ;;
    hpm6e8y)
        printf '%s\n' "$ROOT/firmware/hpm_board/build/app/output/hpm_board_app_hpm6e8y.elf"
        ;;
    hpm5321)
        printf '%s\n' "$ROOT/firmware/hpm_board/build/app/output/hpm_board_app_hpm5321.elf"
        ;;
    *)
        return 1
        ;;
    esac
}

sed_replacement_escape() {
    printf '%s' "$1" | sed -e 's/[&|\\]/\\&/g'
}

make_app_project() {
    local project_name="$1"
    local source_project="$2"
    local app_elf="$3"
    local run_project="$CACHE_DIR/$project_name-app.jdebug"
    local escaped_app_elf

    mkdir -p "$CACHE_DIR"
    escaped_app_elf="$(sed_replacement_escape "$app_elf")"
    sed -E \
        -e 's|Debug\.SetConnectMode *\([^)]*\);|Debug.SetConnectMode (CM_ATTACH_HALT);|' \
        -e "s|File\.Open *\(\"[^\"]*\"\);|File.Open (\"$escaped_app_elf\");|" \
        "$source_project" >"$run_project"
    printf '%s\n' "$run_project"
}

launch_ozone() {
    local run_project="$1"
    local log_file="$2"
    local pid

    if command -v setsid >/dev/null 2>&1; then
        setsid "$OZONE" "$run_project" </dev/null >"$log_file" 2>&1 &
    else
        "$OZONE" "$run_project" </dev/null >"$log_file" 2>&1 &
    fi

    pid="$!"
    disown "$pid" 2>/dev/null || true
    echo ">> Ozone started in background (pid $pid)"
    echo ">> log: $log_file"
}

if (($# > 2)); then
    echo "error: too many arguments" >&2
    echo >&2
    usage >&2
    exit 1
fi

case "${1:-}" in
-h | --help)
    usage
    exit 0
    ;;
--list)
    list_projects
    exit 0
    ;;
"")
    echo "error: no project given" >&2
    echo >&2
    list_projects >&2
    exit 1
    ;;
esac

NAME="$1"
IMAGE="${2:-default}"

PROJECT="$PROJECT_DIR/$NAME.jdebug"
[[ -f "$PROJECT" ]] || {
    echo "error: no such project: $PROJECT" >&2
    echo >&2
    list_projects >&2
    exit 1
}

[[ -n "$OZONE" && -x "$OZONE" ]] || {
    echo "error: Ozone not found. Set OZONE=/path/to/Ozone." >&2
    exit 1
}

case "$IMAGE" in
default)
    RUN_PROJECT="$PROJECT"
    # Project-relative, as written in the .jdebug.
    ELF="$PROJECT_DIR/$(sed -n 's/.*File\.Open *("\([^"]*\)").*/\1/p' "$PROJECT" | head -1)"
    ;;
app)
    ELF="$(app_elf_for_project "$NAME")" || {
        echo "error: no application ELF known for project: $NAME" >&2
        exit 1
    }
    RUN_PROJECT="$(make_app_project "$NAME" "$PROJECT" "$ELF")"
    ;;
*)
    echo "error: unknown image selector: $IMAGE (expected \"app\", or nothing)" >&2
    echo >&2
    usage >&2
    exit 1
    ;;
esac

# Warn instead of failing: Ozone can still open with a stale or missing image,
# and sometimes that is what you want (e.g. inspecting a board you did not build
# for on this machine).
if [[ ! -f "$ELF" ]]; then
    echo "warning: program file not built yet: $ELF" >&2
fi

mkdir -p "$CACHE_DIR"
echo ">> $OZONE $RUN_PROJECT"
launch_ozone "$RUN_PROJECT" "$CACHE_DIR/$NAME-$IMAGE.log"
