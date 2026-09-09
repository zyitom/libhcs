#!/usr/bin/env bash
#
# Host-side latency tuning for the libhcs USB bulk transport.
#
# WHY THIS EXISTS: every setting below is runtime-only and reverts on reboot, and
# the machine's own facts (which xHCI controller the board landed on, which IRQ
# serves it, whether the isolated core is a P-core or an E-core) change without
# anyone editing a doc. So this script DISCOVERS rather than hardcodes, and
# reports what it found even when it changes nothing.
#
# WHAT IS MEASURED AND WHAT IS NOT -- read before believing any of it.
# Host-path numbers below come from host/examples/usb_ep0_rtt.cpp (3000 samples,
# 1 ms gap, SCHED_FIFO 80, pinned); board+USB RTT numbers come from
# can_loopback_latency. Full write-up and the negative results in HOST_TUNING.md.
#
#   CPU frequency       MEASURED, LARGEST SINGLE EFFECT *ON A MACHINE WITHOUT
#                       isolcpus*. On TL101 it is worth ZERO, twice measured
#                       (2026-08-21 and 2026-09-05, 4 interleaved rounds each):
#                       isolcpus=7 + SCHED_FIFO already pins the core at 4100 MHz,
#                       so the governor has nothing left to do. The real
#                       requirement is "something must hold the frequency up",
#                       and either mechanism satisfies it. See HOST_TUNING.md
#                       11.3 and 11.11.3 before quoting the number below.
#                       p50 77.5 -> 67.5 us,
#                       p99 ~100 -> ~82 us, reproduced over 3 rounds. A
#                       SCHED_FIFO thread that SLEEPS between transfers does NOT
#                       hold the clock up: under powersave the core sat at
#                       1.8-2.3 GHz and only reached 4.1 GHz once something
#                       stopped it from idling. This supersedes the older
#                       "governor: measured, no effect" note -- see HOST_TUNING.md
#                       section 4 for why the CAN-loopback test could not see it.
#   PM QoS / C-states   MEASURED TWICE, AND THE SECOND MEASUREMENT MATTERS MORE.
#                       On the HOST path with no idle gap it is the same effect as
#                       the governor, not an additional one: pm_qos=0 gives p50
#                       68.3, governor alone 67.5, both together 68.3.
#                       BUT ON THE BOARD PATH AT A REALISTIC 1 kHz DUTY CYCLE
#                       (HCS_LATENCY_GAP_US=1000, i.e. the core really does go
#                       idle between samples) it buys real tail: p99.9 139.0 ->
#                       129.9 us and p99 128.5 -> 126.8, three rounds each, p50
#                       unchanged at 122. Measured 2026-08-07, two HPM5321.
#                       The older "only worth ~2 us at p99" note was taken at
#                       gap=0 and understates this for control-loop traffic.
#                       Still NOT applied by this script -- it needs a process
#                       holding the fd (see --pmqos) and costs power machine-wide
#                       -- but a 1 kHz loop that cares about p99.9 should run it.
#   Per-core C-state    MEASURED, AND THE CORE YOU PICK IS THE WHOLE STORY.
#     disable           On the APPLICATION core it does nothing (p50 77.8 vs 77.4
#                       baseline). Disabling C6/C10 on ALL cores while leaving
#                       C1E enabled bought only a small tail improvement (p99 ~94
#                       vs ~100) and NO p50 change -- C1E alone is enough to let
#                       the clock fall.
#                       BUT ON THE xHCI IRQ THREAD'S CORE IT IS THE BEST KNOB WE
#                       HAVE. Measured 2026-08-26 on TL101 (both boards on
#                       00:14.0, irq/136 pinned to cpu4): disabling C1_ACPI on
#                       cpu4 alone took usb_ep0_rtt p50 from 94.8-99.7 to
#                       74.4-75.3 us, p99 111-120 -> 91.2-93.0. That BEATS
#                       machine-wide PM QoS (77.0-78.6) while costing one idle
#                       core instead of the whole package.
#                       Root cause is C1E (MSR 0x1FC bit1 = 1; BIOS option
#                       "Enhanced C-states"), NOT package C-states: Pkg%pc2 and
#                       Pkg%pc6 stay at 0.00 throughout, because cpu7 never
#                       idles. Setting governor=performance on cpu4 does NOT
#                       help (p50 95.7-96.3) -- it does not stop the core from
#                       halting. See HOST_TUNING.md 11.9.
#                       The IRQ thread's core changes with enumeration order, so
#                       discover it, never hardcode cpu4:
#                         pgrep -f 'irq/.*xhci' | while read p; do
#                           grep Cpus_allowed_list /proc/$p/status; done
#   cpuidle governor    MEASURED, NO EFFECT. menu -> teo moved nothing outside
#     menu -> teo       noise. Reported below, not changed.
#   Bluetooth on the    MEASURED, NO EFFECT on the host path. btusb shares the
#     board's root hub  board's root hub and does inject periodic split
#                       transactions, but unbinding it moved nothing. Reported
#                       below so the neighbour is at least visible.
#   RT throttling off   Assumed necessary, inherited from earlier work (a ~50 ms
#                       max latency was traced to it). NOT re-measured here.
#   xHCI IRQ thread     MEASURED, NO EFFECT WITH ONE BOARD; REQUIRED WITH TWO.
#     RT priority       See HOST_TUNING.md 4.3: two boards on one controller share
#                       one IRQ thread, and at the RT default of FIFO 50 both
#                       boards' max latency goes to ~1 ms. This script prints the
#                       discovered IRQ and its priority; it does not raise it,
#                       because for the single-board case that is a no-op.
#   USB power/control   Set to on. NOT measured as a delta -- runtime_status was
#                       already 'active', so nothing was ever suspending. Applied
#                       to close off a class of surprise.
#   irqbalance          Must stay OFF (it would migrate the xHCI IRQ at runtime).
#
# EXPLICITLY DISPROVEN -- do NOT add these back:
#   Pinning the xHCI IRQ to the same core as the libusb event thread:
#   p50 124.8 -> 146.2 us, p90 126 -> 157 us. The ISR and the event thread
#   serialize on one core; that costs far more than the cross-core wakeup saves.
#   Leave the xHCI IRQ and the event thread on DIFFERENT cores.
#
# Usage:
#   sudo ./host-tuning.sh              apply everything except PM QoS, then report
#   sudo ./host-tuning.sh --check      report only, change nothing
#   sudo ./host-tuning.sh --pmqos      hold C-states off in the foreground
#                                      (Ctrl-C to release; run in a second
#                                      terminal for the duration of a measurement)
#
set -euo pipefail

USB_VENDOR_ID="a511"   # every libhcs board; the product id differs per board
CHECK_ONLY=0
PMQOS_ONLY=0
case "${1:-}" in
    --check) CHECK_ONLY=1 ;;
    --pmqos) PMQOS_ONLY=1 ;;
    "") ;;
    *) echo "usage: $0 [--check|--pmqos]" >&2; exit 1 ;;
esac

if [[ "$PMQOS_ONLY" == "0" && "$CHECK_ONLY" == "0" && "$EUID" -ne 0 ]]; then
    echo "error: must run as root to apply (use --check to only report)" >&2
    exit 1
fi

ok()   { printf '  \033[32m[ok]\033[0m    %s\n' "$1"; }
warn() { printf '  \033[33m[warn]\033[0m  %s\n' "$1"; }
info() { printf '  [info]  %s\n' "$1"; }

# Expand a kernel CPU list ("6,7" / "12-21" / "0-5,8") into separate numbers.
expand_cpu_list() {
    local out="" part parts
    IFS=',' read -ra parts <<<"$1"
    for part in "${parts[@]}"; do
        if [[ "$part" == *-* ]]; then
            out+=" $(seq "${part%-*}" "${part#*-}")"
        else
            out+=" $part"
        fi
    done
    echo "$out"
}

# --- PM QoS: hold deep C-states off for as long as this process lives --------
# Kept in this script rather than a systemd unit on purpose: it costs real power
# machine-wide. On the host path the governor captures nearly all of the same
# benefit, but on the BOARD path at a 1 kHz duty cycle it is worth 9 us of p99.9
# on its own (139.0 -> 129.9) -- see the table at the top. Hold it for the
# duration of a measurement, and for production only if p99.9 is a real
# requirement.
if [[ "$PMQOS_ONLY" == "1" ]]; then
    if [[ "$EUID" -ne 0 ]]; then
        echo "error: --pmqos must run as root" >&2
        exit 1
    fi
    echo ">> Holding /dev/cpu_dma_latency at 0 (all idle states blocked)."
    echo "   Run the plain script first: on the host path the governor already"
    echo "   captures most of this, at a fraction of the power."
    echo "   What this ADDS, measured at a 1 kHz duty cycle on two HPM5321:"
    echo "     p99.9 139.0 -> 129.9 us,  p99 128.5 -> 126.8,  p50 unchanged."
    echo "   Leave this running for the duration of the measurement; Ctrl-C releases."
    exec python3 -c '
import os, struct, signal
fd = os.open("/dev/cpu_dma_latency", os.O_WRONLY)
os.write(fd, struct.pack("i", 0))
signal.pause()
'
fi

echo "== Kernel boot parameters (cannot be changed at runtime) =="
CMDLINE="$(cat /proc/cmdline)"
# Absence is a deliberate choice here, not a misconfiguration. MEASURED
# 2026-08-04 (HOST_TUNING.md 1.3, 2x2 interleaved, 3 rounds x 20000 samples):
# isolating the event thread's physical P-core buys min -1us, p99 -0.8us,
# p99.9 -2..4us on a ~100us board path of which 50.3us is fixed CAN wire time.
# That is 1-3%, paid for with a whole physical P-core (2 of 12 P threads).
# On a machine that also runs compute the trade is usually not worth it, so this
# reports the state and the price instead of demanding one answer.
MISSING_ISOLATION=""
for want in isolcpus nohz_full rcu_nocbs; do
    if grep -q "$want=" <<<"$CMDLINE"; then
        ok "$want present: $(grep -o "${want}=[^ ]*" <<<"$CMDLINE")"
    else
        MISSING_ISOLATION+="${MISSING_ISOLATION:+ }$want"
    fi
done
if [[ -n "$MISSING_ISOLATION" ]]; then
    info "not set: $MISSING_ISOLATION -- fine, that is this machine's choice."
    info "  Isolating the event thread's P-core measured min -1us, p99 -0.8us,"
    info "  p99.9 -2..4us; the price is a whole physical P-core. See 1.3."
    info "  PINNING the event thread to a P-core still matters and is not this."
fi
if uname -r | grep -qi "realtime\|rt"; then
    ok "PREEMPT_RT kernel: $(uname -r)"
else
    warn "not an RT kernel ($(uname -r)); tail latency will be worse"
fi

# On Intel hybrid parts an isolated E-core is a downgrade, not a tuning: lower
# IPC and a lower clock ceiling than any P-core. cpu_core/cpu_atom are the
# kernel's own classification, so this needs no CPU model table.
if [[ -r /sys/devices/cpu_core/cpus && -r /sys/devices/cpu_atom/cpus ]]; then
    P_CORES="$(cat /sys/devices/cpu_core/cpus)"
    E_CORES="$(cat /sys/devices/cpu_atom/cpus)"
    info "hybrid CPU: P-cores $P_CORES, E-cores $E_CORES"
    ISOL="$(grep -o 'isolcpus=[^ ]*' <<<"$CMDLINE" | cut -d= -f2- || true)"
    if [[ -n "$ISOL" ]]; then
        E_LIST=" $(expand_cpu_list "$E_CORES") "
        ISOL_LIST=" $(expand_cpu_list "$ISOL") "
        BAD=""
        for cpu in $ISOL_LIST; do
            grep -qw "$cpu" <<<"$E_LIST" && BAD+="$cpu "
        done
        if [[ -n "$BAD" ]]; then
            warn "isolcpus lands on E-core(s): $BAD -- isolate a P-core instead"
            warn "  (an isolated E-core gives the busy-poll thread lower IPC and clock)"
        else
            ok "isolated cores are P-cores"
        fi

        # Isolating one SMT thread but not its sibling leaves the shared L1/L2
        # and execution units open to anything the scheduler puts on the sibling.
        # MEASURED (HOST_TUNING.md 1.3): isolating the whole physical core is
        # worth ~5 us of p99 and ~8 us of p99.9. It does nothing for p50.
        LONELY=""
        for cpu in $ISOL_LIST; do
            SIB_FILE="/sys/devices/system/cpu/cpu$cpu/topology/thread_siblings_list"
            [[ -r "$SIB_FILE" ]] || continue
            for sib in $(expand_cpu_list "$(cat "$SIB_FILE")"); do
                grep -qw "$sib" <<<"$ISOL_LIST" || LONELY+="cpu$cpu's sibling cpu$sib; "
            done
        done
        if [[ -n "$LONELY" ]]; then
            warn "isolcpus splits an SMT pair -- not isolated: $LONELY"
            warn "  (the sibling shares L1/L2 and execution units with the event"
            warn "   thread; isolating both measured ~5 us of p99, HOST_TUNING.md 1.3)"
        else
            ok "isolated cores are whole physical cores (SMT siblings included)"
        fi

        # `irqaffinity=` on the cmdline only sets the BOOT-TIME default. Multi-queue
        # drivers (nvme, iwlwifi, igc, ...) call irq_set_affinity_hint at runtime and
        # spread one queue per CPU, which lands them right back on the isolated cores.
        # MEASURED on this machine 2026-08-04: with isolcpus=6,7 and
        # irqaffinity=0-5,8-21 set, nvme0q7+iwlwifi:queue_7 were still pinned to cpu6
        # and nvme0q8+iwlwifi:queue_8 to cpu7 -- i.e. disk and wifi interrupts on the
        # USB event thread's core, the two burstiest sources on a laptop.
        # Housekeeping target = every P-core that is NOT isolated. Building this by
        # trimming the P-core range textually would leave the isolated cores in it,
        # which moves an IRQ onto exactly the core it had to leave.
        HOUSEKEEPING=""
        for cpu in $(expand_cpu_list "$P_CORES"); do
            grep -qw "$cpu" <<<"$ISOL_LIST" && continue
            HOUSEKEEPING+="${HOUSEKEEPING:+,}$cpu"
        done
        [[ -n "$HOUSEKEEPING" ]] || HOUSEKEEPING="0"
        INTRUDERS="" MANAGED=""
        for dir in /proc/irq/[0-9]*; do
            [[ -r "$dir/smp_affinity_list" ]] || continue
            irq="${dir##*/}"
            aff="$(cat "$dir/smp_affinity_list" 2>/dev/null)" || continue
            hit=""
            for cpu in $(expand_cpu_list "$aff"); do
                grep -qw "$cpu" <<<"$ISOL_LIST" && hit=1
            done
            [[ -n "$hit" ]] || continue
            name="$(find "$dir" -maxdepth 1 -mindepth 1 -printf '%f\n' 2>/dev/null \
                | grep -vE '^(affinity_hint|effective_affinity|effective_affinity_list|node|smp_affinity|smp_affinity_list|spurious)$' \
                | head -1)"
            if [[ "$CHECK_ONLY" == "1" ]]; then
                INTRUDERS+="$irq(${name:-?}) "
            elif echo "$HOUSEKEEPING" > "$dir/smp_affinity_list" 2>/dev/null; then
                INTRUDERS+="$irq(${name:-?}) "
            else
                # blk-mq managed interrupts (nvme queues) refuse manual reassignment:
                # the kernel owns one queue per CPU and there is no way to evict them
                # short of reducing the queue count at probe time.
                MANAGED+="$irq(${name:-?}) "
            fi
        done
        if [[ -n "$INTRUDERS" ]]; then
            if [[ "$CHECK_ONLY" == "1" ]]; then
                warn "IRQs sitting on isolated cores: $INTRUDERS"
                warn "  (run without --check to move them to $HOUSEKEEPING)"
            else
                ok "moved IRQs off the isolated cores: $INTRUDERS"
            fi
        fi
        if [[ -n "$MANAGED" ]]; then
            warn "kernel-managed IRQs stuck on isolated cores: $MANAGED"
            warn "  (blk-mq per-CPU queues; smp_affinity is read-only for these."
            warn "   Only fewer queues at probe time would move them.)"
        fi
        if [[ -z "$INTRUDERS$MANAGED" ]]; then
            ok "no IRQ is bound to an isolated core"
        fi
    fi
fi

echo "== RT scheduler =="
RT_RUNTIME="$(cat /proc/sys/kernel/sched_rt_runtime_us)"
if [[ "$RT_RUNTIME" == "-1" ]]; then
    ok "sched_rt_runtime_us = -1 (throttling off)"
elif [[ "$CHECK_ONLY" == "1" ]]; then
    warn "sched_rt_runtime_us = $RT_RUNTIME (throttling ON -- run without --check)"
else
    echo -1 > /proc/sys/kernel/sched_rt_runtime_us
    ok "sched_rt_runtime_us: $RT_RUNTIME -> -1"
fi

echo "== CPU frequency (worth ~10 us of p50 ONLY without isolcpus; 0 on this box, see HOST_TUNING.md 11.3/11.11.3) =="
GOV="$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo n/a)"
if [[ "$GOV" == "n/a" ]]; then
    info "no cpufreq sysfs; skipping"
elif [[ "$GOV" == "performance" ]]; then
    ok "governor already performance"
elif [[ "$CHECK_ONLY" == "1" ]]; then
    warn "governor = $GOV -- run without --check to fix. NOTE: worth ~10 us of p50 only when nothing else holds the clock up; with isolcpus + SCHED_FIFO (this box) it is measured at ZERO"
else
    for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        echo performance > "$g" 2>/dev/null || true
    done
    ok "governor: $GOV -> performance"
fi

echo "== Idle states (context for --pmqos; nothing changed here) =="
IDLE_GOV="$(cat /sys/devices/system/cpu/cpuidle/current_governor 2>/dev/null || echo n/a)"
info "cpuidle governor: $IDLE_GOV (menu vs teo measured identical here)"
for s in /sys/devices/system/cpu/cpu0/cpuidle/state*/; do
    [[ -r "$s/name" ]] || continue
    info "  $(cat "$s/name") exit latency $(cat "$s/latency") us"
done

echo "== USB board =="
BOARD_CONTROLLERS=""
# Match on vendor id only: the product id differs per board (5321 HPM5321 single
# CAN, 5322 HPM5321 dual CAN-FD, 6e84 hpm6e8y), and hardcoding one silently reports
# "board not enumerated" the moment a different board is plugged in.
FOUND_BOARD=0
for d in /sys/bus/usb/devices/*/; do
    [[ "$(cat "$d/idVendor" 2>/dev/null)" == "$USB_VENDOR_ID" ]] || continue
    FOUND_BOARD=1
    PID="$(cat "$d/idProduct" 2>/dev/null || echo '?')"
    BUS="$(cat "$d/busnum" 2>/dev/null || echo '?')"
    SPEED="$(cat "$d/speed" 2>/dev/null || echo '?')"
    PRODUCT="$(cat "$d/product" 2>/dev/null || echo '')"
    CTRL="$(readlink -f "$d" | grep -o '0000:[0-9a-f]*:[0-9a-f]*\.[0-9a-f]*' | tail -1)"
    ok "$USB_VENDOR_ID:$PID on bus $BUS at ${SPEED}M via ${CTRL:-unknown}  $PRODUCT"

    PCTL="$(cat "$d/power/control" 2>/dev/null || echo n/a)"
    if [[ "$PCTL" == "on" ]]; then
        ok "  power/control = on (autosuspend disabled)"
    elif [[ "$CHECK_ONLY" == "1" ]]; then
        warn "  power/control = $PCTL"
    else
        echo on > "$d/power/control" && ok "  power/control: $PCTL -> on"
    fi

    [[ -n "$CTRL" ]] && BOARD_CONTROLLERS+=" $CTRL"

    # Anything else on the same root hub competes for the same microframes.
    # Measured no effect for btusb here, but a device with real periodic
    # bandwidth (a camera streaming isoc) is a different story, so show it.
    if [[ "$BUS" != "?" ]]; then
        NEIGHBOURS="$(lsusb -t 2>/dev/null | awk -v b="$BUS" '
            /^\/:/ { inbus = ($0 ~ ("Bus 00" b "\\.")) ; next }
            inbus && /Driver=/ { print }' | grep -v "Driver=\[none\]" | wc -l)"
        [[ "$NEIGHBOURS" -gt 0 ]] \
            && info "  $NEIGHBOURS other device interface(s) share this root hub"
    fi
done
[[ "$FOUND_BOARD" == "0" ]] && warn "no $USB_VENDOR_ID:* board enumerated"

# xHCI IRQ priority. Boards sharing one controller also share ONE IRQ thread, and
# at the RT default of FIFO 50 that thread is where their tail latency goes.
# MEASURED on two 5321 DualCan boards on 00:14.0, 1 kHz duty cycle, two rounds:
# max 189.9/222.3 us at FIFO 50 -> 126.3/141.4 us at FIFO 90, p50 unchanged.
# With a single board it is a no-op (measured), so only act when sharing.
echo "== xHCI IRQ priority =="
for ctrl in $(tr ' ' '\n' <<<"$BOARD_CONTROLLERS" | grep -v '^$' | sort -u); do
    COUNT="$(tr ' ' '\n' <<<"$BOARD_CONTROLLERS" | grep -c "^$ctrl$")"
    [[ -d "/sys/bus/pci/devices/$ctrl/msi_irqs" ]] || continue
    for irq in /sys/bus/pci/devices/"$ctrl"/msi_irqs/*; do
        n="$(basename "$irq")"
        grep -q "^ *$n:.*xhci" /proc/interrupts || continue
        TH="$(ps -eo pid,rtprio,comm --no-headers 2>/dev/null | grep "irq/$n-xhci" || true)"
        [[ -n "$TH" ]] || continue
        TH_PID="$(awk '{print $1}' <<<"$TH")"
        TH_PRIO="$(awk '{print $2}' <<<"$TH")"
        if [[ "$COUNT" -lt 2 ]]; then
            info "$ctrl: 1 board on IRQ $n (rtprio $TH_PRIO) -- raising it is a no-op here"
        elif [[ "$TH_PRIO" -ge 90 ]]; then
            ok "$ctrl: $COUNT boards share IRQ $n, already rtprio $TH_PRIO"
        elif [[ "$CHECK_ONLY" == "1" ]]; then
            warn "$ctrl: $COUNT boards share IRQ $n at rtprio $TH_PRIO -- costs ~60-80 us of max"
        else
            chrt -f -p 90 "$TH_PID" 2>/dev/null \
                && ok "$ctrl: $COUNT boards share IRQ $n, rtprio $TH_PRIO -> 90" \
                || warn "$ctrl: could not raise IRQ $n priority"
        fi

        # WHICH core that thread lands on matters more than its priority.
        # MEASURED 2026-08-04, A/B/A on two 5321 DualCan, 20000 samples each:
        # IRQ thread on an E-core gives min 83.2 / p50 121.4 / avg 120.7 us; the
        # same run with it on a P-core gives min 76.3-77.0 / p50 100.3-100.4 /
        # avg 102.1-103.4, and moving it back reproduces the loss. That is ~21 us
        # of p50 -- larger than the governor, and nobody had checked it because
        # section 2 only ever tested "same core as the event thread" (harmful).
        # With no irqaffinity= on the cmdline the kernel is free to pick an
        # E-core, which is exactly what it did here.
        # Detection only needs to READ; --check is routinely run unprivileged, so
        # gating the whole block on writability silently hid the diagnosis.
        if [[ -r /sys/devices/cpu_core/cpus && -r "/proc/irq/$n/smp_affinity_list" ]]; then
            IRQ_AFF="$(cat "/proc/irq/$n/smp_affinity_list")"
            # effective_affinity_list, not ps's psr. An MSI vector can target only
            # ONE APIC id, so writing a SET to smp_affinity_list makes the kernel
            # elect one CPU from it -- that election is what effective_affinity
            # reports, and it is the core that actually takes the interrupt.
            # psr only says where the thread last ran, so right after an affinity
            # change it still shows the old core until the thread is next woken;
            # trusting it reports success while the interrupt is still on an E-core.
            IRQ_CORE="$(cat "/proc/irq/$n/effective_affinity_list" 2>/dev/null)"
            [[ "$IRQ_CORE" =~ ^[0-9]+$ ]] || IRQ_CORE="$(ps -eLo psr,comm --no-headers 2>/dev/null \
                | awk -v p="irq/$n-xhci" '$2 ~ p {print $1; exit}')"
            P_LIST=" $(expand_cpu_list "$(cat /sys/devices/cpu_core/cpus)") "
            # cpu_atom lumps two very different tiers together. On this Meteor Lake
            # the E-cores split into 3800 MHz compute-tile cores and 2500 MHz LP
            # cores on the SoC tile, and only the latter really hurts: MEASURED
            # p50 100.4 (P, 4500) / 100.8..119.2 (E, 3800, right on the threshold)
            # / 121.3 (LP E, 2500). Report the clock so the diagnosis is not just
            # "an E-core" when the tiers differ by 44%.
            IRQ_MHZ="$(($(cat "/sys/devices/system/cpu/cpu$IRQ_CORE/cpufreq/cpuinfo_max_freq" \
                2>/dev/null || echo 0) / 1000))"
            [[ "$IRQ_MHZ" == "0" ]] && IRQ_MHZ="?"
            if [[ -n "$IRQ_CORE" ]] && ! grep -qw "$IRQ_CORE" <<<"$P_LIST"; then
                if [[ "$CHECK_ONLY" == "1" ]]; then
                    warn "  IRQ $n thread runs on cpu$IRQ_CORE (E-core, ${IRQ_MHZ} MHz max) -- up to ~21 us of p50"
                    warn "  (run without --check to move it to a P-core)"
                elif echo "$(cat /sys/devices/cpu_core/cpus)" \
                    > "/proc/irq/$n/smp_affinity_list" 2>/dev/null; then
                    ok "  IRQ $n thread was on E-core cpu$IRQ_CORE (${IRQ_MHZ} MHz), affinity $IRQ_AFF -> P-cores"
                else
                    warn "  IRQ $n thread on E-core cpu$IRQ_CORE and affinity is not writable"
                fi
            elif [[ -n "$IRQ_CORE" ]]; then
                ok "  IRQ $n thread is on P-core cpu$IRQ_CORE (${IRQ_MHZ} MHz)"
            fi
        fi
        break
    done
done

# The root hub defaults to power/control=auto. It was never observed suspending
# (a hub cannot while a child is active), so this is hardening rather than a fix.
for hub in /sys/bus/usb/devices/usb*/power/control; do
    [[ -w "$hub" ]] || continue
    if [[ "$(cat "$hub")" != "on" && "$CHECK_ONLY" == "0" ]]; then
        echo on > "$hub" 2>/dev/null || true
    fi
done
if [[ "$CHECK_ONLY" == "0" ]]; then
    ok "root hubs power/control -> on"
fi

echo "== IOMMU (per-URB dma_map cost; NOT measured, see HOST_TUNING.md 5) =="
IOMMU_TYPE=""
for g in /sys/bus/pci/devices/*/iommu_group/type; do
    [[ -r "$g" ]] && { IOMMU_TYPE="$(cat "$g")"; break; }
done
if [[ -z "$IOMMU_TYPE" ]]; then
    ok "no IOMMU groups (translation off)"
else
    info "domain type: $IOMMU_TYPE"
    [[ "$IOMMU_TYPE" == DMA* ]] && info "  iommu.passthrough=1 on the cmdline removes per-URB map/unmap (untested)"
fi

echo "== irqbalance (must stay off: it migrates the xHCI IRQ at runtime) =="
if systemctl is-active --quiet irqbalance 2>/dev/null; then
    warn "irqbalance is RUNNING -- stop and disable it"
else
    ok "irqbalance not running"
fi

echo "== C-states =="
if command -v lsof >/dev/null 2>&1 && lsof /dev/cpu_dma_latency >/dev/null 2>&1; then
    ok "/dev/cpu_dma_latency held (all idle states blocked)"
else
    info "deep C-states enabled. With governor=performance this costs only ~2 us"
    info "at p99; run '$0 --pmqos' in another terminal to close that gap."
fi

echo
echo ">> Done. Nothing here survives a reboot; re-run after every boot."
echo ">> Verify with: sudo HCS_RTT_LABEL=after ./host/build/examples/usb_ep0_rtt"
