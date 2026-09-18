add_executable(servo_test servo_test.cpp)
target_link_libraries(servo_test PRIVATE libhcs-sdk)

add_executable(hpm5321_test hpm5321_test.cpp)
target_link_libraries(hpm5321_test PRIVATE libhcs-sdk)

add_executable(dm_motor_stress dm_motor_stress.cpp)
target_link_libraries(dm_motor_stress PRIVATE libhcs-sdk)

add_executable(uart_stress uart_stress.cpp)
target_link_libraries(uart_stress PRIVATE libhcs-sdk)

add_executable(rx_monitor rx_monitor.cpp)
target_link_libraries(rx_monitor PRIVATE libhcs-sdk)

add_executable(motor_pid_sine motor_pid_sine.cpp)
target_link_libraries(motor_pid_sine PRIVATE libhcs-sdk)

# Two-board suite: a pair of HPM5321 dual-CAN-FD boards wired CAN0<->CAN0,
# CAN1<->CAN1 and UART0<->UART0. Unlike the single-board loopback tools, every
# frame crosses a real bus between two independent CAN controllers.
add_executable(dual_board_test dual_board_test.cpp)
target_link_libraries(dual_board_test PRIVATE libhcs-sdk)

# Heterogeneous pair (built for mc02 <-> HPM5321 dual-CAN-FD hardware): goes through the
# board-agnostic BoardSession, discovers the wiring, and separates USB packet
# rate from CAN frame rate -- the two ends run at different USB speeds.
add_executable(mixed_board_test mixed_board_test.cpp)
target_link_libraries(mixed_board_test PRIVATE libhcs-sdk)

add_executable(can_error_test can_error_test.cpp)
target_link_libraries(can_error_test PRIVATE libhcs-sdk)

add_executable(can_loopback_latency can_loopback_latency.cpp)
target_link_libraries(can_loopback_latency PRIVATE libhcs-sdk)

# Host-path-only USB probe: times a control transfer to EP0, so it needs no CAN
# wiring and no firmware cooperation. The instrument behind HOST_TUNING.md's
# host-side A/B numbers; not a substitute for can_loopback_latency's board RTT.
add_executable(usb_ep0_rtt usb_ep0_rtt.cpp)
target_link_libraries(usb_ep0_rtt PRIVATE libhcs-sdk)
target_include_directories(usb_ep0_rtt SYSTEM PRIVATE ${LIBUSB_INCLUDE_DIRS})

# Splits board-level one-way latency into fixed floor vs per-frame pipeline step,
# by sending a burst of CAN frames inside a single USB packet. Answers whether
# the firmware sits on the critical path or is hidden behind the CAN wire.
add_executable(can_path_split can_path_split.cpp)
target_link_libraries(can_path_split PRIVATE libhcs-sdk)

# USB bulk OUT packet-rate ceiling: sends each record in its own packet so the
# packet rate can be raised without raising the per-bus CAN frame rate. Separates
# the USB transaction limit from the CAN wire limit that caps dual_board_test.
add_executable(usb_packet_rate usb_packet_rate.cpp)
target_link_libraries(usb_packet_rate PRIVATE libhcs-sdk)

add_executable(can_frame_type_test can_frame_type_test.cpp)
target_link_libraries(can_frame_type_test PRIVATE libhcs-sdk)

# CAN-FD loopback stress over the HPM6E8Y USB transport (CAN0->CAN1, CAN2->CAN3).
add_executable(usb_canfd_stress usb_canfd_stress.cpp)
target_link_libraries(usb_canfd_stress PRIVATE libhcs-sdk)

# Same flood as usb_canfd_stress, plus a decoder for the UART0 telemetry a
# libhcs_CAN_DIAG firmware emits. Prints the MCAN and PLIC state around the
# moment forwarding stops (see firmware/hpm_board/app/src/diag/can_diag.hpp).
add_executable(can_stall_probe can_stall_probe.cpp)
target_link_libraries(can_stall_probe PRIVATE libhcs-sdk)

# The main-loop-period half of the same telemetry, for the HPM5321. Separate
# from can_stall_probe because that tool is bound to the HPM6E8Y's four buses
# and its single-board CAN0->CAN1 / CAN2->CAN3 loopback, neither of which exists
# on a two-board HPM5321 rig. Reads only; drive the load from another process.
add_executable(hpm5321_loop_probe hpm5321_loop_probe.cpp)
target_link_libraries(hpm5321_loop_probe PRIVATE libhcs-sdk)

# USB SOF / FRINDEX validation: reads every attached HPM5321 at once and decodes
# the telemetry a -Dlibhcs_SOF_DIAG=ON firmware emits. Step 1 of the cross-board
# time base -- it has to pass before any of the timeline code is written.
add_executable(sof_probe sof_probe.cpp)
target_link_libraries(sof_probe PRIVATE libhcs-sdk)

# Shared time base across every attached board: proves the boards resolve the
# same ABSOLUTE microframe (the wrap that a single board cannot check on its
# own), and exercises the Unix mapping. Needs -Dlibhcs_TIME_SYNC=ON firmware.
add_executable(time_sync_test time_sync_test.cpp)
target_link_libraries(time_sync_test PRIVATE libhcs-sdk)

# The same time base on mc02, which is full speed and has no PTPC: reports the
# board's own out-of-sample fit residual and, separately, how tightly this host
# can place a reported microframe on its own clock. Needs mc02 firmware built
# with -Dlibhcs_APP_TIME_SYNC=ON.
add_executable(mc02_time_sync_test mc02_time_sync_test.cpp)
target_link_libraries(mc02_time_sync_test PRIVATE libhcs-sdk)

# Smallest complete recipe for putting a board timestamp on the host's
# CLOCK_MONOTONIC -- the clock journald stamps its entries with -- and a
# measurement of what that conversion is worth.
add_executable(board_mono_clock board_mono_clock.cpp)
target_link_libraries(board_mono_clock PRIVATE libhcs-sdk)

# Cross-checks the board's SOF counter against the host controller's own MFINDEX,
# read from the mmap'd xHCI BAR. Needs root. Exists because the two disagreed by
# 576 ppm on which is the microframe rate.
add_executable(mfindex_vs_board mfindex_vs_board.cpp)
target_link_libraries(mfindex_vs_board PRIVATE libhcs-sdk)

# Cross-board skew MEASURED through hardware CAN Start-of-Frame captures, with
# a same-board control that establishes the method's own noise floor. Needs all
# CAN ports on one bus and -Dlibhcs_TIME_SYNC=ON firmware.

# The same measurement without PTPC in the path: both boards fire a GPTMR
# compare at one agreed microframe over the existing UART0 cross-wiring and
# capture each other's edge in hardware, so the two-way difference cancels the
# cable delay. Needs -Dlibhcs_PULSE_TEST=ON firmware, which takes UART0 over.
add_executable(pulse_skew_test pulse_skew_test.cpp)
target_link_libraries(pulse_skew_test PRIVATE libhcs-sdk)

# Queue-free CAN0->CAN1 RTT over USB, one frame in flight at a time.
add_executable(bridge_can_loopback_latency bridge_can_loopback_latency.cpp)
target_link_libraries(bridge_can_loopback_latency PRIVATE libhcs-sdk)

add_executable(motor_load motor_load.cpp)
target_link_libraries(motor_load PRIVATE libhcs-sdk)

add_executable(imu_dbus_check imu_dbus_check.cpp)
target_link_libraries(imu_dbus_check PRIVATE libhcs-sdk)

add_executable(mc02_can_diag mc02_can_diag.cpp)
target_link_libraries(mc02_can_diag PRIVATE libhcs-sdk)

add_executable(mc02_tinyusb_probe mc02_tinyusb_probe.cpp)
target_link_libraries(mc02_tinyusb_probe PRIVATE libhcs-sdk)

add_executable(mc02_dual_can_probe mc02_dual_can_probe.cpp)
target_link_libraries(mc02_dual_can_probe PRIVATE libhcs-sdk)

# Two mc02 boards wired TX<->RX on one UART port (default UART10). Exists because
# a same-board loopback cannot fail the way this firmware can: both ends ignoring
# a baudrate request identically still passes a loopback.
add_executable(uart_cross_test uart_cross_test.cpp)
target_link_libraries(uart_cross_test PRIVATE libhcs-sdk)

# Two mc02 boards wired A-A / B-B on an enclosure RS-485 port (UART2 / UART3).
# Separate from uart_cross_test because that suite is TTL TX<->RX; this one
# owns half-duplex turnaround and talks to Mc02 directly. Firmware must be built
# with libhcs_APP_RS485_ENABLE=ON (the default) or uart2/uart3 are rejected.
add_executable(rs485_cross_test rs485_cross_test.cpp)
target_link_libraries(rs485_cross_test PRIVATE libhcs-sdk)

# Bare USB packet-rate probe for mc02: one record per USB packet, no CAN and no
# UART line rate in the number. Pair with -Dlibhcs_APP_LOOP_BALLAST_CYCLES to
# A/B throughput against main-loop period.
add_executable(mc02_packet_rate mc02_packet_rate.cpp)
target_link_libraries(mc02_packet_rate PRIVATE libhcs-sdk)

add_executable(topo_probe topo_probe.cpp)
target_link_libraries(topo_probe PRIVATE libhcs-sdk)

# Does the phase of a downlink flush inside the 125 us USB microframe change
# end-to-end latency? Needs both boards on the shared SOF time base.

# Prices splitting one downlink batch into two packets: what the early command
# gains, and what the late one pays. Needs the shared SOF time base.

# The gimbal/chassis flush split on HCS's real five-frame, two-bus, classic-CAN
# command layout. Needs the shared SOF time base.

# One IMU UART line strapped to both an mc02 and an hpm5321. Both boards see
# the same edges, so subtracting their host arrival times measures the
# difference between the two forwarding paths exactly, with the IMU's own
# transmit instant cancelling out.
add_executable(imu_uart_latency imu_uart_latency.cpp)
target_link_libraries(imu_uart_latency PRIVATE libhcs-sdk)

# Event-loop instrument: idle wakeup rate and teardown latency, neither of which
# the throughput tools can see (they measure the loop only while completions are
# arriving, which is exactly when its timeout never fires).
add_executable(usb_event_loop_bench usb_event_loop_bench.cpp)
target_link_libraries(usb_event_loop_bench PRIVATE libhcs-sdk)

# EP0 configuration channel: proves a rejected UART baudrate and a wrong CAN
# mode expectation are now reported to the host, which the retired in-band
# config field could not do. One hpm5321 dual-CAN board, no CAN/UART wiring.
add_executable(ep0_config_test ep0_config_test.cpp)
target_link_libraries(ep0_config_test PRIVATE libhcs-sdk)

add_executable(mc02_ep0_config_test mc02_ep0_config_test.cpp)
target_link_libraries(mc02_ep0_config_test PRIVATE libhcs-sdk)

add_executable(mc02_flow_control_probe mc02_flow_control_probe.cpp)
target_link_libraries(mc02_flow_control_probe PRIVATE libhcs-sdk)

# Per-bus CAN health from the controller's own PSR/ECR registers, decoded from
# the libhcs_CAN_DIAG telemetry. Tells "transmitted, nobody ACKed" apart from
# "never reached the wire" when a bus delivers nothing.
add_executable(can_bus_diag can_bus_diag.cpp)
target_link_libraries(can_bus_diag PRIVATE libhcs-sdk)

# What the host-side microframe counter is worth, measured rather than argued.
# Sections 4 and 5 replace the two figures that were pure arithmetic: the phase
# precision of one edge (cross-checked two ways) and what edge hunting costs.
# Needs root, needs no board.
add_executable(microframe_source_test microframe_source_test.cpp)
target_link_libraries(microframe_source_test PRIVATE libhcs-sdk)

# Whether edge hunting disturbs the USB data path on the same controller -- the
# one cost of the microframe time source that cannot be computed. Interleaved
# A/B blocks, because this rig's own throughput drifts more between runs than
# the effect being looked for.
add_executable(microframe_interference_test microframe_interference_test.cpp)
target_link_libraries(microframe_interference_test PRIVATE libhcs-sdk)

# End to end: Timeline routed through the host's own microframe counter. Uses
# the better path as the reference for the worse one, so section 4 measures what
# the USB round trip was actually worth. Needs root and one board.
add_executable(microframe_timebase_test microframe_timebase_test.cpp)
target_link_libraries(microframe_timebase_test PRIVATE libhcs-sdk)

# All three time-alignment questions in one run, with every attached board --
# mc02 and hpm5321 together -- on a single Timeline: board<->x86, boards<->x86,
# board<->board, including the 2.048 s wrap check that only a multi-board run
# can make.
add_executable(time_sync_matrix time_sync_matrix.cpp)
target_link_libraries(time_sync_matrix PRIVATE libhcs-sdk)
add_executable(rtt_split rtt_split.cpp)
target_link_libraries(rtt_split PRIVATE libhcs-sdk)

# Two mc02 boards wired PE9<->PE9 and PE13<->PE13. Exercises the GPIO
# session-end firmware changes on real hardware: stop_outputs() on lease
# expiry and on session replacement, the held-duty bug on a root-commit
# board, and the EXTI disarm round-trip on one channel.
add_executable(mc02_gpio_session_test mc02_gpio_session_test.cpp)
target_link_libraries(mc02_gpio_session_test PRIVATE libhcs-sdk)

# Discriminator for mc02_gpio_session_test's P4 anomaly: link state + asap
# liveness around every drive attempt on both wires, both directions, several
# rounds. Separates a loose jumper from a faulted link from a stuck output.
add_executable(mc02_gpio_wire_probe mc02_gpio_wire_probe.cpp)
target_link_libraries(mc02_gpio_wire_probe PRIVATE libhcs-sdk)

# Closes the P4 round-trip phase inside the root-commit board's fresh-link
# window, with a liveness gate on every leg. See mc02_gpio_session_test.
add_executable(mc02_gpio_p4_finish mc02_gpio_p4_finish.cpp)
target_link_libraries(mc02_gpio_p4_finish PRIVATE libhcs-sdk)

# Per-second health+level table for the P2 lease-expiry measurement, run inside
# the root-commit board's fresh-link window so the quiet tail is provably
# sampled rather than merely silent.
add_executable(mc02_gpio_lease_confirm mc02_gpio_lease_confirm.cpp)
target_link_libraries(mc02_gpio_lease_confirm PRIVATE libhcs-sdk)

# Full GPIO driver matrix on two mc02 boards wired PE9<->PE9 and PE13<->PE13:
# digital write, duty sweep, asap/periodic/edge reads, pulls, mode churn,
# cross-timer independence and invalid-channel robustness.
add_executable(mc02_gpio_matrix mc02_gpio_matrix.cpp)
target_link_libraries(mc02_gpio_matrix PRIVATE libhcs-sdk)

# Sends a GPIO frame with an out-of-range channel index from the session owner
# itself: the firmware must refuse it, tear the session down, zero the outputs
# (stop_outputs through the teardown path) and let the link recover.
add_executable(mc02_gpio_invalid_channel_test mc02_gpio_invalid_channel_test.cpp)
target_link_libraries(mc02_gpio_invalid_channel_test PRIVATE libhcs-sdk)

# Net-topology discovery (one driver at a time, everything else reads) plus the
# remaining duty coverage per discovered wire: run this after re-jumpering the
# bench to learn what is actually connected before testing it.
add_executable(mc02_gpio_topology_test mc02_gpio_topology_test.cpp)
target_link_libraries(mc02_gpio_topology_test PRIVATE libhcs-sdk)
