# ch32_board BSP provenance

WCH does not ship a CMake/Kconfig-style SDK for the CH32H417 the way HPMicro
ships `hpm_sdk` — the vendor deliverable is the per-chip **EVT** ("Evaluation
board ToolKit") package: a standard peripheral library plus MounRiver example
projects, with no build-system abstraction. We supply our own CMake glue.

The EVT package **is** published as a git repository, however:
<https://github.com/openwch/ch32h417> (one repo per chip; ~182 MB, mostly
evaluation-board PDFs, schematics and LVGL demo bitmaps). WCH bumps the whole
tree at once — commit messages are literally `V1.4`, `V1.5` — roughly five
releases per year, with no tags.

The peripheral library therefore comes in as a **submodule**,
`bsp/ch32h417-evt` → [`zyitom/ch32h417-evt`](https://github.com/zyitom/ch32h417-evt),
a 2.6 MB subset of that upstream (`wch/` = `EVT/EXAM/SRC/`, `usb/` =
`CH372Device/Common/`) carrying no local modifications. A subset repo rather than
a fork because our USBSS patches are deletions and rewrites — see the patch list
below — which would conflict with upstream on every whole-tree bump; and because
`git rm` of the unwanted 180 MB would not shrink a clone, the blobs stay in
history.

`bsp/usb/` stays **vendored in-tree** precisely because it is patched: those
files call into `app/src/usb/vendor.cpp`, so they do not belong in a third-party
repo. The submodule's own pristine `usb/` copy is the diff baseline:

```bash
diff -r firmware/ch32_board/bsp/usb firmware/ch32_board/bsp/ch32h417-evt/usb
```

Treat `bsp/ch32h417-evt/` (read-only, submodule) and `bsp/usb/` as third-party:
do not hand-edit except to re-vendor (see procedure below). Our own code lives in
`app/` and `bsp/syscalls.c`.

## Source

| | |
|---|---|
| Upstream repo | <https://github.com/openwch/ch32h417> |
| Version | **V1.5**, commit `0d9a5a7b2` (merge `a0a56fa83`, 2026-04-13) |
| Submodule | `zyitom/ch32h417-evt` @ `f83b427` |
| Publisher | Nanjing Qinheng Microelectronics (WCH) |
| Equivalent zip | CH32H417EVT, package date 2026.04 |
| Target core | Qingke V5F (RV32IMAFC + WCH `xw`, built without `xw`) |

Verified 2026-08-05: all 88 files under the previous in-tree `bsp/wch/` were
byte-identical to upstream V1.5 (line endings aside — the zip ships CRLF, both
git repos LF), so the swap to a submodule changed no compiled input.

## Vendored trees and their EVT source paths

| In-repo path | Upstream path |
|---|---|
| `bsp/ch32h417-evt/wch/Core/` | `EVT/EXAM/SRC/Core/` |
| `bsp/ch32h417-evt/wch/Peripheral/` | `EVT/EXAM/SRC/Peripheral/` |
| `bsp/ch32h417-evt/wch/Debug/` | `EVT/EXAM/SRC/Debug/` |
| `bsp/ch32h417-evt/wch/Startup/` | `EVT/EXAM/SRC/Startup/` |
| `bsp/ch32h417-evt/wch/Ld/` | `EVT/EXAM/SRC/Ld/` |
| `bsp/ch32h417-evt/examples/` | `EVT/EXAM/` minus `SRC/`, LVGL, IDE project files |
| `bsp/usb/` (patched) | `EVT/EXAM/USBSS/DEVICE/CH372Device/Common/` |
| `app/User/` (seed) | `EVT/EXAM/USBSS/DEVICE/CH372Device/V5F/User/` |

`wch/Peripheral/` carries the **complete** set of 38 vendor drivers, so bringing up
a peripheral we have not touched yet (sdio, eth, i2c, …) needs no submodule change —
just include its header. The 47 category directories under upstream `EVT/EXAM/` are
not additional drivers; they are 390 MounRiver example projects that all reference
the one shared `SRC/Peripheral/`. Those demo **sources** are in `examples/` as a
usage reference — WCH's demos are frequently the only worked example beyond the RM,
which is how the EP1 chained-DMA receive-length semantics got sorted out. Neither
`examples/` nor the vendor PDFs it keeps are build inputs.

The submodule's `manuals/` holds the datasheet (DS0 V1.8, 2026-05-14) and reference
manual (RM V1.7, 2026-04-10). These do **not** come from the EVT package —
upstream's `Datasheet/` directory ships download links only — so they are versioned
in the submodule README and will need refreshing when WCH revises them; official
download links are there too.

## Pinned file versions (from each file's WCH header)

Core/Startup:
- `core_riscv.c` V1.0.2 (2026/02/02), `core_riscv.h` V1.0.2 (2026/03/18)
- `startup_ch32h417_v5f.S` V1.0.3 (2025/11/20)
- `debug.c` / `debug.h` V1.0.1

Peripheral drivers currently relied on (full list in the tree):
- `ch32h417_can.c` V1.0.2 (2025/10/21)   - classic bxCAN 2.0B, NOT CAN-FD
- `ch32h417_usart.c` V1.0.1 (2025/09/12)
- `ch32h417_rcc.c` V1.0.3 (2026/03/17)
- `ch32h417_gpio.c` V1.0.1, `ch32h417_dma.c` V1.0.0, `ch32h417_tim.c` V1.0.0
- `ch32h417_flash.c` V1.0.2 (2026/04/08)

USBSS stack (`bsp/usb/`): from CH372Device demo, `ch32h417_usbss_device.*`
V1.0.1 (2026/04/10).

## Local patches (must be re-applied after re-vendoring)

`bsp/usb/ch32h417_usbss_it.c` - EP1 repurposed as the libhcs bulk pipe. The
CH372 demo's EP1<->EP2 hardware DMA loopback (in the `USBSS_UIF_TRANSFER` handler,
`DEF_UEP1` cases for both IN and OUT) is replaced by calls to
`usb_ss_ep1_in_complete()` / `usb_ss_ep1_out_complete()`, which live in
`app/src/usb/vendor.cpp`. Search the file for `libhcs LOCAL PATCH`. When
re-vendoring, re-apply these two case-body replacements plus the two `extern`
declarations near the top.

`bsp/usb/usb_desc.h` / `bsp/usb/usb_desc.c` - libhcs device identity. Three
edits, all marked `libhcs LOCAL PATCH`:

1. `usb_desc.h`: `DEF_USB_VID` / `DEF_USB_PID` / `DEF_USB30_PID` changed from the
   CH372 demo's `0x1A86:0x5537` to `0xA511:0xD403` (libhcs VID, ch32_board type
   PID; c_board is `0xF407`, mc02 `0x0723`).
2. `usb_desc.h`: `MyManuInfo` / `MyProdInfo` / `MySerNumInfo` lose their `const`
   and gain a `libhcs_usb_init_descriptors()` declaration.
3. `usb_desc.c`: the definitions of those three string descriptors are deleted.
   They now live in `app/src/usb/descriptors.cpp`, which builds them at runtime -
   the product string has to carry `libhcs_PROJECT_VERSION_STRING` (the host SDK
   checks it) and the serial is derived from the chip signature words.

4. `usb_desc.c`: `SS_ConfigDescriptor` rebuilt. The demo advertised one interface
   with six endpoints (three bulk pairs); libhcs uses a single pair, so EP2/EP3
   and their companion descriptors are deleted (`bNumEndpoints` 6 -> 2) and a DFU
   run-time interface is appended (`bNumInterfaces` 1 -> 2, `wTotalLength` 0x60 ->
   0x3E). The DFU interface is what `app/src/usb/dfu_runtime.cpp` answers class
   requests for; its `iInterface` is 0 because this stack only serves string
   indices 0..3 plus the OS string. Layout mirrors mc02's `TUD_DFU_RT_DESCRIPTOR`.

5. `usb_desc.c`: `SS_ConfigDescriptor` exists twice, selected by
   `libhcs_DFU_DEVICE` (`CMakeLists.txt`: 0 for `ch32_board_app`, 1 for
   `ch32_board_boot`). The `#if` branch is the bootloader's DFU-mode
   configuration - one interface, no endpoints, `bInterfaceProtocol` 0x02 and a
   DFU functional descriptor with `wTransferSize` 512 (== `DEF_USBSSD_UEP0_SIZE`,
   so a DNLOAD block is exactly one control-OUT packet). The `#else` branch is
   the application configuration described in 4. Both images share the same
   VID/PID; the product string and this protocol byte are what tell them apart.

6. `usb_desc.c`: `bcdDFUVersion` in both DFU functional descriptors is `0x0110`
   (1.1), not the CH372 demo's value. This tracks TinyUSB, which the other three
   boards use: `TUD_DFU_RT_DESCRIPTOR` / `TUD_DFU_DESCRIPTOR` emitted `0x0101`
   through TinyUSB 0.20.0 and `0x0110` from 0.21.0 on. Keeping the two in step is
   the whole point of the hand-written copy - if the submodule is ever moved back
   to a pre-0.21.0 tag, revert these two words to `0x01, 0x01`.

Everything else in `usb_desc.c` (device / BOS / HS / FS descriptors) is
unmodified WCH. Note the HS/FS configuration descriptors still describe the demo
layout: with `libhcs_USBSS_HS_FALLBACK=0` they are unreachable, and the DFU
build does not override them either.

`bsp/usb/ch32h417_usbss_it.c` + `ch32h417_usbss_device.c` + `ch32h417_usbss_device.h`
- EP2 and EP3 removed entirely. The demo kept two extra bulk pairs enabled in
`UEP_TX_EN`/`UEP_RX_EN`, armed in `USBSS_Device_Endp_Init()`, and serviced by
loopback code in the ISR - including an EP2-OUT case that armed `EP1_TX` from
EP2's receive registers, i.e. stray host traffic on EP2 could inject into the
libhcs uplink. Deleted: the two `USBSS_EP{2,3}_Rx_Buf` definitions and externs
(64 KB of SRAM), `EP2_Chain_Sel` / `EP3_T_Chain_Sel` / `EP3_R_Chain_Sel`, the
EP2/EP3 arming in `USBSS_Device_Endp_Init()`, and the four EP2/EP3 ISR cases
(which now stall through `default`). Paired with the `SS_ConfigDescriptor` edit
above - the descriptor and the hardware enable must agree.

`bsp/usb/ch32h417_usbss_device.c` + `bsp/usb/ch32h417_usbhs_device.c` - no USB 2.0
fallback. Two guards on `libhcs_USBSS_HS_FALLBACK` (defined in `CMakeLists.txt`,
0 by default), both marked `libhcs LOCAL PATCH`:

1. `USB_Timer_Start()` returns early on `ENABLE`, so TIM12 - which exists only to
   time out SuperSpeed link training and switch to USB 2.0 - is never armed.
2. `USBHS_Device_Init()` returns early on `ENABLE`, catching the paths where the
   USBSS link handler reaches for USB 2.0 directly (`LINK_STATE_INACTIVE`,
   `U2U3_SUCC`) without going through TIM12. Disable paths stay live.

Reason: USB 2.0's D+/D- are PB8/PB9, which on this package are also SWCLK/SWDIO.
A fallback hands those pins to the USB2 PHY and the debug interface drops until
the next power cycle - which happens on every USB disconnect, including the one
caused by halting the core in a debugger. Set the macro to 1 to restore stock
behaviour and USB 2.0 host support.

## Re-vendoring procedure (when a newer EVT is released)

Two stages: refresh the submodule repo, then re-apply our `bsp/usb/` patches.

**Stage 1 — in `zyitom/ch32h417-evt`** (see its README for the copy commands):
pull the new upstream, overwrite `wch/` and `usb/`, `git diff` and review, update
its version table, commit, push.

**Stage 2 — in libhcs:**

1. Bump the submodule pointer and update the Source table above.
2. Diff the new pristine stack against our patched copy and re-apply each entry
   from the patch list — nothing else in `bsp/usb/` may change:

   ```bash
   git submodule update --remote firmware/ch32_board/bsp/ch32h417-evt
   diff -r firmware/ch32_board/bsp/usb firmware/ch32_board/bsp/ch32h417-evt/usb
   ```

   Every hunk in that output must correspond to a `libhcs LOCAL PATCH` marker.
   An unexplained hunk means either a patch was lost or upstream moved code under
   one.
3. Review the `wch/` side for `core_riscv.*`, `startup_*.S`, `Ld/V5F/Link_v5f.ld`
   and any peripheral we drive (can / usart / dma / gpio / rcc / tim / usbss).
   WCH bumps individual file versions independently, so most files stay identical
   even across a whole-tree release.
4. Rebuild and check the linker's FLASH usage — the Debug app sits at ~96 % of
   128 KB, so a vendor growth can silently overflow.
5. Re-verify the interrupt-return invariant if `startup_*.S` or the USBSS IT
   files changed (see `README.md` and `PITFALLS.md`):

   ```bash
   riscv32-wch-elf-objdump -d --disassemble=USBSS_LINK_IRQHandler \
       build/ch32_board_app.elf | tail -4    # must end in mret, not ret
   ```
