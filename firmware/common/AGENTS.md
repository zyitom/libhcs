# firmware/common —— 跨板共享层

c_board、mc02、hpm_board 三块板共用的代码与第三方库。板级目录只放本板专属的
BSP 与差异代码；凡是"第二块板也要用"的东西都上移到这里（SEGGER 于 2026-10-06、
`app/src/utility` 于 2026-10-05、tinyusb 于 2026-10-06）。

## 目录结构

- `bsp/tinyusb/`：**git submodule**（fork `zyitom/tinyusb`，v0.21.0），三块板共用一份
  checkout，构建前需 `git submodule update --init firmware/common/bsp/tinyusb`。
  TinyUSB 源码本身 SoC 无关，各板编译时用 `CFG_TUSB_MCU` 区分：
  - c_board app/bootloader、mc02 app/bootloader：各自 CMakeLists 顶部的
    `*_TINYUSB_ROOT` 指向 `${libhcs_PROJECT_ROOT}/firmware/common/bsp/tinyusb`，
    `CFG_TUSB_MCU=OPT_MCU_STM32F4 / OPT_MCU_STM32H7`；
  - hpm_board：经 [hpm_board/cmake/current_tinyusb.cmake](../hpm_board/cmake/current_tinyusb.cmake)
    的 `libhcs_add_current_tinyusb()`，`CFG_TUSB_MCU=OPT_MCU_HPM`。
  submodule 里有本地补丁（`src/class/vendor/vendor_device.{c,h}`、
  `src/device/usbd.c`，见 hpm_board 的 USB_OPTIMIZATION_LOG.md），视为只读——改它要
  走独立提交，影响所有板。
- `bsp/SEGGER/`：SEGGER RTT 第三方库，c_board 与 mc02 的 app 都编译进构建
  （`*_APP_SEGGER_ROOT`）。当前**没有任何代码调用**，gc-sections 会剔除，不占
  flash/RAM；定位是 J-Link/Ozone 的预留调试通道（配合仓库根 `ozone/*.jdebug`）。
- `app/src/`：跨板共享的应用层头文件——`link/host_session.hpp`、`usb/`（EP0 分级、
  vendor 控制、中断安全缓冲）、`utility/`（`interrupt_lock.hpp` 按架构分支、`lazy.hpp`、
  `ring_buffer.hpp`）。
- `cmake/libhcs_firmware.cmake`：`libhcs_setup_project_context()`（定义
  `libhcs_PROJECT_ROOT` 等，各板顶层 CMakeLists 在 `add_subdirectory(app/bootloader)`
  之前调用）与 `libhcs_add_dfu_image()`（DFU 打包）。

## 约定

- 本目录下的路径引用一律用 `${libhcs_PROJECT_ROOT}/firmware/common/...`，不要用
  跨板的相对路径（`../../c_board/...`）——那正是 2026-10-06 SEGGER 死路径事故的根源。
- 新增共享第三方库时放 `bsp/`，随迁文档引用（各板 AGENTS.md、BUILD_ENVIRONMENT.md）。
