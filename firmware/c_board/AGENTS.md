# c_board 固件指南

> **文档类型**：现行规范（板级）
> **适用范围**：`firmware/c_board/`，RoboMaster C 板 / DJI C-type（STM32F407IGH6）
> **状态**：现行有效
> **相关文档**：[仓库根 AGENTS.md](../../AGENTS.md)（共享约束） · [仓库根 README.md](../../README.md)（烧录流程）

> 本目录专属指南，叠加在仓库根 `AGENTS.md` 之上（根为共享约束，此处只写 c_board 专属）。

## 摘要

c_board 是四块板里最常规的一块：单核 Cortex-M4F、ARM 工具链、CubeMX 生成 BSP、
app 与 bootloader 两套独立镜像。改这块板的代码只需注意一件事——**外设配置必须回到
CubeMX 改**，详见文末的 CubeMX 纪律。DFU 烧录流程与其他 STM32 板一致，见
[仓库根 README.md](../../README.md#烧录-appusb-dfu)。

## 芯片与工具链
- MCU：**STM32F407IGH6**（RoboMaster C 板 / DJI C-type），Cortex-M4F。
- ISA/工具链：ARM，`cmake/gcc-arm-none-eabi.cmake`，需 `arm-none-eabi-gcc`（**不是** RISC-V）。

## 构建
```bash
cmake --preset debug -S firmware/c_board
cmake --build firmware/c_board/build --target c_board_app c_board_bootloader
```
- preset：`debug` / `debug-outside` / `release`。`debug-outside` 置 `HOST_DEBUGGER=ON`（外部调试器场景）。
- target：`c_board_app`、`c_board_bootloader`。

## 目录结构
- `app/`、`bootloader/`：两套独立镜像（C++ libhcs 层）。
- `bsp/cubemx/`：CubeMX 生成产物（含链接脚本/时钟/外设初始化）。
- `bsp/`：`cmsis-core`、`cmsis-device-f4`、`stm32f4xx-hal-driver`、`tinyusb`、`SEGGER`（RTT）——第三方，视为只读。

## CubeMX 纪律（本板适用）
- 外设/时钟/中断/DMA 配置改在 CubeMX，源为 `bsp/cubemx/c_board_slave.ioc`；AI 只指出改哪个 `.ioc` 字段，由人工 Generate，**禁止**直接改 `bsp/cubemx/Core/` 等生成代码。
- `.ioc` 与手维护 `*.ld` 仅在用户明确要求时方可由 AI 编辑。
