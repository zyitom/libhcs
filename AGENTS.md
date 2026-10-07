# 仓库指南

> **文档类型**：现行规范（全仓库共享约束）
> **适用范围**：整个仓库，所有目录、所有芯片
> **状态**：现行有效
> **相关文档**：[README.md](README.md)（面向使用者的上手说明） · [ENV.md](ENV.md)（构建依赖与本机工具状态） · 各芯片 `firmware/<board>/AGENTS.md`

> 本文件是全仓库共享的 agent 指南。为兼容不同工具（Claude Code 读 `CLAUDE.md`，
> Codex 等读 `AGENTS.md`），`CLAUDE.md` 是指向本文件的符号链接——只维护
> `AGENTS.md` 一份即可，两边同步。各芯片子目录下另有专属 `AGENTS.md`（同样带
> `CLAUDE.md` 软链），会在处理该目录代码时叠加加载，不影响本文件的共享约束。

## 摘要

本文件规定全仓库共享的四件事：**架构边界**（板子是什么、不是什么）、**目录职责**（代码
放哪）、**构建与检查流程**（怎么编、怎么过 CI）、**修改纪律**（哪些文件不能碰、提交怎么
写）。芯片专属的工具链、外设、烧录细节不在这里，在各自的 `firmware/<board>/AGENTS.md`。

## 架构边界

libhcs 是**无下位机**控制系统（[README.md](README.md)）：控制环在上位机（HCS），不在
板子上。**全部板卡的固件永远是主机 USB 与外设总线（CAN/UART 等）之间的纯转发桥**：
不做滤波、不做控制律、不跑任何业务逻辑；板子自己提供的只有转发路径、时间基准与诊断
[用户确认 2026-09-12]。一切性能优化、多核/跨核利用与新功能提案，都**不得**以"在板上
引入业务逻辑"为方向。控制环需要板子提供什么（数据年龄，不是对时）见
[CONTROL_TIMING.md](firmware/hpm_board/CONTROL_TIMING.md)；延迟/吞吐实测与"板端为什么
不是瓶颈"见 [HOST_TUNING.md](HOST_TUNING.md)。

## 项目结构与模块组织
- `firmware/`：各板卡固件，详见下方"多芯片固件总览"。每块板一个子目录，含独立的 `CMakePresets.json` 与专属 `AGENTS.md`。
- `firmware/common/`：各板共享的固件代码（`app/src/usb/` 的 EP0 头、`app/src/utility/` 的
  `lazy.hpp` / `ring_buffer.hpp` / `interrupt_lock.hpp`，包含根是仓库根，header-only 不必改
  各板 CMake）。板级事实（mailbox 布局、assert 失败动作、`loop_work.hpp` 等）不进这里，
  仍留在各板 `app/src/utility/`。
- `firmware/*/bsp/`：厂商/子模块依赖；除非有意更新子模块，否则视为第三方代码。

## 多芯片固件总览
每块板都复用 `core/` 与 host SDK，只在 `firmware/<board>/` 下放板级固件。芯片专属
的工具链路径、外设、烧录方式、已知坑等，写在各自的 `firmware/<board>/AGENTS.md`。

| 板子 | MCU | ISA / 工具链 | 构建 target | 备注 |
|---|---|---|---|---|
| `c_board` | STM32F407IGH6TR | ARM `cmake/gcc-arm-none-eabi.cmake` | `c_board_app` `c_board_bootloader` | CubeMX BSP + TinyUSB |
| `mc02` | STM32H723VGT6（M7） | ARM `cmake/gcc-arm-none-eabi.cmake` | `mc02_app` `mc02_bootloader` | CAN-FD，USB Full-Speed |
| `hpm_board` | HPM6E8Y / HPM5321（Andes） | RISC-V 超级构建 + HPMicro GNU 工具链 | `hpm_board_app` `hpm_board_bootloader` | HPM SDK v1.12.0 |

> RISC-V 板 `hpm_board` 用 HPM `riscv32-unknown-elf-gcc`。**不要**用
> `arm-none-eabi-gcc`；后者只给 STM32（`c_board`、`mc02`）用。

## 构建环境与工具链

- 工具链一律**留在仓库外**，不入库、不做 submodule；依赖清单与安装命令见
  [ENV.md](ENV.md)。判据是"不可替代且无法重建"——能重下或有等价替代的工具链一律
  不入库。

### 开发机环境路径约定（重要，先读这条再看任何路径）

仓库文档里出现的 `~/3rd_party/...`、`/opt/...` 这类**绝对路径**是"确实跑通过"的
参照，**不代表当前机器的实际状态**。看到机器相关路径时：

- **当前机器实际装了什么、装在哪、什么版本，唯一权威是
  [ENV.md](ENV.md)「本机实际安装状态」**——先查它或直接 `ls` 确认，不要凭任何文档
  的旧措辞断言"本机没有/有某工具链"。
- 换到新机器时：按 [ENV.md](ENV.md) 的依赖清单重新安装，再把
  `GNURISCV_TOOLCHAIN_PATH`、`PATH` 等环境变量指向新的安装
  位置，然后**只更新 ENV.md 的那张表**。
- 各文档中出现旧机器路径的地方标注为 `[前机路径]`，看到这个标记就按本节理解。

- `firmware/mc02` 的两个子模块（`stm32h7xx-hal-driver`、`cmsis-device-h7`）容易被
  漏掉：仓库 clone 后若 `git submodule status` 输出以 `-` 开头，先执行：
  ```bash
  git submodule update --init firmware/mc02/bsp/stm32h7xx-hal-driver firmware/mc02/bsp/cmsis-device-h7
  ```
  否则 `mc02` configure 可能报 `Cannot find source file`。[实测]

## CubeMX BSP 修改纪律
- `firmware/*/bsp/cubemx/` 下 CubeMX 生成的产物（`Core/`、`USB_DEVICE/`、`cmake/`、`Makefile`、`.mxproject`）禁止 AI 直接修改：下次 Generate 会被覆盖。`.claude/settings.json` 已对这些目录硬禁止 Edit/Write。
- 任何外设/时钟/中断/DMA 配置变更，AI 必须明确指出应在 CubeMX（或对应 `.ioc` 键）的哪个字段修改，由人工在 CubeMX 改后重新 Generate；严禁绕过 `.ioc` 直接改生成代码。
- 例外：`.ioc` 与手维护的链接脚本 `*.ld` 仅在用户明确要求时方可由 AI 编辑。
- 仅 `c_board`、`mc02` 使用 CubeMX；`hpm_board` 禁止 AI 直接修改bsp下内容（见其 `AGENTS.md`）。

## 构建、测试与开发命令

Host SDK（纯 x86，任意机器可编）；测试默认随它一起编，编完跑一遍：
```bash
cmake --preset linux-debug -S host
cmake --build host/build
ctest --test-dir host/build --output-on-failure
```

固件统一形态为 `cmake --preset <preset> -S firmware/<board>` + `cmake --build`。
各板的 preset、target、工具链环境变量见 `firmware/<board>/AGENTS.md`。

Lint（与 CI 对齐）：
```bash
.scripts/clang-format-check --fix    # 应用格式修复
.scripts/clang-tidy-check            # 静态分析（需在 CMake build 之后运行）
```

`.scripts/clang-tidy-check --fix` 可触发 clang-tidy 自动修复，但部分修复可能不符合预期，需手动调整。
例如: int var 会被修复为 int const var. 但项目中使用的 Google 风格要求使用 const int var。

## 文档规范
新增或修改任何 `.md` 前先读 `.claude/skills/doc-convention/SKILL.md`（Claude Code 按需加载，
其他工具直接读该文件）：三层文档模型、四行文档头、摘要与导航、章节编号、来源标注、
历史文档处理。**L2（AGENTS.md）只放命令与约束；实测过程、踩坑记录、工作日志放 L3，
AGENTS.md 里留一句结论加链接。**

## 代码风格与命名规范
- 语言：C11 + C++23，禁用 GNU 扩展。
- 格式：以 `.clang-format` 为准，由 `.scripts/clang-format-check` 强制。
- 命名：Google 风格，但函数命名为小写下划线。
- 代码中除注释外不允许包含任何非 ASCII 字符（标识符、字符串字面量保持 ASCII）；
  注释与 Markdown 文档允许中文。

## 测试指南
- **主机侧测试在 `host/tests/`（GoogleTest），不要硬件**；本仓库作顶层工程时默认构建（`LIBHCS_BUILD_TESTS`），
  CI 在「Build host」后跑 `ctest`；被 HCS 引入时由 HCS 把同一批源文件编进 `test_libhcs_*`。被测代码在 `core/`
  的，主机与固件编的是同一份，测试即固件那一侧的测试：
  - `wire_protocol_test.cpp`：数据流线格式往返。
  - `ep0_declaration_test.cpp`：EP0 声明。假板 = 真实的 `core/src/link/`（注册表、EP0 分发、清单事务）+ 内存驱动 +
    真实板型的口表；末尾钉着线格式版本号（一动就红 = 主机与全部板子要同批重刷）。
  - `sof_stamp_test.cpp`：`SofStamp` 格式、`core/src/time/` 的 SOF 环、`counter_link`、`usb_sof_bits`。
  - `sof_timebase_test.cpp`：`core/src/time/sof_timebase.hpp` 三板共用的时间基准（拟合、锚点回绕、状态机、换算）。
  - `time_affine_test.cpp`：时钟之间的仿射映射（`Affine`、组合与求逆、`AxisMap`/对时上报的两个工厂），只测算术。
  - `sample_time_test.cpp`：回调交出去的 `SampleTime`（CAN 戳 / 板钟 / 未锁定回退到到达时刻 / UART）与板钟 32 位读数的 64 位展开。
  - `time_axis_test.cpp`：主机时间轴（每个主机控制器一条的 `UsbFrameAxis`：往返拟合、控制器计数器 -> `CLOCK_MONOTONIC`），只测算术。
  - `fetchcontent/`：不是 gtest，是 SDK 源码包的外部使用者；CI 用 `.scripts/package_sdk_source` 打包后按 README 的 `FetchContent` 写法编它一次。
  - `board_ownership_test.cpp`：`core/src/link/ownership.hpp` 归属状态机。
  - `session_test.cpp`：`core/src/link/session.hpp` 会话握手与租约；`core/src/link/downlink_errors.hpp`
    链路状态来源（下行流错误）；`core/src/link/port_status.hpp` 端口状态账本。
- **改了 `core/`、`host/src/time/`、`hcs_config.hpp` 或 `vendor_control.hpp` 必须跑这批测试**；新增协议字段或
  EP0 语义时在同一次改动里补用例。
- 运行时的错误与状态一律随 keepalive 应答推送，**不走 EP0**（EP0 只做握手、声明、查清单）：
  一种记录 `kPortStatus`，正文按口种类定、≤16 字节、变了才发；种类只在 `data::PortStatusVariant` 登记一处，
  链路本身（下行流错误）也是一个口（[PORT_STATUS.md](core/PORT_STATUS.md)；下行错误的拒收/丢弃纪律见
  [DOWNLINK_ERRORS.md](core/DOWNLINK_ERRORS.md)）。线格式见 [PROTOCOL.md](core/PROTOCOL.md) 1.3。
- **固件行为（外设启停、总线应答）只有上板才能验**：用例在 HCS 的 `hcs_core/test/test_bench_boards.cpp`
  （两块 5321 / 5321 + mc02 / 两块 5321 + mc02 / 5321 + mc02 + c_board，凑不成就自动跳过）。能不碰寄存器判断对错的逻辑优先放进 `core/`。
- 一次性测量程序不进仓库（`host/examples/` 已删），结论写进对应 L3 文档。
- CI 门禁：clang-format、主机构建 + `ctest`、各板固件编译、clang-tidy。每次修改后跑 `clang-format-check` 并至少
  构建一个相关 target。**clang-tidy 很慢**：只在准备提交前、或用户要求时跑，只跑改动涉及的 target。

## 提交指南
- Git unstaged changes 是必要的 code review 渠道。Agent 严禁执行 git add。 
- Commit Message 全英文，不允许包含任何非 ASCII 字符。
- 遵循仓库的 Conventional Commit 风格；破坏性变更使用 `!` 标记。
- 冒号后首字母需大写：`feat(scope): Capitalize the first letter of the title`。
- 每次提交应聚焦于单个模块或关注点。
