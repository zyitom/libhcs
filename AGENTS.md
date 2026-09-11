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

本文件规定全仓库共享的三件事：**目录职责**（代码放哪）、**构建与检查流程**（怎么编、
怎么过 CI）、**修改纪律**（哪些文件不能碰、提交怎么写）。芯片专属的工具链、外设、
烧录细节不在这里，在各自的 `firmware/<board>/AGENTS.md`。

## 项目结构与模块组织
- `firmware/`：各板卡固件，详见下方"多芯片固件总览"。每块板一个子目录，含独立的 `CMakePresets.json` 与专属 `AGENTS.md`。
- `firmware/*/bsp/`：厂商/子模块依赖；除非有意更新子模块，否则视为第三方代码。

## 多芯片固件总览
每块板都复用 `core/` 与 host SDK，只在 `firmware/<board>/` 下放板级固件。芯片专属
的工具链路径、外设、烧录方式、已知坑等，写在各自的 `firmware/<board>/AGENTS.md`。

| 板子 | MCU | ISA / 工具链 | 构建 target | 备注 |
|---|---|---|---|---|
| `c_board` | STM32F407IGH6TR | ARM `cmake/gcc-arm-none-eabi.cmake` | `c_board_app` `c_board_bootloader` | CubeMX BSP + TinyUSB |
| `mc02` | STM32H723VGT6（M7） | ARM `cmake/gcc-arm-none-eabi.cmake` | `mc02_app` `mc02_bootloader` | CAN-FD，USB Full-Speed |
| `ch32_board` | WCH CH32H417（Qingke V3F + V5F 双核） | RISC-V `cmake/toolchain-wch-riscv.cmake` | `ch32_board_app` `ch32_board_boot` `ch32_board_merged` | USB 3.0 SuperSpeed；`boot` 是 V3F 启动核兼 DFU bootloader |
| `hpm_board` | HPM6E8Y / HPM5321（Andes） | RISC-V 超级构建 + HPMicro GNU 工具链 | `hpm_board_app` `hpm_board_bootloader` | HPM SDK v1.12.0 |

> 两块 RISC-V 板使用相互独立的编译器：`ch32_board` 用 WCH
> `riscv32-wch-elf-gcc`，`hpm_board` 用 HPM `riscv32-unknown-elf-gcc`。**不要**用
> `arm-none-eabi-gcc`；后者只给 STM32（`c_board`、`mc02`）用。

## 构建环境与工具链

- 工具链一律**留在仓库外**，不入库、不做 submodule；依赖清单与安装命令见
  [ENV.md](ENV.md)。
  **唯一例外**：`firmware/ch32_board/tools/openocd-wch/`（WCH 私有 fork 的 OpenOCD，
  CH32H417 调试链路无可替代，源码未公开，所以归档入库）。判据是"不可替代且无法
  重建"——能重下或有等价替代的工具链一律不入库。

### 开发机环境路径约定（重要，先读这条再看任何路径）

仓库文档里出现的 `~/3rd_party/...`、`/opt/...` 这类**绝对路径**是"确实跑通过"的
参照，**不代表当前机器的实际状态**。看到机器相关路径时：

- **当前机器实际装了什么、装在哪、什么版本，唯一权威是
  [ENV.md](ENV.md)「本机实际安装状态」**——先查它或直接 `ls` 确认，不要凭任何文档
  的旧措辞断言"本机没有/有某工具链"。
- 换到新机器时：按 [ENV.md](ENV.md) 的依赖清单重新安装，再把
  `GNURISCV_TOOLCHAIN_PATH`、`WCH_TOOLCHAIN_PATH`、`PATH` 等环境变量指向新的安装
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
- 仅 `c_board`、`mc02` 使用 CubeMX；`ch32_board`、`hpm_board` 禁止 AI 直接修改bsp下内容（见各自 `AGENTS.md`）。

## 构建、测试与开发命令

Host SDK（纯 x86，任意机器可编）：
```bash
cmake --preset linux-debug -S host
cmake --build host/build
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
- 代码不允许包含任何非 ASCII 字符（Markdown 文档除外）。

## 测试指南
- 目前尚未启用 CTest/GTest 测试目标；当前 CI 质量门禁为：clang-format、clang-tidy 和 编译验证。
- 每次修改后，应在本地运行 lint 工具，并至少构建一个相关的构建目标。

## 提交指南
- Git unstaged changes 是必要的 code review 渠道。Agent 严禁执行 git add。 
- Commit Message 全英文，不允许包含任何非 ASCII 字符。
- 遵循仓库的 Conventional Commit 风格；破坏性变更使用 `!` 标记。
- 冒号后首字母需大写：`feat(scope): Capitalize the first letter of the title`。
- 每次提交应聚焦于单个模块或关注点。
