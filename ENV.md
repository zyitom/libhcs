# 构建环境

> **文档类型**：现行规范
> **适用范围**：全仓库构建环境与开发机调试工具安装
> **状态**：现行有效
> **相关文档**：[README.md](README.md) · `firmware/<board>/AGENTS.md` · **[HOST_TUNING.md](HOST_TUNING.md)（运行环境：内核 / USB / 网卡调优，不在本文）**

## 摘要

本文记录全仓库构建依赖、宿主机调试工具，以及**当前机器实际已安装工具的唯一权威清单**
（见下方「本机实际安装状态」）。`ch32_board` 的正式构建要求独立的
MounRiver WCH GCC15，当前机器和 `libhcs-ci` 镜像均已安装。

> **本文不含任何内核调优。** 跑起来之后的主机侧设置——CPU governor、C-state / PM QoS、
> USB autosuspend、`isolcpus`/`nohz_full`、xHCI 中断优先级、IOMMU、网卡参数——
> 全部在 **[HOST_TUNING.md](HOST_TUNING.md)**，配可执行的 `host-tuning.sh`。
> 那些设置**除内核 cmdline 外都不持久化，每次重启要重跑**，和本文的"装一次就好"是
> 两回事，所以分开两份文档。

## 本文导航

| 章节 | 内容 |
|---|---|
| [本机实际安装状态](#本机实际安装状态) | 当前机器已装工具与位置（**机器状态的唯一权威**） |
| [版本要求](#版本要求) | 编译器、构建工具和基础依赖 |
| [Docker 构建环境](#docker-构建环境) | CI / Dev Container 的工具链和职责边界 |
| [宿主机调试工具](#宿主机调试工具可选不进-docker) | J-Link、Ozone 与 WCH-LinkE 的安装边界 |
| [LLVM 20](#llvm-20clangd--clang-format--clang-tidy) | 本机与 CI 共用的编辑器、格式和静态分析工具 |
| [构建命令](#构建命令) | host SDK 与 CH32 的常用命令 |

## 本机实际安装状态

**机器相关状态只在这一节维护**，其他文档（根 `AGENTS.md`、各板文档）一律引用这里，
不要复制。下表 2026-09-11 逐条 `ls` 复核。

| 工具 | 本机位置 | 状态 |
|---|---|---|
| ARM GCC | `/opt/arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi`（**15.2.rel1**，非上方版本表要求的 15.3.rel1；不在 PATH，要手工加） | 已装 |
| RISC-V GCC（HPM）13.2.0 | `~/3rd_party/hpm/rv32imac_zicsr_zifencei_multilib_b_ext-linux`（含 `bin/` 的工具链根，`GNURISCV_TOOLCHAIN_PATH` 指向它） | 已装 |
| HPM OpenOCD | `~/3rd_party/hpm/openocd-linux-x86_64`（同目录还有 `openocd-build`、`openocd-src` 和一份 `hpm_sdk` 副本） | 已装 |
| RISC-V GCC（WCH）15.2.0 | `~/3rd_party/MRS_Toolchain_Linux_X64_V240/Toolchain/RISC-V Embedded GCC15`；同包 `OpenOCD/OpenOCD` 是 WCH OpenOCD | 已装，**`ch32_board` 本机可编可烧** |
| SEGGER | `/opt/SEGGER/JLink`（→ `JLink_V948`）、`/opt/SEGGER/Ozone_V340j` | 已装（J-Link V9.48 / Ozone V3.40j） |
| 其他 | `~/3rd_party/Xuantie-900-*`（T-Head GCC）、`~/3rd_party/soem-1.4.0`（EtherCAT 归档遗留）、`nvtop` | 与仓库固件流程无关 |

> 换机器后本表必然过期：按上文各节重装，然后**只更新本表**，不要往别的文档里写
> 机器状态。

## 版本要求

| 组件 | 版本 | 安装方式 |
|---|---|---|
| CMake | ≥ 3.21 | Ubuntu 22.04 自带 3.22.1，无需额外安装 |
| Ninja | ≥ 1.10 | `sudo apt install ninja-build` |
| GCC (x86) | **≥ 14** | 见下方 |
| ARM GCC | **15.3.rel1** | `arm-none-eabi-gcc`，c_board / mc02，见下方 |
| RISC-V GCC (HPM) | 13.2.0 | hpm_board，见下方 |
| RISC-V GCC (WCH) | 15.2.0 | ch32_board，MounRiver，见下方 |
| clangd / clang-format / clang-tidy | **20.x** | Ubuntu 24.04 官方 apt 源，见下方 |
| libusb-1.0-dev | ≥ 1.0 | `sudo apt install libusb-1.0-0-dev` |
| dfu-util | ≥ 0.9 | `sudo apt install dfu-util` |

## GCC 14（host SDK）

host SDK 用到 C++23 `<print>`（GCC 14+），Ubuntu 22.04 自带只到 12：

```bash
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install -y gcc-14 g++-14
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 140 --slave /usr/bin/g++ g++ /usr/bin/g++-14
```

## ARM GCC（c_board / mc02）

```bash
curl -fL -o ~/Downloads/arm-gnu-toolchain.tar.xz \
  https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/15.3.rel1/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi.tar.xz
mkdir -p ~/3rd_party && tar -xf ~/Downloads/arm-gnu-toolchain.tar.xz -C ~/3rd_party
export PATH="$HOME/3rd_party/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi/bin:$PATH"
```

## RISC-V GCC

两块板用两套工具链，**非常不建议混用**。

### hpm_board（HPM6E8Y / HPM5321）

```bash
curl -fL -o ~/Downloads/riscv-hpm.tar.gz \
  https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz
mkdir -p ~/3rd_party/hpm && tar -xzf ~/Downloads/riscv-hpm.tar.gz -C ~/3rd_party/hpm
export GNURISCV_TOOLCHAIN_PATH=~/3rd_party/hpm
```

详见 `firmware/hpm_board/BUILD_ENVIRONMENT.md`。

### ch32_board（WCH CH32H417）

从 MounRiver 下载 WCH 工具链：<https://www.mounriver.com/download>

详见 `firmware/ch32_board/AGENTS.md`。

## Docker 构建环境

`zyi024424/libhcs-ci:latest` 是 GitHub Actions 和 Release 共用的 `linux/amd64` 构建
环境，包含三套互相隔离的交叉工具链：

| 目标 | 容器路径 | 编译器 |
|---|---|---|
| STM32 `c_board` / `mc02` | `/opt/arm-none-eabi` | ARM GNU 15.3.rel1 |
| HPM `hpm_board` | `/opt/riscv32-none-elf` | HPM GCC 13.2.0 |
| WCH `ch32_board` | `/opt/wch-gcc15` | MounRiver GCC 15.2.0 |

WCH 工具链来自 MounRiver V240 官方 Linux X64 归档。Docker 构建通过官方 API 获取临时
签名地址，固定资源 ID `2030114123741700098`、归档大小 `411269512` 字节和 SHA-256
`1fae593d27e24466f17c2df0fd00f746143f587fe33e912a78e35142fef82a6d`，并且只提取
`Toolchain/RISC-V Embedded GCC15`。[官网与归档实测 2026-08-05]

本地构建镜像：

```bash
docker buildx build --platform linux/amd64 --target ci \
  -t zyi024424/libhcs-ci:latest --load .
```

当前 Docker 镜像、固件 Release 和 SDK Debian 包都只发布 amd64；不再构建 arm64 变体。

日常开发直接拉取 `zyi024424/libhcs-develop:latest` 并使用仓库的 Dev Container 配置；
提交 `Dockerfile` 到 `main` 后，`Docker Image CI` 会自动重新构建并推送 `ci` / `develop`
镜像，成功后再触发 Lint，避免 Lint 提前拉到旧镜像。普通代码提交只拉取镜像执行 Lint
和编译，不会重建工具链镜像。

### 编译与硬件调试边界

- Docker 负责 CMake、编译、clang-format、已启用目标的 clang-tidy 和生成 ELF/HEX/DFU；
  GitHub Runner 没有实体板卡，也只执行这些步骤。`ch32_board` 当前强制执行格式与完整编译，
  但 clang-tidy 因 WCH 厂商宏在调用点产生大量误报而显式禁用，状态会打印在 CI 日志中。
- Ozone、SEGGER J-Link Software、MounRiver WCH OpenOCD 和连接实体 USB 的 `dfu-util`
  在宿主机运行。Dev Container 默认不映射 `/dev`，避免 Docker Desktop USB/IP 和特权
  容器成为日常开发依赖。
- 镜像内仍安装 `dfu-util`，目的是构建时使用 `dfu-suffix` 生成 DFU 产物，不表示容器负责
  访问物理 USB。
- 将来若建立专用原生 Linux 硬件测试机，可另建 hardware-in-the-loop Runner，再按设备
  白名单映射 USB；不要把该权限加回通用 CI/开发容器。

## 宿主机调试工具（可选，不进 Docker）

SEGGER 工具只安装在需要连接实体调试器的 Linux x86_64 开发机上：

- J-Link Software：<https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb>
- Ozone：<https://www.segger.com/downloads/jlink/Ozone_Linux_x86_64.deb>

**不用开浏览器也能装**（2026-08-14 补充）。J-Link 的下载地址对 `curl` / `wget` 直接
GET 只返回许可确认 HTML（约 27 KB），不是 `.deb`——这一点没有变。变的是知道了它要的
只是一个表单字段：`accept_license_agreement=accepted` 用 POST 提交即可拿到真包
（约 59 MB）。Ozone 没有这道门，GET 就能直接下（约 21 MB）。`[实测]`

```bash
cd ~/Downloads
# J-Link：POST 提交许可接受；直接 GET 只会拿到 HTML
curl -L -X POST -d "accept_license_agreement=accepted" \
     -o JLink_Linux_x86_64.deb \
     https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb
# Ozone：无许可门，直接下
curl -L -o Ozone_Linux_x86_64.deb \
     https://www.segger.com/downloads/jlink/Ozone_Linux_x86_64.deb

# 下之前先确认拿到的是包不是 HTML
file JLink_Linux_x86_64.deb Ozone_Linux_x86_64.deb   # 应为 "Debian binary package"

sudo apt install ./JLink_Linux_x86_64.deb ./Ozone_Linux_x86_64.deb
dpkg-query -W jlink ozone
```

> **那个 POST 是在代替使用者接受 SEGGER Terms of Use，不要当成纯技术步骤。** 它只适合
> 使用者本人（或已明确授权的自动化）在自己的开发机上执行；**仍然不要写进 Dockerfile
> 或无人值守 CI**——原因不是技术上做不到，而是许可接受需要有一个确定的责任人。
> J-Link 软件本身免费，但授权范围限于配正版 SEGGER 探针使用。

Ozone 是依赖实体 J-Link 的 GUI 调试器，同样只在宿主机安装。本机当前为
J-Link **V9.48** / Ozone **V3.40j**（以 `/opt/SEGGER/` 实际目录为准，见
「本机实际安装状态」），二进制落在 `/usr/bin/JLinkExe`、`/usr/bin/JLinkGDBServer`、
`/usr/bin/ozone`，`.deb` 安装时会自动装 udev 规则。`[实测 2026-08-14 安装方式；版本 2026-09-11 复核]`

安装后，`./tools/jlink-debug.sh` 提供命令行 J-Link GDB Server 流程，`./tools/ozone-debug.sh` 打开
仓库中预设的 Ozone 工程；可用 `--list` 查看目标。它们用于 `c_board`、`mc02` 和
`hpm_board`，**不用于 `ch32_board`**。

CH32H417 的当前官方/已验证链路是 **WCH-LinkE + MounRiver WCH OpenOCD + WCH GDB**。
[SEGGER 当前支持设备页](https://www.segger.com/supported-devices/jlink/)在 WCH 下只列出
CH32F1 / CH32F2，没有 CH32H417；本机 J-Link V9.48 设备库也没有该芯片。因此现阶段不能
用 J-Link/Ozone 替代 WCH-LinkE。[官网与本机实测 2026-08-05] MounRiver 的 SDI 配置、
烧录命令和双核注意事项见 `firmware/ch32_board/AGENTS.md`。

## LLVM 20（clangd / clang-format / clang-tidy）

本机与 `libhcs-ci` 统一使用 Ubuntu 24.04 `noble-updates/universe` 提供的 LLVM 20；
不添加 `apt.llvm.org`，也不安装无版本的 `clangd` / `clang-format` / `clang-tidy` 元包，
因为 Ubuntu 24.04 的无版本元包仍指向 LLVM 18。[Ubuntu apt 实测 2026-08-06]

```bash
sudo apt update
sudo apt install clangd-20 clang-format-20 clang-tidy-20
sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-20 200
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-20 200
sudo update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-20 200
clangd --version
clang-format --version
clang-tidy --version
```

`clangd` 是编辑器语言服务器；CI 实际执行的是 `clang-format` 和 `clang-tidy`。三个工具固定
同一主版本，才能让本机编辑器诊断、格式检查和云端质量门禁保持一致。

## 构建命令

```bash
# host SDK
cmake --preset linux-debug -S host && cmake --build host/build

# ch32_board
export WCH_TOOLCHAIN_PATH="$HOME/3rd_party/MRS_Toolchain_Linux_X64_V240/Toolchain/RISC-V Embedded GCC15"
export WCH_TOOLCHAIN_PREFIX=riscv32-wch-elf-
cmake --preset debug -S firmware/ch32_board && cmake --build firmware/ch32_board/build
```

在 `libhcs-ci` / `libhcs-develop` 容器内无需手动导出上述两个 WCH 变量，镜像已经固定为
`/opt/wch-gcc15` 和 `riscv32-wch-elf-`。

> `RAM: 100.00%` 是正常的，不是溢出。判断是否撑爆看 FLASH。
