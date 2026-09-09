# OpenOCD (WCH fork) — 归档副本

> **文档类型**：现行事实（工具归档）
> **适用范围**：`firmware/ch32_board/` 的烧录与调试链路
> **状态**：现行有效
> **相关文档**：[../../AGENTS.md](../../AGENTS.md)（烧录命令） · [../../PITFALLS.md](../../PITFALLS.md)（`unfreeze` 位置等坑） · [仓库根 AGENTS.md](../../../../AGENTS.md)（工具链留在仓库外的通则）

CH32H417 的调试链路只能用这一份 OpenOCD。**它是 WCH 从主线 0.11 分叉的私有版本，
主线和发行版的 openocd 一律不能替代**，`apt install openocd` 装到的东西第一行就报错。

## 为什么破例入库

仓库根 `AGENTS.md` 的通则是"工具链一律留在仓库外，不入库、不做 submodule"。
**这里是有意例外**，判据是"不可替代且无法重建"：

| | |
|---|---|
| 唯一来源 | MounRiver Studio 的 Linux 工具链包（mounriver.com），**没有**独立的 Linux 分发、apt 包或第三方镜像 |
| 源码 | **未公开**。`openwch/openocd_wch` 只有 2023-05-30 的 Windows 二进制（v1.6，早于本芯片），其 README 要求发邮件到 support@mounriver.com 索取源码与构建步骤 |
| 能否自行编译主线 | 不能。主线 0.12 的 72 个 adapter 驱动里没有 wlink/sdi 任何一个 |

GCC15 不入库是因为它能重下、极端情况下还能用别的 RISC-V GCC 凑；这份 openocd
丢了就没有第二个来源。**不要按仓库根第 70 行的通则把这个目录清掉。**

## 版本

| | |
|---|---|
| 版本 | `0.11.0+dev-snapshot (2026-02-28-11:01)`，GNU GPL v2 |
| 取自 | `~/3rd_party/MRS_Toolchain_Linux_X64_V240/OpenOCD/OpenOCD/` |
| 平台 | **x86-64 Linux only**，动态链接，依赖宿主的 `libusb-1.0.so.0` |

其他平台（ARM Mac、Windows）用不了这份二进制，装 MounRiver 对应平台的包。
`bin/` 与 `share/` 原样复制，未做任何修改。

## fork 私有的东西

`bin/wch-riscv.cfg` 依赖四个主线没有的标识，这是"换个 openocd 就跑不了"的根因：

| | 主线情况 |
|---|---|
| `adapter driver wlinke` | 无此驱动 |
| `transport select sdi` | 无此 transport（WCH 自家两线调试协议） |
| `target create ... wch_riscv` | 自定义 target 类型，不是主线的 `riscv` |
| `flash bank ... wch_riscv` | 自定义 flash driver |
| `wch_riscv unfreeze` | fork 私有扩展命令 |

三个 cfg：`wch-riscv.cfg`（单 target，**跑板子用这个**）、`wch-dual-core.cfg`
（两个 hart 同时放出，`reset run` 会让 V5F 跑 V3F 镜像，只在需要同时观察两核时用）、
`wch-arm.cfg`。`share/openocd/scripts/` 是 openocd 标准脚本库，运行时需要。

## 用法

调用序列本身有坑（`unfreeze` 必须夹在 `init` 和 `halt` 之间、不要额外
`flash erase_sector`、halt 后读到的 PC 不可信），全部记在 `../../PITFALLS.md`，
上板前先读那份。

```bash
OCD=firmware/ch32_board/tools/openocd-wch
$OCD/bin/openocd -f $OCD/bin/wch-riscv.cfg \
    -c "init" -c "wch_riscv unfreeze" -c "halt" \
    -c "program firmware/ch32_board/build/ch32_board_merged.hex verify" \
    -c "reset run" -c "exit"
```

需要 WCH-LinkE（不是通用 J-Link/SWD 探针）。CI 不使用本目录——烧录只在宿主机进行。

归档时去掉了 `share/info/` 和 `share/man/`（GNU info/man 格式的 openocd 手册，
用不到），其余原样。

## 免 root 访问

`share/openocd/contrib/60-openocd.rules` 是主线带的规则文件，**没有 WCH 的
VID `1a86`**——这份 fork 没往里加。所以装了它也不够，WCH-LinkE 要自己写一条：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-wch-link.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

不配就得 `sudo` 跑 openocd。（本机未验证该规则，只核对了规则文件里确实没有 `1a86`。）
