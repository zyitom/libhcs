# 意优 RP CAN-FD 的 Windows 配套 DLL

> **文档类型**：背景说明
> **适用范围**：HPM5321 WinUSB 固件、32 位意优 RP Series 上位机
> **状态**：现行有效；具体实测结果见板级协议记录第 7.11 节
> **相关文档**：[板级规范](../../../firmware/hpm_board/AGENTS.md) · [实测记录](../../../firmware/hpm_board/PCAN_PROTOCOL.md)

## 摘要

本目录实现独立的 PCAN-Basic CAN-FD ABI 兼容层，将意优原来的 API 调用转发到 HCS 板。
USB 使用 Windows 自带 WinUSB，设备有独立身份，不加载 PEAK 驱动或其 DLL。
固件仍是纯转发桥，电机协议与控制环留在上位机。

## 1. 配套关系

必须同时使用 `hpm_board_app_pcan_winusb` 固件与本 DLL。固件 VID/PID 为
`A511:F531`（单 CAN）/`A511:F532`（双 CAN），MS OS 2.0 描述符让 Windows 10/11
自动绑定 WinUSB，无需安装抓包软件或 PEAK 驱动。两路映射为 `0x51`、`0x52`。
已有 PEAK 驱动可以保留，它不匹配新身份。

只替换意优安装目录中的 `PCANBasic.dll`；保留 `eyou_canfdx.exe`、`eu_candrv.dll`
和其他文件。意优提供的程序为 32 位，因此必须搭配 x86 DLL，即使 Windows 是 x64。

## 2. 安装与恢复

关闭意优软件后，将构建出的 DLL 放到 `install.ps1` 旁，运行 `install.cmd`。
脚本请求管理员权限，自动查找标准安装目录，首次保存
`PCANBasic.before-libhcs.dll`；重复安装不会覆盖这份原 DLL 备份。
非标准目录在管理员 PowerShell 使用 `install.ps1 -InstallDir '<目录>'`。

恢复原 DLL 用 `install.ps1 -Restore`；板子也需刷回相配固件。
Windows 测试环境的磁盘覆盖层不会替用户实际 Windows 安装本 DLL。

## 3. 已实现的 API 范围

导出名、序号与 32 位 stdcall ABI 对齐意优调用的 PCAN-Basic 接口，提供
InitializeFD、ReadFD、WriteFD、Uninitialize、Reset、GetStatus、GetErrorText
和通道/版本/FD 时序查询。接收线程使用独立有界队列，按硬件时间戳返回 CAN 记录。
DLL 支持 API 初始化中的 80 MHz 时钟、CAN-FD ISO 与可表达的仲裁/数据 TQ 参数。

当前不实现经典 CAN 初始化/Read/Write、FilterMessages、SetValue、LookUpChannel，
这些接口返回不支持。不存在完整 PCAN-Basic 兼容、PCAN-View 或其他厂家上位机兼容承诺。
双通道由固件与 DLL 提供；实体电机实测通道范围以板级记录为准。

## 4. 重新构建

本目录是独立 CMake 工程，可用 Windows C11 工具链构建；x86 MinGW 示例：

```bash
cmake -S host/compat/pcan_basic -B <build> -DCMAKE_SYSTEM_NAME=Windows \
  -DCMAKE_C_COMPILER=i686-w64-mingw32-gcc
cmake --build <build>
```

链接 Windows SDK 的 `winusb`、`setupapi`。依赖为系统 DLL，不需要安装 MinGW 运行时。
固件构建与 DFU 命令的唯一规范见板级 AGENTS.md 第 9 节。
