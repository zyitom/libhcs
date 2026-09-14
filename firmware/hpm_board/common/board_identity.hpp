#pragma once

#include <cstdint>

#include <hpm_otp_drv.h>

namespace libhcs::firmware::board {

// 本芯片焊在哪块物理 PCB 上, 运行时从出厂烧写的 OTP shadow 读取, 而非编译期
// 写死。
//
// 各板自行选择启用
//
// 只有 hpm5321 板启用它, 即在其 CMakeLists.txt 中定义 libhcs_BOARD_OTP_IDENTITY=1,
// 因为只有它的目录服务两块电气不同的 PCB。其余板保持单板型身份, 无条件识别并
// 上报编译期 PID, 因此下面的共享 bootloader/app 代码在调用点无需 #if。
//
// 该开关不是摆设。拒绝路径以 word 25 的两个特定值为准, 而这两个值来自
// hpm5321 的实测: boards/hpm6e8y/README.md 记录该系列芯片 word 25 =
// 0x00000006。若不加开关, 所有 hpm6e8y 与 hpm6e80ivm1 都会被判为未识别而拒绝
// 启动 -- 与本机制要解决的问题毫无关系的硬件也会遭殃。
//
// 存在的原因
//
// 本项目的两块 HPM5321 PCB 在两个焊盘上电气不兼容: PA30/PA31 在单 CAN 板上是
// 绿/红 LED 阴极, 在双 CAN-FD 板上是 MCAN3 RXD/TXD。同一镜像在不知道自己跑在
// 哪块板上之前无法配置这些焊盘, 且双->单方向的错误会让收发器输出灌进 LED
// 网络。镜像必须问硬件, 且拒绝猜测。
//
// 证据实际支持什么  [实测 2026-08-05, 4 chips]
//
// 采样的两块单 CAN 板 word 25 均读 0, 两块双 CAN-FD 板均读 2。四份 dump 之间
// 其余差异(word 5 批次号、word 21 TSNS trim、words 88-91 UUID)都是每颗 die
// 各异。
//
// 这不能证明 word 25 编码的是板型。四个样本按批次排列 -- lots 375/377 是双
// CAN 板, 378/380 是单 CAN 板 -- 样本中"板型"与"生产批次"完全混同, 且
// word 25 在 SDK 的 hpm_otp_table.h 中无定义(HPM6E8Y 的板级笔记称该字为
// program-count/config 标志)。要区分两者需要来自交错批次的反例: lot 375-377
// 的单 CAN 板, 或 lot >=378 的双 CAN 板。
//
// 下面的严格性正是可接受的原因: 只接受两个观测值, 其余一律停止启动而非选
// 默认。若真是批次痕迹, 会表现为响亮、可排查的拒绝运行, 而不是配置错误的
// 焊盘。
//
// 读取的代价与安全性
//
// 每次启动一次 MMIO 读。otp_read_from_shadow() 检查下标后读
// HPM_OTP->SHADOW[index]; SDK 中 otp_init()/otp_deinit() 为空, 读路径没有时钟
// 门控与命令序列。烧写 OTP 是完全独立的机制(2.5 V LDO 使能、UNLOCK 魔数、写
// FUSE[]), 读操作不涉及 -- 因此读不会磨损或损坏阵列。固件两半本来每次启动就
// 读四个 OTP shadow 字用于 UUID 派生 USB 序列号, 这里加第五个。
enum class BoardVariant : uint8_t {
    kSingleCan, // 单路 CAN, RGB LED 在 PA29/PA30/PA31, 无 CAN 指示灯
    kDualCanFd, // 双路 CAN-FD(MCAN0 + MCAN3 用 PA30/PA31), LED 在 PA26/PA27/PA28
    kFixed,     // 单板型板: 无需判别
    kUnknown,   // word 25 不是任一已知值 -- 拒绝运行
};

// 本板是否从 OTP 解析板型。所有单板型板均为 false, 其身份是 kFixed 且恒被
// 识别。
inline constexpr bool kOtpIdentityEnabled =
#if defined(libhcs_BOARD_OTP_IDENTITY) && libhcs_BOARD_OTP_IDENTITY
    true;
#else
    false;
#endif

// 承载判别字的 OTP shadow word, 以及本固件愿意采信的仅有的两个值。
inline constexpr uint32_t kVariantOtpIndex = 25U;
inline constexpr uint32_t kVariantOtpValueSingleCan = 0U;
inline constexpr uint32_t kVariantOtpValueDualCanFd = 2U;

struct BoardIdentity {
    BoardVariant variant = BoardVariant::kFixed;

    // 实际读到的原值, 供拒绝启动时报告肇因数值而非只说"未知"。排查无法启动
    // 的板子时就报这个数。
    uint32_t otp_word = 0U;

    [[nodiscard]] constexpr bool recognized() const { return variant != BoardVariant::kUnknown; }

    [[nodiscard]] constexpr bool dual_can() const { return variant == BoardVariant::kDualCanFd; }
};

inline BoardIdentity read_board_identity() {
    BoardIdentity identity;

    // 单板型板完全不读 OTP: 无物可判, 且下面两个接受值是 hpm5321 的数字, 会
    // 把那些芯片直接判死。
    if constexpr (!kOtpIdentityEnabled) {
        identity.variant = BoardVariant::kFixed;
        return identity;
    }

    identity.otp_word = otp_read_from_shadow(kVariantOtpIndex);

    if (identity.otp_word == kVariantOtpValueSingleCan) {
        identity.variant = BoardVariant::kSingleCan;
    } else if (identity.otp_word == kVariantOtpValueDualCanFd) {
        identity.variant = BoardVariant::kDualCanFd;
    } else {
        identity.variant = BoardVariant::kUnknown;
    }

    return identity;
}

// 全镜像范围缓存: 芯片上电期间 OTP 不会变, 每次启动读一次即可, 且所有调用方
// 看到同一答案。焊盘配置路径的调用方(board_app.cpp)依赖这一点 -- 两次读取
// 不一致会让 MCAN3 与 LED 的配置互相打架。
inline const BoardIdentity& board_identity() {
    static const BoardIdentity kIdentity = read_board_identity();
    return kIdentity;
}

} // namespace libhcs::firmware::board
