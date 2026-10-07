#pragma once

#include <array>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/port.hpp>

// hpm5321 的口: 叫什么、是什么类型。主机接线表(Spec::kCans.kCan1)与固件绑定
// (firmware/hpm_board/boards/hpm5321/app/ports.hpp)共用这一份名字。
//
// 一份镜像服务两块 PCB: 单 CAN 板(PID 0x5321)与双 CAN 板(0x5322), 由 OTP word 25 区分。
// 口表因此是镜像的容量; 本 PCB 实际有哪些口、每个口能做什么, 由固件按上电时初始化了
// 哪些驱动报告(kGetPortList) -- 单 CAN 板上 CAN2 的焊盘实为 LED 阴极, 驱动不初始化,
// 口清单里就没有它。
namespace libhcs::spec::hpm5321 {

struct Spec {
    using Can = TypedPortDescriptor<Spec, PortKind::kCan>;
    using Uart = TypedPortDescriptor<Spec, PortKind::kUart>;

    // 丝印 CAN1 / CAN2。hpm6e8y 从 CAN0 起编, 本板从 CAN1 起 -- DataId 就是丝印号。
    struct Cans {
        static constexpr Can kCan1{data::DataId::kCan1};
        static constexpr Can kCan2{data::DataId::kCan2};
    };
    // 丝印 UART0, 板上唯一的数据串口。
    struct Uarts {
        static constexpr Uart kUart0{data::DataId::kUart0};
    };
    static constexpr Cans kCans{};
    static constexpr Uarts kUarts{};

    // 镜像的全部口: 固件绑定与它一一对应, 主机按它编译期展开回调分发。
    static constexpr std::array kPorts{
        port(Cans::kCan1),
        port(Cans::kCan2),
        port(Uarts::kUart0),
    };
};
static_assert(no_duplicate_data_ids(Spec::kPorts));
static_assert(manifest_fits(Spec::kPorts, 0));

} // namespace libhcs::spec::hpm5321
