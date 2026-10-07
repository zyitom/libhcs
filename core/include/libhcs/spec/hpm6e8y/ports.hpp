#pragma once

#include <array>

#include <libhcs/data/datas.hpp>
#include <libhcs/spec/port.hpp>

// hpm6e8y 的口: 叫什么、是什么类型。主机接线表(Spec::kCans.kCan0)与固件绑定
// (firmware/hpm_board/boards/hpm6e8y/app/ports.hpp)共用这一份名字。每个口能做什么由
// 固件报告(kGetPortList), 不在这里。
//
// 只有四路 CAN。板上的 UART1(PY06/PY07)是调试焊盘, 不是对外连接器: 生产镜像里不带
// 它的驱动, 它也就不是一个口。
namespace libhcs::spec::hpm6e8y {

struct Spec {
    using Can = TypedPortDescriptor<Spec, PortKind::kCan>;
    using Uart = TypedPortDescriptor<Spec, PortKind::kUart>; // 本板没有 UART 口

    // 丝印 CAN0..CAN3(= MCAN0..MCAN3)。本板是唯一从 CAN0 起编的板型 -- DataId 即丝印号。
    struct Cans {
        static constexpr Can kCan0{data::DataId::kCan0};
        static constexpr Can kCan1{data::DataId::kCan1};
        static constexpr Can kCan2{data::DataId::kCan2};
        static constexpr Can kCan3{data::DataId::kCan3};
    };
    struct Uarts {};
    static constexpr Cans kCans{};
    static constexpr Uarts kUarts{};

    static constexpr std::array kPorts{
        port(Cans::kCan0),
        port(Cans::kCan1),
        port(Cans::kCan2),
        port(Cans::kCan3),
    };
};
static_assert(no_duplicate_data_ids(Spec::kPorts));
static_assert(manifest_fits(Spec::kPorts, 0));

} // namespace libhcs::spec::hpm6e8y
