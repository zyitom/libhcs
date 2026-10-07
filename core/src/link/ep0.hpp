#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>
#include <utility>

#include "core/include/libhcs/protocol/vendor_control.hpp"
#include "core/include/libhcs/spec/port.hpp"
#include "core/src/link/port.hpp"
#include "core/src/link/port_ops.hpp"
#include "core/src/link/registry.hpp"

// EP0 配置通道的纯逻辑核心 -- 请求分发、清单的两阶段提交、拒绝原因锁存。模板参数是
// 板子的端口注册表(PortRegistry)与板级上下文(Board), 三个固件与主机测试的假驱动
// 实例化的是这同一份代码: 测试跑的就是板上实际运行的逻辑。
//
// 板级上下文(Board)是固件保留话语权的地方:
//   void* reply_buffer()                     IN 应答的写字处(即固件的暂存缓冲)
//   record_error(request, wIndex, outcome)   拒绝原因锁存(各板自己的存储)
//   on_manifest_begin()                      清单通过校验、即将动端口: 从附加功能手里
//                                            接过板子(hpm), 别的板无事
//   on_manifest_accepted(now)                清单被完整应用: 归属落定(hpm)或置握手门
//                                            (mc02/c_board)
//   on_manifest_failed()                     应用中途失败、端口已整体挂起: 把板子还给
//                                            接手前的主人(hpm), 别的板无事
//   kBoardCaps                               本板的整板能力位(vc::BoardCapability),
//                                            kGetPortList 原样报出
//   set_time_sync(bool) -> bool              开/关共享时间基准; 只在 kBoardCaps 带
//                                            kBoardCapTimeSync 时会被要求打开, false =
//                                            没开起来(清单回滚)
//   now()                                    声明时限的时钟(只 hpm 真读)
//   read_latency(reset) -> bool              kGetLatencyBreakdown; 应答自行写进
//                                            reply_buffer(); false = 本板没有
//   journal()                                最近一次清单的逐口结果
//   fill_last_error(payload&)                锁存读回
//
// 这里不包含任何 USB 栈的头: SETUP/DATA 两段怎么搬运由固件胶水完成(三个固件共用
// firmware/common 的 ep0_staging.hpp), 核心只决定"答什么/收多少/为什么拒绝"。胶水侧
// 以一个 RequestView( requester 说的话: brequest()/windex()/wlength()/is_in() )把
// SETUP 包交给 setup(), 数据段到齐后把暂存字节交给 data()。

namespace libhcs::core::link::ep0 {

namespace vc = libhcs::core::protocol::vendor_control;
using link::PortOutcome;

// 最近一次清单的逐项结果。kApplyManifest 的处理过程逐项填写, kGetManifestResult 按
// 它应答。固定容量(清单本身有界), 无堆。
class ManifestJournal {
public:
    void reset() noexcept {
        count_ = 0;
        outcome_ = vc::ManifestOutcome::kManifestRefused;
    }

    void record(data::DataId data_id, std::uint8_t line, PortOutcome outcome) noexcept {
        if (count_ < std::size(entries_)) {
            entries_[count_] = {
                .data_id = std::to_underlying(data_id),
                .reason = std::to_underlying(outcome.reason),
                .line = line,
                .reserved = 0,
                .value = outcome.value,
            };
            ++count_;
        }
    }

    // 应用阶段的失败: 校验阶段的记录还在, 把失败项的条目改成失败原因 -- 主机读到的
    // 是"这一项为什么没生效", 而不是"校验时它还是好的"。
    void record_failure(data::DataId data_id, std::uint8_t line, PortOutcome outcome) noexcept {
        for (std::size_t i = count_; i > 0; --i) {
            if (entries_[i - 1].data_id == std::to_underlying(data_id)
                && entries_[i - 1].line == line) {
                entries_[i - 1].reason = std::to_underlying(outcome.reason);
                entries_[i - 1].value = outcome.value;
                return;
            }
        }
    }

    constexpr void set_outcome(vc::ManifestOutcome outcome) noexcept { outcome_ = outcome; }

    void fill(vc::ManifestResultPayload& out) const noexcept {
        out = {};
        out.entry_count = count_;
        out.outcome = std::to_underlying(outcome_);
        for (std::size_t i = 0; i < count_; ++i)
            out.entries[i] = entries_[i];
    }

private:
    vc::ManifestResultEntry entries_[spec::kMaxManifestEntries]{};
    std::uint8_t count_ = 0;
    vc::ManifestOutcome outcome_ = vc::ManifestOutcome::kManifestRefused;
};

// ---- 清单解析 ----

// 线上形状: 4 字节头 + n 个定长项(vendor_control.hpp 的 ManifestPayload)。
inline constexpr std::size_t kManifestHeader = offsetof(vc::ManifestPayload, entries);
inline constexpr std::size_t kManifestEntry = sizeof(vc::ManifestEntry);
static_assert(
    sizeof(vc::ManifestPayload) == kManifestHeader + kManifestEntry * spec::kMaxManifestEntries);

[[nodiscard]] constexpr bool manifest_length_ok(std::size_t length) noexcept {
    return length >= kManifestHeader && length <= sizeof(vc::ManifestPayload)
        && (length - kManifestHeader) % kManifestEntry == 0;
}

// 一条已解析的清单项: 身份 + 类型 + (GPIO 的)线号 + 设置字节(按口的类型解释)。
struct ManifestItem {
    data::DataId data_id;
    spec::PortKind kind;
    std::uint8_t line = 0; // GPIO 口的第几根线; 别的类型恒为 0
    // kind 的设置载荷(ConfigPayloadOf<kind>), 未用尾部为 0。
    std::array<std::byte, vc::kManifestSettingCapacity> setting{};
};

struct ParsedManifest {
    std::uint8_t flags = 0; // vc::ManifestFlag
    std::uint8_t count = 0;
    ManifestItem items[spec::kMaxManifestEntries]{};

    // 这一项(口 + 线)在清单里。
    [[nodiscard]] constexpr bool declares(data::DataId data_id, std::uint8_t line) const noexcept {
        for (std::uint8_t i = 0; i < count; ++i) {
            if (items[i].data_id == data_id && items[i].line == line)
                return true;
        }
        return false;
    }
};

// 把线上的清单字节解析成定长数组。载荷形状(wLength = 头 + n × 定长项)在 SETUP 段
// 已核对; 这里核对版本与逐项编码。任何一项不认识都让整个清单被拒 -- 解析是"全或无"
// 的, 与两阶段提交同一立场。
template <typename PayloadView>
[[nodiscard]] bool parse_manifest(PayloadView staged, ParsedManifest& out) noexcept {
    if (!manifest_length_ok(staged.size()))
        return false;
    const auto byte_at = [&staged](std::size_t i) { return static_cast<std::uint8_t>(staged[i]); };
    const auto version =
        static_cast<std::uint16_t>(byte_at(0)) | (static_cast<std::uint16_t>(byte_at(1)) << 8);
    // 不认识的整板请求位与不认识的项一样让整个清单被拒。
    constexpr std::uint8_t kKnownFlags = vc::kManifestFlagTimeSync;
    if (version != vc::kVersion || (byte_at(3) & ~kKnownFlags) != 0)
        return false;
    out.flags = byte_at(3);
    out.count = byte_at(2);
    if (out.count != (staged.size() - kManifestHeader) / kManifestEntry)
        return false;
    for (std::uint8_t i = 0; i < out.count; ++i) {
        const std::size_t base = kManifestHeader + static_cast<std::size_t>(i) * kManifestEntry;
        const auto kind = static_cast<spec::PortKind>(byte_at(base + 1));
        const std::uint8_t line = byte_at(base + 2);
        if (byte_at(base + 3) != 0)
            return false;
        if (kind != spec::PortKind::kCan && kind != spec::PortKind::kUart
            && kind != spec::PortKind::kImu && kind != spec::PortKind::kGpio
            && kind != spec::PortKind::kBuzzer)
            return false;
        // 线号只属于 GPIO 口。
        if (kind != spec::PortKind::kGpio && line != 0)
            return false;
        ManifestItem& item = out.items[i];
        item.data_id = static_cast<data::DataId>(byte_at(base));
        item.kind = kind;
        item.line = line;
        for (std::size_t b = 0; b < vc::kManifestSettingCapacity; ++b)
            item.setting[b] = static_cast<std::byte>(byte_at(base + 4 + b));
    }
    return true;
}

// ---- 按类型的分发 ----
//
// 注册表给出绑定与驱动实例, 这里按口的类型把设置字节还原成载荷并调用对应 kind 的
// 操作。kind 与口不符是 kConfigErrorKindMismatch -- 清单项的类型字段就是为了这一步
// 能便宜地核对。

// 一种口的设置载荷: 清单项的设置段与 kGetPortConfig 的应答都是它。
template <spec::PortKind kKind>
using ConfigPayloadOf = std::conditional_t<
    kKind == spec::PortKind::kCan, vc::CanConfigPayload,
    std::conditional_t<
        kKind == spec::PortKind::kUart, vc::UartConfigPayload,
        std::conditional_t<
            kKind == spec::PortKind::kImu, vc::ImuConfigPayload,
            std::conditional_t<
                kKind == spec::PortKind::kGpio, vc::GpioConfigPayload, vc::BuzzerConfigPayload>>>>;

template <spec::PortKind kKind>
[[nodiscard]] ConfigPayloadOf<kKind> setting_of(const ManifestItem& item) noexcept {
    ConfigPayloadOf<kKind> payload{};
    static_assert(sizeof(payload) <= vc::kManifestSettingCapacity);
    std::memcpy(&payload, item.setting.data(), sizeof(payload));
    return payload;
}

template <typename Binding, typename Driver>
[[nodiscard]] PortOutcome dispatch_validate(const ManifestItem& item, Driver& port) noexcept {
    constexpr spec::PortKind kKind = Binding::kind;
    if (item.kind != kKind)
        return PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorKindMismatch);
    const auto setting = setting_of<kKind>(item);
    if constexpr (kKind == spec::PortKind::kCan)
        return port_ops::can_validate<kCapabilitiesOf<Binding>>(port, setting);
    else if constexpr (kKind == spec::PortKind::kUart)
        return port_ops::uart_validate(port, setting);
    else if constexpr (kKind == spec::PortKind::kImu)
        return port_ops::imu_validate(port, setting);
    else if constexpr (kKind == spec::PortKind::kGpio)
        return port_ops::gpio_validate_line<Driver>(item.line, setting);
    else
        return port_ops::buzzer_validate(setting);
}

template <typename Binding, typename Driver>
[[nodiscard]] PortOutcome dispatch_apply(const ManifestItem& item, Driver& port) noexcept {
    constexpr spec::PortKind kKind = Binding::kind;
    const auto setting = setting_of<kKind>(item);
    if constexpr (kKind == spec::PortKind::kCan)
        return port_ops::can_apply<kCapabilitiesOf<Binding>>(port, setting);
    else if constexpr (kKind == spec::PortKind::kUart)
        return port_ops::uart_apply(port, setting);
    else if constexpr (kKind == spec::PortKind::kImu)
        return port.apply(setting);
    else if constexpr (kKind == spec::PortKind::kGpio)
        return port_ops::gpio_apply(*port.line(item.line), setting);
    else
        return port.apply(setting);
}

// ---- wIndex: 口的 DataId(低字节) + GPIO 口的线号(高字节) ----
struct PortIndex {
    data::DataId data_id;
    std::uint8_t line;
};

[[nodiscard]] constexpr PortIndex decode_index(std::uint16_t index) noexcept {
    return {
        .data_id = static_cast<data::DataId>(index & 0xFFU),
        .line = static_cast<std::uint8_t>(index >> 8U),
    };
}

// 应答载荷的写入: IN 的应答统一进胶水的暂存缓冲, 长度随 SetupDecision 上交。核心
// 经这里写的最大载荷是清单结果; 更大的没有 -- 时延分解(56 字节)由 hpm 板上下文自己写。
template <typename Board, typename Payload>
void stage_reply(Board& board, const Payload& payload) noexcept {
    static_assert(
        sizeof(Payload) <= sizeof(vc::ManifestResultPayload),
        "reply payload exceeds what the core stages");
    std::memcpy(board.reply_buffer(), &payload, sizeof(Payload));
}

// ---- 归属的事务半边 ----
//
// 校验通过后、动第一个端口之前开始, 作用域结束时要么已提交, 要么回退 -- 回退由析构
// 保证, 应用阶段的任何出口都不会把板子留在"接过来了却既没生效也没还回去"的状态。
template <typename Board>
class [[nodiscard]] ManifestClaim {
public:
    explicit ManifestClaim(Board& board) noexcept
        : board_(board) {
        board_.on_manifest_begin();
    }
    ManifestClaim(const ManifestClaim&) = delete;
    ManifestClaim& operator=(const ManifestClaim&) = delete;
    ~ManifestClaim() {
        if (!committed_)
            board_.on_manifest_failed();
    }

    void commit(std::uint64_t now) noexcept {
        board_.on_manifest_accepted(now);
        committed_ = true;
    }

private:
    Board& board_;
    bool committed_ = false;
};

// ---- kApplyManifest: 两阶段提交 ----
//
// 返回 false = STALL(原因已锁存)。校验失败: 板子、端口与归属都停在处理前的状态(附加
// 功能照常用它的口); 应用失败: 所有口挂起, 板子还给接手前的主人(见 ownership.hpp)。
template <typename Registry, typename Board, typename PayloadView>
bool apply_manifest(Board& board, PayloadView staged) noexcept {
    ParsedManifest manifest{};
    if (!parse_manifest(staged, manifest)) {
        board.record_error(
            vc::Request::kApplyManifest, 0,
            PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest));
        return false;
    }

    auto& journal = board.journal();
    journal.reset();

    // ---- 阶段一: 全部校验, 不碰硬件 ----
    // 整板请求: 要的能力本板得有。主机发清单前已按 board_caps 核对过, 这里是兜底。
    const bool time_sync = (manifest.flags & vc::kManifestFlagTimeSync) != 0;
    if (time_sync && (Board::kBoardCaps & vc::kBoardCapTimeSync) == 0) {
        board.record_error(
            vc::Request::kApplyManifest, 0,
            PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest));
        return false;
    }
    // 同一项(口 + 线)出现两次的清单形状上合法但语义上说不清, 先在这里挡掉。
    for (std::uint8_t i = 0; i < manifest.count; ++i) {
        for (std::uint8_t j = static_cast<std::uint8_t>(i + 1); j < manifest.count; ++j) {
            if (manifest.items[i].data_id == manifest.items[j].data_id
                && manifest.items[i].line == manifest.items[j].line) {
                const auto outcome = PortOutcome::refuse(
                    vc::ConfigErrorReason::kConfigErrorBadRequest,
                    std::to_underlying(manifest.items[i].data_id));
                journal.record(manifest.items[i].data_id, manifest.items[i].line, outcome);
                board.record_error(vc::Request::kApplyManifest, 0, outcome);
                return false;
            }
        }
    }

    for (std::uint8_t i = 0; i < manifest.count; ++i) {
        const ManifestItem& item = manifest.items[i];
        PortOutcome outcome = PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadIndex);
        (void)Registry::call(item.data_id, outcome, [&](auto binding, auto& port) {
            outcome = dispatch_validate<decltype(binding)>(item, port);
        }); // 找不到时 outcome 仍是 kConfigErrorBadIndex
        journal.record(item.data_id, item.line, outcome);
        if (!outcome.ok()) {
            // 一项不过, 一项不生效: 校验阶段没碰任何硬件, 板子整体保持原状, 归属
            // 不动(附加功能继续用它的口)。
            board.record_error(vc::Request::kApplyManifest, 0, outcome);
            return false;
        }
    }

    // ---- 阶段二: 接手, 应用 ----
    // 校验全部通过, 从这里起只有两种结局: 全部生效并提交归属, 或整体回滚并把板子还给
    // 接手前的主人。先接手再动端口: 附加功能看不到清单应用的任何中间态。声明的口
    // apply(配置 + 回读 + 上线)。
    ManifestClaim claim{board};

    // 时间基准先于端口: 端口一上线, 收到的第一帧就能打时间戳。开不起来与某个口应用失败
    // 同一结局 -- 端口还一个没动, 挂起全部(各口本就在挂起态)、把板子还回去。没要它的
    // 清单在这里把它关掉: 时间基准只在要了它的清单生效期间运行。
    if (!board.set_time_sync(time_sync)) {
        (void)board.set_time_sync(false);
        Registry::suspend_all();
        const auto outcome = PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorVerifyFailed);
        board.record_error(vc::Request::kApplyManifest, 0, outcome);
        journal.set_outcome(vc::ManifestOutcome::kManifestRolledBack);
        return false;
    }

    PortOutcome failure = PortOutcome::pass();
    const ManifestItem* failed_item = nullptr;
    for (std::uint8_t i = 0; i < manifest.count && failure.ok(); ++i) {
        const ManifestItem& item = manifest.items[i];
        failed_item = &item;
        PortOutcome miss{};
        const bool found = Registry::call(item.data_id, miss, [&](auto binding, auto& port) {
            failure = dispatch_apply<decltype(binding)>(item, port);
        });
        if (!found || !miss.ok())
            failure = PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadIndex);
    }
    if (!failure.ok()) {
        // 整体回滚: 所有口挂起(apply 过的口挂起即撤销, 还没轮到的本来就没动), 时间基准
        // 关掉。claim 析构时把板子还给接手前的主人 -- 归附加功能的板子随之恢复它的口。
        Registry::suspend_all();
        (void)board.set_time_sync(false);
        board.record_error(
            vc::Request::kApplyManifest,
            static_cast<std::uint16_t>(
                std::to_underlying(failed_item->data_id) | (failed_item->line << 8U)),
            failure);
        journal.record_failure(failed_item->data_id, failed_item->line, failure);
        journal.set_outcome(vc::ManifestOutcome::kManifestRolledBack);
        return false;
    }

    // 未声明的口与线: 清单就是这一轮声明的全部, 它们不留在总线上("配置即声明")。常开的
    // 板(c_board)的 CAN/UART/IMU 的 suspend() 是空操作, 由驱动自己说明。
    Registry::for_each_live([&manifest](auto binding, auto& port) {
        if constexpr (decltype(binding)::kind == spec::PortKind::kGpio) {
            using Driver = std::remove_cvref_t<decltype(port)>;
            for (std::uint8_t line = 0; line < Driver::kLineCount; ++line) {
                if (!manifest.declares(binding.data_id, line))
                    port.line(line)->suspend();
            }
        } else if (!manifest.declares(binding.data_id, 0)) {
            port.suspend();
        }
    });

    // 全部生效: 此刻才提交归属。声明与应用之间没有窗口 -- 两者本就是同一次传输。
    claim.commit(board.now());
    journal.set_outcome(vc::ManifestOutcome::kManifestApplied);
    return true;
}

// ---- 请求分发 ----
//
// SETUP 段的决策: 应答什么(IN)、接受多少数据段(OUT)、还是拒绝。
enum class SetupAction : std::uint8_t {
    kReply,     // IN: 载荷已写进 board.reply_buffer(), 长度见 reply_size
    kAcceptOut, // OUT: 接受 reply_size 字节的数据段, 值等 DATA 段再校验
    kStall,     // 拒绝(原因已按需锁存)
};

struct SetupDecision {
    SetupAction action;
    std::size_t reply_size = 0;
};

template <typename Registry, typename Board, typename RequestView>
SetupDecision setup(Board& board, const RequestView& request) noexcept {
    const auto brequest = static_cast<vc::Request>(request.brequest());
    const std::uint16_t index = request.windex();
    const bool is_in = request.is_in();

    switch (brequest) {
    case vc::Request::kGetPortList: {
        if (!is_in || index != 0)
            return {.action = SetupAction::kStall};
        // 纯读, 无任何副作用: 不动归属, 不停端口。归属切换与声明轮次都在
        // kApplyManifest。定长应答: 条目补零, 主机按 port_count 取用。
        vc::PortListPayload payload{};
        payload.version = vc::kVersion;
        std::size_t count = 0;
        Registry::for_each_live([&](auto binding, auto& port) {
            if (count >= spec::kMaxPorts)
                return;
            const PortStatus status = port.describe();
            payload.ports[count++] = {
                .data_id = std::to_underlying(binding.data_id),
                .kind = std::to_underlying(binding.kind),
                .capabilities = kCapabilitiesOf<decltype(binding)>,
                .status = static_cast<std::uint8_t>(
                    (status.running ? static_cast<unsigned>(spec::kPortRunning) : 0U)
                    | (status.fd ? static_cast<unsigned>(spec::kPortFd) : 0U)),
            };
        });
        payload.port_count = static_cast<std::uint8_t>(count);
        payload.board_caps = Board::kBoardCaps;
        stage_reply(board, payload);
        return {.action = SetupAction::kReply, .reply_size = sizeof(payload)};
    }

    case vc::Request::kGetPortConfig: {
        if (!is_in)
            return {.action = SetupAction::kStall};
        // 按口的类型取硬件事实: 载荷类型由 DataId 的类型决定, 请求本身不带期望类型。
        // GPIO 口按线答; 别的口的线号必须是 0。
        const PortIndex target = decode_index(index);
        PortOutcome miss{};
        std::size_t reply_size = 0;
        const bool found = Registry::call(target.data_id, miss, [&](auto binding, auto& port) {
            ConfigPayloadOf<decltype(binding)::kind> payload{};
            if constexpr (decltype(binding)::kind == spec::PortKind::kGpio) {
                using Driver = std::remove_cvref_t<decltype(port)>;
                if (target.line >= Driver::kLineCount) {
                    miss = PortOutcome::refuse(
                        vc::ConfigErrorReason::kConfigErrorBadIndex, target.line);
                    return;
                }
                port.line(target.line)->read_config(payload);
            } else {
                if (target.line != 0) {
                    miss = PortOutcome::refuse(
                        vc::ConfigErrorReason::kConfigErrorBadIndex, target.line);
                    return;
                }
                port.read_config(payload);
            }
            stage_reply(board, payload);
            reply_size = sizeof(payload);
        });
        if (!found || !miss.ok()) {
            board.record_error(
                vc::Request::kGetPortConfig, index,
                found ? miss : PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadIndex));
            return {.action = SetupAction::kStall};
        }
        return {.action = SetupAction::kReply, .reply_size = reply_size};
    }

    case vc::Request::kApplyManifest: {
        // 形状先行: 定长项, 项数由 wLength 反推, 上限是清单容量。错形状在这里拒绝,
        // 不占数据段。
        if (is_in || index != 0)
            return {.action = SetupAction::kStall};
        // 空清单(0 项 = 什么口都不用)合法: 板端把所有口留在挂起状态。
        if (!manifest_length_ok(request.wlength())) {
            board.record_error(
                vc::Request::kApplyManifest, index,
                PortOutcome::refuse(vc::ConfigErrorReason::kConfigErrorBadRequest));
            return {.action = SetupAction::kStall};
        }
        return {.action = SetupAction::kAcceptOut, .reply_size = request.wlength()};
    }

    case vc::Request::kGetManifestResult: {
        if (!is_in || index != 0)
            return {.action = SetupAction::kStall};
        vc::ManifestResultPayload payload{};
        board.journal().fill(payload);
        stage_reply(board, payload);
        return {.action = SetupAction::kReply, .reply_size = sizeof(payload)};
    }

    case vc::Request::kGetLastConfigError: {
        if (!is_in || index != 0)
            return {.action = SetupAction::kStall};
        vc::LastConfigErrorPayload payload{};
        board.fill_last_error(payload);
        stage_reply(board, payload);
        return {.action = SetupAction::kReply, .reply_size = sizeof(payload)};
    }

    case vc::Request::kGetLatencyBreakdown:
        if (!is_in)
            return {.action = SetupAction::kStall};
        if (!board.read_latency(request.windex() != 0))
            return {.action = SetupAction::kStall};
        return {.action = SetupAction::kReply, .reply_size = sizeof(vc::LatencyBreakdownPayload)};

    default: return {.action = SetupAction::kStall}; // 旧协议的与未知的一律拒绝
    }
}

// DATA 段: 只有 kApplyManifest 还有事可做。返回 false 会 STALL 状态段; IN 的应答
// 发出后也会触发 DATA 段, 直接放行。
template <typename Registry, typename Board, typename RequestView, typename PayloadView>
bool data(Board& board, const RequestView& request, PayloadView staged) noexcept {
    if (request.is_in()
        || static_cast<vc::Request>(request.brequest()) != vc::Request::kApplyManifest)
        return true;
    return apply_manifest<Registry>(board, staged);
}

} // namespace libhcs::core::link::ep0
