# 主机侧时间子系统重构设计

> **文档类型**：背景说明（设计方案；已按第 6 节步骤实施，留作设计记录与索引)
> **适用范围**：`host/`（`libhcs/time/*`、`protocol/handler`、`board/board.hpp`）与 HCS 板层（`hcs_core/src/hardware/board/`）；固件、`core/` 的数据视图与线格式**不动**
> **状态**：第 1–5 步已实施（2026-10-07；含第 5 步收尾：旧回调签名与 `Timeline::instance()` 已删，`Timeline` 改名 `UsbFrameAxis` 并移进 `host/src/time/`，本节所述公开类型只剩 `SampleTime`）；第 6、7 步视需要
> **相关文档**：[CONTROL_TIMING.md](../firmware/hpm_board/CONTROL_TIMING.md)（控制环要的是数据年龄） · [SOF_TIMEBASE.md](../firmware/hpm_board/SOF_TIMEBASE.md)（时间轴机制与实测） · [core/PROTOCOL.md](../core/PROTOCOL.md)

## 摘要

今天使用者要理解六个公开类型（`Timeline`、`AxisMap`、`BoardClock`、`MicroframeTimebase`、`MicroframeSource`、
`SofStamp`）才能知道"这个样本是什么时候的"，换算逻辑漏到了 HCS（`board.hpp` 自己拼），时间轴是进程级单例且
名字写死 USB。重构后主机 SDK 在 IO 线程解码时为每个上行样本算好一个 `SampleTime`——本机时刻（永远有，附带来源）
与板上时刻（64 位、不回绕，算样本间隔用）——作为回调的一个参数交给使用者；内部是带单位的 chrono 类型、一个
可组合可求逆的仿射映射、每个主机控制器一条的时间轴。固件代码与线格式不动；但版本门按产品字符串整串比对，
打 v4.0.0 tag 后要按新树把全部板子重刷一次（用户决定 2026-10-07）。

## 本文导航

- 第 1 节：谁要什么——四种需求决定 API 的形状
- 第 2 节：已定的决定（与理由）
- 第 3 节：现状的问题
- 第 4 节：设计（类型、映射、时间轴、板钟、换算在哪做、怎么交给使用者、线程）
- 第 5 节：公开 API 的最终形态与 HCS 侧改动
- 第 6 节：实施步骤、每步的验收
- 第 7 节：实施须知（约束、命令、台架）
- 第 8 节：动作层（可选，暂不做）
- 第 9 节：考虑过、不采用的方案

## 1. 谁要什么

| 使用者 | 要什么 | 今天在哪 |
|---|---|---|
| 控制环 | 这个样本在本机 `CLOCK_MONOTONIC` 上是什么时候的（数据年龄 = 现在 − 它） | 实施前：HCS `devices.hpp` 的 `feedback_time()`、`sample_time()`（没有调用方，图上也读不到）；实施后：CAN 设备与板载 IMU 的 `<名字>/time` 输出（2026-10-07；串口设备不取，见第 5 节） |
| IMU 积分 | 相邻两个样本在**那块板的钟**上隔了多久（dt），不经主机换算 | HCS `bmi088.hpp` 拿 `timestamp_quarter_us` 相减 |
| 台架 / 诊断 | 原始 24 位 `SofStamp`：两块板对同一帧的时间戳之差就是跨板误差（16–50 ns 就是这么量的） | `hcs_core/test/test_bench_boards.cpp` |
| 动作层（未定） | 反方向：本机某个时刻 → 各板的板上时刻 | 没有（第 8 节） |

## 2. 已定的决定

1. **保留板上时刻**（用户确认 2026-10-07）。只给本机时刻不够：
   - 板上时间戳不开时间基准也一直有；本机时刻要声明时间基准并等它锁定（c_board 实测约 3 s）才有。只给本机时刻
     会让 IMU 积分依赖时间同步，且启动后头几秒没有 dt。
   - 没有板上戳时本机时刻只能退回"到达时刻"，带 USB 抖动（0.1–1 ms），拿来算 2 kHz 陀螺仪的 dt（500 µs）不可用。
   - 换算直线每 50 ms 更新一次，跨更新点的那个 dt 多一个亚微秒台阶——这一条影响可忽略，只是记录在案。
2. **`SampleTime` 不进 `core/` 的数据视图**。`CanDataView`、`Imu*DataView` 等固件也在用（CAN 中断里就地构造
   再序列化）；加一个只有主机用的字段（约 32 字节）会让固件每帧多清零一次。时间作为**主机回调的参数**交出去
   （4.6）。`DataCallback` 虽定义在 `core/include/libhcs/data/datas.hpp`，只有主机用（`host/src/protocol/handler.cpp`、
   `host/include/libhcs/{protocol/handler,board/board}.hpp`），可以改。
3. **时间轴每个主机控制器一条**，不是"去掉单例"：控制器的帧计数器是真实的进程级物理资源；不同控制器上的板
   不在同一条 SOF 流上（今天只靠日志提醒"插到同一个控制器"）。
4. **现在不建后端虚接口**：只有 USB 一个实现；SocketCAN 开工时再从干净的内部边界抽（4.8）。
5. **动作层暂不做**，但映射可逆，反方向换算免费提供（第 8 节）。
6. **v4.0.0 随 tag 重刷**（用户决定 2026-10-07）：版本号由 `git describe` 生成，版本门
   （`host/src/transport/usb/device_scanner.hpp`）按 "HCS Agent v<版本>" 整串比对。打 v4.0.0 tag 时全部板子按
   tag 的树重编重刷；版本门本身不改。

## 3. 现状的问题

1. 公开类型太多，换算要使用者自己拼：HCS `BoardCore` 有 `axis_map_`、`board_clock_inbox_`、`board_clock_`、
   `timeline_`、`feedback_time(stamp, around)`、`board_time(quarter_us)`，`CanBus`/`Can<>` 用 `tick.scheduled`
   当参照展开 24 位戳。
2. `Timeline::instance()` 进程级单例，`use_controller_of_usb_bus()` 写死 USB。
3. `AxisMap`（微帧 → 主机）与 `BoardClock`（板上 quarter-us → 微帧）都是"参考点 + 斜率"，各写一遍，单位靠字段名区分。
4. `Timeline::observe()`/`anchor_for()` 内部用 `Clock::now()` 的地方与纯算术混在一起，只能上硬件验证。
5. `host/src/time/timeline.hpp` 是已提交的 0 字节文件。
6. 32 位板上时钟的展开散落在 HCS（`device/board_clock_lifter.hpp`、`bmi088.hpp` 的无符号减法）。

## 4. 设计

### 4.1 带单位的时钟类型

```cpp
// host/include/libhcs/time/sample_time.hpp（公开）
namespace libhcs::time {
using HostClock = std::chrono::steady_clock;           // 本机 CLOCK_MONOTONIC
using HostTime = HostClock::time_point;
struct BoardClock {                                    // 一块板自己的 1/4 us 计时器，展开到 64 位
    using rep = std::int64_t;
    using period = std::ratio<1, 4'000'000>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<BoardClock>;
    static constexpr bool is_steady = true;
};
}
// host/src/time/frame_clock.hpp（内部）
namespace libhcs::host::time {
struct FrameClock {                                    // 一个主机控制器的 USB 微帧轴，Q16 小数
    using rep = std::int64_t;
    using period = std::ratio<1, 8000LL * 65536>;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<FrameClock>;
    static constexpr bool is_steady = true;
};
}
```

注意命名过渡：旧的 `libhcs::host::time::BoardClock`（`board_clock.hpp`，结构体）与新的 `libhcs::time::BoardClock`
（chrono 时钟）在不同命名空间，第 5 步删掉旧的之前两者并存，写代码时用全名。

### 4.2 仿射映射：一个模板代替 `AxisMap` 与旧 `BoardClock`

```cpp
// host/src/time/affine.hpp（内部）
template <class From, class To>
struct Affine {                       // 不可变 POD，可放进 seqlock
    typename From::time_point from0{};
    typename To::time_point to0{};
    double rate = 0.0;                // 每个 From 刻度折合多少 To 刻度；0 = 无效
    [[nodiscard]] bool valid() const noexcept { return rate > 0.0; }
    [[nodiscard]] typename To::time_point operator()(typename From::time_point t) const noexcept {
        return to0 + typename To::duration{std::llround(double((t - from0).count()) * rate)};
    }
    [[nodiscard]] Affine<To, From> inverse() const noexcept; // {to0, from0, 1 / rate}
};
template <class A, class B, class C>
[[nodiscard]] Affine<A, C> compose(const Affine<B, C>& outer, const Affine<A, B>& inner) noexcept;
// = {inner.from0, outer(inner.to0), inner.rate * outer.rate}
```

- 只对"离参考点的差"做 double 乘法（差值受拟合窗口约束，远小于 2^53），参考点是整数刻度——与今天
  `Timeline::refit_locked()` 防精度丢失的手法相同，写进类型一次。
- 由旧结构体换过来的两个工厂（第 1 步用来证明等价）：
  - `AxisMap` → `Affine<FrameClock, HostClock>`：`from0 = llround(reference_microframe * 65536)`，
    `rate = period_ns / 65536`，`to0 = reference_ns` 再按 `from0` 的舍入量补正（保证与 `AxisMap::time_of` 同值）。
  - `TimeStatusView` → `Affine<BoardClock, FrameClock>`：`from0 = 板钟展开(timestamp_quarter_us)`（4.4），
    `to0 = microframe * 65536 + microframe_fraction_q16`，`rate = 65536 * 65536 / ticks_per_microframe_q16`；
    `state != kValid` 或 `ticks_per_microframe_q16 == 0` 时无效（与旧 `BoardClock::from` 同判据）。
- 单元测试：组合律、`inverse()` 往返、跨大 `int64` 值的精度、两个工厂与旧公式逐值比较。

### 4.3 时间轴：每个主机控制器一条

第 1–4 步**不新建类**，把 `Timeline` 改成可多实例 + 注册表，减少搬动；第 5 步移进 `host/src/time/` 时改名
`UsbFrameAxis`。

```cpp
class Timeline {                                       // 第 5 步起：host/src/time/usb_frame_axis.hpp
public:
    // 同一控制器（按 pci_device_for_usb_bus(bus) 的 PCI 地址区分；查不到则共用 "" 这一项）上的板拿到同一个
    // 对象，进程内常驻（注册表持有 shared_ptr）。第一次拿到时 use_controller_of_usb_bus(bus)。
    static std::shared_ptr<Timeline> for_usb_bus(int bus_number);
    // 过渡期保留：返回"默认轴"——第一条被 for_usb_bus() 拿到的轴；在此之前被调用则先建一条未绑定的默认轴，
    // 之后第一个 for_usb_bus() 认领它。单控制器进程里 instance() 与板子用的是同一个对象，旧代码照常工作。
    // 第 5 步删除。
    static Timeline& instance();

    std::uint64_t anchor_for(HostTime when) const;     // 不变
    void observe(double microframe, HostTime sent, HostTime received); // 不变
    [[nodiscard]] AxisMap axis_map() const noexcept;   // 不变（seqlock，无锁）
    [[nodiscard]] Affine<FrameClock, HostClock> map() const noexcept; // 新增：axis_map() 换成 Affine
    // 其余成员不变
};
```

- **只搬家、重组，不改算法**：往返中点、1024 样本最小二乘、±3% 斜率门、1 s 断轴重置、控制器计数器优先，
  全部原样。
- 把纯算术抽成不取 `now()` 的自由函数（`host/src/time/axis_fit.hpp` 已有一部分）：时刻全作为参数传入，单元测试
  与等价性对拍才能离线做。

### 4.4 板钟：每块板一个，IO 线程独占

```cpp
// host/src/time/board_timebase.hpp（内部），Handler 成员
class BoardTimebase {
public:
    // 32 位读数展开到 64 位：与上一次展开结果的差按 int32 解释（容许小幅倒退：IMU、GPIO、对时上报交错到达）。
    [[nodiscard]] libhcs::time::BoardClock::time_point extend(std::uint32_t raw) noexcept;
    void observe(const data::TimeStatusView& status) noexcept;   // 用 extend(status.timestamp_quarter_us) 建 4.2 的映射
    [[nodiscard]] const Affine<libhcs::time::BoardClock, FrameClock>& map() const noexcept;
    void reset() noexcept;                                         // 会话重建（新 kStart）时
};
```

板上时刻对外就是 64 位、不回绕的；HCS 的 `board_clock_lifter.hpp` 第 4 步删除。所有经同一个 Handler 的板上
时间戳（对时上报、IMU 三路、带戳 GPIO）都过同一个 `extend()`，按到达顺序。

### 4.5 换算在哪做：IO 线程、解码当时（纯函数）

```cpp
// host/src/time/sample_timer.hpp（内部，纯函数，单元测试直接喂数据）
libhcs::time::SampleTime time_of_stamped(          // CAN 帧带 SofStamp
    std::optional<libhcs::time::SofStamp> stamp, HostTime arrival,
    const Affine<FrameClock, HostClock>& axis);
libhcs::time::SampleTime time_of_board_ticks(      // IMU / 带戳 GPIO 的 quarter-us
    std::optional<libhcs::time::BoardClock::time_point> board, HostTime arrival,
    const Affine<FrameClock, HostClock>& axis,
    const Affine<libhcs::time::BoardClock, FrameClock>& board_map);
```

- **到达时刻**：每个 USB 接收传输完成时、喂给反序列化器之前读一次 `steady_clock::now()`，这一传输里的所有
  记录共用它（每传输一次读时钟，不是每记录一次）。
- **CAN 帧**：用 `axis.inverse()(arrival)` 求到达时的微帧位置，以它为参照 `libhcs::time::unwrap()` 展开 24 位戳
  （比今天 HCS 用 `tick.scheduled` 当参照更准），再经 `axis` 到本机时刻；`source = kBoardStamp`，`board` 为空
  （CAN 戳在微帧轴上，不在板钟上）。
- **IMU / 带戳 GPIO**：`board = extend(raw)`；轴与板钟都有效时 `host = compose(axis, board_map)(board)`，
  `source = kBoardStamp`。
- **其余情况**（没有戳、没开时间基准、轴或板钟未锁定）：`host = arrival`，`source = kHostArrival`；IMU/GPIO 的
  `board` 照样填（板上时间戳总在）。
- **UART**：没有板上戳，`host = arrival`、`source = kHostArrival`、`board` 为空——串口设备（如 HI91 IMU）也能
  算数据年龄。
- 与今天 HCS"每拍开头取一份映射"的取舍：一次映射更新让直线挪动亚微秒，对数据年龄是噪声以下；换来 HCS 不再
  持有任何时间换算状态。第 3 步的台架对比要把这个数量出来写进本节——**还没测**：实施那天台架没接板子；
  旧路径已删，补测要从 git 里取改动前的 `AxisMap::time_of` 与新 `SampleTime.host` 在同一条台架用例里对拍。
  换算算术本身的等价性已由 `time_affine_test.cpp`、`time_axis_test.cpp` 逐值钉住。

### 4.6 怎么交给使用者：回调的一个参数

`SampleTime` 不进视图（决定 2）。主机回调接口新增带时间的重载；过渡期旧签名保留并由新签名的默认实现转调，
每一步都能编过：

```cpp
// core/include/libhcs/data/datas.hpp 的 DataCallback（只有主机用），以 CAN 为例：
[[nodiscard]] virtual bool can_receive_callback(DataId id, const CanDataView& data,
                                                const libhcs::time::SampleTime& time) {
    (void)time;
    return can_receive_callback(id, data);             // 过渡：默认转调旧签名
}
[[deprecated]] [[nodiscard]] virtual bool can_receive_callback(DataId id, const CanDataView& data) = 0;
// uart / gpio_digital / accelerometer / gyroscope / temperature 同样处理；gpio_analog 无戳，同样加 time（arrival）。
// host/include/libhcs/board/board.hpp 的 Board<Spec>::Callback 同法：按描述符的回调也加 SampleTime 参数。
```

- `DataCallback` 的头文件依赖：`sample_time.hpp` 放在 `host/include`，而 `DataCallback` 在 `core/include`。实施时
  二选一（推荐前者）：(a) 把 `DataCallback` 挪到 `host/include/libhcs/data/callback.hpp`（它本来就只属于主机）；
  (b) `sample_time.hpp` 放 `core/include/libhcs/time/`（只含类型，固件不包含就零成本）。
- 第 5 步删掉旧签名，新签名改为纯虚。

### 4.7 线程

| 对象 | 写者 | 读者 | 同步 |
|---|---|---|---|
| `Timeline`（每控制器）的拟合 | 各 Handler 的 IO 线程（每块板每 50 ms 一次 `observe`） | 各 IO 线程换算时 | 互斥量护写；映射用现成的 seqlock（`detail::PublishedAxisMap`）发布，读无锁 |
| `BoardTimebase` | 本 Handler 的 IO 线程 | 同一线程 | 无 |
| `SampleTime` | IO 线程算好 | 回调参数 | 值传递 |

控制环（HCS 周期域）只读样本里已经算好的值，不碰任何时间对象：周期域里没有锁、没有 seqlock 重试。

### 4.8 后端缝（现在不建虚接口）

`Timeline`（第 5 步后 `UsbFrameAxis`）+ `BoardTimebase` + `sample_timer` 的输入输出都是纯数据（对时上报、往返、
到达时刻 → 映射 → `SampleTime`）。SocketCAN 开工时（内核 `SO_TIMESTAMPING` 直接给本机时刻，`source = kBoardStamp`
语义改称"硬件戳"，映射是恒等）从这条边界抽出 `TimeSource`，两个真实实现对着写。

## 5. 公开 API 的最终形态与 HCS 侧改动

```cpp
namespace libhcs::time {
using HostTime = std::chrono::steady_clock::time_point;
struct BoardClock { /* 4.1 */ };
struct SampleTime {
    enum class Source : std::uint8_t { kHostArrival = 0, kBoardStamp = 1 };
    HostTime host{};                                // 永远有：数据年龄 = 现在 - host
    Source source = Source::kHostArrival;           // 精度：板戳约 0.1–1 µs，到达时刻带 USB 抖动
    std::optional<BoardClock::time_point> board;    // 板上时刻（64 位、不回绕），IMU/GPIO 有，CAN/UART 无
    friend constexpr bool operator==(const SampleTime&, const SampleTime&) = default;
};
}
// 回调：can / uart / gpio_digital / gpio_analog / accelerometer / gyroscope / temperature 都多一个
// const libhcs::time::SampleTime& 参数（4.6）。原始 SofStamp 仍在 CanDataView::sof_stamp（诊断用）。
// Handler（反方向，求逆，零成本；第 8 节用）：
std::optional<libhcs::time::BoardClock::time_point> board_time_at(libhcs::time::HostTime when) const;
```

**HCS 侧**（第 4 步）：

- `board.hpp` 的 `Callback`：`bus->receive(..., data.sof_stamp)` 改为传 `SampleTime`；IMU 样本带 `SampleTime`。
- `devices.hpp`：`Can<>` 收件箱里存 `SampleTime` 代替 `optional<SofStamp>`；`feedback_time()` 改为返回最近一帧的
  `SampleTime`（或其 `host`），不再调 `board_.feedback_time()`；IMU 的 `sample_time()` 同理。
  **补（2026-10-07）**：两个访问器合并成 `TimedDevice::time()`（`CanDevice`、`ImuDevice` 的基类），并作为
  `<名字>/time` 输出（`std::optional<SampleTime>`）发到图上——控制器经输入读它，不碰板组件对象。
  **串口设备不取时刻**（用户决定 2026-10-07）：线上没有戳，到达时刻里是模块内处理 + 串口传输（921600 下
  一帧 82 字节约 0.9 ms）+ USB 抖动，与样本时刻差得比它想说明的量还大；各 IMU 本来就各自自由触发、不锁相，
  拿它对齐也没有意义。SDK 照样把到达时刻交给回调，HCS 的板层不往下传。
- `device/imu_sample.hpp` 加 `libhcs::time::SampleTime time`；`bmi088.hpp` 的 dt 改用 `time.board` 相减
  （为空时保持今天的无符号减法——事实上 IMU 样本总有 `board`）。
- 删除：`BoardCore` 的 `axis_map_`、`board_clock_inbox_`、`board_clock_`、`timeline_`、`feedback_time(stamp, around)`、
  `board_time()`、`axis_map()`、`receive_time_status()` 里的 `BoardClock` 发布；`board.cpp` 第 276、326–329 行
  的取轴逻辑；`device/board_clock_lifter.hpp`（唯一的另一个使用者 `deprecated_reference_rmcs_real_car.cpp` 不在构建里，
  2026-10-07 经用户同意一并删除）。
- `test_board.cpp`：假 SDK 的 `inject_can`/`inject_accelerometer` 直接给 `SampleTime`；"轴锁定后反馈带时刻"
  那条用例（约第 440–480 行，现在用 `timeline().observe()` 造轴）改为验证"SDK 给的 `SampleTime` 原样到达设备"——
  换算本身的测试搬到 libhcs `host/tests/`。

**退出公开头文件**（第 5 步）：`timeline.hpp`、`axis_map.hpp`、`board_clock.hpp`、`microframe_timebase.hpp`、
`microframe_source.hpp` 移进 `host/src/time/`；删空文件 `host/src/time/timeline.hpp`。`SofStamp` 留在
`core/include`（线格式类型）。

## 6. 实施步骤

每一步结束时：libhcs `ctest` 全过、HCS `tools/check.sh` 全过、四块固件照常编过（固件不该有任何改动）；第 3 步
之前对外 API 不变。

1. **类型、`Affine`、两个工厂**（`host/src/time/{frame_clock,affine}.hpp`，公开的 `sample_time.hpp`）。
   新单元测试 `host/tests/time_affine_test.cpp`：4.2 所列。**等价性**：随机生成数千组 `AxisMap` / `TimeStatusView`
   与输入时刻，新工厂 + `Affine` 的结果与旧 `AxisMap::time_of` / `BoardClock::microframe_at` 之差 ≤ 1 ns（写进测试）。
2. **`Timeline` 可多实例 + 注册表**（4.3），纯算术抽成自由函数；Handler 改持 `shared_ptr<Timeline>`
   （`attach_time_source()` 里 `for_usb_bus(bus)`），`send_time_anchor()`、`time_status_deserialized_callback()`
   改用它。**等价性**：把改动前的 `Timeline` 拷一份到测试里作参照（不进产品代码），随机的 `observe` / `anchor_for`
   序列喂给新旧两份，锚点与映射逐值相等（做法同 [SOF_TIMEBASE.md](../firmware/hpm_board/SOF_TIMEBASE.md) 4.3 固件
   timebase 搬家时的对拍）。
3. **`BoardTimebase`、`sample_timer`、回调带 `SampleTime`**（4.4–4.6），旧签名保留、标 `[[deprecated]]`。
   新单元测试 `host/tests/sample_time_test.cpp`：喂构造的戳、到达时刻、映射，检查四种情况（CAN 有戳 / IMU 有戳
   / 未锁定退回到达时刻 / UART）与 32 位回绕。**台架对比**：在一条台架用例里同时算旧路径（`AxisMap::time_of`、
   `BoardClock`——第 5 步前还在）与新 `SampleTime.host`，打印差的分布（预期亚微秒），写进 4.5。
4. **HCS 改用 `SampleTime`**（第 5 节 HCS 侧）。台架：时间戳用例改读 `SampleTime`，数值与改前一致。
5. **收尾**：删旧回调签名与 `Timeline::instance()`/`timeline()`、`Timeline` 改名 `UsbFrameAxis` 并移进 `host/src/time/`，
   第 5 节所列头文件移走，台架用例里的旧 API 全换掉；写 CHANGELOG；SDK 主版本号加一（线格式不变；版本号来自 git tag，打 tag 时全部板子按新树重刷，见第 2 节决定 6）；
   更新 `AGENTS.md` 测试清单与 README 里提到这些类型的地方。
6. **（SocketCAN 开工时）** 抽 `TimeSource`，写第二个实现，`vcan` 测试。
7. **（可选）动作层**，第 8 节。

## 7. 实施须知

- **提交**：agent 不执行 `git add`（libhcs `AGENTS.md` 提交指南）；改动留在工作区，提交信息草稿可写在说明里。
- **代码风格**：libhcs 以 `.clang-format` 为准（改完跑 `.scripts/clang-format-check`，只格式化自己改的文件——
  树里已有三个不合格文件不是本次的：`firmware/c_board/app/src/ports.hpp`、`firmware/hpm_board/app/src/dmtool/dm_persist.cpp`、
  `host/include/libhcs/time/board_clock.hpp`）。HCS 仓库没有 `.clang-format`，**不要**对 HCS 文件跑 `clang-format -i`
  （会按 LLVM 风格整体重写）。`host/` 代码注释用英文（与现有一致），HCS 与固件注释用中文；代码里除注释外只许 ASCII。
- **构建与测试**：libhcs 单独 `cmake --preset linux-debug -S host && cmake --build host/build && ctest --test-dir host/build`；
  HCS `tools/check.sh`（产物在 `~/.cache/hcs-check`），加 `--clang` 检查周期域阻塞调用，加 `--firmware` 确认固件没被碰。
  HCS 把 libhcs 的主机测试编成 `test_libhcs_<名字>`：新增测试要同时加进 `host/tests/CMakeLists.txt` 与
  `hcs_core/CMakeLists.txt` 的 `foreach(_libhcs_test ...)` 列表。
- **HCS 周期域**：`HCS_NONBLOCKING` 函数里不许读时钟、加锁、分配；`SampleTime` 在 SDK 的 IO 线程里算好，HCS 只拷贝。
- **台架**（`hcs_core/test/test_bench_boards.cpp`）：形态按插着的板子互斥选择（`Bench` 两块 5321、`Mc02Bench`
  5321 + mc02、`ThreeBoardBench` 两块 5321 + mc02、`CBoardBench` 5321 + mc02 + c_board，插着 c_board 时前三种跳过）。
  时间戳用例在 `Mc02Bench`/`ThreeBoardBench`，CAN1 跑 FD 1M/5M。2026-10-07 的台架是三块板同挂 CAN1：这条线上
  5321↔mc02 的 FD 帧过不去（5 Mbit 数据段出错，推断是三个终端并联），要跑那些用例得先把 c_board 从 CAN1 上
  拆下来并拔掉它的 USB；第 3 步的台架对比也可以改用经典 1M（c_board 只有经典 CAN）。另有会话可能同时动同一棵树
  和台架：动手前看 `~/.claude/projects/-home-zyi-Desktop-HCS/` 下最新的会话记录时间。

## 8. 动作层（可选，暂不做）："在本机时刻 T 让多块板同时发一帧"

主机一侧第 5 节的 `board_time_at()` 已经够用（映射求逆）。缺的在线上与固件：

- **线格式**：新下行记录 `kCanAt` = 普通 CAN 记录 + 目标板上时刻（板钟刻度的低 32 位）。版本号进 v18，全部板子重刷。
- **固件**：每路 CAN 一个小定时队列（8 帧），按目标时刻装定时器比较中断（hpm 用 GPTMR，mc02 用 TIM5 比较，
  c_board 用 TIM2 比较），到点把帧写进发送邮箱。用 SOF 中断放行不够：mc02、c_board 是全速，SOF 每 1 ms 一次。
- **精度预算**：板上时刻的拟合误差（亚微秒）+ 中断进入延迟（约 1 µs）+ CAN 仲裁（总线忙时一帧时间）。最后一项
  通常最大：同一条总线上"同时发"本来就做不到，要同时生效只能是不同总线、不同板。
- **判据**：CONTROL_TIMING.md 的结论是控制环不需要；测到多关节之间命令生效时刻差（今天由 USB 抖动决定，约
  0.1–1 ms）确实影响控制时再做。

## 9. 考虑过、不采用的

| 方案 | 不采用的理由 |
|---|---|
| `SampleTime` 加进 `core/` 的数据视图 | 视图固件也在用，CAN 中断里就地构造；多 32 字节的字段每帧多清零一次（决定 2） |
| 只给本机时刻、不给板上时刻 | IMU 积分会依赖时间同步、启动头几秒无 dt、退回到达时刻时 dt 不可用（决定 1） |
| 时间做成 `std::optional<HostTime>` | 每个使用者都要写"没有时间怎么办"；退回到达时刻并标来源更简单也更诚实 |
| 现在就建 `ClockSource` 虚接口 + 依赖注入 | 只有一个实现，接口会被它的形状绑死；等第二个后端再抽（4.8） |
| 卡尔曼 / 锁相环代替滑窗最小二乘 | 实测往返拟合 8 µs、控制器计数器微秒级，已经够；滤波器要调参，换不来可测的好处 |
| `std::atomic<std::shared_ptr<const Map>>` 发布映射 | libstdc++ 下不保证无锁、每次发布一次分配；映射是小 POD，现成的 seqlock 更合适 |
| 保留 HCS 每拍取映射、延后换算 | 换算状态留在使用者一侧，SDK 的每个使用者都得重写一遍；一致性的收益在亚微秒以下 |
| 用线程局部"当前记录的时间"代替回调参数 | 隐式耦合，回调里调别的 SDK 函数就可能读错；参数是显式的 |
| 公开 API 用模板参数区分板型的时间类型 | 板型差异只在固件；主机一侧所有板的时间语义相同 |
