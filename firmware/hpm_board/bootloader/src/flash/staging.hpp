#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "firmware/hpm_board/bootloader/src/flash/layout.hpp"
#include "firmware/hpm_board/bootloader/src/flash/metadata.hpp"
#include "firmware/hpm_board/bootloader/src/flash/validation.hpp"
#include "firmware/hpm_board/bootloader/src/flash/writer.hpp"
#include "firmware/hpm_board/bootloader/src/flash/xpi_nor.hpp"
#include "firmware/hpm_board/common/foe_staging.hpp"

namespace libhcs::firmware::flash {

#if defined(BOARD_FOE_STAGING_ADDR)

namespace detail {

inline bool clear_staging_record() {
    return XpiNor::instance().erase_sector(foe::kStagingMetadataStart);
}

} // namespace detail

// 安装运行中的 app 暂存的固件镜像(若存在且完好)。仅当 app 槽已装好暂存镜像时
// 返回 true。
//
// 顺序即设计本身, 下面每一步的位置都有其原因:
//
//  1. 先校验暂存副本。此后所有步骤都会擦除 app 槽, 损坏的下载必须在已装 app
//     仍是可启动镜像时被拒。这正是 validate_image_at() 存在的原因。
//
//  2. 校验失败时清除 staging 记录。否则永久坏的镜像会在每次启动都被重新
//     检查并拒绝 -- 设备看起来无法启动, 实际上 app 是好的。
//
//  3. 成功路径上, 清除必须放在 app metadata 提交之后。擦除到提交之间 app 槽
//     是写了一半的镜像, staging 记录是重建它的唯一凭据; 记录已清后掉电是唯一
//     能让这条路变砖的方式, 故清除必须最后 -- 见 common/foe_staging.hpp。
//
// 步骤 1 之后任何一步失败都保持 staging 记录完好、app metadata 未提交, 下次
// 启动重试。Writer 会跳过内容已一致的扇区, 重试因此廉价, 无需整体重写。
inline bool install_staged_image_if_ready() {
    if (!XpiNor::instance().available())
        return false;

    if (!foe::staging_record_is_ready(kStagingMaxImageSize))
        return false;

    const uint32_t size = foe::staging_record()->image_size;

    // (1) 动任何东西之前先证明候选镜像。
    if (!validate_image_at(foe::kStagingImageStart, size, kStagingMaxImageSize)) {
        // (2) 丢弃; 已装 app 未被触碰, 仍可启动。
        (void)detail::clear_staging_record();
        return false;
    }

    auto& metadata = Metadata::get_instance();
    if (!metadata.begin_flashing())
        return false;

    Writer writer;
    writer.begin_session();

    const auto* source = reinterpret_cast<const std::byte*>(foe::kStagingImageStart);
    for (uint32_t offset = 0U; offset < size; offset += Writer::kTransferBlockSize) {
        const auto chunk =
            static_cast<size_t>(std::min<uint32_t>(Writer::kTransferBlockSize, size - offset));
        if (!writer.write(kAppStartAddress + offset, std::span{source + offset, chunk})) {
            writer.abort_session();
            return false;
        }
    }

    if (!writer.finish_session())
        return false;

    if (!metadata.finish_flashing(size))
        return false;

    // 重验实际落进 app 槽的内容。暂存副本通过不能证明写入正确: 这是唯一覆盖
    // 写入结果本身的检查, 且此时 staging 记录仍可用于重试。
    if (!validate_candidate_image(size))
        return false;

    // (3) 已提交且验证通过 -- 暂存副本再无用处。
    (void)detail::clear_staging_record();
    return true;
}

#else

inline bool install_staged_image_if_ready() { return false; }

#endif

} // namespace libhcs::firmware::flash
