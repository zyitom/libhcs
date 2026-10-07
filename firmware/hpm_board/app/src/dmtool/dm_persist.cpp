#include "firmware/hpm_board/app/src/dmtool/dm_persist.hpp"

#include <cstring>

#include <hpm_csr_regs.h>
#include <hpm_l1c_drv.h>
#include <hpm_romapi.h>
#include <hpm_romapi_xpi_def.h>
#include <hpm_romapi_xpi_nor_def.h>
#include <hpm_soc.h>
#include <hpm_soc_feature.h>

#include "board.h"

namespace libhcs::firmware::dmtool::persist {

namespace {

// 参数扇区: 1MB NOR 的最后一个 4KB 扇区(见 dm_persist.hpp 头注)。
constexpr uintptr_t kParamSectorOffset = 0xFF000U;
constexpr uintptr_t kParamAddress = BOARD_FLASH_BASE_ADDRESS + kParamSectorOffset;
constexpr uint32_t kMagic = 0x46434D44U; // "DMCF" 小端
// 版本 2: 版本 1 的记录总长没给 CRC 留位置, CRC 压在通道 2 的数据段 seg2 / sjw 上,
// 通道 2 存下的时序读回来非法(mcan_init 拒绝, 端口退回编译期默认时序)。版本 1 的
// 记录一律不认, 两路回到 DMTool 的 1M/5M 预设。
constexpr uint8_t kVersion = 2U;

// ROM XPI NOR API 的调用形态与 bootloader/src/flash/xpi_nor.hpp 一致(同一套
// 已在板上验证的 auto-config + 擦除/编程 + cache 维护序列), 只是面向参数扇区。
class Nor {
public:
    static Nor& instance() {
        static Nor nor;
        return nor;
    }

    bool available() const { return available_; }
    uint32_t sector_size() const { return sector_size_; }

    bool erase_sector(uintptr_t address) {
        if (!available_ || sector_size_ == 0U || (address % sector_size_) != 0U)
            return false;
        const uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
        const hpm_stat_t status = rom_xpi_nor_erase_sector(
            BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_config_, to_offset(address));
        restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
        if (status != status_success)
            return false;
        invalidate(address, sector_size_);
        return true;
    }

    bool program(uintptr_t address, const uint8_t* data, uint32_t size) {
        if (!available_ || (address & 3U) != 0U || (size & 3U) != 0U
            || (reinterpret_cast<uintptr_t>(data) & 3U) != 0U)
            return false;
        writeback(reinterpret_cast<uintptr_t>(data), size);
        const uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
        const hpm_stat_t status = rom_xpi_nor_program(
            BOARD_APP_XPI_NOR_XPI_BASE, xpi_xfer_channel_auto, &nor_config_,
            reinterpret_cast<const uint32_t*>(data), to_offset(address), size);
        restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
        if (status != status_success)
            return false;
        invalidate(address, size);
        return true;
    }

private:
    // auto_config 会重新探测并配置正在 XIP 执行的这颗 flash, 与擦写一样关中断: 期间
    // 放进来一个从 flash 取指的中断, 取到的是半配置状态下的数据, 板子卡死(2026-10-03
    // 实测: DMTool 保存参数后 EP0 都不再应答)。bootloader 不开中断, 所以从没出过事。
    Nor() {
        xpi_nor_config_option_t option{};
        option.header.U = BOARD_APP_XPI_NOR_CFG_OPT_HDR;
        option.option0.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT0;
        option.option1.U = BOARD_APP_XPI_NOR_CFG_OPT_OPT1;
        const uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
        available_ = rom_xpi_nor_auto_config(BOARD_APP_XPI_NOR_XPI_BASE, &nor_config_, &option)
                      == status_success
                  && rom_xpi_nor_get_property(
                         BOARD_APP_XPI_NOR_XPI_BASE, &nor_config_, xpi_nor_property_sector_size,
                         &sector_size_)
                         == status_success;
        restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    }

    static uint32_t to_offset(uintptr_t address) {
        return static_cast<uint32_t>(address - BOARD_FLASH_BASE_ADDRESS);
    }
    static void invalidate(uintptr_t address, size_t size) {
        l1c_dc_invalidate(
            HPM_L1C_CACHELINE_ALIGN_DOWN(address),
            HPM_L1C_CACHELINE_ALIGN_UP(address + size) - HPM_L1C_CACHELINE_ALIGN_DOWN(address));
    }
    static void writeback(uintptr_t address, size_t size) {
        l1c_dc_writeback(
            HPM_L1C_CACHELINE_ALIGN_DOWN(address),
            HPM_L1C_CACHELINE_ALIGN_UP(address + size) - HPM_L1C_CACHELINE_ALIGN_DOWN(address));
    }

    xpi_nor_config_t nor_config_{};
    uint32_t sector_size_ = 0U;
    bool available_ = false;
};

uint16_t crc16_ibm(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 1U) != 0U ? static_cast<uint16_t>(crc >> 1U) ^ 0xA001U
                                   : static_cast<uint16_t>(crc >> 1U);
    }
    return crc;
}

// 序列化布局: magic u32 | version u8 | count u8 | pad u16 |
//             每通道 12B(fd+pad+nominal 4B+data 4B) × 2 | CRC16 u16 | pad u16
// 末尾补齐到 4 字节: ROM 的 NOR 编程只收 4 字节对齐的长度。
constexpr size_t kCrcOffset = 8U + (kChannelCount * 12U);
constexpr size_t kPayloadSize = kCrcOffset + 4U;

void serialize(const Config& config, uint8_t* out) {
    std::memset(out, 0, kPayloadSize);
    std::memcpy(out, &kMagic, sizeof(kMagic));
    out[4] = kVersion;
    out[5] = static_cast<uint8_t>(kChannelCount);
    for (size_t ch = 0; ch < kChannelCount; ch++) {
        uint8_t* p = out + 8 + (ch * 12);
        p[0] = config[ch].fd ? 1U : 0U;
        p[4] = config[ch].nominal_prescaler;
        p[5] = config[ch].nominal_seg1;
        p[6] = config[ch].nominal_seg2;
        p[7] = config[ch].nominal_sjw;
        p[8] = config[ch].data_prescaler;
        p[9] = config[ch].data_seg1;
        p[10] = config[ch].data_seg2;
        p[11] = config[ch].data_sjw;
    }
    const uint16_t crc = crc16_ibm(out, kCrcOffset);
    out[kCrcOffset] = static_cast<uint8_t>(crc & 0xFFU);
    out[kCrcOffset + 1U] = static_cast<uint8_t>(crc >> 8U);
}

bool deserialize(const uint8_t* in, Config& config) {
    uint32_t magic{};
    std::memcpy(&magic, in, sizeof(magic));
    if (magic != kMagic || in[4] != kVersion || in[5] != kChannelCount)
        return false;
    const uint16_t crc =
        static_cast<uint16_t>(in[kCrcOffset]) | static_cast<uint16_t>(in[kCrcOffset + 1U] << 8U);
    if (crc != crc16_ibm(in, kCrcOffset))
        return false;
    for (size_t ch = 0; ch < kChannelCount; ch++) {
        const uint8_t* p = in + 8 + (ch * 12);
        config[ch] = {
            .fd = p[0] != 0,
            .nominal_prescaler = p[4],
            .nominal_seg1 = p[5],
            .nominal_seg2 = p[6],
            .nominal_sjw = p[7],
            .data_prescaler = p[8],
            .data_seg1 = p[9],
            .data_seg2 = p[10],
            .data_sjw = p[11]};
    }
    return true;
}

} // namespace

bool load(Config& out) {
    out.fill({});
    Config candidate{};
    if (!deserialize(reinterpret_cast<const uint8_t*>(kParamAddress), candidate))
        return false;
    out = candidate;
    return true;
}

bool store(const Config& config) {
    auto& nor = Nor::instance();
    if (!nor.available())
        return false;
    alignas(4) std::array<uint8_t, kPayloadSize> image{};
    serialize(config, image.data());
    if (!nor.erase_sector(kParamAddress))
        return false;
    return nor.program(kParamAddress, image.data(), kPayloadSize);
}

} // namespace libhcs::firmware::dmtool::persist
