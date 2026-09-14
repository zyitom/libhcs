#pragma once

#include <cstdint>
#include <cstring>

#include "firmware/mc02/bootloader/src/crypto/sha256.hpp"
#include "firmware/mc02/bootloader/src/flash/layout.hpp"
#include "firmware/mc02/bootloader/src/flash/metadata.hpp"

namespace libhcs::firmware::flash {

inline constexpr uint32_t kDtcmramStart = 0x20000000U;
inline constexpr uint32_t kDtcmramEnd   = 0x20020000U; // 不含
inline constexpr uint32_t kAxiSramStart  = 0x24000000U;
inline constexpr uint32_t kAxiSramEnd    = 0x24020000U; // 不含
inline constexpr uint32_t kImageHashMagic = 0x48415348U;                 // "HASH"
inline constexpr uint32_t kImageHashSuffixSize =
    sizeof(uint32_t) + static_cast<uint32_t>(crypto::kSha256DigestSize); // 36

inline bool is_vector_table_valid() {
    const uint32_t initial_msp = *reinterpret_cast<volatile const uint32_t*>(kAppStartAddress);
    const uint32_t reset_handler =
        *reinterpret_cast<volatile const uint32_t*>(kAppStartAddress + 4U);

    // 初始 MSP 允许恰好等于 RAM 区域末尾: Cortex-M 栈向下生长, 复位代码常把 SP
    // 设为最后一个有效 RAM 地址 +1。
    const bool valid_dtcm = (initial_msp >= kDtcmramStart && initial_msp <= kDtcmramEnd);
    const bool valid_axi  = (initial_msp >= kAxiSramStart  && initial_msp <= kAxiSramEnd);
    if (!valid_dtcm && !valid_axi)
        return false;

    if ((initial_msp & 0x7U) != 0U)
        return false;

    if ((reset_handler & 0x1U) == 0U)
        return false;

    const uint32_t reset_handler_addr = reset_handler & ~0x1U;
    return reset_handler_addr >= kAppStartAddress && reset_handler_addr < kAppEndAddress;
}

inline void compute_image_sha256(uint32_t address, uint32_t size, uint8_t* hash) {
    const auto* data = reinterpret_cast<const uint8_t*>(address);
    crypto::Sha256Ctx ctx;
    crypto::sha256_init(&ctx);
    crypto::sha256_update(&ctx, data, size);
    crypto::sha256_final(&ctx, hash);
}

// SHA-256 后缀是镜像完整性的全部依据。注意它只证明镜像完好, 不证明来源可信
// -- 摘要未签名, 且随镜像本身一同存储。
inline bool validate_image_hash(uint32_t address, uint32_t size) {
    if (size <= kImageHashSuffixSize)
        return false;

    const auto* suffix_ptr =
        reinterpret_cast<const uint8_t*>(address + size - kImageHashSuffixSize);

    uint32_t suffix_magic;
    std::memcpy(&suffix_magic, suffix_ptr, sizeof(suffix_magic));

    if (suffix_magic != kImageHashMagic)
        return false;

    const uint32_t firmware_size = size - kImageHashSuffixSize;
    const uint8_t* expected_sha256 = suffix_ptr + sizeof(uint32_t);

    uint8_t computed_sha256[crypto::kSha256DigestSize];
    compute_image_sha256(address, firmware_size, computed_sha256);

    return std::memcmp(computed_sha256, expected_sha256, crypto::kSha256DigestSize) == 0;
}

inline bool validate_candidate_image(uint32_t size) {
    if (size == 0U || size > kAppMaxImageSize)
        return false;

    if (!is_vector_table_valid())
        return false;

    return validate_image_hash(kAppStartAddress, size);
}

inline bool validate_app_image() {
    auto& meta = Metadata::get_instance();

    if (!meta.is_ready())
        return false;

    return validate_candidate_image(meta.image_size());
}

} // namespace libhcs::firmware::flash
