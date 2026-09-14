#pragma once

#include <cstdint>
#include <cstring>

#include <board.h>

#include "firmware/hpm_board/bootloader/src/crypto/sha256.hpp"
#include "firmware/hpm_board/bootloader/src/flash/layout.hpp"
#include "firmware/hpm_board/bootloader/src/flash/metadata.hpp"

namespace libhcs::firmware::flash {

inline constexpr uint32_t kImageHashMagic = 0x48415348U; // "HASH"
inline constexpr uint32_t kImageHashSuffixSize =
    sizeof(uint32_t) + static_cast<uint32_t>(crypto::kSha256DigestSize);

inline bool has_valid_signature_at(uintptr_t address) {
    return *reinterpret_cast<volatile const uint32_t*>(address) == BOARD_UF2_SIGNATURE;
}

inline bool has_valid_app_signature() { return has_valid_signature_at(kAppStartAddress); }

inline void compute_image_sha256(uintptr_t address, uint32_t size, uint8_t* hash) {
    const auto* data = reinterpret_cast<const uint8_t*>(address);
    crypto::Sha256Ctx ctx;
    crypto::sha256_init(&ctx);
    crypto::sha256_update(&ctx, data, size);
    crypto::sha256_final(&ctx, hash);
}

// 镜像完整性只由 SHA-256 后缀保证: 密码学摘要已覆盖 CRC 能检出的一切, 额外的
// CRC32 扫描纯属浪费。注意这证明的是镜像完好, 而非来源可信 -- 摘要无签名,
// 随镜像本身一同传输。
inline bool validate_image_hash(uintptr_t address, uint32_t size) {
    if (size <= kImageHashSuffixSize)
        return false;

    const auto* suffix_ptr =
        reinterpret_cast<const uint8_t*>(address + size - kImageHashSuffixSize);

    uint32_t suffix_magic = 0U;
    std::memcpy(&suffix_magic, suffix_ptr, sizeof(suffix_magic));
    if (suffix_magic != kImageHashMagic)
        return false;

    const uint32_t firmware_size = size - kImageHashSuffixSize;
    const uint8_t* expected_sha256 = suffix_ptr + sizeof(uint32_t);

    uint8_t computed_sha256[crypto::kSha256DigestSize];
    compute_image_sha256(address, firmware_size, computed_sha256);

    return std::memcmp(computed_sha256, expected_sha256, crypto::kSha256DigestSize) == 0;
}

// 与 app 槽相同的检查, 作用于任意地址, 使暂存的候选镜像能在 app 槽为其擦除
// 之前先证明完好。单一实现比参数更重要: bootloader 稍后会拒绝的暂存镜像必须
// 在这里被拒, 此时运行的镜像仍是可启动的那个。
inline bool validate_image_at(uintptr_t address, uint32_t size, size_t max_size) {
    if (size == 0U || size > max_size)
        return false;

    if (!has_valid_signature_at(address))
        return false;

    return validate_image_hash(address, size);
}

inline bool validate_candidate_image(uint32_t size) {
    return validate_image_at(kAppStartAddress, size, kAppMaxImageSize);
}

inline bool validate_app_image() {
    auto& meta = Metadata::get_instance();
    if (!meta.is_ready())
        return false;

    return validate_candidate_image(meta.image_size());
}

} // namespace libhcs::firmware::flash
