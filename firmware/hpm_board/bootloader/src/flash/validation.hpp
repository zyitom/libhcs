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

// The SHA-256 suffix is the whole integrity story for the image. A CRC32 pass
// used to run alongside it, which cost a second full-image scan on every boot
// while adding nothing: a cryptographic digest already covers everything a CRC
// detects. Note this proves the image is intact, not that it is authentic --
// the digest is unsigned and travels with the image.
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

// Same checks the app slot gets, applied at an arbitrary address so a staged
// candidate can be proven intact BEFORE the app slot is erased for it. Keeping
// one implementation matters more than the parameter: a staged image that the
// bootloader would later reject must be rejected here, while the running image
// is still the one that boots.
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
