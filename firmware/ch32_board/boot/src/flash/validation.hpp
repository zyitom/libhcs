#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "firmware/ch32_board/boot/src/crypto/sha256.hpp"
#include "firmware/ch32_board/boot/src/flash/layout.hpp"
#include "firmware/ch32_board/boot/src/flash/metadata.hpp"

namespace libhcs::firmware::flash {

// Hash the application slot and compare against the committed record. The image
// is read through the flash alias, which is directly addressable, so no staging
// buffer is needed.
inline void
    hash_region(uintptr_t address, size_t size, uint8_t (&digest)[crypto::kSha256DigestSize]) {
    crypto::Sha256Ctx ctx;
    crypto::sha256_init(&ctx);
    crypto::sha256_update(&ctx, reinterpret_cast<const uint8_t*>(address), size);
    crypto::sha256_final(&ctx, digest);
}

// The single gate the bootloader uses before handing control to the app.
inline bool app_image_is_valid() {
    const Metadata& record = MetadataStore::stored();
    if (!record.is_valid_record())
        return false;

    uint8_t digest[crypto::kSha256DigestSize];
    hash_region(kAppStartAddress, record.image_size, digest);
    return std::memcmp(digest, record.sha256, sizeof(digest)) == 0;
}

} // namespace libhcs::firmware::flash
