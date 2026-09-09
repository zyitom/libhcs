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

// Install a firmware image the running app staged for us, if one is there and
// intact. Returns true only when the app slot now holds the staged image.
//
// Ordering is the whole design; every step below is placed for one reason:
//
//  1. Validate the STAGED copy first. Everything after this erases the app slot,
//     so a corrupt download must be rejected while the installed app is still
//     the one that boots. This is why validate_image_at() exists.
//
//  2. Clear the staging record on a validation failure. Otherwise a permanently
//     bad image would be re-examined, and re-rejected, on every single boot --
//     an unbootable-looking device whose app is in fact fine.
//
//  3. Clear it on SUCCESS only after the app metadata has committed. Between the
//     erase and that commit the app slot holds a half-written image, and the
//     staging record is the only thing that can rebuild it. Losing power there
//     with the record already cleared is the one way to brick this path, so the
//     clear must come last -- see common/foe_staging.hpp.
//
// A failure anywhere after step 1 leaves the staging record intact and the app
// metadata un-committed, so the next boot retries. Writer skips sectors whose
// content already matches, which makes that retry cheap rather than a full
// rewrite.
inline bool install_staged_image_if_ready() {
    if (!XpiNor::instance().available())
        return false;

    if (!foe::staging_record_is_ready(kStagingMaxImageSize))
        return false;

    const uint32_t size = foe::staging_record()->image_size;

    // (1) Prove the candidate before touching anything.
    if (!validate_image_at(foe::kStagingImageStart, size, kStagingMaxImageSize)) {
        // (2) Drop it; the installed app is untouched and still boots.
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

    // Re-validate what actually landed in the app slot. The staged copy passing
    // does not prove the write did: this is the only check that covers the copy
    // itself, and it runs while the staging record is still available to retry.
    if (!validate_candidate_image(size))
        return false;

    // (3) Committed and proven -- the staged copy has no further job.
    (void)detail::clear_staging_record();
    return true;
}

#else

inline bool install_staged_image_if_ready() { return false; }

#endif

} // namespace libhcs::firmware::flash
