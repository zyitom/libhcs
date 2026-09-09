#!/usr/bin/env python3
"""Emit the bootloader's app-image metadata record as an Intel HEX fragment.

The V3F bootloader refuses to wake V5F unless a committed record at
kMetadataStartAddress describes the image in the app slot -- see
boot/src/flash/validation.hpp (app_image_is_valid) and boot/src/flash/metadata.hpp.
The DFU path writes that record itself after a download, but flashing the merged
hex with openocd / WCH-LinkUtility bypasses DFU entirely, so without this the
board cold-boots straight into the bootloader's DFU park loop every time and the
app never runs.

The record must match boot/src/flash/metadata.hpp exactly:

    struct Metadata {
        uint32_t magic;       // kMagicValid, "RGM1"
        uint32_t image_size;  // bytes hashed, starting at kAppStartAddress
        uint8_t  sha256[32];
    };

and the digest is taken over exactly image_size bytes read from the flash alias
at kAppStartAddress, which is what the app .bin contains verbatim.
"""

import argparse
import hashlib
import struct

MAGIC_VALID = 0x314D4752  # "RGM1", see Metadata::kMagicValid
DIGEST_SIZE = 32
RECORD_FORMAT = "<II32s"


def hex_record(record_type: int, offset: int, payload: bytes) -> str:
    body = bytes([len(payload), (offset >> 8) & 0xFF, offset & 0xFF, record_type]) + payload
    checksum = (-sum(body)) & 0xFF
    return ":" + body.hex().upper() + f"{checksum:02X}"


def to_intel_hex(address: int, payload: bytes) -> str:
    lines = []
    upper = address >> 16
    lines.append(hex_record(0x04, 0, struct.pack(">H", upper)))
    for chunk_start in range(0, len(payload), 16):
        chunk = payload[chunk_start : chunk_start + 16]
        offset = (address & 0xFFFF) + chunk_start
        # A record must not straddle a 64 KiB boundary; the record is 40 bytes
        # and sits at the start of its own erase block, so this cannot happen.
        assert offset + len(chunk) <= 0x10000, "metadata record crosses a 64 KiB boundary"
        lines.append(hex_record(0x00, offset, chunk))
    lines.append(hex_record(0x01, 0, b""))
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("app_bin", help="Flat app image, as programmed at the app start address")
    parser.add_argument(
        "--address", required=True, help="Metadata record address (kMetadataStartAddress)"
    )
    parser.add_argument(
        "--max-image-size",
        required=True,
        help="kAppMaxImageSize; the bootloader rejects records above it",
    )
    parser.add_argument("-o", "--output", required=True, help="Intel HEX fragment to write")
    args = parser.parse_args()

    address = int(args.address, 0)
    max_image_size = int(args.max_image_size, 0)

    with open(args.app_bin, "rb") as handle:
        image = handle.read()

    if not image:
        raise SystemExit(f"{args.app_bin}: empty app image")
    if len(image) > max_image_size:
        raise SystemExit(
            f"{args.app_bin}: image is {len(image)} bytes, "
            f"exceeds kAppMaxImageSize ({max_image_size})"
        )

    digest = hashlib.sha256(image).digest()
    record = struct.pack(RECORD_FORMAT, MAGIC_VALID, len(image), digest)
    assert len(record) == 8 + DIGEST_SIZE

    with open(args.output, "w", encoding="ascii") as handle:
        handle.write(to_intel_hex(address, record))


if __name__ == "__main__":
    main()
