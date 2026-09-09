#!/usr/bin/env python3
"""Merge Intel HEX fragments into one flashable image (.hex and flat .bin).

CH32H417 is dual-core with each core in its own flash partition (V3F @ 0x0,
V5F @ 0x10000), plus a third fragment carrying the bootloader's app-image
metadata record (cmake/gen_metadata_hex.py). There is no "embed one core into
the other" model on this part, so the only single-file form is the fragments
concatenated by address.

This is NOT a text concatenation. objcopy emits Extended *Segment* Address
records (type 0x02) for images below 1 MiB, while the metadata fragment uses
Extended *Linear* Address records (type 0x04). The two are mutually exclusive
addressing modes -- a reader that honours only one silently drops every fragment
written in the other, and mixing them in a single file made objcopy add the
stale segment base to the linear base. The observed damage was the V5F image
landing on top of the V3F bootloader at 0x0 and the metadata record going to
0x90000 instead of 0x70000, i.e. a merged image that flashed cleanly and then
did nothing at all.

So: parse every fragment into absolute addresses, check that they do not
overlap, and re-emit ONE file that uses type 0x04 throughout.

Outputs:
  .hex - absolute addresses, what openocd's `program` takes
  .bin - flat image based at 0x00000000, gaps filled with 0xFF; what MounRiver
         Studio and WCH-LinkUtility take
"""

import argparse
import struct

RECORD_DATA = 0x00
RECORD_EOF = 0x01
RECORD_EXTENDED_SEGMENT_ADDRESS = 0x02
RECORD_START_SEGMENT_ADDRESS = 0x03
RECORD_EXTENDED_LINEAR_ADDRESS = 0x04
RECORD_START_LINEAR_ADDRESS = 0x05

BIN_BASE_ADDRESS = 0x00000000
GAP_FILL = 0xFF
RECORD_PAYLOAD_SIZE = 16


def parse_intel_hex(path: str) -> dict[int, int]:
    """Return {absolute address: byte} for one fragment."""
    memory: dict[int, int] = {}
    base = 0
    segment_mode = False

    with open(path, "r", encoding="ascii") as handle:
        for number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise SystemExit(f"{path}:{number}: not an Intel HEX record")

            body = bytes.fromhex(line[1:])
            if len(body) < 5 or len(body) != body[0] + 5:
                raise SystemExit(f"{path}:{number}: bad record length")
            if (sum(body) & 0xFF) != 0:
                raise SystemExit(f"{path}:{number}: bad checksum")

            offset = (body[1] << 8) | body[2]
            record_type = body[3]
            payload = body[4:-1]

            if record_type == RECORD_DATA:
                for index, value in enumerate(payload):
                    # Type 0x02 addressing wraps inside its 64 KiB segment,
                    # type 0x04 does not. Only the former needs the mask, and
                    # applying it to the latter would be wrong, so track which
                    # mode set the base rather than masking unconditionally.
                    if segment_mode:
                        address = base + ((offset + index) & 0xFFFF)
                    else:
                        address = base + offset + index
                    if address in memory:
                        raise SystemExit(
                            f"{path}:{number}: address {address:#010x} written twice"
                        )
                    memory[address] = value
            elif record_type == RECORD_EXTENDED_SEGMENT_ADDRESS:
                base = struct.unpack(">H", payload)[0] << 4
                segment_mode = True
            elif record_type == RECORD_EXTENDED_LINEAR_ADDRESS:
                base = struct.unpack(">H", payload)[0] << 16
                segment_mode = False
            elif record_type in (
                RECORD_EOF,
                RECORD_START_SEGMENT_ADDRESS,
                RECORD_START_LINEAR_ADDRESS,
            ):
                # Entry-point records are per-image and meaningless once several
                # images share a file; the CH32H417 cores start from their reset
                # vectors, not from anything a hex file says.
                continue
            else:
                raise SystemExit(f"{path}:{number}: unsupported record type {record_type:#04x}")

    return memory


def hex_record(record_type: int, offset: int, payload: bytes) -> str:
    body = bytes([len(payload), (offset >> 8) & 0xFF, offset & 0xFF, record_type]) + payload
    checksum = (-sum(body)) & 0xFF
    return ":" + body.hex().upper() + f"{checksum:02X}"


def to_intel_hex(memory: dict[int, int]) -> str:
    lines = []
    upper = None

    address = min(memory)
    end = max(memory) + 1
    while address < end:
        # Never let a record straddle a 64 KiB boundary: the offset field is 16
        # bits and the extended base only advances between records.
        limit = min(address + RECORD_PAYLOAD_SIZE, (address & ~0xFFFF) + 0x10000, end)
        chunk = bytearray()
        while address + len(chunk) < limit and (address + len(chunk)) in memory:
            chunk.append(memory[address + len(chunk)])

        if not chunk:
            address += 1
            continue

        if (address >> 16) != upper:
            upper = address >> 16
            lines.append(hex_record(RECORD_EXTENDED_LINEAR_ADDRESS, 0, struct.pack(">H", upper)))
        lines.append(hex_record(RECORD_DATA, address & 0xFFFF, bytes(chunk)))
        address += len(chunk)

    lines.append(hex_record(RECORD_EOF, 0, b""))
    return "\n".join(lines) + "\n"


def to_flat_binary(memory: dict[int, int]) -> bytes:
    end = max(memory) + 1
    if min(memory) < BIN_BASE_ADDRESS:
        raise SystemExit(f"fragment below the .bin base address {BIN_BASE_ADDRESS:#010x}")

    image = bytearray([GAP_FILL]) * (end - BIN_BASE_ADDRESS)
    for address, value in memory.items():
        image[address - BIN_BASE_ADDRESS] = value
    return bytes(image)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Intel HEX fragments to merge")
    parser.add_argument("--output-hex", required=True, help="Merged Intel HEX to write")
    parser.add_argument("--output-bin", required=True, help="Flat binary to write")
    args = parser.parse_args()

    merged: dict[int, int] = {}
    for path in args.inputs:
        fragment = parse_intel_hex(path)
        if not fragment:
            raise SystemExit(f"{path}: no data records")

        overlap = merged.keys() & fragment.keys()
        if overlap:
            # This is the check that the old text-concatenation merge lacked.
            raise SystemExit(
                f"{path}: overlaps an earlier fragment at "
                f"{min(overlap):#010x}..{max(overlap):#010x}"
            )
        merged.update(fragment)

    with open(args.output_hex, "w", encoding="ascii") as handle:
        handle.write(to_intel_hex(merged))
    with open(args.output_bin, "wb") as handle:
        handle.write(to_flat_binary(merged))


if __name__ == "__main__":
    main()
