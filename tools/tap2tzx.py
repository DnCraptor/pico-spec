#!/usr/bin/env python3
"""Convert a TAP file to TZX format (type 0x10 Standard Speed Data blocks)."""

import sys
import struct


def tap2tzx(tap_path: str, tzx_path: str, pause_ms: int = 1000) -> int:
    """Convert tap_path -> tzx_path. Returns number of blocks converted."""
    with open(tap_path, "rb") as f:
        tap_data = f.read()

    blocks = []
    pos = 0
    while pos < len(tap_data):
        if pos + 2 > len(tap_data):
            break
        length = struct.unpack_from("<H", tap_data, pos)[0]
        pos += 2
        if pos + length > len(tap_data):
            print(f"Warning: truncated block at offset {pos - 2}", file=sys.stderr)
            break
        blocks.append(tap_data[pos:pos + length])
        pos += length

    with open(tzx_path, "wb") as f:
        # TZX header: signature + EOF marker + version 1.20
        f.write(b"ZXTape!\x1a\x01\x14")

        for block in blocks:
            # Block type 0x10: Standard Speed Data
            # pause_ms (2 bytes LE) + data_length (2 bytes LE) + data
            f.write(bytes([0x10]))
            f.write(struct.pack("<H", pause_ms))
            f.write(struct.pack("<H", len(block)))
            f.write(block)

    return len(blocks)


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} input.tap output.tzx [pause_ms]")
        print("  pause_ms: pause after each block in milliseconds (default 1000)")
        sys.exit(1)

    tap_path = sys.argv[1]
    tzx_path = sys.argv[2]
    pause_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 1000

    n = tap2tzx(tap_path, tzx_path, pause_ms)
    print(f"Converted {n} block(s): {tap_path} -> {tzx_path}")


if __name__ == "__main__":
    main()
