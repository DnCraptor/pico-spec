#!/usr/bin/env python3
"""Convert pico-spec framebuffer dump to PNG.

Usage:
    python3 fb2png.py fb.bin pal.bin WIDTH HEIGHT out.png

Optional check: if a sibling rowptrs.bin exists in /tmp, we report whether the
framebuffer rows are contiguous (i.e. row[k] == row[0] + k*width). If they are
NOT contiguous, the dump-by-(fb0..fb0+w*h) approach will produce garbage; the
script warns so you can switch to per-row dumping.

The framebuffer is 8-bit palette indices, width*height bytes.
HDMI driver reads bytes with `input_buffer[(x++) ^ 2]`, so adjacent
4-byte groups are swapped pairwise. We undo that to get visual order.
The palette is 256 x uint32_t little-endian, format 0x00RRGGBB.
"""
import os
import struct
import sys
from PIL import Image


def check_contiguous(width: int, height: int) -> None:
    path = '/tmp/picospec_rowptrs.bin'
    if not os.path.exists(path):
        return
    data = open(path, 'rb').read()
    if len(data) < height * 4:
        return
    ptrs = [struct.unpack_from('<I', data, i * 4)[0] for i in range(height)]
    base = ptrs[0]
    contig = all(ptrs[k] == base + k * width for k in range(height))
    if contig:
        print(f"rows: contiguous (base=0x{base:08x})")
    else:
        diffs = [ptrs[k] - base - k * width for k in range(min(8, height))]
        print(f"rows: NON-contiguous! base=0x{base:08x}")
        print(f"  first 8 row offsets vs expected: {diffs}")
        print("  WARNING: per-row dump needed; current image will likely be wrong.")


# Standard ZX Spectrum palette for indices 0..15 (BRIGHT 0 then BRIGHT 1).
ZX_PALETTE = [
    (0x00, 0x00, 0x00), (0x00, 0x00, 0xCD), (0xCD, 0x00, 0x00), (0xCD, 0x00, 0xCD),
    (0x00, 0xCD, 0x00), (0x00, 0xCD, 0xCD), (0xCD, 0xCD, 0x00), (0xCD, 0xCD, 0xCD),
    (0x00, 0x00, 0x00), (0x00, 0x00, 0xFF), (0xFF, 0x00, 0x00), (0xFF, 0x00, 0xFF),
    (0x00, 0xFF, 0x00), (0x00, 0xFF, 0xFF), (0xFF, 0xFF, 0x00), (0xFF, 0xFF, 0xFF),
]


def main():
    if len(sys.argv) < 6:
        print(__doc__)
        sys.exit(1)

    fb_path, pal_path, w, h, out_path = sys.argv[1:6]
    w, h = int(w), int(h)
    use_zx = '--zx' in sys.argv[6:]
    no_xor = '--no-xor' in sys.argv[6:]

    check_contiguous(w, h)

    fb = open(fb_path, 'rb').read()
    if len(fb) < w * h:
        print(f"fb too small: {len(fb)} < {w*h}")
        sys.exit(1)

    pal_raw = open(pal_path, 'rb').read()
    pal = []
    for i in range(256):
        v = struct.unpack_from('<I', pal_raw, i * 4)[0]
        r = (v >> 16) & 0xFF
        g = (v >> 8) & 0xFF
        b = v & 0xFF
        pal.append((r, g, b))

    # Heuristic: detect when the dumped palette doesn't look like a real
    # color table for ZX — pure gradient (picodvi-shim default), or random
    # framebuffer bytes (HDMI build optimizes palette away). In both cases
    # substitute the standard ZX palette for indices 0..15.
    looks_default = all(pal[i] == (i, i, i) for i in range(3, 16))
    # ZX-like: indices 1..6 should have R/G/B in {0, 0xCD}. If almost none
    # of indices 1..15 match that pattern, palette is bogus.
    zx_match = sum(
        1 for i in range(1, 16)
        if all(c in (0x00, 0xCD, 0xFF) for c in pal[i]) and pal[i] != (0, 0, 0)
    )
    looks_bogus = zx_match < 4
    if use_zx or looks_default or looks_bogus:
        for i in range(16):
            pal[i] = ZX_PALETTE[i]
        print("palette: using ZX standard for 0..15 (dumped palette looked default)")

    # Auto-pick XOR variant if not forced. Try both, score by local smoothness
    # (lower neighbor-difference = correct layout). The HDMI/VGA serializer
    # uses (x ^ 2) byte-swap, but picodvi-shim does not.
    auto_xor = '--no-xor' not in sys.argv[6:] and '--xor' not in sys.argv[6:]
    forced_no_xor = no_xor

    def score_layout(use_xor: bool) -> int:
        # Sum of |row[x+1]-row[x]| over the central region (where content lives).
        # Lower = smoother = likely correct layout. Garbled byte order produces
        # high variance because adjacent original pixels end up far apart.
        total = 0
        y0, y1 = h // 4, (h * 3) // 4  # skip empty top/bottom border
        for y in range(y0, y1):
            row = fb[y * w:(y + 1) * w]
            prev = None
            for x in range(w):
                idx = row[x ^ 2] if use_xor and (x ^ 2) < w else row[x]
                if prev is not None:
                    total += abs(idx - prev)
                prev = idx
        return total

    if auto_xor:
        s_xor = score_layout(True)
        s_noxor = score_layout(False)
        use_xor = s_xor <= s_noxor
        print(f"layout: xor-score={s_xor} noxor-score={s_noxor} → {'XOR' if use_xor else 'NO-XOR'}")
    else:
        use_xor = not forced_no_xor

    img = Image.new('RGB', (w, h))
    px = img.load()
    for y in range(h):
        row = fb[y * w:(y + 1) * w]
        for x in range(w):
            if use_xor and (x ^ 2) < w:
                idx = row[x ^ 2]
            else:
                idx = row[x]
            px[x, y] = pal[idx]

    img.save(out_path)
    print(f"saved {out_path} ({w}x{h})")


if __name__ == '__main__':
    main()
