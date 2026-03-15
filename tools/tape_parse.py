#!/usr/bin/env python3
"""
TZX and PZX tape format parsers.

Usage as library:
    from tools.tape_parse import parse_tzx, parse_pzx, parse_pzx_puls, parse_pzx_data, parse_tzx_gdb

Usage as CLI:
    python3 tools/tape_parse.py file.tzx   # dump TZX structure
    python3 tools/tape_parse.py file.pzx   # dump PZX structure
"""

import struct
import sys
from pathlib import Path


# ============================================================
# TZX parsing
# ============================================================

TZX_BLOCK_NAMES = {
    0x10: "Standard Data",
    0x11: "Turbo Data",
    0x12: "Pure Tone",
    0x13: "Pulse Sequence",
    0x14: "Pure Data",
    0x15: "Direct Recording",
    0x19: "Generalized Data (GDB)",
    0x20: "Silence/Pause",
    0x21: "Group Start",
    0x22: "Group End",
    0x23: "Jump",
    0x24: "Loop Start",
    0x25: "Loop End",
    0x2A: "Stop 48K",
    0x30: "Text Description",
    0x31: "Message",
    0x32: "Archive Info",
    0x33: "Hardware Type",
    0x35: "Custom Info",
    0x5A: "Glue",
}


def parse_tzx(path):
    """Parse TZX file and return list of (block_id, offset, raw_data_after_id).

    block_id: int, TZX block type (0x10, 0x11, 0x19, etc.)
    offset: int, byte offset in file of the block ID byte
    raw_data_after_id: bytes, all block data after the ID byte
    """
    data = Path(path).read_bytes()
    assert data[:7] == b'ZXTape!', "Not a TZX file"
    pos = 10
    blocks = []
    while pos < len(data):
        block_id = data[pos]; pos += 1
        if block_id == 0x10:
            length = struct.unpack_from('<H', data, pos + 2)[0]
            blocks.append((block_id, pos - 1, data[pos:pos + 4 + length]))
            pos += 4 + length
        elif block_id == 0x11:
            length = struct.unpack_from('<I', data, pos + 15)[0] & 0xFFFFFF
            blocks.append((block_id, pos - 1, data[pos:pos + 18 + length]))
            pos += 18 + length
        elif block_id == 0x12:
            blocks.append((block_id, pos - 1, data[pos:pos + 4])); pos += 4
        elif block_id == 0x13:
            n = data[pos]
            blocks.append((block_id, pos - 1, data[pos:pos + 1 + n * 2]))
            pos += 1 + n * 2
        elif block_id == 0x14:
            length = struct.unpack_from('<I', data, pos + 7)[0] & 0xFFFFFF
            blocks.append((block_id, pos - 1, data[pos:pos + 10 + length]))
            pos += 10 + length
        elif block_id == 0x15:
            length = struct.unpack_from('<I', data, pos + 5)[0]
            blocks.append((block_id, pos - 1, data[pos:pos + 9 + length]))
            pos += 9 + length
        elif block_id == 0x19:
            length = struct.unpack_from('<I', data, pos)[0]
            blocks.append((block_id, pos - 1, data[pos:pos + 4 + length]))
            pos += 4 + length
        elif block_id == 0x20:
            blocks.append((block_id, pos - 1, data[pos:pos + 2])); pos += 2
        elif block_id == 0x21:
            n = data[pos]
            blocks.append((block_id, pos - 1, data[pos:pos + 1 + n])); pos += 1 + n
        elif block_id == 0x22:
            blocks.append((block_id, pos - 1, b''))
        elif block_id == 0x23:
            blocks.append((block_id, pos - 1, data[pos:pos + 2])); pos += 2
        elif block_id == 0x24:
            blocks.append((block_id, pos - 1, data[pos:pos + 2])); pos += 2
        elif block_id == 0x25:
            blocks.append((block_id, pos - 1, b''))
        elif block_id == 0x2A:
            blocks.append((block_id, pos - 1, data[pos:pos + 4])); pos += 4
        elif block_id == 0x30:
            n = data[pos]
            blocks.append((block_id, pos - 1, data[pos:pos + 1 + n])); pos += 1 + n
        elif block_id == 0x31:
            n = data[pos + 1]
            blocks.append((block_id, pos - 1, data[pos:pos + 2 + n])); pos += 2 + n
        elif block_id == 0x32:
            length = struct.unpack_from('<H', data, pos)[0]
            blocks.append((block_id, pos - 1, data[pos:pos + 2 + length])); pos += 2 + length
        elif block_id == 0x33:
            n = data[pos]
            blocks.append((block_id, pos - 1, data[pos:pos + 1 + n * 3])); pos += 1 + n * 3
        elif block_id == 0x35:
            length = struct.unpack_from('<I', data, pos + 16)[0]
            blocks.append((block_id, pos - 1, data[pos:pos + 20 + length])); pos += 20 + length
        elif block_id == 0x5A:
            blocks.append((block_id, pos - 1, data[pos:pos + 9])); pos += 9
        else:
            print(f"Unknown TZX block 0x{block_id:02X} at offset {pos-1}")
            break
    return blocks


def parse_tzx_gdb(block_data):
    """Parse a TZX GDB (0x19) block's data (after block ID byte).

    Returns dict with keys:
        total_len, pause_ms, totp, npp, asp, totd, npd, asd,
        pilot_symdefs: [(flags, [pulse_durations])],
        pilot_prle: [(sym_idx, repeat)],
        data_symdefs: [(flags, [pulse_durations])],
        data_bytes: bytes,
        nb: int (bits per data symbol)
    """
    d = block_data
    pos = 0

    total_len = struct.unpack_from('<I', d, pos)[0]; pos += 4
    pause_ms = struct.unpack_from('<H', d, pos)[0]; pos += 2
    totp = struct.unpack_from('<I', d, pos)[0]; pos += 4
    npp = d[pos]; pos += 1
    asp = d[pos]; pos += 1
    if asp == 0: asp = 256
    totd = struct.unpack_from('<I', d, pos)[0]; pos += 4
    npd = d[pos]; pos += 1
    asd = d[pos]; pos += 1
    if asd == 0: asd = 256

    # Calculate bits per symbol
    nb = 0
    i = asd
    while i > 0:
        nb += 1
        i >>= 1
    if (asd & (asd - 1)) == 0:
        nb -= 1

    # Pilot symbol definitions
    pilot_symdefs = []
    for _ in range(asp):
        flags = d[pos]; pos += 1
        pls = []
        for _ in range(npp):
            pls.append(struct.unpack_from('<H', d, pos)[0]); pos += 2
        pilot_symdefs.append((flags, pls))

    # Pilot PRLE table
    pilot_prle = []
    for _ in range(totp):
        sym = d[pos]; pos += 1
        reps = struct.unpack_from('<H', d, pos)[0]; pos += 2
        pilot_prle.append((sym, reps))

    # Data symbol definitions
    data_symdefs = []
    for _ in range(asd):
        flags = d[pos]; pos += 1
        pls = []
        for _ in range(npd):
            pls.append(struct.unpack_from('<H', d, pos)[0]); pos += 2
        data_symdefs.append((flags, pls))

    # Data bytes
    data_bits = totd * nb
    data_byte_count = (data_bits + 7) // 8
    data_bytes = d[pos:pos + data_byte_count]

    # Count pilot pulses (non-zero duration only)
    total_pilot_pulses = 0
    for sym_idx, reps in pilot_prle:
        flags, pls = pilot_symdefs[sym_idx]
        nonzero = sum(1 for p in pls if p != 0)
        total_pilot_pulses += reps * nonzero

    return {
        'total_len': total_len,
        'pause_ms': pause_ms,
        'totp': totp,
        'npp': npp,
        'asp': asp,
        'totd': totd,
        'npd': npd,
        'asd': asd,
        'nb': nb,
        'pilot_symdefs': pilot_symdefs,
        'pilot_prle': pilot_prle,
        'data_symdefs': data_symdefs,
        'data_bytes': data_bytes,
        'data_byte_count': data_byte_count,
        'total_pilot_pulses': total_pilot_pulses,
    }


# ============================================================
# PZX parsing
# ============================================================

PZX_TAG_NAMES = {
    0x54585A50: "PZXT",  # header
    0x534C5550: "PULS",  # pulse sequence
    0x41544144: "DATA",  # data block
    0x53554150: "PAUS",  # pause
    0x53575242: "BRWS",  # browse point
    0x504F5453: "STOP",  # stop tape
}


def parse_pzx(path):
    """Parse PZX file and return list of (tag_str, offset, raw_block_data, size).

    tag_str: str, 4-char tag (e.g. "PZXT", "PULS", "DATA", "PAUS")
    offset: int, byte offset in file of the block tag
    raw_block_data: bytes, block data after the 8-byte header
    size: int, block data size
    """
    data = Path(path).read_bytes()
    pos = 0
    blocks = []
    while pos + 8 <= len(data):
        tag_int = struct.unpack_from('<I', data, pos)[0]
        size = struct.unpack_from('<I', data, pos + 4)[0]
        tag_str = data[pos:pos+4].decode('ascii', errors='replace')
        block_data = data[pos + 8:pos + 8 + size]
        blocks.append((tag_str, pos, block_data, size))
        pos += 8 + size
    return blocks


def parse_pzx_puls(block_data):
    """Decode PZX PULS block data into list of (repeat, duration) entries.

    Returns list of (repeat: int, duration: int) tuples.
    duration=0 means toggle-only (no actual pulse).
    """
    d = block_data
    pos = 0
    entries = []
    while pos < len(d):
        rep = 1
        w = struct.unpack_from('<H', d, pos)[0]; pos += 2
        if w > 0x8000:
            rep = w & 0x7FFF
            w = struct.unpack_from('<H', d, pos)[0]; pos += 2
        if w >= 0x8000:
            dur = ((w & 0x7FFF) << 16) | struct.unpack_from('<H', d, pos)[0]; pos += 2
        else:
            dur = w
        entries.append((rep, dur))
    return entries


def parse_pzx_data(block_data):
    """Parse PZX DATA block.

    Returns dict with keys:
        bit_count, initial_level, tail_len, p0, p1,
        s0: [int], s1: [int],
        data_bytes: bytes, byte_count: int
    """
    d = block_data
    pos = 0

    count_field = struct.unpack_from('<I', d, pos)[0]; pos += 4
    bit_count = count_field & 0x7FFFFFFF
    initial_level = (count_field >> 31) & 1

    tail_len = struct.unpack_from('<H', d, pos)[0]; pos += 2
    p0 = d[pos]; pos += 1
    p1 = d[pos]; pos += 1

    s0 = []
    for _ in range(p0):
        s0.append(struct.unpack_from('<H', d, pos)[0]); pos += 2
    s1 = []
    for _ in range(p1):
        s1.append(struct.unpack_from('<H', d, pos)[0]); pos += 2

    data_bytes = d[pos:]
    byte_count = (bit_count + 7) // 8

    return {
        'bit_count': bit_count,
        'initial_level': initial_level,
        'tail_len': tail_len,
        'p0': p0, 'p1': p1,
        's0': s0, 's1': s1,
        'data_bytes': data_bytes[:byte_count],
        'byte_count': byte_count,
    }


def parse_pzx_paus(block_data):
    """Parse PZX PAUS block.

    Returns dict with keys: duration (T-states), initial_level, duration_ms
    """
    dur_field = struct.unpack_from('<I', block_data, 0)[0]
    duration = dur_field & 0x7FFFFFFF
    initial_level = (dur_field >> 31) & 1
    return {
        'duration': duration,
        'initial_level': initial_level,
        'duration_ms': duration / 3500.0,
    }


# ============================================================
# CLI: dump file structure
# ============================================================

def dump_tzx(path):
    blocks = parse_tzx(path)
    print(f"TZX file: {path}")
    print(f"Total blocks: {len(blocks)}")
    print()

    for i, (bid, offset, bdata) in enumerate(blocks):
        name = TZX_BLOCK_NAMES.get(bid, "Unknown")
        print(f"Block {i}: 0x{bid:02X} {name}  (offset={offset}, data_size={len(bdata)})")

        if bid == 0x10:
            pause = struct.unpack_from('<H', bdata, 0)[0]
            length = struct.unpack_from('<H', bdata, 2)[0]
            flag = bdata[4] if len(bdata) > 4 else '?'
            print(f"  pause={pause}ms  length={length}  flag=0x{flag:02X}")

        elif bid == 0x19:
            gdb = parse_tzx_gdb(bdata)
            print(f"  pause={gdb['pause_ms']}ms  totp={gdb['totp']}  npp={gdb['npp']}  asp={gdb['asp']}")
            print(f"  totd={gdb['totd']}  npd={gdb['npd']}  asd={gdb['asd']}  nb={gdb['nb']}")
            print(f"  data_bytes={gdb['data_byte_count']}  pilot_pulses={gdb['total_pilot_pulses']}")
            for j, (sym, reps) in enumerate(gdb['pilot_prle']):
                flags, pls = gdb['pilot_symdefs'][sym]
                print(f"    PRLE[{j}]: sym={sym} x{reps}  flags={flags}  pulses={pls}")
            for j, (flags, pls) in enumerate(gdb['data_symdefs']):
                print(f"    DataSym[{j}]: flags={flags}  pulses={pls}")
            first16 = gdb['data_bytes'][:16]
            print(f"  first 16 bytes: {' '.join(f'{b:02X}' for b in first16)}")

        elif bid == 0x21:
            name_str = bdata[1:1+bdata[0]].decode('ascii', errors='replace')
            print(f"  name=\"{name_str}\"")

        elif bid == 0x12:
            pulse_len = struct.unpack_from('<H', bdata, 0)[0]
            num_pulses = struct.unpack_from('<H', bdata, 2)[0]
            print(f"  pulse_len={pulse_len}  num_pulses={num_pulses}")

        elif bid == 0x13:
            n = bdata[0]
            pulses = [struct.unpack_from('<H', bdata, 1 + j*2)[0] for j in range(n)]
            print(f"  num_pulses={n}  durations={pulses}")

        elif bid == 0x20:
            pause = struct.unpack_from('<H', bdata, 0)[0]
            print(f"  pause={pause}ms")

        print()


def dump_pzx(path):
    blocks = parse_pzx(path)
    print(f"PZX file: {path}")
    print(f"Total blocks: {len(blocks)}")
    print()

    for i, (tag, offset, bdata, size) in enumerate(blocks):
        print(f"Block {i}: {tag}  (offset={offset}, size={size})")

        if tag == 'PZXT':
            if len(bdata) >= 2:
                print(f"  version={bdata[0]}.{bdata[1]}")
                info = bdata[2:].split(b'\x00')
                for j, s in enumerate(info):
                    if s:
                        print(f"  info[{j}]: {s.decode('utf-8', errors='replace')}")

        elif tag == 'PULS':
            entries = parse_pzx_puls(bdata)
            total_pulses = sum(r for r, d in entries if d > 0)
            total_tstates = sum(r * d for r, d in entries)
            zero_toggles = sum(r for r, d in entries if d == 0)
            print(f"  entries={len(entries)}  real_pulses={total_pulses}  total_T={total_tstates}  zero_toggles={zero_toggles}")
            for j, (rep, dur) in enumerate(entries):
                extra = " (toggle)" if dur == 0 else ""
                print(f"    [{j}] {rep}x {dur}T{extra}")

        elif tag == 'DATA':
            dat = parse_pzx_data(bdata)
            print(f"  bits={dat['bit_count']}  bytes={dat['byte_count']}  tail={dat['tail_len']}T")
            print(f"  initial_level={dat['initial_level']}  p0={dat['p0']}  p1={dat['p1']}")
            print(f"  s0={dat['s0']}  s1={dat['s1']}")
            first16 = dat['data_bytes'][:16]
            print(f"  first 16 bytes: {' '.join(f'{b:02X}' for b in first16)}")

        elif tag == 'PAUS':
            paus = parse_pzx_paus(bdata)
            print(f"  duration={paus['duration']}T ({paus['duration_ms']:.1f}ms)  level={paus['initial_level']}")

        elif tag == 'BRWS':
            text = bdata.decode('utf-8', errors='replace')
            print(f"  text=\"{text}\"")

        elif tag == 'STOP':
            flags = struct.unpack_from('<H', bdata, 0)[0] if len(bdata) >= 2 else 0
            print(f"  flags={flags}" + (" (48K only)" if flags == 1 else " (always)" if flags == 0 else ""))

        print()


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <file.tzx|file.pzx>")
        sys.exit(1)

    path = sys.argv[1]
    if path.lower().endswith('.tzx'):
        dump_tzx(path)
    elif path.lower().endswith('.pzx'):
        dump_pzx(path)
    else:
        print(f"Unknown file type: {path}")
        sys.exit(1)


if __name__ == '__main__':
    main()
