#!/usr/bin/env python3
"""Compare Dan Dare 2 TZX vs PZX block structure."""

import struct
import math
import sys

TZX_FILE = "/home/drew/github/pico-spec/debug/Dan Dare 2 - Mekon's Revenge.tzx"
PZX_FILE = "/home/drew/github/pico-spec/debug/Dan Dare 2 - Mekon's Revenge.pzx"
OUT_FILE = "/home/drew/github/pico-spec/debug/dandare2_structure.txt"

def u8(data, off):
    return data[off]

def u16(data, off):
    return struct.unpack_from('<H', data, off)[0]

def u32(data, off):
    return struct.unpack_from('<I', data, off)[0]


# ============================================================
# TZX Parser
# ============================================================

TZX_BLOCK_NAMES = {
    0x10: "Standard Speed Data",
    0x11: "Turbo Speed Data",
    0x12: "Pure Tone",
    0x13: "Pulse Sequence",
    0x14: "Pure Data",
    0x15: "Direct Recording",
    0x18: "CSW Recording",
    0x19: "Generalized Data Block",
    0x20: "Pause / Stop the Tape",
    0x21: "Group Start",
    0x22: "Group End",
    0x23: "Jump to Block",
    0x24: "Loop Start",
    0x25: "Loop End",
    0x26: "Call Sequence",
    0x27: "Return from Sequence",
    0x28: "Select Block",
    0x2A: "Stop if 48K",
    0x2B: "Set Signal Level",
    0x30: "Text Description",
    0x31: "Message Block",
    0x32: "Archive Info",
    0x33: "Hardware Type",
    0x35: "Custom Info Block",
    0x5A: "Glue Block",
}

def parse_tzx(data):
    """Parse TZX file, return list of block dicts."""
    # Header: "ZXTape!" 0x1A major minor
    sig = data[0:7]
    if sig != b'ZXTape!':
        raise ValueError(f"Not a TZX file: {sig}")
    eof_byte = data[7]
    major = data[8]
    minor = data[9]

    blocks = []
    off = 10
    idx = 0

    while off < len(data):
        block_id = data[off]
        off += 1
        block = {'index': idx, 'id': block_id, 'name': TZX_BLOCK_NAMES.get(block_id, f"Unknown(0x{block_id:02X})"), 'offset': off - 1}

        if block_id == 0x10:
            pause = u16(data, off)
            length = u16(data, off + 2)
            bdata = data[off + 4: off + 4 + length]
            block['pause'] = pause
            block['data_len'] = length
            block['data'] = bdata
            block['flag'] = bdata[0] if length > 0 else None
            off += 4 + length

        elif block_id == 0x11:
            pilot_pulse = u16(data, off)
            sync1 = u16(data, off + 2)
            sync2 = u16(data, off + 4)
            zero_pulse = u16(data, off + 6)
            one_pulse = u16(data, off + 8)
            pilot_len = u16(data, off + 10)
            used_bits = data[off + 12]
            pause = u16(data, off + 13)
            length = u16(data, off + 15) | (data[off + 17] << 16)
            bdata = data[off + 18: off + 18 + length]
            block['pilot_pulse'] = pilot_pulse
            block['sync1'] = sync1
            block['sync2'] = sync2
            block['zero_pulse'] = zero_pulse
            block['one_pulse'] = one_pulse
            block['pilot_len'] = pilot_len
            block['used_bits'] = used_bits
            block['pause'] = pause
            block['data_len'] = length
            block['data'] = bdata
            block['flag'] = bdata[0] if length > 0 else None
            off += 18 + length

        elif block_id == 0x12:
            pulse_len = u16(data, off)
            num_pulses = u16(data, off + 2)
            block['pulse_len'] = pulse_len
            block['num_pulses'] = num_pulses
            off += 4

        elif block_id == 0x13:
            num = data[off]
            pulses = []
            for i in range(num):
                pulses.append(u16(data, off + 1 + i * 2))
            block['pulses'] = pulses
            off += 1 + num * 2

        elif block_id == 0x14:
            zero_pulse = u16(data, off)
            one_pulse = u16(data, off + 2)
            used_bits = data[off + 4]
            pause = u16(data, off + 5)
            length = u16(data, off + 7) | (data[off + 9] << 16)
            bdata = data[off + 10: off + 10 + length]
            block['zero_pulse'] = zero_pulse
            block['one_pulse'] = one_pulse
            block['used_bits'] = used_bits
            block['pause'] = pause
            block['data_len'] = length
            block['data'] = bdata
            block['flag'] = bdata[0] if length > 0 else None
            off += 10 + length

        elif block_id == 0x19:
            block_len = u32(data, off)
            base = off
            pause = u16(data, off + 4)
            totp = u32(data, off + 6)
            npp = data[off + 10]
            asp = data[off + 11]
            totd = u32(data, off + 12)
            npd = data[off + 16]
            asd = data[off + 17]

            block['pause'] = pause
            block['totp'] = totp
            block['npp'] = npp
            block['asp'] = asp
            block['totd'] = totd
            block['npd'] = npd
            block['asd'] = asd

            # Parse pilot symbol definitions: asp entries, each (1 + npp*2) bytes
            # Each: flags(u8) + npp pulse durations (u16 each)
            pilot_symdefs = []
            sym_off = off + 18
            for s in range(asp):
                flags = data[sym_off]
                pulses = []
                for p in range(npp):
                    pulses.append(u16(data, sym_off + 1 + p * 2))
                pilot_symdefs.append({'flags': flags, 'pulses': pulses})
                sym_off += 1 + npp * 2
            block['pilot_symdefs'] = pilot_symdefs

            # Parse pilot PRLE: totp entries, each 3 bytes (sym_idx u8, repeat u16)
            pilot_prle = []
            total_pilot_pulses = 0
            for pr in range(totp):
                sym_idx = data[sym_off]
                repeat = u16(data, sym_off + 1)
                sym = pilot_symdefs[sym_idx] if sym_idx < len(pilot_symdefs) else None
                npulses_per_sym = len(sym['pulses']) if sym else 0
                pilot_prle.append({
                    'sym_idx': sym_idx,
                    'repeat': repeat,
                    'sym': sym,
                })
                total_pilot_pulses += repeat * npulses_per_sym
                sym_off += 3
            block['pilot_prle'] = pilot_prle
            block['total_pilot_pulses'] = total_pilot_pulses

            # Parse data symbol definitions
            data_symdefs = []
            for s in range(asd):
                flags = data[sym_off]
                pulses = []
                for p in range(npd):
                    pulses.append(u16(data, sym_off + 1 + p * 2))
                data_symdefs.append({'flags': flags, 'pulses': pulses})
                sym_off += 1 + npd * 2
            block['data_symdefs'] = data_symdefs

            # Data bits
            if asd > 0:
                nb = max(1, math.ceil(math.log2(asd))) if asd > 1 else 1
            else:
                nb = 0
            total_data_bits = totd * nb if nb else 0
            data_bytes = math.ceil(total_data_bits / 8) if total_data_bits else 0
            block['data_byte_count'] = data_bytes
            block['nb'] = nb
            bdata = data[sym_off: sym_off + data_bytes]
            block['data'] = bdata
            block['data_len'] = len(bdata)

            off += 4 + block_len

        elif block_id == 0x20:
            pause = u16(data, off)
            block['pause'] = pause
            off += 2

        elif block_id == 0x21:
            length = data[off]
            name = data[off + 1: off + 1 + length].decode('ascii', errors='replace')
            block['group_name'] = name
            off += 1 + length

        elif block_id == 0x22:
            pass  # no data

        elif block_id == 0x30:
            length = data[off]
            text = data[off + 1: off + 1 + length].decode('ascii', errors='replace')
            block['text'] = text
            off += 1 + length

        elif block_id == 0x32:
            length = u16(data, off)
            block['archive_len'] = length
            off += 2 + length

        elif block_id == 0x33:
            n = data[off]
            off += 1 + n * 3

        elif block_id == 0x35:
            # Custom info
            off += 16  # id string
            length = u32(data, off)
            off += 4 + length

        elif block_id == 0x5A:
            off += 9

        else:
            # Try to read length from first 4 bytes for unknown blocks
            print(f"WARNING: Unknown TZX block 0x{block_id:02X} at offset {off-1}")
            break

        blocks.append(block)
        idx += 1

    return blocks, major, minor


# ============================================================
# PZX Parser
# ============================================================

def parse_puls_entries(data):
    """Parse PULS block data into list of (repeat, duration) entries."""
    entries = []
    off = 0
    while off < len(data):
        w = u16(data, off)
        off += 2

        if w > 0x8000:
            count = w & 0x7FFF
            if off + 2 > len(data):
                break
            w2 = u16(data, off)
            off += 2
            if w2 >= 0x8000:
                if off + 2 > len(data):
                    break
                dur = ((w2 & 0x7FFF) << 16) | u16(data, off)
                off += 2
            else:
                dur = w2
            entries.append((count, dur))
        elif w == 0x8000:
            # count = 0x8000 & 0x7FFF = 0 — actually this means count follows
            # w == 0x8000 means count=0 (& 0x7FFF), which is meaningless
            # Actually: if bit15 set, count = w & 0x7FFF, then read duration
            count = 0
            if off + 2 > len(data):
                break
            w2 = u16(data, off)
            off += 2
            if w2 >= 0x8000:
                if off + 2 > len(data):
                    break
                dur = ((w2 & 0x7FFF) << 16) | u16(data, off)
                off += 2
            else:
                dur = w2
            entries.append((count, dur))
        else:
            # w is a duration, count = 1
            entries.append((1, w))

    return entries


def parse_pzx(data):
    """Parse PZX file, return header info and list of block dicts."""
    # First block must be PZXT
    tag = data[0:4]
    if tag != b'PZXT':
        raise ValueError(f"Not a PZX file: {tag}")

    size = u32(data, 4)
    header_data = data[8:8 + size]
    # PZXT: major(u8) + minor(u8) + info strings (null-terminated)
    pzx_major = header_data[0]
    pzx_minor = header_data[1]
    info_strings = header_data[2:].split(b'\x00')
    info_strings = [s.decode('ascii', errors='replace') for s in info_strings if s]

    blocks = []
    off = 8 + size
    idx = 0

    while off < len(data):
        if off + 8 > len(data):
            break
        btag = data[off:off + 4].decode('ascii', errors='replace')
        bsize = u32(data, off + 4)
        bdata = data[off + 8: off + 8 + bsize]

        block = {'index': idx, 'tag': btag, 'size': bsize, 'offset': off}

        if btag == 'PULS':
            entries = parse_puls_entries(bdata)
            block['entries'] = entries
            total_pulses = sum(e[0] for e in entries)
            total_tstates = sum(e[0] * e[1] for e in entries)
            zero_dur_toggles = sum(e[0] for e in entries if e[1] == 0)
            block['total_pulses'] = total_pulses
            block['total_tstates'] = total_tstates
            block['zero_dur_toggles'] = zero_dur_toggles

        elif btag == 'DATA':
            if len(bdata) < 4:
                block['error'] = 'DATA block too short'
            else:
                count_raw = u32(bdata, 0)
                initial_level = (count_raw >> 31) & 1
                bit_count = count_raw & 0x7FFFFFFF
                tail = u16(bdata, 4)
                p0 = bdata[6]
                p1 = bdata[7]

                doff = 8
                s0 = []
                for i in range(p0):
                    s0.append(u16(bdata, doff))
                    doff += 2
                s1 = []
                for i in range(p1):
                    s1.append(u16(bdata, doff))
                    doff += 2

                byte_count = math.ceil(bit_count / 8)
                payload = bdata[doff:doff + byte_count]

                block['bit_count'] = bit_count
                block['byte_count'] = byte_count
                block['tail'] = tail
                block['initial_level'] = initial_level
                block['p0'] = p0
                block['p1'] = p1
                block['s0'] = s0
                block['s1'] = s1
                block['data'] = payload

        elif btag == 'PAUS':
            if len(bdata) >= 4:
                dur_raw = u32(bdata, 0)
                initial_level = (dur_raw >> 31) & 1
                duration = dur_raw & 0x7FFFFFFF
                block['duration_tstates'] = duration
                block['duration_ms'] = duration / 3500.0
                block['initial_level'] = initial_level

        elif btag == 'BRWS':
            block['text'] = bdata.decode('ascii', errors='replace').rstrip('\x00')

        elif btag == 'STOP':
            if len(bdata) >= 2:
                block['flags'] = u16(bdata, 0)

        blocks.append(block)
        off += 8 + bsize
        idx += 1

    return blocks, pzx_major, pzx_minor, info_strings


# ============================================================
# Output
# ============================================================

def hex16(data):
    """First 16 bytes as hex string."""
    return ' '.join(f'{b:02X}' for b in data[:16])

def format_tzx_block(b):
    lines = []
    hdr = f"Block {b['index']:3d}: ID=0x{b['id']:02X} ({b['name']})"
    lines.append(hdr)

    bid = b['id']
    if bid == 0x10:
        lines.append(f"  Pause: {b['pause']} ms")
        lines.append(f"  Data length: {b['data_len']} bytes")
        lines.append(f"  Flag byte: 0x{b['flag']:02X}" if b['flag'] is not None else "  Flag byte: N/A")
        lines.append(f"  First 16 bytes: {hex16(b['data'])}")

    elif bid == 0x11:
        lines.append(f"  Pilot pulse: {b['pilot_pulse']} T")
        lines.append(f"  Sync1: {b['sync1']} T, Sync2: {b['sync2']} T")
        lines.append(f"  Zero pulse: {b['zero_pulse']} T, One pulse: {b['one_pulse']} T")
        lines.append(f"  Pilot length: {b['pilot_len']} pulses")
        lines.append(f"  Used bits in last byte: {b['used_bits']}")
        lines.append(f"  Pause: {b['pause']} ms")
        lines.append(f"  Data length: {b['data_len']} bytes")
        lines.append(f"  Flag byte: 0x{b['flag']:02X}" if b['flag'] is not None else "  Flag byte: N/A")
        lines.append(f"  First 16 bytes: {hex16(b['data'])}")

    elif bid == 0x12:
        lines.append(f"  Pulse length: {b['pulse_len']} T, Count: {b['num_pulses']}")

    elif bid == 0x13:
        lines.append(f"  Pulses ({len(b['pulses'])}): {b['pulses']}")

    elif bid == 0x14:
        lines.append(f"  Zero pulse: {b['zero_pulse']} T, One pulse: {b['one_pulse']} T")
        lines.append(f"  Used bits in last byte: {b['used_bits']}")
        lines.append(f"  Pause: {b['pause']} ms")
        lines.append(f"  Data length: {b['data_len']} bytes")
        lines.append(f"  Flag byte: 0x{b['flag']:02X}" if b['flag'] is not None else "  Flag byte: N/A")
        lines.append(f"  First 16 bytes: {hex16(b['data'])}")

    elif bid == 0x19:
        lines.append(f"  Pause: {b['pause']} ms")
        lines.append(f"  TOTP (pilot entries): {b['totp']}")
        lines.append(f"  NPP (max pilot pulses/sym): {b['npp']}")
        lines.append(f"  ASP (pilot symbol count): {b['asp']}")
        lines.append(f"  TOTD (data symbols total): {b['totd']}")
        lines.append(f"  NPD (max data pulses/sym): {b['npd']}")
        lines.append(f"  ASD (data symbol count): {b['asd']}")
        lines.append(f"  Data byte count: {b['data_byte_count']}")
        lines.append(f"  Bits per data symbol: {b['nb']}")

        if b['pilot_symdefs']:
            lines.append(f"  Pilot symbol definitions ({len(b['pilot_symdefs'])}):")
            for si, sd in enumerate(b['pilot_symdefs']):
                lines.append(f"    Sym {si}: flags={sd['flags']}, pulses={sd['pulses']}")

        if b['pilot_prle']:
            lines.append(f"  Pilot PRLE entries ({len(b['pilot_prle'])}):")
            total_pp = 0
            for pr in b['pilot_prle']:
                sym = pr['sym']
                npulses = len(sym['pulses']) if sym else 0
                entry_pulses = pr['repeat'] * npulses
                total_pp += entry_pulses
                lines.append(f"    sym_idx={pr['sym_idx']}, repeat={pr['repeat']}, "
                           f"sym_pulses={sym['pulses'] if sym else '?'}, "
                           f"total_pulses={entry_pulses}")
            lines.append(f"  Total pilot pulse count: {total_pp}")

        if b['data_symdefs']:
            lines.append(f"  Data symbol definitions ({len(b['data_symdefs'])}):")
            for si, sd in enumerate(b['data_symdefs']):
                lines.append(f"    Sym {si}: flags={sd['flags']}, pulses={sd['pulses']}")

        if b.get('data'):
            lines.append(f"  First 16 data bytes: {hex16(b['data'])}")

    elif bid == 0x20:
        lines.append(f"  Pause: {b['pause']} ms")

    elif bid == 0x21:
        lines.append(f"  Group name: \"{b['group_name']}\"")

    elif bid == 0x22:
        lines.append(f"  (no data)")

    elif bid == 0x30:
        lines.append(f"  Text: \"{b['text']}\"")

    elif bid == 0x32:
        lines.append(f"  Archive info, {b['archive_len']} bytes")

    return '\n'.join(lines)


def format_pzx_block(b):
    lines = []
    hdr = f"Block {b['index']:3d}: {b['tag']}  size={b['size']}"
    lines.append(hdr)

    tag = b['tag']
    if tag == 'PULS':
        lines.append(f"  Entries ({len(b['entries'])}):")
        for i, (count, dur) in enumerate(b['entries']):
            lines.append(f"    [{i:2d}] repeat={count}, duration={dur} T")
        lines.append(f"  Total real pulses: {b['total_pulses']}")
        lines.append(f"  Total T-states: {b['total_tstates']}")
        lines.append(f"  Zero-duration toggle count: {b['zero_dur_toggles']}")

    elif tag == 'DATA':
        if 'error' in b:
            lines.append(f"  ERROR: {b['error']}")
        else:
            lines.append(f"  Bit count: {b['bit_count']}")
            lines.append(f"  Byte count: {b['byte_count']}")
            lines.append(f"  Tail length: {b['tail']} T")
            lines.append(f"  Initial level: {b['initial_level']}")
            lines.append(f"  p0={b['p0']}, p1={b['p1']}")
            lines.append(f"  s0={b['s0']}")
            lines.append(f"  s1={b['s1']}")
            if b.get('data'):
                lines.append(f"  First 16 data bytes: {hex16(b['data'])}")

    elif tag == 'PAUS':
        lines.append(f"  Duration: {b.get('duration_tstates', '?')} T-states "
                     f"({b.get('duration_ms', 0):.1f} ms at 3.5MHz)")
        lines.append(f"  Initial level: {b.get('initial_level', '?')}")

    elif tag == 'BRWS':
        lines.append(f"  Text: \"{b.get('text', '')}\"")

    elif tag == 'STOP':
        lines.append(f"  Flags: {b.get('flags', '?')}")

    return '\n'.join(lines)


def build_block_mapping(tzx_blocks, pzx_blocks):
    """Map TZX blocks to PZX blocks and compare."""
    lines = []

    # Build lists of data-carrying blocks from each
    # Strategy: walk through both in order, matching data blocks

    tzx_data_blocks = []
    for b in tzx_blocks:
        if b['id'] in (0x10, 0x11, 0x14, 0x19) and b.get('data'):
            tzx_data_blocks.append(b)

    pzx_data_blocks = []
    pzx_puls_before_data = {}  # pzx data block index -> preceding PULS blocks
    pzx_paus_after_data = {}   # pzx data block index -> following PAUS block

    pending_puls = []
    for i, b in enumerate(pzx_blocks):
        if b['tag'] == 'PULS':
            pending_puls.append(b)
        elif b['tag'] == 'DATA':
            didx = len(pzx_data_blocks)
            pzx_puls_before_data[didx] = list(pending_puls)
            pending_puls = []
            pzx_data_blocks.append(b)
            # Look for PAUS after
            if i + 1 < len(pzx_blocks) and pzx_blocks[i + 1]['tag'] == 'PAUS':
                pzx_paus_after_data[didx] = pzx_blocks[i + 1]

    lines.append(f"Data-carrying blocks: TZX={len(tzx_data_blocks)}, PZX={len(pzx_data_blocks)}")
    lines.append("")

    for di in range(max(len(tzx_data_blocks), len(pzx_data_blocks))):
        lines.append(f"--- Data Block Pair {di} ---")

        tb = tzx_data_blocks[di] if di < len(tzx_data_blocks) else None
        pb = pzx_data_blocks[di] if di < len(pzx_data_blocks) else None

        if tb:
            lines.append(f"TZX Block {tb['index']}: ID=0x{tb['id']:02X} ({tb['name']})")
            lines.append(f"  Data: {tb['data_len']} bytes, first 16: {hex16(tb['data'])}")

            # Pilot info
            if tb['id'] == 0x10:
                # Standard: 8063 pilot pulses for flag=0, 3223 for flag!=0
                flag = tb.get('flag', 0)
                pilot_count = 8063 if flag == 0 else 3223
                lines.append(f"  Standard block pilot: {pilot_count} pulses (2168 T each)")
                lines.append(f"  Pause: {tb['pause']} ms")
            elif tb['id'] == 0x11:
                lines.append(f"  Turbo pilot: {tb['pilot_len']} pulses ({tb['pilot_pulse']} T each)")
                lines.append(f"  Pause: {tb['pause']} ms")
            elif tb['id'] == 0x14:
                lines.append(f"  Pure data (no pilot)")
                lines.append(f"  Pause: {tb['pause']} ms")
            elif tb['id'] == 0x19:
                lines.append(f"  GDB pilot: {tb['total_pilot_pulses']} total pulses")
                lines.append(f"  Pause: {tb['pause']} ms")
        else:
            lines.append(f"TZX: (no matching block)")

        if pb:
            puls_blocks = pzx_puls_before_data.get(di, [])
            lines.append(f"PZX Block {pb['index']}: DATA")
            lines.append(f"  Data: {pb['byte_count']} bytes, first 16: {hex16(pb['data'])}")

            if puls_blocks:
                total_pilot = sum(b['total_pulses'] for b in puls_blocks)
                lines.append(f"  Preceding PULS blocks: {[b['index'] for b in puls_blocks]}, "
                           f"total pulses: {total_pilot}")

            paus = pzx_paus_after_data.get(di)
            if paus:
                lines.append(f"  Following PAUS block {paus['index']}: "
                           f"{paus.get('duration_ms', 0):.1f} ms")
        else:
            lines.append(f"PZX: (no matching block)")

        # Comparisons
        if tb and pb:
            td = tb['data']
            pd = pb['data']
            match = td[:16] == pd[:16]
            full_match = td == pd
            lines.append(f"  DATA MATCH (first 16): {'YES' if match else 'NO'}")
            lines.append(f"  DATA MATCH (full): {'YES' if full_match else 'NO'}")
            if not match:
                lines.append(f"    TZX: {hex16(td)}")
                lines.append(f"    PZX: {hex16(pd)}")

            # Pause comparison
            tzx_pause = tb.get('pause', 0)
            paus = pzx_paus_after_data.get(di)
            pzx_pause_ms = paus.get('duration_ms', 0) if paus else 0
            lines.append(f"  PAUSE: TZX={tzx_pause} ms, PZX={pzx_pause_ms:.1f} ms")

        lines.append("")

    return '\n'.join(lines)


def find_key_differences(tzx_blocks, pzx_blocks):
    """Summarize key differences."""
    lines = []

    # Count block types
    tzx_types = {}
    for b in tzx_blocks:
        t = f"0x{b['id']:02X}"
        tzx_types[t] = tzx_types.get(t, 0) + 1

    pzx_types = {}
    for b in pzx_blocks:
        pzx_types[b['tag']] = pzx_types.get(b['tag'], 0) + 1

    lines.append(f"TZX block type counts: {tzx_types}")
    lines.append(f"PZX block type counts: {pzx_types}")
    lines.append("")

    # Check data consistency
    tzx_data_blocks = [b for b in tzx_blocks if b['id'] in (0x10, 0x11, 0x14, 0x19) and b.get('data')]
    pzx_data_blocks = [b for b in pzx_blocks if b['tag'] == 'DATA']

    if len(tzx_data_blocks) != len(pzx_data_blocks):
        lines.append(f"DIFFERENCE: Data block count mismatch: TZX={len(tzx_data_blocks)}, PZX={len(pzx_data_blocks)}")

    for i in range(min(len(tzx_data_blocks), len(pzx_data_blocks))):
        tb = tzx_data_blocks[i]
        pb = pzx_data_blocks[i]
        if tb['data'] != pb['data']:
            lines.append(f"DIFFERENCE: Data block {i} content mismatch! "
                       f"TZX len={len(tb['data'])}, PZX len={len(pb['data'])}")

        # Check pause
        tzx_pause = tb.get('pause', 0)
        # Find PAUS after this PZX DATA block
        pb_idx_in_all = pb['index']
        pzx_paus = None
        for j, b in enumerate(pzx_blocks):
            if b is pb:
                if j + 1 < len(pzx_blocks) and pzx_blocks[j + 1]['tag'] == 'PAUS':
                    pzx_paus = pzx_blocks[j + 1]
                break

        if pzx_paus:
            pzx_pause_ms = pzx_paus.get('duration_ms', 0)
            if abs(tzx_pause - pzx_pause_ms) > 1.0:
                lines.append(f"DIFFERENCE: Data block {i} pause mismatch: "
                           f"TZX={tzx_pause} ms, PZX={pzx_pause_ms:.1f} ms")

    # Check for GDB blocks (these are complex)
    gdb_blocks = [b for b in tzx_blocks if b['id'] == 0x19]
    if gdb_blocks:
        lines.append(f"\nGDB blocks found: {len(gdb_blocks)} (complex encoding, may differ in pilot representation)")
        for gb in gdb_blocks:
            lines.append(f"  TZX Block {gb['index']}: {gb['totd']} data symbols, "
                       f"{gb['total_pilot_pulses']} pilot pulses")

    # Pilot pulse count differences
    lines.append("\nPilot pulse count analysis:")
    lines.append("TZX GDB counts pilot pulses as TOTP * pulses_per_symbol (including zero-duration half-pulses).")
    lines.append("PZX PULS counts actual pulse entries (excluding zero-duration toggles from real pulse count).")
    lines.append("")

    tzx_data_blocks2 = [b for b in tzx_blocks if b['id'] in (0x10, 0x11, 0x14, 0x19) and b.get('data')]
    pzx_puls_map = {}
    pending = []
    di2 = 0
    for b in pzx_blocks:
        if b['tag'] == 'PULS':
            pending.append(b)
        elif b['tag'] == 'DATA':
            pzx_puls_map[di2] = list(pending)
            pending = []
            di2 += 1

    for i in range(min(len(tzx_data_blocks2), len(pzx_data_blocks))):
        tb = tzx_data_blocks2[i]
        puls_list = pzx_puls_map.get(i, [])
        pzx_pulses = sum(b['total_pulses'] for b in puls_list)
        pzx_zero_toggles = sum(b['zero_dur_toggles'] for b in puls_list)

        if tb['id'] == 0x10:
            flag = tb.get('flag', 0)
            tzx_pilot = 8063 if flag == 0 else 3223
            tzx_pilot_desc = f"{tzx_pilot} (standard 0x10, flag=0x{flag:02X})"
        elif tb['id'] == 0x11:
            tzx_pilot = tb['pilot_len']
            tzx_pilot_desc = f"{tzx_pilot} (turbo 0x11)"
        elif tb['id'] == 0x19:
            tzx_pilot = tb.get('total_pilot_pulses', 0)
            tzx_pilot_desc = f"{tzx_pilot} (GDB 0x19, TOTP={tb['totp']})"
        else:
            tzx_pilot = 0
            tzx_pilot_desc = "0 (pure data)"

        diff = ""
        if tzx_pilot != pzx_pulses:
            diff = f" << DIFFERENCE (delta={tzx_pilot - pzx_pulses})"
            # Explain: GDB sym0 has zero-dur second pulse, so TZX counts 2*repeat but PZX only repeat real + toggles
            if tb['id'] == 0x19 and tb.get('pilot_symdefs'):
                sym0 = tb['pilot_symdefs'][0]
                if len(sym0['pulses']) >= 2 and sym0['pulses'][1] == 0:
                    prle0 = tb['pilot_prle'][0] if tb['pilot_prle'] else None
                    if prle0:
                        diff += f"\n    GDB sym0 has zero-dur pulse: {sym0['pulses']}, repeat={prle0['repeat']}"
                        diff += f"\n    TZX counts {prle0['repeat']}*2={prle0['repeat']*2} pulses; PZX counts {prle0['repeat']} real + 1 zero-dur toggle"

        lines.append(f"  Pair {i:2d}: TZX pilot={tzx_pilot_desc}, PZX PULS={pzx_pulses} (zero-toggles={pzx_zero_toggles}){diff}")

    # Standard block pilot difference: 8063 vs 8065, 3223 vs 3225 (sync pulses included in PZX PULS)
    lines.append("")
    lines.append("Standard block (0x10) note: PZX PULS includes 2 sync pulses (667T + 735T) that TZX")
    lines.append("encodes implicitly in the 0x10 block format. So PZX pilot count = TZX pilot + 2 sync.")
    lines.append("")
    lines.append("GDB block (0x19) note: TZX pilot symbol 0 typically has [2168, 0] (2 pulses per symbol),")
    lines.append("so TOTP * 2 includes zero-duration half-pulses. PZX PULS encodes the 1599 real 2168T pulses")
    lines.append("separately from the remaining pilot sequence entries, without zero-duration inflation.")
    lines.append("The actual waveform is equivalent -- just counted differently.")

    lines.append("")
    lines.append("Tail pulse note: PZX DATA blocks in the Speedlock section (blocks 17-35) have tail=0,")
    lines.append("while TZX GDB blocks 8-16 also have pause=0. The first GDB blocks (3-5) have non-zero")
    lines.append("pause which maps to PZX PAUS blocks with matching duration.")

    return '\n'.join(lines)


def main():
    with open(TZX_FILE, 'rb') as f:
        tzx_data = f.read()
    with open(PZX_FILE, 'rb') as f:
        pzx_data = f.read()

    tzx_blocks, tzx_major, tzx_minor = parse_tzx(tzx_data)
    pzx_blocks, pzx_major, pzx_minor, pzx_info = parse_pzx(pzx_data)

    out = []
    out.append("=" * 80)
    out.append("Dan Dare 2 - Mekon's Revenge: TZX vs PZX Block Structure Comparison")
    out.append("=" * 80)
    out.append(f"TZX file: {TZX_FILE} ({len(tzx_data)} bytes)")
    out.append(f"PZX file: {PZX_FILE} ({len(pzx_data)} bytes)")
    out.append(f"TZX version: {tzx_major}.{tzx_minor}")
    out.append(f"PZX version: {pzx_major}.{pzx_minor}")
    if pzx_info:
        out.append(f"PZX info: {pzx_info}")
    out.append("")

    # Part 1: TZX Structure
    out.append("=" * 80)
    out.append("PART 1: TZX STRUCTURE")
    out.append("=" * 80)
    out.append(f"Total blocks: {len(tzx_blocks)}")
    out.append("")
    for b in tzx_blocks:
        out.append(format_tzx_block(b))
        out.append("")

    # Part 2: PZX Structure
    out.append("=" * 80)
    out.append("PART 2: PZX STRUCTURE")
    out.append("=" * 80)
    out.append(f"Total blocks (after PZXT header): {len(pzx_blocks)}")
    out.append("")
    for b in pzx_blocks:
        out.append(format_pzx_block(b))
        out.append("")

    # Part 3: Block Mapping
    out.append("=" * 80)
    out.append("PART 3: BLOCK MAPPING (TZX -> PZX)")
    out.append("=" * 80)
    out.append("")
    out.append(build_block_mapping(tzx_blocks, pzx_blocks))

    # Part 4: Key Differences
    out.append("=" * 80)
    out.append("PART 4: KEY DIFFERENCES")
    out.append("=" * 80)
    out.append("")
    out.append(find_key_differences(tzx_blocks, pzx_blocks))

    text = '\n'.join(out)
    with open(OUT_FILE, 'w') as f:
        f.write(text)

    print(f"Written {len(text)} bytes to {OUT_FILE}")
    print(f"TZX: {len(tzx_blocks)} blocks, PZX: {len(pzx_blocks)} blocks")


if __name__ == '__main__':
    main()
