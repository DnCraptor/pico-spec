#!/usr/bin/env python3
"""
Compare TZX GDB vs PZX PULS+DATA pulse waveforms for Dan Dare 2.

Generates the exact sequence of (ear_level, duration_tstates) pulses
for the first turbo block in both formats, matching the emulator logic
in Tape.cpp / Tape_PZX.cpp / Tape_TZX.cpp.
"""

import struct
import sys
from pathlib import Path

MAX_PULSES = 1660  # Total pulses to compare (enough for 1599 pilot + some data)

# ============================================================
# TZX parsing
# ============================================================

def parse_tzx(path):
    """Parse TZX file and return list of (block_id, offset, raw_data_after_id)."""
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


def simulate_tzx_gdb(block_data, initial_ear=0, max_pulses=MAX_PULSES):
    """
    Simulate GDB block pulse generation matching Tape_TZX.cpp + Tape.cpp.
    block_data starts at the 4-byte length field (after block ID).
    Returns (list of (ear, duration), pilot_count).
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

    nb = 0
    i = asd
    while i > 0:
        nb += 1
        i >>= 1
    if (asd & (asd - 1)) == 0:
        nb -= 1

    print(f"  TZX GDB: pause={pause_ms}ms totp={totp} npp={npp} asp={asp} totd={totd} npd={npd} asd={asd} nb={nb}")

    ear = initial_ear
    pulses = []

    def emit(dur):
        pulses.append((ear, dur))

    def apply_flags(sf):
        nonlocal ear
        if sf == 0: ear ^= 1
        elif sf == 2: ear = 0
        elif sf == 3: ear = 1

    # Read pilot symbol definition table
    pilot_symdefs = []
    for i in range(asp):
        flags = d[pos]; pos += 1
        pls = []
        for j in range(npp):
            pls.append(struct.unpack_from('<H', d, pos)[0]); pos += 2
        pilot_symdefs.append((flags, pls))
        print(f"    Pilot sym[{i}]: flags={flags} pulses={pls}")

    # Read pilot PRLE table
    pilot_prle = []
    for i in range(totp):
        sym = d[pos]; pos += 1
        reps = struct.unpack_from('<H', d, pos)[0]; pos += 2
        pilot_prle.append((sym, reps))
        print(f"    PRLE[{i}]: sym={sym} reps={reps}")

    # --- Simulate GDB_PILOTSYNC ---
    if totp > 0:
        prle_idx = 0
        sym_idx, reps = pilot_prle[prle_idx]

        # GetBlock: apply flags, set tapeNext, curGDBPulse=0
        apply_flags(pilot_symdefs[sym_idx][0])
        tape_next = pilot_symdefs[sym_idx][1][0]
        cur_pulse = 0

        emit(tape_next)  # First pulse from GetBlock

        while len(pulses) < max_pulses:
            # Read() GDB_PILOTSYNC
            cur_pulse += 1
            if cur_pulse < npp:
                tape_next = pilot_symdefs[sym_idx][1][cur_pulse]

            if tape_next == 0 or cur_pulse == npp:
                # Symbol instance done, next rep
                reps -= 1
                if reps == 0:
                    prle_idx += 1
                    if prle_idx >= totp:
                        break  # End of pilot
                    sym_idx, reps = pilot_prle[prle_idx]
                    apply_flags(pilot_symdefs[sym_idx][0])
                    tape_next = pilot_symdefs[sym_idx][1][0]
                    cur_pulse = 0
                else:
                    # Next rep of same symbol
                    apply_flags(pilot_symdefs[sym_idx][0])
                    tape_next = pilot_symdefs[sym_idx][1][0]
                    cur_pulse = 0
                emit(tape_next)
            else:
                # Between pulses within same symbol: TOGGLE (line 1374)
                ear ^= 1
                emit(tape_next)

    pilot_count = len(pulses)
    print(f"  After pilot: {pilot_count} pulses, ear={ear}")

    # --- Read data symbol definition table ---
    data_symdefs = []
    for i in range(asd):
        flags = d[pos]; pos += 1
        pls = []
        for j in range(npd):
            pls.append(struct.unpack_from('<H', d, pos)[0]); pos += 2
        data_symdefs.append((flags, pls))
        print(f"    Data sym[{i}]: flags={flags} pulses={pls}")

    # --- Simulate GDB_DATA ---
    cur_bit = 7
    cur_byte = d[pos]; pos += 1

    # Read first nb-bit symbol
    gdb_symbol = 0
    for i in range(nb):
        gdb_symbol <<= 1
        gdb_symbol |= (cur_byte >> cur_bit) & 1
        if cur_bit == 0:
            cur_byte = d[pos]; pos += 1
            cur_bit = 7
        else:
            cur_bit -= 1

    cur_gdb_symbol = 0

    apply_flags(data_symdefs[gdb_symbol][0])
    tape_next = data_symdefs[gdb_symbol][1][0]
    cur_pulse = 0

    emit(tape_next)

    while len(pulses) < max_pulses and cur_gdb_symbol < totd:
        cur_pulse += 1
        if cur_pulse < npd:
            tape_next = data_symdefs[gdb_symbol][1][cur_pulse]

        if cur_pulse == npd or tape_next == 0:
            cur_gdb_symbol += 1
            if cur_gdb_symbol >= totd:
                break

            gdb_symbol = 0
            for i in range(nb):
                gdb_symbol <<= 1
                gdb_symbol |= (cur_byte >> cur_bit) & 1
                if cur_bit == 0:
                    if pos < len(d):
                        cur_byte = d[pos]; pos += 1
                    cur_bit = 7
                else:
                    cur_bit -= 1

            apply_flags(data_symdefs[gdb_symbol][0])
            tape_next = data_symdefs[gdb_symbol][1][0]
            cur_pulse = 0

            emit(tape_next)
        else:
            # Between pulses within same symbol: TOGGLE (line 1463)
            ear ^= 1
            emit(tape_next)

    return pulses, pilot_count


# ============================================================
# PZX parsing
# ============================================================

def parse_pzx(path):
    """Parse PZX file and return list of (tag_str, offset, raw_block_data, size)."""
    data = Path(path).read_bytes()
    pos = 0
    blocks = []
    while pos + 8 <= len(data):
        tag = data[pos:pos+4].decode('ascii', errors='replace')
        size = struct.unpack_from('<I', data, pos + 4)[0]
        block_data = data[pos + 8:pos + 8 + size]
        blocks.append((tag, pos, block_data, size))
        pos += 8 + size
    return blocks


def simulate_pzx_puls(block_data):
    """
    Simulate PZX PULS block matching Tape_PZX.cpp.
    Returns (list of (ear, duration), final_ear).
    """
    d = block_data
    pos = 0
    ear = 0  # PULS block always sets ear=0

    # Parse all entries
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

    print(f"  PZX PULS entries: {entries}")

    pulses = []

    # GetBlock: skip zero-dur entries, find first non-zero
    entry_idx = 0
    pzx_rep = 0
    pzx_dur = 0

    while entry_idx < len(entries):
        rep, dur = entries[entry_idx]
        entry_idx += 1
        if dur == 0:
            if rep & 1:
                ear ^= 1
            continue
        pzx_rep = rep
        pzx_dur = dur
        break

    if pzx_rep == 0:
        return pulses, ear

    # Emit first pulse from GetBlock
    pulses.append((ear, pzx_dur))

    # Simulate Read() calls
    while True:
        # Read() PZX_PULS: toggle first, then decrement
        ear ^= 1
        pzx_rep -= 1

        while pzx_rep == 0:
            if entry_idx >= len(entries):
                # End of PULS block
                return pulses, ear
            rep, dur = entries[entry_idx]
            entry_idx += 1
            if dur == 0:
                if rep & 1:
                    ear ^= 1
                continue
            pzx_rep = rep
            pzx_dur = dur

        pulses.append((ear, pzx_dur))

    return pulses, ear


def simulate_pzx_data(block_data, initial_ear, max_pulses=MAX_PULSES):
    """
    Simulate PZX DATA block (p0=2, p1=2 standard path) matching Tape_PZX.cpp.
    Returns list of (ear, duration).
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
    for i in range(p0):
        s0.append(struct.unpack_from('<H', d, pos)[0]); pos += 2
    s1 = []
    for i in range(p1):
        s1.append(struct.unpack_from('<H', d, pos)[0]); pos += 2

    print(f"  PZX DATA: bits={bit_count} tail={tail_len} p0={p0} p1={p1} s0={s0} s1={s1} initial_level={initial_level}")

    data_bytes = d[pos:]

    # GetBlock sets tapeEarBit = initialLevel
    ear = initial_level

    pulses = []

    if p0 != 2 or p1 != 2 or bit_count == 0:
        print("  WARNING: Non-standard p0/p1 or zero bits")
        return pulses

    bit0_len = s0[0]
    bit1_len = s1[0]

    block_len = (bit_count + 7) // 8
    last_bits = bit_count & 7
    last_byte_used_bits = last_bits if last_bits else 8

    byte_idx = 0
    cur_byte = data_bytes[byte_idx]; byte_idx += 1
    bit_mask = 0x80
    end_bit_mask = 0x80
    if block_len == 1 and last_byte_used_bits < 8:
        end_bit_mask >>= last_byte_used_bits

    buf_byte_count = 0

    # GetBlock: tapePhase=DATA1, tapeNext based on first bit
    tape_next = bit1_len if (cur_byte & bit_mask) else bit0_len

    # First pulse from GetBlock
    pulses.append((ear, tape_next))

    while len(pulses) < max_pulses:
        # DATA1: toggle ear, phase -> DATA2 (tapeNext unchanged)
        ear ^= 1
        pulses.append((ear, tape_next))

        # DATA2: toggle ear, advance bit
        ear ^= 1
        bit_mask = ((bit_mask >> 1) | (bit_mask << 7)) & 0xFF

        if bit_mask == end_bit_mask:
            buf_byte_count += 1
            if buf_byte_count >= block_len:
                break
            if byte_idx < len(data_bytes):
                cur_byte = data_bytes[byte_idx]; byte_idx += 1
            else:
                break
            if buf_byte_count + 1 >= block_len:
                end_bit_mask = (0x80 >> last_byte_used_bits) if last_byte_used_bits < 8 else 0x80
            else:
                end_bit_mask = 0x80

        tape_next = bit1_len if (cur_byte & bit_mask) else bit0_len
        pulses.append((ear, tape_next))

    return pulses


# ============================================================
# Main
# ============================================================

def main():
    tzx_path = "/home/drew/github/pico-spec/debug/Dan Dare 2 - Mekon's Revenge.tzx"
    pzx_path = "/home/drew/github/pico-spec/debug/Dan Dare 2 - Mekon's Revenge.pzx"

    # --- TZX ---
    print("=" * 80)
    print("TZX PARSING")
    print("=" * 80)
    tzx_blocks = parse_tzx(tzx_path)

    print(f"\nTotal TZX blocks: {len(tzx_blocks)}")
    for i, (bid, offset, bdata) in enumerate(tzx_blocks):
        extra = ""
        if bid == 0x19:
            extra = f" (GDB, data_len={len(bdata)})"
        elif bid == 0x10:
            extra = f" (Standard)"
        print(f"  Block {i}: ID=0x{bid:02X} offset={offset} size={len(bdata)}{extra}")

    # Find first GDB block
    gdb_block = None
    gdb_idx = -1
    for i, (bid, offset, bdata) in enumerate(tzx_blocks):
        if bid == 0x19:
            gdb_block = bdata
            gdb_idx = i
            break

    if gdb_block is None:
        print("ERROR: No GDB block found in TZX!")
        return

    print(f"\nFirst GDB block at TZX index {gdb_idx}, size {len(gdb_block)} bytes")
    print(f"\nSimulating TZX GDB:")
    tzx_pulses, tzx_pilot_count = simulate_tzx_gdb(gdb_block, initial_ear=0, max_pulses=MAX_PULSES)

    # --- PZX ---
    print()
    print("=" * 80)
    print("PZX PARSING")
    print("=" * 80)
    pzx_blocks = parse_pzx(pzx_path)

    print(f"\nTotal PZX blocks: {len(pzx_blocks)}")
    for i, (tag, offset, bdata, size) in enumerate(pzx_blocks):
        print(f"  Block {i}: {tag} offset={offset} sz={size}")

    # Find the PULS+DATA pair for the turbo block
    puls_idx = data_idx = None
    for i, (tag, offset, bdata, size) in enumerate(pzx_blocks):
        if tag == 'PULS' and size == 40 and puls_idx is None:
            puls_idx = i
        if tag == 'DATA' and size == 5552 and data_idx is None:
            data_idx = i

    if puls_idx is None:
        print("PULS sz=40 not found, searching for first PULS after standard blocks...")
        for i, (tag, offset, bdata, size) in enumerate(pzx_blocks):
            if tag == 'PULS' and i >= 4:
                puls_idx = i
                break
    if data_idx is None and puls_idx is not None:
        data_idx = puls_idx + 1

    print(f"\nUsing PULS block idx={puls_idx}: {pzx_blocks[puls_idx][0]} sz={pzx_blocks[puls_idx][3]}")
    print(f"Using DATA block idx={data_idx}: {pzx_blocks[data_idx][0]} sz={pzx_blocks[data_idx][3]}")

    print(f"\nSimulating PZX PULS:")
    pzx_puls_pulses, ear_after_puls = simulate_pzx_puls(pzx_blocks[puls_idx][2])
    print(f"  PULS produced {len(pzx_puls_pulses)} pulses, ear after PULS={ear_after_puls}")

    print(f"\nSimulating PZX DATA:")
    remaining = MAX_PULSES - len(pzx_puls_pulses)
    pzx_data_pulses = simulate_pzx_data(pzx_blocks[data_idx][2], initial_ear=ear_after_puls, max_pulses=max(remaining, 40))

    pzx_pulses = pzx_puls_pulses + pzx_data_pulses
    pzx_pilot_count = len(pzx_puls_pulses)

    # --- Comparison ---
    print()
    print("=" * 80)
    print("PULSE COMPARISON")
    print("=" * 80)
    print()
    n = min(len(tzx_pulses), len(pzx_pulses), MAX_PULSES)
    print(f"{'#':>4}  {'TZX ear':>7} {'TZX dur':>8}  {'PZX ear':>7} {'PZX dur':>8}  {'Match':>5}  Note")
    print("-" * 75)

    mismatches = 0
    for i in range(n):
        te, td = tzx_pulses[i] if i < len(tzx_pulses) else ('-', '-')
        pe, pd = pzx_pulses[i] if i < len(pzx_pulses) else ('-', '-')

        if isinstance(te, int) and isinstance(pe, int):
            match = "YES" if te == pe and td == pd else "NO"
        else:
            match = "N/A"

        if match == "NO":
            mismatches += 1

        note = ""
        if i == tzx_pilot_count:
            note += " <-- TZX pilot->data"
        if i == pzx_pilot_count:
            note += " <-- PZX pilot->data"

        print(f"{i:4}  {str(te):>7} {str(td):>8}  {str(pe):>7} {str(pd):>8}  {match:>5}  {note}")

    print()
    print(f"Total mismatches in first {n} pulses: {mismatches}")
    print(f"TZX pilot pulses: {tzx_pilot_count}")
    print(f"PZX pilot pulses: {pzx_pilot_count}")

    # --- Waveform summary ---
    print()
    print("=" * 80)
    print("WAVEFORM PATTERN ANALYSIS (data section)")
    print("=" * 80)

    # Show pattern around pilot->data transition for both
    for name, pulse_list, pilot_n in [("TZX", tzx_pulses, tzx_pilot_count),
                                       ("PZX", pzx_pulses, pzx_pilot_count)]:
        start = max(0, pilot_n - 4)
        end = min(len(pulse_list), pilot_n + 12)
        print(f"\n{name} around pilot->data transition (pilot ends at index {pilot_n}):")
        for i in range(start, end):
            marker = " <<< DATA START" if i == pilot_n else ""
            print(f"  [{i:3}] ear={pulse_list[i][0]} dur={pulse_list[i][1]}{marker}")

    # Show first few data bytes
    print()
    print("=" * 80)
    print("DATA BYTE COMPARISON")
    print("=" * 80)

    d = gdb_block
    pos = 4 + 2  # skip length + pause
    totp = struct.unpack_from('<I', d, pos)[0]; pos += 4
    npp = d[pos]; pos += 1
    asp = d[pos]; pos += 1
    if asp == 0: asp = 256
    totd = struct.unpack_from('<I', d, pos)[0]; pos += 4
    npd = d[pos]; pos += 1
    asd = d[pos]; pos += 1
    if asd == 0: asd = 256
    pos += asp * (1 + npp * 2)  # skip pilot symdefs
    pos += totp * 3             # skip PRLE
    pos += asd * (1 + npd * 2)  # skip data symdefs

    tzx_data = d[pos:pos + 16]
    print(f"TZX GDB data (first 16 bytes): {' '.join(f'{b:02X}' for b in tzx_data)}")

    pd = pzx_blocks[data_idx][2]
    pp = 4 + 2 + 1 + 1  # count + tail + p0 + p1
    p0_n = pd[6]; p1_n = pd[7]
    pp += p0_n * 2 + p1_n * 2
    pzx_data = pd[pp:pp + 16]
    print(f"PZX DATA data (first 16 bytes): {' '.join(f'{b:02X}' for b in pzx_data)}")
    print(f"Data bytes match: {tzx_data == pzx_data}")

    # --- Key finding summary ---
    print()
    print("=" * 80)
    print("KEY FINDINGS")
    print("=" * 80)

    # Check if durations match but polarity is inverted
    dur_match = 0
    pol_inverted = 0
    exact_match = 0
    n_compare = min(len(tzx_pulses), len(pzx_pulses))
    for i in range(n_compare):
        te, td = tzx_pulses[i]
        pe, pd = pzx_pulses[i]
        if td == pd:
            dur_match += 1
        if te != pe and td == pd:
            pol_inverted += 1
        if te == pe and td == pd:
            exact_match += 1

    print(f"\n  Total pulses compared: {n_compare}")
    print(f"  Duration matches: {dur_match} ({100*dur_match//n_compare}%)")
    print(f"  Exact matches (ear+dur): {exact_match}")
    print(f"  Polarity-inverted (same dur, opposite ear): {pol_inverted}")

    if pol_inverted == n_compare:
        print(f"\n  *** ALL pulses have IDENTICAL durations but INVERTED polarity! ***")
        print(f"\n  ROOT CAUSE:")
        print(f"  TZX GDB pilot sym[0] has flag=0 (toggle), so first PRLE entry")
        print(f"  toggles ear from 0->1 before first pilot pulse.")
        print(f"  First TZX pilot pulse: ear=1, dur=2168")
        print(f"")
        print(f"  PZX PULS block always sets ear=0 at GetBlock entry (line 219: tapeEarBit = 0)")
        print(f"  First PZX pilot pulse: ear=0, dur=2168")
        print(f"")
        print(f"  This 1-bit polarity offset propagates through the entire waveform.")
        print(f"  The waveform SHAPES are identical; only the absolute polarity differs.")
        print(f"")
        print(f"  For the Speedlock loader, this should NOT matter because turbo loaders")
        print(f"  detect edges (transitions), not absolute levels. The edge timing is")
        print(f"  identical between TZX and PZX.")
    elif dur_match == n_compare:
        print(f"\n  All durations match. Polarity differences exist but pattern is consistent.")
    else:
        print(f"\n  There are duration differences - this indicates a real waveform mismatch!")

    # Show data section pattern
    if tzx_pilot_count < len(tzx_pulses) and pzx_pilot_count < len(pzx_pulses):
        print(f"\n  Data section first 3 pulses:")
        for name, plist, pc in [("TZX", tzx_pulses, tzx_pilot_count),
                                 ("PZX", pzx_pulses, pzx_pilot_count)]:
            bits = []
            for j in range(pc, min(pc + 6, len(plist))):
                bits.append(f"ear={plist[j][0]} dur={plist[j][1]}")
            print(f"    {name}: {' | '.join(bits)}")

        print(f"\n  Toggle pattern (data section, first 12 transitions):")
        for name, plist, pc in [("TZX", tzx_pulses, tzx_pilot_count),
                                 ("PZX", pzx_pulses, pzx_pilot_count)]:
            toggles = []
            for j in range(pc, min(pc + 12, len(plist) - 1)):
                t = "T" if plist[j][0] != plist[j+1][0] else "="
                toggles.append(t)
            print(f"    {name}: {' '.join(toggles)}")

        print(f"\n  Both formats toggle at every pulse boundary in the data section.")
        print(f"  TZX: flag=0 toggle at symbol start + inter-pulse toggle = alternating per pulse")
        print(f"  PZX: DATA1 toggle + DATA2 toggle = alternating per pulse")


if __name__ == '__main__':
    main()
