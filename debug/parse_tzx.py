#!/usr/bin/env python3
"""Parse and dump TZX file structure."""

import struct
import sys
import math

def read_u8(f):
    return struct.unpack('<B', f.read(1))[0]

def read_u16(f):
    return struct.unpack('<H', f.read(2))[0]

def read_u24(f):
    b = f.read(3)
    return b[0] | (b[1] << 8) | (b[2] << 16)

def read_u32(f):
    return struct.unpack('<I', f.read(4))[0]

BLOCK_NAMES = {
    0x10: "Standard Speed Data",
    0x11: "Turbo Speed Data",
    0x12: "Pure Tone",
    0x13: "Pulse Sequence",
    0x14: "Pure Data",
    0x15: "Direct Recording",
    0x18: "CSW Recording",
    0x19: "Generalized Data Block",
    0x20: "Pause/Stop",
    0x21: "Group Start",
    0x22: "Group End",
    0x23: "Jump To Block",
    0x24: "Loop Start",
    0x25: "Loop End",
    0x26: "Call Sequence",
    0x27: "Return from Sequence",
    0x28: "Select Block",
    0x2A: "Stop Tape (48K)",
    0x2B: "Set Signal Level",
    0x30: "Text Description",
    0x31: "Message",
    0x32: "Archive Info",
    0x33: "Hardware Type",
    0x34: "Emulation Info",
    0x35: "Custom Info",
    0x5A: "Glue Block",
}

GDB_SYM_FLAGS = {0: "opposite edge", 1: "same edge", 2: "force high", 3: "force low"}

def describe_flag_byte(flag):
    if flag == 0x00:
        return "header"
    elif flag == 0xFF:
        return "data"
    else:
        return f"0x{flag:02X}"

def describe_header_type(t):
    return {0: "Program", 1: "Number array", 2: "Character array", 3: "Code"}.get(t, f"Unknown({t})")

def parse_standard_header(data):
    if len(data) < 19 or data[0] != 0x00:
        return None
    htype = data[1]
    name = data[2:12].decode('ascii', errors='replace').rstrip()
    data_len = data[12] | (data[13] << 8)
    param1 = data[14] | (data[15] << 8)
    param2 = data[16] | (data[17] << 8)
    return {'type': describe_header_type(htype), 'name': name,
            'data_len': data_len, 'param1': param1, 'param2': param2, 'htype': htype}

def parse_tzx(filename):
    blocks = []
    with open(filename, 'rb') as f:
        sig = f.read(7)
        assert sig == b'ZXTape!', f"Bad signature: {sig}"
        eof = f.read(1)
        assert eof == b'\x1a'
        ver_major = read_u8(f)
        ver_minor = read_u8(f)

        import os
        fsize = os.path.getsize(filename)
        print(f"TZX Version: {ver_major}.{ver_minor}")
        print(f"File size: {fsize} bytes")
        print("=" * 80)

        block_num = 0
        while True:
            pos = f.tell()
            id_byte = f.read(1)
            if not id_byte:
                break
            block_id = id_byte[0]
            block_name = BLOCK_NAMES.get(block_id, f"Unknown(0x{block_id:02X})")
            block_num += 1

            print(f"\nBlock #{block_num:3d} @ 0x{pos:05X}: ID=0x{block_id:02X} ({block_name})")

            if block_id == 0x10:
                pause_ms = read_u16(f)
                data_len = read_u16(f)
                data = f.read(data_len)
                flag = data[0] if data else 0
                print(f"  pause={pause_ms}ms, data_len={data_len}, flag={describe_flag_byte(flag)}")
                if flag == 0x00 and data_len == 19:
                    hdr = parse_standard_header(data)
                    if hdr:
                        print(f"  -> {hdr['type']}: \"{hdr['name']}\" len={hdr['data_len']}"
                              f" param1={hdr['param1']} param2=0x{hdr['param2']:04X}")
                elif flag == 0xFF:
                    print(f"  -> Data block: {data_len - 2} payload bytes")
                blocks.append(('standard', block_id, data_len, pause_ms))

            elif block_id == 0x11:
                pilot_pulse = read_u16(f)
                sync1 = read_u16(f)
                sync2 = read_u16(f)
                bit0 = read_u16(f)
                bit1 = read_u16(f)
                pilot_len = read_u16(f)
                used_bits = read_u8(f)
                pause_ms = read_u16(f)
                data_len = read_u24(f)
                data = f.read(data_len)
                flag = data[0] if data else 0
                print(f"  pilot={pilot_pulse}T x{pilot_len}, sync={sync1}/{sync2}T")
                print(f"  bit0={bit0}T, bit1={bit1}T, used_bits_last={used_bits}")
                print(f"  pause={pause_ms}ms, data_len={data_len}, flag={describe_flag_byte(flag)}")
                if flag == 0x00 and data_len == 19:
                    hdr = parse_standard_header(data)
                    if hdr:
                        print(f"  -> {hdr['type']}: \"{hdr['name']}\" len={hdr['data_len']}")
                elif flag == 0xFF:
                    print(f"  -> Data block: {data_len - 2} payload bytes")
                blocks.append(('turbo', block_id, data_len, pause_ms))

            elif block_id == 0x12:
                pulse_len = read_u16(f)
                num_pulses = read_u16(f)
                print(f"  pulse_len={pulse_len}T, num_pulses={num_pulses}")
                blocks.append(('pure_tone', block_id, 0, 0))

            elif block_id == 0x13:
                num_pulses = read_u8(f)
                pulses = [read_u16(f) for _ in range(num_pulses)]
                print(f"  num_pulses={num_pulses}, pulses={pulses}")
                blocks.append(('pulse_seq', block_id, 0, 0))

            elif block_id == 0x14:
                bit0 = read_u16(f)
                bit1 = read_u16(f)
                used_bits = read_u8(f)
                pause_ms = read_u16(f)
                data_len = read_u24(f)
                f.read(data_len)
                print(f"  bit0={bit0}T, bit1={bit1}T, used_bits_last={used_bits}")
                print(f"  pause={pause_ms}ms, data_len={data_len}")
                blocks.append(('pure_data', block_id, data_len, pause_ms))

            elif block_id == 0x15:
                tstates = read_u16(f)
                pause_ms = read_u16(f)
                used_bits = read_u8(f)
                data_len = read_u24(f)
                f.read(data_len)
                print(f"  tstates/sample={tstates}, pause={pause_ms}ms, data_len={data_len}")
                blocks.append(('direct', block_id, data_len, pause_ms))

            elif block_id == 0x19:
                block_len = read_u32(f)
                block_start = f.tell()
                pause_ms = read_u16(f)
                totp = read_u32(f)    # number of PRLE entries in pilot table
                npp = read_u8(f)      # max pulses per pilot symbol
                asp = read_u8(f)      # pilot alphabet size (BYTE)
                totd = read_u32(f)    # total number of data symbols
                npd = read_u8(f)      # max pulses per data symbol
                asd = read_u8(f)      # data alphabet size (BYTE)

                print(f"  block_len={block_len}, pause={pause_ms}ms")
                print(f"  TOTP={totp} (pilot PRLE entries), NPP={npp}, ASP={asp}")
                print(f"  TOTD={totd} (total data symbols), NPD={npd}, ASD={asd}")

                # Pilot symbol definitions
                pilot_symdefs = []
                if asp > 0 and npp > 0:
                    print(f"  Pilot symbols ({asp}):")
                    for i in range(asp):
                        flags = read_u8(f)
                        pulses = [read_u16(f) for _ in range(npp)]
                        pilot_symdefs.append((flags, pulses))
                        print(f"    [{i}] flags={flags} ({GDB_SYM_FLAGS.get(flags,'?')}) pulses={pulses}")

                # Pilot PRLE table
                pilot_prle_details = []
                if totp > 0:
                    print(f"  Pilot PRLE ({totp} entries):")
                    total_reps = 0
                    for _ in range(totp):
                        sym = read_u8(f)
                        reps = read_u16(f)
                        total_reps += reps
                        sf, sp = pilot_symdefs[sym] if sym < len(pilot_symdefs) else (0, [])
                        pilot_prle_details.append((sym, reps, sp, sf))
                        print(f"    sym={sym} x{reps}")
                    print(f"    Total pilot/sync pulses: {total_reps}")

                # Data symbol definitions
                if asd > 0 and npd > 0:
                    print(f"  Data symbols ({asd}):")
                    for i in range(asd):
                        flags = read_u8(f)
                        pulses = [read_u16(f) for _ in range(npd)]
                        print(f"    [{i}] flags={flags} ({GDB_SYM_FLAGS.get(flags,'?')}) pulses={pulses}")

                # Simulate pilot playback
                if totp > 0 and asp > 0:
                    print(f"  --- Pilot playback simulation ---")
                    level = 0  # start low
                    total_t = 0
                    for prle_sym, prle_reps, prle_pulses, prle_flags in pilot_prle_details:
                        for rep in range(prle_reps):
                            flag = prle_flags
                            if flag == 0:  # opposite edge: toggle before first pulse
                                level = 1 - level
                            elif flag == 2:  # force high
                                level = 1
                            elif flag == 3:  # force low
                                level = 0
                            # flag==1: same edge, no change
                            for pi, p in enumerate(prle_pulses):
                                if p == 0:
                                    continue
                                total_t += p
                                if pi < len(prle_pulses) - 1:
                                    # After each pulse except last in symbol, toggle
                                    level = 1 - level
                            # After last pulse of symbol, do NOT toggle (next symbol start will handle it)
                    print(f"    Final level after pilot: {'HIGH' if level else 'LOW'}")
                    print(f"    Total pilot T-states: {total_t}")
                    # Show waveform summary
                    print(f"  --- Pilot waveform detail ---")
                    level = 0
                    for prle_sym, prle_reps, prle_pulses, prle_flags in pilot_prle_details:
                        flag = prle_flags
                        flag_name = GDB_SYM_FLAGS.get(flag, '?')
                        sym_desc = f"sym[{prle_sym}] x{prle_reps} flag={flag}({flag_name}) pulses={prle_pulses}"
                        # Show first iteration
                        edges = []
                        save_level = level
                        for rep in range(min(prle_reps, 2)):
                            if flag == 0:
                                level = 1 - level
                            elif flag == 2:
                                level = 1
                            elif flag == 3:
                                level = 0
                            for pi, p in enumerate(prle_pulses):
                                if p == 0:
                                    edges.append(f"{'H' if level else 'L'}(0T=skip)")
                                    continue
                                edges.append(f"{'H' if level else 'L'}({p}T)")
                                if pi < len(prle_pulses) - 1:
                                    level = 1 - level
                        level_for_all = save_level
                        # Restore and compute for all reps
                        for rep in range(prle_reps):
                            if flag == 0:
                                level_for_all = 1 - level_for_all
                            elif flag == 2:
                                level_for_all = 1
                            elif flag == 3:
                                level_for_all = 0
                            for pi, p in enumerate(prle_pulses):
                                if pi < len(prle_pulses) - 1 and p > 0:
                                    level_for_all = 1 - level_for_all
                        level = level_for_all
                        edge_str = " -> ".join(edges[:8])
                        if prle_reps > 2:
                            edge_str += " -> ..."
                        print(f"    {sym_desc}")
                        print(f"      {edge_str}")

                # Data stream
                data_end = block_start + block_len
                stream_len = data_end - f.tell()
                if stream_len > 0:
                    f.read(stream_len)
                    if asd > 1:
                        bits = math.ceil(math.log2(asd))
                        data_bytes = totd // (8 // bits) if bits <= 8 else totd
                        print(f"  Data stream: {stream_len} bytes ({totd} symbols, {bits} bit/sym, ~{data_bytes} payload bytes)")
                    else:
                        print(f"  Data stream: {stream_len} bytes")

                blocks.append(('gdb', block_id, block_len, pause_ms))

            elif block_id == 0x20:
                pause_ms = read_u16(f)
                print(f"  {'STOP THE TAPE' if pause_ms == 0 else f'pause={pause_ms}ms'}")
                blocks.append(('pause', block_id, 0, pause_ms))

            elif block_id == 0x21:
                name_len = read_u8(f)
                name = f.read(name_len).decode('ascii', errors='replace')
                print(f"  \"{name}\"")
                blocks.append(('group_start', block_id, 0, 0))

            elif block_id == 0x22:
                blocks.append(('group_end', block_id, 0, 0))

            elif block_id == 0x23:
                offset = struct.unpack('<h', f.read(2))[0]
                print(f"  offset={offset}")
                blocks.append(('jump', block_id, 0, 0))

            elif block_id == 0x24:
                count = read_u16(f)
                print(f"  count={count}")
                blocks.append(('loop_start', block_id, 0, 0))

            elif block_id == 0x25:
                blocks.append(('loop_end', block_id, 0, 0))

            elif block_id == 0x26:
                num = read_u16(f)
                offsets = [struct.unpack('<h', f.read(2))[0] for _ in range(num)]
                print(f"  calls={num}, offsets={offsets}")
                blocks.append(('call', block_id, 0, 0))

            elif block_id == 0x27:
                blocks.append(('return', block_id, 0, 0))

            elif block_id == 0x28:
                length = read_u16(f)
                f.read(length)
                print(f"  length={length}")
                blocks.append(('select', block_id, 0, 0))

            elif block_id == 0x2A:
                length = read_u32(f)
                f.read(length)
                blocks.append(('stop48k', block_id, 0, 0))

            elif block_id == 0x2B:
                length = read_u32(f)
                f.read(length)
                blocks.append(('signal_level', block_id, 0, 0))

            elif block_id == 0x30:
                text_len = read_u8(f)
                text = f.read(text_len).decode('ascii', errors='replace')
                print(f"  \"{text}\"")
                blocks.append(('text', block_id, 0, 0))

            elif block_id == 0x31:
                time = read_u8(f)
                msg_len = read_u8(f)
                msg = f.read(msg_len).decode('ascii', errors='replace')
                print(f"  time={time}s \"{msg}\"")
                blocks.append(('message', block_id, 0, 0))

            elif block_id == 0x32:
                length = read_u16(f)
                num_strings = read_u8(f)
                info_ids = {0: "Title", 1: "Publisher", 2: "Author", 3: "Year",
                           4: "Language", 5: "Type", 6: "Price", 7: "Protection",
                           8: "Origin", 0xFF: "Comment"}
                print(f"  {num_strings} entries:")
                for _ in range(num_strings):
                    tid = read_u8(f)
                    tlen = read_u8(f)
                    ttext = f.read(tlen).decode('ascii', errors='replace')
                    print(f"    {info_ids.get(tid, f'ID=0x{tid:02X}')}: \"{ttext}\"")
                blocks.append(('archive', block_id, 0, 0))

            elif block_id == 0x33:
                num = read_u8(f)
                f.read(num * 3)
                print(f"  {num} hardware entries")
                blocks.append(('hardware', block_id, 0, 0))

            elif block_id == 0x35:
                ident = f.read(16).decode('ascii', errors='replace').rstrip('\x00')
                length = read_u32(f)
                f.read(length)
                print(f"  id=\"{ident}\", length={length}")
                blocks.append(('custom', block_id, 0, 0))

            elif block_id == 0x5A:
                f.read(9)
                blocks.append(('glue', block_id, 0, 0))

            else:
                print(f"  *** UNKNOWN BLOCK ID 0x{block_id:02X} ***")
                length = read_u32(f)
                print(f"  Skipping {length} bytes")
                f.read(length)
                blocks.append(('unknown', block_id, length, 0))

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"Total TZX blocks: {len(blocks)}")
    print()

    type_counts = {}
    for b in blocks:
        name = BLOCK_NAMES.get(b[1], f"0x{b[1]:02X}")
        type_counts[name] = type_counts.get(name, 0) + 1
    print("Block type counts:")
    for name, count in sorted(type_counts.items()):
        print(f"  {name}: {count}")

    data_blocks = [b for b in blocks if b[0] in ('standard', 'turbo', 'pure_data', 'gdb')]
    total_data = sum(b[2] for b in data_blocks)
    print(f"\nData-carrying blocks: {len(data_blocks)}")
    print(f"Total data bytes (approx): {total_data}")

    # PZX mapping
    print("\n" + "=" * 80)
    print("PZX COMPARISON (target: 37 PZX blocks)")
    print("=" * 80)
    print("""
PZX block types and TZX equivalents:
  PZXT (info)     <- TZX 0x30/0x32 (Text/Archive)
  PULS (pulses)   <- TZX 0x12/0x13 (Pure Tone/Pulse Seq), or pilot portion of 0x10/0x11/0x19
  DATA (data)     <- TZX 0x10/0x11/0x14/0x19 data portion
  PAUS (pause)    <- TZX 0x20, or trailing pause in 0x10/0x11/0x14/0x19
  STOP (stop)     <- TZX 0x20 (pause=0) or 0x2A
  BRWS (browse)   <- TZX 0x21 (Group Start)
""")

    # Estimate PZX block count
    pzx = []
    for b in blocks:
        t, bid = b[0], b[1]
        if t == 'standard':
            # Standard -> PULS (pilot 8063x2168 + sync 667+735) + DATA + optional PAUS
            pzx.append('PULS')
            pzx.append('DATA')
            if b[3] > 0:
                pzx.append('PAUS')
        elif t == 'turbo':
            pzx.append('PULS')
            pzx.append('DATA')
            if b[3] > 0:
                pzx.append('PAUS')
        elif t == 'pure_data':
            pzx.append('DATA')
            if b[3] > 0:
                pzx.append('PAUS')
        elif t == 'gdb':
            pzx.append('PULS')
            pzx.append('DATA')
            if b[3] > 0:
                pzx.append('PAUS')
        elif bid == 0x12:
            pzx.append('PULS')
        elif bid == 0x13:
            pzx.append('PULS')
        elif bid == 0x20:
            if b[3] == 0:
                pzx.append('STOP')
            else:
                pzx.append('PAUS')
        elif bid == 0x32 or bid == 0x30:
            pzx.append('PZXT')
        elif bid == 0x21:
            pzx.append('BRWS')

    print(f"Estimated PZX blocks from TZX: {len(pzx)}")
    pcounts = {}
    for p in pzx:
        pcounts[p] = pcounts.get(p, 0) + 1
    for name, count in sorted(pcounts.items()):
        print(f"  {name}: {count}")

    print(f"\nNote: Actual PZX conversion may differ - consecutive PULS blocks can be")
    print(f"merged, Group Start/End are metadata-only, and standard blocks expand to")
    print(f"PULS+DATA+PAUS. A converter may also add initial PZXT or merge pauses.")


if __name__ == '__main__':
    parse_tzx(sys.argv[1] if len(sys.argv) > 1
              else "/home/drew/github/pico-spec/debug/Dan Dare 2 - Mekon's Revenge.tzx")
