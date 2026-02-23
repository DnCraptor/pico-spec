#!/usr/bin/env python3
"""
Z80 disassembler — standalone, no external dependencies.

Supports:
  - ZX Spectrum TAP files (auto-parses blocks, headers, BASIC)
  - Raw binary files (specify --org for base address)

Usage:
  z80disasm.py input.tap                     # auto-parse TAP, write input_disasm.txt
  z80disasm.py input.tap -o output.txt       # specify output file
  z80disasm.py code.bin --org 0x8000         # raw binary at address 0x8000
  z80disasm.py code.bin --org 0x8000 --hex   # include hex dump for non-code areas
  z80disasm.py input.tap --block 5           # disassemble only TAP block 5
"""

import struct
import sys
import argparse
import os

# ============================================================
# Z80 Instruction Tables
# ============================================================

CC = ['NZ', 'Z', 'NC', 'C', 'PO', 'PE', 'P', 'M']
R8 = ['B', 'C', 'D', 'E', 'H', 'L', '(HL)', 'A']
RP = ['BC', 'DE', 'HL', 'SP']
RP2 = ['BC', 'DE', 'HL', 'AF']
ALU = ['ADD A,', 'ADC A,', 'SUB', 'SBC A,', 'AND', 'XOR', 'OR', 'CP']
ROT = ['RLC', 'RRC', 'RL', 'RR', 'SLA', 'SRA', 'SLL', 'SRL']


def read_byte(code, offset):
    if offset >= len(code):
        return None, offset
    return code[offset], offset + 1


def read_word(code, offset):
    if offset + 1 >= len(code):
        return None, offset
    val = code[offset] | (code[offset + 1] << 8)
    return val, offset + 2


def signed8(val):
    return val - 256 if val >= 128 else val


def fmt_hex8(val):
    return f"0x{val:02X}"


def fmt_hex16(val):
    return f"0x{val:04X}"


def fmt_displacement(d):
    return f"+0x{d:02X}" if d >= 0 else f"-0x{(-d):02X}"


# ============================================================
# CB prefix (bit operations, rotates, shifts)
# ============================================================

def disassemble_cb(code, offset, base_addr):
    b, offset = read_byte(code, offset)
    if b is None:
        return "DB 0xCB", offset, 1
    x, y, z = (b >> 6) & 3, (b >> 3) & 7, b & 7
    if x == 0:
        mnemonic = f"{ROT[y]} {R8[z]}"
    elif x == 1:
        mnemonic = f"BIT {y}, {R8[z]}"
    elif x == 2:
        mnemonic = f"RES {y}, {R8[z]}"
    else:
        mnemonic = f"SET {y}, {R8[z]}"
    return mnemonic, offset, 1


# ============================================================
# DD CB / FD CB prefix (indexed bit operations)
# ============================================================

def disassemble_ddcb_fdcb(code, offset, base_addr, reg):
    d, offset = read_byte(code, offset)
    if d is None:
        return f"DB 0x{'DD' if reg == 'IX' else 'FD'}, 0xCB", offset, 2
    ds = signed8(d)
    disp = fmt_displacement(ds)
    b, offset = read_byte(code, offset)
    if b is None:
        return f"DB 0x{'DD' if reg == 'IX' else 'FD'}, 0xCB, {fmt_hex8(d)}", offset, 3
    x, y, z = (b >> 6) & 3, (b >> 3) & 7, b & 7
    idx = f"({reg}{disp})"
    if x == 0:
        mnemonic = f"{ROT[y]} {idx}" if z == 6 else f"{ROT[y]} {idx}, {R8[z]}"
    elif x == 1:
        mnemonic = f"BIT {y}, {idx}"
    elif x == 2:
        mnemonic = f"RES {y}, {idx}" if z == 6 else f"RES {y}, {idx}, {R8[z]}"
    else:
        mnemonic = f"SET {y}, {idx}" if z == 6 else f"SET {y}, {idx}, {R8[z]}"
    return mnemonic, offset, 2


# ============================================================
# ED prefix
# ============================================================

def disassemble_ed(code, offset, base_addr):
    b, offset = read_byte(code, offset)
    if b is None:
        return "DB 0xED", offset, 1
    x, y, z = (b >> 6) & 3, (b >> 3) & 7, b & 7
    p, q = (y >> 1) & 3, y & 1

    if x == 1:
        if z == 0:
            mnemonic = "IN (C)" if y == 6 else f"IN {R8[y]}, (C)"
        elif z == 1:
            mnemonic = "OUT (C), 0" if y == 6 else f"OUT (C), {R8[y]}"
        elif z == 2:
            mnemonic = f"SBC HL, {RP[p]}" if q == 0 else f"ADC HL, {RP[p]}"
        elif z == 3:
            nn, offset = read_word(code, offset)
            if nn is None:
                return f"DB 0xED, {fmt_hex8(b)}", offset, 2
            mnemonic = f"LD ({fmt_hex16(nn)}), {RP[p]}" if q == 0 else f"LD {RP[p]}, ({fmt_hex16(nn)})"
        elif z == 4:
            mnemonic = "NEG"
        elif z == 5:
            mnemonic = "RETI" if y == 1 else "RETN"
        elif z == 6:
            im_modes = [0, 0, 1, 2, 0, 0, 1, 2]
            mnemonic = f"IM {im_modes[y]}"
        elif z == 7:
            ops = ['LD I, A', 'LD R, A', 'LD A, I', 'LD A, R', 'RRD', 'RLD', 'NOP', 'NOP']
            mnemonic = ops[y]
        return mnemonic, offset, 1

    elif x == 2:
        if z <= 3 and y >= 4:
            block_ops = {
                (4, 0): 'LDI', (5, 0): 'LDD', (6, 0): 'LDIR', (7, 0): 'LDDR',
                (4, 1): 'CPI', (5, 1): 'CPD', (6, 1): 'CPIR', (7, 1): 'CPDR',
                (4, 2): 'INI', (5, 2): 'IND', (6, 2): 'INIR', (7, 2): 'INDR',
                (4, 3): 'OUTI', (5, 3): 'OUTD', (6, 3): 'OTIR', (7, 3): 'OTDR',
            }
            mnemonic = block_ops.get((y, z), f"DB 0xED, {fmt_hex8(b)}")
            return mnemonic, offset, 1

    return f"DB 0xED, {fmt_hex8(b)}", offset, 1


# ============================================================
# Standard (unprefixed) opcode decoder
# ============================================================

def disasm_standard_opcode(b, code, offset, base_addr, prefix=None):
    x, y, z = (b >> 6) & 3, (b >> 3) & 7, b & 7
    p, q = (y >> 1) & 3, y & 1
    mnemonic = None

    if x == 0:
        if z == 0:
            if y == 0:
                mnemonic = "NOP"
            elif y == 1:
                mnemonic = "EX AF, AF'"
            elif y == 2:
                d, offset = read_byte(code, offset)
                if d is None: return f"DB {fmt_hex8(b)}", offset, 0
                target = (base_addr + offset + signed8(d)) & 0xFFFF
                mnemonic = f"DJNZ {fmt_hex16(target)}"
            elif y == 3:
                d, offset = read_byte(code, offset)
                if d is None: return f"DB {fmt_hex8(b)}", offset, 0
                target = (base_addr + offset + signed8(d)) & 0xFFFF
                mnemonic = f"JR {fmt_hex16(target)}"
            else:
                d, offset = read_byte(code, offset)
                if d is None: return f"DB {fmt_hex8(b)}", offset, 0
                target = (base_addr + offset + signed8(d)) & 0xFFFF
                mnemonic = f"JR {CC[y-4]}, {fmt_hex16(target)}"
        elif z == 1:
            if q == 0:
                nn, offset = read_word(code, offset)
                if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                mnemonic = f"LD {RP[p]}, {fmt_hex16(nn)}"
            else:
                mnemonic = f"ADD HL, {RP[p]}"
        elif z == 2:
            if q == 0:
                if p == 0: mnemonic = "LD (BC), A"
                elif p == 1: mnemonic = "LD (DE), A"
                elif p == 2:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                    mnemonic = f"LD ({fmt_hex16(nn)}), HL"
                elif p == 3:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                    mnemonic = f"LD ({fmt_hex16(nn)}), A"
            else:
                if p == 0: mnemonic = "LD A, (BC)"
                elif p == 1: mnemonic = "LD A, (DE)"
                elif p == 2:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                    mnemonic = f"LD HL, ({fmt_hex16(nn)})"
                elif p == 3:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                    mnemonic = f"LD A, ({fmt_hex16(nn)})"
        elif z == 3:
            mnemonic = f"INC {RP[p]}" if q == 0 else f"DEC {RP[p]}"
        elif z == 4:
            mnemonic = f"INC {R8[y]}"
        elif z == 5:
            mnemonic = f"DEC {R8[y]}"
        elif z == 6:
            n, offset = read_byte(code, offset)
            if n is None: return f"DB {fmt_hex8(b)}", offset, 0
            mnemonic = f"LD {R8[y]}, {fmt_hex8(n)}"
        elif z == 7:
            ops = ['RLCA', 'RRCA', 'RLA', 'RRA', 'DAA', 'CPL', 'SCF', 'CCF']
            mnemonic = ops[y]

    elif x == 1:
        if y == 6 and z == 6:
            mnemonic = "HALT"
        else:
            mnemonic = f"LD {R8[y]}, {R8[z]}"

    elif x == 2:
        mnemonic = f"{ALU[y]} {R8[z]}"

    elif x == 3:
        if z == 0:
            mnemonic = f"RET {CC[y]}"
        elif z == 1:
            if q == 0:
                mnemonic = f"POP {RP2[p]}"
            else:
                mnemonic = ["RET", "EXX", "JP (HL)", "LD SP, HL"][p]
        elif z == 2:
            nn, offset = read_word(code, offset)
            if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
            mnemonic = f"JP {CC[y]}, {fmt_hex16(nn)}"
        elif z == 3:
            if y == 0:
                nn, offset = read_word(code, offset)
                if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                mnemonic = f"JP {fmt_hex16(nn)}"
            elif y == 1:
                mnemonic = "CB PREFIX"
            elif y == 2:
                n, offset = read_byte(code, offset)
                if n is None: return f"DB {fmt_hex8(b)}", offset, 0
                mnemonic = f"OUT ({fmt_hex8(n)}), A"
            elif y == 3:
                n, offset = read_byte(code, offset)
                if n is None: return f"DB {fmt_hex8(b)}", offset, 0
                mnemonic = f"IN A, ({fmt_hex8(n)})"
            elif y == 4: mnemonic = "EX (SP), HL"
            elif y == 5: mnemonic = "EX DE, HL"
            elif y == 6: mnemonic = "DI"
            elif y == 7: mnemonic = "EI"
        elif z == 4:
            nn, offset = read_word(code, offset)
            if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
            mnemonic = f"CALL {CC[y]}, {fmt_hex16(nn)}"
        elif z == 5:
            if q == 0:
                mnemonic = f"PUSH {RP2[p]}"
            else:
                if p == 0:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {fmt_hex8(b)}", offset, 0
                    mnemonic = f"CALL {fmt_hex16(nn)}"
                else:
                    mnemonic = ["DD PREFIX", "ED PREFIX", "FD PREFIX"][p - 1]
        elif z == 6:
            n, offset = read_byte(code, offset)
            if n is None: return f"DB {fmt_hex8(b)}", offset, 0
            mnemonic = f"{ALU[y]} {fmt_hex8(n)}"
        elif z == 7:
            mnemonic = f"RST {fmt_hex8(y * 8)}"

    if mnemonic is None:
        mnemonic = f"DB {fmt_hex8(b)}"
    return mnemonic, offset, 0


# ============================================================
# DD/FD prefix (IX/IY)
# ============================================================

def disassemble_ix_iy(code, offset, base_addr, prefix, reg):
    b, offset = read_byte(code, offset)
    if b is None:
        return f"DB {prefix}", offset, 1

    if b == 0xCB:
        mnemonic, offset, extra = disassemble_ddcb_fdcb(code, offset, base_addr, reg)
        return mnemonic, offset, 2 + extra

    def ix_r8(r_idx):
        if r_idx == 4: return f"{reg}H"
        elif r_idx == 5: return f"{reg}L"
        elif r_idx == 6: return None
        else: return R8[r_idx]

    x, y, z = (b >> 6) & 3, (b >> 3) & 7, b & 7
    p, q = (y >> 1) & 3, y & 1

    if x == 0:
        if z == 1:
            if q == 0:
                nn, offset = read_word(code, offset)
                if nn is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                mnemonic = f"LD {reg}, {fmt_hex16(nn)}" if p == 2 else f"LD {RP[p]}, {fmt_hex16(nn)}"
            else:
                mnemonic = f"ADD {reg}, {reg}" if p == 2 else f"ADD {reg}, {RP[p]}"
            return mnemonic, offset, 1

        if z == 2:
            if q == 0:
                if p == 2:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                    mnemonic = f"LD ({fmt_hex16(nn)}), {reg}"
                else:
                    return disasm_standard_opcode(b, code, offset, base_addr, prefix)
            else:
                if p == 2:
                    nn, offset = read_word(code, offset)
                    if nn is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                    mnemonic = f"LD {reg}, ({fmt_hex16(nn)})"
                else:
                    return disasm_standard_opcode(b, code, offset, base_addr, prefix)
            return mnemonic, offset, 1

        if z == 3:
            if p == 2:
                mnemonic = f"INC {reg}" if q == 0 else f"DEC {reg}"
            else:
                return disasm_standard_opcode(b, code, offset, base_addr, prefix)
            return mnemonic, offset, 1

        if z == 4:
            if y == 4: mnemonic = f"INC {reg}H"
            elif y == 5: mnemonic = f"INC {reg}L"
            elif y == 6:
                d, offset = read_byte(code, offset)
                if d is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                mnemonic = f"INC ({reg}{fmt_displacement(signed8(d))})"
            else:
                return disasm_standard_opcode(b, code, offset, base_addr, prefix)
            return mnemonic, offset, 1

        if z == 5:
            if y == 4: mnemonic = f"DEC {reg}H"
            elif y == 5: mnemonic = f"DEC {reg}L"
            elif y == 6:
                d, offset = read_byte(code, offset)
                if d is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                mnemonic = f"DEC ({reg}{fmt_displacement(signed8(d))})"
            else:
                return disasm_standard_opcode(b, code, offset, base_addr, prefix)
            return mnemonic, offset, 1

        if z == 6:
            if y == 4:
                n, offset = read_byte(code, offset)
                if n is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                mnemonic = f"LD {reg}H, {fmt_hex8(n)}"
            elif y == 5:
                n, offset = read_byte(code, offset)
                if n is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                mnemonic = f"LD {reg}L, {fmt_hex8(n)}"
            elif y == 6:
                d, offset = read_byte(code, offset)
                if d is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
                n, offset = read_byte(code, offset)
                if n is None: return f"DB {prefix}, {fmt_hex8(b)}, {fmt_hex8(d)}", offset, 3
                mnemonic = f"LD ({reg}{fmt_displacement(signed8(d))}), {fmt_hex8(n)}"
            else:
                return disasm_standard_opcode(b, code, offset, base_addr, prefix)
            return mnemonic, offset, 1

        # z == 0 or z == 7: prefix has no effect
        return disasm_standard_opcode(b, code, offset, base_addr, prefix)

    elif x == 1:
        src_is_idx = (z == 6)
        dst_is_idx = (y == 6)
        if src_is_idx and dst_is_idx:
            return "HALT", offset, 1
        if src_is_idx or dst_is_idx:
            d, offset = read_byte(code, offset)
            if d is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
            idx = f"({reg}{fmt_displacement(signed8(d))})"
            mnemonic = f"LD {idx}, {R8[z]}" if dst_is_idx else f"LD {R8[y]}, {idx}"
        else:
            dst = ix_r8(y) or R8[y]
            src = ix_r8(z) or R8[z]
            mnemonic = f"LD {dst}, {src}"
        return mnemonic, offset, 1

    elif x == 2:
        if z == 6:
            d, offset = read_byte(code, offset)
            if d is None: return f"DB {prefix}, {fmt_hex8(b)}", offset, 2
            mnemonic = f"{ALU[y]} ({reg}{fmt_displacement(signed8(d))})"
        else:
            src = ix_r8(z) or R8[z]
            mnemonic = f"{ALU[y]} {src}"
        return mnemonic, offset, 1

    elif x == 3:
        ix_ops = {0xE1: f"POP {reg}", 0xE3: f"EX (SP), {reg}",
                  0xE5: f"PUSH {reg}", 0xE9: f"JP ({reg})", 0xF9: f"LD SP, {reg}"}
        if b in ix_ops:
            return ix_ops[b], offset, 1
        return disasm_standard_opcode(b, code, offset, base_addr, prefix)

    return disasm_standard_opcode(b, code, offset, base_addr, prefix)


# ============================================================
# Top-level instruction decoder
# ============================================================

def disassemble_one(code, offset, base_addr):
    """Disassemble one instruction. Returns (addr, bytes_list, mnemonic, new_offset)."""
    start = offset
    addr = base_addr + offset
    b, offset = read_byte(code, offset)
    if b is None:
        return addr, [], "END", offset
    if b == 0xCB:
        mnemonic, offset, _ = disassemble_cb(code, offset, base_addr)
    elif b == 0xDD:
        mnemonic, offset, _ = disassemble_ix_iy(code, offset, base_addr, "0xDD", "IX")
    elif b == 0xFD:
        mnemonic, offset, _ = disassemble_ix_iy(code, offset, base_addr, "0xFD", "IY")
    elif b == 0xED:
        mnemonic, offset, _ = disassemble_ed(code, offset, base_addr)
    else:
        mnemonic, offset, _ = disasm_standard_opcode(b, code, offset, base_addr)
    return addr, list(code[start:offset]), mnemonic, offset


def disassemble_block(code, base_addr):
    """Disassemble a block of Z80 code. Returns list of (addr, bytes, mnemonic)."""
    result = []
    offset = 0
    while offset < len(code):
        addr, instr_bytes, mnemonic, offset = disassemble_one(code, offset, base_addr)
        result.append((addr, instr_bytes, mnemonic))
    return result


def format_disassembly(instructions):
    """Format disassembled instructions as text lines."""
    lines = []
    for addr, instr_bytes, mnemonic in instructions:
        hex_bytes = ' '.join(f'{b:02X}' for b in instr_bytes)
        lines.append(f"{addr:04X}: {hex_bytes:<16s} {mnemonic}")
    return lines


def hex_dump(data, base_addr, bytes_per_line=16):
    """Create a hex dump of data."""
    lines = []
    for i in range(0, len(data), bytes_per_line):
        addr = base_addr + i
        chunk = data[i:i + bytes_per_line]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        ascii_str = ''.join(chr(b) if 0x20 <= b < 0x7F else '.' for b in chunk)
        lines.append(f"{addr:04X}: {hex_str:<{bytes_per_line * 3 - 1}s}  {ascii_str}")
    return lines


# ============================================================
# TAP file parsing
# ============================================================

def parse_tap(filename):
    """Parse a TAP file into blocks. Returns list of raw block data (including flag + checksum)."""
    with open(filename, 'rb') as f:
        data = f.read()
    blocks = []
    offset = 0
    while offset < len(data):
        if offset + 2 > len(data):
            break
        block_len = struct.unpack_from('<H', data, offset)[0]
        offset += 2
        blocks.append(data[offset:offset + block_len])
        offset += block_len
    return blocks


def parse_tap_header(block):
    """Parse a TAP header block. Returns dict or None if not a header."""
    if len(block) != 19 or block[0] != 0x00:
        return None
    type_names = {0: 'Program', 1: 'Number array', 2: 'Character array', 3: 'Code'}
    block_type = block[1]
    return {
        'type': block_type,
        'type_name': type_names.get(block_type, 'Unknown'),
        'name': block[2:12].decode('ascii', errors='replace').rstrip(),
        'data_length': struct.unpack_from('<H', block, 12)[0],
        'param1': struct.unpack_from('<H', block, 14)[0],
        'param2': struct.unpack_from('<H', block, 16)[0],
    }


def disassemble_tap(tap_file, output_file, only_block=None, include_hex=False):
    """Parse and disassemble a TAP file."""
    blocks = parse_tap(tap_file)
    lines = []
    lines.append(f"; {'='*60}")
    lines.append(f"; Z80 Disassembly of {os.path.basename(tap_file)}")
    lines.append(f"; {'='*60}")
    lines.append(f"; Total blocks: {len(blocks)}")
    lines.append("")

    # First pass: pair headers with data blocks, figure out load addresses
    block_info = []  # list of (header_dict_or_None, data_block, block_index)
    i = 0
    while i < len(blocks):
        hdr = parse_tap_header(blocks[i])
        if hdr is not None and i + 1 < len(blocks):
            block_info.append((hdr, blocks[i + 1], i))
            i += 2
        else:
            block_info.append((None, blocks[i], i))
            i += 1

    for hdr, data_block, blk_idx in block_info:
        if only_block is not None and blk_idx != only_block:
            continue

        flag = data_block[0] if len(data_block) > 0 else 0
        payload = data_block[1:-1] if len(data_block) > 2 else data_block  # strip flag + checksum

        if hdr:
            lines.append(f"; === Block {blk_idx}/{blk_idx+1}: {hdr['type_name']} \"{hdr['name']}\" ===")

            if hdr['type'] == 0:  # BASIC Program
                lines.append(f"; Auto-start line: {hdr['param1']}")
                lines.append(f"; Data length: {hdr['data_length']} bytes")
                lines.append("; (BASIC program — not disassembled)")
                lines.append("")
                if include_hex:
                    for line in hex_dump(payload, 0):
                        lines.append(f"; {line}")
                    lines.append("")

            elif hdr['type'] == 3:  # Code
                load_addr = hdr['param1']
                lines.append(f"; Load address: {fmt_hex16(load_addr)}")
                lines.append(f"; Length: {hdr['data_length']} bytes")
                lines.append("")
                instrs = disassemble_block(payload, load_addr)
                lines.extend(format_disassembly(instrs))
                lines.append("")

            else:
                lines.append(f"; Data length: {hdr['data_length']} bytes")
                lines.append("; (Data block — not disassembled)")
                lines.append("")
                if include_hex:
                    for line in hex_dump(payload, 0):
                        lines.append(f"; {line}")
                    lines.append("")
        else:
            lines.append(f"; === Block {blk_idx}: Headerless data block ===")
            lines.append(f"; Flag: {fmt_hex8(flag)}")
            lines.append(f"; Payload length: {len(payload)} bytes")

            # Heuristic: if it looks like screen data (6912 bytes), label it
            if len(payload) == 6912:
                lines.append("; Likely ZX Spectrum screen data (6144 pixels + 768 attributes)")
                lines.append("")
                if include_hex:
                    lines.append("; First 128 bytes:")
                    for line in hex_dump(payload[:128], 0x4000):
                        lines.append(f"; {line}")
                    lines.append("; ...")
                    lines.append("")
            else:
                lines.append("; (Headerless block — load address unknown, disassembling from 0x0000)")
                lines.append("; Use --org to specify the correct base address for raw blocks.")
                lines.append("")
                instrs = disassemble_block(payload, 0x0000)
                lines.extend(format_disassembly(instrs))
                lines.append("")

    lines.append("; === End of Disassembly ===")

    output_text = '\n'.join(lines) + '\n'
    with open(output_file, 'w') as f:
        f.write(output_text)
    print(f"Disassembly written to {output_file} ({len(lines)} lines)")


def disassemble_raw(bin_file, output_file, org=0):
    """Disassemble a raw binary file."""
    with open(bin_file, 'rb') as f:
        code = f.read()
    lines = []
    lines.append(f"; {'='*60}")
    lines.append(f"; Z80 Disassembly of {os.path.basename(bin_file)}")
    lines.append(f"; {'='*60}")
    lines.append(f"; ORG: {fmt_hex16(org)}")
    lines.append(f"; Length: {len(code)} bytes ({fmt_hex16(len(code))})")
    lines.append("")
    instrs = disassemble_block(code, org)
    lines.extend(format_disassembly(instrs))
    lines.append("")
    lines.append("; === End of Disassembly ===")

    output_text = '\n'.join(lines) + '\n'
    with open(output_file, 'w') as f:
        f.write(output_text)
    print(f"Disassembly written to {output_file} ({len(lines)} lines)")


# ============================================================
# API: for use as a library from other scripts
# ============================================================

def disasm_bytes(code_bytes, org=0):
    """Disassemble bytes and return list of (addr, hex_bytes, mnemonic) tuples."""
    if isinstance(code_bytes, (bytes, bytearray)):
        code = code_bytes
    else:
        code = bytes(code_bytes)
    return disassemble_block(code, org)


def disasm_bytes_text(code_bytes, org=0):
    """Disassemble bytes and return formatted text string."""
    instrs = disasm_bytes(code_bytes, org)
    return '\n'.join(format_disassembly(instrs))


# ============================================================
# CLI
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description='Z80 disassembler for TAP files and raw binaries (no dependencies)')
    parser.add_argument('input', help='Input file (.tap or raw binary)')
    parser.add_argument('-o', '--output', help='Output file (default: <input>_disasm.txt)')
    parser.add_argument('--org', type=lambda x: int(x, 0), default=0,
                        help='Origin address for raw binary (e.g. 0x8000)')
    parser.add_argument('--raw', action='store_true',
                        help='Force raw binary mode (skip TAP parsing)')
    parser.add_argument('--block', type=int, default=None,
                        help='Disassemble only this TAP block number')
    parser.add_argument('--hex', action='store_true',
                        help='Include hex dumps for non-code areas')

    args = parser.parse_args()

    if args.output:
        output_file = args.output
    else:
        base = os.path.splitext(args.input)[0]
        output_file = base + '_disasm.txt'

    is_tap = args.input.lower().endswith('.tap') and not args.raw

    if is_tap:
        disassemble_tap(args.input, output_file, only_block=args.block, include_hex=args.hex)
    else:
        disassemble_raw(args.input, output_file, org=args.org)


if __name__ == '__main__':
    main()
