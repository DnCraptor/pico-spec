/*
    zxnDMA / Z80 DMA emulation for ZX Spectrum Next compatibility
    Supports both Port #0B (Zilog Z80 DMA) and Port #6B (zxnDMA) modes.

    Reference: https://wiki.specnext.dev/DMA
*/

#include "Z80DMA.h"

#if !PICO_RP2040

#include <string.h>
#include "CPU.h"
#include "Config.h"
#include "MemESP.h"
#include "Ports.h"
#include "Video.h"

#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("z80dma")
#pragma GCC optimize("O3")

// Configured state
uint8_t  Z80DMA::transfer_dir = 0;
uint16_t Z80DMA::port_a_addr = 0;
uint16_t Z80DMA::block_length = 0;
bool     Z80DMA::port_a_is_io = false;
int8_t   Z80DMA::port_a_inc = 1;
uint8_t  Z80DMA::port_a_cycles = 3;  // FPGA default: "01" = 3

uint16_t Z80DMA::port_b_addr = 0;
bool     Z80DMA::port_b_is_io = false;
int8_t   Z80DMA::port_b_inc = 1;
uint8_t  Z80DMA::port_b_cycles = 3;  // FPGA default: "01" = 3

uint8_t  Z80DMA::prescalar = 0;
bool     Z80DMA::auto_restart = false;

// Working state
uint16_t Z80DMA::cur_port_a = 0;
uint16_t Z80DMA::cur_port_b = 0;
uint32_t Z80DMA::byte_counter = 0;
bool     Z80DMA::transfer_active = false;
bool     Z80DMA::transfer_started = false;
bool     Z80DMA::block_end = false;

// Write state machine
Z80DMA::WriteState Z80DMA::write_state = WS_COMMAND;
uint8_t  Z80DMA::current_wr = 0;
uint8_t  Z80DMA::param_mask = 0;
uint8_t  Z80DMA::param_index = 0;
bool     Z80DMA::read_mask_pending = false;
bool     Z80DMA::wr2_prescalar_pending = false;

// Read state machine
uint8_t  Z80DMA::read_mask = 0x7F;
uint8_t  Z80DMA::read_index = 0;
bool     Z80DMA::read_sequence_active = false;

// Reentrancy guard
bool     Z80DMA::dma_in_progress = false;

// Per-scanline attr shadow buffer
uint8_t  Z80DMA::dma_attr_shadow[192 * 32];
bool     Z80DMA::dma_attr_valid[192];
bool     Z80DMA::dma_charrow_active[24];
static uint8_t charrow_write_cnt[24];
static uint8_t prev_attrs[24][32];
static bool    prev_attrs_saved[24];

// Decode WR1/WR2 address increment from bits 4:3
static inline int8_t decodeIncrement(uint8_t bits) {
    // bits = (reg >> 3) & 0x03
    // 00=decrement, 01=increment, 10=fixed, 11=fixed
    switch (bits) {
        case 0: return -1;
        case 1: return  1;
        default: return 0;
    }
}

// Decode WR1/WR2 cycle length from bits 1:0 of timing byte
// FPGA: 00=4 cycles, 01=3 cycles, 10=2 cycles, 11=4 cycles
static inline uint8_t decodeCycles(uint8_t bits) {
    switch (bits & 0x03) {
        case 0: return 4;
        case 1: return 3;
        case 2: return 2;
        case 3: return 4;
    }
    return 3;
}

void Z80DMA::reset() {
    transfer_dir = 0;
    port_a_addr = 0;
    block_length = 0;
    port_a_is_io = false;
    port_a_inc = 1;
    port_a_cycles = 3;  // FPGA default: "01" = 3 cycles

    port_b_addr = 0;
    port_b_is_io = false;
    port_b_inc = 1;
    port_b_cycles = 3;  // FPGA default: "01" = 3 cycles

    prescalar = 0;
    auto_restart = false;

    cur_port_a = 0;
    cur_port_b = 0;
    byte_counter = 0;
    transfer_active = false;
    transfer_started = false;
    block_end = false;

    write_state = WS_COMMAND;
    current_wr = 0;
    param_mask = 0;
    param_index = 0;
    read_mask_pending = false;
    wr2_prescalar_pending = false;

    read_mask = 0x7F;
    read_index = 0;
    read_sequence_active = false;

    dma_in_progress = false;

    memset(dma_attr_shadow, 0, sizeof(dma_attr_shadow));
    memset(dma_attr_valid, 0, sizeof(dma_attr_valid));
    memset(dma_charrow_active, 0, sizeof(dma_charrow_active));
    memset(charrow_write_cnt, 0, sizeof(charrow_write_cnt));
    memset(prev_attrs_saved, 0, sizeof(prev_attrs_saved));
}

void Z80DMA::resetAttrShadow() {
    memset(dma_attr_valid, 0, sizeof(dma_attr_valid));
    memset(dma_charrow_active, 0, sizeof(dma_charrow_active));
    memset(charrow_write_cnt, 0, sizeof(charrow_write_cnt));
    memset(prev_attrs_saved, 0, sizeof(prev_attrs_saved));
}

IRAM_ATTR void Z80DMA::writePort(uint8_t data) {
    if (dma_in_progress) return;

    // Read mask byte (after 0xBB command)
    if (read_mask_pending) {
        read_mask = data & 0x7F;
        read_mask_pending = false;
        read_index = 0;
        read_sequence_active = true;
        return;
    }

    // WR2 prescalar byte
    if (wr2_prescalar_pending) {
        prescalar = data;
        wr2_prescalar_pending = false;
        write_state = WS_COMMAND;
        return;
    }

    if (write_state == WS_PARAM_BYTE) {
        collectParam(data);
        return;
    }

    // WS_COMMAND: decode which WR register
    processCommand(data);
}

IRAM_ATTR void Z80DMA::processCommand(uint8_t data) {
    // WR6: bit7=1, bits[1:0]=11
    if ((data & 0x83) == 0x83) {
        processWR6(data);
        return;
    }
    // WR5: top 3 bits=100, bottom 3 bits=010 → mask 0xC7 == 0x82
    if ((data & 0xC7) == 0x82) {
        processWR5(data);
        return;
    }
    // WR4: bit7=1, bits[1:0]=01
    if ((data & 0x83) == 0x81) {
        processWR4(data);
        return;
    }
    // WR3: bit7=1, bits[1:0]=00
    // Bit 6 = enable DMA flag. On real HW this starts bus request sequence
    // but transfer doesn't happen until bus is granted. In our sequential
    // model, actual transfer is triggered by WR6 ENABLE (0x87).
    if ((data & 0x83) == 0x80) {
        // bits 3,4 may indicate mask/match parameter bytes follow
        // (not implemented — ignored)
        return;
    }
    // WR1: bit7=0, bits[2:0]=100 (mask 0x87 == 0x04)
    if ((data & 0x87) == 0x04) {
        processWR1(data);
        return;
    }
    // WR2: bit7=0, bits[2:0]=000 (mask 0x87 == 0x00)
    if ((data & 0x87) == 0x00) {
        processWR2(data);
        return;
    }
    // WR0: bit7=0 and bits[1:0] != 00
    if ((data & 0x80) == 0) {
        processWR0(data);
        return;
    }
}

void Z80DMA::processWR0(uint8_t data) {
    // FPGA: R0_dir_AtoB_s <= cpu_d_i(2)
    // Bit 2 = direction: 0=B->A, 1=A->B
    transfer_dir = (data >> 2) & 1;

    // FPGA: D3=Port A addr LOW, D4=Port A addr HIGH, D5=Block len LOW, D6=Block len HIGH
    param_mask = (data >> 3) & 0x0F;
    if (param_mask) {
        write_state = WS_PARAM_BYTE;
        current_wr = 0;
        param_index = 0;
    }
}

void Z80DMA::processWR1(uint8_t data) {
    // FPGA: WR1 pattern bits[2:0]=100
    // bit 3 = R1_portAisIO_s
    // bits 5:4 = R1_portA_addrMode_s
    // bit 6: if set, timing parameter byte follows (R1_BYTE_0)
    port_a_is_io = (data >> 3) & 1;
    port_a_inc = decodeIncrement((data >> 4) & 0x03);
    if (data & 0x40) {
        // Timing byte follows
        write_state = WS_PARAM_BYTE;
        current_wr = 1;
        param_mask = 1; // one byte expected
        param_index = 0;
    }
}

void Z80DMA::processWR2(uint8_t data) {
    // FPGA: WR2 pattern bits[2:0]=000
    // bit 3 = R2_portBisIO_s
    // bits 5:4 = R2_portB_addrMode_s
    // bit 6: if set, timing parameter byte follows (R2_BYTE_0)
    //   then if prescalar enabled, R2_BYTE_1 = prescalar
    port_b_is_io = (data >> 3) & 1;
    port_b_inc = decodeIncrement((data >> 4) & 0x03);
    if (data & 0x40) {
        // Timing byte follows (and possibly prescalar after that)
        write_state = WS_PARAM_BYTE;
        current_wr = 2;
        param_mask = 1; // timing byte expected
        param_index = 0;
    }
}

void Z80DMA::processWR4(uint8_t data) {
    // FPGA: WR4 pattern bit7=1, bits[1:0]=01
    // bit 2 = Port B addr LOW follows (R4_BYTE_0)
    // bit 3 = Port B addr HIGH follows (R4_BYTE_1)
    param_mask = (data >> 2) & 0x03;
    if (param_mask) {
        write_state = WS_PARAM_BYTE;
        current_wr = 4;
        param_index = 0;
    }
}

void Z80DMA::processWR5(uint8_t data) {
    // bit 5: auto-restart (0=stop on block end, 1=auto-restart)
    auto_restart = (data >> 5) & 1;
}

IRAM_ATTR void Z80DMA::processWR6(uint8_t cmd) {
    switch (cmd) {
        case 0xC3: // RESET
            transfer_active = false;
            transfer_started = false;
            block_end = false;
            port_a_cycles = 3;
            port_b_cycles = 3;
            prescalar = 0;
            auto_restart = false;
            read_mask = 0x7F;
            write_state = WS_COMMAND;
            read_mask_pending = false;
            wr2_prescalar_pending = false;
            read_sequence_active = false;
            read_index = 0;
            dma_in_progress = false;
            break;
        case 0xC7: // Reset Port A Timing (FPGA: only resets timing byte to "01"=3)
            port_a_cycles = 3;
            break;
        case 0xCB: // Reset Port B Timing (FPGA: only resets timing byte to "01"=3)
            port_b_cycles = 3;
            break;
        case 0xCF: // LOAD
            doLoad();
            break;
        case 0xD3: // CONTINUE
            doContinue();
            break;
        case 0x87: // ENABLE
            doEnable();
            break;
        case 0x83: // DISABLE
            doDisable();
            break;
        case 0xBF: // Read Status Byte
            read_sequence_active = false;
            break;
        case 0x8B: // Reinitialize Status
            block_end = false;
            transfer_started = false;
            break;
        case 0xA7: // Initialize Read Sequence
            read_index = 0;
            read_sequence_active = true;
            break;
        case 0xBB: // Read Mask Follows
            read_mask_pending = true;
            break;
        case 0xB3: // Force Ready (irrelevant for zxnDMA)
            break;
        default:
            break;
    }
}

IRAM_ATTR void Z80DMA::collectParam(uint8_t data) {
    // Find next set bit in param_mask starting from param_index
    while (param_index < 8 && !(param_mask & (1 << param_index))) {
        param_index++;
    }

    if (param_index >= 8) {
        // No more params expected
        write_state = WS_COMMAND;
        return;
    }

    if (current_wr == 0) {
        switch (param_index) {
            case 0: port_a_addr = (port_a_addr & 0xFF00) | data; break;
            case 1: port_a_addr = (port_a_addr & 0x00FF) | (data << 8); break;
            case 2: block_length = (block_length & 0xFF00) | data; break;
            case 3: block_length = (block_length & 0x00FF) | (data << 8); break;
        }
    } else if (current_wr == 1) {
        // WR1 timing byte (R1_BYTE_0): bits 1:0 = cycle length for Port A
        // FPGA: bit 5 of timing byte → R1_BYTE_1 follows (prescalar, ignored)
        port_a_cycles = decodeCycles(data & 0x03);
        if (data & 0x20) {
            // Port A prescalar byte follows (ignored per FPGA)
            wr2_prescalar_pending = false; // just consume next byte
            param_mask = 1; // expect one more byte
            param_index = 0;
            current_wr = 10; // dummy: consume and discard
            return; // don't fall through to clear logic below
        }
    } else if (current_wr == 2) {
        // WR2 timing byte (R2_BYTE_0): bits 1:0 = cycle length for Port B
        // FPGA: bit 5 → R2_BYTE_1 (prescalar) follows
        port_b_cycles = decodeCycles(data & 0x03);
        if (data & 0x20) {
            wr2_prescalar_pending = true; // next byte is prescalar
        }
    } else if (current_wr == 10) {
        // Dummy: discard WR1 prescalar byte (not used per FPGA)
    } else if (current_wr == 4) {
        // WR4 params: bit2=portB addr low, bit3=portB addr high
        switch (param_index) {
            case 0: // Port B address low
                port_b_addr = (port_b_addr & 0xFF00) | data;
                break;
            case 1: // Port B address high
                port_b_addr = (port_b_addr & 0x00FF) | (data << 8);
                break;
        }
    }

    // Clear this bit and advance
    param_mask &= ~(1 << param_index);
    param_index++;

    // Check if more params remain
    if (param_mask == 0) {
        write_state = WS_COMMAND;
    }
}

void Z80DMA::doLoad() {
    cur_port_a = port_a_addr;
    cur_port_b = port_b_addr;
    // FPGA: zxn mode counter starts at 0, z80 mode counter starts at 0xFFFF
    // zxn: transfers block_length bytes; z80: transfers block_length+1 bytes
    byte_counter = block_length;
    if (Config::dma_mode == 1) byte_counter++; // z80 DMA always: length+1
    block_end = false;
}

void Z80DMA::doContinue() {
    // Reset byte counter but keep current working pointers
    byte_counter = block_length;
    if (Config::dma_mode == 1) byte_counter++;
    block_end = false;
}

void Z80DMA::doEnable() {
    transfer_active = true;
    executeTransfer();
}

void Z80DMA::doDisable() {
    transfer_active = false;
}

// Called from CPU loop — stub for future burst mode
IRAM_ATTR void Z80DMA::handleDMA() {
}

// Shadow previous attrs before DMA overwrites them.
// First write per charrow: save as reference, no shadow.
// Second write: compare — if different, charrow is "per-scanline active",
// then all subsequent writes skip compare and always shadow.
static IRAM_ATTR void captureAttrAfterTransfer(uint16_t dest_start) {
    if (dest_start < 0x5800 || dest_start > 0x5AFF) return;

    uint8_t charrow = (dest_start - 0x5800) >> 5;
    if (charrow >= 24) return;

    uint8_t sub = charrow_write_cnt[charrow];
    if (sub >= 8) return;
    charrow_write_cnt[charrow] = sub + 1;

    int scanline = charrow * 8 + sub;
    if (scanline >= 192) return;

    uint16_t attr_base = 0x1800 + (charrow << 5);

    if (sub == 0) {
        // First write: save as reference
        memcpy(prev_attrs[charrow], &VIDEO::grmem[attr_base], 32);
        prev_attrs_saved[charrow] = true;
        return;
    }

    if (!prev_attrs_saved[charrow]) return;

    // Once confirmed active, skip compare for remaining writes
    if (!Z80DMA::dma_charrow_active[charrow]) {
        // Check if attrs changed
        if (memcmp(&VIDEO::grmem[attr_base], prev_attrs[charrow], 32) == 0) {
            memcpy(prev_attrs[charrow], &VIDEO::grmem[attr_base], 32);
            return;
        }
        // Confirmed per-scanline effect — mark active
        Z80DMA::dma_charrow_active[charrow] = true;
        // Shadow sub=0 retroactively
        memcpy(&Z80DMA::dma_attr_shadow[charrow * 8 * 32], prev_attrs[charrow], 32);
        Z80DMA::dma_attr_valid[charrow * 8] = true;
    }

    // Shadow previous scanline with prev_attrs (before DMA overwrote)
    int prev_scanline = charrow * 8 + sub - 1;
    if (prev_scanline >= 0 && prev_scanline < 192) {
        memcpy(&Z80DMA::dma_attr_shadow[prev_scanline * 32], prev_attrs[charrow], 32);
        Z80DMA::dma_attr_valid[prev_scanline] = true;
    }
    // Also shadow current scanline with current grmem (may be overwritten by next DMA)
    memcpy(&Z80DMA::dma_attr_shadow[scanline * 32], &VIDEO::grmem[attr_base], 32);
    Z80DMA::dma_attr_valid[scanline] = true;

    memcpy(prev_attrs[charrow], &VIDEO::grmem[attr_base], 32);
}

IRAM_ATTR void Z80DMA::executeTransfer() {
    dma_in_progress = true;

    uint16_t dest_start = transfer_dir ? cur_port_b : cur_port_a;

    while (byte_counter > 0 && transfer_active) {
        transfer_started = true;

        uint8_t val;
        if (transfer_dir) {
            // A → B: read A, write B
            VIDEO::Draw(port_a_cycles, false);
            val = port_a_is_io ? Ports::dmaInput(cur_port_a) : MemESP::readbyte(cur_port_a);
            VIDEO::Draw(port_b_cycles, false);
            if (port_b_is_io) Ports::dmaOutput(cur_port_b, val);
            else MemESP::writebyte(cur_port_b, val);
        } else {
            // B → A: read B, write A
            VIDEO::Draw(port_b_cycles, false);
            val = port_b_is_io ? Ports::dmaInput(cur_port_b) : MemESP::readbyte(cur_port_b);
            VIDEO::Draw(port_a_cycles, false);
            if (port_a_is_io) Ports::dmaOutput(cur_port_a, val);
            else MemESP::writebyte(cur_port_a, val);
        }

        cur_port_a += port_a_inc;
        cur_port_b += port_b_inc;
        byte_counter--;

        // Handle frame boundary mid-transfer
        if (CPU::tstates >= CPU::statesInFrame) {
            VIDEO::EndFrame();
            CPU::global_tstates += CPU::statesInFrame;
            CPU::tstates -= CPU::statesInFrame;
        }
    }

    // Snapshot attr state for per-scanline effects
    captureAttrAfterTransfer(dest_start);

    if (byte_counter == 0) {
        block_end = true;
        if (auto_restart) {
            doLoad();
            if (transfer_active) {
                dma_in_progress = false;
                executeTransfer();
                return;
            }
        } else {
            transfer_active = false;
        }
    }

    dma_in_progress = false;
}

IRAM_ATTR void Z80DMA::transferOneByte() {
    transfer_started = true;
    uint8_t val;

    if (transfer_dir) {
        // A -> B (bit 2 = 1): read A (portA timing), write B (portB timing)
        if (port_a_is_io) {
            val = Ports::input(cur_port_a);
        } else {
            val = MemESP::readbyte(cur_port_a);
        }
        if (port_b_is_io) {
            Ports::output(cur_port_b, val);
        } else {
            MemESP::writebyte(cur_port_b, val);
        }
    } else {
        // B -> A (bit 2 = 0): read B (portB timing), write A (portA timing)
        if (port_b_is_io) {
            val = Ports::input(cur_port_b);
        } else {
            val = MemESP::readbyte(cur_port_b);
        }
        if (port_a_is_io) {
            Ports::output(cur_port_a, val);
        } else {
            MemESP::writebyte(cur_port_a, val);
        }
    }

    // DMA runs at CPU speed. Each read/write = port_X_cycles T-states.
    VIDEO::Draw(port_a_cycles + port_b_cycles, false);

    cur_port_a += port_a_inc;
    cur_port_b += port_b_inc;
    byte_counter--;
}

uint8_t Z80DMA::getStatusByte() {
    // FPGA: "00" & status_endofblock_n & "1101" & status_atleastone
    // Bit 5 = endofblock_n (1=NOT ended, 0=ended), Bit 0 = atleastone (transfer started)
    return 0x1A | (block_end ? 0x00 : 0x20) | (transfer_started ? 0x01 : 0);
}

IRAM_ATTR uint8_t Z80DMA::readPort() {
    if (!read_sequence_active) {
        return getStatusByte();
    }

    // Find next set bit in read_mask starting from read_index
    while (read_index < 7 && !(read_mask & (1 << read_index))) {
        read_index++;
    }
    if (read_index >= 7) {
        // Wrap around
        read_index = 0;
        while (read_index < 7 && !(read_mask & (1 << read_index))) {
            read_index++;
        }
        if (read_index >= 7) {
            return getStatusByte();
        }
    }

    uint8_t result;
    switch (read_index) {
        case 0: result = getStatusByte(); break;
        case 1: result = byte_counter & 0xFF; break;
        case 2: result = byte_counter >> 8; break;
        case 3: result = cur_port_a & 0xFF; break;
        case 4: result = cur_port_a >> 8; break;
        case 5: result = cur_port_b & 0xFF; break;
        case 6: result = cur_port_b >> 8; break;
        default: result = 0xFF; break;
    }

    // Advance to next set bit
    read_index++;
    if (read_index >= 7) read_index = 0; // wrap

    return result;
}

#endif // !PICO_RP2040
