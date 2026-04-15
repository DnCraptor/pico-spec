#include "MB02.h"

#if !PICO_RP2040

#include "Config.h"
#include "MemESP.h"
#include "ESPectrum.h"
#include "Debug.h"
#include "wd1793.h"
#include "Z80DMA.h"
#include "roms/mb02boot.h"

extern int butter_pages;
extern uint8_t* PSRAM_DATA;
uint32_t butter_psram_size();

bool MB02::enabled = false;
uint8_t MB02::paging_reg = 0;
uint8_t MB02::floppy_reg = 0xFF;
uint8_t* MB02::sram_base = nullptr;
bool MB02::write_enabled = false;
uint8_t MB02::page0_composite[0x2000] = {};
uint8_t MB02::motor_state = 0;

void MB02::init() {
    enabled = false;
    sram_base = nullptr;
    MemESP::mb02_write_gate = true; // restore default (allow all writes)
    Z80DMA::mb02_deferred = false;

    // Unmap SRAM/EPROM when disabling
    MemESP::divmmc_mapped = false;
    MemESP::recoverPage0();

    if (!Config::mb02) return;

    // Calculate PSRAM allocation after Spectrum pages
    // MB-02 and DivMMC are mutually exclusive (both page into 0x0000-0x3FFF),
    // but DivMMC may have already reserved PSRAM space. Account for it.
    size_t butter_used = (size_t)butter_pages * MEM_PG_SZ;
    if (Config::esxdos) {
        butter_used += 16 * 0x2000; // DivMMC banks already allocated
    }

    size_t mb02_total = MB02_NUM_PAGES * MB02_PAGE_SIZE; // 512KB
    size_t butter_total = butter_psram_size();

    if (butter_total < butter_used + mb02_total) {
        Debug::log("MB02: not enough PSRAM (%d KB free, need %d KB)",
                   (int)((butter_total - butter_used) / 1024),
                   (int)(mb02_total / 1024));
        return;
    }

    sram_base = PSRAM_DATA + butter_used;

    // Clear SRAM
    memset(sram_base, 0, mb02_total);

    // Set up the MB-02 FDD instance
    memset(&ESPectrum::mb02_fdd, 0, sizeof(rvmWD1793));
    ESPectrum::mb02_fdd.wd2797_mode = true;
    rvmWD1793Reset(&ESPectrum::mb02_fdd);
    ESPectrum::mb02_fdd.wd2797_mode = true; // preserve after reset

    // MB-02+ requires Z80-DMA on port #0B
    if (!Config::dma_mode) {
        Config::dma_mode = 1; // Port #0B (Z80 DMA)
    }

    // MB-02 uses DMA with READY/DRQ signaling — defer executeTransfer()
    Z80DMA::mb02_deferred = true;

    enabled = true;
    Debug::log("MB02: enabled, 512K SRAM @ %p, dma_mode=%d", sram_base, Config::dma_mode);
}

void MB02::reset() {
    if (!enabled) return;

    // Reset FDD
    rvmWD1793Reset(&ESPectrum::mb02_fdd);
    ESPectrum::mb02_fdd.wd2797_mode = true;

    // Boot state: EPROM enabled, SRAM disabled
    paging_reg = MB02_EPROM_ENABLE;
    floppy_reg = 0xFF; // all drives deselected (active low)
    motor_state = 0;
    write_enabled = false;

    applyMapping();
}

void MB02::applyMapping() {
    bool sram_en = paging_reg & MB02_SRAM_ENABLE;
    bool eprom_en = paging_reg & MB02_EPROM_ENABLE;

    write_enabled = (paging_reg & MB02_WRITE_ENABLE) != 0;
    MemESP::mb02_write_gate = write_enabled || !enabled; // allow writes when MB-02 is off

    if (sram_en && eprom_en) {
        // Both SRAM and EPROM enabled → /RESET on real hardware
        // Trigger a soft reset
        MemESP::divmmc_mapped = false;
        MemESP::recoverPage0();
        return;
    }

    if (eprom_en) {
        // EPROM mode: 2K EPROM at 0x0000-0x07FF, normal ROM at 0x0800-0x3FFF
        // Build composite page0_lo (8KB: 2K EPROM + 6K ROM)
        memcpy(page0_composite, gb_rom_mb02boot, MB02_EPROM_SIZE);
        // Patch: skip RAM test at boot (0x0021: JR NZ → JR unconditional)
        // Boot code tests RAM via 0x7FFD bank switching, causing screen artifacts.
        // In emulation RAM is always good, so skip directly to disk loader.
        page0_composite[0x0021] = 0x18; // JR NZ,xx → JR xx
        uint8_t* rom_ptr = MemESP::rom[MemESP::romInUse].direct();
        if (rom_ptr) {
            memcpy(page0_composite + MB02_EPROM_SIZE, rom_ptr + MB02_EPROM_SIZE,
                   0x2000 - MB02_EPROM_SIZE);
        }

        MemESP::page0_lo = page0_composite;
        MemESP::page0_hi = rom_ptr ? rom_ptr + 0x2000 : page0_composite; // 0x2000-0x3FFF from ROM
        MemESP::divmmc_mapped = true;
        MemESP::divmmc_lo_dirty = nullptr;
        MemESP::divmmc_hi_dirty = nullptr;
        return;
    }

    if (sram_en) {
        // SRAM mode: page SRAM into 0x0000-0x3FFF
        uint8_t page = paging_reg & MB02_PAGE_MASK;
        uint8_t* sram_page = sram_base + (uint32_t)page * MB02_PAGE_SIZE;

        MemESP::page0_lo = sram_page;
        MemESP::page0_hi = sram_page + 0x2000;
        MemESP::divmmc_mapped = true;
        MemESP::divmmc_lo_dirty = nullptr;
        MemESP::divmmc_hi_dirty = nullptr;
        return;
    }

    // Neither SRAM nor EPROM: restore normal ROM
    MemESP::divmmc_mapped = false;
    MemESP::recoverPage0();
}

void MB02::writePort17(uint8_t data) {
    static uint8_t last_paging = 0xFF;
    if (data != last_paging && data != 0x60 && data != 0x61) {
        Debug::log("MB02 OUT #17=%02X (page=%d sram=%d eprom=%d write=%d)",
                   data, data & 0x1F, (data>>6)&1, (data>>7)&1, (data>>5)&1);
    }
    last_paging = data;
    paging_reg = data;
    applyMapping();
}

void MB02::writePort13(uint8_t data) {
    floppy_reg = data;

    // Port #13 OUT — register is active-HIGH (1 = drive/motor selected)
    // (hardware inverts to active-low FDD cable signals)
    // Bit 0: ACTIVE_A, Bit 2: ACTIVE_B, Bit 4: ACTIVE_C, Bit 6: ACTIVE_D
    uint8_t drive = ESPectrum::mb02_fdd.diskS; // keep current if none selected
    if (data & 0x01) drive = 0;
    else if (data & 0x04) drive = 1;
    else if (data & 0x10) drive = 2;
    else if (data & 0x40) drive = 3;

    if (ESPectrum::mb02_fdd.diskS != drive) {
        ESPectrum::mb02_fdd.diskS = drive;
        if (ESPectrum::mb02_fdd.disk[drive] &&
            ESPectrum::mb02_fdd.side &&
            ESPectrum::mb02_fdd.disk[drive]->sides == 1)
            ESPectrum::mb02_fdd.side = 0;
    }

    // Motor state: Bit 1: MOTOR_A, Bit 3: MOTOR_B (active-high: 1 = on)
    motor_state = 0;
    if (data & 0x02) motor_state |= 0x01;
    if (data & 0x08) motor_state |= 0x02;
    if (data & 0x20) motor_state |= 0x04;
    if (data & 0x80) motor_state |= 0x08;

    // Power flags are set by InsertDisk and must persist regardless of motor state.
    // Don't modify Power flags here at all — motor on/off is cosmetic in emulation.
}

uint8_t MB02::readPort13() {
    // Port #13 IN — documentation says active-low on FDD bus, but the
    // register inverts signals so software sees active-HIGH:
    // 1 = signal active, 0 = signal inactive
    uint8_t result = 0;

    // Bit 0: DRQ (1 = active)
    if (ESPectrum::mb02_fdd.control & kRVMWD177XDRQ)
        result |= 0x01;

    // Bit 1: DISK CHANGE (1 = disk was changed/inserted)
    // Set to 0 = no change (disk stable) — BS-DOS uses this for drive detection
    // BS-DOS aktive routine: RET C when /DISK_CHANGE(bit1)=1 in active-low terms
    // which is bit1=1 in register = no change → drive ready, proceed
    result |= 0x02; // no disk change = drive stable

    // Bit 2: INTRQ (1 = active)
    if (ESPectrum::mb02_fdd.control & (kRVMWD177XINTRQ | kRVMWD177XFINTRQ))
        result |= 0x04;

    // Bit 3: HDIN (1 = HD)
    result |= 0x08;

    // Bits 4-7: MOTOR ACTIVE A-D (1 = motor running)
    if (motor_state & 0x01) result |= 0x10;
    if (motor_state & 0x02) result |= 0x20;
    if (motor_state & 0x04) result |= 0x40;
    if (motor_state & 0x08) result |= 0x80;

    return result;
}

#endif // !PICO_RP2040
