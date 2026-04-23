#include "MB02.h"

#if !PICO_RP2040

#include "Config.h"
#include "MemESP.h"
#include "ESPectrum.h"
#include "Debug.h"
#include "wd1793.h"
#include "Z80DMA.h"
#include "roms/mb02boot.h"

bool MB02::enabled = false;
uint8_t MB02::paging_reg = 0;
uint8_t MB02::floppy_reg = 0xFF;
bool MB02::write_enabled = false;
uint8_t MB02::page0_composite[0x2000] = {};
uint8_t MB02::motor_state = 0;
int MB02::ram_base_idx = -1;
bool MB02::disk_changed = false;

void MB02::init() {
    enabled = false;
    MemESP::mb02_write_gate = true;
    Z80DMA::mb02_deferred = false;

    // Unmap SRAM/EPROM when disabling
    MemESP::divmmc_mapped = false;
    MemESP::recoverPage0();

    if (!Config::mb02) {
        Config::dma_mode = 0;
        return;
    }

    // Use MemESP::ram[] pages for MB-02 SRAM.
    // Spectrum 128K uses pages 0-7, +2 hidden ROM pages = 10 pages.
    // Pages 10+ are available. We need 32 pages (512KB).
    // MEM_PG_CNT+2 pages total are allocated by ESPectrum::setup().
    int total_pages = MEM_PG_CNT + 2; // typically 66
    int base = total_pages - MB02_NUM_PAGES; // use last 32 pages
    if (base < 10) base = 10; // don't overlap with Spectrum pages
    if (base + MB02_NUM_PAGES > total_pages) {
        Debug::log("MB02: not enough RAM pages (%d available, need %d)",
                   total_pages - base, MB02_NUM_PAGES);
        Config::mb02 = 0;
        return;
    }
    ram_base_idx = base;

    // Clear SRAM pages
    for (int i = 0; i < MB02_NUM_PAGES; i++) {
        uint8_t* p = MemESP::ram[ram_base_idx + i].sync(0);
        if (p) memset(p, 0, MB02_PAGE_SIZE);
    }

    // Set up the MB-02 FDD instance (preserve existing disks on re-init)
    bool has_disks = false;
    for (int i = 0; i < 4; i++)
        if (ESPectrum::mb02_fdd.disk[i]) has_disks = true;
    if (!has_disks) {
        memset(&ESPectrum::mb02_fdd, 0, sizeof(rvmWD1793));
    }
    ESPectrum::mb02_fdd.wd2797_mode = true;
    rvmWD1793Reset(&ESPectrum::mb02_fdd);
    ESPectrum::mb02_fdd.wd2797_mode = true;

    // MB-02+ requires Z80-DMA on port #0B
    if (!Config::dma_mode) {
        Config::dma_mode = 1;
    }
    Z80DMA::mb02_deferred = true;

    enabled = true;
    Debug::log("MB02: enabled, ram[%d..%d], dma_mode=%d",
               ram_base_idx, ram_base_idx + MB02_NUM_PAGES - 1, Config::dma_mode);
}

void MB02::reset() {
    if (!enabled) return;

    rvmWD1793Reset(&ESPectrum::mb02_fdd);
    ESPectrum::mb02_fdd.wd2797_mode = true;

    paging_reg = MB02_EPROM_ENABLE;
    floppy_reg = 0xFF;
    motor_state = 0;
    write_enabled = false;

    applyMapping();
}

uint8_t* MB02::getPage(uint8_t page_idx) {
    if (page_idx >= MB02_NUM_PAGES || ram_base_idx < 0) return nullptr;
    return MemESP::ram[ram_base_idx + page_idx].sync(0);
}

void MB02::applyMapping() {
    bool sram_en = paging_reg & MB02_SRAM_ENABLE;
    bool eprom_en = paging_reg & MB02_EPROM_ENABLE;

    write_enabled = (paging_reg & MB02_WRITE_ENABLE) != 0;
    MemESP::mb02_write_gate = write_enabled || !enabled;

    if (sram_en && eprom_en) {
        MemESP::divmmc_mapped = false;
        MemESP::recoverPage0();
        return;
    }

    if (eprom_en) {
        memcpy(page0_composite, gb_rom_mb02boot, MB02_EPROM_SIZE);
        page0_composite[0x0021] = 0x18; // skip RAM test
        uint8_t* rom_ptr = MemESP::rom[MemESP::romInUse].direct();
        if (rom_ptr) {
            memcpy(page0_composite + MB02_EPROM_SIZE, rom_ptr + MB02_EPROM_SIZE,
                   0x2000 - MB02_EPROM_SIZE);
        }
        MemESP::page0_lo = page0_composite;
        MemESP::page0_hi = rom_ptr ? rom_ptr + 0x2000 : page0_composite;
        MemESP::divmmc_mapped = true;
        MemESP::divmmc_lo_dirty = nullptr;
        MemESP::divmmc_hi_dirty = nullptr;
        return;
    }

    if (sram_en) {
        uint8_t page = paging_reg & MB02_PAGE_MASK;
        uint8_t* p = getPage(page);
        if (!p) {
            MemESP::divmmc_mapped = false;
            MemESP::recoverPage0();
            return;
        }
        MemESP::page0_lo = p;
        MemESP::page0_hi = p + 0x2000;
        MemESP::divmmc_mapped = true;
        MemESP::divmmc_lo_dirty = nullptr;
        MemESP::divmmc_hi_dirty = nullptr;
        return;
    }

    MemESP::divmmc_mapped = false;
    MemESP::recoverPage0();
}

void MB02::writePort17(uint8_t data) {
    paging_reg = data;
    applyMapping();
}

void MB02::writePort13(uint8_t data) {
    floppy_reg = data;

    uint8_t drive = ESPectrum::mb02_fdd.diskS;
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

    motor_state = 0;
    if (data & 0x02) motor_state |= 0x01;
    if (data & 0x08) motor_state |= 0x02;
    if (data & 0x20) motor_state |= 0x04;
    if (data & 0x80) motor_state |= 0x08;
}

void MB02::signalDiskChange() {
    disk_changed = true;
}

uint8_t MB02::readPort13() {
    uint8_t result = 0;

    if (ESPectrum::mb02_fdd.control & kRVMWD177XDRQ)
        result |= 0x01;

    // Bit 1: DISK CHANGE — 1 = no change (stable), 0 = disk was changed
    // After hot-swap, temporarily return 0 (changed) until BS-DOS reads it
    if (!disk_changed)
        result |= 0x02;  // no change
    else
        disk_changed = false; // clear after first read (edge-triggered)

    if (ESPectrum::mb02_fdd.control & (kRVMWD177XINTRQ | kRVMWD177XFINTRQ))
        result |= 0x04;

    result |= 0x08; // HDIN = HD

    if (motor_state & 0x01) result |= 0x10;
    if (motor_state & 0x02) result |= 0x20;
    if (motor_state & 0x04) result |= 0x40;
    if (motor_state & 0x08) result |= 0x80;

    return result;
}

#endif // !PICO_RP2040
