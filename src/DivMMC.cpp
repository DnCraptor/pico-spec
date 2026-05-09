#include "DivMMC.h"

#if !PICO_RP2040

#include <cstring>
#include "MemESP.h"
#include "Config.h"
#include "Debug.h"
#include "roms.h"
extern "C" {
    #include "diskio.h"
}
extern int butter_pages;

// Static member definitions
bool DivMMC::enabled = false;
bool DivMMC::automap = false;
bool DivMMC::trap_after = false;
bool DivMMC::unmap_after = false;
bool DivMMC::conmem = false;
bool DivMMC::mapram = false;
uint8_t DivMMC::bank = 0;
uint8_t *DivMMC::esxdos_rom = nullptr;
uint8_t* DivMMC::bank_ptr[DIVMMC_NUM_BANKS] = {};
bool DivMMC::rom_loaded = false;
bool DivMMC::divsd_mode = false;
bool DivMMC::divide_mode = false;
bool DivMMC::sdhc_mode = false;
bool DivMMC::zc_enabled = false;
uint8_t DivMMC::zc_config = 0;

// Bank memory management
bool DivMMC::use_psram = false;
int8_t DivMMC::hi_slot = -1;
int8_t DivMMC::lo_slot = -1;
uint8_t* DivMMC::active_buf[DIVMMC_CACHE_SLOTS] = {};
int8_t DivMMC::active_bank[DIVMMC_CACHE_SLOTS] = {};
bool DivMMC::slot_dirty[DIVMMC_CACHE_SLOTS] = {};
uint8_t DivMMC::slot_lru[DIVMMC_CACHE_SLOTS] = {};
uint8_t DivMMC::lru_clock = 0;
FIL DivMMC::swap_file;
bool DivMMC::swap_open = false;

// IDE/ATA state (DivIDE)
uint8_t DivMMC::ide_feature = 0;
uint8_t DivMMC::ide_sector_count = 0;
uint8_t DivMMC::ide_sector = 0;
uint8_t DivMMC::ide_cylinder_lo = 0;
uint8_t DivMMC::ide_cylinder_hi = 0;
uint8_t DivMMC::ide_head = 0;
uint8_t DivMMC::ide_status = 0;
uint8_t DivMMC::ide_error = 0;
uint8_t DivMMC::ide_buffer[512];
int DivMMC::ide_data_index = -1;
bool DivMMC::ide_data_write = false;
uint32_t DivMMC::ide_hdf_data_offset[2] = {128, 128};
uint8_t DivMMC::ide_identity[2][106];
uint16_t DivMMC::ide_cylinders[2] = {0, 0};
uint16_t DivMMC::ide_heads[2] = {0, 0};
uint16_t DivMMC::ide_sectors[2] = {0, 0};

// SD protocol state
uint8_t DivMMC::mmc_last_command = 0;
int DivMMC::mmc_index_command = 0;
uint8_t DivMMC::mmc_params[6];
uint8_t DivMMC::mmc_r1 = 0;
bool DivMMC::mmc_cs_active = false;

int DivMMC::mmc_read_index = -1;
int DivMMC::mmc_write_index = -1;
int DivMMC::mmc_csd_index = -1;
int DivMMC::mmc_cid_index = -1;
int DivMMC::mmc_ocr_index = -1;

uint32_t DivMMC::mmc_read_address = 0;
uint32_t DivMMC::mmc_write_address = 0;

uint8_t DivMMC::mmc_sector_buf[512];
uint32_t DivMMC::mmc_sector_buf_addr = 0xFFFFFFFF;
bool DivMMC::mmc_sector_dirty = false;

FIL DivMMC::mmc_file[2];
bool DivMMC::mmc_file_open[2] = {false, false};
uint32_t DivMMC::mmc_file_size[2] = {0, 0};

// CSD: fabricated for .mmc image size (updated in init)
uint8_t DivMMC::mmc_csd[16] = {11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11};

// CID: "PI" OEM, "CO SD" product name
uint8_t DivMMC::mmc_cid[16] = {1,'P','I','C','O',' ','S','D',' ',1,1,1,1,1,127,128};

// OCR: standard capacity by default, updated to SDHC in init() for DivSD
uint8_t DivMMC::mmc_ocr[5] = {5,0,0,0,0};

void DivMMC::init() {
    enabled = (Config::esxdos != 0);
    divsd_mode = (Config::esxdos == 3);
    divide_mode = (Config::esxdos == 2);

    // Free previous allocations if switching modes or disabling
    if (!enabled) {
        // Flush pending writes and unmap before freeing memory
        flushWriteBuffer();
        automap = false;
        conmem = false;
        mapram = false;
        applyMapping(); // clears divmmc_mapped, restores page0

        // Close open image files
        for (int d = 0; d < 2; d++) {
            if (mmc_file_open[d]) {
                f_close(&mmc_file[d]);
                mmc_file_open[d] = false;
                mmc_file_size[d] = 0;
            }
        }

        // Bank cache slots are intentionally NOT freed: re-allocating 3 × 8 KB
        // contiguous blocks after a toggle cycle (DivSD→MB02→DivSD) fails on
        // tight-heap boards (ZERO2 without PSRAM, ~17 KB free) due to
        // fragmentation. Keep the slots and the swap file open across toggles —
        // they are reused on next enable. Slot bookkeeping is reset so the
        // cache acts empty.
        if (!use_psram) {
            for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
                active_bank[i] = -1;
                slot_dirty[i] = false;
                slot_lru[i] = 0;
            }
        }
        memset(bank_ptr, 0, sizeof(bank_ptr));
        esxdos_rom = nullptr;
        rom_loaded = false;
        return;
    }

    // Allocate DivMMC bank RAM: prefer butter PSRAM, fallback to swap
    size_t divmmc_total = DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE;
    size_t butter_used = (size_t)butter_pages * MEM_PG_SZ;
    size_t butter_free = butter_psram_size() > butter_used ? butter_psram_size() - butter_used : 0;

    if (butter_free >= divmmc_total) {
        use_psram = true;
        uint8_t* base = PSRAM_DATA + butter_used;
        for (int i = 0; i < DIVMMC_NUM_BANKS; i++)
            bank_ptr[i] = base + i * DIVMMC_BANK_SIZE;
        Debug::log("DivMMC: %d banks in butter PSRAM @ %p", DIVMMC_NUM_BANKS, base);
    } else {
        use_psram = false;
        // Allocate as many cache slots as possible (minimum 2)
        int allocated = 0;
        for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
            if (!active_buf[i]) active_buf[i] = (uint8_t*)malloc(DIVMMC_BANK_SIZE);
            if (active_buf[i]) allocated++;
            active_bank[i] = -1;
            slot_dirty[i] = false;
            slot_lru[i] = 0;
        }
        lru_clock = 0;
        if (allocated < 2) {
            for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
                free(active_buf[i]); active_buf[i] = nullptr;
            }
            enabled = false; return;
        }
        if (!swap_open) {
            f_unlink("/tmp/divmmc-pico-spec.swap");
            FRESULT fr = f_open(&swap_file, "/tmp/divmmc-pico-spec.swap", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
            if (fr != FR_OK) {
                for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
                    free(active_buf[i]); active_buf[i] = nullptr;
                }
                enabled = false; return;
            }
            swap_open = true;
        }
        memset(bank_ptr, 0, sizeof(bank_ptr));
        Debug::log("DivMMC: %d banks via swap file (%d cache slots)", DIVMMC_NUM_BANKS, allocated);
    }
    clearAllBanks();

    // Point ROM directly to flash (no heap allocation needed)
    const char* mode_name;
    if (Config::esxdos == 2) {
        esxdos_rom = (uint8_t *)gb_rom_esxide;
        mode_name = "DivIDE";
    } else {
        esxdos_rom = (uint8_t *)gb_rom_esxdos;
        mode_name = divsd_mode ? "DivSD" : "DivMMC";
    }
    rom_loaded = true;
    Debug::log("%s ROM verify: %02X %02X %02X %02X %02X %02X %02X @ %p",
        mode_name, esxdos_rom[0], esxdos_rom[1], esxdos_rom[2], esxdos_rom[3],
        esxdos_rom[4], esxdos_rom[5], esxdos_rom[6], esxdos_rom);

    // Close previous images if open
    for (int d = 0; d < 2; d++) {
        if (mmc_file_open[d]) {
            if (d == 0) flushWriteBuffer();
            f_close(&mmc_file[d]);
            mmc_file_open[d] = false;
            mmc_file_size[d] = 0;
        }
    }

    if (divsd_mode && enabled) {
        // DivSD: raw SD access — SDHC mode (sector-addressed)
        sdhc_mode = true;
        DWORD sector_count = 0;
        disk_ioctl(0, GET_SECTOR_COUNT, &sector_count);
        mmc_file_size[0] = (uint32_t)((uint64_t)sector_count * 512 > 0xFFFFFFFF ? 0xFFFFFFFF : sector_count * 512);
        buildCSD_real(sector_count);
        // SDHC OCR: power_up done (bit31) + CCS=1 (bit30) + voltage
        mmc_ocr[0] = 0xC0;  // bits 31:24 — power_up=1, CCS=1
        mmc_ocr[1] = 0xFF;  // bits 23:16 — all voltages
        mmc_ocr[2] = 0x80;  // bits 15:8
        mmc_ocr[3] = 0x00;  // bits 7:0
        mmc_ocr[4] = 0x00;
        Debug::log("%s: raw SD, %lu sectors, SDHC mode", mode_name, (unsigned long)sector_count);
        Debug::log("CSD: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            mmc_csd[0], mmc_csd[1], mmc_csd[2], mmc_csd[3], mmc_csd[4],
            mmc_csd[5], mmc_csd[6], mmc_csd[7], mmc_csd[8], mmc_csd[9], mmc_csd[10]);
        Debug::log("OCR: %02X %02X %02X %02X %02X", mmc_ocr[0], mmc_ocr[1], mmc_ocr[2], mmc_ocr[3], mmc_ocr[4]);
    } else if (divide_mode && enabled) {
        sdhc_mode = false;
        // Reset OCR to non-SDHC (DivIDE uses IDE not SPI, but keep clean state)
        mmc_ocr[0] = 5; mmc_ocr[1] = 0; mmc_ocr[2] = 0; mmc_ocr[3] = 0; mmc_ocr[4] = 0;
        // DivIDE: open HDF images (hd0=master, hd1=slave)
        const char* defaults[2] = {"/esxdos.hdf", ""};
        for (int d = 0; d < 2; d++) {
            const char* image_path = Config::esxdos_hdf_image[d].empty() ? defaults[d] : Config::esxdos_hdf_image[d].c_str();
            if (image_path[0] == '\0') continue; // no slave configured
            FRESULT fr = f_open(&mmc_file[d], image_path, FA_READ | FA_WRITE);
            if (fr == FR_OK) {
                mmc_file_open[d] = true;
                mmc_file_size[d] = f_size(&mmc_file[d]);
                Debug::log("%s hd%d: opened %s (%u bytes)", mode_name, d, image_path, mmc_file_size[d]);
            } else {
                fr = f_open(&mmc_file[d], image_path, FA_READ);
                if (fr == FR_OK) {
                    mmc_file_open[d] = true;
                    mmc_file_size[d] = f_size(&mmc_file[d]);
                    Debug::log("%s hd%d: opened %s read-only (%u bytes)", mode_name, d, image_path, mmc_file_size[d]);
                } else {
                    Debug::log("%s hd%d: %s not found (err=%d)", mode_name, d, image_path, fr);
                }
            }
        }
        if (mmc_file_open[0]) buildCSD();
    } else if (enabled) {
        // DivMMC: report SDHC (CCS=1) so ZP4 uses sector-addressing.
        // ESXDOS itself adapts to either; .mmc file is read by sector either way.
        sdhc_mode = true;
        mmc_ocr[0] = 0xC0; mmc_ocr[1] = 0xFF; mmc_ocr[2] = 0x80; mmc_ocr[3] = 0x00; mmc_ocr[4] = 0x00;
        // DivMMC: open .mmc image (shared hd0 slot with DivIDE).
        const char* default_path = "/esxdos.mmc";
        const char* image_path = Config::esxdos_hdf_image[0].empty() ? default_path : Config::esxdos_hdf_image[0].c_str();
        FRESULT fr = f_open(&mmc_file[0], image_path, FA_READ | FA_WRITE);
        if (fr == FR_OK) {
            mmc_file_open[0] = true;
            mmc_file_size[0] = f_size(&mmc_file[0]);
            buildCSD_real(mmc_file_size[0] / 512);
            Debug::log("%s: opened %s (%u bytes, SDHC)", mode_name, image_path, mmc_file_size[0]);
        } else {
            fr = f_open(&mmc_file[0], image_path, FA_READ);
            if (fr == FR_OK) {
                mmc_file_open[0] = true;
                mmc_file_size[0] = f_size(&mmc_file[0]);
                buildCSD_real(mmc_file_size[0] / 512);
                Debug::log("%s: opened %s read-only (%u bytes, SDHC)", mode_name, image_path, mmc_file_size[0]);
            } else {
                Debug::log("%s: %s not found (err=%d)", mode_name, image_path, fr);
            }
        }
    }

    // Parse HDF headers for DivIDE
    if (divide_mode) {
        for (int d = 0; d < 2; d++) {
            if (!mmc_file_open[d]) continue;
            uint8_t hdr[128];
            UINT br;
            f_lseek(&mmc_file[d], 0);
            f_read(&mmc_file[d], hdr, 128, &br);
            if (br == 128 && memcmp(hdr, "RS-IDE", 6) == 0 && hdr[6] == 0x1A) {
                ide_hdf_data_offset[d] = hdr[9] | (hdr[10] << 8);
                memcpy(ide_identity[d], &hdr[0x16], 106);
                ide_cylinders[d] = ide_identity[d][2] | (ide_identity[d][3] << 8);
                ide_heads[d]     = ide_identity[d][6] | (ide_identity[d][7] << 8);
                ide_sectors[d]   = ide_identity[d][12] | (ide_identity[d][13] << 8);
                Debug::log("%s hd%d: HDF C=%u H=%u S=%u data@%u",
                    mode_name, d, ide_cylinders[d], ide_heads[d], ide_sectors[d], ide_hdf_data_offset[d]);
            } else {
                Debug::log("%s hd%d: invalid HDF header", mode_name, d);
            }
        }
    }

    Debug::log("%s: ESXDOS ROM initialized (built-in)", mode_name);
    reset();
}

void DivMMC::reopenFiles() {
    if (!enabled) return;
    // Reopen MMC/HDF image files
    for (int d = 0; d < 2; d++) {
        if (mmc_file_open[d]) {
            FSIZE_t pos = f_tell(&mmc_file[d]);
            f_close(&mmc_file[d]);
            const char* path;
            if (enabled == 2 || enabled == 3) { // DivIDE/DivSD
                const char* defaults[] = {"/esxdos.hdf", ""};
                path = Config::esxdos_hdf_image[d].empty() ? defaults[d] : Config::esxdos_hdf_image[d].c_str();
            } else {
                // DivMMC: only hd0 is used.
                if (d != 0) continue;
                path = Config::esxdos_hdf_image[0].empty() ? "/esxdos.mmc" : Config::esxdos_hdf_image[0].c_str();
            }
            if (path[0] && f_open(&mmc_file[d], path, FA_READ | FA_WRITE) == FR_OK) {
                f_lseek(&mmc_file[d], pos);
            } else if (path[0] && f_open(&mmc_file[d], path, FA_READ) == FR_OK) {
                f_lseek(&mmc_file[d], pos);
            } else {
                mmc_file_open[d] = false;
            }
        }
    }
    // Reopen swap file
    if (swap_open) {
        f_close(&swap_file);
        if (f_open(&swap_file, "/tmp/divmmc-pico-spec.swap", FA_READ | FA_WRITE) != FR_OK) {
            swap_open = false;
        }
    }
}

void DivMMC::reset() {
    automap = false;
    trap_after = false;
    unmap_after = false;
    conmem = false;
    mapram = false;
    bank = 0;

    // Reset SD protocol state
    mmc_last_command = 0;
    mmc_index_command = 0;
    mmc_r1 = 1; // Start in idle state
    mmc_cs_active = false;
    mmc_read_index = -1;
    mmc_write_index = -1;
    mmc_csd_index = -1;
    mmc_cid_index = -1;
    mmc_ocr_index = -1;
    mmc_sector_buf_addr = 0xFFFFFFFF;
    mmc_sector_dirty = false;

    // Reset IDE state
    ide_feature = 0;
    ide_sector_count = 0;
    ide_sector = 0;
    ide_cylinder_lo = 0;
    ide_cylinder_hi = 0;
    ide_head = 0;
    ide_error = 0;
    ide_status = (mmc_file_open[0] || mmc_file_open[1]) ? 0x40 : 0x00; // DRDY if any disk present
    ide_data_index = -1;
    ide_data_write = false;

    applyMapping();
}

void DivMMC::applyMapping() {
    if (conmem || automap) {
        uint8_t b = bank & (DIVMMC_NUM_BANKS - 1);
        if (mapram) {
            materialize(3);
            MemESP::page0_lo = bank_ptr[3];
        } else {
            MemESP::page0_lo = esxdos_rom;
        }
        materialize(b);
        MemESP::page0_hi = bank_ptr[b];
        MemESP::divmmc_mapped = true;
        // Track slot indices for dirty marking
        if (!use_psram) {
            hi_slot = -1;
            lo_slot = -1;
            for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
                if (active_bank[i] == b) hi_slot = i;
                if (mapram && active_bank[i] == 3) lo_slot = i;
            }
            MemESP::divmmc_hi_dirty = (hi_slot >= 0) ? &slot_dirty[hi_slot] : nullptr;
            MemESP::divmmc_lo_dirty = (lo_slot >= 0) ? &slot_dirty[lo_slot] : nullptr;
        }
    } else {
        MemESP::divmmc_mapped = false;
        MemESP::recoverPage0();
        hi_slot = -1;
        lo_slot = -1;
        MemESP::divmmc_hi_dirty = nullptr;
        MemESP::divmmc_lo_dirty = nullptr;
    }
}

// Build CSD register based on .mmc file size
void DivMMC::buildCSD() {
    // Simple CSD v1.0 encoding
    // We encode capacity so ESXDOS can compute total sectors
    // capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^(READ_BL_LEN)
    // For 64MB: C_SIZE=4095, C_SIZE_MULT=7, READ_BL_LEN=9 (512 bytes)
    // That gives (4096) * (512) * (512) = 1GB max
    // For simplicity use fixed values that work for <=1GB images

    memset(mmc_csd, 0, 16);

    uint32_t sectors = mmc_file_size[0] / 512;
    // C_SIZE_MULT = 7, READ_BL_LEN = 9
    // capacity = (C_SIZE+1) * 512 * 512
    // C_SIZE = sectors / 512 - 1
    uint32_t c_size = (sectors / 512);
    if (c_size > 0) c_size--;
    if (c_size > 4095) c_size = 4095;

    // CSD v1.0 format (simplified)
    mmc_csd[0] = 0x00;  // CSD structure v1.0
    mmc_csd[1] = 0x00;
    mmc_csd[2] = 0x00;
    mmc_csd[3] = 0x00;
    mmc_csd[4] = 0x00;
    mmc_csd[5] = 0x59;  // READ_BL_LEN = 9
    mmc_csd[6] = ((c_size >> 10) & 0x03);           // C_SIZE [11:10]
    mmc_csd[7] = ((c_size >> 2) & 0xFF);             // C_SIZE [9:2]
    mmc_csd[8] = ((c_size & 0x03) << 6) | 0x00;     // C_SIZE [1:0] + VDD_R_CURR_MIN
    mmc_csd[9] = 0x03 | (7 << 2);                    // VDD_W_CURR_MAX + C_SIZE_MULT[2:1]
    mmc_csd[10] = 0x80 | (1 << 6);                   // C_SIZE_MULT[0] + ...
    mmc_csd[11] = 0x00;
    mmc_csd[12] = 0x00;
    mmc_csd[13] = 0x00;
    mmc_csd[14] = 0x00;
    mmc_csd[15] = 0x00;
}

// Build CSD v2.0 (SDHC) for real SD card
void DivMMC::buildCSD_real(uint32_t sector_count) {
    memset(mmc_csd, 0, 16);
    // CSD v2.0: capacity = (C_SIZE + 1) * 512KB
    // C_SIZE = sector_count / 1024 - 1
    uint32_t c_size = sector_count / 1024;
    if (c_size > 0) c_size--;
    mmc_csd[0] = 0x40;  // CSD structure v2.0
    mmc_csd[5] = 0x59;  // READ_BL_LEN = 9
    mmc_csd[7] = (c_size >> 16) & 0x3F;
    mmc_csd[8] = (c_size >> 8) & 0xFF;
    mmc_csd[9] = c_size & 0xFF;
}

// Bank memory management
void DivMMC::clearAllBanks() {
    if (use_psram) {
        // All banks are contiguous in butter PSRAM
        memset(bank_ptr[0], 0, DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE);
    } else {
        // Clear active buffers
        for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
            if (active_buf[i]) memset(active_buf[i], 0, DIVMMC_BANK_SIZE);
            active_bank[i] = -1;
            slot_dirty[i] = false;
            slot_lru[i] = 0;
        }
        lru_clock = 0;
        memset(bank_ptr, 0, sizeof(bank_ptr));
        // Zero the swap file
        if (swap_open) {
            uint8_t zeros[256];
            memset(zeros, 0, sizeof(zeros));
            f_lseek(&swap_file, 0);
            UINT bw;
            for (size_t i = 0; i < DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE; i += sizeof(zeros))
                f_write(&swap_file, zeros, sizeof(zeros), &bw);
        }
    }
}

void DivMMC::touch(int slot) {
    slot_lru[slot] = ++lru_clock;
}

int DivMMC::find_victim(uint8_t exclude_bank) {
    // Find free slot first
    for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
        if (!active_buf[i]) continue; // not allocated
        if (active_bank[i] == -1) return i;
    }
    // Find LRU slot, excluding the given bank
    int best = -1;
    uint8_t best_lru = 255;
    for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
        if (!active_buf[i]) continue;
        if (active_bank[i] == exclude_bank) continue;
        if (best == -1 || slot_lru[i] < best_lru) {
            best = i;
            best_lru = slot_lru[i];
        }
    }
    return best;
}

void DivMMC::evict(int slot) {
    if (active_bank[slot] < 0) return;
    // Only write back if dirty
    if (slot_dirty[slot]) {
        UINT bw;
        f_lseek(&swap_file, (FSIZE_t)active_bank[slot] * DIVMMC_BANK_SIZE);
        f_write(&swap_file, active_buf[slot], DIVMMC_BANK_SIZE, &bw);
        slot_dirty[slot] = false;
    }
    bank_ptr[active_bank[slot]] = nullptr;
    active_bank[slot] = -1;
}

void DivMMC::materialize(uint8_t bank_idx) {
    if (use_psram) return; // butter mode: always available
    if (bank_ptr[bank_idx]) {
        // Already cached — find slot and touch LRU
        for (int i = 0; i < DIVMMC_CACHE_SLOTS; i++) {
            if (active_bank[i] == bank_idx) { touch(i); break; }
        }
        return;
    }

    // Find slot: prefer free, then LRU (excluding the other needed bank)
    uint8_t cur = bank & (DIVMMC_NUM_BANKS - 1);
    uint8_t other_needed = (bank_idx == cur) ? 3 : cur;
    int slot = find_victim(other_needed);
    if (slot < 0) return; // shouldn't happen

    evict(slot);

    // Load from swap
    UINT br;
    f_lseek(&swap_file, (FSIZE_t)bank_idx * DIVMMC_BANK_SIZE);
    f_read(&swap_file, active_buf[slot], DIVMMC_BANK_SIZE, &br);
    bank_ptr[bank_idx] = active_buf[slot];
    active_bank[slot] = bank_idx;
    slot_dirty[slot] = false;
    touch(slot);
}

// Read a byte from image/SD, using sector cache
// address is always a byte offset (even in SDHC mode — converted at CMD level)
uint8_t DivMMC::readByte(uint32_t address) {
    if (!divsd_mode && !mmc_file_open[0]) return 0xFF;

    uint32_t sector = address / 512;
    uint32_t offset = address % 512;

    if (sector != mmc_sector_buf_addr) {
        flushWriteBuffer();
        loadSector(sector);
        mmc_sector_buf_addr = sector;
    }

    return mmc_sector_buf[offset];
}

// Write a byte to image/SD, using sector cache
void DivMMC::writeByte(uint32_t address, uint8_t value) {
    if (!divsd_mode && !mmc_file_open[0]) return;

    uint32_t sector = address / 512;
    uint32_t offset = address % 512;

    if (sector != mmc_sector_buf_addr) {
        flushWriteBuffer();
        loadSector(sector);
        mmc_sector_buf_addr = sector;
    }

    mmc_sector_buf[offset] = value;
    mmc_sector_dirty = true;
}

// Flush dirty sector buffer
void DivMMC::flushWriteBuffer() {
    if (!mmc_sector_dirty || mmc_sector_buf_addr == 0xFFFFFFFF) return;
    storeSector(mmc_sector_buf_addr);
    mmc_sector_dirty = false;
}

// Read one sector into mmc_sector_buf, with synthetic MBR for DivMMC images.
// In DivMMC mode the .mmc file is "superfloppy" formatted (FAT16 starts at
// sector 0). ZP4 expects an MBR with a partition table at sector 0, so we
// synthesize one and shift the .mmc data by +1 sector.
void DivMMC::loadSector(uint32_t sector) {
    if (divsd_mode) {
        disk_read(0, mmc_sector_buf, sector, 1);
        return;
    }
    if (!mmc_file_open[0]) {
        memset(mmc_sector_buf, 0, 512);
        return;
    }
    // Read the sector directly from the .mmc file. The image is presented
    // byte-for-byte: sector 0 holds the FAT16 BPB (superfloppy), there is no MBR.
    UINT br;
    f_lseek(&mmc_file[0], (FSIZE_t)sector * 512);
    f_read(&mmc_file[0], mmc_sector_buf, 512, &br);
    if (br < 512) memset(mmc_sector_buf + br, 0, 512 - br);
}

void DivMMC::storeSector(uint32_t sector) {
    if (divsd_mode) {
        disk_write(0, mmc_sector_buf, sector, 1);
        return;
    }
    if (!mmc_file_open[0]) return;
    UINT bw;
    f_lseek(&mmc_file[0], (FSIZE_t)sector * 512);
    f_write(&mmc_file[0], mmc_sector_buf, 512, &bw);
    f_sync(&mmc_file[0]);
}

// Port 0xE7 write — chip select
// Different DivMMC clones drive CS via different bits:
//   Standard DivMMC: bit 0 (active-low)
//   Some clones (ZP4-style): bit 1 (active-low)
// Treat CS as active if EITHER bit 0 or bit 1 is 0.
void DivMMC::mmc_cs(uint8_t value) {
    bool was_active = mmc_cs_active;
    mmc_cs_active = (value & 0x01) == 0;  // Active low

    // Reset protocol state on CS change (like ZEsarUX)
    mmc_r1 = 1;
    mmc_last_command = 0;
    mmc_index_command = 0;
    mmc_read_index = -1;
    mmc_write_index = -1;
    mmc_csd_index = -1;
    mmc_cid_index = -1;
    mmc_ocr_index = -1;
}

// Port 0xEB read — SD protocol response
uint8_t DivMMC::mmc_read() {
    if (!mmc_file_open[0] && !divsd_mode) return 0xFF;
    if (!mmc_cs_active) return 0xFF;

    // If not idle, return R1
    if ((mmc_r1 & 1) == 0) {
        return mmc_r1;
    }

    uint8_t value = 0xFF;

    switch (mmc_last_command) {
        case 0x00:
            // After CS — no command yet, idle bus = 0xFF
            return 0xFF;

        case 0x40: // CMD0 GO_IDLE_STATE — return R1=01 once, then idle
            mmc_last_command = 0;
            return 1;

        case 0x48: // CMD8 SEND_IF_COND
            // R7 response: R1 + 4-byte echo of command argument (voltage + check pattern).
            // SDv2 host expects: 0x01, 0x00, 0x00, 0x01, 0xAA — then FF until next command.
            switch (mmc_index_command) {
                case 0:  mmc_index_command = 1; return 0x01;             // R1 = idle
                case 1:  mmc_index_command = 2; return mmc_params[0];
                case 2:  mmc_index_command = 3; return mmc_params[1];
                case 3:  mmc_index_command = 4; return mmc_params[2];
                case 4:  mmc_index_command = 0; mmc_last_command = 0;
                         return mmc_params[3];                            // last byte → reset
                default: return 0xFF;
            }

        case 0x49: // CMD9 SEND_CSD
            if (mmc_csd_index >= 0) {
                if (mmc_csd_index == 0) value = 0xFF;       // NCR
                if (mmc_csd_index == 1) value = 0;           // R1 = OK
                if (mmc_csd_index == 2) value = 0xFE;        // Data token
                if (mmc_csd_index >= 3 && mmc_csd_index <= 18)
                    value = mmc_csd[mmc_csd_index - 3];
                if (mmc_csd_index == 19 || mmc_csd_index == 20)
                    value = 0xFF; // CRC
                mmc_csd_index++;
                if (mmc_csd_index == 21) mmc_csd_index = -1;
                return value;
            }
            return 0xFF;

        case 0x4A: // CMD10 SEND_CID
            if (mmc_cid_index >= 0) {
                if (mmc_cid_index == 0) value = 0xFF;
                if (mmc_cid_index == 1) value = 0;
                if (mmc_cid_index == 2) value = 0xFE;
                if (mmc_cid_index >= 3 && mmc_cid_index <= 18)
                    value = mmc_cid[mmc_cid_index - 3];
                if (mmc_cid_index == 19 || mmc_cid_index == 20)
                    value = 0xFF;
                mmc_cid_index++;
                if (mmc_cid_index == 21) mmc_cid_index = -1;
                return value;
            }
            return 0xFF;

        case 0x4C: // CMD12 STOP_TRANSMISSION — card is in transmission state,
                   // not idle; return R1=00 (ready, not idle)
            mmc_last_command = 0;
            return 0;

        case 0x51: // CMD17 READ_SINGLE_BLOCK
            if (mmc_read_index >= 0) {
                if (mmc_read_index == 0) value = 0xFF;       // NCR
                if (mmc_read_index == 1) value = 0;           // R1 = OK
                if (mmc_read_index == 2) value = 0xFE;        // Data token
                if (mmc_read_index >= 3 && mmc_read_index <= 514) {
                    if (sdhc_mode)
                        value = mmc_sector_buf[mmc_read_index - 3];
                    else
                        value = readByte(mmc_read_address + mmc_read_index - 3);
                }
                if (mmc_read_index == 515 || mmc_read_index == 516)
                    value = 0xFF; // CRC
                mmc_read_index++;
                if (mmc_read_index == 516) mmc_read_index = -1;
                return value;
            }
            return 0xFF;

        case 0x52: // CMD18 READ_MULTIPLE_BLOCK
            if (mmc_read_index >= 0) {
                if (mmc_read_index == 0) value = 0xFF;
                if (mmc_read_index == 1) value = 0;
                if (mmc_read_index == 2) value = 0xFE;
                if (mmc_read_index >= 3 && mmc_read_index <= 514) {
                    if (sdhc_mode)
                        value = mmc_sector_buf[mmc_read_index - 3];
                    else
                        value = readByte(mmc_read_address + mmc_read_index - 3);
                }
                mmc_read_index++;
                if (mmc_read_index == 516) mmc_read_index = -1;
                // Auto-advance to next sector
                if (mmc_read_index == -1) {
                    mmc_read_index = 0;
                    mmc_read_address += sdhc_mode ? 1 : 512;
                    if (sdhc_mode) {
                        loadSector(mmc_read_address);
                        mmc_sector_buf_addr = mmc_read_address;
                    }
                }
                return value;
            }
            return 0xFF;

        case 0x58: // CMD24 WRITE_BLOCK
            if (mmc_write_index >= 0) {
                if (mmc_write_index == 0) value = 0xFF;       // NCR
                if (mmc_write_index == 1) value = 0;           // R1
                if (mmc_write_index == 2) value = 0xFF;
                if (mmc_write_index == 3) value = 0xFF;
                if (mmc_write_index >= 4) value = 0x05;        // Data accepted
                mmc_write_index++;
                return value;
            }
            return 0xFF;

        case 0x77: // CMD55 APP_CMD (prefix for ACMD) — R1=00 once
            mmc_last_command = 0;
            return 0;

        case 0x69: // ACMD41 SD_SEND_OP_COND — R1=00 (init complete) once
            mmc_last_command = 0;
            return 0;

        case 0x7A: // CMD58 READ_OCR
            if (mmc_ocr_index >= 0) {
                if (mmc_ocr_index == 0) value = 0xFF;
                if (mmc_ocr_index == 1) value = 0;
                if (mmc_ocr_index >= 2 && mmc_ocr_index <= 6)
                    value = mmc_ocr[mmc_ocr_index - 2];
                if (mmc_ocr_index == 7 || mmc_ocr_index == 8)
                    value = 0xFF;
                mmc_ocr_index++;
                if (mmc_ocr_index == 9) mmc_ocr_index = -1;
                return value;
            }
            return 0xFF;

        default:
            // Unknown command — idle bus
            mmc_last_command = 0;
            return 0xFF;
    }

    return 0xFF;
}

// Port 0xEB write — SD protocol command/data
void DivMMC::mmc_write(uint8_t value) {
    if (!mmc_file_open[0] && !divsd_mode) return;
    if (!mmc_cs_active) return;

    if (mmc_index_command == 0) {
        // SD command frame: bits 7:6 must be 01b. Anything else is a fill /
        // wake-up byte (typically 0xFF) that the card silently discards.
        if ((value & 0xC0) != 0x40) return;
        // Receive command byte
        mmc_last_command = value;
        mmc_index_command++;
        return;
    }

    // Receive parameter bytes
    switch (mmc_last_command) {
        case 0x40: // CMD0 GO_IDLE_STATE
            if (mmc_index_command == 5) {
                mmc_r1 = 1; // Idle
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        case 0x48: // CMD8 SEND_IF_COND
            if (mmc_index_command >= 1 && mmc_index_command <= 4) {
                mmc_params[mmc_index_command - 1] = value; // echo back in R7
            }
            if (mmc_index_command == 5) {
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        case 0x49: // CMD9 SEND_CSD
            if (mmc_index_command == 5) {
                mmc_csd_index = 0;
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        case 0x4A: // CMD10 SEND_CID
            if (mmc_index_command == 5) {
                mmc_cid_index = 0;
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        case 0x4C: // CMD12 STOP_TRANSMISSION
            if (mmc_index_command == 5) {
                mmc_r1 = 1;
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        case 0x51: // CMD17 READ_SINGLE_BLOCK
            mmc_params[mmc_index_command - 1] = value;
            mmc_index_command++;
            if (mmc_index_command == 6) {
                mmc_index_command = 0;
                mmc_read_address = ((uint32_t)mmc_params[0] << 24) |
                                   ((uint32_t)mmc_params[1] << 16) |
                                   ((uint32_t)mmc_params[2] << 8) |
                                   mmc_params[3];
                if (sdhc_mode) {
                    flushWriteBuffer();
                    loadSector(mmc_read_address);
                    mmc_sector_buf_addr = mmc_read_address;
                }
                mmc_read_index = 0;
            }
            break;

        case 0x52: // CMD18 READ_MULTIPLE_BLOCK
            mmc_params[mmc_index_command - 1] = value;
            mmc_index_command++;
            if (mmc_index_command == 6) {
                mmc_index_command = 0;
                mmc_read_address = ((uint32_t)mmc_params[0] << 24) |
                                   ((uint32_t)mmc_params[1] << 16) |
                                   ((uint32_t)mmc_params[2] << 8) |
                                   mmc_params[3];
                if (sdhc_mode) {
                    flushWriteBuffer();
                    loadSector(mmc_read_address);
                    mmc_sector_buf_addr = mmc_read_address;
                }
                mmc_read_index = 0;
            }
            break;

        case 0x58: { // CMD24 WRITE_BLOCK
            #define WRITE_BLOCK_OFFSET 5
            if (mmc_index_command < 5) {
                mmc_params[mmc_index_command - 1] = value;
            }
            if (mmc_index_command == WRITE_BLOCK_OFFSET) {
                mmc_write_address = ((uint32_t)mmc_params[0] << 24) |
                                    ((uint32_t)mmc_params[1] << 16) |
                                    ((uint32_t)mmc_params[2] << 8) |
                                    mmc_params[3];
                mmc_write_index = 0;
                if (sdhc_mode) {
                    // Pre-read sector for partial writes; sector_buf_addr = sector number
                    loadSector(mmc_write_address);
                    mmc_sector_buf_addr = mmc_write_address;
                }
            }
            // After gap byte and data token, receive 512 data bytes
            if (mmc_index_command >= WRITE_BLOCK_OFFSET + 2 &&
                mmc_index_command <= WRITE_BLOCK_OFFSET + 2 + 511) {
                int byte_idx = mmc_index_command - (WRITE_BLOCK_OFFSET + 2);
                if (sdhc_mode) {
                    mmc_sector_buf[byte_idx] = value;
                    mmc_sector_dirty = true;
                } else {
                    writeByte(mmc_write_address + byte_idx, value);
                }
            }
            mmc_index_command++;
            if (mmc_index_command == WRITE_BLOCK_OFFSET + 2 + 512) {
                if (sdhc_mode) {
                    storeSector(mmc_write_address);
                    mmc_sector_dirty = false;
                } else {
                    flushWriteBuffer();
                }
            }
            #undef WRITE_BLOCK_OFFSET
            break;
        }

        case 0x77: // CMD55 APP_CMD (prefix for ACMD)
        case 0x69: // ACMD41 SD_SEND_OP_COND
            if (mmc_index_command == 5) {
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        case 0x7A: // CMD58 READ_OCR
            if (mmc_index_command == 5) {
                mmc_ocr_index = 0;
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;

        default:
            // Unknown command — consume 5 parameter bytes then reset
            if (mmc_index_command >= 5) {
                mmc_index_command = 0;
            } else {
                mmc_index_command++;
            }
            break;
    }
}

// ============================================================
// IDE/ATA emulation for DivIDE
// ============================================================

#define IDE_STATUS_BSY   0x80
#define IDE_STATUS_DRDY  0x40
#define IDE_STATUS_DRQ   0x08
#define IDE_STATUS_ERR   0x01
#define IDE_ERROR_ABRT   0x04
#define IDE_ERROR_IDNF   0x10
#define IDE_LBA_BIT      0x40

uint32_t DivMMC::ide_lba() {
    if (ide_head & IDE_LBA_BIT) {
        // LBA mode
        return ((uint32_t)(ide_head & 0x0F) << 24) |
               ((uint32_t)ide_cylinder_hi << 16) |
               ((uint32_t)ide_cylinder_lo << 8) |
               ide_sector;
    } else {
        // CHS mode
        uint16_t cyl = (ide_cylinder_hi << 8) | ide_cylinder_lo;
        uint8_t head = ide_head & 0x0F;
        int d = ide_drive();
        return ((uint32_t)cyl * ide_heads[d] + head) * ide_sectors[d] + (ide_sector - 1);
    }
}

void DivMMC::ide_read_sector() {
    int d = ide_drive();
    if (!mmc_file_open[d]) {
        ide_error = IDE_ERROR_IDNF;
        ide_status = IDE_STATUS_DRDY | IDE_STATUS_ERR;
        return;
    }
    uint32_t lba = ide_lba();
    FSIZE_t pos = (FSIZE_t)ide_hdf_data_offset[d] + (FSIZE_t)lba * 512;
    UINT br;
    f_lseek(&mmc_file[d], pos);
    f_read(&mmc_file[d], ide_buffer, 512, &br);
    if (br < 512) memset(ide_buffer + br, 0xFF, 512 - br);
    ide_data_index = 0;
    ide_data_write = false;
    ide_status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
}

void DivMMC::ide_write_sector_done() {
    int d = ide_drive();
    if (!mmc_file_open[d]) return;
    uint32_t lba = ide_lba();
    FSIZE_t pos = (FSIZE_t)ide_hdf_data_offset[d] + (FSIZE_t)lba * 512;
    UINT bw;
    f_lseek(&mmc_file[d], pos);
    f_write(&mmc_file[d], ide_buffer, 512, &bw);
    f_sync(&mmc_file[d]);
}

void DivMMC::ide_execute_command(uint8_t cmd) {
    ide_error = 0;
    ide_status = IDE_STATUS_DRDY;

    switch (cmd) {
        case 0x20: // READ SECTOR (with retry)
        case 0x21: // READ SECTOR (no retry)
            ide_read_sector();
            break;

        case 0x30: // WRITE SECTOR (with retry)
        case 0x31: // WRITE SECTOR (no retry)
            ide_data_index = 0;
            ide_data_write = true;
            ide_status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
            break;

        case 0x91: { // INITIALIZE DEVICE PARAMETERS
            int d = ide_drive();
            uint8_t new_heads = (ide_head & 0x0F) + 1;
            uint8_t new_sectors = ide_sector_count;
            if (new_heads && new_sectors) {
                uint32_t total = (uint32_t)ide_cylinders[d] * ide_heads[d] * ide_sectors[d];
                ide_heads[d] = new_heads;
                ide_sectors[d] = new_sectors;
                ide_cylinders[d] = total / (ide_heads[d] * ide_sectors[d]);
            }
            break;
        }

        case 0xEC: { // IDENTIFY DEVICE
            int d = ide_drive();
            if (!mmc_file_open[d]) {
                ide_error = IDE_ERROR_ABRT;
                ide_status = IDE_STATUS_DRDY | IDE_STATUS_ERR;
                break;
            }
            memset(ide_buffer, 0, 512);
            memcpy(ide_buffer, ide_identity[d], 106);
            ide_buffer[106] = 0x07; ide_buffer[107] = 0x00;
            ide_buffer[108] = ide_cylinders[d] & 0xFF; ide_buffer[109] = ide_cylinders[d] >> 8;
            ide_buffer[110] = ide_heads[d] & 0xFF; ide_buffer[111] = ide_heads[d] >> 8;
            ide_buffer[112] = ide_sectors[d] & 0xFF; ide_buffer[113] = ide_sectors[d] >> 8;
            uint32_t cap = (uint32_t)ide_cylinders[d] * ide_heads[d] * ide_sectors[d];
            ide_buffer[114] = cap & 0xFF;
            ide_buffer[115] = (cap >> 8) & 0xFF;
            ide_buffer[116] = (cap >> 16) & 0xFF;
            ide_buffer[117] = (cap >> 24) & 0xFF;
            // Word 60-61: total LBA sectors (same as capacity for now)
            ide_buffer[120] = cap & 0xFF;
            ide_buffer[121] = (cap >> 8) & 0xFF;
            ide_buffer[122] = (cap >> 16) & 0xFF;
            ide_buffer[123] = (cap >> 24) & 0xFF;

            ide_data_index = 0;
            ide_data_write = false;
            ide_sector_count = 0; // prevent read-next-sector
            ide_status = IDE_STATUS_DRDY | IDE_STATUS_DRQ;
            break;
        }

        default:
            // Unknown command — abort
            ide_error = IDE_ERROR_ABRT;
            ide_status = IDE_STATUS_DRDY | IDE_STATUS_ERR;
            break;
    }
}

uint8_t DivMMC::ide_read(uint8_t reg) {
    switch (reg) {
        case 0: // Data register (0xA3)
            if (ide_data_index >= 0 && !ide_data_write) {
                uint8_t val = ide_buffer[ide_data_index++];
                if (ide_data_index >= 512) {
                    ide_data_index = -1;
                    if (ide_sector_count > 0) {
                        ide_sector_count--;
                        if (ide_sector_count > 0) {
                            // Advance LBA to next sector (with carry through all bytes)
                            if (ide_head & IDE_LBA_BIT) {
                                // LBA mode: increment full 28-bit address
                                ide_sector++;
                                if (ide_sector == 0) {
                                    ide_cylinder_lo++;
                                    if (ide_cylinder_lo == 0) {
                                        ide_cylinder_hi++;
                                        if (ide_cylinder_hi == 0)
                                            ide_head = (ide_head & 0xF0) | ((ide_head + 1) & 0x0F);
                                    }
                                }
                            } else {
                                // CHS mode: just increment sector
                                ide_sector++;
                            }
                            ide_read_sector();
                        } else {
                            ide_status = IDE_STATUS_DRDY;
                        }
                    } else {
                        ide_status = IDE_STATUS_DRDY;
                    }
                }
                return val;
            }
            return 0xFF;
        case 1: return ide_error;           // Error (0xA7)
        case 2: return ide_sector_count;    // Sector Count (0xAB)
        case 3: return ide_sector;          // Sector Number (0xAF)
        case 4: return ide_cylinder_lo;     // Cylinder Low (0xB3)
        case 5: return ide_cylinder_hi;     // Cylinder High (0xB7)
        case 6: return ide_head;            // Drive/Head (0xBB)
        case 7: return ide_status;          // Status (0xBF)
        default: return 0xFF;
    }
}

void DivMMC::ide_write(uint8_t reg, uint8_t value) {
    switch (reg) {
        case 0: // Data register (0xA3)
            if (ide_data_index >= 0 && ide_data_write) {
                ide_buffer[ide_data_index++] = value;
                if (ide_data_index >= 512) {
                    ide_write_sector_done();
                    ide_data_index = -1;
                    if (ide_sector_count > 0) {
                        ide_sector_count--;
                        if (ide_sector_count > 0) {
                            // Advance LBA to next sector
                            if (ide_head & IDE_LBA_BIT) {
                                ide_sector++;
                                if (ide_sector == 0) {
                                    ide_cylinder_lo++;
                                    if (ide_cylinder_lo == 0) {
                                        ide_cylinder_hi++;
                                        if (ide_cylinder_hi == 0)
                                            ide_head = (ide_head & 0xF0) | ((ide_head + 1) & 0x0F);
                                    }
                                }
                            } else {
                                ide_sector++;
                            }
                            ide_data_index = 0; // ready for next sector
                        } else {
                            ide_status = IDE_STATUS_DRDY;
                        }
                    } else {
                        ide_status = IDE_STATUS_DRDY;
                    }
                }
            }
            break;
        case 1: ide_feature = value; break;       // Features (0xA7)
        case 2: ide_sector_count = value; break;  // Sector Count (0xAB)
        case 3: ide_sector = value; break;        // Sector Number (0xAF)
        case 4: ide_cylinder_lo = value; break;   // Cylinder Low (0xB3)
        case 5: ide_cylinder_hi = value; break;   // Cylinder High (0xB7)
        case 6: ide_head = value; break;          // Drive/Head (0xBB)
        case 7: ide_execute_command(value); break; // Command (0xBF)
    }
}

// ============================================================
// Z-Controller raw SD on ZX-BUS ports 0x77 (config) / 0x57 (data)
// Reuses the DivSD SPI state machine; mutually exclusive with esxDOS.
// ============================================================

void DivMMC::zc_init() {
    if (zc_enabled) return;
    // Real SD card in SDHC sector-addressed mode.
    divsd_mode = true;
    sdhc_mode = true;
    DWORD sector_count = 0;
    disk_ioctl(0, GET_SECTOR_COUNT, &sector_count);
    mmc_file_size[0] = (uint32_t)((uint64_t)sector_count * 512 > 0xFFFFFFFF ? 0xFFFFFFFF : sector_count * 512);
    buildCSD_real(sector_count);
    mmc_ocr[0] = 0xC0;
    mmc_ocr[1] = 0xFF;
    mmc_ocr[2] = 0x80;
    mmc_ocr[3] = 0x00;
    mmc_ocr[4] = 0x00;

    mmc_last_command = 0;
    mmc_index_command = 0;
    mmc_r1 = 1;
    mmc_cs_active = false;
    mmc_read_index = -1;
    mmc_write_index = -1;
    mmc_csd_index = -1;
    mmc_cid_index = -1;
    mmc_ocr_index = -1;
    mmc_sector_buf_addr = 0xFFFFFFFF;
    mmc_sector_dirty = false;
    zc_config = 0;
    zc_enabled = true;
    Debug::log("Z-Controller: raw SD, %lu sectors, SDHC mode", (unsigned long)sector_count);
}

void DivMMC::zc_shutdown() {
    if (!zc_enabled) return;
    flushWriteBuffer();
    zc_enabled = false;
    zc_config = 0;
    mmc_cs_active = false;
    // Leave divsd_mode/sdhc_mode as-is — DivMMC::init() owns these and will
    // reset them on the next mode change.
}

void DivMMC::zc_write_config(uint8_t value) {
    zc_config = value;
    // Port 0x77 bit1 drives the SD CS pin directly; CS is active-low, so
    // bit1=0 means card selected. bit0 is SD power and is ignored here.
    bool new_cs = (value & 0x02) == 0;
    if (new_cs != mmc_cs_active) {
        mmc_cs(new_cs ? 0x00 : 0x01); // mmc_cs treats bit0==0 as active
    }
}

uint8_t DivMMC::zc_read_status() {
    // bit0=0 → SD card present; bit1=0 → not read-only.
    return 0x00;
}

void DivMMC::zc_write_data(uint8_t value) {
    if (!mmc_cs_active) return;
    mmc_write(value);
}

uint8_t DivMMC::zc_read_data() {
    if (!mmc_cs_active) return 0xFF;
    return mmc_read();
}

#endif // !PICO_RP2040
