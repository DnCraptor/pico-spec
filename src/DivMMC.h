#ifndef __DIVMMC_H
#define __DIVMMC_H

#include <inttypes.h>

#if !PICO_RP2040

#include "ff.h"

#define DIVMMC_ROM_SIZE  0x2000   // 8KB ESXDOS ROM
#define DIVMMC_BANK_SIZE 0x2000   // 8KB per bank
#define DIVMMC_NUM_BANKS 16       // 16 banks = 128KB RAM
#define DIVMMC_CACHE_SLOTS 3      // swap mode: number of cached bank buffers

// DivMMC automap trap addresses (M1 cycle entry points)
#define DIVMMC_TRAP_0000 0x0000
#define DIVMMC_TRAP_0008 0x0008   // RST 8 - ESXDOS API
#define DIVMMC_TRAP_0038 0x0038   // IM1 interrupt
#define DIVMMC_TRAP_0066 0x0066   // NMI
#define DIVMMC_TRAP_04C6 0x04C6
#define DIVMMC_TRAP_0562 0x0562
// 0x3D00-0x3DFF also triggers automap (same as TR-DOS)

// Automap off-points
#define DIVMMC_OFF_START 0x1FF8
#define DIVMMC_OFF_END   0x1FFF

class DivMMC {
public:
    static bool enabled;          // Config: DivMMC active
    static bool automap;          // Currently automapped (DivMMC memory visible)
    static bool conmem;           // Port 0xE3 bit 7: manual map control
    static bool mapram;           // Port 0xE3 bit 6: sticky MAPRAM flag
    static uint8_t bank;          // Port 0xE3 bits 5:0: RAM bank select

    static uint8_t *esxdos_rom;   // 8KB ESXDOS ROM (dynamically allocated)
    static uint8_t* bank_ptr[DIVMMC_NUM_BANKS]; // direct pointer per bank (null = swapped out)

    static bool use_psram;            // true = all banks in butter PSRAM (direct pointers)
    static int8_t hi_slot;            // swap mode: slot index of page0_hi bank (-1 if none)
    static int8_t lo_slot;            // swap mode: slot index of page0_lo bank (-1 if none)
    static bool rom_loaded;       // ESXDOS ROM successfully loaded from SD
    static bool divsd_mode;       // true = raw SD access (Config::esxdos == 3)
    static bool divide_mode;      // true = DivIDE (IDE/ATA, Config::esxdos == 2)
    static bool sdhc_mode;        // true = SDHC (sector-addressed), false = standard (byte-addressed)
    static bool zc_enabled;       // true = Z-Controller raw SD on ports 0x77/0x57 (no ROM/banking)
    static uint8_t zc_config;     // Z-Controller port 0x77 latched config (bit0=power, bit1=CS)

    static void init();           // Load ROM, open .mmc/.hdf image
    static void reset();          // Reset state
    static void applyMapping();   // Update page0 pointers based on state
    static inline void markHiDirty() { if (hi_slot >= 0) slot_dirty[hi_slot] = true; }
    static inline void markLoDirty() { if (lo_slot >= 0) slot_dirty[lo_slot] = true; }

    // SD/SPI protocol emulation (DivMMC/DivSD)
    static void mmc_cs(uint8_t value);    // Port 0xE7 write: chip select
    static void mmc_write(uint8_t value); // Port 0xEB write: send command/data byte
    static uint8_t mmc_read();            // Port 0xEB read: receive response/data byte

    // Z-Controller raw SD on ports 0x77 (config/status) / 0x57 (SPI data).
    // Reuses the DivMMC SPI state machine in DivSD/SDHC mode without ROM
    // or memory banking. Mutually exclusive with esxDOS modes.
    static void zc_init();                // Initialize raw SD access (called when Config::zcontroller toggles)
    static void zc_shutdown();            // Tear down raw SD access
    static void zc_write_config(uint8_t value); // Port 0x77 write: power + CS
    static uint8_t zc_read_status();      // Port 0x77 read: card detect / WP
    static void zc_write_data(uint8_t value);   // Port 0x57 write: SPI byte out
    static uint8_t zc_read_data();        // Port 0x57 read: SPI byte in

    // IDE/ATA emulation (DivIDE)
    static void ide_write(uint8_t reg, uint8_t value);  // ATA register write
    static uint8_t ide_read(uint8_t reg);               // ATA register read

    // ZEsarUX-style automap: call pre before fetch, post after fetch
    static inline void preOpcFetch(uint16_t pc);
    static inline void postOpcFetch();

    // Reopen files after SD remount
    static void reopenFiles();

private:
    // Automap deferred signals (ZEsarUX model)
    static bool trap_after;       // Entry point hit → map after fetch
    static bool unmap_after;      // Off-point hit → unmap after fetch

    // SD protocol state machine
    static uint8_t mmc_last_command;
    static int mmc_index_command;
    static uint8_t mmc_params[6];
    static uint8_t mmc_r1;
    static bool mmc_cs_active;

    static int mmc_read_index;
    static int mmc_write_index;
    static int mmc_csd_index;
    static int mmc_cid_index;
    static int mmc_ocr_index;

    static uint32_t mmc_read_address;
    static uint32_t mmc_write_address;

    static uint8_t mmc_sector_buf[512];
    static uint32_t mmc_sector_buf_addr;
    static bool mmc_sector_dirty;

    static FIL mmc_file[2];        // [0]=primary/master, [1]=slave (DivIDE only)
    static bool mmc_file_open[2];
    static uint32_t mmc_file_size[2];

    static uint8_t mmc_csd[16];
    static uint8_t mmc_cid[16];
    static uint8_t mmc_ocr[5];

    static uint8_t readByte(uint32_t address);
    static void writeByte(uint32_t address, uint8_t value);
    static void flushWriteBuffer();
    static void buildCSD();
    static void buildCSD_real(uint32_t sector_count);

    // For DivMMC superfloppy images: synthesize an MBR at sector 0 so the
    // host sees a partition table; sectors >=1 are read from .mmc with offset.
    static void loadSector(uint32_t sector);
    static void storeSector(uint32_t sector);

    // Bank memory management (butter PSRAM or swap)
    static uint8_t* active_buf[DIVMMC_CACHE_SLOTS];
    static int8_t active_bank[DIVMMC_CACHE_SLOTS];
    static bool slot_dirty[DIVMMC_CACHE_SLOTS];
    static uint8_t slot_lru[DIVMMC_CACHE_SLOTS]; // LRU counter (higher = more recently used)
    static uint8_t lru_clock;
    static FIL swap_file;
    static bool swap_open;
    static void materialize(uint8_t bank_idx);
    static void evict(int slot);
    static void touch(int slot);
    static int find_victim(uint8_t exclude_bank);
    static void clearAllBanks();

    // IDE/ATA state (DivIDE)
    static uint8_t ide_feature;
    static uint8_t ide_sector_count;
    static uint8_t ide_sector;
    static uint8_t ide_cylinder_lo;
    static uint8_t ide_cylinder_hi;
    static uint8_t ide_head;
    static uint8_t ide_status;
    static uint8_t ide_error;
    static uint8_t ide_buffer[512];   // sector transfer buffer
    static int ide_data_index;        // byte position in buffer (-1 = no transfer)
    static bool ide_data_write;       // true = PIO_OUT (write), false = PIO_IN (read)
    static uint32_t ide_hdf_data_offset[2]; // byte offset to data area in HDF [master/slave]
    static uint8_t ide_identity[2][106];   // ATA IDENTIFY data from HDF header
    static uint16_t ide_cylinders[2];
    static uint16_t ide_heads[2];
    static uint16_t ide_sectors[2];
    static inline int ide_drive() { return (ide_head >> 4) & 1; }
    static void ide_execute_command(uint8_t cmd);
    static void ide_read_sector();
    static void ide_write_sector_done();
    static uint32_t ide_lba();
};

// ZEsarUX-style automap: pre-fetch check
// Called BEFORE opcode is fetched
inline void DivMMC::preOpcFetch(uint16_t pc) {
    trap_after = false;
    unmap_after = false;

    // Entry points: map AFTER fetch, only if automap is currently OFF
    if (!automap) {
        if (pc == DIVMMC_TRAP_0000 || pc == DIVMMC_TRAP_0008 ||
            pc == DIVMMC_TRAP_0038 || pc == DIVMMC_TRAP_0066 ||
            pc == DIVMMC_TRAP_04C6 || pc == DIVMMC_TRAP_0562) {
            trap_after = true;
        }
    }

    // 0x3D00-0x3DFF: instant map BEFORE fetch
    if (pc >= 0x3D00 && pc <= 0x3DFF) {
        if (!automap) {
            automap = true;
            if (!conmem) applyMapping();
        }
    }

    // Off-points: unmap AFTER fetch, only if automap is currently ON
    if (automap && pc >= DIVMMC_OFF_START && pc <= DIVMMC_OFF_END) {
        unmap_after = true;
    }
}

// Called AFTER opcode is fetched (and PC incremented)
inline void DivMMC::postOpcFetch() {
    if (trap_after) {
        automap = true;
        if (!conmem) applyMapping();
    }
    if (unmap_after) {
        automap = false;
        if (!conmem) applyMapping();
    }
}

#endif // !PICO_RP2040

#endif // __DIVMMC_H
