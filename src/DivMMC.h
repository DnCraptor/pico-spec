#ifndef __DIVMMC_H
#define __DIVMMC_H

#include <inttypes.h>

#if !PICO_RP2040

#define DIVMMC_ROM_SIZE  0x2000   // 8KB ESXDOS ROM
#define DIVMMC_BANK_SIZE 0x2000   // 8KB per bank
#define DIVMMC_NUM_BANKS 16       // 16 banks = 128KB RAM

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
    static bool automap_pending;  // Delayed: set on M1 trap, applied on next M1
    static bool conmem;           // Port 0xE3 bit 7: manual map control
    static bool mapram;           // Port 0xE3 bit 6: sticky MAPRAM flag
    static uint8_t bank;          // Port 0xE3 bits 5:0: RAM bank select

    static uint8_t esxdos_rom[DIVMMC_ROM_SIZE];  // 8KB ESXDOS ROM
    static uint8_t ram[DIVMMC_NUM_BANKS][DIVMMC_BANK_SIZE]; // 128KB DivMMC RAM

    static uint8_t spi_cs;        // Port 0xE7 CS state

    static void init();           // Load ROM from SD card
    static void reset();          // Reset state
    static void applyMapping();   // Update page0 pointers based on state

    static inline void checkM1(uint16_t pc);  // M1 cycle trap check
    static uint8_t spiTransfer(uint8_t data); // Port 0xEB SPI byte exchange
};

// Inline M1 trap check - called from fetchOpcode and exec_nocheck
inline void DivMMC::checkM1(uint16_t pc) {
    // Apply pending automap from previous M1 trap
    if (automap_pending) {
        automap_pending = false;
        automap = true;
        applyMapping();
    }

    if (!automap && !conmem) {
        // Check entry trap addresses
        if (pc == DIVMMC_TRAP_0000 || pc == DIVMMC_TRAP_0008 ||
            pc == DIVMMC_TRAP_0038 || pc == DIVMMC_TRAP_0066 ||
            pc == DIVMMC_TRAP_04C6 || pc == DIVMMC_TRAP_0562 ||
            (pc >= 0x3D00 && pc <= 0x3DFF)) {
            automap_pending = true;
        }
    } else if (automap && !conmem) {
        // Check exit conditions: PC >= 0x4000 or in off-points 0x1FF8-0x1FFF
        if (pc >= 0x4000 || (pc >= DIVMMC_OFF_START && pc <= DIVMMC_OFF_END)) {
            automap = false;
            automap_pending = false;
            applyMapping();
        }
    }
}

#endif // !PICO_RP2040

#endif // __DIVMMC_H
