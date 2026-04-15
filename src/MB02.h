#ifndef __MB02_H
#define __MB02_H

#if !PICO_RP2040

#include <inttypes.h>
#include <string>

#define MB02_NUM_PAGES    32       // 512KB / 16KB per page
#define MB02_PAGE_SIZE    0x4000   // 16KB per SRAM page
#define MB02_EPROM_SIZE   0x0800   // 2KB boot EPROM

// Port #17 (Memory Paging Register) bits
#define MB02_PAGE_MASK    0x1F     // bits 0-4: page number (0-31)
#define MB02_WRITE_ENABLE 0x20     // bit 5: SRAM write enable
#define MB02_SRAM_ENABLE  0x40     // bit 6: page SRAM into 0x0000-0x3FFF
#define MB02_EPROM_ENABLE 0x80     // bit 7: page boot EPROM into 0x0000-0x07FF

class MB02 {
public:
    static bool enabled;           // Config: MB-02 active
    static uint8_t paging_reg;     // Last value written to port #17
    static uint8_t floppy_reg;     // Last value written to port #13
    static uint8_t* sram_base;     // Base pointer in butter PSRAM (512KB)
    static bool write_enabled;     // Derived from bit 5 of paging_reg

    static void init();            // Allocate SRAM, set up EPROM
    static void reset();           // Reset to boot state (EPROM mapped)
    static void applyMapping();    // Update page0_lo/page0_hi from paging_reg

    // Port handlers
    static void writePort17(uint8_t data);   // Memory paging register
    static void writePort13(uint8_t data);   // Floppy control (drive/motor select)
    static uint8_t readPort13();             // Floppy status (DRQ, INTRQ, motors)

private:
    // Composite 8K buffer for EPROM mode (2K EPROM + 6K ROM)
    static uint8_t page0_composite[0x2000];
    static uint8_t motor_state;    // Motor on/off state per drive
};

#endif // !PICO_RP2040
#endif // __MB02_H
