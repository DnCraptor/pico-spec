#include "DivMMC.h"

#if !PICO_RP2040

#include <cstring>
#include "MemESP.h"
#include "Config.h"
#include "Debug.h"
#include "ff.h"

// Static member definitions
bool DivMMC::enabled = false;
bool DivMMC::automap = false;
bool DivMMC::automap_pending = false;
bool DivMMC::conmem = false;
bool DivMMC::mapram = false;
uint8_t DivMMC::bank = 0;
uint8_t DivMMC::esxdos_rom[DIVMMC_ROM_SIZE];
uint8_t DivMMC::ram[DIVMMC_NUM_BANKS][DIVMMC_BANK_SIZE];
uint8_t DivMMC::spi_cs = 1; // CS deselected by default

// External PIO SPI instance from sdcard.c
extern "C" {
    #include "pio_spi.h"
    extern pio_spi_inst_t pio_spi;
}

void DivMMC::init() {
    memset(ram, 0, sizeof(ram));
    memset(esxdos_rom, 0xFF, sizeof(esxdos_rom));

    // Load ESXDOS ROM from SD card
    FIL f;
    FRESULT fr = f_open(&f, "SD/r/esxdos.rom", FA_READ);
    if (fr == FR_OK) {
        UINT br;
        f_read(&f, esxdos_rom, DIVMMC_ROM_SIZE, &br);
        f_close(&f);
        Debug::log("DivMMC: ESXDOS ROM loaded (%d bytes)", br);
        enabled = Config::divmmc;
    } else {
        Debug::log("DivMMC: esxdos.rom not found, DivMMC disabled");
        enabled = false;
    }

    reset();
}

void DivMMC::reset() {
    automap = false;
    automap_pending = false;
    conmem = false;
    mapram = false;
    bank = 0;
    spi_cs = 1;
    applyMapping();
}

void DivMMC::applyMapping() {
    if (conmem || automap) {
        // DivMMC mapped in
        // 0x0000-0x1FFF: ESXDOS ROM (or RAM bank 3 if MAPRAM)
        if (mapram) {
            MemESP::page0_lo = ram[3];
        } else {
            MemESP::page0_lo = esxdos_rom;
        }
        // 0x2000-0x3FFF: selected RAM bank
        MemESP::page0_hi = ram[bank & (DIVMMC_NUM_BANKS - 1)];
        MemESP::divmmc_mapped = true;
    } else {
        // DivMMC unmapped - restore normal ROM
        MemESP::divmmc_mapped = false;
        MemESP::recoverPage0();
    }
}

uint8_t DivMMC::spiTransfer(uint8_t data) {
    uint8_t result = 0xFF;
    pio_spi_write8_read8_blocking(&pio_spi, &data, &result, 1);
    return result;
}

#endif // !PICO_RP2040
