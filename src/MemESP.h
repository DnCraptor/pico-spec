/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or
visit https://zxespectrum.speccy.org/contacto

*/

#ifndef MemESP_h
#define MemESP_h

#include <inttypes.h>
#include <list>
#include "ff.h"
#include "roms.h"
#include "Debug.h"

#define MEM_PG_SZ 0x4000
#if PICO_RP2350
// with gigascreen
#define MEM_REMAIN (11*16*1024)
#else
#define MEM_REMAIN (6*16**1024)
#endif

extern uint32_t MEM_PG_CNT;
extern uint8_t* PSRAM_DATA;
extern uint8_t psram_pin;
extern bool rp2350a;
uint32_t butter_psram_size();
extern uint8_t rx[4];

enum mem_type_t {
    POINTER = 0,
    PSRAM_SPI,
    SWAP
};

class mem_desc_t {
    static std::list<mem_desc_t> pages; // a pool of assigned pages
    static uint8_t* plugged_in[4]; // pointers are plugged to 64k space (do not revoke 'em)
    struct mem_desc_int_t {
        uint8_t* p;
        uint32_t vram_off;
        mem_type_t mem_type;
        bool is_rom;
        mem_desc_int_t() : p(0), vram_off(0), mem_type(POINTER), is_rom(false) {}
    };
    mem_desc_int_t* _int;
    uint8_t* to_vram(void);
    void from_vram(uint8_t* p);
    void _sync(uint8_t bank);
    uint8_t _read(uint16_t addr);
    void _write(uint16_t addr, uint8_t v);
public:
    static void reset(void);
    mem_desc_t() : _int( new mem_desc_int_t() ) {}
    mem_desc_t(const mem_desc_t& s) : _int( s._int ) {}
    mem_desc_t(uint8_t* p, uint32_t page) : _int( new mem_desc_int_t() ) {
        this->_int->p = p;
        this->_int->vram_off = page * MEM_PG_SZ;
    }
    void operator=(const mem_desc_t& s) {
        _int = s._int;
    }
    inline uint8_t* direct(void) {
        return _int->p;
    }
    inline uint8_t* sync(uint8_t bank) {
        if (_int->mem_type != POINTER) {
            _sync(bank);
        }
        uint8_t* res = _int->p;
        if (bank < 4) plugged_in[bank] = res;
        return res;
    }
    inline uint8_t read(uint16_t addr) {
        if (_int->mem_type != POINTER) {
            return _read(addr);
        }
        return _int->p[addr];
    }
    inline void write(uint16_t addr, uint8_t v) {
        if (_int->mem_type != POINTER) {
            return _write(addr, v);
        }
        _int->p[addr] = v;
    }
     // virtual RAM - PSRAM or swap
    inline void assign_vram(uint32_t page, mem_type_t mem_type) {
        this->_int->p = 0;
        this->_int->vram_off = page * MEM_PG_SZ;
        this->_int->mem_type = mem_type;
    }
    static inline uint8_t* revoke_1_ram_page() {
        auto it = pages.begin();
        if (it == pages.end()) return 0;
        it->sync(5); // TODO: optimize it
        uint8_t* p = it->_int->p;
        if (!it->_int->mem_type != POINTER) {
            it->to_vram();
        }
        pages.erase(it);
        return p;
    }
    inline void assign_ram(uint8_t* p, uint32_t page, bool locked) {
        this->_int->p = p;
        this->_int->vram_off = page * MEM_PG_SZ;
        this->_int->mem_type = POINTER;
        if (!locked) {
            pages.push_back(*this);
        }
    }
    inline void assign_rom(const uint8_t* p) { // TODO: prev?
        this->_int->p = (uint8_t*)p;
        this->_int->vram_off = 0;
        this->_int->mem_type = POINTER;
        this->_int->is_rom = true;
    }
    inline bool is_rom(void) {
        return this->_int->is_rom;
    }
    void from_file(FIL* f, size_t sz);
    void to_file(FIL* f, size_t sz);
    void from_mem(mem_desc_t& ram, size_t sz);
    void cleanup();
};

class MemESP
{
public:
    static mem_desc_t rom[64];
    static mem_desc_t* ram;

    static bool newSRAM;

    static uint8_t* ramCurrent[4];
    static bool ramContended[4];

    static uint8_t notMore128;
    static uint32_t page0ram;
    static uint32_t bankLatch;
    static uint8_t videoLatch;
    static uint8_t romLatch;
    static uint8_t pagingLock;

    static uint8_t romInUse;

    static uint8_t byteMemMode;

    static uint8_t readbyte(uint16_t addr);
    static uint16_t readword(uint16_t addr);
    static void writebyte(uint16_t addr, uint8_t data);
    static void writeword(uint16_t addr, uint16_t data);

    static int getByteContention(uint16_t addr);
    //static void UpdateByteMem0000(int ramIndex, int SovmestMode);

    inline static void recoverPage0() {
        MemESP::ramCurrent[0] = MemESP::newSRAM ? MemESP::ram[MEM_PG_CNT + MemESP::romLatch].sync(0) :
                               (MemESP::page0ram ? MemESP::ram[0].sync(0) : MemESP::rom[MemESP::romInUse].direct());
    }
};

// Inline memory access functions

// ==== Функция получения задержки для адреса ====
inline int MemESP::getByteContention(uint16_t addr) {
    if (addr < 0xC000) return 0;

    int res = 0;
    uint16_t offset = addr - 0xC000;
    uint8_t val = (offset < 512) ? romDd10[offset] : romDd11[offset - 512];
    if (val == 0xEE) res = 4;
    else if (val == 0xFE) res = 3;
    else if (val == 0xBE) res = 2;
    else res = 1;

    return res;
}

inline uint8_t MemESP::readbyte(uint16_t addr) {
    uint8_t page = addr >> 14;
    uint8_t* p = ramCurrent[page];
    return p[addr & 0x3fff];
}

inline uint16_t MemESP::readword(uint16_t addr) {
    return ((readbyte(addr + 1) << 8) | readbyte(addr));
}

inline void MemESP::writebyte(uint16_t addr, uint8_t data)
{
    uint8_t page = addr >> 14;
    uint8_t* p = ramCurrent[page];
    if (p < (uint8_t*)0x11000000) return;
    p[addr & 0x3fff] = data;
}

inline void MemESP::writeword(uint16_t addr, uint16_t data) {
    writebyte(addr, (uint8_t)data);
    writebyte(addr + 1, (uint8_t)(data >> 8));
}


#endif