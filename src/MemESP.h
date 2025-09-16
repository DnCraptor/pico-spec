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

#ifdef BUTTER_PSRAM_GPIO
#if !defined(PICO_RP2040) && !defined(PICO_RP2350)
extern volatile uint8_t* PSRAM_DATA;
uint32_t butter_psram_size();
#else
static uint8_t* PSRAM_DATA = (uint8_t*)0;
inline static uint32_t butter_psram_size() { return 0; }
#endif
#else
static uint8_t* PSRAM_DATA = (uint8_t*)0;
inline static uint32_t butter_psram_size() { return 0; }
#endif

class mem_desc_t {
    static std::list<mem_desc_t> pages; // a pool of assigned pages
    struct mem_desc_int_t {
        uint8_t* p;
        uint32_t vram_off;
        bool outside;
        bool is_rom;
        mem_desc_int_t() : p(0), vram_off(0), outside(false), is_rom(false) {}
    };
    mem_desc_int_t* _int;
    uint8_t* to_vram(void);
    void from_vram(uint8_t* p);
    void _sync(void);
    uint8_t _read(uint16_t addr);
    void _write(uint16_t addr, uint8_t v);
public:
    static void reset(void);
    mem_desc_t() : _int( new mem_desc_int_t() ) {}
    mem_desc_t(const mem_desc_t& s) : _int( s._int ) {}
    void operator=(const mem_desc_t& s) {
        _int = s._int;
    }
    inline uint8_t* direct(void) {
        return _int->p;
    }
    inline uint8_t* sync(void) {
        if (_int->outside) {
            _sync();
        }
        return _int->p;
    }
    inline uint8_t read(uint16_t addr) {
        if (_int->outside) {
            return _read(addr);
        }
        return _int->p[addr];
    }
    inline void write(uint16_t addr, uint8_t v) {
        if (_int->outside) {
            return _write(addr, v);
        }
        _int->p[addr] = v;
    }
    inline void assign_vram(uint32_t page) { // virtual RAM - PSRAM or swap
        this->_int->p = 0;
        this->_int->vram_off = page * 0x4000;
        this->_int->outside = true;
    }
    static inline uint8_t* revoke_1_ram_page() {
        auto it = pages.begin();
        if (it == pages.end()) return 0;
        it->sync(); // TODO: optimize it
        uint8_t* p = it->_int->p;
        if (!it->_int->outside) {
            it->to_vram();
        }
        pages.erase(it);
        return p;
    }
    inline void assign_ram(uint8_t* p, uint32_t page, bool locked) {
        this->_int->p = p;
        this->_int->vram_off = page * 0x4000;
        this->_int->outside = false;
        if (!locked) {
            pages.push_back(*this);
        }
    }
    inline void assign_rom(const uint8_t* p) { // TODO: prev?
        this->_int->p = (uint8_t*)p;
        this->_int->vram_off = 0;
        this->_int->outside = false;
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

#define MEM_PG_SZ 0x4000
class MemESP
{
public:
    static mem_desc_t rom[64];
    static mem_desc_t ram[64 + 2];

    static bool newSRAM;

    static uint8_t* ramCurrent[4];
    static bool ramContended[4];

    static uint8_t notMore128;
    static uint8_t page0ram;
    static uint8_t bankLatch;
    static uint8_t videoLatch;
    static uint8_t romLatch;
    static uint8_t sramLatch;
    static uint8_t pagingLock;

    static uint8_t romInUse;

    static uint8_t readbyte(uint16_t addr);
    static uint16_t readword(uint16_t addr);
    static void writebyte(uint16_t addr, uint8_t data);
    static void writeword(uint16_t addr, uint16_t data);
};

// Inline memory access functions

inline uint8_t MemESP::readbyte(uint16_t addr) {
    uint8_t page = addr >> 14;
    switch (page) {
    case 1:
        return ram[5].direct()[addr - 0x4000];
    case 2:
        return ram[2].direct()[addr - 0x8000];
    case 3:
        return ram[bankLatch].read(addr - 0xC000);
    default:
        return newSRAM ? ram[64 + MemESP::sramLatch].read(addr) : (page0ram ? ram[0].read(addr) : rom[romInUse].direct()[addr]);
    }
}

inline uint16_t MemESP::readword(uint16_t addr) {
    return ((readbyte(addr + 1) << 8) | readbyte(addr));
}

inline void MemESP::writebyte(uint16_t addr, uint8_t data)
{
    uint8_t page = addr >> 14;
    switch (page) {
    case 0:
        if (newSRAM) ram[64 + MemESP::sramLatch].write(addr, data);
        else if (page0ram && !ram[0].is_rom()) ram[0].write(addr, data);
        break;
    case 1:
        ram[5].direct()[addr - 0x4000] = data;
        break;
    case 2:
        ram[2].direct()[addr - 0x8000] = data;
        break;
    case 3:
        ram[bankLatch].write(addr - 0xC000, data);
        break;
    }
    return;
}

inline void MemESP::writeword(uint16_t addr, uint16_t data) {
    writebyte(addr, (uint8_t)data);
    writebyte(addr + 1, (uint8_t)(data >> 8));
}


#endif