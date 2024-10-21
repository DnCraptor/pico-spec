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

class mem_desc_t {
    static std::list<mem_desc_t> pages; // a pool of assigned pages
    struct mem_desc_int_t {
        uint8_t* p;
        uint32_t vram_off;
        bool in_vram;
        mem_desc_int_t() : p(0), vram_off(0), in_vram(false) {}
    };
    mem_desc_int_t* _int;
    uint8_t* to_vram(void);
    void from_vram(uint8_t* p);
public:
    static void reset(void);
    mem_desc_t() : _int( new mem_desc_int_t() ) {}
    mem_desc_t(const mem_desc_t& s) : _int( s._int ) {}
    void operator=(const mem_desc_t& s) {
        _int = s._int;
    }
    void _sync(void);
    inline uint8_t* direct(void) {
        return _int->p;
    }
    inline uint8_t* sync(void) {
        if (_int->in_vram) {
            _sync();
        }
        return _int->p;
    }
    inline void assign_vram(uint32_t page) { // virtual RAM - PSRAM or swap
        this->_int->p = 0;
        this->_int->vram_off = page * 0x4000;
        this->_int->in_vram = true;
    }
    inline void assign_ram(uint8_t* p, uint32_t page, bool locked) {
        this->_int->p = p;
        this->_int->vram_off = page * 0x4000;
        this->_int->in_vram = false;
        if (!locked) {
            pages.push_back(*this);
        }
    }
    inline void assign_rom(const uint8_t* p) { // TODO: prev?
        this->_int->p = (uint8_t*)p;
        this->_int->vram_off = 0;
        this->_int->in_vram = false;
    }
};

#define MEM_PG_SZ 0x4000
class MemESP
{
public:
    static mem_desc_t rom[5];
    static mem_desc_t ram[32];

    static mem_desc_t ramCurrent[4];    
    static bool ramContended[4];

    static uint8_t notMore128;
    static uint8_t page0ram;
    static uint8_t hiddenROM;
    static uint8_t page128;
    static uint8_t shiftScorp;
    static uint8_t bankLatch;
    static uint8_t videoLatch;
    static uint8_t romLatch;
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
        return ram[2].sync()[addr - 0x8000];
    case 3:
        return ram[bankLatch].sync()[addr - 0xC000];
    default:
        return (page0ram ? ram[0].sync() : rom[romInUse].direct())[addr];
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
        if (page0ram) ram[0].sync()[addr] = data;
        return;
    case 1:
        ram[5].direct()[addr - 0x4000] = data;
        break;
    case 2:
        ram[2].sync()[addr - 0x8000] = data;
        break;
    case 3:
        ram[bankLatch].sync()[addr - 0xC000] = data;
        break;
    }
    return;
}

inline void MemESP::writeword(uint16_t addr, uint16_t data) {
    writebyte(addr, (uint8_t)data);
    writebyte(addr + 1, (uint8_t)(data >> 8));
}


#endif