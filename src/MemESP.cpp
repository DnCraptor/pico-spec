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

#include "MemESP.h"
#include <stddef.h>
#include "psram_spi.h"
#include "ff.h"

std::list<mem_desc_t> mem_desc_t::pages;

static FIL f;
static const char PAGEFILE[] = "/tmp/pico-spec.512k";
void mem_desc_t::reset(void) {
    pages.clear();
    if ( !psram_size() ) {
        f_close(&f);
        f_unlink(PAGEFILE); // ensure it is new file
        f_open(&f, PAGEFILE, FA_WRITE | FA_CREATE_ALWAYS);
        f_close(&f);
        f_open(&f, PAGEFILE, FA_READ | FA_WRITE);
    }
}

uint8_t* mem_desc_t::to_vram(void) {
    uint8_t* res = _int->p;
    uint32_t ba = _int->vram_off;
    if (psram_size()) {
        for (size_t i = 0; i < 0x4000; i += 4) {
            write32psram(ba + i, *(uint32_t*)(res + i));
        }
    } else {
        UINT bw;
        FSIZE_t lba = ba;
        f_lseek(&f, lba);
        f_write(&f, res, 0x4000, &bw);
    }
    _int->outside = true;
    _int->p = 0;
    return res;
}
void mem_desc_t::from_vram(uint8_t* p) {
    this->_int->p = p;
    uint32_t ba = _int->vram_off;
    if (psram_size()) {
        for (size_t i = 0; i < 0x4000; i += 4) {
            *(uint32_t*)(p + i) = read32psram(ba + i);
        }
    } else {
        UINT br;
        FSIZE_t lba = ba;
        f_lseek(&f, lba);
        f_read(&f, p, 0x4000, &br);
    }
    _int->outside = false;
}
uint8_t mem_desc_t::_read(uint16_t addr) {
    uint32_t ba = _int->vram_off;
    if (psram_size()) {
        return read8psram(ba + addr);
    }
    UINT br;
    FSIZE_t lba = ba;
    f_lseek(&f, lba + addr);
    uint8_t r;
    f_read(&f, &r, 1, &br);
    return r;
}
void mem_desc_t::_write(uint16_t addr, uint8_t v) {
    uint32_t ba = _int->vram_off;
    if (psram_size()) {
        write8psram(ba + addr, v);
    }
    UINT br;
    FSIZE_t lba = ba;
    f_lseek(&f, lba + addr);
    f_write(&f, &v, 1, &br);
}
void mem_desc_t::_sync(void) {
    for (auto it = pages.begin(); it != pages.end(); ++it) {
        mem_desc_t& page = *it;
        if (!page._int->outside) {
            from_vram( page.to_vram() );
            pages.erase(it);
            pages.push_back(*this);
            break;
        }
    }
}
/// TODO: packet mode
void mem_desc_t::from_file(FIL* f_in, size_t sz) {
    UINT br;
    if (!_int->outside) {
        f_read(f_in, direct(), sz, &br);
        return;
    }
    uint8_t v;
    uint32_t ba = _int->vram_off;
    if (psram_size()) {
        for (size_t addr = 0; addr < sz; ++addr) {
            f_read(f_in, &v, 1, &br);
            write8psram(ba + addr, v);
        }
        return;
    }
    FSIZE_t lba = ba;
    f_lseek(&f, lba);
    for (size_t addr = 0; addr < sz; ++addr) {
        f_read(f_in, &v, 1, &br);
        f_write(&f, &v, 1, &br);
    }
}
void mem_desc_t::to_file(FIL* f_out, size_t sz) {
    UINT br;
    if (!_int->outside) {
        f_write(f_out, direct(), sz, &br);
        return;
    }
    uint8_t v;
    uint32_t ba = _int->vram_off;
    if (psram_size()) {
        for (size_t addr = 0; addr < sz; ++addr) {
            v = read8psram(ba + addr);
            f_write(f_out, &v, 1, &br);
        }
        return;
    }
    FSIZE_t lba = ba;
    f_lseek(&f, lba);
    for (size_t addr = 0; addr < sz; ++addr) {
        f_read(&f, &v, 1, &br);
        f_write(f_out, &v, 1, &br);
    }
}
void mem_desc_t::from_mem(mem_desc_t& ram, size_t sz) {
    if (!_int->outside) {
        if (!ram._int->outside) {
            memcpy(direct(), ram.direct(), sz);
        } else {
            uint8_t* p = direct();
            for (size_t addr = 0; addr < sz; ++addr) {
                p[addr] = ram._read(addr);
            }
        }
    } else {
        if (!ram._int->outside) {
            uint8_t* p = ram.direct();
            for (size_t addr = 0; addr < sz; ++addr) {
                _write(addr, p[addr]);
            }
        } else {
            for (size_t addr = 0; addr < sz; ++addr) {
                _write(addr, ram._read(addr));
            }
        }
    }

}
void mem_desc_t::cleanup() {
    if (_int->outside) {
        for (size_t addr = 0; addr < 0x4000; ++addr) {
            _write(addr, 0);
        }
    } else {
        memset(direct(), 0, 0x4000);
    }
}

mem_desc_t MemESP::rom[64];
mem_desc_t MemESP::ram[64 + 2];
bool MemESP::newSRAM = false;

uint8_t* MemESP::ramCurrent[4];
bool MemESP::ramContended[4];

uint8_t MemESP::notMore128 = 0;
uint8_t MemESP::page0ram = 0;
uint8_t MemESP::bankLatch = 0;
uint8_t MemESP::videoLatch = 0;
uint8_t MemESP::romLatch = 0;
uint8_t MemESP::sramLatch = 0;
uint8_t MemESP::pagingLock = 0;
uint8_t MemESP::romInUse = 0;

