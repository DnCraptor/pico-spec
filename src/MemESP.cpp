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
    _int->in_vram = true;
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
    _int->in_vram = false;
}
void mem_desc_t::_sync(void) {
    for (auto it = pages.begin(); it != pages.end(); ++it) {
        mem_desc_t& page = *it;
        if (!page._int->in_vram) {
            from_vram( page.to_vram() );
            pages.erase(it);
            pages.push_back(*this);
            break;
        }
    }
}

mem_desc_t MemESP::rom[5];
mem_desc_t MemESP::ram[64];
mem_desc_t MemESP::ramCurrent[4];
bool MemESP::ramContended[4];

uint8_t MemESP::notMore128 = 0;
uint8_t MemESP::page0ram = 0;
uint8_t MemESP::page128 = 0;
uint8_t MemESP::bankLatch = 0;
uint8_t MemESP::videoLatch = 0;
uint8_t MemESP::romLatch = 0;
uint8_t MemESP::pagingLock = 0;
uint8_t MemESP::romInUse = 0;

