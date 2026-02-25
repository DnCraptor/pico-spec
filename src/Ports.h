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

#ifndef Ports_h
#define Ports_h

#include <inttypes.h>
#include "ESPectrum.h"

class Ports {

public:

    static uint8_t input(uint16_t address);
    static void output(uint16_t address, uint8_t data);
    static uint8_t port[128];

    static uint8_t (*getFloatBusData)();
    static uint8_t getFloatBusData48();
    static uint8_t getFloatBusData128();

    static void FDDStep(bool force);

    static uint8_t portAFF7;

#if !PICO_RP2040
    // KR580VI53 (Intel 8253 PIT) — Byte computer sound synthesizer
    struct PIT8253Channel {
        uint16_t count_value;  // Programmed divisor (16-bit)
        int counter;           // Current counter position
        uint8_t output;        // Current output state (0 or 1)
        uint8_t lsb;           // Latched LSB for 2-byte load
        bool lsb_loaded;       // Whether LSB is waiting for MSB
        bool active;           // Whether channel has been programmed
    };
    static PIT8253Channel pitChannels[3];
    static void pitGenSound(uint8_t* buf, int bufsize);
#endif

private :

    static void ioContentionLate(bool contend);
    static uint8_t port254;
    static uint8_t speaker_values[8];

};

#endif // Ports_h
