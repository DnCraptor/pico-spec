/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

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

*/

#ifndef VIDPRECALC_h
#define VIDPRECALC_h

// AluByte[nibble] → pointer to 256-entry table
// Each entry is uint32_t with 4 packed palette indices
static unsigned int* AluByte[16];

// Precalculated standard Spectrum AluBytes table (in flash)
extern "C" const unsigned int AluBytesStd_flash[16][256];

// Point AluByte[] to the flash table
static inline void initAluBytes() {
    for (int n = 0; n < 16; n++)
        AluByte[n] = (unsigned int*)AluBytesStd_flash[n];
}

#endif // VIDPRECALC_h
