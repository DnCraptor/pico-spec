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

// Runtime-generated AluBytes: pure palette indices, no sync bits
// Single set (no longer need two copies for different sync polarities)
static unsigned int AluBytesStd[16][256];

// Generate standard Spectrum AluBytes table
// Palette indices: 0-7 = normal colors, 8-15 = bright colors
static inline void initAluBytes() {
    for (int nibble = 0; nibble < 16; nibble++) {
        for (int att = 0; att < 256; att++) {
            // Extract ink/paper/bright from attribute byte
            // bit 7 = flash (handled by bitmap inversion at runtime)
            // bit 6 = bright
            // bits 5:3 = paper color
            // bits 2:0 = ink color
            uint8_t ink_code = att & 0x07;
            uint8_t paper_code = (att >> 3) & 0x07;
            uint8_t bright = (att >> 6) & 1;

            uint8_t ink_idx = ink_code + bright * 8;
            uint8_t paper_idx = paper_code + bright * 8;

            uint8_t px[2] = { paper_idx, ink_idx };

            // Pack 4 pixels into uint32_t
            // Nibble bit mapping (MSB first = leftmost pixel):
            //   bit 3 → byte 2 (x=0), bit 2 → byte 3 (x=1),
            //   bit 1 → byte 0 (x=2), bit 0 → byte 1 (x=3)
            // This accounts for x^2 byte-swap in frame buffer
            AluBytesStd[nibble][att] =
                px[(nibble >> 1) & 1]        |
                (px[nibble & 1]       << 8)  |
                (px[(nibble >> 3) & 1] << 16)|
                (px[(nibble >> 2) & 1] << 24);
        }
    }

    // Point AluByte[] to the standard table
    for (int n = 0; n < 16; n++)
        AluByte[n] = AluBytesStd[n];
}

#endif // VIDPRECALC_h
