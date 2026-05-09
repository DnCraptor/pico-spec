/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Victor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramon Martinez and Jorge Fuertes
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

#include "stdio.h"
#define CAPTUREBMP_IMPL
#include "CaptureBMP.h"
#include "Video.h"
#include "FileUtils.h"
#include "messages.h"
#include "Config.h"
#include "OSDMain.h"
#include "ff.h"

size_t fwrite(const void* v, size_t sz1, size_t sz2, FIL* f);

void CaptureToBmp()
{
    char filename[] = "ESP00000.bmp";
    unsigned char bmp_header2[BMP_HEADER2_SIZE] = {
        0xaa,0xaa,0xaa,0xaa,0xbb,
        0xbb,0xbb,0xbb,0x01,0x00,
        0x08,0x00,0x00,0x00,0x00,
        0x00,0xcc,0xcc,0xcc,0xcc
    };

    // framebuffer size
    int w = VIDEO::vga.xres;
    int h = OSD::scrH;

    // number of uint32_t words
    int count = w >> 2;

    // allocate line buffer
    uint32_t *linebuf = new uint32_t[count];
    if (NULL == linebuf) {
        printf("Capture BMP: unable to allocate line buffer\n");
        return;
    }

    static const char scrdir[] = CONFIG_DIR DISK_SCR_DIR;

    // Create dir if it doesn't exist
    FILINFO stat_buf;
    if (f_stat(scrdir, &stat_buf) != FR_OK) {
        if (f_mkdir(scrdir) != FR_OK) {
            printf("Capture BMP: problem creating capture dir\n");
            delete[] linebuf;
            return;
        }
    }

    DIR dir;
    if (f_opendir(&dir, scrdir) != FR_OK) {
        printf("Capture BMP: problem accessing capture dir\n");
        delete[] linebuf;
        return;
    }
    int bmpnumber = 0;
    while (f_readdir(&dir, &stat_buf) == FR_OK && stat_buf.fname[0] != '\0') {
        if (stat_buf.fname[0] == 'E' && stat_buf.fname[1] == 'S' && stat_buf.fname[2] == 'P'
            && stat_buf.fname[8] == '.' && stat_buf.fname[9] == 'b') {
            int fnum = atoi(&stat_buf.fname[3]);
            if (fnum > bmpnumber) bmpnumber = fnum;
        }
    }
    f_closedir(&dir);

    bmpnumber++;

    if (Config::slog_on) printf("BMP number -> %.5d\n",bmpnumber);

    sprintf((char *)filename,"ESP%.5d.bmp",bmpnumber);

    // Full filename. Save only to SD.
    char fullfn[32];
    snprintf(fullfn, sizeof(fullfn), "%s/%s", scrdir, filename);

    // open file for writing
    FIL* f = fopen2(fullfn, FA_CREATE_ALWAYS | FA_WRITE);
    if (!f) {
        delete[] linebuf;
        printf("Capture BMP: unable to open file %s for writing\n", fullfn);
        return;
    }

    // put width, height and size values in header
    uint32_t* biWidth     = (uint32_t*)(&bmp_header2[0]);
    uint32_t* biHeight    = (uint32_t*)(&bmp_header2[4]);
    uint32_t* biSizeImage = (uint32_t*)(&bmp_header2[16]);
    *biWidth = w;
    *biHeight = h;
    *biSizeImage = w * h;

    // write header 1
    fwrite(bmp_header1, BMP_HEADER1_SIZE, 1, f);

    // write header 2
    fwrite(bmp_header2, BMP_HEADER2_SIZE, 1, f);

    // write header 3 info fields (xPelsPerMeter, yPelsPerMeter, clrUsed, clrImportant)
    fwrite(bmp_header3_info, BMP_HEADER3_INFO_SIZE, 1, f);

    // Generate and write palette dynamically from current VGA palette (G3R3B2 format)
    uint8_t bmp_palette[BMP_PALETTE_SIZE];
    VIDEO::getBmpPalette(bmp_palette);
    fwrite(bmp_palette, BMP_PALETTE_SIZE, 1, f);

    // process every scanline in reverse order (BMP is bottom-up)
    for (int y = h - 1; y >= 0; y--) {
        uint32_t* src = (uint32_t*)VIDEO::vga.frameBuffer[y];
        uint32_t* dst = linebuf;
        // process every uint32 in scanline
        for (int i = 0; i < count; i++) {
            uint32_t srcval = *src++;
            uint32_t dstval = 0;
            // swap uint16 halves to undo x^2 XOR framebuffer indexing
            dstval |= ((srcval & 0xFFFF0000) >> 16);
            dstval |= ((srcval & 0x0000FFFF) << 16);
            *dst++ = dstval;
        }
        // write line to file
        fwrite(linebuf, sizeof(uint32_t), count, f);
    }

    // cleanup
    fclose2(f);

    delete[] linebuf;
}
