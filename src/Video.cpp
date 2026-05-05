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

#include "Video.h"
#include "Debug.h"
#include "Tape.h"
#include "FileUtils.h"
#include "VidPrecalc.h"
#include "CPU.h"
#include "MemESP.h"
#include "Config.h"
#include "OSDMain.h"
#include "hardconfig.h"
#include "hardpins.h"
#include "Z80_JLS/z80.h"
#include "Z80_JLS/z80operations.h"
#include "psram_spi.h"
#if !PICO_RP2040
#include "Z80DMA.h"
#endif
extern "C" void graphics_set_palette(uint8_t i, uint32_t color888);
extern "C" void vga_set_palette_entry_solid(uint8_t i, uint32_t color888);
extern "C" void graphics_set_buffer(uint8_t* buffer, uint16_t width, uint16_t height);
extern "C" void graphics_set_scanlines(bool enabled);
extern "C" void graphics_set_dither(bool enabled);
extern "C" void hdmi_reinit(void);
extern "C" void vga_reinit(void);
// Place hot video functions in SRAM instead of XIP flash
#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("video")

#pragma GCC optimize("O3")

VGA8Bit VIDEO::vga;

extern "C" uint8_t* getLineBuffer(int line) {
    if (!VIDEO::vga.frameBuffer) return 0;
    return (uint8_t*)VIDEO::vga.frameBuffer[line];
}

extern "C" void ESPectrum_vsync() {
    ESPectrum::v_sync = true;
}

extern "C" int __not_in_flash_func(get_video_mode)() {
    return VIDEO::video_mode;
}

extern "C" int get_framebuffer_width() {
    return VIDEO::vga.xres;
}

extern "C" int get_framebuffer_height() {
    return VIDEO::vga.yres;
}

// extern "C" video_mode_t ESPectrum_VideoMode() {
//     return VIDEO::video_mode[0];
// }

#ifdef VGA_HDMI
extern bool SELECT_VGA;
int VIDEO::video_mode = 0;
#endif

uint16_t VIDEO::spectrum_colors[NUM_SPECTRUM_COLORS] = {
    BLACK,     BLUE,     RED,     MAGENTA,     GREEN,     CYAN,     YELLOW,     WHITE,
    BRI_BLACK, BRI_BLUE, BRI_RED, BRI_MAGENTA, BRI_GREEN, BRI_CYAN, BRI_YELLOW, BRI_WHITE,
    ORANGE
};

uint8_t VIDEO::borderColor = 0;
uint32_t VIDEO::brd;
uint32_t VIDEO::border32[8];
uint8_t VIDEO::flashing = 0;
uint8_t VIDEO::flash_ctr= 0;
uint8_t VIDEO::OSD = 0;
uint8_t VIDEO::tStatesPerLine;
int VIDEO::tStatesScreen;
int VIDEO::tStatesBorder;
uint8_t* VIDEO::grmem;
uint16_t VIDEO::offBmp[SPEC_H];
uint16_t VIDEO::offAtt[SPEC_H];
SaveRectT VIDEO::SaveRect;
int VIDEO::VsyncFinetune[2];
uint32_t VIDEO::framecnt = 0;
uint8_t VIDEO::dispUpdCycle;
uint8_t VIDEO::att1;
uint8_t VIDEO::bmp1;
uint8_t VIDEO::att2;
uint8_t VIDEO::bmp2;
bool VIDEO::snow_att = false;
bool VIDEO::dbl_att = false;
// bool VIDEO::opCodeFetch;
uint8_t VIDEO::lastbmp;
uint8_t VIDEO::lastatt;    
uint8_t VIDEO::snowpage;
uint8_t VIDEO::snowR;
bool VIDEO::snow_toggle = false;

// Border column variables (runtime-switchable per machine)
// 48K/128K: step=4, brdPairWrite=true (uint32_t pair writes via brdptr16 cast)
// Pentagon:  step=1, brdPairWrite=false (uint16_t XOR writes)
// brdcol_cnt always counts in T-states (1T = 2px = 1 uint16_t)
static int brdcol_start = 0;       // first visible column (T-states from line start)
static int brdcol_end = 0;         // end of visible line (T-states)
static int brdcol_end1 = 0;        // end of left border / paper skip point (T-states)
static int brdcol_retrace = 0;     // where H-retrace begins (= brdcol_end when no retrace visible)
static int brdcol_step = 4;        // T-states per column (4 for 48K/128K, 1 for Pentagon)
static bool brdPairWrite = true;   // true: uint32_t pair writes, false: uint16_t XOR
static void Select_Update_Border(); // forward declaration

// Timex SCLD video modes
#if !PICO_RP2040
uint8_t VIDEO::timex_port_ff = 0;
uint8_t VIDEO::timex_mode = 0;
uint8_t VIDEO::timex_hires_ink = 0;

#endif

// ULA+
#if !PICO_RP2040
bool VIDEO::ulaplus_enabled = false;
uint8_t VIDEO::ulaplus_reg = 0;
// Default palette: standard Spectrum colors in G3R3B2 format
// G/R=5 for normal (truncates to 2-bit level 2), G/R=7 for bright (level 3)
// B=2 for normal, B=3 for bright
// Color order: Black, Blue, Red, Magenta, Green, Cyan, Yellow, White
static const uint8_t ulaplus_default_palette[64] = {
    // CLUT 0 (FLASH=0, BRIGHT=0): INK 0-7, PAPER 0-7
    0x00, 0x02, 0x14, 0x16, 0xA0, 0xA2, 0xB4, 0xB6,
    0x00, 0x02, 0x14, 0x16, 0xA0, 0xA2, 0xB4, 0xB6,
    // CLUT 1 (FLASH=0, BRIGHT=1): INK 0-7, PAPER 0-7
    0x00, 0x03, 0x1C, 0x1F, 0xE0, 0xE3, 0xFC, 0xFF,
    0x00, 0x03, 0x1C, 0x1F, 0xE0, 0xE3, 0xFC, 0xFF,
    // CLUT 2 (FLASH=1, BRIGHT=0): same as CLUT 0
    0x00, 0x02, 0x14, 0x16, 0xA0, 0xA2, 0xB4, 0xB6,
    0x00, 0x02, 0x14, 0x16, 0xA0, 0xA2, 0xB4, 0xB6,
    // CLUT 3 (FLASH=1, BRIGHT=1): same as CLUT 1
    0x00, 0x03, 0x1C, 0x1F, 0xE0, 0xE3, 0xFC, 0xFF,
    0x00, 0x03, 0x1C, 0x1F, 0xE0, 0xE3, 0xFC, 0xFF,
};
uint8_t VIDEO::ulaplus_palette[64];
bool VIDEO::ulaplus_palette_dirty = false;
// AluBytesUlaPlus moved to flash — see roms/AluBytesUlaPlus.c
#endif

#ifdef DIRTY_LINES
uint8_t VIDEO::dirty_lines[SPEC_H];
// uint8_t VIDEO::linecalc[SPEC_H];
#endif //  DIRTY_LINES
static unsigned int is169;
static unsigned int isFullBorder;
static unsigned int lineptr_offset; // uint32_t offset for screen start in line buffer

static uint32_t* lineptr32;
static uint16_t* prevLineptr16; // 4-bit packed: 1 uint16 = 4 pixels

static unsigned int tstateDraw; // Drawing start point (in Tstates)
static unsigned int linedraw_cnt;
static int brdcol_cnt = 0;
static int brdlin_cnt = 0;
static unsigned int lin_end, lin_end2 /*, lin_end3*/;

static unsigned int coldraw_cnt;
static unsigned int col_end;
static unsigned int video_rest;
static unsigned int video_opcode_rest;
static unsigned int curline;

static unsigned int bmpOffset;  // offset for bitmap in graphic memory
static unsigned int attOffset;  // offset for attrib in graphic memory

#if !PICO_RP2040
// Per-scanline DMA attr shadow: non-null when DMA wrote attrs for current scanline
static const uint8_t* dma_attr_override = nullptr;
#endif

static const uint8_t wait_st[128] = {
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
    6, 5, 4, 3, 2, 1, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0,
}; // sequence of wait states

IRAM_ATTR void VGA8Bit::interrupt(void *arg) {
    static int64_t prevmicros = 0;
    static int64_t elapsedmicros = 0;
    static int cntvsync = 0;

    if (Config::tape_player /* || Config::real_player */ ) {
        ESPectrum::v_sync = true;
        return;
    }

    int64_t currentmicros = time_us_64(); /// esp_timer_get_time();

    if (prevmicros) {

        elapsedmicros += currentmicros - prevmicros;

        if (elapsedmicros >= ESPectrum::target) {

            ESPectrum::v_sync = true;

            // This code is needed to "finetune" the sync. Without it, vsync and emu video gets slowly desynced.
            if (VIDEO::VsyncFinetune[0]) {
                if (cntvsync++ == VIDEO::VsyncFinetune[1]) {
                    elapsedmicros += VIDEO::VsyncFinetune[0];
                    cntvsync = 0;
                }
            }

            elapsedmicros -= ESPectrum::target;

        } else ESPectrum::v_sync = false;
    
    } else {

        elapsedmicros = 0;
        ESPectrum::v_sync = false;

    }

    prevmicros = currentmicros;

}

void (*VIDEO::Draw)(unsigned int, bool) = &VIDEO::Blank;
void (*VIDEO::Draw_Opcode)(bool) = &VIDEO::Blank_Opcode;
void (*VIDEO::Draw_OSD169)(unsigned int, bool) = &VIDEO::MainScreen;
void (*VIDEO::Draw_OSD43)() = &VIDEO::BottomBorder;

void (*VIDEO::DrawBorder)() = &VIDEO::TopBorder_Blank;


static uint16_t* brdptr16;
static uint8_t* prevBrdptr8; // 4-bit packed: 1 byte = 2 pixels

uint32_t VIDEO::lastBrdTstate;
bool VIDEO::brdChange = false;
bool VIDEO::brdnextframe = true;
bool VIDEO::brdGigascreenChange = true;
bool VIDEO::gigascreen_enabled = false;
uint8_t VIDEO::gigascreen_auto_countdown = 0;

// void precalcColors() {
    
//     for (int i = 0; i < NUM_SPECTRUM_COLORS; i++) {
//         // printf("RGBAXMask: %d, SBits: %d\n",(int)VIDEO::vga.RGBAXMask,(int)VIDEO::vga.SBits);
//         // printf("Before: %d -> %d, ",i,(int)spectrum_colors[i]);
//         spectrum_colors[i] = (spectrum_colors[i] & VIDEO::vga.RGBAXMask) | VIDEO::vga.SBits;
//         // printf("After : %d -> %d\n",i,(int)spectrum_colors[i]);
//     }

// }

// void precalcAluBytes() {


//     uint16_t specfast_colors[128]; // Array for faster color calc in Draw

//     unsigned int pal[2],b0,b1,b2,b3;

//     // Calc array for faster color calcs in Draw
//     for (int i = 0; i < (NUM_SPECTRUM_COLORS >> 1); i++) {
//         // Normal
//         specfast_colors[i] = spectrum_colors[i];
//         specfast_colors[i << 3] = spectrum_colors[i];
//         // Bright
//         specfast_colors[i | 0x40] = spectrum_colors[i + (NUM_SPECTRUM_COLORS >> 1)];
//         specfast_colors[(i << 3) | 0x40] = spectrum_colors[i + (NUM_SPECTRUM_COLORS >> 1)];
//     }

//     // // Alloc ALUbytes
//     // for (int i = 0; i < 16; i++) {
//     //     AluBytes[i] = (uint32_t *) heap_caps_malloc(0x400, MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);
//     // }

//     FILE *f;

//     f = fopen("/sd/alubytes", "w");
//     fprintf(f,"{\n");
    
//     for (int i = 0; i < 16; i++) {
//         fprintf(f,"{");
//         for (int n = 0; n < 256; n++) {
//             pal[0] = specfast_colors[n & 0x78];
//             pal[1] = specfast_colors[n & 0x47];
//             b0 = pal[(i >> 3) & 0x01];
//             b1 = pal[(i >> 2) & 0x01];
//             b2 = pal[(i >> 1) & 0x01];
//             b3 = pal[i & 0x01];

//             // AluBytes[i][n]=b2 | (b3<<8) | (b0<<16) | (b1<<24);

//             int dato = b2 | (b3<<8) | (b0<<16) | (b1<<24);

//             fprintf(f,"0x%08x,",dato);

//         }
//         fprintf(f,"},\n");
//     }    

//     fprintf(f,"},\n");

//     fclose(f);

// }

// Precalc ULA_SWAP
#define ULA_SWAP(y) ((y & 0xC0) | ((y & 0x38) >> 3) | ((y & 0x07) << 3))
void precalcULASWAP() {
    for (int i = 0; i < SPEC_H; i++) {
        VIDEO::offBmp[i] = ULA_SWAP(i) << 5;
        VIDEO::offAtt[i] = ((i >> 3) << 5) + 0x1800;
        // VIDEO::linecalc[i] = ULA_SWAP(i);
    }
}

void precalcborder32()
{
    for (int i = 0; i < 8; i++) {
        uint8_t border = zxColor(i,0);
        VIDEO::border32[i] = border | (border << 8) | (border << 16) | (border << 24);
    }
}

// Palette definitions (Unreal Speccy format)
// Brightness levels: ZZ (black), NN (normal), BB (bright)
// Color matrix 3x3: values 0..0x100 (0x100 = 1.0)
struct PaletteDef {
    uint8_t ZZ, NN, BB;         // brightness levels
    uint16_t matrix[9];         // color transform matrix
};

static const PaletteDef builtin_palette_defs[] = {
    // 0: Pulsar (ZZ=00, NN=CD, BB=FF, identity matrix)
    { 0x00, 0xCD, 0xFF, { 0x100,0x000,0x000, 0x000,0x100,0x000, 0x000,0x000,0x100 } },
    // 1: Alone (dimmer normal colors: NN=A0)
    { 0x00, 0xA0, 0xFF, { 0x100,0x000,0x000, 0x000,0x100,0x000, 0x000,0x000,0x100 } },
    // 2: Grayscale (Grey from Unreal: BT.601 luminance matrix)
    { 0x00, 0xCD, 0xFF, { 0x049,0x092,0x024, 0x049,0x092,0x024, 0x049,0x092,0x024 } },
    // 3: Mars (warm tones)
    { 0x00, 0xCD, 0xFF, { 0x100,0x000,0x000, 0x040,0x0C0,0x000, 0x000,0x040,0x0C0 } },
    // 4: Ocean (cool tones)
    { 0x00, 0xCD, 0xFF, { 0x0D0,0x000,0x030, 0x000,0x0D0,0x030, 0x000,0x000,0x100 } },
};
#define BUILTIN_PALETTE_COUNT (sizeof(builtin_palette_defs) / sizeof(builtin_palette_defs[0]))

static const char* builtin_palette_names[] = {
    "Pulsar", "Alone", "Grayscale", "Mars", "Ocean"
};

// Custom palettes from /palette.nvs
#if PICO_RP2040
#define MAX_CUSTOM_PALETTES 4
#else
#define MAX_CUSTOM_PALETTES 11
#endif
static PaletteDef custom_palette_defs[MAX_CUSTOM_PALETTES];
static char custom_palette_names[MAX_CUSTOM_PALETTES][13]; // 12 chars + null
static uint8_t custom_palette_count = 0;

static uint8_t total_palette_count() {
    return BUILTIN_PALETTE_COUNT + custom_palette_count;
}

static const PaletteDef& getPaletteDef(uint8_t idx) {
    if (idx < BUILTIN_PALETTE_COUNT)
        return builtin_palette_defs[idx];
    uint8_t ci = idx - BUILTIN_PALETTE_COUNT;
    if (ci < custom_palette_count)
        return custom_palette_defs[ci];
    return builtin_palette_defs[0];
}

uint8_t VIDEO::paletteCount() { return total_palette_count(); }

const char* VIDEO::paletteName(uint8_t idx) {
    if (idx < BUILTIN_PALETTE_COUNT)
        return builtin_palette_names[idx];
    uint8_t ci = idx - BUILTIN_PALETTE_COUNT;
    if (ci < custom_palette_count)
        return custom_palette_names[ci];
    return "?";
}

// Parse hex value from string (up to 3 hex digits)
static uint16_t parseHex(const char* s, int len) {
    uint16_t v = 0;
    for (int i = 0; i < len && s[i]; i++) {
        char c = s[i];
        v <<= 4;
        if (c >= '0' && c <= '9') v |= c - '0';
        else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
        else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10;
    }
    return v;
}

// Load custom palettes from /palette.nvs
// Format per line (JSON-like, one object per line):
// { "name": "MyPal", "ZZ": 0x00, "NN": 0xCD, "BB": 0xFF, "matrix": [0x100,0x000,...] }
// Lines starting with // are comments and ignored.
// Simplified parser: extract name, ZZ, NN, BB, and 9 matrix values from hex
void VIDEO::loadCustomPalettes() {
    custom_palette_count = 0;
    if (!FileUtils::fsMount) return;

    // Create default palette.nvs if it doesn't exist
    {
        FILINFO fi;
        if (f_stat(MOUNT_POINT_SD "/palette.nvs", &fi) != FR_OK) {
            FIL cf;
            if (f_open(&cf, MOUNT_POINT_SD "/palette.nvs", FA_WRITE | FA_CREATE_NEW) == FR_OK) {
                static const char tmpl[] =
                    "// Custom palettes for ZX Spectrum emulator\n"
                    "//\n"
                    "// One palette per line. Lines starting with // are comments.\n"
                    "// To enable a palette, remove the // prefix.\n"
                    "//\n"
                    "// Fields:\n"
                    "//   name   - palette name shown in menu (max 12 chars)\n"
                    "//   ZZ     - black level (hex byte, usually 0x00)\n"
                    "//   NN     - normal brightness (hex byte, e.g. 0xCD)\n"
                    "//   BB     - bright brightness (hex byte, e.g. 0xFF)\n"
                    "//   matrix - 3x3 RGB color transform (9 hex values, 0x000..0x100)\n"
                    "//            rows: R-out, G-out, B-out; cols: R-in, G-in, B-in\n"
                    "//            0x100 = 1.0 (identity), 0x080 = 0.5, 0x000 = 0.0\n"
                    "//\n"
                    "//   Matrix formula:        [m0 m1 m2]\n"
                    "//     oR = (R*m0 + G*m1 + B*m2) >> 8\n"
                    "//     oG = (R*m3 + G*m4 + B*m5) >> 8\n"
                    "//     oB = (R*m6 + G*m7 + B*m8) >> 8\n"
                    "//\n"
                    "// Example (identity palette, same as Pulsar):\n"
                    "// { \"name\": \"MyPal\", \"ZZ\": 0x00, \"NN\": 0xCD, \"BB\": 0xFF, \"matrix\": [0x100,0x000,0x000, 0x000,0x100,0x000, 0x000,0x000,0x100] }\n";
                UINT bw;
                f_write(&cf, tmpl, sizeof(tmpl) - 1, &bw);
                f_close(&cf);
            }
        }
    }

    FIL fil;
    if (f_open(&fil, MOUNT_POINT_SD "/palette.nvs", FA_READ) != FR_OK)
        return;

    char line[256];
    UINT br;
    int linepos = 0;

    auto processLine = [&](const char* line) {
        if (custom_palette_count >= MAX_CUSTOM_PALETTES) return;
        // Skip empty lines and comments (// or #)
        const char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\0' || *p == '\n') return;
        if (p[0] == '/' && p[1] == '/') return;

        // Find "name": "..."
        const char* nq = strstr(p, "\"name\"");
        if (!nq) return;
        nq = strchr(nq + 6, '\"');
        if (!nq) return;
        nq++; // start of name
        const char* nqe = strchr(nq, '\"');
        if (!nqe) return;
        int nlen = nqe - nq;
        if (nlen > 12) nlen = 12;

        PaletteDef& pal = custom_palette_defs[custom_palette_count];
        char* name = custom_palette_names[custom_palette_count];
        memcpy(name, nq, nlen);
        name[nlen] = '\0';

        // Parse "ZZ": 0xHH
        auto findHexVal = [](const char* s, const char* key) -> uint16_t {
            const char* k = strstr(s, key);
            if (!k) return 0;
            k += strlen(key);
            while (*k == ' ' || *k == ':' || *k == '\t') k++;
            if (k[0] == '0' && (k[1] == 'x' || k[1] == 'X')) k += 2;
            // Read up to 3 hex digits
            int len = 0;
            while (len < 3 && ((k[len] >= '0' && k[len] <= '9') ||
                   (k[len] >= 'A' && k[len] <= 'F') ||
                   (k[len] >= 'a' && k[len] <= 'f'))) len++;
            return parseHex(k, len);
        };

        pal.ZZ = (uint8_t)findHexVal(p, "\"ZZ\"");
        pal.NN = (uint8_t)findHexVal(p, "\"NN\"");
        pal.BB = (uint8_t)findHexVal(p, "\"BB\"");

        // Parse "matrix": [0x100, 0x000, ...]
        const char* mq = strstr(p, "\"matrix\"");
        if (!mq) return;
        mq = strchr(mq, '[');
        if (!mq) return;
        mq++;
        for (int i = 0; i < 9; i++) {
            while (*mq == ' ' || *mq == ',') mq++;
            if (*mq == '0' && (mq[1] == 'x' || mq[1] == 'X')) mq += 2;
            int len = 0;
            while (len < 3 && ((mq[len] >= '0' && mq[len] <= '9') ||
                   (mq[len] >= 'A' && mq[len] <= 'F') ||
                   (mq[len] >= 'a' && mq[len] <= 'f'))) len++;
            if (len == 0) return;
            pal.matrix[i] = parseHex(mq, len);
            mq += len;
        }

        custom_palette_count++;
    };

    // Read char by char, process lines
    linepos = 0;
    char c;
    while (f_read(&fil, &c, 1, &br) == FR_OK && br == 1) {
        if (c == '\n' || c == '\r') {
            if (linepos > 0) {
                line[linepos] = '\0';
                processLine(line);
                linepos = 0;
            }
        } else if (linepos < 255) {
            line[linepos++] = c;
        }
    }
    if (linepos > 0) {
        line[linepos] = '\0';
        processLine(line);
    }
    f_close(&fil);
}

// Build 16 Spectrum RGB888 colors from brightness levels
// Color bit pattern: index 0-7 normal (B=NN), 8-15 bright (B=BB)
// Bit 0=Blue, Bit 1=Red, Bit 2=Green; Bit 3=Bright
static void buildSpectrumRGB(const PaletteDef &pal, uint32_t out[16]) {
    for (int i = 0; i < 16; i++) {
        uint8_t lvl = (i & 8) ? pal.BB : pal.NN;
        uint8_t R = (i & 2) ? lvl : pal.ZZ;
        uint8_t G = (i & 4) ? lvl : pal.ZZ;
        uint8_t B = (i & 1) ? lvl : pal.ZZ;
        out[i] = (R << 16) | (G << 8) | B;
    }
}

// Standard Spectrum RGB888 palette (default, rebuilt on palette change)
static uint32_t spectrum_rgb888[16];

// Initialize spectrum_rgb888 with default palette
static void initDefaultPalette() {
    buildSpectrumRGB(builtin_palette_defs[0], spectrum_rgb888);
}

#if !PICO_RP2040
void initGigascreenBlendLUT();
#endif

// Apply color matrix transform to an RGB888 color
static inline uint32_t matrixTransform(uint32_t rgb, const uint16_t *m) {
    uint32_t R = (rgb >> 16) & 0xFF;
    uint32_t G = (rgb >> 8) & 0xFF;
    uint32_t B = rgb & 0xFF;
    uint32_t oR = (R * m[0] + G * m[1] + B * m[2]) >> 8;
    uint32_t oG = (R * m[3] + G * m[4] + B * m[5]) >> 8;
    uint32_t oB = (R * m[6] + G * m[7] + B * m[8]) >> 8;
    if (oR > 255) oR = 255;
    if (oG > 255) oG = 255;
    if (oB > 255) oB = 255;
    return (oR << 16) | (oG << 8) | oB;
}

// Apply current palette transform to an RGB888 color
static inline uint32_t paletteTransform(uint32_t rgb) {
    uint8_t p = Config::palette < total_palette_count() ? Config::palette : 0;
    const uint16_t *m = getPaletteDef(p).matrix;
    // Check if identity matrix (fast path)
    if (m[0] == 0x100 && m[4] == 0x100 && m[8] == 0x100 &&
        m[1] == 0 && m[2] == 0 && m[3] == 0 && m[5] == 0 && m[6] == 0 && m[7] == 0)
        return rgb;
    return matrixTransform(rgb, m);
}

// ULA+ G3R3B2 to full RGB888 conversion
static inline uint32_t grb_to_rgb888(uint8_t grb) {
    // G3R3B2 format: bits 7:5=green, bits 4:2=red, bits 1:0=blue
    uint8_t g3 = (grb >> 5) & 0x07;
    uint8_t r3 = (grb >> 2) & 0x07;
    uint8_t b2 = grb & 0x03;
    // Scale: 3-bit (0-7) → 8-bit, 2-bit (0-3) → 8-bit
    uint8_t R = (r3 << 5) | (r3 << 2) | (r3 >> 1);
    uint8_t G = (g3 << 5) | (g3 << 2) | (g3 >> 1);
    uint8_t B = (b2 << 6) | (b2 << 4) | (b2 << 2) | b2;
    return (R << 16) | (G << 8) | B;
}

// Bayer dither neighbour: brighten each channel by ~1 VGA-step (saturating).
// Used only by HDMI dither path; mimics the visual look of VGA Bayer.
static inline uint32_t dither_neighbour(uint32_t rgb) {
    int R = ((rgb >> 16) & 0xFF) + 8; if (R > 255) R = 255;
    int G = ((rgb >> 8) & 0xFF) + 8; if (G > 255) G = 255;
    int B = (rgb & 0xFF) + 8; if (B > 255) B = 255;
    return (R << 16) | (G << 8) | B;
}

#if !PICO_RP2040
// Apply ULA+ palette entry i (and Bayer-dither neighbour at i|0x40 for HDMI)
static inline void applyUlaPlusPalette(int i) {
    uint32_t base = paletteTransform(grb_to_rgb888(VIDEO::ulaplus_palette[i]));
    graphics_set_palette(i, base);
    // Always populate the dither slot so toggling Config::hdmi_dither at runtime
    // does not require a palette rebuild. When dither is off, hdmi.c does not
    // read palette[i|0x40], so the cost is just an extra LUT write per entry.
    graphics_set_palette(i | 0x40, dither_neighbour(base));
}

void VIDEO::regenerateUlaPlusAluBytes() {
    for (int i = 0; i < 64; i++) applyUlaPlusPalette(i);

    // Point AluByte to flash-resident ULA+ table (palette indices 0-63,
    // fixed mapping that depends only on attribute format, not palette contents)
    for (int n = 0; n < 16; n++)
        AluByte[n] = (unsigned int*)AluBytesUlaPlus_flash[n];

    graphics_set_dither(Config::hdmi_dither);
}

// Fast path: update single palette entry without rebuilding AluBytes
// Deferred: only marks dirty, actual conv_color update happens in ulaPlusFlushPalette()
// to avoid palette tearing when HDMI DMA scans top lines before Z80 ISR finishes
void VIDEO::ulaPlusUpdatePaletteEntry(uint8_t entry) {
    ulaplus_palette_dirty = true;
}

// Apply all pending ULA+ palette changes to hardware (conv_color / VGA LUT).
// Called from EndFrame() so palette is stable before HDMI active area begins.
void VIDEO::ulaPlusFlushPalette() {
    if (!ulaplus_palette_dirty) return;
    ulaplus_palette_dirty = false;
    for (int i = 0; i < 64; i++) applyUlaPlusPalette(i);
}

void VIDEO::ulaPlusUpdateBorder() {
    // ULA+ border = paper color from CLUT 0 for current borderColor
    // CLUT 0 paper entries are at indices 8-15, so index = 8 + borderColor
    uint8_t brd_color = 8 + borderColor;
    brd = brd_color | (brd_color << 8) | (brd_color << 16) | (brd_color << 24);
    brdChange = true;
}

void VIDEO::ulaPlusDisable() {
    ulaplus_enabled = false;
    ulaplus_palette_dirty = false;
    flashing = 0;
    // Dither is meaningful only with ULA+; force it off so palette[64..127]
    // (which now hold default G3R3B2 values, not Bayer neighbours) won't be
    // sampled by the HDMI ISR.
    graphics_set_dither(false);
    // Restore palette: indices 0-63 back to G3R3B2 defaults, then 0-15 to Spectrum (solid)
    for (int i = 0; i < 64; i++)
        graphics_set_palette(i, paletteTransform(grb_to_rgb888(i)));
    for (int i = 0; i < 16; i++) {
        uint32_t color = paletteTransform(spectrum_rgb888[i]);
        graphics_set_palette(i, color);
        vga_set_palette_entry_solid(i, color);
    }

    // Restore GigaScreen blend palette (slots 17-136) if GigaScreen is configured,
    // since palette entries 17-63 were overwritten by ULA+ above
    if (Config::gigascreen_enabled)
        initGigascreenBlendLUT();

    for (int n = 0; n < 16; n++)
        AluByte[n] = (unsigned int*)AluBytesStd_flash[n];
    brd = border32[borderColor];
    brdChange = true;
}
#endif

// Apply palette: rebuild spectrum_rgb888 from current palette's brightness levels,
// then apply color matrix to all palette entries.
void VIDEO::applyPalette() {
    uint8_t p = Config::palette < total_palette_count() ? Config::palette : 0;
    Debug::log2SD("applyPalette: palette=%d", p);
    // Rebuild base 16 colors from brightness levels
    buildSpectrumRGB(getPaletteDef(p), spectrum_rgb888);

    Debug::log2SD("applyPalette: spectrum_rgb888[0]=0x%06X [1]=0x%06X [7]=0x%06X [15]=0x%06X",
        spectrum_rgb888[0], spectrum_rgb888[1], spectrum_rgb888[7], spectrum_rgb888[15]);

    // G3R3B2 entries (0-239) — apply matrix transform
    for (int i = 0; i < 240; i++)
        graphics_set_palette(i, paletteTransform(grb_to_rgb888(i)));
    // Override indices 0-15 with Spectrum colors (already rebuilt with correct brightness)
    for (int i = 0; i < 16; i++) {
        uint32_t color = paletteTransform(spectrum_rgb888[i]);
        graphics_set_palette(i, color);
        vga_set_palette_entry_solid(i, color);
    }
    // Orange (index 16)
    graphics_set_palette(16, paletteTransform(0xFF7F00));

    Debug::log2SD("applyPalette: done, 240+16+1 entries written");

    // Re-apply GigaScreen blend palette if active
#if !PICO_RP2040
    if (Config::gigascreen_enabled)
        initGigascreenBlendLUT();
#endif
}

// Fill 256-entry BMP palette (1024 bytes, BGRA format) matching current VGA palette.
// Replicates the same logic as applyPalette() but writes BMP BGRA entries instead
// of programming the VGA hardware.
void VIDEO::getBmpPalette(uint8_t* out) {
    // G3R3B2 entries (0-239) with palette transform
    for (int i = 0; i < 240; i++) {
        uint32_t c = paletteTransform(grb_to_rgb888(i));
        out[i * 4 + 0] = c & 0xFF;         // B
        out[i * 4 + 1] = (c >> 8) & 0xFF;  // G
        out[i * 4 + 2] = (c >> 16) & 0xFF; // R
        out[i * 4 + 3] = 0;                 // A
    }
    // Indices 240-255: same G3R3B2 fallback
    for (int i = 240; i < 256; i++) {
        uint32_t c = paletteTransform(grb_to_rgb888(i));
        out[i * 4 + 0] = c & 0xFF;
        out[i * 4 + 1] = (c >> 8) & 0xFF;
        out[i * 4 + 2] = (c >> 16) & 0xFF;
        out[i * 4 + 3] = 0;
    }
    // Override indices 0-15 with Spectrum colors (matching applyPalette)
    for (int i = 0; i < 16; i++) {
        uint32_t c = paletteTransform(spectrum_rgb888[i]);
        out[i * 4 + 0] = c & 0xFF;
        out[i * 4 + 1] = (c >> 8) & 0xFF;
        out[i * 4 + 2] = (c >> 16) & 0xFF;
        out[i * 4 + 3] = 0;
    }
    // Orange (index 16)
    {
        uint32_t c = paletteTransform(0xFF7F00);
        out[16 * 4 + 0] = c & 0xFF;
        out[16 * 4 + 1] = (c >> 8) & 0xFF;
        out[16 * 4 + 2] = (c >> 16) & 0xFF;
        out[16 * 4 + 3] = 0;
    }
#if !PICO_RP2040
    // ULA+ palette override (indices 0-63)
    if (ulaplus_enabled) {
        for (int i = 0; i < 64; i++) {
            uint32_t c = paletteTransform(grb_to_rgb888(ulaplus_palette[i]));
            out[i * 4 + 0] = c & 0xFF;
            out[i * 4 + 1] = (c >> 8) & 0xFF;
            out[i * 4 + 2] = (c >> 16) & 0xFF;
            out[i * 4 + 3] = 0;
        }
    }
#endif
}

const int redPins[] = {RED_PINS_6B};
const int grePins[] = {GRE_PINS_6B};
const int bluPins[] = {BLU_PINS_6B};

void VIDEO::vgataskinit(void *unused) {
    uint8_t Mode;
    Mode = 16 + ((Config::arch == "48K") ? 0 : (Config::arch == "128K" || Config::arch == "ALF" ? 2 : 4)) + (Config::aspect_16_9 ? 1 : 0);
    OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
    OSD::scrH = vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv];
    vga.useInterrupt_flag = true;
    // Init mode
    vga.init(Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);    
    for (;;){}    
}

///TaskHandle_t VIDEO::videoTaskHandle;

#if !PICO_RP2040
// Shared framebuffer data block — allocated once at boot for max resolution (360x288).
// Both main FB and prevFB (Gigascreen) live in the same contiguous block.
// changeMode() reconfigures pointer arrays without any alloc/free.
#define FB_MAX_LINES 289   // calcLines(288)
#define FB_MAX_STRIDE 360
#define FB_MAX_SIZE (FB_MAX_LINES * FB_MAX_STRIDE)  // 104,040 bytes per FB
// prevFrameBuffer stores only lower 4 bits of palette index per pixel
// (Gigascreen blendLUT only uses prev & 0x0F). Pack 2 px per byte → half size.
#define FB_PREV_STRIDE (FB_MAX_STRIDE / 2)
#define FB_PREV_SIZE (FB_MAX_LINES * FB_PREV_STRIDE)  // 52,020 bytes

static int fbCalcLines(int count) {
    if (count == 288) return 289;
    if (count == 240) return 241;
    if (count == 480) return 241;
    if (count == 400) return 201;
    return count;
}

static uint8_t *sharedFB_block = nullptr;
static void **sharedFB_arr1 = nullptr;  // pointer array for frameBuffer
static void **sharedFB_arr2 = nullptr;  // pointer array for prevFrameBuffer

static void setupSharedFBPointers(Graphics<unsigned char> &vga, int lines, int stride) {
    int prev_stride = stride / 2; // 4-bit packed
    for (int i = 0; i < lines; i++) {
        sharedFB_arr1[i] = sharedFB_block + i * stride;
        sharedFB_arr2[i] = sharedFB_block + FB_MAX_SIZE + i * prev_stride;
    }
    vga.frameBuffer = (unsigned char **)sharedFB_arr1;
    vga.prevFrameBuffer = (unsigned char **)sharedFB_arr2;
}
#endif

void VIDEO::Init() {
    int Mode;
#ifdef VGA_HDMI
    if (VIDEO::isFullBorder288()) {
        Mode = 22; // VgaMode_360x288 — full border 360x288
    } else if (VIDEO::isFullBorder240()) {
        Mode = 23; // VgaMode_360x240 — half border 360x240
    } else
#endif
    {
        Mode = Config::aspect_16_9 ? 2 : 0;
    }
    OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
    OSD::scrH = vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv];
    vga.useInterrupt_flag = false;

#if !PICO_RP2040
    // Allocate shared data block for main + prev framebuffers at max resolution
    // BEFORE vga.init() — while heap is still unfragmented (full MEM_REMAIN available).
    // The block is never freed; changeMode() only reconfigures pointer arrays.
    if (!sharedFB_block) {
        sharedFB_block = (uint8_t *)malloc((FB_MAX_SIZE + FB_PREV_SIZE));
        if (sharedFB_block) {
            memset(sharedFB_block, 0, (FB_MAX_SIZE + FB_PREV_SIZE));
            sharedFB_arr1 = (void **)malloc(FB_MAX_LINES * sizeof(void *));
            sharedFB_arr2 = (void **)malloc(FB_MAX_LINES * sizeof(void *));
            if (!sharedFB_arr1 || !sharedFB_arr2) {
                free(sharedFB_block); sharedFB_block = nullptr;
                free(sharedFB_arr1); sharedFB_arr1 = nullptr;
                free(sharedFB_arr2); sharedFB_arr2 = nullptr;
            }
        }
    }
    if (sharedFB_block) {
        int lines = fbCalcLines(
            vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv]);
        int stride = (vidmodes[Mode][vmodeproperties::hRes] + 3) & ~3;
        setupSharedFBPointers(vga, lines, stride);
        // frameBuffer is set — vga.init()'s allocateFrameBuffers() will skip allocation
    }
#endif

    vga.init( Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);

    graphics_set_scanlines(Config::scanlines);

    // Generate AluBytes table with palette indices (no sync bits)
    initAluBytes();

    precalcULASWAP();   // precalculate ULA SWAP values

    precalcborder32();  // Precalc border 32 bits values

    // Build and apply palette (brightness levels + color matrix)
    applyPalette();

#if !PICO_RP2040
    if (Config::gigascreen_enabled && vga.prevFrameBuffer)
    {
        VIDEO::gigascreen_enabled = (Config::gigascreen_onoff == 1); // On=enabled, Auto=start disabled
        VIDEO::gigascreen_auto_countdown = 0;
        initGigascreenBlendLUT(); // Pre-compute blend palette entries
    }
#endif
}

static void freeFrameBuffer(void **fb) {
    if (!fb) return;
    free(fb[0]);  // contiguous data block allocated by heap_caps_malloc
    free(fb);     // pointer array allocated by malloc
}

#ifdef VGA_HDMI
void VIDEO::changeMode() {
    // 1. Determine new VGA Mode index (same logic as Init())
    int Mode;
    if (VIDEO::isFullBorder288()) {
        Mode = 22;
    } else if (VIDEO::isFullBorder240()) {
        Mode = 23;
    } else {
        Mode = Config::aspect_16_9 ? 2 : 0;
    }

    int newW = vidmodes[Mode][vmodeproperties::hRes];
    int newH = vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv];

    bool sameDims = (vga.frameBuffer && vga.xres == newW && vga.yres == newH);

#if !PICO_RP2040
    // Shared block path: no alloc/free, just reconfigure pointers
    if (sharedFB_block) {
        if (!sameDims) {
            vga.frameBuffer = nullptr;  // blank output while reconfiguring
            SaveRect.clear();
            int lines = fbCalcLines(newH);
            int stride = (newW + 3) & ~3;
            setupSharedFBPointers(vga, lines, stride);
        }
    } else
#endif
    {
        // Non-shared fallback (RP2040 always; RP2350 only if shared alloc failed).
        // prevFrameBuffer is RP2350-only (Gigascreen) — guard the cleanup.
#if !PICO_RP2040
        if (vga.prevFrameBuffer) {
            auto oldPrev = vga.prevFrameBuffer;
            vga.prevFrameBuffer = nullptr;
            freeFrameBuffer((void**)oldPrev);
        }
#endif
        // Only null FB when dims change (alloc step below will rebuild it).
        // If sameDims, keep current FB to avoid driver reading NULL.
        if (!sameDims) {
            vga.frameBuffer = nullptr;
        }
    }

    // 2. Update video_mode BEFORE reinit (hdmi_init reads it via get_video_mode())
    if (SELECT_VGA) {
        switch (Config::vga_video_mode) {
            case Config::VM_640x480_50:
                if (Config::arch == "48K") video_mode = 2;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 3;
                else video_mode = 1;
                break;
            case Config::VM_720x480_60: video_mode = 7; break;
            case Config::VM_720x576_60: video_mode = 8; break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K") video_mode = 5;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 6;
                else video_mode = 4;
                break;
            default: video_mode = 0; break;
        }
    } else {
        switch (Config::hdmi_video_mode) {
            case Config::VM_640x480_60: video_mode = 0; break;
            case Config::VM_640x480_50:
                if (Config::arch == "48K") video_mode = 2;
                else if (Config::arch == "128K") video_mode = 3;
                else video_mode = 1;
                break;
            case Config::VM_720x480_60: video_mode = 7; break;
            case Config::VM_720x576_60: video_mode = 8; break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K") video_mode = 5;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 6;
                else video_mode = 4;
                break;
            default: video_mode = 0; break;
        }
    }

    // 3. Update driver buffer dimensions + reinit video output
    vga.mode = Mode;
    vga.xres = newW;
    vga.yres = newH;
    OSD::scrW = newW;
    OSD::scrH = newH;
    graphics_set_buffer(NULL, newW, newH);
    graphics_set_scanlines(Config::scanlines);
    if (SELECT_VGA) {
        vga_reinit();
    } else {
        hdmi_reinit();
    }

    // 4. Allocate framebuffer (non-shared path only)
#if !PICO_RP2040
    if (!sharedFB_block) {
#endif
        if (sameDims) {
            // frameBuffer already nulled above for non-shared; won't reach here for shared
        } else {
            auto oldFB = vga.frameBuffer;
            vga.frameBuffer = nullptr;
            freeFrameBuffer((void**)oldFB);
            vga.frameBuffer = vga.allocateFrameBuffer();
            SaveRect.clear();
        }
#if !PICO_RP2040
    }
#endif

    // 5. Recalculate border timing + precalc tables (preserve border color)
    uint8_t savedBorderColor = borderColor;
    VIDEO::Reset();
    borderColor = savedBorderColor;
    brd = border32[borderColor];
    precalcborder32();

    // 6. Repaint framebuffer with current border color
    if (vga.frameBuffer) {
        int stride = (vga.xres + 3) & ~3;
        memset(vga.frameBuffer[0], zxColor(borderColor, 0), vga.yres * stride);
    }

#if !PICO_RP2040
    // 7. Gigascreen
    if (Config::gigascreen_enabled && vga.prevFrameBuffer) {
        VIDEO::gigascreen_enabled = (Config::gigascreen_onoff == 1);
    } else if (Config::gigascreen_enabled) {
        InitPrevBuffer();
        if (!vga.prevFrameBuffer) {
            Config::gigascreen_enabled = false;
            VIDEO::gigascreen_enabled = false;
        }
    }
#endif
}
#endif

void VIDEO::Reset() {

    borderColor = 7;
    brd = border32[7];

#if !PICO_RP2040
    // Reset Timex SCLD state
    timex_port_ff = 0;
    timex_mode = 0;
    timex_hires_ink = 0;

    // Reset ULA+ state
    if (ulaplus_enabled) ulaPlusDisable();
    ulaplus_reg = 0;
    memcpy(ulaplus_palette, ulaplus_default_palette, 64);
#endif

    is169 = Config::aspect_16_9 ? 1 : 0;
#ifdef VGA_HDMI
    isFullBorder = VIDEO::isFullBorderMode() ? 1 : 0;
#else
    isFullBorder = 0;
#endif

    uint8_t prevOSDstats = OSD & 0x03; // Preserve stats mode across reset
    OSD = 0;

    bool isFullBorder240 = VIDEO::isFullBorder240();

    if (Config::arch == "48K") {
        if (Config::romSet48 == "48Kby") {
            tStatesPerLine = TSTATES_PER_LINE_BYTE;
            tStatesScreen = TS_SCREEN_BYTE;
            tStatesBorder = isFullBorder ? (isFullBorder240 ? TS_BORDER_360x240_BYTE : TS_BORDER_360x288_BYTE)
                          : is169 ? TS_BORDER_360x200_BYTE : TS_BORDER_320x240_BYTE;
        }
        else
        {
            tStatesPerLine = TSTATES_PER_LINE;
            tStatesScreen = TS_SCREEN_48;
            tStatesBorder = isFullBorder ? (isFullBorder240 ? TS_BORDER_360x240 : TS_BORDER_360x288)
                          : is169 ? TS_BORDER_360x200 : TS_BORDER_320x240;
        }
        VsyncFinetune[0] = 0;
        VsyncFinetune[1] = 0;

        Draw_OSD169 = MainScreen;
        Draw_OSD43 = BottomBorder;
        DrawBorder = TopBorder_Blank;
    } else if (Config::arch == "128K" || Config::arch == "ALF") {
        if (Config::romSet128 == "128Kby" || Config::romSet128 == "128Kbg") {
            tStatesPerLine = TSTATES_PER_LINE_BYTE;
            tStatesScreen = TS_SCREEN_BYTE;
            tStatesBorder = isFullBorder ? (isFullBorder240 ? TS_BORDER_360x240_BYTE : TS_BORDER_360x288_BYTE)
                          : is169 ? TS_BORDER_360x200_BYTE : TS_BORDER_320x240_BYTE;
        }
        else
        {
            tStatesPerLine = TSTATES_PER_LINE_128;
            tStatesScreen = TS_SCREEN_128;
            tStatesBorder = isFullBorder ? (isFullBorder240 ? TS_BORDER_360x240_128 : TS_BORDER_360x288_128)
                          : is169 ? TS_BORDER_360x200_128 : TS_BORDER_320x240_128;
        }
        VsyncFinetune[0] = 0;
        VsyncFinetune[1] = 0;

        Draw_OSD169 = MainScreen;
        Draw_OSD43 = BottomBorder;
        DrawBorder = TopBorder_Blank;
    } else if (Config::arch == "Pentagon" || Config::arch == "P512" || Config::arch == "P1024") {
        tStatesPerLine = TSTATES_PER_LINE_PENTAGON;
        tStatesScreen = TS_SCREEN_PENTAGON;
        tStatesBorder = isFullBorder ? (isFullBorder240 ? TS_BORDER_360x240_PENTAGON : TS_BORDER_360x288_PENTAGON)
                      : is169 ? TS_BORDER_360x200_PENTAGON : TS_BORDER_320x240_PENTAGON;
        VsyncFinetune[0] = 0;
        VsyncFinetune[1] = 0;

        Draw_OSD169 = MainScreen;
        Draw_OSD43 = BottomBorder;
        DrawBorder = TopBorder_Blank;
    }

    // Border column layout (unified for all models):
    // brdcol_cnt counts T-states (1T = 2px = 1 uint16_t in framebuffer)
    // 48K/128K: step=4 (8px per column), brdPairWrite=true
    // Pentagon:  step=1 (2px per column), brdPairWrite=false (XOR)
    brdcol_end = isFullBorder ? 180 : 160;  // vga.xres / 2 (T-states = half pixel count)
    if (Z80Ops::isPentagon) {
        brdcol_step = 1;
        brdPairWrite = false;
        brdcol_start = is169 ? 10 : 0;
        if (isFullBorder) {
            brdcol_end1 = 26;
        } else {
            brdcol_end1 = brdcol_start + (brdcol_end - brdcol_start - 128) / 2;
        }
        brdcol_retrace = brdcol_end;    // no retrace visible for Pentagon
    } else {
        brdcol_step = 4;
        brdPairWrite = true;
        brdcol_start = 0;
        brdcol_end1 = isFullBorder ? 24 : 16;  // left border T-states
        brdcol_retrace = brdcol_end;            // no retrace visible with step=4
    }
    Select_Update_Border();

    if (isFullBorder && !isFullBorder240) {
        lin_end = 48;
        lin_end2 = 240;
        lineptr_offset = (Z80Ops::isPentagon ? 26 : 24) / 2;
    } else if (isFullBorder && isFullBorder240) {
        lin_end = 24;
        lin_end2 = 216;
        lineptr_offset = (Z80Ops::isPentagon ? 26 : 24) / 2;
    } else if (is169) {
        lin_end = 4;
        lin_end2 = 196;
        lineptr_offset = 13;
    } else {
        lin_end = 24;
        lin_end2 = 216;
        lineptr_offset = 8;  // 32 bytes = (320-256)/2 pixels
    }

    grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();

    #ifdef DIRTY_LINES
    // for (int i=0; i < SPEC_H; i++) VIDEO::dirty_lines[i] = 0x01;
    memset((uint8_t *)VIDEO::dirty_lines,0x01,SPEC_H);
    #endif // DIRTY_LINES

    VIDEO::snow_toggle = Config::arch != "P1024" && Config::arch != "P512" && Config::arch != "Pentagon" ? Config::render : false;

    if (VIDEO::snow_toggle) {
        Draw = &Blank_Snow;
        Draw_Opcode = &Blank_Snow_Opcode;
    } else {
        Draw = &Blank;
        Draw_Opcode = &Blank_Opcode;
    }

    // Restart border drawing + main screen draw state
    linedraw_cnt = lin_end;
    tstateDraw = tStatesScreen;
    lastBrdTstate = tStatesBorder;
    brdChange = false;
    brdnextframe = true;
    brdcol_cnt = brdcol_start;
    brdlin_cnt = 0;
#ifdef VGA_HDMI
    if (SELECT_VGA)
    {
        switch (Config::vga_video_mode) {
            case Config::VM_640x480_50:
                if (Config::arch == "48K") video_mode = 2;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 3;
                else video_mode = 1; // Pentagon
                break;
            case Config::VM_720x480_60:
                video_mode = 7;
                break;
            case Config::VM_720x576_60:
                video_mode = 8;
                break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K") video_mode = 5;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 6;
                else video_mode = 4; // Pentagon
                break;
            default: // VM_640x480_60
                video_mode = 0;
                break;
        }
    }
    else
    {
        // HDMI: map Config enum to graphics.c video_mode index
        switch (Config::hdmi_video_mode) {
            case Config::VM_640x480_60:
                video_mode = 0;
                break;
            case Config::VM_640x480_50:
                if (Config::arch == "48K") video_mode = 2;
                else if (Config::arch == "128K") video_mode = 3;
                else video_mode = 1; // Pentagon
                break;
            case Config::VM_720x480_60:
                video_mode = 7;
                break;
            case Config::VM_720x576_60:
                video_mode = 8;
                break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K") video_mode = 5;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 6;
                else video_mode = 4; // Pentagon
                break;
            default:
                video_mode = 0;
                break;
        }
    }
#endif

    // Restore stats mode that was active before reset
    if (prevOSDstats) {
        OSD = prevOSDstats;
        if (Config::aspect_16_9)
            Draw_OSD169 = MainScreen_OSD;
        else
            Draw_OSD43 = BottomBorder_OSD;
    }
}

extern size_t getFreeHeap(void);
extern size_t getContiguousHeap(void);

#if !PICO_RP2040
void VIDEO::InitPrevBuffer() {
    if (!vga.prevFrameBuffer) {
        vga.prevFrameBuffer = vga.allocateFrameBuffer();
    }
    if (!vga.prevFrameBuffer) return;
    const int h = VIDEO::vga.yres;
    const int w = VIDEO::vga.xres;
    for (int y = 0; y < h; ++y) {
        uint8_t *src = VIDEO::vga.frameBuffer[y];
        uint8_t *dst = VIDEO::vga.prevFrameBuffer[y];
        if (!src || !dst) continue;
        // Pack 2 src pixels (8-bit palette idx) into 1 dst byte (low+high nibble).
        for (int x = 0; x < w; x += 2) {
            dst[x >> 1] = (src[x] & 0x0F) | ((src[x + 1] & 0x0F) << 4);
        }
    }
}
#endif

//  VIDEO DRAW FUNCTIONS
IRAM_ATTR void VIDEO::MainScreen_Blank(unsigned int statestoadd, bool contended) {    
    
    CPU::tstates += statestoadd;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder(); // Needed to avoid tearing in demos like Gabba (Pentagon)
        
        lineptr32 = (uint32_t *)(vga.frameBuffer[linedraw_cnt]) + lineptr_offset;
        prevLineptr16 = (uint16_t *)(vga.prevFrameBuffer[linedraw_cnt]) + lineptr_offset;

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
#if !PICO_RP2040
        if (Config::timex_video && VIDEO::timex_mode != 0) {
            switch (VIDEO::timex_mode) {
                case 1: // Second screen
                    bmpOffset = 0x2000 + offBmp[curline];
                    attOffset = 0x2000 + offAtt[curline];
                    break;
                case 2: // Hi-colour (8x1 attrs from screen 1, ULA-swapped)
                    bmpOffset = offBmp[curline];
                    attOffset = 0x2000 + offBmp[curline];
                    break;
                case 6: // Hi-res (both screens as bitmap)
                    bmpOffset = offBmp[curline];
                    attOffset = 0x2000 + offBmp[curline];
                    break;
                default: // Undefined modes -> standard
                    bmpOffset = offBmp[curline];
                    attOffset = offAtt[curline];
                    break;
            }
        } else
#endif
        {
            bmpOffset = offBmp[curline];
            attOffset = offAtt[curline];
        }

        #ifdef DIRTY_LINES
        // Force line draw (for testing)
        // dirty_lines[curline] = 1;
        #endif // DIRTY_LINES

#if !PICO_RP2040
        // DMA per-scanline attr shadow: use snapshot if DMA wrote attrs for this scanline
        if (Config::dma_mode && Z80DMA::dma_attr_valid[curline])
            dma_attr_override = &Z80DMA::dma_attr_shadow[curline * 32];
        else
            dma_attr_override = nullptr;
#endif

        Draw = linedraw_cnt >= 176 && linedraw_cnt <= 191 ? Draw_OSD169 : MainScreen;
        Draw_Opcode = MainScreen_Opcode;


        video_rest = CPU::tstates - tstateDraw;
        Draw(0,false);

    }

}

IRAM_ATTR void VIDEO::MainScreen_Blank_Opcode(bool contended) { MainScreen_Blank(4, contended); }

IRAM_ATTR void VIDEO::MainScreen_Blank_Snow(unsigned int statestoadd, bool contended) {

    CPU::tstates += statestoadd;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder();

        lineptr32 = (uint32_t *)(vga.frameBuffer[linedraw_cnt]) + lineptr_offset;
        prevLineptr16 = (uint16_t *)(vga.prevFrameBuffer[linedraw_cnt]) + lineptr_offset;

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
#if !PICO_RP2040
        if (Config::timex_video && VIDEO::timex_mode != 0) {
            switch (VIDEO::timex_mode) {
                case 1:
                    bmpOffset = 0x2000 + offBmp[curline];
                    attOffset = 0x2000 + offAtt[curline];
                    break;
                case 2:
                    bmpOffset = offBmp[curline];
                    attOffset = 0x2000 + offBmp[curline];
                    break;
                case 6:
                    bmpOffset = offBmp[curline];
                    attOffset = 0x2000 + offBmp[curline];
                    break;
                default:
                    bmpOffset = offBmp[curline];
                    attOffset = offAtt[curline];
                    break;
            }
        } else
#endif
        {
            bmpOffset = offBmp[curline];
            attOffset = offAtt[curline];
        }

        snowpage = MemESP::videoLatch ? 7 : 5;
        
        dispUpdCycle = 0; // For ULA cycle perfect emulation

        #ifdef DIRTY_LINES
        // Force line draw (for testing)
        // dirty_lines[curline] = 1;
        #endif // DIRTY_LINES

        Draw = &MainScreen_Snow;
        Draw_Opcode = &MainScreen_Snow_Opcode;

        // For ULA cycle perfect emulation
        int vid_rest = CPU::tstates - tstateDraw;
        if (vid_rest) {
            CPU::tstates = tstateDraw;
            Draw(vid_rest,false);
        }

    }

}    

IRAM_ATTR void VIDEO::MainScreen_Blank_Snow_Opcode(bool contended) {    
    
    CPU::tstates += 4;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder();

        lineptr32 = (uint32_t *)(vga.frameBuffer[linedraw_cnt]) + lineptr_offset;
        prevLineptr16 = (uint16_t *)(vga.prevFrameBuffer[linedraw_cnt]) + lineptr_offset;

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];

        snowpage = MemESP::videoLatch ? 7 : 5;
        
        dispUpdCycle = 0; // For ptime-128 compliant version

        #ifdef DIRTY_LINES
        // Force line draw (for testing)
        // dirty_lines[curline] = 1;
        #endif // DIRTY_LINES

        Draw = &MainScreen_Snow;
        Draw_Opcode = &MainScreen_Snow_Opcode;

        // For ULA cycle perfect emulation
        video_opcode_rest = CPU::tstates - tstateDraw;
        if (video_opcode_rest) {
            CPU::tstates = tstateDraw;
            Draw_Opcode(false);
            video_opcode_rest = 0;
        }

    }

}    

#ifndef DIRTY_LINES

#if !PICO_RP2040
// GigaScreen blend LUT: maps (prev_palette_idx, cur_palette_idx) → blended palette_idx
// Supports standard 16 Spectrum colors (indices 0-15)
// Blended colors stored in palette slots 17-239
static uint8_t gigsBlendLUT[256]; // indexed by (prev * 16 + cur)
static bool gigsBlendLUTReady = false;

void initGigascreenBlendLUT() {
    uint8_t nextSlot = 17; // start after ORANGE(16)

    // For each pair (i, j), compute average RGB and assign palette slot
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 16; j++) {
            if (i == j) {
                // Same color — no blend needed
                gigsBlendLUT[i * 16 + j] = i;
                continue;
            }
            // Check if reverse pair already computed
            if (j < i) {
                gigsBlendLUT[i * 16 + j] = gigsBlendLUT[j * 16 + i];
                continue;
            }
            // Compute average RGB888
            uint32_t c0 = spectrum_rgb888[i];
            uint32_t c1 = spectrum_rgb888[j];
            uint8_t R = (((c0 >> 16) & 0xFF) + ((c1 >> 16) & 0xFF)) / 2;
            uint8_t G = (((c0 >> 8) & 0xFF) + ((c1 >> 8) & 0xFF)) / 2;
            uint8_t B = ((c0 & 0xFF) + (c1 & 0xFF)) / 2;
            uint32_t blended = (R << 16) | (G << 8) | B;

            // Assign to next available palette slot
            if (nextSlot < 240) {
                gigsBlendLUT[i * 16 + j] = nextSlot;
                graphics_set_palette(nextSlot, paletteTransform(blended));
                nextSlot++;
            } else {
                gigsBlendLUT[i * 16 + j] = i; // fallback
            }
        }
    }
    gigsBlendLUTReady = true;
}

// Blend 4 packed palette-index pixels via LUT
// Uses shift/mask instead of byte pointers to avoid aliasing overhead on ARM
inline uint32_t blendPixels32(uint32_t cur, uint32_t prev) {
    if (cur == prev) return cur;
    return  gigsBlendLUT[(prev       & 0x0F) * 16 + (cur       & 0x0F)]
        | (gigsBlendLUT[((prev >> 8) & 0x0F) * 16 + ((cur >> 8) & 0x0F)] << 8)
        | (gigsBlendLUT[((prev >>16) & 0x0F) * 16 + ((cur >>16) & 0x0F)] << 16)
        | (gigsBlendLUT[((prev >>24) & 0x0F) * 16 + ((cur >>24) & 0x0F)] << 24);
}

// Pack lower 4 bits of each of 4 cur pixels (uint32) into 2-pixel-per-byte
// uint16 (low nibble = even pixel, high nibble = odd pixel)
inline uint16_t packPixels32(uint32_t cur) {
    uint32_t p0 = cur & 0x0F;
    uint32_t p1 = (cur >> 8) & 0x0F;
    uint32_t p2 = (cur >> 16) & 0x0F;
    uint32_t p3 = (cur >> 24) & 0x0F;
    return (uint16_t)(p0 | (p1 << 4) | (p2 << 8) | (p3 << 12));
}

// Blend 4 cur pixels with 4 prev pixels stored as packed uint16 (2 px per byte)
inline uint32_t blendPixels32_packed(uint32_t cur, uint16_t prev16) {
    uint32_t p0 = prev16 & 0x0F;
    uint32_t p1 = (prev16 >> 4) & 0x0F;
    uint32_t p2 = (prev16 >> 8) & 0x0F;
    uint32_t p3 = (prev16 >> 12) & 0x0F;
    return  gigsBlendLUT[p0 * 16 + (cur & 0x0F)]
        | (gigsBlendLUT[p1 * 16 + ((cur >> 8) & 0x0F)] << 8)
        | (gigsBlendLUT[p2 * 16 + ((cur >> 16) & 0x0F)] << 16)
        | (gigsBlendLUT[p3 * 16 + ((cur >> 24) & 0x0F)] << 24);
}
#else
// RP2040: gigascreen_enabled is always false, but compiler still needs the symbol
inline uint32_t blendPixels32(uint32_t cur, uint32_t) { return cur; }
inline uint16_t packPixels32(uint32_t) { return 0; }
inline uint32_t blendPixels32_packed(uint32_t cur, uint16_t) { return cur; }
#endif // !PICO_RP2040

// ----------------------------------------------------------------------------------
// Fast video emulation with no ULA cycle emulation and no snow effect support
// ----------------------------------------------------------------------------------
IRAM_ATTR void VIDEO::MainScreen(unsigned int statestoadd, bool contended) {

    if (contended) statestoadd += wait_st[CPU::tstates - tstateDraw];

    CPU::tstates += statestoadd;
    statestoadd += video_rest;
    video_rest = statestoadd & 0x03;
    unsigned int loopCount = statestoadd >> 2;
    coldraw_cnt += loopCount;

    if (coldraw_cnt >= 32) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank;
            Draw_Opcode = &Blank_Opcode;
        } else {
            Draw = &MainScreen_Blank;
            Draw_Opcode = &MainScreen_Blank_Opcode;
        }
        loopCount -= coldraw_cnt - 32;
    }

#if !PICO_RP2040
    if (Config::timex_video && VIDEO::timex_mode == 6) {
        // Hi-res mode 6 (512->256): real SCLD alternates byte-columns from
        // screen0 and screen1 at same address, 64 cols x 8 bits = 512 pixels.
        // For 256px output, OR-merge each pair (s0[addr] | s1[addr]) into
        // 8 output pixels — preserves all set bits, covers all 32 addresses.
        uint8_t hires_att = VIDEO::timex_hires_ink;
        for (; loopCount--; ) {
            uint8_t combined = grmem[bmpOffset++] | grmem[attOffset++];
            *lineptr32++ = AluByte[combined >> 4][hires_att];
            *lineptr32++ = AluByte[combined & 0xF][hires_att];
        }
    } else
#endif
    if (VIDEO::gigascreen_enabled) {
        for (; loopCount--; ) {
#if !PICO_RP2040
            uint8_t att = dma_attr_override ? dma_attr_override[attOffset & 0x1F] : grmem[attOffset];
            attOffset++;
#else
            uint8_t att = grmem[attOffset++];
#endif
            uint8_t bmp = grmem[bmpOffset++] ^ (-((att & flashing) >> 7));
            uint32_t newPixel1 = AluByte[bmp >> 4][att];
            uint32_t newPixel2 = AluByte[bmp & 0xF][att];

            uint32_t mix1 = blendPixels32_packed(newPixel1, prevLineptr16[0]);
            uint32_t mix2 = blendPixels32_packed(newPixel2, prevLineptr16[1]);
            prevLineptr16[0] = packPixels32(newPixel1);
            prevLineptr16[1] = packPixels32(newPixel2);
            prevLineptr16 += 2;
            *lineptr32++ = mix1;
            *lineptr32++ = mix2;
        }
    } else {
        for (; loopCount--; ) {
#if !PICO_RP2040
            uint8_t att = dma_attr_override ? dma_attr_override[attOffset & 0x1F] : grmem[attOffset];
            attOffset++;
#else
            uint8_t att = grmem[attOffset++];
#endif
            uint8_t bmp = grmem[bmpOffset++] ^ (-((att & flashing) >> 7));
            *lineptr32++ = AluByte[bmp >> 4][att];
            *lineptr32++ = AluByte[bmp & 0xF][att];
        }
    }
}

IRAM_ATTR void VIDEO::MainScreen_OSD(unsigned int statestoadd, bool contended) {    

    if (contended) statestoadd += wait_st[CPU::tstates - tstateDraw];

    CPU::tstates += statestoadd;
    statestoadd += video_rest;
    video_rest = statestoadd & 0x03;
    unsigned int loopCount = statestoadd >> 2;
    unsigned int coldraw_osd = coldraw_cnt;
    
    coldraw_cnt += loopCount;

    if (coldraw_cnt >= 32) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank;
            Draw_Opcode = &Blank_Opcode;
        } else {
            Draw = &MainScreen_Blank;
            Draw_Opcode = &MainScreen_Blank_Opcode;
        }
        loopCount -= coldraw_cnt - 32;
    }

    for (;loopCount--;) {
        lineptr32+=2;
        attOffset++;
        bmpOffset++;
    }
}

IRAM_ATTR void VIDEO::MainScreen_Opcode(bool contended) { Draw(4,contended); }

// ----------------------------------------------------------------------------------
// ULA cycle perfect emulation with snow effect support
// ----------------------------------------------------------------------------------
IRAM_ATTR void VIDEO::MainScreen_Snow(unsigned int statestoadd, bool contended) {

    bool do_stats = false;

    if (contended) statestoadd += wait_st[coldraw_cnt]; // [CPU::tstates - tstateDraw];

    CPU::tstates += statestoadd;
    
    unsigned int col_osd = coldraw_cnt >> 2;
    if (linedraw_cnt >= 176 && linedraw_cnt <= 191) do_stats = (VIDEO::Draw_OSD169 == VIDEO::MainScreen_OSD);
    
    coldraw_cnt += statestoadd;

    if (coldraw_cnt >= 128) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw = &Blank_Snow;
            Draw_Opcode = &Blank_Snow_Opcode;
        } else {
            Draw = &MainScreen_Blank_Snow;
            Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
        }
        statestoadd -= coldraw_cnt - 128;  
    }

    for (;statestoadd--;) {

        switch(dispUpdCycle) {
            
            // In Weiv's Spectramine cycle starts in 2 and half black strip shows at 14349 in ptime-128.tap (early timings).
            // In SpecEmu cycle starts in 3, black strip at 14350. Will use Weiv's data for now.
            case 2:
                bmp1 = grmem[bmpOffset++];
                lastbmp = bmp1;
                break;
            case 3:
                if (snow_att) {
                    att1 = MemESP::ram[snowpage].direct()[(attOffset++ & 0xff80) | snowR];  // get attribute byte
                    snow_att = false;
                } else
                    att1 = grmem[attOffset++];  // get attribute byte                

                lastatt = att1;

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {                    
                    lineptr32 += 2;
                } else {
                    if (att1 & flashing) bmp1 = ~bmp1;
                    *lineptr32++ = AluByte[bmp1 >> 4][att1];
                    *lineptr32++ = AluByte[bmp1 & 0xF][att1];
                }

                col_osd++;

                break;
            case 4:
                bmp2 = grmem[bmpOffset++];
                break;
            case 5:
                if (dbl_att) {
                    att2 = lastatt;
                    attOffset++;
                    dbl_att = false;
                } else
                    att2 = grmem[attOffset++];  // get attribute byte

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att2 & flashing) bmp2 = ~bmp2;
                    *lineptr32++ = AluByte[bmp2 >> 4][att2];
                    *lineptr32++ = AluByte[bmp2 & 0xF][att2];

                }

                col_osd++;

        }

        ++dispUpdCycle &= 0x07; // Update the cycle counter.

    }

}

// ----------------------------------------------------------------------------------
// ULA cycle perfect emulation with snow effect support
// ----------------------------------------------------------------------------------
IRAM_ATTR void VIDEO::MainScreen_Snow_Opcode(bool contended) {

    int snow_effect = 0;
    unsigned int addr;
    bool do_stats = false;

    unsigned int statestoadd = video_opcode_rest ? video_opcode_rest : 4;

    if (contended) statestoadd += wait_st[coldraw_cnt]; // [CPU::tstates - tstateDraw];

    CPU::tstates += statestoadd;

    unsigned int col_osd = coldraw_cnt >> 2;
    if (linedraw_cnt >= 176 && linedraw_cnt <= 191) do_stats = (VIDEO::Draw_OSD169 == VIDEO::MainScreen_OSD);
    
    coldraw_cnt += statestoadd;

    if (coldraw_cnt >= 128) {
        tstateDraw += tStatesPerLine;
        if (++linedraw_cnt == lin_end2) {
            Draw =&Blank_Snow;
            Draw_Opcode = &Blank_Snow_Opcode;
        } else {
            Draw = &MainScreen_Blank_Snow;
            Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
        }

        statestoadd -= coldraw_cnt - 128;

    }

    if (dispUpdCycle == 6) {
        dispUpdCycle = 2;
        return;
    }

    // Determine if snow effect can be applied
    uint8_t page = Z80::getRegI() & 0xc0;
    if (page == 0x40) { // Snow 48K, 128K
        snow_effect = 1;
        snowpage = MemESP::videoLatch ? 7 : 5;
    } else if (Z80Ops::is128 && (MemESP::bankLatch & 0x01) && page == 0xc0) {  // Snow 128K
        snow_effect = 1;
        if (MemESP::bankLatch == 1 || MemESP::bankLatch == 3)
            snowpage = MemESP::videoLatch ? 3 : 1;
        else
            snowpage = MemESP::videoLatch ? 7 : 5;
    }

    for (;statestoadd--;) {

        switch(dispUpdCycle) {
            
            // In Weiv's Spectramine cycle starts in 2 and half black strip shows at 14349 in ptime-128.tap (early timings).
            // In SpecEmu cycle starts in 3, black strip at 14350. Will use Weiv's data for now.
            
            case 2:

                if (snow_effect && statestoadd == 0) {
                    snowR = Z80::getRegR() & 0x7f;
                    bmp1 = MemESP::ram[snowpage].direct()[(bmpOffset++ & 0xff80) | snowR];
                    snow_att = true;
                } else
                    bmp1 = grmem[bmpOffset++];

                lastbmp = bmp1;

                break;

            case 3:

                if (snow_att) {
                    att1 = MemESP::ram[snowpage].direct()[(attOffset++ & 0xff80) | snowR];  // get attribute byte
                    snow_att = false;
                } else
                    att1 = grmem[attOffset++];  // get attribute byte                

                lastatt = att1;

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att1 & flashing) bmp1 = ~bmp1;
                    *lineptr32++ = AluByte[bmp1 >> 4][att1];
                    *lineptr32++ = AluByte[bmp1 & 0xF][att1];
                }

                col_osd++;

                break;

            case 4:

                if (snow_effect && statestoadd == 0) {
                    bmp2 = lastbmp;
                    bmpOffset++;
                    dbl_att = true;
                } else
                    bmp2 = grmem[bmpOffset++];

                break;

            case 5:

                if (dbl_att) {
                    att2 = lastatt;
                    attOffset++;
                    dbl_att = false;
                } else
                    att2 = grmem[attOffset++];  // get attribute byte

                if (do_stats && (col_osd >= 13 && col_osd <= 30)) {
                    lineptr32 += 2;
                } else {
                    if (att2 & flashing) bmp2 = ~bmp2;
                    *lineptr32++ = AluByte[bmp2 >> 4][att2];
                    *lineptr32++ = AluByte[bmp2 & 0xF][att2];
                }

                col_osd++;

        }

        ++dispUpdCycle &= 0x07; // Update the cycle counter.

    }

}

#else

// IRAM_ATTR void VIDEO::MainScreen(unsigned int statestoadd, bool contended) {    

//     if (contended) statestoadd += wait_st[CPU::tstates - tstateDraw];

//     CPU::tstates += statestoadd;
//     statestoadd += video_rest;
//     video_rest = statestoadd & 0x03;
//     unsigned int loopCount = statestoadd >> 2;
//     coldraw_cnt += loopCount;

//     if (coldraw_cnt >= 32) {
//         tstateDraw += tStatesPerLine;
//         Draw = ++linedraw_cnt == lin_end2 ? &Blank : &MainScreen_Blank;
//         if (dirty_lines[curline]) {
//             loopCount -= coldraw_cnt - 32;
//             for (;loopCount--;) {
//                 uint8_t att = grmem[attOffset++];
//                 uint8_t bmp = att & flashing ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
//                 *lineptr32++ = AluByte[bmp >> 4][att];
//                 *lineptr32++ = AluByte[bmp & 0xF][att];
//             }
//             dirty_lines[curline] &= 0x80;
//         }
//         return;
//     }

//     if (dirty_lines[curline]) {
//         for (;loopCount--;) {
//             uint8_t att = grmem[attOffset++];
//             uint8_t bmp = att & flashing ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
//             *lineptr32++ = AluByte[bmp >> 4][att];
//             *lineptr32++ = AluByte[bmp & 0xF][att];
//         }
//     } else {
//         attOffset += loopCount;
//         bmpOffset += loopCount;
//         lineptr32 += loopCount << 1;
//     }

// }

#endif

IRAM_ATTR void VIDEO::Blank(unsigned int statestoadd, bool contended) { CPU::tstates += statestoadd; }
IRAM_ATTR void VIDEO::Blank_Opcode(bool contended) { CPU::tstates += 4; }
IRAM_ATTR void VIDEO::Blank_Snow(unsigned int statestoadd, bool contended) { CPU::tstates += statestoadd; }
IRAM_ATTR void VIDEO::Blank_Snow_Opcode(bool contended) { CPU::tstates += 4; }

IRAM_ATTR void VIDEO::EndFrame() {

    linedraw_cnt = lin_end;

    tstateDraw = tStatesScreen;

#if !PICO_RP2040
    // Clear DMA attr shadow and charrow write counters for next frame
    if (Config::dma_mode)
        Z80DMA::resetAttrShadow();
    dma_attr_override = nullptr;

    // Flush deferred ULA+ palette updates so HDMI DMA sees consistent palette
    // for the entire next frame (prevents top-of-screen palette tearing)
    if (ulaplus_enabled)
        ulaPlusFlushPalette();
#endif

    static uint8_t skipCnt = 0;
    static bool wasMaxSpeed = false;
    bool skipFrame = ESPectrum::maxSpeed && (++skipCnt & 63);
    if (!ESPectrum::maxSpeed && wasMaxSpeed) {
        // Exiting maxSpeed: fill entire framebuffer border with current color
        uint8_t border = brd & 0xFF;
        for (int y = 0; y < (int)vga.yres; y++)
            memset(vga.frameBuffer[y], border, vga.xres);
    }
    wasMaxSpeed = ESPectrum::maxSpeed;
    if (skipFrame) {
        // Skip rendering: 1/1024 frames during tape loading, 1/256 otherwise
        Draw = VIDEO::snow_toggle ? &Blank_Snow : &Blank;
        Draw_Opcode = VIDEO::snow_toggle ? &Blank_Snow_Opcode : &Blank_Opcode;
    } else if (VIDEO::snow_toggle
#if !PICO_RP2040
        && !(Config::timex_video && VIDEO::timex_mode != 0)
#endif
    ) {
        Draw = &MainScreen_Blank_Snow;
        Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
    } else {
        Draw = &MainScreen_Blank;
        Draw_Opcode = &MainScreen_Blank_Opcode;
    }

    if (!skipFrame) {
        if (brdChange || brdGigascreenChange) {
            DrawBorder();
            brdnextframe = true;
            brdGigascreenChange = false;
        } else {
            if (brdnextframe) {
                DrawBorder();
                brdnextframe = false;
            }
        }
    } else {
        brdGigascreenChange = false;
    }

    // Restart border drawing (single TopBorder_Blank for all models)
    if (skipFrame)
        DrawBorder = &Border_Blank;
    else
        DrawBorder = &TopBorder_Blank;
    lastBrdTstate = tStatesBorder;
    brdChange = false;

#if !PICO_RP2040
    if (Config::gigascreen_onoff == 2) { // Auto mode
        if (gigascreen_auto_countdown > 0) {
            gigascreen_auto_countdown--;
            if (!gigascreen_enabled) {
                if (!gigsBlendLUTReady) initGigascreenBlendLUT();
                // Seed prev from current FB so first blended frame matches current
                // (otherwise stale prev from previous session causes a flash)
                InitPrevBuffer();
                gigascreen_enabled = true;
            }
        } else {
            if (gigascreen_enabled) gigascreen_enabled = false;
        }
    }
#endif

    framecnt++;
}

//----------------------------------------------------------------------------------------------------------------
// Border Drawing
//----------------------------------------------------------------------------------------------------------------

// IRAM_ATTR void VIDEO::DrawBorderFast() {

//     uint8_t border = zxColor(borderColor,0);

//     int i = 0;

//     // Top border
//     for (; i < lin_end; i++) memset((uint32_t *)(vga.frameBuffer[i]),border, vga.xres);

//     // Paper border
//     int brdsize = (vga.xres - SPEC_W) >> 1;
//     for (; i < lin_end2; i++) {
//         memset((uint32_t *)(vga.frameBuffer[i]), border, brdsize);
//         memset((uint32_t *)(vga.frameBuffer[i] + vga.xres - brdsize), border, brdsize);
//     }

//     // Bottom border
//     for (; i < OSD::scrH; i++) memset((uint32_t *)(vga.frameBuffer[i]),border, vga.xres);

// }

IRAM_ATTR void VIDEO::Border_Blank() {

}

//----------------------------------------------------------------------------------------------------------------
// Specialized Update_Border variants — function pointer avoids per-call branching
// 48K/128K (brdPairWrite=true, step=4): writes 2 uint32_t (8px) per step via brdptr16 cast
// Pentagon (brdPairWrite=false, step=1): writes 1 uint16_t (2px) per step via XOR indexing
//----------------------------------------------------------------------------------------------------------------

static void (*Update_Border)();

// 48K/128K: write 2 uint32_t (8px) at brdptr16[brdcol_cnt] (step=4, brdcol_cnt aligned to 4)
IRAM_ATTR static void Update_Border_Pair() {
    uint32_t color32 = VIDEO::brd | (VIDEO::brd << 16);
    ((uint32_t *)&brdptr16[brdcol_cnt])[0] = color32;
    ((uint32_t *)&brdptr16[brdcol_cnt])[1] = color32;
}

IRAM_ATTR static void Update_Border_Pair_Gig() {
    uint32_t color32 = VIDEO::brd | (VIDEO::brd << 16);
    // Packed border prev: brdcol_cnt is in uint16-units; in packed (1 byte = 2 px)
    // 8 px occupy 4 bytes → uint32 covers 8 px. Same nibble repeated → 0x11*nib.
    uint8_t nib = VIDEO::brd & 0x0F;
    uint32_t packed32 = nib * 0x11111111u;
    uint32_t* prevWord = (uint32_t*)&prevBrdptr8[brdcol_cnt];
    uint32_t old_packed = prevWord[0];
    prevWord[0] = packed32;
    if (old_packed != packed32) {
        // Decompose old packed back to full uint32 per pair (low+high nibble identical
        // for border since prev was filled with uniform color)
        uint32_t old_lo_nib = old_packed & 0x0F; // single nibble suffices
        uint32_t old32 = old_lo_nib * 0x01010101u;
        uint32_t mixed = blendPixels32(color32, old32);
        ((uint32_t *)&brdptr16[brdcol_cnt])[0] = mixed;
        ((uint32_t *)&brdptr16[brdcol_cnt])[1] = mixed;
        VIDEO::brdGigascreenChange = true;
    } else {
        ((uint32_t *)&brdptr16[brdcol_cnt])[0] = color32;
        ((uint32_t *)&brdptr16[brdcol_cnt])[1] = color32;
    }
}

// Pentagon: write 1 uint16_t (2px) at brdptr16[brdcol_cnt ^ 1] (step=1)
IRAM_ATTR static void Update_Border_XOR() {
    brdptr16[brdcol_cnt ^ 1] = VIDEO::brd;
}

IRAM_ATTR static void Update_Border_XOR_Gig() {
    uint32_t newColor = VIDEO::brd; // 0xCCCCCCCC: 4 copies of palette index byte
    int idx = brdcol_cnt ^ 1;
    uint8_t newNib = newColor & 0x0F;
    uint8_t newPacked = newNib | (newNib << 4);
    uint8_t oldPacked = prevBrdptr8[idx];
    if (oldPacked != newPacked) {
        prevBrdptr8[idx] = newPacked;
        uint8_t oldNib = oldPacked & 0x0F;
        uint32_t oldColor = oldNib * 0x01010101u;
        brdptr16[idx] = blendPixels32(newColor, oldColor);
        VIDEO::brdGigascreenChange = true;
    } else {
        brdptr16[idx] = newColor;
    }
}

static void Select_Update_Border() {
    if (brdPairWrite)
        Update_Border = VIDEO::gigascreen_enabled ? &Update_Border_Pair_Gig : &Update_Border_Pair;
    else
        Update_Border = VIDEO::gigascreen_enabled ? &Update_Border_XOR_Gig : &Update_Border_XOR;
}

//----------------------------------------------------------------------------------------------------------------
// Unified border functions (all models: 48K, 128K, Pentagon; all resolutions)
// Uses brdcol_step/brdcol_end/brdcol_end1/brdcol_start/brdcol_retrace variables
//----------------------------------------------------------------------------------------------------------------

IRAM_ATTR void VIDEO::TopBorder_Blank() {
    if (CPU::tstates >= tStatesBorder) {
        static bool brd_logged = false;
        if (!brd_logged) {
            brd_logged = true;
            Debug::log("BRD: yres=%d step=%d end=%d end1=%d ret=%d lin_end=%d/%d start=%d isFB=%d is169=%d tsBrd=%d tsLine=%d fb=%p",
                (int)vga.yres, brdcol_step, brdcol_end, brdcol_end1, brdcol_retrace,
                lin_end, lin_end2, brdcol_start, isFullBorder, is169,
                tStatesBorder, tStatesPerLine, vga.frameBuffer);
        }
        Select_Update_Border();
        brdcol_cnt = brdcol_start;
        brdlin_cnt = 0;
        brdptr16 = (uint16_t *)(vga.frameBuffer[0]);
        prevBrdptr8 = vga.prevFrameBuffer ? (uint8_t *)(vga.prevFrameBuffer[0]) : (uint8_t *)brdptr16;
        DrawBorder = &TopBorder;
        DrawBorder();
    }
}

IRAM_ATTR void VIDEO::TopBorder() {
    while (lastBrdTstate <= CPU::tstates) {
        if (brdcol_cnt < brdcol_retrace) {
            Update_Border();
        } else if (brdcol_retrace < brdcol_end) {
            int lastPair = (brdcol_retrace - 1) & ~1;
            int curPair = brdcol_cnt & ~1;
            ((uint32_t *)&brdptr16[curPair])[0] = ((uint32_t *)&brdptr16[lastPair])[0];
            if (gigascreen_enabled) ((uint16_t *)&prevBrdptr8[curPair])[0] = ((uint16_t *)&prevBrdptr8[lastPair])[0];
        }

        lastBrdTstate += brdcol_step;
        brdcol_cnt += brdcol_step;

        if (brdcol_cnt >= brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr8 = vga.prevFrameBuffer ? (uint8_t *)(vga.prevFrameBuffer[brdlin_cnt]) : (uint8_t *)brdptr16;
            brdcol_cnt = brdcol_start;
            lastBrdTstate += tStatesPerLine - brdcol_end;

            if (brdlin_cnt == lin_end) {
                DrawBorder = &MiddleBorder;
                MiddleBorder();
                return;
            }
        }
    }
}

IRAM_ATTR void VIDEO::MiddleBorder() {
    while (lastBrdTstate <= CPU::tstates) {
        if (brdcol_cnt < brdcol_retrace) {
            Update_Border();
        } else if (brdcol_retrace < brdcol_end) {
            int lastPair = (brdcol_retrace - 1) & ~1;
            int curPair = brdcol_cnt & ~1;
            ((uint32_t *)&brdptr16[curPair])[0] = ((uint32_t *)&brdptr16[lastPair])[0];
            if (gigascreen_enabled) ((uint16_t *)&prevBrdptr8[curPair])[0] = ((uint16_t *)&prevBrdptr8[lastPair])[0];
        }

        lastBrdTstate += brdcol_step;
        brdcol_cnt += brdcol_step;

        if (brdcol_cnt == brdcol_end1) {
            lastBrdTstate += 128;
            brdcol_cnt = brdcol_end1 + 128;
        } else if (brdcol_cnt >= brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr8 = vga.prevFrameBuffer ? (uint8_t *)(vga.prevFrameBuffer[brdlin_cnt]) : (uint8_t *)brdptr16;
            brdcol_cnt = brdcol_start;
            lastBrdTstate += tStatesPerLine - brdcol_end;
            if (brdlin_cnt == lin_end2) {
                DrawBorder = Draw_OSD43;
                DrawBorder();
                return;
            }
        }
    }
}

IRAM_ATTR void VIDEO::BottomBorder() {
    while (lastBrdTstate <= CPU::tstates) {
        if (brdcol_cnt < brdcol_retrace) {
            Update_Border();
        } else if (brdcol_retrace < brdcol_end) {
            int lastPair = (brdcol_retrace - 1) & ~1;
            int curPair = brdcol_cnt & ~1;
            ((uint32_t *)&brdptr16[curPair])[0] = ((uint32_t *)&brdptr16[lastPair])[0];
            if (gigascreen_enabled) ((uint16_t *)&prevBrdptr8[curPair])[0] = ((uint16_t *)&prevBrdptr8[lastPair])[0];
        }

        lastBrdTstate += brdcol_step;
        brdcol_cnt += brdcol_step;

        if (brdcol_cnt >= brdcol_end) {
            brdlin_cnt++;
            brdcol_cnt = brdcol_start;
            lastBrdTstate += tStatesPerLine - brdcol_end;
            if (brdlin_cnt == (int)vga.yres) {
                DrawBorder = &Border_Blank;
                return;
            }
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr8 = vga.prevFrameBuffer ? (uint8_t *)(vga.prevFrameBuffer[brdlin_cnt]) : (uint8_t *)brdptr16;
        }
    }
}

IRAM_ATTR void VIDEO::BottomBorder_OSD() {
    const bool isFB = VIDEO::isFullBorder288() || VIDEO::isFullBorder240();
    const int osd_y_start = VIDEO::isFullBorder288() ? 268 : 220;
    const int osd_y_end = osd_y_start + 15;
    // OSD x coords in uint16_t units, aligned down/up to brdcol_step for step=4
    const int osd_x_start = isFB ? (94 & ~(brdcol_step - 1)) : (84 & ~(brdcol_step - 1));
    const int osd_x_end = isFB ? ((166 + brdcol_step - 1) & ~(brdcol_step - 1)) : ((156 + brdcol_step - 1) & ~(brdcol_step - 1));
    while (lastBrdTstate <= CPU::tstates) {
        if (brdcol_cnt < brdcol_retrace) {
            if (brdlin_cnt < osd_y_start || brdlin_cnt > osd_y_end) {
                Update_Border();
            } else if (brdcol_cnt < osd_x_start || brdcol_cnt >= osd_x_end) {
                Update_Border();
            }
        } else if (brdcol_retrace < brdcol_end) {
            int lastPair = (brdcol_retrace - 1) & ~1;
            int curPair = brdcol_cnt & ~1;
            ((uint32_t *)&brdptr16[curPair])[0] = ((uint32_t *)&brdptr16[lastPair])[0];
            if (gigascreen_enabled) ((uint16_t *)&prevBrdptr8[curPair])[0] = ((uint16_t *)&prevBrdptr8[lastPair])[0];
        }

        lastBrdTstate += brdcol_step;
        brdcol_cnt += brdcol_step;

        if (brdcol_cnt >= brdcol_end) {
            brdlin_cnt++;
            brdcol_cnt = brdcol_start;
            lastBrdTstate += tStatesPerLine - brdcol_end;
            if (brdlin_cnt == (int)vga.yres) {
                DrawBorder = &Border_Blank;
                return;
            }
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr8 = vga.prevFrameBuffer ? (uint8_t *)(vga.prevFrameBuffer[brdlin_cnt]) : (uint8_t *)brdptr16;
        }
    }
}

// SaveRect starts in PSRAM past the region reserved for MemESP pages. Dynamic so
// it moves up if MEM_PG_CNT is raised. +64 KB gap as a safety margin.
static inline size_t saveRectShift() {
    size_t memesp_end = ((size_t)MEM_PG_CNT + 2) * MEM_PG_SZ + (64ul << 10);
    return memesp_end < (2ul << 20) ? (2ul << 20) : memesp_end;
}
// Reserve up to this much SaveRect space; abort save if offset exceeds it.
#define SAVE_RECT_PSRAM_MAX (256ul << 10)

void SaveRectT::save(int16_t x, int16_t y, int16_t w, int16_t h) {
    if (offsets.empty()) {
        offsets.push_back(0);
    }
    x -= 2; if (x < 0) x = 0; // W/A
    w += 4; // W/A
    size_t off = offsets.back();
    size_t shift = saveRectShift();
    // Without SD, prefer SPI PSRAM over heap/RAM — std::vector on tight heap panics
    // (check_alloc → panic) when fragmentation beats mallinfo()'s fordblks view.
    if (!FileUtils::fsMount && psram_size() >= shift + SAVE_RECT_PSRAM_MAX) {
        size_t need = 8 + (size_t)w * h;
        if (off + need > SAVE_RECT_PSRAM_MAX) {
            offsets.push_back(off); // out of room — dummy, restore will redraw
            return;
        }
        size_t pos = off + shift;
        write16psram(pos, (uint16_t)x); pos += 2;
        write16psram(pos, (uint16_t)y); pos += 2;
        write16psram(pos, (uint16_t)w); pos += 2;
        write16psram(pos, (uint16_t)h); pos += 2;
        for (size_t line = y; line < (size_t)y + h; ++line) {
            writepsram(pos, VIDEO::vga.frameBuffer[line] + x, w);
            pos += w;
        }
        offsets.push_back(off + need);
        return;
    }
    if (FileUtils::fsMount) {
        FIL f;
        f_open(&f, "/tmp/save_rect.tmp", FA_WRITE | FA_OPEN_ALWAYS);
        f_lseek(&f, off);
        UINT bw;
        f_write(&f, &x, 2, &bw);
        f_write(&f, &y, 2, &bw);
        f_write(&f, &w, 2, &bw);
        f_write(&f, &h, 2, &bw);
        off += 8;
        for (size_t line = y; line < y + h; ++line) {
            uint8_t *backbuffer = VIDEO::vga.frameBuffer[line];
            f_write(&f, backbuffer + x, w, &bw);
            off += w;
        }
        f_close(&f);
        offsets.push_back(off);
    } else {
        // RAM fallback when no SD card — skip if not enough contiguous heap.
        // SDK's malloc wraps with check_alloc → panic on OOM, so we must reject
        // before the allocation. getContiguousHeap() (sbrk headroom) is the only
        // size we can trust to succeed; fordblks may be fragmented.
        size_t need = 8 + (size_t)w * h;
        size_t new_size = off + need;
        size_t cur_cap = ram_buf.capacity();
        if (new_size > cur_cap) {
            // Need to allocate a new buffer of at least new_size, while the old
            // cur_cap buffer is still held. Peak contiguous need = new_size.
            if (getContiguousHeap() < new_size + 1024) {
                offsets.push_back(off); // dummy — restore will redraw
                return;
            }
            std::vector<uint8_t> tmp;
            tmp.reserve(new_size);
            tmp.resize(off);
            if (off) memcpy(tmp.data(), ram_buf.data(), off);
            ram_buf.swap(tmp);
        }
        ram_buf.resize(new_size);
        uint8_t *p = ram_buf.data() + off;
        memcpy(p, &x, 2); p += 2;
        memcpy(p, &y, 2); p += 2;
        memcpy(p, &w, 2); p += 2;
        memcpy(p, &h, 2); p += 2;
        for (size_t line = y; line < y + h; ++line) {
            memcpy(p, VIDEO::vga.frameBuffer[line] + x, w);
            p += w;
        }
        offsets.push_back(new_size);
    }
}
void SaveRectT::restore_last() {
    if (offsets.size() <= 1) return; // nothing saved to restore
    size_t saved_end = offsets.back();
    offsets.pop_back();
    size_t off = offsets.back();
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    size_t shift = saveRectShift();
    if (!FileUtils::fsMount && psram_size() >= shift + SAVE_RECT_PSRAM_MAX) {
        if (saved_end == off) {
            if (offsets.empty()) offsets.push_back(0);
            return; // dummy save — nothing to restore
        }
        size_t pos = off + shift;
        x = read16psram(pos); pos += 2;
        y = read16psram(pos); pos += 2;
        w = read16psram(pos); pos += 2;
        h = read16psram(pos); pos += 2;
        if (!w || !h) return;
        for (size_t line = y; line < (size_t)y + h; ++line) {
            readpsram(VIDEO::vga.frameBuffer[line] + x, pos, w);
            pos += w;
        }
        if (offsets.empty()) offsets.push_back(0);
        return;
    }
    if (FileUtils::fsMount) {
        FIL f;
        f_open(&f, "/tmp/save_rect.tmp", FA_READ);
        f_lseek(&f, off);
        UINT br;
        f_read(&f, &x, 2, &br);
        f_read(&f, &y, 2, &br);
        f_read(&f, &w, 2, &br);
        f_read(&f, &h, 2, &br);
        if (!w || !h) {
            f_close(&f);
            return;
        }
        for (size_t line = y; line < y + h; ++line) {
            f_read(&f, VIDEO::vga.frameBuffer[line] + x, w, &br);
        }
        f_close(&f);
    } else if (off < ram_buf.size()) {
        // RAM fallback when no SD card
        uint8_t *p = ram_buf.data() + off;
        memcpy(&x, p, 2); p += 2;
        memcpy(&y, p, 2); p += 2;
        memcpy(&w, p, 2); p += 2;
        memcpy(&h, p, 2); p += 2;
        if (!w || !h) return;
        for (size_t line = y; line < y + h; ++line) {
            memcpy(VIDEO::vga.frameBuffer[line] + x, p, w);
            p += w;
        }
        ram_buf.resize(off); // shrink
    }
    if (offsets.empty()) {
        offsets.push_back(0);
    }
}

void SaveRectT::store_ram(const void* p, size_t sz) {
    if (offsets.empty()) {
        offsets.push_back(0);
    }
    size_t off = offsets.back();
    UINT bw;
    FIL f;
    f_open(&f, "/tmp/save_rect.tmp", FA_WRITE | FA_OPEN_ALWAYS);
    f_lseek(&f, off);
    f_write(&f, p, sz, &bw);
    f_close(&f);
}

void SaveRectT::restore_ram(void* p, size_t sz) {
    if (offsets.empty()) return;
    offsets.pop_back();
    size_t off = offsets.back();
    UINT br;
    FIL f;
    f_open(&f, "/tmp/save_rect.tmp", FA_READ);
    f_lseek(&f, off);
    f_read(&f, p, sz, &br);
    f_close(&f);
    if (offsets.empty()) {
        offsets.push_back(0);
    }
}
