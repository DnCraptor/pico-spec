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

extern "C" void graphics_set_palette(uint8_t i, uint32_t color888);
extern "C" void vga_set_palette_entry_solid(uint8_t i, uint32_t color888);

#pragma GCC optimize("O3")

VGA6Bit VIDEO::vga;

extern "C" uint8_t* getLineBuffer(int line) {
    if (!VIDEO::vga.frameBuffer) return 0;
    return (uint8_t*)VIDEO::vga.frameBuffer[line];
}

extern "C" void ESPectrum_vsync() {
    ESPectrum::v_sync = true;
}

extern "C" int get_video_mode() {
    return VIDEO::video_mode;
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

// ULA+
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
unsigned int VIDEO::AluBytesUlaPlus[16][256] = {};

#ifdef DIRTY_LINES
uint8_t VIDEO::dirty_lines[SPEC_H];
// uint8_t VIDEO::linecalc[SPEC_H];
#endif //  DIRTY_LINES
static unsigned int is169;

static uint32_t* lineptr32;
static uint32_t* prevLineptr32;

static unsigned int tstateDraw; // Drawing start point (in Tstates)
static unsigned int linedraw_cnt;
static unsigned int lin_end, lin_end2 /*, lin_end3*/;
static unsigned int coldraw_cnt;
static unsigned int col_end;
static unsigned int video_rest;
static unsigned int video_opcode_rest;
static unsigned int curline;

static unsigned int bmpOffset;  // offset for bitmap in graphic memory
static unsigned int attOffset;  // offset for attrib in graphic memory

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

IRAM_ATTR void VGA6Bit::interrupt(void *arg) {
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


static uint32_t* brdptr32;
static uint32_t* prevBrdptr32;
static uint16_t* brdptr16;
static uint16_t* prevBrdptr16;

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

// Standard Spectrum RGB888 palette
static const uint32_t spectrum_rgb888[16] = {
    0x000000, 0x0000CD, 0xCD0000, 0xCD00CD,  // black, blue, red, magenta
    0x00CD00, 0x00CDCD, 0xCDCD00, 0xCDCDCD,  // green, cyan, yellow, white
    0x000000, 0x0000FF, 0xFF0000, 0xFF00FF,  // bright variants
    0x00FF00, 0x00FFFF, 0xFFFF00, 0xFFFFFF,
};

void initGigascreenBlendLUT();

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

void VIDEO::regenerateUlaPlusAluBytes() {
    // Set palette colors for all 64 ULA+ entries
    for (int i = 0; i < 64; i++)
        graphics_set_palette(i, grb_to_rgb888(ulaplus_palette[i]));

    // Build AluBytes with fixed indices 0-63 (only depends on attribute format,
    // not palette contents — only needs to be done once at ULA+ enable)
    for (int nibble = 0; nibble < 16; nibble++) {
        for (int att = 0; att < 256; att++) {
            uint8_t group = (att >> 6) & 0x03;
            uint8_t ink_idx   = group * 16 + (att & 0x07);
            uint8_t paper_idx = group * 16 + ((att >> 3) & 0x07) + 8;

            uint8_t px[2] = { paper_idx, ink_idx };
            AluBytesUlaPlus[nibble][att] =
                px[(nibble >> 1) & 1]        |
                (px[nibble & 1]       << 8)  |
                (px[(nibble >> 3) & 1] << 16)|
                (px[(nibble >> 2) & 1] << 24);
        }
    }
    for (int n = 0; n < 16; n++)
        AluByte[n] = AluBytesUlaPlus[n];
}

// Fast path: update single palette entry without rebuilding AluBytes
void VIDEO::ulaPlusUpdatePaletteEntry(uint8_t entry) {
    graphics_set_palette(entry, grb_to_rgb888(ulaplus_palette[entry]));
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
    flashing = 0;
    // Restore palette: indices 0-63 back to G3R3B2 defaults, then 0-15 to Spectrum (solid)
    for (int i = 0; i < 64; i++)
        graphics_set_palette(i, grb_to_rgb888(i));
    for (int i = 0; i < 16; i++) {
        graphics_set_palette(i, spectrum_rgb888[i]);
        vga_set_palette_entry_solid(i, spectrum_rgb888[i]);
    }

    // Restore GigaScreen blend palette (slots 17-136) if GigaScreen is configured,
    // since palette entries 17-63 were overwritten by ULA+ above
    if (Config::gigascreen_enabled)
        initGigascreenBlendLUT();

    for (int n = 0; n < 16; n++)
        AluByte[n] = AluBytesStd[n];
    brd = border32[borderColor];
    brdChange = true;
}

const int redPins[] = {RED_PINS_6B};
const int grePins[] = {GRE_PINS_6B};
const int bluPins[] = {BLU_PINS_6B};

void VIDEO::vgataskinit(void *unused) {
    uint8_t Mode;
    Mode = 16 + ((Config::arch == "48K") ? 0 : (Config::arch == "128K" || Config::arch == "ALF" ? 2 : 4)) + (Config::aspect_16_9 ? 1 : 0);
    OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
    OSD::scrH = vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv];
    vga.VGA6Bit_useinterrupt = true; // ????
    // Init mode
    vga.init(Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);    
    for (;;){}    
}

///TaskHandle_t VIDEO::videoTaskHandle;
static __aligned(4) uint8_t SAVE_RECT[0x9000] = {0};

void VIDEO::Init() {
    int Mode = Config::aspect_16_9 ? 2 : 0;
    Mode += Config::scanlines;
    OSD::scrW = vidmodes[Mode][vmodeproperties::hRes];
    OSD::scrH = (vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv]) >> Config::scanlines;
    vga.VGA6Bit_useinterrupt = false;
    vga.init( Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);

    // Initialize full 256-entry palette: G3R3B2 → RGB888 for all possible values
    // This covers ULA+ (any G3R3B2 value maps to correct color) and provides
    // a reasonable default for all indices
    for (int i = 0; i < 240; i++)
        graphics_set_palette(i, grb_to_rgb888(i));

    // Override indices 0-15 with standard Spectrum colors (solid, no dithering)
    for (int i = 0; i < 16; i++) {
        graphics_set_palette(i, spectrum_rgb888[i]);
        vga_set_palette_entry_solid(i, spectrum_rgb888[i]);
    }

    // Index 16 = ORANGE
    graphics_set_palette(16, 0xFF7F00);

    // Generate AluBytes table with palette indices (no sync bits)
    initAluBytes();

    precalcULASWAP();   // precalculate ULA SWAP values

    precalcborder32();  // Precalc border 32 bits values

    if (Config::gigascreen_enabled)
    {
        VIDEO::gigascreen_enabled = (Config::gigascreen_onoff == 1); // On=enabled, Auto=start disabled
        VIDEO::gigascreen_auto_countdown = 0;
        initGigascreenBlendLUT(); // Pre-compute blend palette entries
        InitPrevBuffer();   // For Gigascreen (needed for both On and Auto modes)
    }

///    SaveRect = (uint32_t *) SAVE_RECT; ///heap_caps_malloc(0x9000, MALLOC_CAP_INTERNAL | MALLOC_CAP_32BIT);

}

void VIDEO::Reset() {

    borderColor = 7;
    brd = border32[7];

    // Reset ULA+ state
    if (ulaplus_enabled) ulaPlusDisable();
    ulaplus_reg = 0;
    memcpy(ulaplus_palette, ulaplus_default_palette, 64);

    is169 = Config::aspect_16_9 ? 1 : 0;

    OSD = 0;

    if (Config::arch == "48K") {
        if (Config::romSet48 == "48Kby") {
            tStatesPerLine = TSTATES_PER_LINE_BYTE;
            tStatesScreen = TS_SCREEN_BYTE;
            tStatesBorder = is169 ? TS_BORDER_360x200_BYTE : TS_BORDER_320x240_BYTE;
        }
        else
        {
            tStatesPerLine = TSTATES_PER_LINE;
            tStatesScreen = TS_SCREEN_48;
            tStatesBorder = is169 ? TS_BORDER_360x200 : TS_BORDER_320x240;
        }
        VsyncFinetune[0] = is169 ? 0 : 0;
        VsyncFinetune[1] = is169 ? 0 : 0;

        Draw_OSD169 = MainScreen;
        Draw_OSD43 = BottomBorder;
        DrawBorder = TopBorder_Blank;
    } else if (Config::arch == "128K" || Config::arch == "ALF") {
        if (Config::romSet128 == "128Kby" || Config::romSet128 == "128Kbg") {
            tStatesPerLine = TSTATES_PER_LINE_BYTE;
            tStatesScreen = TS_SCREEN_BYTE;
            tStatesBorder = is169 ? TS_BORDER_360x200_BYTE : TS_BORDER_320x240_BYTE;
        }
        else
        {
            tStatesPerLine = TSTATES_PER_LINE_128;
            tStatesScreen = TS_SCREEN_128;
            tStatesBorder = is169 ? TS_BORDER_360x200_128 : TS_BORDER_320x240_128;
        }
        VsyncFinetune[0] = is169 ? 0 : 0;
        VsyncFinetune[1] = is169 ? 0 : 0;

        Draw_OSD169 = MainScreen;
        Draw_OSD43 = BottomBorder;
        DrawBorder = TopBorder_Blank;
    } else if (Config::arch == "Pentagon" || Config::arch == "P512" || Config::arch == "P1024") {
        tStatesPerLine = TSTATES_PER_LINE_PENTAGON;
        tStatesScreen = TS_SCREEN_PENTAGON;
        tStatesBorder = is169 ? TS_BORDER_360x200_PENTAGON : TS_BORDER_320x240_PENTAGON;
        // TODO: ADJUST THESE VALUES FOR PENTAGON
        VsyncFinetune[0] = is169 ? 0 : 0;
        VsyncFinetune[1] = is169 ? 0 : 0;

        Draw_OSD169 = MainScreen;
        Draw_OSD43 = BottomBorder_Pentagon;
        DrawBorder = TopBorder_Blank_Pentagon;
    }

    if (is169) {
        lin_end = 4;
        lin_end2 = 196;
    } else {
        lin_end = 24;
        lin_end2 = 216;
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

    // Restart border drawing
    lastBrdTstate = tStatesBorder;
    brdChange = false;
    brdnextframe = true;
#ifdef VGA_HDMI
    if (SELECT_VGA)
    {
        if (Config::vga_video_mode > 0)
        {
            if (Config::arch == "48K")
                Config::vga_video_mode=2;
            else if (Config::arch == "128K")
                Config::vga_video_mode=3;
            else
                Config::vga_video_mode=1;
        }
        video_mode = Config::vga_video_mode;
    }
    else
    {
        if (Config::hdmi_video_mode > 0)
        {
            if (Config::arch == "48K")
                Config::hdmi_video_mode=2;
            else if (Config::arch == "128K")
                Config::hdmi_video_mode=3;
            else
                Config::hdmi_video_mode=1;
        }
        video_mode = Config::hdmi_video_mode;
    }
#endif
}

void VIDEO::InitPrevBuffer() {
    vga.prevFrameBuffer = vga.allocateFrameBuffer();
    const int h = VIDEO::vga.yres;
    const size_t lineBytes = (size_t)VIDEO::vga.xres;
    for (int y = 0; y < h; ++y) {
        uint8_t *src = VIDEO::vga.frameBuffer[y];
        uint8_t *dst = VIDEO::vga.prevFrameBuffer[y];
        if (src && dst) std::memcpy(dst, src, lineBytes);
    }
}

//  VIDEO DRAW FUNCTIONS
IRAM_ATTR void VIDEO::MainScreen_Blank(unsigned int statestoadd, bool contended) {    
    
    CPU::tstates += statestoadd;

    if (CPU::tstates >= tstateDraw) {

        if (brdChange) DrawBorder(); // Needed to avoid tearing in demos like Gabba (Pentagon)
        
        lineptr32 = (uint32_t *)(vga.frameBuffer[linedraw_cnt]) + (is169 ? 13: 8);
        prevLineptr32 = (uint32_t *)(vga.prevFrameBuffer[linedraw_cnt]) + (is169 ? 13: 8);

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];
        
        #ifdef DIRTY_LINES
        // Force line draw (for testing)
        // dirty_lines[curline] = 1;
        #endif // DIRTY_LINES

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

        lineptr32 = (uint32_t *)(vga.frameBuffer[linedraw_cnt]) + (is169 ? 13: 8);
        prevLineptr32 = (uint32_t *)(vga.prevFrameBuffer[linedraw_cnt]) + (is169 ? 13: 8);

        coldraw_cnt = 0;

        curline = linedraw_cnt - lin_end;
        bmpOffset = offBmp[curline];
        attOffset = offAtt[curline];

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

        lineptr32 = (uint32_t *)(vga.frameBuffer[linedraw_cnt]) + (is169 ? 13: 8);
        prevLineptr32 = (uint32_t *)(vga.prevFrameBuffer[linedraw_cnt]) + (is169 ? 13: 8);

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
                graphics_set_palette(nextSlot, blended);
                nextSlot++;
            } else {
                gigsBlendLUT[i * 16 + j] = i; // fallback
            }
        }
    }
    gigsBlendLUTReady = true;
}

// Blend 4 packed palette-index pixels via LUT
inline uint32_t blendPixels32(uint32_t cur, uint32_t prev) {
    if (cur == prev) return cur;
    uint32_t result;
    uint8_t* r = (uint8_t*)&result;
    uint8_t* c = (uint8_t*)&cur;
    uint8_t* p = (uint8_t*)&prev;
    r[0] = gigsBlendLUT[(p[0] & 0x0F) * 16 + (c[0] & 0x0F)];
    r[1] = gigsBlendLUT[(p[1] & 0x0F) * 16 + (c[1] & 0x0F)];
    r[2] = gigsBlendLUT[(p[2] & 0x0F) * 16 + (c[2] & 0x0F)];
    r[3] = gigsBlendLUT[(p[3] & 0x0F) * 16 + (c[3] & 0x0F)];
    return result;
}

// ----------------------------------------------------------------------------------
// Fast video emulation with no ULA cycle emulation and no snow effect support
// ----------------------------------------------------------------------------------
/*IRAM_ATTR*/ void VIDEO::MainScreen(unsigned int statestoadd, bool contended) {    

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

    // Основной цикл
    for (; loopCount--; ) {
        uint8_t att = grmem[attOffset++];
        uint8_t bmp = (att & flashing) ? ~grmem[bmpOffset++] : grmem[bmpOffset++];

        uint32_t newPixel1 = AluByte[bmp >> 4][att];
        uint32_t newPixel2 = AluByte[bmp & 0xF][att];

        if (VIDEO::gigascreen_enabled)
        {
            uint32_t oldPixel1 = *prevLineptr32;
            uint32_t oldPixel2 = *(prevLineptr32 + 1);

            // Blend via palette LUT (4 pixels at once)
            uint32_t mix1 = blendPixels32(newPixel1, oldPixel1);
            uint32_t mix2 = blendPixels32(newPixel2, oldPixel2);

            *prevLineptr32++ = newPixel1;
            *prevLineptr32++ = newPixel2;

            *lineptr32++ = mix1;
            *lineptr32++ = mix2;
        }
        else
        {
            *lineptr32++ = newPixel1;
            *lineptr32++ = newPixel2;
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
        if (coldraw_osd >= 13 && coldraw_osd <= 30) {
            lineptr32+=2;
            attOffset++;
            bmpOffset++;
        } else {
            uint8_t att = grmem[attOffset++];
            uint8_t bmp = (att & flashing) ? ~grmem[bmpOffset++] : grmem[bmpOffset++];
            *lineptr32++ = AluByte[bmp >> 4][att];
            *lineptr32++ = AluByte[bmp & 0xF][att];
        }
        coldraw_osd++;
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

    linedraw_cnt = is169 ? 4 : 24;

    tstateDraw = tStatesScreen;

    if (VIDEO::snow_toggle) {
        Draw = &MainScreen_Blank_Snow;
        Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
    } else {
        Draw = &MainScreen_Blank;
        Draw_Opcode = &MainScreen_Blank_Opcode;
    }

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
    
    // Restart border drawing
    DrawBorder = Z80Ops::isPentagon ? &TopBorder_Blank_Pentagon : &TopBorder_Blank;
    lastBrdTstate = tStatesBorder;
    brdChange = false;

    if (Config::gigascreen_onoff == 2) { // Auto mode
        if (gigascreen_auto_countdown > 0) {
            gigascreen_auto_countdown--;
            if (!gigascreen_enabled) {
                if (!gigsBlendLUTReady) initGigascreenBlendLUT();
                gigascreen_enabled = true;
            }
        } else {
            if (gigascreen_enabled) gigascreen_enabled = false;
        }
    }

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

static int brdcol_cnt = 0;
static int brdlin_cnt = 0;

IRAM_ATTR void VIDEO::TopBorder_Blank() {
    if (CPU::tstates >= tStatesBorder) {
        brdcol_cnt = 0;
        brdlin_cnt = 0;
        brdptr32 = (uint32_t *)(vga.frameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
        prevBrdptr32 = (uint32_t *)(vga.prevFrameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
        DrawBorder = &TopBorder;
        DrawBorder();
    }
}    

IRAM_ATTR void VIDEO::TopBorder() {

    while (lastBrdTstate <= CPU::tstates) {

        Update_Border();

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == 40) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(vga.frameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            prevBrdptr32 = (uint32_t *)(vga.prevFrameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            brdcol_cnt = 0;
            lastBrdTstate += Z80Ops::is128 ? 68 : 64;
            if (brdlin_cnt == (is169 ? 4 : 24)) {                                
                DrawBorder = &MiddleBorder;
                MiddleBorder();
                return;
            }
        }

    }

}    

IRAM_ATTR void VIDEO::MiddleBorder() {
    while (lastBrdTstate <= CPU::tstates) {

        Update_Border();

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == 4) {
            lastBrdTstate += 128;
            brdcol_cnt = 36;
        } else if (brdcol_cnt == 40) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(vga.frameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            prevBrdptr32 = (uint32_t *)(vga.prevFrameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            brdcol_cnt = 0;          
            lastBrdTstate += Z80Ops::is128 ? 68 : 64;            
            if (brdlin_cnt == (is169 ? 196 : 216)) {                                
                DrawBorder = Draw_OSD43;
                DrawBorder();
                return;
            }
        }

    }

}    

IRAM_ATTR void VIDEO::BottomBorder() {

    while (lastBrdTstate <= CPU::tstates) {

        Update_Border();

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == 40) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(vga.frameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            prevBrdptr32 = (uint32_t *)(vga.prevFrameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            brdcol_cnt = 0;
            lastBrdTstate += Z80Ops::is128 ? 68 : 64;                        
            if (brdlin_cnt == (is169 ? 200 : 240)) {                
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}    

IRAM_ATTR void VIDEO::BottomBorder_OSD() {

    while (lastBrdTstate <= CPU::tstates) {

        if (brdlin_cnt < 220 || brdlin_cnt > 235) {
            Update_Border();
        } else if (brdcol_cnt < 21 || brdcol_cnt > 38) {
            Update_Border();
        } else {
            // OSD area: skip without writing (indexed access handles position via brdcol_cnt)
        }

        lastBrdTstate+=4;

        brdcol_cnt++;

        if (brdcol_cnt == 40) {
            brdlin_cnt++;
            brdptr32 = (uint32_t *)(vga.frameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            prevBrdptr32 = (uint32_t *)(vga.prevFrameBuffer[brdlin_cnt]) + (is169 ? 5 : 0);
            brdcol_cnt = 0;
            lastBrdTstate += Z80Ops::is128 ? 68 : 64;                                    
            if (brdlin_cnt == 240) {
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}    

IRAM_ATTR void VIDEO::Border_Blank() {

}    

static int brdcol_end = 0;
static int brdcol_end1 = 0;


// Border GigaScreen blend — uses same palette LUT as pixel blending
inline uint32_t Border_Gigascreen(uint32_t c0, uint32_t c1) {
    return blendPixels32(c0, c1);
}

void VIDEO::Update_Border() {
    uint32_t newColor = brd;
    uint8_t brdColIndex = brdcol_cnt * 2;
    if (VIDEO::gigascreen_enabled)
    {
        uint32_t oldColor = prevBrdptr32[brdColIndex];
        uint32_t mixedColor = Border_Gigascreen(newColor, oldColor);

        prevBrdptr32[brdColIndex] = newColor;
        prevBrdptr32[brdColIndex+1] = newColor;

        brdptr32[brdColIndex] = mixedColor;
        brdptr32[brdColIndex+1] = mixedColor;

        if (oldColor != newColor)
            brdGigascreenChange = true;
    }
    else
    {
        brdptr32[brdColIndex] = newColor;
        brdptr32[brdColIndex+1] = newColor;
    }
}

void VIDEO::Update_Border_Pentagon() {
    uint32_t newColor = brd;
    if (VIDEO::gigascreen_enabled)
    {
        uint8_t brdColIndex = brdcol_cnt ^ 1;
        uint32_t oldColor = prevBrdptr16[brdColIndex];
        uint32_t mixedColor = Border_Gigascreen(newColor, oldColor);

        prevBrdptr16[brdColIndex] = newColor;
        brdptr16[brdColIndex] = mixedColor;

        if (oldColor != newColor)
            brdGigascreenChange = true;
    }
    else
    {
        brdptr16[brdcol_cnt ^ 1] = newColor;
    }
}

IRAM_ATTR void VIDEO::TopBorder_Blank_Pentagon() {

    if (CPU::tstates >= tStatesBorder) {
        brdcol_cnt = is169 ? 10 : 0;
        brdcol_end = is169 ? 170 : 160;
        brdcol_end1 = is169 ? 26 : 16;
        brdlin_cnt = 0;
        brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
        prevBrdptr16 = (uint16_t *)(vga.prevFrameBuffer[brdlin_cnt]);
        DrawBorder = &TopBorder_Pentagon;
        DrawBorder();
    }

}

IRAM_ATTR void VIDEO::TopBorder_Pentagon() {
    while (lastBrdTstate <= CPU::tstates) {
        Update_Border_Pentagon();

        lastBrdTstate++;
        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr16 = (uint16_t *)(vga.prevFrameBuffer[brdlin_cnt]);
            brdcol_cnt = is169 ? 10 : 0;
            lastBrdTstate += 64;

            if (brdlin_cnt == (is169 ? 4 : 24)) {
                DrawBorder = &MiddleBorder_Pentagon;
                MiddleBorder_Pentagon();
                return;
            }
        }
    }
}

IRAM_ATTR void VIDEO::MiddleBorder_Pentagon() {

    while (lastBrdTstate <= CPU::tstates) {
        Update_Border_Pentagon();

        lastBrdTstate++;
        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end1) {
            lastBrdTstate += 128;
            brdcol_cnt = 144 + (is169 ? 10 : 0);
        } else if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr16 = (uint16_t *)(vga.prevFrameBuffer[brdlin_cnt]);
            brdcol_cnt = is169 ? 10 : 0;
            lastBrdTstate += 64;
            if (brdlin_cnt == (is169 ? 196 : 216)) {
                DrawBorder = Draw_OSD43;
                DrawBorder();
                return;
            }
        }

    }

}    

IRAM_ATTR void VIDEO::BottomBorder_Pentagon() {

    while (lastBrdTstate <= CPU::tstates) {
        Update_Border_Pentagon();

        lastBrdTstate++;
        brdcol_cnt++;

        if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr16 = (uint16_t *)(vga.prevFrameBuffer[brdlin_cnt]);
            brdcol_cnt = is169 ? 10 : 0;
            lastBrdTstate += 64;
            if (brdlin_cnt == (is169 ? 200 : 240)) {
                DrawBorder = &Border_Blank;
                return;
            }
        }

    }

}    

IRAM_ATTR void VIDEO::BottomBorder_OSD_Pentagon() {
    while (lastBrdTstate <= CPU::tstates) {
        if (brdlin_cnt < 220 || brdlin_cnt > 235)
            Update_Border_Pentagon();
        else if (brdcol_cnt < 84 || brdcol_cnt > 155)
            Update_Border_Pentagon();
        lastBrdTstate++;
        brdcol_cnt++;
        if (brdcol_cnt == brdcol_end) {
            brdlin_cnt++;
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr16 = (uint16_t *)(vga.prevFrameBuffer[brdlin_cnt]);
            brdcol_cnt = is169 ? 10 : 0;
            brdcol_cnt = 0;
            lastBrdTstate += 64;
            if (brdlin_cnt == 240) {
                DrawBorder = &Border_Blank;
                return;
            }
        }
    }
}

#define PSRAM_SHIFT_RAM (2ul << 20)

void SaveRectT::save(int16_t x, int16_t y, int16_t w, int16_t h) {
    if (offsets.empty()) {
        offsets.push_back(0);
    }
    x -= 2; if (x < 0) x = 0; // W/A
    w += 4; // W/A
    size_t off = offsets.back();
    #if 0
    if (butter_psram_size() >= PSRAM_SHIFT_RAM) {
        off += PSRAM_SHIFT_RAM;
        *(int16_t*)(PSRAM_DATA + off) = x; off += 2;
        *(int16_t*)(PSRAM_DATA + off) = y; off += 2;
        *(int16_t*)(PSRAM_DATA + off) = w; off += 2;
        *(int16_t*)(PSRAM_DATA + off) = h; off += 2;
        for (size_t line = y; line < y + h; ++line) {
            uint8_t *backbuffer = VIDEO::vga.frameBuffer[line];
            memcpy((void*)(PSRAM_DATA + off), backbuffer, w);
            for (int i = 0; i < w; ++i) {
                PSRAM_DATA[off + i] = VIDEO::vga.frameBuffer[line][x + i];
            }
            off += w;
        }
        offsets.push_back(off - PSRAM_SHIFT_RAM);
        return;
    }
    if (psram_size() >= PSRAM_SHIFT_RAM) {
        off += PSRAM_SHIFT_RAM;
        write16psram(off, x); off += 2;
        write16psram(off, y); off += 2;
        write16psram(off, w); off += 2;
        write16psram(off, h); off += 2;
        for (size_t line = y; line < y + h; ++line) {
            uint8_t *backbuffer = VIDEO::vga.frameBuffer[line];
            writepsram(off, backbuffer + x, w);
            off += w;
        }
        offsets.push_back(off - PSRAM_SHIFT_RAM);
        return;
    }
    #endif
    if (FileUtils::fsMount) {
        FIL f;
        f_open(&f, "/tmp/save_rect.tmp", FA_WRITE | FA_CREATE_ALWAYS);
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
    }
}
void SaveRectT::restore_last() {
    if (offsets.empty()) return; /// ???
    offsets.pop_back();
    size_t off = offsets.back();
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    #if 0
    if (butter_psram_size() >= PSRAM_SHIFT_RAM) {
        off += PSRAM_SHIFT_RAM;
        x = *(int16_t*)(PSRAM_DATA + off); off += 2;
        y = *(int16_t*)(PSRAM_DATA + off); off += 2;
        w = *(int16_t*)(PSRAM_DATA + off); off += 2;
        h = *(int16_t*)(PSRAM_DATA + off); off += 2;
        if (!w || !h) return;
        for (size_t line = y; line < y + h; ++line) {
            for (int i = 0; i < w; ++i) {
                VIDEO::vga.frameBuffer[line][x + i] = PSRAM_DATA[off + i];
            }
            off += w;
        }
    } else if (psram_size() >= PSRAM_SHIFT_RAM) {
        off += PSRAM_SHIFT_RAM;
        x = read16psram(off); off += 2;
        y = read16psram(off); off += 2;
        w = read16psram(off); off += 2;
        h = read16psram(off); off += 2;
        if (!w || !h) return;
        for (size_t line = y; line < y + h; ++line) {
            readpsram(VIDEO::vga.frameBuffer[line] + x, off, w);
            off += w;
        }
    } else
    #endif
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
    f_open(&f, "/tmp/save_rect.tmp", FA_WRITE | FA_CREATE_ALWAYS);
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
