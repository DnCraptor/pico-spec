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
#include "Subsystem.h"
#include "Buffer.h"
#include "Tape.h"
#include "FileUtils.h"
#include "VidPrecalc.h"
#include "CPU.h"
#include "ESPectrum.h"
#include "MemESP.h"
#include "Config.h"
#include "OSDMain.h"
#include "LEDIndicators.h"
#include "hardconfig.h"
#include "hardpins.h"
#include "Z80_JLS/z80.h"
#include "Z80_JLS/z80operations.h"
#include "psram_spi.h"
#include "Ports.h"
#if !PICO_RP2040
#include "Z80DMA.h"
#include "hardware/xip_cache.h"
#include "hardware/regs/addressmap.h"
#endif
extern "C" void graphics_set_palette(uint8_t i, uint32_t color888);
extern "C" void vga_set_palette_entry_solid(uint8_t i, uint32_t color888);
extern "C" void graphics_set_buffer(uint8_t* buffer, uint16_t width, uint16_t height);
extern "C" void graphics_set_scanlines(uint8_t level);
extern "C" void graphics_set_dither(bool enabled);
extern "C" void hdmi_reinit(void);
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
extern "C" void vga_reinit(void);
#endif
#if !PICO_RP2040
extern "C" void hdmi_set_profi_ds80_mode(bool active, const uint32_t *palette16, const uint8_t *pair_lut);
extern "C" void vga_set_profi_ds80_mode(bool active, const uint32_t *palette16, const uint8_t *pair_lut);
extern "C" volatile bool profi_ds80_active;
extern "C" volatile uint hdmi_current_line;

#ifndef VGA_HDMI
// Profi DS80 packed-pair display mode is implemented entirely inside the VGA/HDMI
// drivers (which SOFTTV/TFT/TV builds do not link).  These symbols are still
// referenced from shared code on the (never-taken) DS80 code paths, so provide
// fallbacks here: profi_ds80_active stays false → every DS80 branch is dead, and
// the mode-switch entry points become no-ops.
extern "C" volatile bool profi_ds80_active = false;
extern "C" void hdmi_set_profi_ds80_mode(bool, const uint32_t *, const uint8_t *) {}
extern "C" void vga_set_profi_ds80_mode(bool, const uint32_t *, const uint8_t *) {}
#endif
#endif
// Place hot video functions in SRAM instead of XIP flash
#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("video")

#pragma GCC optimize("O3")

VGA8Bit VIDEO::vga;

extern "C" uint8_t* __not_in_flash_func(getLineBuffer)(int line) {
    if (!VIDEO::vga.frameBuffer) return 0;
    return (uint8_t*)VIDEO::vga.frameBuffer[line];
}

extern "C" void __not_in_flash_func(ESPectrum_vsync)() {
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
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
extern bool SELECT_VGA;
#endif
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
uint8_t* VIDEO::profi_clrmem = nullptr;

// DS80 write-to-display-page interceptor (PROFI_PORT_TRACE only).
// writebyte() in MemESP.h compares ramCurrent[slot] against these to detect
// when the Z80 writes into the currently-displayed pixel or colour page.
#if PROFI_PORT_TRACE
extern uint8_t* ds80_dbg_clrmem;
extern uint8_t* ds80_dbg_grmem;
extern int      ds80_dbg_wr_cnt;
#endif

#if !PICO_RP2040
extern "C" uint8_t  read8psram(uint32_t addr32);
#endif

// Graphics-layer DS80 colour remap state (see Graphics8BitPalette).  Declared
// unconditionally in the header and referenced by inline dot()/drawChar()/etc.
// accessors in every build, so the definitions must exist for all targets — on
// RP2040 ds80_active stays false (no DS80 hires there) so the lut is never used.
bool    Graphics8BitPalette::ds80_active = false;
uint8_t Graphics8BitPalette::ds80_color_lut[17] = {0};

#if !PICO_RP2040
// Profi DS80 packed-pair framebuffer in butter PSRAM.
// Layout: PROFI_FB_W bytes/row = 32 black-pad + 256 content + 32 black-pad.
// 1 byte = pair of 4-bit palette indices via profi_pair_lookup[ink][paper].
// HDMI driver expands each fb byte to 2 different HDMI pixels → 512 native pixels.
#define PROFI_FB_W 320   // 32 left-pad + 256 content + 32 right-pad
#define PROFI_FB_H 240
// DS80 vertical border for the 720×576 (yres=288) full-border mode: the 240 content
// lines are centred, leaving 48 fb rows split symmetrically (24 top + 24 bottom).
// In 640×480 (yres=240) there is no vertical border (content fills the height).
#define DS80_BORDER_TOP 24
uint8_t  VIDEO::profi_pair_lookup[16][16];

// Profi DS80 default palette — GGGRRRBb format (3-3-2 bits).
// Raw startup bytes: {0x00,0x02,0x10,0x12,0x80,0x82,0x90,0x92,
//                     0x00,0x03,0x18,0x1B,0xC0,0xC3,0xD8,0xDB}
// Standard bytes have bit5=bit2=0 → G/R use only their 2 MSBs → dim=170(0xAA), bright=255.
extern "C" const uint32_t profi_default_palette16[16] = {
    0x000000, // 0  0x00 black
    0x0000AA, // 1  0x02 dark blue
    0xAA0000, // 2  0x10 dark red
    0xAA00AA, // 3  0x12 dark magenta
    0x00AA00, // 4  0x80 dark green
    0x00AAAA, // 5  0x82 dark cyan
    0xAAAA00, // 6  0x90 dark yellow
    0xAAAAAA, // 7  0x92 light gray
    0x000000, // 8  0x00 bright-black = same as index 0
    0x0000FF, // 9  0x03 bright blue
    0xFF0000, // 10 0x18 bright red
    0xFF00FF, // 11 0x1B bright magenta
    0x00FF00, // 12 0xC0 bright green
    0x00FFFF, // 13 0xC3 bright cyan
    0xFFFF00, // 14 0xD8 bright yellow
    0xFFFFFF, // 15 0xDB white
};

// Live Profi palette: modifiable at runtime via OUT (port_low=0x7E).
// Initialized to defaults at boot; refresh-applied to HDMI on each write.
uint32_t VIDEO::profi_palette_live[16] = {
    0x000000, 0x0000AA, 0xAA0000, 0xAA00AA, 0x00AA00, 0x00AAAA, 0xAAAA00, 0xAAAAAA,
    0x000000, 0x0000FF, 0xFF0000, 0xFF00FF, 0x00FF00, 0x00FFFF, 0xFFFF00, 0xFFFFFF
};

// Convert a Profi DS80 palette write to RGB888.
// Byte written via the #7E-style port trick: bits[7:5]=GX2:0, bits[4:2]=RX2:0,
// bits[1:0]=BX2:1. bx0 is the separate BX0 latch (port #FE bit7) — real DS80
// hardware combines it with BX2:1 to form a full 3-bit blue component, giving a
// genuine 3:3:3 (512-color) palette instead of the 3:3:2 (256-color) subset that
// bx0=0 software (which never touches #FE bit7) reduces to.
// G/R/B: 3-bit scale — levels 0..6 map to 0,43,85,128,170,213,255; level 7 clamps
// to 255. Standard palette bytes (bit5=bit2=0, bx0=0) decode identically to the
// old 3:3:2-only table: dim=170 (0xAA), bright=255.
static inline uint32_t profi_color_to_rgb888(uint8_t c, uint8_t bx0) {
    static const uint8_t gr[8] = {0, 43, 85, 128, 170, 213, 255, 255};
    uint8_t R = gr[(c >> 2) & 0x07];                 // bits[4:2] = RX2:0
    uint8_t G = gr[(c >> 5) & 0x07];                 // bits[7:5] = GX2:0
    uint8_t B = gr[((c & 0x03) << 1) | (bx0 & 1)];   // BX2:1 (bits[1:0]) + BX0
    return ((uint32_t)R << 16) | ((uint32_t)G << 8) | B;
}

// Per-frame swap/accessor snapshot for [NEG] attribution (see EndFrame).
volatile uint32_t g_frame_swap_us = 0, g_frame_swap_idle_us = 0, g_frame_accb = 0;

uint8_t VIDEO::profi_bx0_latch = 0;
uint8_t VIDEO::profi_gx0_latch = 0;
volatile bool VIDEO::profi_palette_dirty = false;
volatile bool VIDEO::profi_ds80_activate_pending   = false;
volatile bool VIDEO::profi_ds80_deactivate_pending = false;
bool VIDEO::profi_ds80_osd_active = false;

// Graphics-layer DS80 colour remap (see Graphics8BitPalette).  When DS80 is active,
// dotFast()/dot()/fillRect() pass standard ZX colour indices (0..16) through
// Graphics8BitPalette::ds80_color_lut[] so OSD/LED/menu drawing renders in the
// intended colour while the framebuffer byte still indexes the DS80 packed-pair
// conv_color table.  The lut maps ZX index → profi_pair_lookup[c][c] (a SOLID pair
// slot: both half-pixels = palette[c]).  ORANGE (16) has no DS80 slot → BRI_YELLOW.
// (ds80_active / ds80_color_lut defined unconditionally above.)

void VIDEO::rebuildDS80ColorLut() {
#if !PICO_RP2040
    for (int c = 0; c < 16; c++)
        Graphics8BitPalette::ds80_color_lut[c] = profi_pair_lookup[c][c];
    // ORANGE (16): no DS80 palette entry — fall back to BRI_YELLOW (14).
    Graphics8BitPalette::ds80_color_lut[16] = profi_pair_lookup[14][14];
#endif
}

// Saved copy of the running app's live Profi palette while the OSD "STD" override is
// active (so it can be restored verbatim on close).
#if !PICO_RP2040
static uint32_t profi_palette_saved[16];
static bool     profi_palette_saved_valid = false;

// Apply/clear the Profi DS80 packed-pair display mode on whichever video driver is
// compiled in.  DS80 relies on the VGA/HDMI driver's conv_color pair-slot machinery
// (and SELECT_VGA to pick between the two) — neither exists under SOFTTV/TFT, where
// there is no DS80 output path, so this is a no-op for those builds.
static inline void profi_ds80_driver_set(bool active, const uint32_t *palette16, const uint8_t *pair_lut) {
#ifdef VGA_HDMI
    extern bool SELECT_VGA;
    if (SELECT_VGA)
        vga_set_profi_ds80_mode(active, palette16, pair_lut);
    else
        hdmi_set_profi_ds80_mode(active, palette16, pair_lut);
#else
    (void)active; (void)palette16; (void)pair_lut;
#endif
}
#endif

// OSD palette override for DS80.  The Graphics-layer ZX→DS80 colour remap
// (Graphics8BitPalette::ds80_active) is already ON for the whole DS80 session — these
// functions only choose WHICH palette the solid pair slots resolve to, per the
// "OSD palette" menu toggle (Config::profi_ds80_std_palette_osd):
//
//   DS80: keep the running app's live palette unchanged → menu in app colours,
//         Profi background stays fully correct.  (apply = no-op)
//
//   STD : temporarily load the standard ZX palette into profi_palette_live and refresh
//         the DS80 pair slots (hdmi_set_profi_ds80_mode only rewrites palette slots
//         0..255 — it never touches the sync/audio/DMA region, so HDMI sync is safe).
//         → menu in TRUE ZX colours; the Profi background also shifts to ZX colours
//         (the accepted "OSD in ZX, background sacrificed" tradeoff).  The app palette
//         is saved here and restored by restoreProfiLivePalette().
void VIDEO::applyProfiOSDPalette() {
#if !PICO_RP2040
    if (Config::profi_ds80_std_palette_osd && !profi_palette_saved_valid) {
        // Save the app palette, load standard ZX defaults, refresh the pair slots + lut.
        for (int i = 0; i < 16; i++) profi_palette_saved[i] = profi_palette_live[i];
        profi_palette_saved_valid = true;
        for (int i = 0; i < 16; i++) profi_palette_live[i] = profi_default_palette16[i];
        profi_ds80_driver_set(true, profi_palette_live, &profi_pair_lookup[0][0]);
        rebuildDS80ColorLut();
    }
#endif
}

// Undo applyProfiOSDPalette()'s STD palette swap (restore the running app's palette).
// Does NOT touch ds80_active — the remap stays ON while DS80 is active.
void VIDEO::restoreProfiLivePalette() {
#if !PICO_RP2040
    if (profi_palette_saved_valid) {
        for (int i = 0; i < 16; i++) profi_palette_live[i] = profi_palette_saved[i];
        profi_palette_saved_valid = false;
        profi_ds80_driver_set(true, profi_palette_live, &profi_pair_lookup[0][0]);
        rebuildDS80ColorLut();
    }
#endif
}

// Drop the saved app-palette snapshot WITHOUT touching HDMI or re-enabling DS80.
// Used when DS80 was switched off (machine reset) while the OSD STD-palette override
// was active: the saved palette is no longer meaningful and must not trigger a
// hdmi_set_profi_ds80_mode(true,…) re-activation.
void VIDEO::discardProfiOSDPaletteSnapshot() {
#if !PICO_RP2040
    profi_palette_saved_valid = false;
#endif
}

// Re-blacken the DS80 side-padding columns (left 0..pad_l-1 and right pad_l+256..xres-1)
// across all framebuffer rows.  The DS80 renderer only ever rewrites the 256 content
// bytes per row, so padding that an OSD dialog drew over is otherwise left dirty after
// the menu closes (visible artefacts in the side border).  Call after OSD close.
void VIDEO::clearDS80Padding() {
#if !PICO_RP2040
    if (!vga.frameBuffer) return;
    const int pad_l = ((int)vga.xres - 256) / 2;
    if (pad_l <= 0) return;
    const int right_off = pad_l + 256;
    const int pad_r = (int)vga.xres - right_off;
    for (int y = 0; y < (int)vga.yres; y++) {
        uint8_t* row = (uint8_t*)vga.frameBuffer[y];
        if (!row) continue;
        memset(row, 0, pad_l);
        if (pad_r > 0) memset(row + right_off, 0, pad_r);
    }
#endif
}

void VIDEO::profiPaletteReset() {
    for (int i = 0; i < 16; i++) profi_palette_live[i] = profi_default_palette16[i];
    profi_palette_dirty = true; // refresh on next EndFrame if DS80 active
}

void VIDEO::profiPaletteApplyPending() {
#if !PICO_RP2040
    if (profi_palette_dirty && profi_ds80_active
        && !profi_ds80_activate_pending && !profi_ds80_deactivate_pending) {
        profi_ds80_driver_set(true, profi_palette_live, &profi_pair_lookup[0][0]);
        profi_palette_dirty = false;
    }
#endif
}

void VIDEO::profiPaletteWrite(uint8_t index, uint8_t profi_color) {
#if !PICO_RP2040
    // Only honor palette writes when DS80 mode is active — avoids corrupting
    // defaults from incidental port-0x7E writes during BIOS startup setup.
    if (!profi_ds80_active) return;
    uint8_t idx = index & 0x0F;
    uint32_t newRgb = profi_color_to_rgb888(profi_color, profi_bx0_latch);
    profi_palette_live[idx] = newRgb;
    // Defer HDMI refresh to HDMI ISR vblank (set dirty flag, applied in EndFrame).
    profi_palette_dirty = true;
#endif
}

// Build profi_pair_lookup[ink][paper] → HDMI conv_color slot index.
//
// Slot budget (250 available):
//   0..239:   free pair data slots (220..237 = audio range, suppressed in DS80 by DMA handler)
//   240..244: BASE_HDMI_CTRL_INX + IDX_SCANLINE — NEVER pair data (sync/porch)
//   245..254: free pair data slots
//   255:      border fill — NEVER pair data
//
// We need 250 unique pairs (256 total − 6 merges).
// Merges: ink=0..5, paper=8 → paper=0  (bright-black bg = black bg for these inks).
//   ink=6..15 (INCLUDING ink=8): NO normalization.
//   → (8,8) gets its OWN slot → both pixels render as palette[8] → solid color, no stripes.
//   → ink=8 is fully independent from ink=0 → changing palette[8] can't corrupt black pixels.
// written[] guard in hdmi_set_profi_ds80_mode() ensures paper=0 TMDS wins for merged slots.
static void init_profi_pair_lookup() {
    // safe[]: all slots except 240..244 (sync/scanline) and 255 (border) → 250 slots.
    // With HDMI audio the Data Island machinery owns 40 more indices (184..199 =
    // DI data set 2, 216..239 = preambles/guards/data set 0 — see hdmi.c), so
    // pair_lut must avoid them → 210 slots; the bright-ink × bright-paper merges
    // below free exactly the difference (40 extra merges). VGA video or any
    // non-HDMI audio driver keeps the full 250-pair palette.
    bool reserve_di = false;
#if defined(VGA_HDMI)
    extern bool SELECT_VGA;
    reserve_di = !SELECT_VGA && Config::audio_driver == 4;
#endif
    int safe[256], ns = 0;
    for (int i = 0; i < 256; i++) {
        if (i >= 240 && i <= 244) continue;
        if (i == 255) continue;
        if (reserve_di && ((i >= 184 && i <= 199) || (i >= 216 && i <= 239))) continue;
        safe[ns++] = i;  // ns == 250 (210 with reserve_di)
    }
    bool assigned[16][16] = {};
    int slot_idx = 0;
    for (int ink = 0; ink < 16; ink++) {
        for (int paper = 0; paper < 16; paper++) {
            // For inks 0..5 only: paper=8 → paper=0 (6 merges to fit in 250 slots).
            // For inks 6..15 (including ink=8): full independence — paper=8 is own slot.
            int cp = (paper == 8 && ink <= 5) ? 0 : paper;
            // HDMI-audio slot diet: bright ink (9..15) on a DIFFERENT bright paper
            // renders on the base paper (p-8) instead — 42 candidates minus the two
            // preserved classic text schemes (white-on-bright-blue and
            // bright-blue-on-white) = exactly the 40 reserved DI slots. Diagonals
            // (ink==paper) keep their own slots, so solid fills are unaffected.
            if (reserve_di && ink >= 9 && paper >= 9 && ink != paper
                && !(ink == 15 && paper == 9) && !(ink == 9 && paper == 15))
                cp = paper - 8;
            if (!assigned[ink][cp]) {
                assigned[ink][cp] = true;
                VIDEO::profi_pair_lookup[ink][paper] = (uint8_t)safe[slot_idx++];
            } else {
                VIDEO::profi_pair_lookup[ink][paper] = VIDEO::profi_pair_lookup[ink][cp];
            }
        }
    }
    // slot_idx == 250 here (210 with reserve_di)
}

bool VIDEO::isProfiDS80() {
    return Config::arch == "Profi" && (Ports::portDFFD & 0x80);
}
#endif
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
#if !PICO_RP2040
// Profi DS80 border: per-T-state writes into the packed-pair framebuffer
// (1T = 4px = 1 uint16_t of two pair bytes).  Geometry applied by
// applyDS80BorderGeometry(); ds80_brd_col_off shifts the 160 visible
// T-columns inside wider rows (720×576: 180 uint16 cols → off=10).
static bool ds80_border_geom = false;
static int  ds80_brd_col_off = 0;
static bool ds80_osd_carve = false; // stats overlay visible → carve its rect
static bool ds80_carve240 = false;  // stats rect coords differ 640×480 vs 720×576
#endif // !PICO_RP2040
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
bool VIDEO::ulaplus_alubytes_dirty = false;
static bool gigascreen_lut_rebuild_deferred = false;
// AluBytesUlaPlus moved to flash — see roms/AluBytesUlaPlus.c

// 16col mode (Pentagon, Alone Coder)
bool VIDEO::mode16col_enabled = false;
const uint8_t* VIDEO::mode16col_planes[4] = { nullptr, nullptr, nullptr, nullptr };
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
static unsigned int video_rest;
static unsigned int video_opcode_rest;
static unsigned int curline;

#if !PICO_RP2040
// DS80 per-frame display-page latch (UnrealSpeccy rend_profi model).
// The Kings Valley CP/M game flips videoLatch (0x7FFD bit3) hundreds of times per
// display frame, so reading the live grmem/profi_clrmem per scanline (ZXMAK2 beam
// model) interleaves pages 4/6 across the screen → horizontal stripes.
// UnrealSpeccy instead picks the page ONCE per frame (base = p7FFD bit3 ? page6:page4)
// and renders all 240 lines from it.  We mirror that: latch these at the first content
// line (curline==0) and use them for the entire frame; mid-frame Z80 flips only change
// the live grmem/profi_clrmem, which we pick up at the next frame's line 0.
static uint8_t* ds80_frame_grmem  = nullptr;
static uint8_t* ds80_frame_clrmem = nullptr;
// SRAM snapshot of the 16 KB clrmem page (pages 56/58).  Lives at file scope so
// EndFrame (vblank) can populate it before the rasterizer runs.  Allocated from
// heap only when arch==Profi on butter/QSPI boards (see VIDEO::Reset), where
// pages 56/58 are plain XIP butter pointers; on SPI-PSRAM/SWAP boards those
// pages are force_sram_locked heap SRAM and no snapshot is needed.
#define DS80_CLR_SRAM_SIZE 16384
static uint8_t* ds80_clr_sram = nullptr;
#endif

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

#if !PICO_RP2040
// Apply / restore the Profi DS80 screen+border timing geometry (ZXMAK2
// ProfiRenderer model: 192 T/line, paper at line 48 tact 24 minus 19T INT
// offset (see Video.h), 16T side borders, 1T = 4 px = 1 uint16_t of packed pair bytes).
// on=true:  called from Reset() DS80 branch and EndFrame() activation.
// on=false: called from EndFrame() deactivation — restores the standard
// Profi geometry that Reset() computes for non-DS80 (224 T/line).
static void applyDS80BorderGeometry(bool on) {
    if (on) {
        VIDEO::tStatesPerLine = TSTATES_PER_LINE_PROFI_DS80;
        VIDEO::tStatesScreen  = TS_SCREEN_PROFI_DS80;
        const bool tall = (int)VIDEO::vga.yres >= 288;  // 720×576: 24-row top/bottom bands
        VIDEO::tStatesBorder = tall ? TS_BORDER_PROFI_DS80_288 : TS_BORDER_PROFI_DS80_240;
        lin_end  = tall ? DS80_BORDER_TOP : 0;
        lin_end2 = lin_end + 240;
        // Defensive: never let the border/content machines run past the fb
        // (e.g. DS80 forced while a <240-row video mode is active).
        if (lin_end2 > VIDEO::vga.yres) lin_end2 = VIDEO::vga.yres;
        brdcol_step    = 1;
        brdPairWrite   = false;
        brdcol_start   = 0;
        brdcol_end     = 160;          // 16T left + 128T paper + 16T right
        brdcol_end1    = 16;
        brdcol_retrace = brdcol_end;
        // Row width in uint16 cols minus the 160 visible T-cols, split evenly:
        // 640×480 (320 B/row → 160 cols) → 0; 720×576 (360 B → 180 cols) → 10.
        ds80_brd_col_off = ((int)VIDEO::vga.xres / 2 - 160) / 2;
        ds80_border_geom = true;
    } else {
        ds80_border_geom = false;
        ds80_brd_col_off = 0;
        VIDEO::tStatesPerLine = TSTATES_PER_LINE_PROFI;
        VIDEO::tStatesScreen  = TS_SCREEN_PROFI;
        const bool isFB    = VIDEO::isFullBorderMode();
        const bool isFB240 = VIDEO::isFullBorder240();
        const bool is169   = Config::aspect_16_9 != 0;
        VIDEO::tStatesBorder = isFB ? (isFB240 ? TS_BORDER_360x240_PROFI : TS_BORDER_360x288_PROFI)
                             : is169 ? TS_BORDER_360x200_PROFI : TS_BORDER_320x240_PROFI;
        if      (isFB && !isFB240) { lin_end = 48; lin_end2 = 240; }
        else if (isFB)             { lin_end = 24; lin_end2 = 216; }
        else if (is169)            { lin_end = 4;  lin_end2 = 196; }
        else                       { lin_end = 24; lin_end2 = 216; }
        brdcol_end     = isFB ? 180 : 160;
        brdcol_step    = 1;
        brdPairWrite   = false;
        brdcol_start   = is169 ? 10 : 0;
        brdcol_end1    = isFB ? 26 : brdcol_start + (brdcol_end - brdcol_start - 128) / 2;
        brdcol_retrace = brdcol_end;
    }
    Select_Update_Border();
}
#endif // !PICO_RP2040

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

void VIDEO::updateBorderBrd() {
#if !PICO_RP2040
    if (isProfiDS80()) {
        // DS80 border colour = Palette[(~borderIndex) & 7] (inverse index, per
        // ZXMAK2 ProfiRenderer m_borderColorPaper), mapped to its solid pair slot.
        uint8_t bidx = (uint8_t)(~borderColor) & 0x07;
        uint8_t b = profi_pair_lookup[bidx][bidx];
        brd = (uint32_t)b * 0x01010101u;
        return;
    }
#endif
    brd = border32[borderColor];
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
        if (f_stat(PALETTE_NVS, &fi) != FR_OK) {
            FileUtils::mkdirParents(CONFIG_DIR);
            FIL cf;
            if (f_open(&cf, PALETTE_NVS, FA_WRITE | FA_CREATE_NEW) == FR_OK) {
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
    if (f_open(&fil, PALETTE_NVS, FA_READ) != FR_OK)
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

// VGA Bayer lo/hi for a single channel: snaps to the /21->2bit grid (0,85,170,255).
// lo = VGA level * 85, hi = (VGA level + 1) * 85, sub = fraction within a VGA level group.
static inline int vga_lo_chan(int c) { return ((c / 21) >> 2) * 85; }
static inline int vga_hi_chan(int c) { int hi = ((c / 21) >> 2) + 1; return (hi > 3 ? 3 : hi) * 85; }
static inline int vga_sub_chan(int c) { return (c / 21) & 3; }

#if !PICO_RP2040
// Apply ULA+ palette entry i (and Bayer-dither neighbour at i|0x40 for HDMI).
// Mirrors VGA Bayer: only dither when sub > 0 for at least one channel.
// When sub=0 for all channels the colour lands exactly on the VGA grid → solid,
// no checkerboard (same as vga_rgb888_dither which sets all 4 positions to lo).
static inline void applyUlaPlusPalette(int i) {
    uint32_t orig = paletteTransform(grb_to_rgb888(VIDEO::ulaplus_palette[i]));
    int R = (orig >> 16) & 0xFF, G = (orig >> 8) & 0xFF, B = orig & 0xFF;
    bool needs = vga_sub_chan(R) || vga_sub_chan(G) || vga_sub_chan(B);
    uint32_t neigh = needs
        ? (((uint32_t)vga_hi_chan(R) << 16) | ((uint32_t)vga_hi_chan(G) << 8) | vga_hi_chan(B))
        : orig;
    graphics_set_palette(i, orig);
    // Always populate the dither slot so toggling Config::hdmi_dither at runtime
    // does not require a palette rebuild. When dither is off, hdmi.c does not
    // read palette[i|0x40], so the cost is just an extra LUT write per entry.
    graphics_set_palette(i | 0x40, neigh);
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
    ulaplus_alubytes_dirty = false;
    flashing = 0;
    // Dither is meaningful only with ULA+; force it off so palette[64..127]
    // (which now hold default G3R3B2 values, not Bayer neighbours) won't be
    // sampled by the HDMI ISR.
    graphics_set_dither(false);
    // Restore palette: indices 0-63 back to G3R3B2 defaults, then 0-15 to Spectrum (solid).
    // When GigaScreen is enabled we must NOT touch slots 17..63 — those hold blend
    // values built at boot, and rewriting them here would force a heavy 120-entry
    // re-emit and race with HDMI DMA reading conv_color from port-write context.
    int g3r3b2_upper = Config::gigascreen_enabled ? 17 : 64;
    for (int i = 0; i < g3r3b2_upper; i++)
        graphics_set_palette(i, paletteTransform(grb_to_rgb888(i)));
    for (int i = 0; i < 16; i++) {
        uint32_t color = paletteTransform(spectrum_rgb888[i]);
        graphics_set_palette(i, color);
        vga_set_palette_entry_solid(i, color);
    }
    if (Config::gigascreen_enabled) {
        // ULA+ active phase clobbered Gigascreen blend slots 17..127 via applyUlaPlusPalette.
        // Signal EndFrame to rebuild them from the safe blanking context.
        gigascreen_lut_rebuild_deferred = true;
        // Reseed prevFrameBuffer so the first Gigascreen frame after ULA+ doesn't blend
        // against stale ULA+ pixel indices (which were never stored as valid 4-bit Gigs data).
        InitPrevBuffer();
    }

    for (int n = 0; n < 16; n++)
        AluByte[n] = (unsigned int*)AluBytesStd_flash[n];
    updateBorderBrd();
    brdChange = true;
}

// 16col (Alone Coder, Pentagon): 4bpp packed pixels, no attributes. Each byte
// = 2 horizontally-adjacent pixels (hi nibble = left/even, lo = right/odd).
// Per speccy.info: 4 6144-byte planes living in pages 5 (#4000/#6000) and 4
// (#C000/#E000 when bank=4). Pixel order in the 8-pixel byte-column: planes
// are sourced for pixel pairs in the order described by UnrealSpeccy
// dxr_4bpp.cpp p4bpp_ofs[] — column-pair i mod 4 selects the plane.
// Precomputed LUT: IiGRBgrb byte -> uint16_t with two 4-bit palette indices.
// Low byte = LEFT pixel  = (bit6 << 3) | (bits 2..0)  = (i, grb)
// High byte = RIGHT pixel = (bit7 << 3) | (bits 5..3) = (I, GRB)
// 512 bytes in SRAM, hot in cache. Reading one byte from a Pentagon plane
// becomes a single LDRH from this table — no shift/mask in the inner loop.
// 512 B heap LUT, allocated only while 16col is enabled (Config::mode16col_onoff)
// and freed when off — so the feature costs ZERO SRAM when disabled.
static uint16_t* mode16col_decode_lut = nullptr;

void VIDEO::ensure16colLut() {
    if (!mode16col_decode_lut) {
        mode16col_decode_lut = (uint16_t*)malloc(256 * sizeof(uint16_t));
        if (!mode16col_decode_lut) return;
    }
    for (int x = 0; x < 256; x++) {
        uint8_t L = (uint8_t)((((x) >> 3) & 0x08) | ((x) & 0x07));
        uint8_t R = (uint8_t)((((x) >> 4) & 0x08) | (((x) >> 3) & 0x07));
        mode16col_decode_lut[x] = (uint16_t)L | ((uint16_t)R << 8);
    }
}

void VIDEO::free16colLut() {
    free(mode16col_decode_lut);
    mode16col_decode_lut = nullptr;
}

void VIDEO::mode16colUpdatePlanes() {
    uint8_t pLow = MemESP::videoLatch ? 6 : 4;   // #C000/#E000 area (when bank=4)
    uint8_t pHi  = MemESP::videoLatch ? 7 : 5;   // #4000/#6000 area
    uint8_t* baseLow = MemESP::ram[pLow].direct();
    uint8_t* baseHi  = MemESP::ram[pHi].direct();
    mode16col_planes[0] = baseLow;            // page 4/6, +0x0000  (#C000)
    mode16col_planes[1] = baseHi;             // page 5/7, +0x0000  (#4000)
    mode16col_planes[2] = baseLow + 0x2000;   // page 4/6, +0x2000  (#E000)
    mode16col_planes[3] = baseHi  + 0x2000;   // page 5/7, +0x2000  (#6000)
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

extern size_t getContiguousHeap(void);

// Minimum heap headroom before calling FatFS f_open. ff_memalloc reserves
// (FF_MAX_LFN+1)*2 + MAXDIRB(FF_MAX_LFN) ≈ 1-2 KB for LFN/VFAT scratch. SDK
// malloc panics on OOM (no NULL return), so we must gate this with sbrk
// headroom (the only allocator-friendly free-memory measure). Needed by
// SaveRectT below on every platform — keep outside the !PICO_RP2040 block.
#define FF_OPEN_HEAP_FLOOR 4096u

#if !PICO_RP2040
// Shared framebuffer pointer arrays sized for the build-time maximum line count
// so they don't need realloc on mode changes. The actual data blocks
// (sharedFB_main / sharedFB_prev) are sized to fit the current mode — see
// ensureMainFB / ensurePrevFB.
// prevFrameBuffer is 4-bit packed (2 px/byte): Gigascreen blendLUT only uses
// prev & 0x0F, so the upper nibble is free for the next pixel.
#define FB_MAX_LINES 289   // calcLines(288), the largest mode (360x288)

static int fbCalcLines(int count) {
    if (count == 288) return 289;
    if (count == 240) return 241;
    if (count == 480) return 241;
    if (count == 400) return 201;
    return count;
}

// Two separate blocks: main FB (always present) and prev FB (Gigascreen only).
// Splitting them lets GsSubsys free up to 52 020 B of SRAM when Gigascreen is off.
// Each block is sized to fit the current video mode, not the build-time maximum,
// and grown/shrunk via realloc on mode changes (see ensureMainFB/ensurePrevFB).
static uint8_t *sharedFB_main = nullptr;  // sized for current mode
static uint8_t *sharedFB_prev = nullptr;  // sized for current mode (Gigascreen only)
static size_t sharedFB_main_size = 0;     // actual byte capacity of sharedFB_main
static size_t sharedFB_prev_size = 0;     // actual byte capacity of sharedFB_prev
// Backing store for sharedFB_prev. Tiered: butter PSRAM (XIP-addressable) is
// preferred so the ~52 KB prev-FB no longer eats SRAM on PSRAM boards; falls back
// to the heap when no butter PSRAM is present. NEED_POINTER keeps it addressable
// for the per-pixel blend hot path (SPI PSRAM/SD-swap are never selected).
static Buffer sharedFB_prevBuf;
// Chunked fallback for the prev-FB. It is only ever addressed row by row, through
// vga.prevFrameBuffer[] — so it does not have to be one block, and cutting it into
// whole-row pieces makes a mid-session Gigascreen enable work on a fragmented heap
// (hw, PICO_DV without PSRAM: 66 KB free but the largest hole 36.5 KB vs a 37.7 KB
// prev-FB — previously only a reboot could place it, because setup() allocates before
// anything else fragments the heap). The single block is still tried first, so butter
// boards keep the XIP placement and nothing below changes for them.
// The three things that DO require one region opt out explicitly: the XIP scanline
// window (butter-only, guarded in pwRefreshGate), whole-region cache maintenance
// (only reachable with that window on) and gigascreenLendRegion (declines).
#define PREV_CHUNKS_MIN 4
#define PREV_CHUNKS_MAX 8
// Never ask for the heap's very last block: Buffer's last-resort malloc PANICS on
// failure (pico_malloc), so every chunk is pre-checked with this much to spare.
#define PREV_CHUNK_SLACK 1024
static Buffer sharedFB_prevChunk[PREV_CHUNKS_MAX];
static int    sharedFB_prev_nchunks = 0;      // 0 = single block in sharedFB_prevBuf
static int    sharedFB_prev_chunk_rows = 0;   // rows per chunk (last one may hold fewer)
static void **sharedFB_arr1 = nullptr;    // pointer array for frameBuffer (FB_MAX_LINES slots)
static void **sharedFB_arr2 = nullptr;    // pointer array for prevFrameBuffer

// Cached current resolution so GsSubsys can re-call setupSharedFBPointers
// without reaching back into vidmodes[].
static int sharedFB_lines = 0;
static int sharedFB_stride = 0;

// prevFB DMA scanline window (defined below, butter-PSRAM boards only).
#if !PICO_RP2040
static void pwShutdown();
static void pwDrop();
#else
static inline void pwShutdown() {}
static inline void pwDrop() {}
#endif

static inline size_t fbMainBytes(int lines, int stride) {
    return (size_t)lines * (size_t)stride;
}
static inline size_t fbPrevBytes(int lines, int stride) {
    return (size_t)lines * (size_t)(stride / 2);  // 4-bit packed: 2 px/byte
}

// FB allocation happens once at boot via these. Mode-change runtime resize
// was removed because heap fragmentation made grow impossible — switching
// video modes now triggers a hard reset (see Config::pending_vga/hdmi mode).
static bool ensureMainFB(int lines, int stride) {
    size_t want = fbMainBytes(lines, stride);
    if (sharedFB_main && sharedFB_main_size == want) return true;
    if (sharedFB_main) { free(sharedFB_main); sharedFB_main = nullptr; sharedFB_main_size = 0; }
    uint8_t *p = (uint8_t*)malloc(want);
    if (!p) return false;
    memset(p, 0, want);
    sharedFB_main = p;
    sharedFB_main_size = want;
    return true;
}

extern "C" size_t getLargestAllocatable(void);  // OSDMain.cpp — malloc panics on OOM
extern size_t getFreeHeap(void);                // ESPectrum.cpp (also declared below)

// Row `row` of the prev-FB, whichever way it is backed. The ONLY way the render
// path reaches prev-FB rows is the pointer array these fill, so chunking is
// invisible above this line.
static inline uint8_t* prevRowPtr(int row, int prev_stride) {
    if (!sharedFB_prev_nchunks) return sharedFB_prev + (size_t)row * prev_stride;
    const int c = row / sharedFB_prev_chunk_rows;
    return (uint8_t*)sharedFB_prevChunk[c].data()
         + (size_t)(row - c * sharedFB_prev_chunk_rows) * prev_stride;
}

static void prevFBFree() {
    sharedFB_prevBuf.free();
    for (int c = 0; c < sharedFB_prev_nchunks; c++) sharedFB_prevChunk[c].free();
    sharedFB_prev_nchunks = 0;
    sharedFB_prev_chunk_rows = 0;
    sharedFB_prev = nullptr;
    sharedFB_prev_size = 0;
}

// Rows per chunk / largest single allocation at the finest split we will attempt.
static inline int prevChunkRows(int lines, int n) { return (lines + n - 1) / n; }
size_t VIDEO::gigascreenPrevFBBlockBytes() {
    if (!sharedFB_lines) return gigascreenPrevFBBytes();
    return (size_t)prevChunkRows(sharedFB_lines, PREV_CHUNKS_MAX) * (sharedFB_stride / 2);
}

// Split the prev-FB across n..PREV_CHUNKS_MAX blocks of whole rows, fewest first.
static bool prevFBAllocChunked(int lines, int prev_stride, size_t want) {
    // Butter PSRAM absorbs the prev-FB (Buffer routes PREFER_PSRAM there first), so
    // the heap pre-checks below must not speak for it — see ensurePrevFB.
    const bool heap_backed = (butter_psram_size() == 0);
    for (int n = PREV_CHUNKS_MIN; n <= PREV_CHUNKS_MAX; n *= 2) {
        const int rows = prevChunkRows(lines, n);
        int got = 0;
        bool ok = true;
        for (int c = 0; c < n; c++) {
            const int r0 = c * rows;
            if (r0 >= lines) break;                      // fewer chunks than n suffice
            const int rc = (r0 + rows <= lines) ? rows : (lines - r0);
            const size_t bytes = (size_t)rc * prev_stride;
            if ((heap_backed && getLargestAllocatable() < bytes + PREV_CHUNK_SLACK) ||
                !sharedFB_prevChunk[c].alloc(bytes, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM)) {
                ok = false;
                break;
            }
            memset(sharedFB_prevChunk[c].data(), 0, bytes);
            got = c + 1;
        }
        if (ok) {
            sharedFB_prev_nchunks    = got;
            sharedFB_prev_chunk_rows = rows;
            sharedFB_prev            = sharedFB_prevChunk[0].data();  // also the "exists" marker
            sharedFB_prev_size       = want;
            Debug::log("VIDEO: prevFB %uKB in %d chunks x %d rows on %s (fragmented heap)",
                       (unsigned)(want >> 10), got, rows, sharedFB_prevChunk[0].tierName());
            return true;
        }
        for (int c = 0; c < got; c++) sharedFB_prevChunk[c].free();
        sharedFB_prev_nchunks = 0;
    }
    return false;
}

static bool ensurePrevFB(int lines, int stride) {
    if (Config::arch == "Profi") return true; // Gigascreen not available for Profi
    const int prev_stride = stride / 2;
    size_t want = fbPrevBytes(lines, stride);
    if (sharedFB_prev && sharedFB_prev_size == want) return true;
    if (sharedFB_prev) {
        pwShutdown();  // stop window DMA / drop cached state before freeing
        prevFBFree();
    }
    // Memory policy lives in Subsystem: can this prevFB be allocated without starving
    // the heap on a butter-less board? (Butter-PSRAM boards always pass — it goes to
    // XIP.) The block requirement is per-allocation, so it is asked about the chunk
    // size, not the total. Decline → GsSubsys::apply() cleanly disables Gigascreen.
    const size_t block_need = (size_t)prevChunkRows(lines, PREV_CHUNKS_MAX) * prev_stride;
    if (!Subsystems::gigascreenPrevFBAffordable(want, block_need)) return false;
    // Butter PSRAM first (frees SRAM), heap fallback. NEED_POINTER keeps it
    // addressable for the per-pixel blend; SPI PSRAM / SD-swap are never picked.
    // One block is preferred (butter/XIP placement, lendable, DMA window eligible);
    // whole-row chunks only when the heap has no hole that big.
    // The heap pre-check guards the heap fallback only: on a butter board the buffer
    // lands in XIP PSRAM at no heap cost (which is also why gigascreenPrevFBAffordable
    // short-circuits there), and letting a fragmented heap push it to chunks would
    // needlessly forfeit the XIP scanline window (pwRefreshGate declines chunks).
    if ((butter_psram_size() != 0 || getLargestAllocatable() >= want + PREV_CHUNK_SLACK) &&
        sharedFB_prevBuf.alloc(want, Buffer::NEED_POINTER | Buffer::PREFER_PSRAM)) {
        uint8_t *p = sharedFB_prevBuf.data();
        memset(p, 0, want);
        sharedFB_prev = p;
        sharedFB_prev_size = want;
        Debug::log("VIDEO: prevFB %uKB on %s", (unsigned)(want >> 10), sharedFB_prevBuf.tierName());
        return true;
    }
    sharedFB_prevBuf.free();
    if (prevFBAllocChunked(lines, prev_stride, want)) return true;
    Debug::log("VIDEO: prevFB alloc failed (want=%u) — Gigascreen off this session", (unsigned)want);
    return false;
}

static void setupSharedFBPointers(Graphics<unsigned char> &vga, int lines, int stride) {
    sharedFB_lines = lines;
    sharedFB_stride = stride;
    for (int i = 0; i < lines; i++) {
        sharedFB_arr1[i] = sharedFB_main + i * stride;
    }
    vga.frameBuffer = (unsigned char **)sharedFB_arr1;
    if (sharedFB_prev && sharedFB_arr2) {
        int prev_stride = stride / 2; // 4-bit packed
        for (int i = 0; i < lines; i++) {
            sharedFB_arr2[i] = prevRowPtr(i, prev_stride);
        }
        vga.prevFrameBuffer = (unsigned char **)sharedFB_arr2;
    } else {
        vga.prevFrameBuffer = nullptr;
    }
}

// ── Gigascreen prevFB scanline window (butter/XIP PSRAM boards) ──────────────
// On butter boards the prev-FB lives in XIP PSRAM. Direct CACHED access there
// is memory-bound: the per-frame full sweep (52 KB > 16 KB XIP cache) stalls
// the CPU on every read miss AND on every dirty-line writeback eviction.
// This window keeps the render hot path in SRAM: the blend/border code
// reads/writes small SRAM row buffers, and rows are staged to/from the PSRAM
// prev-FB through the UNCACHED XIP alias (+PW_UNCACHED) by tight RAM-resident
// CPU copy loops. Sequential accesses merge into linear QMI bursts (COOLDOWN),
// and the XIP cache never holds prev-FB lines while the window is active.
//
// Transport history (important — do not "optimize" back):
//  • CPU cached memcpy window: measured WORSE than direct access (cache thrash
//    + writeback storms) — see m1p2 notes.
//  • DMA chains through the uncached alias: passed all functional self-tests
//    but KILLED the video output — all DMA channels share one read and one
//    write bus manager, and a single in-flight beat to the slow QMI PSRAM
//    blocks the manager for tens of cycles while the HDMI scanout DMA needs a
//    beat every ~14 cycles. HIGH_PRIORITY cannot preempt an in-flight beat →
//    PIO FIFO underrun → no sync. (16-bit DMA beats to QMI also stalled
//    outright on non-word-aligned segment starts.)
//  • CPU uncached copies (this code): the CPU uses its own bus port, so the
//    DMA managers — and therefore HDMI — never see this traffic at all.
//
// Two independent raster-ordered streams, each with 2 row buffers:
//   • content — MainScreen blend loop rows [lin_end, lin_end2)
//   • border  — Update_Border rows [0, yres), possibly swept in bursts
// Both may hold the SAME row concurrently; writebacks are restricted to the
// byte segments each stream owns (content: [lineptr_offset*2, +128); border:
// the complement), so they never clobber each other. Prefetches read the full
// row — reads are harmless.
//
// Gate: prev-FB in butter PSRAM + geometry where border/content segments line
// up on even offsets (all 4:3 and fullborder modes; 16:9 Pentagon falls back
// to the direct cached-XIP path). Cache coherence across gate transitions is
// handled with xip_cache_clean_range / invalidate_range.
#if !PICO_RP2040

#define PW_ROW_MAX 184  // largest prev row: 360 px / 2 = 180 B, rounded up
// Cached XIP (0x1x......) → uncached, non-allocating alias (0x1(x+4)......)
#define PW_UNCACHED (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE)

struct PrevWin {
    uint8_t buf[2][PW_ROW_MAX] __attribute__((aligned(4)));
    int     row[2];             // prev-FB row each buffer holds (-1 = invalid)
    int     cur;                // buffer the CPU currently owns
    bool    isBorder;
};
static PrevWin pwCont = { {}, {-1, -1}, 0, false };
static PrevWin pwBrd  = { {}, {-1, -1}, 0, true  };
static bool     pw_gate = false;
static bool     pw_failed = false;      // self-test failed — window disabled for good
static uint32_t pw_geom_sig = 0;

static inline bool pwPrevInButter() {
    uintptr_t p = (uintptr_t)sharedFB_prev;
    return p >= (XIP_BASE + 0x01000000u) && p < XIP_NOCACHE_NOALLOC_BASE;
}

static inline uint8_t* pwUncachedRow(int row) {
    return sharedFB_prev + (size_t)row * (sharedFB_stride / 2) + PW_UNCACHED;
}

// Byte segments of a prev row owned by a stream (writeback scope). Boundaries
// are even but not always word-aligned (Pentagon fullborder: 26/154): the
// copy runs words over the aligned interior and halfwords at the ragged edges.
static int __not_in_flash_func(pwSegs)(bool isBorder, int row, int* off, int* len) {
    if (!isBorder) { off[0] = (int)lineptr_offset * 2; len[0] = 128; return 1; }
    if (row >= (int)lin_end && row < (int)lin_end2) {
        off[0] = 0;                len[0] = brdcol_end1;
        off[1] = brdcol_end1 + 128; len[1] = brdcol_end - off[1];
        return (len[1] > 0) ? 2 : 1;
    }
    off[0] = 0; len[0] = brdcol_end;  // top/bottom border rows: full width
    return 1;
}

// Stage buffer b to/from PSRAM: writeback wbRow's owned segments (if >= 0),
// then prefetch rdRow's owned segments (if >= 0). Synchronous CPU copies
// through the uncached alias — see the transport note above.
static void __not_in_flash_func(pwKick)(PrevWin& w, int b, int wbRow, int rdRow) {
    if (wbRow >= 0) {
        int off[2], len[2];
        int nw = pwSegs(w.isBorder, wbRow, off, len);
        uint8_t* rowU = pwUncachedRow(wbRow);
        for (int s = 0; s < nw; s++) {
            int a = off[s], e = off[s] + len[s];
            if (e <= a) continue;
            if (a & 2) { *(uint16_t*)(rowU + a) = *(uint16_t*)(w.buf[b] + a); a += 2; }
            if ((e & 2) && e > a) { e -= 2; *(uint16_t*)(rowU + e) = *(uint16_t*)(w.buf[b] + e); }
            uint32_t*       d   = (uint32_t*)(rowU + a);
            const uint32_t* s32 = (const uint32_t*)(w.buf[b] + a);
            for (int n = (e - a) >> 2; n > 0; n--) *d++ = *s32++;
        }
    }
    if (rdRow >= 0) {
        // Prefetch only the owned segments — uncached reads are the expensive
        // half of the staging cost (writes are posted). Reads round outward to
        // word boundaries: overreading a couple of foreign bytes is harmless
        // (they are never consumed nor written back).
        int off[2], len[2];
        int nr = pwSegs(w.isBorder, rdRow, off, len);
        const uint8_t* rowU = pwUncachedRow(rdRow);
        for (int s = 0; s < nr; s++) {
            if (len[s] <= 0) continue;
            int a = off[s] & ~3;
            int e = (off[s] + len[s] + 3) & ~3;
            const uint32_t* s32 = (const uint32_t*)(rowU + a);
            uint32_t*       d   = (uint32_t*)(w.buf[b] + a);
            for (int n = (e - a) >> 2; n > 0; n--) *d++ = *s32++;
        }
    }
}

// Per-row entry point: returns the SRAM buffer holding prev row `row`
// (full-row layout, so existing pointer math applies unchanged).
static uint8_t* __not_in_flash_func(pwFetch)(PrevWin& w, int row) {
    if (row == w.row[w.cur]) return w.buf[w.cur];
    const int maxRow = w.isBorder ? (int)VIDEO::vga.yres : (int)lin_end2;
    int other = w.cur ^ 1;
    if (w.row[other] == row) {
        // Advance: flush the old current row, prefetch row+1 behind it.
        int old = w.cur, wbRow = w.row[old];
        w.cur = other;
        int nxt = (row + 1 < maxRow) ? row + 1 : -1;
        w.row[old] = nxt;
        pwKick(w, old, wbRow, nxt);
        return w.buf[w.cur];
    }
    // Resync (frame start / skipped frames): flush + fetch.
    pwKick(w, w.cur, w.row[w.cur], row);
    w.row[w.cur] = row;
    int nxt = row + 1;
    if (nxt < maxRow) { pwKick(w, other, -1, nxt); w.row[other] = nxt; }
    else w.row[other] = -1;
    return w.buf[w.cur];
}

// Discard all buffered rows WITHOUT writeback — for callers about to rewrite
// or free the whole prev-FB (InitPrevBuffer, geometry change, shutdown).
static void pwDrop() {
    pwCont.row[0] = pwCont.row[1] = -1;
    pwBrd.row[0]  = pwBrd.row[1]  = -1;
}

// Disable the window and restore the cached-access regime (before freeing
// prev-FB or when the gate closes). Must run while sharedFB_prev is valid.
static void pwShutdown() {
    pwDrop();
    if (pw_gate && sharedFB_prev)
        xip_cache_invalidate_range((uintptr_t)sharedFB_prev - XIP_BASE, sharedFB_prev_size);
    pw_gate = false;
}

static bool pwSelfTest();  // one-shot data-integrity probe, defined below

// Re-evaluate the gate. Called from Select_Update_Border() — i.e. on every
// geometry/arch change and once per frame (cheap compare on the steady path).
static void pwRefreshGate() {
    const int W = sharedFB_stride / 2;
    bool want = sharedFB_prev != nullptr
        && !pw_failed
        && !sharedFB_prev_nchunks          // chunked: not one region, no row arithmetic
        && ((uintptr_t)sharedFB_prev & 3) == 0
        && pwPrevInButter()
        && !ds80_border_geom
        && brdcol_start == 0
        && (brdcol_end1 & 1) == 0
        && (int)lineptr_offset * 2 == brdcol_end1
        && brdcol_retrace == brdcol_end
        && brdcol_end == W
        && W > 0 && W <= PW_ROW_MAX && (W & 3) == 0;
    uint32_t sig = (uint32_t)brdcol_end1 | ((uint32_t)brdcol_end << 8)
                 | ((uint32_t)lin_end << 16) | ((uint32_t)lin_end2 << 24);
    if (want == pw_gate) {
        if (pw_gate && sig != pw_geom_sig) { pwDrop(); pw_geom_sig = sig; }
        return;
    }
    if (want) {
        pwDrop();
        pw_geom_sig = sig;
        // Push any dirty prev-FB lines (alloc-time memset, pre-gate CPU writes)
        // out of the XIP cache so the uncached-alias copies see current data.
        xip_cache_clean_range((uintptr_t)sharedFB_prev - XIP_BASE, sharedFB_prev_size);
        pwCont.cur = 0; pwBrd.cur = 0;
        bool st_ok = pwSelfTest();
        pwDrop();
        if (!st_ok) {
            pw_failed = true;
            Debug::log("VIDEO: prevFB window self-test FAILED — disabled");
            return;
        }
        pw_gate = true;
        Debug::log("VIDEO: prevFB window ON (W=%d content=[%d,%d))", W, brdcol_end1, brdcol_end1 + 128);
    } else {
        pwShutdown();
        Debug::log("VIDEO: prevFB window OFF");
    }
}

static inline bool pwOn() { return pw_gate && VIDEO::gigascreen_enabled; }

// ── Self-test ────────────────────────────────────────────────────────────────
// Runs once before the window is allowed into the render path; drives the
// production pwKick machinery and verifies data integrity through the uncached
// alias. Each stage logs BEFORE executing so a wedge is identifiable from UART.
static bool pwSelfTest() {
    const int W = sharedFB_stride / 2;
    uint8_t* unc = pwUncachedRow(0);
    const int coff = (int)lineptr_offset * 2;
    const int mid = (int)lin_end;      // first middle row (2-segment border shape)
    Debug::log("PW st: prev=%p unc=%p W=%d cont=[%d,%d) end1=%d mid=%d",
               sharedFB_prev, unc, W, coff, coff + 128, brdcol_end1, mid);
    Debug::log("PW st0: cpu uncached read");
    volatile uint32_t* u32 = (volatile uint32_t*)unc;
    uint32_t v = u32[0];
    Debug::log("PW st0: ok val=%08x cached=%08x", (unsigned)v, (unsigned)*(uint32_t*)sharedFB_prev);
    Debug::log("PW st1: cpu uncached write");
    u32[0] = v;                       // same value — harmless wherever it lands
    uint32_t v2 = u32[0];
    Debug::log("PW st1: ok readback=%08x", (unsigned)v2);
    // Prefetch row 0 and verify against the cached view (coherent post-clean).
    Debug::log("PW st2: prefetch row0 + verify");
    pwKick(pwCont, 0, -1, 0);
    // Content-stream prefetch stages only the owned segment — compare that.
    bool match = memcmp((void*)(pwCont.buf[0] + coff), sharedFB_prev + coff, 128) == 0;
    Debug::log("PW st2: done match=%d", (int)match);
    if (!match) {
        Debug::log("PW st2: buf=%02x%02x%02x%02x prev=%02x%02x%02x%02x",
                   pwCont.buf[0][coff], pwCont.buf[0][coff + 1], pwCont.buf[0][coff + 2], pwCont.buf[0][coff + 3],
                   sharedFB_prev[coff], sharedFB_prev[coff + 1], sharedFB_prev[coff + 2], sharedFB_prev[coff + 3]);
        return false;
    }
    // Advance shape: writeback row0 (its own data) + prefetch row1.
    Debug::log("PW st3: advance (wb0+rd1)");
    pwKick(pwCont, 0, 0, 1);
    // Border shape on a middle row: prefetch, then 2-segment writeback + rd.
    Debug::log("PW st4: border wb+rd (mid row)");
    pwKick(pwBrd, 0, -1, mid);
    pwKick(pwBrd, 0, mid, mid + 1);
    // 64-row sweep of production advance kicks (runtime cadence), then verify
    // a row survived the round-trip intact.
    Debug::log("PW st5: 64-row sweep");
    pwKick(pwCont, 0, -1, 0);
    pwKick(pwCont, 1, -1, 1);
    for (int r = 0; r + 2 < 64; r++)
        pwKick(pwCont, r & 1, r, r + 2);   // buf holds row r → clean writeback
    pwKick(pwCont, 0, -1, 5);              // re-read a swept row
    match = memcmp((void*)(pwCont.buf[0] + coff), sharedFB_prev + (size_t)5 * W + coff, 128) == 0;
    Debug::log("PW st5: done match=%d", (int)match);
    if (!match) return false;
    return true;
}

#endif // !PICO_RP2040

// GsSubsys storage and apply() — implemented here so it can touch the
// sharedFB_* internals directly. Declared in Subsystem.h.
volatile bool GsSubsys::enabled = false;
bool GsSubsys::wanted = false;
bool GsSubsys::dirty = false;

void GsSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool GsSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        // Size prev FB to the *current* mode (sharedFB_lines × sharedFB_stride),
        // not the build-time max. Saves SRAM at lower resolutions.
        if (!ensurePrevFB(sharedFB_lines, sharedFB_stride)) {
            wanted = false;
            Config::gigascreen_enabled = false;
            VIDEO::gigascreen_enabled = false;
            return false;
        }
        if (!sharedFB_arr2) {
            sharedFB_arr2 = (void**)malloc(FB_MAX_LINES * sizeof(void*));
            if (!sharedFB_arr2) {
                prevFBFree();
                wanted = false;
                Config::gigascreen_enabled = false;
                VIDEO::gigascreen_enabled = false;
                return false;
            }
        }
        // Re-derive prev pointer array from the current resolution.
        int prev_stride = sharedFB_stride / 2;
        for (int i = 0; i < sharedFB_lines; i++) {
            sharedFB_arr2[i] = prevRowPtr(i, prev_stride);
        }
        VIDEO::vga.prevFrameBuffer = (unsigned char**)sharedFB_arr2;
        enabled = true;
    } else {
        // Drop pointer first so render path stops accessing prevFrameBuffer.
        // Producers (renderers) check vga.prevFrameBuffer != nullptr before use
        // (see MainScreen_Snow*); border path already had a null-guard.
        VIDEO::vga.prevFrameBuffer = nullptr;
        enabled = false;
        pwShutdown();  // wait out window DMA before the backing store goes away
        free(sharedFB_arr2);  sharedFB_arr2  = nullptr;
        prevFBFree();
    }
    return true;
}

// ── Lend the dormant Gigascreen prev-FB to the network buffer pool ────────────
// During a network session the emulator is paused, so the renderer never reads
// prevFrameBuffer — its ~52 KB of SRAM can back the TLS/socket working set that
// would otherwise OOM the heap on a butter-less board. We borrow the region
// in place (no free/realloc → no fragmentation, unlike past attempts) and detach
// vga.prevFrameBuffer so even a stray render falls back to the no-blend path.
static bool s_prev_lent = false;

bool VIDEO::gigascreenLendRegion(void*& base, size_t& size) {
    // Only worth it where prevFB exists AND there's no butter PSRAM to absorb the
    // TLS working set instead. On butter boards palloc already routes it to XIP.
    if (s_prev_lent) return false;
    if (!Config::gigascreen_enabled || !sharedFB_prev || sharedFB_prev_size == 0) return false;
    // Chunked prev-FB is not one region — the borrower gets base+size, so decline
    // rather than hand out a span that ends inside somebody else's allocation.
    if (sharedFB_prev_nchunks) return false;
    if (butter_psram_size() != 0) return false;
    base = sharedFB_prev;
    size = sharedFB_prev_size;
    VIDEO::vga.prevFrameBuffer = nullptr;   // renderer → no-blend branch while lent
    s_prev_lent = true;
    return true;
}

void VIDEO::gigascreenReclaimRegion() {
    if (!s_prev_lent) return;
    s_prev_lent = false;
    // Clear stale network data so the next Gigascreen frame doesn't blend garbage,
    // then re-attach the prev pointer array.
    if (sharedFB_prev && sharedFB_prev_size) memset(sharedFB_prev, 0, sharedFB_prev_size);
    if (sharedFB_arr2) VIDEO::vga.prevFrameBuffer = (unsigned char**)sharedFB_arr2;
}

// ── Chunked prev-FB: hand the whole buffer back for a network session ─────────
// A chunked prev-FB is not one region, so gigascreenLendRegion declines it — and
// with a small GIGASCREEN_PREVFB_HEADROOM that would leave a TLS session (16 KB
// input buffer + 4 KB output + alt stack) with only the bare headroom, which is the
// OOM the lease exists to prevent. So give the buffer up entirely instead: the
// emulator is paused, nothing reads prev-FB, the heap gets all ~38 KB back, and the
// chunks are normally consecutive so freeing them coalesces into large runs.
// Rebuilt on restore; if the heap has fragmented past hope by then, Gigascreen goes
// quiet until a reboot rather than taking the config down with it.
static bool s_prev_released_for_net = false;

bool VIDEO::gigascreenReleaseForNet() {
    if (s_prev_released_for_net || s_prev_lent) return false;
    if (!Config::gigascreen_enabled || !sharedFB_prev) return false;
    if (!sharedFB_prev_nchunks) return false;    // single block: lending is cheaper
    if (butter_psram_size() != 0) return false;  // lives in XIP, the heap doesn't care
    GsSubsys::request(false);
    GsSubsys::apply();                           // detaches, frees the chunks + arr2
    // Renderers do guard on prevFrameBuffer, but RedrawPausedFrame() runs while the
    // OSD is up — don't leave the live flag claiming Gigascreen with no buffer.
    VIDEO::gigascreen_enabled = false;           // restore puts it back from _onoff
    VIDEO::gigascreen_auto_countdown = 0;
    s_prev_released_for_net = true;
    Debug::log("VIDEO: prevFB (chunked) released for network session, freeHeap=%u",
               (unsigned)getFreeHeap());
    return true;
}

void VIDEO::gigascreenRestoreAfterNet() {
    if (!s_prev_released_for_net) return;
    s_prev_released_for_net = false;
    // apply()'s own OOM path clears Config::gigascreen_enabled; the user's choice is
    // not ours to drop over a transient heap state, so put it back and let the live
    // flags carry the "off for now" state (every reader pairs them with a
    // prevFrameBuffer null check).
    const bool cfg_on = Config::gigascreen_enabled;
    GsSubsys::request(true);
    if (!GsSubsys::apply() || !VIDEO::vga.prevFrameBuffer) {
        Config::gigascreen_enabled = cfg_on;
        VIDEO::gigascreen_enabled  = false;
        VIDEO::gigascreen_auto_countdown = 0;
        Debug::log("VIDEO: prevFB not recoverable after network session (freeHeap=%u)"
                   " — Gigascreen off until reboot", (unsigned)getFreeHeap());
        return;
    }
    VIDEO::InitPrevBuffer();                     // reseed from the live frame
    VIDEO::gigascreen_enabled = (Config::gigascreen_onoff == 1);  // On=live, Auto=armed
    VIDEO::gigascreen_auto_countdown = 0;
}

size_t VIDEO::gigascreenPrevFBBytes() {
    // Cost of the prev-FB in the *current* video mode. sharedFB_lines/stride are
    // set at Init() regardless of whether Gigascreen is on, so this is the size it
    // would claim if enabled. ~38 KB @640x480, ~52 KB @720x576.
    return fbPrevBytes(sharedFB_lines, sharedFB_stride);
}

void VIDEO::disableGigascreenForProfi() {
    if (Config::arch != "Profi") return; // only Profi is incompatible
    // Clear the persisted/live enable flags so nothing re-arms it (Auto mode
    // checks gigascreen_onoff; force it Off too) and free the prev-FB.
    Config::gigascreen_enabled = false;
    VIDEO::gigascreen_enabled  = false;
    Config::gigascreen_onoff   = 0; // Off — also disarms Auto countdown
    VIDEO::gigascreen_auto_countdown = 0;
    GsSubsys::request(false);
    GsSubsys::apply();
}
#else
void VIDEO::disableGigascreenForProfi() {} // RP2040: Gigascreen never allocated, Profi hidden
#endif

// Row accessors used by the render hot paths. Window active → SRAM buffer;
// otherwise the direct prev-FB pointer (heap SRAM, or cached XIP fallback).
// Callers already guard on vga.prevFrameBuffer != nullptr.
static inline uint16_t* prevRowContent(int row) {
#if !PICO_RP2040
    if (pwOn()) return (uint16_t*)pwFetch(pwCont, row);
#endif
    return (uint16_t*)(VIDEO::vga.prevFrameBuffer[row]);
}

static inline uint8_t* prevRowBorder(int row) {
#if !PICO_RP2040
    if (pwOn()) return pwFetch(pwBrd, row);
#endif
    return (uint8_t*)(VIDEO::vga.prevFrameBuffer[row]);
}

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
    // Allocate the main framebuffer block sized for the *initial* mode BEFORE
    // vga.init() — while heap is still unfragmented. It can be realloc'd later
    // on changeMode (see ensureMainFB). The prev block (Gigascreen) is allocated
    // on demand by GsSubsys, also sized for the current mode.
    int initLines = fbCalcLines(
        vidmodes[Mode][vmodeproperties::vRes] / vidmodes[Mode][vmodeproperties::vDiv]);
    int initStride = (vidmodes[Mode][vmodeproperties::hRes] + 3) & ~3;
    if (!sharedFB_arr1) {
        sharedFB_arr1 = (void **)malloc(FB_MAX_LINES * sizeof(void *));
    }
    if (sharedFB_arr1 && ensureMainFB(initLines, initStride)) {
        setupSharedFBPointers(vga, initLines, initStride);
        // frameBuffer is set — vga.init()'s allocateFrameBuffers() will skip allocation
    } else {
        // Out of memory for shared FB — fall back to legacy allocator path.
        free(sharedFB_arr1); sharedFB_arr1 = nullptr;
        free(sharedFB_main); sharedFB_main = nullptr; sharedFB_main_size = 0;
    }
    // If we boot into Profi, Gigascreen is incompatible: clear the flags so the
    // prev-FB is never allocated below (and any stale NVS enable is dropped).
    disableGigascreenForProfi();
    // Pre-allocate prev framebuffer if Gigascreen is enabled at boot,
    // BEFORE the heap fragments.
    if (sharedFB_main && Config::gigascreen_enabled) {
        GsSubsys::request(true);
        GsSubsys::apply();
    }
#endif

    vga.init( Mode, redPins, grePins, bluPins, HSYNC_PIN, VSYNC_PIN);

    graphics_set_scanlines(Config::scanlines);

    // Generate AluBytes table with palette indices (no sync bits)
    initAluBytes();

#if !PICO_RP2040
    // 16col byte->2-pixel LUT: build it only when the mode is enabled, release
    // it otherwise so a disabled 16col reserves no SRAM.
    if (Config::mode16col_onoff) VIDEO::ensure16colLut();
    else VIDEO::free16colLut();
#endif

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
    // Shared block path: realloc to fit the new mode (saves SRAM at smaller
    // resolutions; grows on the way up). Pointer arrays rebuilt afterwards.
    if (sharedFB_main) {
        if (!sameDims) {
            // Runtime resolution change is not supported — mode-switch callers
            // savePendingVideoMode() then esp_hard_reset(). If we got here with
            // a dim change anyway, refuse: heap fragmentation has no in-place
            // remedy and a NULL frameBuffer would SIGBUS-storm the renderer.
            Debug::log("changeMode: ignored runtime dim change %dx%d -> %dx%d",
                       (int)vga.xres, (int)vga.yres, newW, newH);
            return;
        }
    } else
#endif
    {
        // Non-shared fallback (RP2040 always; RP2350 only if shared alloc failed).
        // prevFrameBuffer is RP2350-only (Gigascreen) — guard the cleanup.
#if !PICO_RP2040
        // freeFrameBuffer frees fb[0] as "the one data block", which is only true for
        // an allocateFrameBuffer() array — the shared array's row 0 belongs to
        // sharedFB_prevBuf/prevChunk[0] and is owned by GsSubsys. Reaching here with
        // it installed would be a double free (this branch means the shared scheme was
        // never used, so it cannot happen; the check keeps it that way).
        if (vga.prevFrameBuffer && (void**)vga.prevFrameBuffer != sharedFB_arr2) {
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
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    if (SELECT_VGA) {
        switch (Config::vga_video_mode) {
            case Config::VM_640x480_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 2;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 3;
                else video_mode = 1;
                break;
            case Config::VM_720x480_60: video_mode = 7; break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 5;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 6;
                else video_mode = 4;
                break;
            default: video_mode = 0; break;
        }
    } else
#endif
    {
        switch (Config::hdmi_video_mode) {
            case Config::VM_640x480_60: video_mode = 0; break;
            case Config::VM_640x480_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 2;
                else if (Config::arch == "128K") video_mode = 3;
                else video_mode = 1;
                break;
            case Config::VM_720x480_60: video_mode = 7; break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 5;
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
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    if (SELECT_VGA) {
        vga_reinit();
    } else 
#endif
    {
        hdmi_reinit();
    }

    // 4. Allocate framebuffer (non-shared path only)
#if !PICO_RP2040
    if (!sharedFB_main) {
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
    updateBorderBrd();
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

    // Reset 16col state
    mode16col_enabled = false;
    mode16colUpdatePlanes();
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
    } else if (Config::arch == "Profi") {
        tStatesPerLine = TSTATES_PER_LINE_PROFI;
        tStatesScreen = TS_SCREEN_PROFI;
        tStatesBorder = isFullBorder ? (isFullBorder240 ? TS_BORDER_360x240_PROFI : TS_BORDER_360x288_PROFI)
                      : is169 ? TS_BORDER_360x200_PROFI : TS_BORDER_320x240_PROFI;
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
#if !PICO_RP2040
    // Clear stale DS80 border geometry first (re-applied below if DS80 active) —
    // Select_Update_Border() would otherwise keep the DS80 updater on arch switch.
    ds80_border_geom = false;
    ds80_brd_col_off = 0;
#endif
    brdcol_end = isFullBorder ? 180 : 160;  // vga.xres / 2 (T-states = half pixel count)
    if ((Z80Ops::isPentagon || Z80Ops::isProfi)) {
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
        lineptr_offset = ((Z80Ops::isPentagon || Z80Ops::isProfi) ? 26 : 24) / 2;
    } else if (isFullBorder && isFullBorder240) {
        // Profi centred like Pentagon (24 top / 24 bottom border): using 32/224
        // shifted the picture down 1 char row and squeezed the bottom border so
        // the stats overlay (y=220) fell inside the paper area → flicker.
        lin_end = 24;
        lin_end2 = 216;
        lineptr_offset = ((Z80Ops::isPentagon || Z80Ops::isProfi) ? 26 : 24) / 2;
    } else if (is169) {
        lin_end = 4;
        lin_end2 = 196;
        lineptr_offset = 13;
    } else {
        // Profi centred like Pentagon (24 top / 24 bottom border): using 32/224
        // shifted the picture down 1 char row and squeezed the bottom border so
        // the stats overlay (y=220) fell inside the paper area → flicker.
        lin_end = 24;
        lin_end2 = 216;
        lineptr_offset = 8;  // 32 bytes = (320-256)/2 pixels
    }

    if (Config::arch == "Profi" && (Ports::portDFFD & 0x80)) {
        grmem = MemESP::videoLatch ? MemESP::ram[6].direct() : MemESP::ram[4].direct();
        uint32_t clrPage = MemESP::videoLatch ? 58 : 56;
        extern int ram_pages, butter_pages, psram_pages, swap_pages;
        int totPg = ram_pages + butter_pages + psram_pages + swap_pages;
        profi_clrmem = ((int)clrPage < totPg) ? MemESP::ram[clrPage].direct() : nullptr;
        Debug::log("[VID] DS80 Reset: clrPage=%u totPg=%d clrmem=%p grmem=%p", clrPage, totPg, profi_clrmem, grmem);
#if !PICO_RP2040
        // DS80: 512×240 content + per-T-state side borders, ZXMAK2 timing
        // (192 T/line).  The grmem layout (pixCoff formula) covers all 240
        // lines; lines 192..239 live at offsets 6144..8191 (odd) and
        // 14336..16383 (even) of the 16KB page.
        applyDS80BorderGeometry(true);
#endif
    } else {
        grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
        profi_clrmem = nullptr;
    }

#if !PICO_RP2040
    if (Config::arch == "Profi") {
        // Build pair_lookup every reset (palette may change). Cheap — 16×16 = 256 iters.
        init_profi_pair_lookup();
        // Reset live palette to defaults on machine reset
        profiPaletteReset();
        // DS80 clrmem snapshot: needed only when pages 56/58 live in XIP butter
        // PSRAM (butter boards keep the whole Profi RAM as direct XIP pointers —
        // per A/B measurement the SRAM pool gave no cpu gain there).  A direct
        // per-scanline read would hit the XIP cache from the render path; the
        // snapshot converts that into one sequential burst per frame.
        // On SPI-PSRAM/SWAP boards pages 56/58 are force_sram_locked heap SRAM,
        // so ds80_frame_clrmem is already a plain SRAM pointer — no snapshot.
        if (!ds80_clr_sram && butter_psram_size() > 0)
            ds80_clr_sram = (uint8_t*)malloc(DS80_CLR_SRAM_SIZE);
    } else if (ds80_clr_sram) {
        free(ds80_clr_sram);
        ds80_clr_sram = nullptr;
    }
#endif

    #ifdef DIRTY_LINES
    // for (int i=0; i < SPEC_H; i++) VIDEO::dirty_lines[i] = 0x01;
    memset((uint8_t *)VIDEO::dirty_lines,0x01,SPEC_H);
    #endif // DIRTY_LINES

    VIDEO::snow_toggle = (Config::arch != "P1024" && Config::arch != "P512" && Config::arch != "Pentagon" && Config::arch != "Profi") ? Config::render : false;

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
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    if (SELECT_VGA)
    {
        switch (Config::vga_video_mode) {
            case Config::VM_640x480_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 2;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 3;
                else video_mode = 1; // Pentagon
                break;
            case Config::VM_720x480_60:
                video_mode = 7;
                break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 5;
                else if (Config::arch == "128K" || Config::arch == "ALF") video_mode = 6;
                else video_mode = 4; // Pentagon
                break;
            default: // VM_640x480_60
                video_mode = 0;
                break;
        }
    }
    else
#endif
    {
        // HDMI: map Config enum to graphics.c video_mode index
        switch (Config::hdmi_video_mode) {
            case Config::VM_640x480_60:
                video_mode = 0;
                break;
            case Config::VM_640x480_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 2;
                else if (Config::arch == "128K") video_mode = 3;
                else video_mode = 1; // Pentagon
                break;
            case Config::VM_720x480_60:
                video_mode = 7;
                break;
            case Config::VM_720x576_50:
                if (Config::arch == "48K" || Config::arch == "Profi") video_mode = 5;
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

#if !PICO_RP2040
    // DS80 transition on reset: ensure DS80 state is fully cleaned up whenever
    // we're entering non-DS80 mode (e.g. after F11 reset from service menu).
    //
    // Two cases that must both be handled:
    //   A) profi_ds80_active=true  → HDMI is in DS80 mode, must deactivate.
    //   B) profi_ds80_active=false AND profi_ds80_activate_pending=true →
    //      DS80 was requested (port write) but EndFrame hasn't processed it yet.
    //      Without clearing activate_pending here, the very next EndFrame after
    //      reset fires the activate handler → DS80 HDMI active on a machine now
    //      running standard Profi code → standard palette bytes interpreted as
    //      DS80 packed-pairs → fine vertical colour stripes on every pixel pair.
    {
        bool ds80_should_be_active = (Config::arch == "Profi") && (Ports::portDFFD & 0x80);
        Debug::log("[VRESET] portDFFD=0x%02X ds80=%d act=%d deact=%d should=%d",
            (int)Ports::portDFFD, (int)profi_ds80_active,
            (int)profi_ds80_activate_pending, (int)profi_ds80_deactivate_pending,
            (int)ds80_should_be_active);
        if (!ds80_should_be_active) {
            // Always kill any pending activation — prevents case B above.
            profi_ds80_activate_pending = false;
            if (profi_ds80_active) {
                // Case A: schedule deactivation; pre-fill with 0xFF so DS80 HDMI
                // shows clean black for the one frame until EndFrame processes it.
                // (0xFF = slot 255 = black/black in DS80 mode per hdmi.c line 810-811)
                profi_ds80_deactivate_pending = true;
                if (vga.frameBuffer) {
                    for (int _y = 0; _y < (int)vga.yres; _y++)
                        if (vga.frameBuffer[_y]) memset(vga.frameBuffer[_y], 0xFF, vga.xres);
                }
            }
        }
    }
#endif
}

extern size_t getFreeHeap(void);
extern size_t getContiguousHeap(void);

#if !PICO_RP2040
void VIDEO::InitPrevBuffer() {
    if (!vga.prevFrameBuffer) {
        // Use the GsSubsys-managed prev buffer rather than a one-off
        // allocateFrameBuffer() — keeps a single owner for the 52 KB block.
        GsSubsys::request(true);
        GsSubsys::apply();
    }
    if (!vga.prevFrameBuffer) return;
    // Whole prev-FB is being reseeded: discard any rows buffered by the DMA
    // window (their pending writebacks would clobber the new seed).
    pwDrop();
    const int h = VIDEO::vga.yres;
    const int w = VIDEO::vga.xres;
    for (int y = 0; y < h; ++y) {
        uint8_t *src = VIDEO::vga.frameBuffer[y];
        uint8_t *dst = VIDEO::vga.prevFrameBuffer[y];
        if (!src || !dst) continue;
        // While the DMA window is active the prev-FB must never enter the XIP
        // cache — write through the uncached alias instead.
        if (pw_gate) dst += PW_UNCACHED;
        // Pack 2 src pixels (8-bit palette idx) into 1 dst byte (low+high nibble).
        if (((uintptr_t)dst & 3) == 0 && (w & 7) == 0) {
            // Word-packed fast path — matters for uncached XIP (4 B per store).
            uint32_t *dst32 = (uint32_t*)dst;
            for (int x = 0; x < w; x += 8) {
                uint32_t v =  (uint32_t)(src[x]     & 0x0F)
                           | ((uint32_t)(src[x + 1] & 0x0F) << 4)
                           | ((uint32_t)(src[x + 2] & 0x0F) << 8)
                           | ((uint32_t)(src[x + 3] & 0x0F) << 12)
                           | ((uint32_t)(src[x + 4] & 0x0F) << 16)
                           | ((uint32_t)(src[x + 5] & 0x0F) << 20)
                           | ((uint32_t)(src[x + 6] & 0x0F) << 24)
                           | ((uint32_t)(src[x + 7] & 0x0F) << 28);
                *dst32++ = v;
            }
        } else {
            for (int x = 0; x < w; x += 2) {
                dst[x >> 1] = (src[x] & 0x0F) | ((src[x + 1] & 0x0F) << 4);
            }
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
        prevLineptr16 = vga.prevFrameBuffer
                          ? prevRowContent(linedraw_cnt) + lineptr_offset
                          : (uint16_t *)lineptr32;

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
        // (dma_attr_valid/shadow are null unless the DMA attr buffer is allocated)
        if (Config::dma_mode && Z80DMA::dma_attr_valid && Z80DMA::dma_attr_valid[curline])
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
        prevLineptr16 = vga.prevFrameBuffer
                          ? prevRowContent(linedraw_cnt) + lineptr_offset
                          : (uint16_t *)lineptr32;

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

        if (Config::arch == "Profi" && (Ports::portDFFD & 0x80))
            snowpage = MemESP::videoLatch ? 6 : 4;
        else
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
        prevLineptr16 = vga.prevFrameBuffer
                          ? prevRowContent(linedraw_cnt) + lineptr_offset
                          : (uint16_t *)lineptr32;

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
    // One-shot. Table + palette slots 17..136 only depend on spectrum_rgb888[],
    // which never changes after boot. Re-entering this from an emulation context
    // (e.g. ULA+ disable -> palette restore) races with HDMI DMA reading conv_color
    // and produces palette tearing at the top of the screen.
    if (gigsBlendLUTReady) return;
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

// Blend 4 cur pixels with 4 prev pixels stored as packed uint16 (2 px per byte).
// Per-pixel: if low nibble of cur matches prev nibble, return cur byte unchanged
// (preserves ULA+ upper bits — palette indices 0..63 alias by low nibble in blendLUT).
inline uint32_t blendPixels32_packed(uint32_t cur, uint16_t prev16) {
    uint8_t c0 = cur & 0xFF, c1 = (cur >> 8) & 0xFF, c2 = (cur >> 16) & 0xFF, c3 = (cur >> 24) & 0xFF;
    uint8_t p0 = prev16 & 0x0F, p1 = (prev16 >> 4) & 0x0F, p2 = (prev16 >> 8) & 0x0F, p3 = (prev16 >> 12) & 0x0F;
    uint8_t r0 = ((c0 & 0x0F) == p0) ? c0 : gigsBlendLUT[p0 * 16 + (c0 & 0x0F)];
    uint8_t r1 = ((c1 & 0x0F) == p1) ? c1 : gigsBlendLUT[p1 * 16 + (c1 & 0x0F)];
    uint8_t r2 = ((c2 & 0x0F) == p2) ? c2 : gigsBlendLUT[p2 * 16 + (c2 & 0x0F)];
    uint8_t r3 = ((c3 & 0x0F) == p3) ? c3 : gigsBlendLUT[p3 * 16 + (c3 & 0x0F)];
    return r0 | (r1 << 8) | (r2 << 16) | (r3 << 24);
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
    } else if (VIDEO::mode16col_enabled && mode16col_decode_lut) {
        // 16col (Pentagon, Alone Coder, ZXPress Inferno #08).
        // Byte layout: %IiGRBgrb. mode16col_decode_lut[] precomputes
        // (left_pixel | right_pixel<<8) for every input byte.
        // Plane order per pixel-pair: A,B,C,D = #C000, #4000, #E000, #6000.
        // Frame buffer order is byte-swapped (^2) per AluByte convention so
        // HDMI scanout ISR reads pixels in correct visual order.
        const uint8_t* pA = VIDEO::mode16col_planes[0];
        const uint8_t* pB = VIDEO::mode16col_planes[1];
        const uint8_t* pC = VIDEO::mode16col_planes[2];
        const uint8_t* pD = VIDEO::mode16col_planes[3];
        const uint16_t* lut = mode16col_decode_lut;
        for (; loopCount--; ) {
            uint16_t off = bmpOffset++;
            uint32_t la = lut[pA[off]];
            uint32_t lb = lut[pB[off]];
            uint32_t lc = lut[pC[off]];
            uint32_t ld = lut[pD[off]];
            // pixels 0..3 = L(a), R(a), L(b), R(b) — written as bytes [2,3,0,1]
            *lineptr32++ = lb | (la << 16);
            // pixels 4..7 = L(c), R(c), L(d), R(d) — same swap
            *lineptr32++ = ld | (lc << 16);
        }
    } else
#endif
    if (VIDEO::gigascreen_enabled
#if !PICO_RP2040
        && !VIDEO::ulaplus_enabled
#endif
    ) {
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
#if !PICO_RP2040
    } else if (Config::arch == "Profi" && (Ports::portDFFD & 0x80)) {
        // Profi DS80 native rendering — writes 256-byte-wide packed pairs directly into
        // vga.frameBuffer (1 byte = pair of 4-bit palette indices: high nibble =
        // left source pixel, low nibble = right). HDMI ISR is configured (via
        // hdmi_set_profi_ds80_mode) so each fb byte expands to 2 *different*
        // HDMI pixels — native 512 horizontal resolution.
        //
        // Per ZXMAK2 ProfiRenderer (byte-column interleaved): for col j (0..31)
        //   even src byte-col 2j in second half (+0x2000), odd 2j+1 in first half (+0).
        // Attr (4-bit): ink = bits 2..0 | (bit6 << 3); paper = bits 5..3 | (bit7 << 3).
        uint32_t line = curline;
        uint32_t pixCoff = 2048 * (line >> 6) + 256 * (line & 7) + ((line & 0x38) << 2);
        unsigned int end_col = coldraw_cnt < 32u ? coldraw_cnt : 32u;
        unsigned int start_col = end_col - loopCount;
        // Latch the display pages ONCE at the start of the frame (UnrealSpeccy rend_profi
        // model): all 240 lines render from the page selected at line 0, so mid-frame
        // videoLatch flips don't tear the current frame.  Also (re)latch if null (first
        // frame after activation).
        // grmem (pages 4/6) is already in SRAM — left as-is.
        if ((line == 0 && start_col == 0) || ds80_frame_grmem == nullptr) {
            ds80_frame_grmem  = grmem;
            ds80_frame_clrmem = profi_clrmem;
            // Refresh snapshot at the START of each active scan so Z80 writes made
            // since vblank (e.g. ROM service-menu attr init over multiple frames) are
            // captured immediately.  One sequential burst here; renderer reads SRAM
            // for all remaining lines without scattered XIP stalls.
            if (profi_clrmem && ds80_clr_sram)
                memcpy(ds80_clr_sram, profi_clrmem, DS80_CLR_SRAM_SIZE);
        }
        uint8_t* fgrmem  = ds80_frame_grmem;
        // ds80_clr_sram is only allocated on butter-PSRAM boards (XIP cache stall risk);
        // on SPI-PSRAM/SWAP boards it is null and ds80_frame_clrmem (force_sram_locked
        // SRAM pointer) is used directly — no stall risk, saves 16 KB heap.
        uint8_t* fclrmem = ds80_frame_clrmem ? (ds80_clr_sram ? ds80_clr_sram : ds80_frame_clrmem) : nullptr;
        //
        // Row layout: pad_l bytes of border, then 256 content bytes (with (k^2) pre-swap
        // for the ISR's x^2 read pattern), then pad_r bytes of border.
        // Vertical: 640×480 (yres=240) shows content full-height (voff=0, no top/bottom
        // border). 720×576 (yres=288) centres the 240 content lines with a symmetric
        // 24-row top/bottom border band (voff=DS80_BORDER_TOP) — top/bottom rows are
        // filled in EndFrame; here we offset content + paint the per-line side borders.
        const int pad_l = vga.frameBuffer ? ((int)vga.xres - 256) / 2 : 32;
        const int ds80_voff = (vga.yres >= 288) ? DS80_BORDER_TOP : 0;
        const uint32_t frow = line + (uint32_t)ds80_voff;   // framebuffer row for this content line
        uint8_t* fb_row = (vga.frameBuffer && frow < (uint32_t)vga.yres)
                          ? (uint8_t*)vga.frameBuffer[frow] : nullptr;
        // F8 stats overlay (DS80 640×480 only): drawStats writes the cached lines into
        // fb rows 220..235 every frame (in vblank).  The content renderer + side-border
        // fill would overwrite them during the next active scan → 1-frame content /
        // 1-frame stats → flicker.  The stats rectangle (fb bytes 168..311) straddles
        // the content area AND the right border pad (288..319), so BOTH the content
        // write and the right-pad fill must carve it out, leaving it for drawStats.
        // (720×576 keeps stats in the bottom border band, carved out in Update_Border_DS80.)
        bool skip_stats_row = (ds80_voff == 0) && (VIDEO::OSD & 0x03) && !(VIDEO::OSD & 0x04)
                              && frow >= 220 && frow < 236;
        // Side borders are rendered per-T-state by the border state machine
        // (Update_Border_DS80, ZXMAK2 192T-line timing) — no per-line fill here.
        for (unsigned int j = start_col; j < end_col; j++) {
            uint8_t bmpEven = fgrmem[pixCoff + j + 8192];
            uint8_t bmpOdd  = fgrmem[pixCoff + j];
            uint8_t rawE = fclrmem ? fclrmem[pixCoff + j + 8192] : 0x07;
            uint8_t rawO = fclrmem ? fclrmem[pixCoff + j]        : 0x07;
            uint8_t inkE = (rawE & 0x07) | ((rawE & 0x40) >> 3);
            uint8_t papE = ((rawE & 0x38) >> 3) | ((rawE & 0x80) >> 4);
            uint8_t inkO = (rawO & 0x07) | ((rawO & 0x40) >> 3);
            uint8_t papO = ((rawO & 0x38) >> 3) | ((rawO & 0x80) >> 4);
            // Stats overlay carve-out: skip the 8-byte column block if it overlaps the
            // stats rectangle (fb bytes 168..311) on a stats row — drawStats owns it.
            size_t pbase = (size_t)pad_l + (size_t)j * 8;
            bool in_stats = skip_stats_row && (pbase + 8 > 168) && (pbase < 312);
            if (fb_row && !in_stats) {
                // 16 source pixels per col → 8 packed bytes at content offset pad_l + j*8.
                // Pre-apply (k^2) swap so ISR reads in correct order.
                // Physical byte layout after k^2: [k=2,k=3,k=0,k=1] at pbase+[0,1,2,3].
                // Pack into two aligned uint32_t writes (4× fewer SRAM transactions than
                // 8 individual byte writes, reducing core0 bus contention with video DMA).
                // pbase is always 4-byte aligned (pad_l divisible by 4, j*8 divisible by 8).
                uint8_t e0 = profi_pair_lookup[((bmpEven>>3)&1)?inkE:papE][((bmpEven>>2)&1)?inkE:papE]; // k=2 → +0
                uint8_t e1 = profi_pair_lookup[((bmpEven>>1)&1)?inkE:papE][((bmpEven>>0)&1)?inkE:papE]; // k=3 → +1
                uint8_t e2 = profi_pair_lookup[((bmpEven>>7)&1)?inkE:papE][((bmpEven>>6)&1)?inkE:papE]; // k=0 → +2
                uint8_t e3 = profi_pair_lookup[((bmpEven>>5)&1)?inkE:papE][((bmpEven>>4)&1)?inkE:papE]; // k=1 → +3
                *(uint32_t*)(fb_row + pbase) = (uint32_t)e0 | ((uint32_t)e1<<8) | ((uint32_t)e2<<16) | ((uint32_t)e3<<24);
                uint8_t o0 = profi_pair_lookup[((bmpOdd>>3)&1)?inkO:papO][((bmpOdd>>2)&1)?inkO:papO]; // k=2 → +4
                uint8_t o1 = profi_pair_lookup[((bmpOdd>>1)&1)?inkO:papO][((bmpOdd>>0)&1)?inkO:papO]; // k=3 → +5
                uint8_t o2 = profi_pair_lookup[((bmpOdd>>7)&1)?inkO:papO][((bmpOdd>>6)&1)?inkO:papO]; // k=0 → +6
                uint8_t o3 = profi_pair_lookup[((bmpOdd>>5)&1)?inkO:papO][((bmpOdd>>4)&1)?inkO:papO]; // k=1 → +7
                *(uint32_t*)(fb_row + pbase + 4) = (uint32_t)o0 | ((uint32_t)o1<<8) | ((uint32_t)o2<<16) | ((uint32_t)o3<<24);
            }
            // Advance lineptr32 to keep std fb cursor in sync with standard renderer stride.
            lineptr32 += 2;
        }
#endif // !PICO_RP2040 (Profi DS80 branch)
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
        if (Config::arch == "Profi" && (Ports::portDFFD & 0x80))
            snowpage = MemESP::videoLatch ? 6 : 4;
        else
            snowpage = MemESP::videoLatch ? 7 : 5;
    } else if (Z80Ops::is128 && (MemESP::bankLatch & 0x01) && page == 0xc0) {  // Snow 128K
        snow_effect = 1;
        if (MemESP::bankLatch == 1 || MemESP::bankLatch == 3)
            snowpage = MemESP::videoLatch ? 3 : 1;
        else if (Config::arch == "Profi" && (Ports::portDFFD & 0x80))
            snowpage = MemESP::videoLatch ? 6 : 4;
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
    // ----------------------------------------------------------------
    // Profi DS80 deferred HDMI mode switch — applied NOW (vblank context).
    //
    // Problem: hdmi_set_profi_ds80_mode() rewrites conv_color[] which the
    // HDMI DMA reads in real time.  Calling it from Ports.cpp (Z80 loop,
    // core0) races the DMA on core1 during active scan → TMDS corruption
    // → picture disappears / flickers.
    //
    // Fix: Ports.cpp only sets flags; EndFrame() (always at vblank start)
    // applies them safely.  Deactivation restores 1240 conv_color entries
    // (incl. sync/audio); activation encodes 225+1 DS80 pair slots — both
    // must run during blanking.
    // ----------------------------------------------------------------
    {
        if (profi_ds80_activate_pending) {
            profi_ds80_activate_pending   = false;
            profi_palette_dirty           = false; // palette included in activate
            profi_ds80_driver_set(true, profi_palette_live, &profi_pair_lookup[0][0]);
            // Enable the Graphics-layer ZX→DS80 colour remap for the whole DS80 session
            // (not just OSD): any vga.* draw with a standard ZX index (FDD indicator,
            // LED legend, OSD, …) is then mapped to the correct solid DS80 pair slot.
            // The DS80 scan-time renderer writes pair slots into the fb directly (not via
            // dotFast), so it is unaffected by the remap.
            rebuildDS80ColorLut();
            Graphics8BitPalette::ds80_active = true;
            // DS80 border colour = Palette[(~borderColor) & 7] (inverse index, per
            // ZXMAK2).  Reset() sets borderColor=7 → (~7)&7 = 0 → Palette[0] = BLACK,
            // which is the desired default until a guest OUT 0xFE changes it.  (Do NOT
            // force borderColor=0 here: that would invert to Palette[7] = light grey.)
            // Apply DS80 geometry: full 240-line screen, no border.
            // (Mirrors Reset() DS80 branch; skips palette/pair-lookup re-init.)
            ds80_frame_grmem = nullptr; // force re-latch of display page on next frame
            grmem = MemESP::videoLatch ? MemESP::ram[6].direct() : MemESP::ram[4].direct();
            {
                uint32_t clrPg = MemESP::videoLatch ? 58u : 56u;
                // Pages 56 and 58 are locked (assign_ram locked=true): permanently
                // SRAM-resident, never in the evictable pool, direct() is always
                // valid — no preload/pin/precheck needed.
                profi_clrmem = MemESP::ram[clrPg].direct();
                // Snapshot the 16 KB clrmem page into SRAM here (vblank) so the
                // rasterizer never touches XIP PSRAM during active scan.
                if (profi_clrmem && ds80_clr_sram)
                    memcpy(ds80_clr_sram, profi_clrmem, DS80_CLR_SRAM_SIZE);
#if PROFI_PORT_TRACE
                ds80_dbg_grmem  = grmem;
                ds80_dbg_clrmem = profi_clrmem;
#endif
            }
            applyDS80BorderGeometry(true);
            tstateDraw   = tStatesScreen;
            linedraw_cnt = lin_end;
            updateBorderBrd();        // seed DS80 border pair colour
            DrawBorder = &Border_Blank; // mid-frame machine state is stale — skip this frame's flush
            brdChange = true;           // schedule full border repaint next frame
            // Zero all vga.frameBuffer rows: scan-time renderer writes only content bytes
            // (pad_l..pad_l+255) into each DS80 row; padding bytes must start at 0 (black
            // pair) and are never overwritten. Rows 240..yres-1 stay black throughout DS80.
            if (vga.frameBuffer) {
                for (int _y = 0; _y < (int)vga.yres; _y++)
                    if (vga.frameBuffer[_y]) memset(vga.frameBuffer[_y], 0, vga.xres);
            }
            Debug::log("[DS80] activate: tStScreen=%d grmem=%p clrmem=%p (SRAM=%d) videoLatch=%d",
                       tStatesScreen, grmem, profi_clrmem,
                       (profi_clrmem && (uintptr_t)profi_clrmem < 0x11000000u),
                       (int)MemESP::videoLatch);
        } else if (profi_ds80_deactivate_pending) {
            profi_ds80_deactivate_pending = false;
            Debug::log("[EF] DS80 deactivate: grmem=%p clrmem=%p", grmem, profi_clrmem);
            profi_ds80_driver_set(false, nullptr, nullptr);
            Graphics8BitPalette::ds80_active = false; // leave DS80 → raw ZX indices again
            // Clear framebuffer: DS80 packed-pair slot values look like garbage
            // when re-interpreted through the standard HDMI conv_color table.
            if (vga.frameBuffer) {
                for (int _y = 0; _y < (int)vga.yres; _y++)
                    if (vga.frameBuffer[_y]) memset(vga.frameBuffer[_y], 0, vga.xres);
            }
            // Restore standard Profi (non-DS80) geometry.
            grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
            profi_clrmem = nullptr;
            applyDS80BorderGeometry(false);
            tstateDraw    = tStatesScreen;
            linedraw_cnt  = lin_end;
            updateBorderBrd();        // back to std border palette
            DrawBorder = &Border_Blank; // mid-frame machine state is stale — skip this frame's flush
            brdChange = true;           // schedule full border repaint next frame
        }
    }

    // Profi palette refresh. EndFrame is NOT in the display blanking window:
    // ESPectrum_vsync (fired at scanout line v_active) only STARTS the frame's
    // emulation, and EndFrame runs when the Z80 frame finishes — typically
    // mid-scanout of the next refresh. Rewriting conv_color here while the DMA
    // is on-screen splits the picture into old/new palette halves (stable tear
    // line during guest palette fades — Karabas-Pro palette test). With v-sync
    // pacing ON the refresh is applied by ESPectrum::loop right after the
    // v_sync wait (true blanking start) via profiPaletteApplyPending(); apply
    // here only when that hook won't run (v-sync pacing off, or maxSpeed which
    // skips the wait) — unsynced anyway, a tear is unavoidable then.
    if (!Config::v_sync_enabled || ESPectrum::maxSpeed)
        profiPaletteApplyPending();

    // DS80 top/bottom border bands (720×576) are rendered per-T-state by the
    // border state machine (TopBorder/BottomBorder with DS80 geometry) — no
    // frame-granular fill here.  The stats rectangle is carved out inside
    // Update_Border_DS80 and owned by OSD::drawStats.

// DS80 framebuffer diagnostic: log framebuffer bytes and latched pointers every 60 frames.
    // This tells us whether the renderer is filling the framebuffer correctly.
    {
// #if !PICO_RP2040
//         static uint32_t ds80_diag_frame = 0;
//         if (profi_ds80_active && ++ds80_diag_frame >= 60) {
//             ds80_diag_frame = 0;
//             const int ds80_voff = (vga.yres >= 288) ? 24 : 0;
//             const int pad_l     = ((int)vga.xres - 256) / 2;
//             // Log latched pointers and 8 framebuffer bytes from content row 0.
//             uint8_t* row0 = vga.frameBuffer ? (uint8_t*)vga.frameBuffer[ds80_voff]     : nullptr;
//             uint8_t* row1 = vga.frameBuffer ? (uint8_t*)vga.frameBuffer[ds80_voff+120] : nullptr;
//             if (row0 && row1)
//                 Debug::log("[DS80D] grmem=%p clrmem=%p vl=%d fb[%d][%d..%d]=%02X%02X%02X%02X fb[%d]=%02X%02X%02X%02X",
//                     ds80_frame_grmem, ds80_frame_clrmem, (int)MemESP::videoLatch,
//                     ds80_voff, pad_l, pad_l+3,
//                     row0[pad_l], row0[pad_l+1], row0[pad_l+2], row0[pad_l+3],
//                     ds80_voff+120,
//                     row1[pad_l], row1[pad_l+1], row1[pad_l+2], row1[pad_l+3]);
//         }
// #endif
    }

    // Frame-timing diagnostic: average CPU::loop() time, logged every 60 frames.
#if PERF_TRACE
    {   // all archs: cpu_frame_us is accumulated unconditionally in CPU::loop()
        extern volatile uint32_t cpu_frame_us;
        extern volatile uint32_t fdd_step_us;   // time in end-of-frame rvmWD1793Step
        extern volatile uint32_t hdmi_irq_max_gap_us;
        static uint32_t port_log_frame = 0;
        static uint32_t cpu_accum = 0, cpu_max = 0, gap_max = 0;
        static uint32_t fdd_step_accum = 0, fdd_step_max = 0;
        static uint32_t fdd_ports_accum = 0, fdd_ports_max = 0;
        static uint64_t wall_t0 = 0;
        cpu_accum += cpu_frame_us;
        if (cpu_frame_us > cpu_max) cpu_max = cpu_frame_us;
        fdd_step_accum += fdd_step_us;
        if (fdd_step_us > fdd_step_max) fdd_step_max = fdd_step_us;
        fdd_ports_accum += Ports::fdd_ports_us;
        if (Ports::fdd_ports_us > fdd_ports_max) fdd_ports_max = Ports::fdd_ports_us;
        if (hdmi_irq_max_gap_us > gap_max) gap_max = hdmi_irq_max_gap_us;
        hdmi_irq_max_gap_us = 0;
        cpu_frame_us = 0;
        fdd_step_us = 0;
        Ports::fdd_ports_us = 0;
        if (++port_log_frame >= 60) {
            uint64_t now = time_us_64();
            float fps = wall_t0 ? (60.0f * 1000000.0f / (float)(now - wall_t0)) : 0.0f;
            wall_t0 = now;
            port_log_frame = 0;
            Debug::log("[PERF] 60f: cpu=%.1fms (max %.1fms) fdd_step=%.1fms (max %.1fms) fdd_ports=%.1fms (max %.1fms) hdmiGapMax=%uus realFPS=%.2f",
                cpu_accum / 60000.0f, cpu_max / 1000.0f,
                fdd_step_accum / 60000.0f, fdd_step_max / 1000.0f,
                fdd_ports_accum / 60000.0f, fdd_ports_max / 1000.0f,
                (unsigned)gap_max, fps);
            cpu_accum = cpu_max = gap_max = 0;
            fdd_step_accum = fdd_step_max = 0;
            fdd_ports_accum = fdd_ports_max = 0;
        }
    }
#endif // PERF_TRACE

    // Per-frame swap/accessor snapshot for the [NEG] worst-frame attribution
    // in ESPectrum::loop (this block resets the live counters every frame, so
    // the loop tail could not read them otherwise).  swap_us includes the
    // idle-window part accrued after the previous frame's pacing; the in-frame
    // share is swap_us - swap_idle_us.
    {
        extern volatile uint32_t g_frame_swap_us, g_frame_swap_idle_us, g_frame_accb;
        g_frame_swap_us      = mem_spi_swap_us;
        g_frame_swap_idle_us = mem_spi_swap_idle_us;
        g_frame_accb         = mem_spi_accb;
    }

    // SPI PSRAM swap diagnostics: log eviction count once every 60 frames.
    {
        static uint32_t evict_log_frame = 0;
        static uint32_t evict_accum = 0;
        static uint32_t evict_last_page = 0;
        static uint32_t skip_accum = 0;
        static uint32_t wbskip_accum = 0;
        static uint32_t swap_us_accum = 0;
        static uint32_t accb_accum = 0;
        static uint32_t promo_accum = 0;
        static uint32_t promo_idle_accum = 0;
        static uint32_t swap_idle_us_accum = 0;
        evict_accum  += mem_spi_evict_count;
        skip_accum   += mem_spi_read_skip;
        wbskip_accum += mem_spi_wb_skip;
        swap_us_accum += mem_spi_swap_us;
        accb_accum   += mem_spi_accb;
        promo_accum  += mem_spi_promo;
        promo_idle_accum   += mem_spi_promo_idle;
        swap_idle_us_accum += mem_spi_swap_idle_us;
        evict_last_page = mem_spi_evict_page;
        mem_spi_evict_count = 0;
        mem_spi_read_skip = 0;
        mem_spi_wb_skip = 0;
        mem_spi_swap_us = 0;
        mem_spi_accb = 0;
        mem_spi_promo = 0;
        mem_spi_promo_idle = 0;
        mem_spi_swap_idle_us = 0;
        // Every 300 frames (5 s): the Debug::log line itself costs ~6 ms over
        // USB-CDC and was reliably the worst frame of every 60-frame window
        // during disk/page activity — keep the diagnostic, shed 80% of its
        // frame-time pollution.
        if (++evict_log_frame >= 300) {
            evict_log_frame = 0;
            // swap=TOTALms(idle Xms): X of the total ran in the frame's idle
            // window (MemESP::idleService) — only TOTAL−X ate emulation time.
            if (evict_accum || skip_accum || accb_accum)
                Debug::log("[SPI] evict/60f=%u skip=%u wbskip=%u accb=%u promo=%u(idle %u) swap=%ums(idle %u) last_pg=%u (%.1f/frame)",
                           evict_accum, skip_accum, wbskip_accum, accb_accum, promo_accum,
                           promo_idle_accum, swap_us_accum / 1000u,
                           swap_idle_us_accum / 1000u, evict_last_page, evict_accum / 60.0f);
            skip_accum = 0;
            wbskip_accum = 0;
            swap_us_accum = 0;
            accb_accum = 0;
            promo_accum = 0;
            promo_idle_accum = 0;
            swap_idle_us_accum = 0;
            evict_accum = 0;
#if MEM_ACCESS_TRACE
            // Accesses served per evicted-page load (accessor-mode feasibility):
            // clean victims with <512 accesses would have been cheaper served
            // per-byte over SPI than via the 16KB page load (~600 = breakeven).
            if (mem_acc_clean_cnt || mem_acc_dirty_cnt) {
                Debug::log("[ACC] clean n=%u avg=%u max=%u <128=%u <512=%u | dirty n=%u avg=%u",
                           mem_acc_clean_cnt,
                           mem_acc_clean_cnt ? mem_acc_clean_sum / mem_acc_clean_cnt : 0,
                           mem_acc_clean_max, mem_acc_lo128, mem_acc_lo512,
                           mem_acc_dirty_cnt,
                           mem_acc_dirty_cnt ? mem_acc_dirty_sum / mem_acc_dirty_cnt : 0);
                mem_acc_clean_cnt = mem_acc_clean_sum = mem_acc_clean_max = 0;
                mem_acc_lo128 = mem_acc_lo512 = 0;
                mem_acc_dirty_cnt = mem_acc_dirty_sum = 0;
            }
#endif
        }
    }

    // ----------------------------------------------------------------
    // DS80 frame finalization: NO BLIT NEEDED.
    // vga.frameBuffer written scan-line-synchronous by the Z80 renderer;
    // rows 240..479 zeroed at DS80 activation and never touched again.
    // ----------------------------------------------------------------
    // Clear DMA attr shadow and charrow write counters for next frame
    if (Config::dma_mode)
        Z80DMA::resetAttrShadow();
    dma_attr_override = nullptr;

    // Rebuild Gigascreen blend palette if a ULA+ session clobbered its slots.
    // Safe here: EndFrame runs during blanking, HDMI DMA is not reading conv_color.
    if (gigascreen_lut_rebuild_deferred && Config::gigascreen_enabled) {
        gigascreen_lut_rebuild_deferred = false;
        gigsBlendLUTReady = false;
        initGigascreenBlendLUT();
    }

    // Flush deferred ULA+ palette updates so HDMI DMA sees consistent palette
    // for the entire next frame (prevents top-of-screen palette tearing)
    if (ulaplus_enabled) {
        if (ulaplus_alubytes_dirty) {
            ulaplus_alubytes_dirty = false;
            ulaplus_palette_dirty = false; // full rebuild supersedes per-entry flush
            regenerateUlaPlusAluBytes();
        } else {
            ulaPlusFlushPalette();
        }
    }
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

#if !PICO_RP2040
    // DS80 borders are rendered by the state machine with DS80 geometry
    // (applyDS80BorderGeometry).  TopBorder_Blank handles lin_end==0 and
    // MiddleBorder guards lin_end2==yres, so no Border_Blank override needed.
    // Refresh the per-frame stats carve-out flags for Update_Border_DS80.
    ds80_osd_carve = ds80_border_geom && (VIDEO::OSD & 0x03) && !(VIDEO::OSD & 0x04);
    ds80_carve240  = (int)vga.yres < 288;
#endif

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

    // Decay activity counters every frame regardless of the border-glyph setting —
    // the corner FDD lamp + motor-hum sound (Config::trdosSoundLed) consume them too.
    LED::decay();
    if (Config::ledIndicators) {
        if (gigascreen_enabled) LED::touchR(LED::GIGASCREEN);
        LED::draw();
    }

    framecnt++;

#if !PICO_RP2040
    // Debug::log("[HB] f=%u pc=0x%04X bc=0x%04X hl=0x%04X iff=%u rom=%u dffd=0x%02X eff7=0x%02X bl=%u vidlatch=%u",
    //     framecnt, Z80::getRegPC(), Z80::getRegBC(), Z80::getRegHL(),
    //     (unsigned)(Z80::isIFF1()?1:0), MemESP::romInUse, Ports::portDFFD, Ports::portEFF7,
    //     MemESP::bankLatch, MemESP::videoLatch);
#endif
}

// Repaint one full frame from the frozen machine state. The scanline renderer
// only paints as a side effect of Z80 execution advancing CPU::tstates, so a
// paused machine never repaints — an OSD window closed while paused would stay
// on screen until unpause. Walks the paper renderer and the border state
// machine across a whole frame without executing a single instruction (same
// video-only technique as CPU::FlushOnHalt / EndFrame's brdChange repaint).
void VIDEO::RedrawPausedFrame() {

    if (!vga.frameBuffer) return;

    uint32_t saved_tstates = CPU::tstates;

    // Arm the paper renderer at start-of-frame state (mirror of EndFrame's
    // re-arm — while paused it may be left at Blank by a maxSpeed skip-frame).
    linedraw_cnt = lin_end;
    tstateDraw = tStatesScreen;
    void (*blank)(unsigned int, bool);
    if (snow_toggle) {
        Draw = &MainScreen_Blank_Snow;
        Draw_Opcode = &MainScreen_Blank_Snow_Opcode;
        blank = &Blank_Snow;
    } else {
        Draw = &MainScreen_Blank;
        Draw_Opcode = &MainScreen_Blank_Opcode;
        blank = &Blank;
    }

    // Walk the whole paper area; the chain parks itself at Blank after lin_end2.
    // Guard: ~1 line per call, so a frame is ~lines calls — cap well above that.
    CPU::tstates = 0;
    for (int guard = 2048; Draw != blank && guard; guard--)
        Draw(tStatesPerLine, false);

    // Full border repaint: with tstates at end-of-frame the border state machine
    // walks Top→Middle→Bottom→Blank in one call, preserving the stats carve-outs.
    CPU::tstates = CPU::statesInFrame;
    lastBrdTstate = tStatesBorder;
    DrawBorder = &TopBorder_Blank;
    DrawBorder();

    CPU::tstates = saved_tstates;
    // Draw/DrawBorder are left "done" (Blank/Border_Blank); the per-frame
    // EndFrame call in CPU::loop's paused branch re-arms them as usual.
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
    if (old_packed != packed32) {
        prevWord[0] = packed32;
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

#if !PICO_RP2040
// Profi DS80: write 1 uint16_t (2 packed pair bytes = 4px) per T-state.
// The HDMI/VGA ISR reads pair bytes in (x^2) order, so within an aligned
// uint16 pair display order is swapped — same ^1 trick as Pentagon.
// VIDEO::brd already holds the solid border pair byte replicated.
// ds80_brd_col_off centres the 160 visible T-cols in wider rows (720×576);
// the off-screen edge cols (h-blanking) are extended from the nearest tact.
IRAM_ATTR static void Update_Border_DS80() {
    if (ds80_osd_carve) {
        // Stats overlay rectangle is owned by OSD::drawStats — skip it.
        if (ds80_carve240) {
            // 640×480: rows 220..235, fb bytes 288..311 (right pad part)
            if (brdlin_cnt >= 220 && brdlin_cnt < 236
                && brdcol_cnt >= 144 && brdcol_cnt < 156) return;
        } else {
            // 720×576: rows 268..283, fb bytes 188..331 (bottom band)
            int c = brdcol_cnt + ds80_brd_col_off;
            if (brdlin_cnt >= 268 && brdlin_cnt < 284
                && c >= 94 && c < 166) return;
        }
    }
    const uint16_t v = (uint16_t)VIDEO::brd;
    const int col = brdcol_cnt + ds80_brd_col_off;
    brdptr16[col ^ 1] = v;
    if (ds80_brd_col_off) {
        if (brdcol_cnt == 0) {
            for (int c = 0; c < ds80_brd_col_off; c++) brdptr16[c ^ 1] = v;
        } else if (brdcol_cnt == brdcol_end - 1) {
            for (int c = col + 1; c <= col + ds80_brd_col_off; c++) brdptr16[c ^ 1] = v;
        }
    }
}
#endif // !PICO_RP2040

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

//----------------------------------------------------------------------------------------------------------------
// Span variants: paint n consecutive border columns starting at brdcol_cnt in
// one call — semantically identical to n per-column Update_Border() calls (the
// catch-up colour VIDEO::brd is constant across the whole span), but filled
// with word stores instead of ~50 cycles of call+loop overhead per column.
// This is what makes animated-border Gigascreen demos fit at 378 MHz: the
// per-column machine alone cost ~7 of the ~9 ms/frame worst case.
// Globals (brdcol_cnt/lastBrdTstate) are advanced by the caller.
//----------------------------------------------------------------------------------------------------------------

static void (*Update_Border_Span)(int n);

IRAM_ATTR static void Update_Border_Span_Pair(int n) {
    uint32_t color32 = VIDEO::brd | (VIDEO::brd << 16);
    uint32_t* p = (uint32_t*)&brdptr16[brdcol_cnt];   // cnt aligned to 4 (step=4)
    for (int k = 2 * n; k > 0; k--) *p++ = color32;   // n cols × 8 px = 2n words
}

IRAM_ATTR static void Update_Border_Span_Pair_Gig(int n) {
    uint32_t color32 = VIDEO::brd | (VIDEO::brd << 16);
    uint8_t  nib = VIDEO::brd & 0x0F;
    uint32_t packed32 = nib * 0x11111111u;
    uint32_t* prevW = (uint32_t*)&prevBrdptr8[brdcol_cnt];
    uint32_t* fb    = (uint32_t*)&brdptr16[brdcol_cnt];
    for (int k = 0; k < n; k++, prevW++, fb += 2) {
        uint32_t old = *prevW;
        if (old != packed32) {
            *prevW = packed32;
            uint32_t mixed = blendPixels32(color32, (old & 0x0F) * 0x01010101u);
            fb[0] = mixed; fb[1] = mixed;
            VIDEO::brdGigascreenChange = true;
        } else {
            fb[0] = color32; fb[1] = color32;
        }
    }
}

IRAM_ATTR static void Update_Border_Span_XOR(int n) {
    // Display order is swapped within uint16 pairs (^1); a uniform fill over an
    // even-aligned range is permutation-invariant, so only odd edges need care.
    int c0 = brdcol_cnt, c1 = brdcol_cnt + n;
    const uint16_t v = (uint16_t)VIDEO::brd;
    if (c0 & 1) { brdptr16[c0 ^ 1] = v; c0++; }
    if ((c1 & 1) && c1 > c0) { c1--; brdptr16[c1 ^ 1] = v; }
    uint32_t vv = (uint32_t)v | ((uint32_t)v << 16);
    uint32_t* p = (uint32_t*)&brdptr16[c0];
    for (int k = (c1 - c0) >> 1; k > 0; k--) *p++ = vv;
}

// One column of the XOR_Gig shape at source column idx (target idx^1).
static inline void xorGigOne(int idx, uint32_t newColor, uint8_t newPacked) {
    int t = idx ^ 1;
    uint8_t oldPacked = prevBrdptr8[t];
    if (oldPacked != newPacked) {
        prevBrdptr8[t] = newPacked;
        brdptr16[t] = blendPixels32(newColor, (uint32_t)(oldPacked & 0x0F) * 0x01010101u);
        VIDEO::brdGigascreenChange = true;
    } else {
        brdptr16[t] = (uint16_t)newColor;
    }
}

IRAM_ATTR static void Update_Border_Span_XOR_Gig(int n) {
    int c0 = brdcol_cnt, c1 = brdcol_cnt + n;
    uint32_t newColor = VIDEO::brd;
    uint8_t  nib = (uint8_t)(newColor & 0x0F);
    uint8_t  newPacked = (uint8_t)(nib | (nib << 4));
    if (c0 & 1) { xorGigOne(c0, newColor, newPacked); c0++; }
    if ((c1 & 1) && c1 > c0) { c1--; xorGigOne(c1, newColor, newPacked); }
    // Even-aligned middle: 2 cols per step (prev uint16, fb uint32). Uniform
    // old pairs (the norm — prev was filled with runs of one colour) blend once.
    const uint16_t new2 = (uint16_t)(newPacked | (newPacked << 8));
    for (int c = c0; c < c1; c += 2) {
        uint16_t old2 = *(uint16_t*)&prevBrdptr8[c];
        if (old2 == new2) {
            *(uint32_t*)&brdptr16[c] = newColor;
        } else if ((uint8_t)(old2 & 0xFF) == (uint8_t)(old2 >> 8)) {
            *(uint16_t*)&prevBrdptr8[c] = new2;
            *(uint32_t*)&brdptr16[c] =
                blendPixels32(newColor, (uint32_t)(old2 & 0x0F) * 0x01010101u);
            VIDEO::brdGigascreenChange = true;
        } else {
            xorGigOne(c, newColor, newPacked);
            xorGigOne(c + 1, newColor, newPacked);
        }
    }
}

#if !PICO_RP2040
// Fallback for shapes without a fast span (DS80): n per-column calls.
IRAM_ATTR static void Update_Border_Span_Generic(int n) {
    int save = brdcol_cnt;
    for (int k = 0; k < n; k++) { Update_Border(); brdcol_cnt += brdcol_step; }
    brdcol_cnt = save;
}
#endif

static void Select_Update_Border() {
#if !PICO_RP2040
    pwRefreshGate();  // geometry may have changed — re-evaluate the DMA window
    if (ds80_border_geom) {
        Update_Border = &Update_Border_DS80;  // Gigascreen incompatible with Profi
        Update_Border_Span = &Update_Border_Span_Generic;
        return;
    }
#endif
    if (brdPairWrite) {
        Update_Border = VIDEO::gigascreen_enabled ? &Update_Border_Pair_Gig : &Update_Border_Pair;
        Update_Border_Span = VIDEO::gigascreen_enabled ? &Update_Border_Span_Pair_Gig : &Update_Border_Span_Pair;
    } else {
        Update_Border = VIDEO::gigascreen_enabled ? &Update_Border_XOR_Gig : &Update_Border_XOR;
        Update_Border_Span = VIDEO::gigascreen_enabled ? &Update_Border_Span_XOR_Gig : &Update_Border_Span_XOR;
    }
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
        prevBrdptr8 = vga.prevFrameBuffer ? prevRowBorder(0) : (uint8_t *)brdptr16;
        // lin_end==0 (DS80 640×480): no top border rows — straight to side borders.
        // (TopBorder would otherwise paint row 0 full-width over the content.)
        DrawBorder = lin_end ? &TopBorder : &MiddleBorder;
        DrawBorder();
    }
}

IRAM_ATTR void VIDEO::TopBorder() {
    while (lastBrdTstate <= CPU::tstates) {
        if (brdcol_cnt < brdcol_retrace) {
            // Paint every column this catch-up covers in the current row at
            // once (the colour is constant across the span — see span fns).
            int lim = (brdcol_retrace - brdcol_cnt) / brdcol_step;
            unsigned int avail = (CPU::tstates - lastBrdTstate) / (unsigned)brdcol_step + 1;
            int n = (avail < (unsigned)lim) ? (int)avail : lim;
            Update_Border_Span(n);
            lastBrdTstate += (n - 1) * brdcol_step;
            brdcol_cnt += (n - 1) * brdcol_step;
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
            prevBrdptr8 = vga.prevFrameBuffer ? prevRowBorder(brdlin_cnt) : (uint8_t *)brdptr16;
            brdcol_cnt = brdcol_start;
            lastBrdTstate += tStatesPerLine - brdcol_end;

            if (brdlin_cnt >= lin_end) {
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
            // Span must stop exactly at brdcol_end1 — the paper-skip check
            // below fires on equality.
            int stop = (brdcol_cnt < brdcol_end1) ? brdcol_end1 : brdcol_retrace;
            int lim = (stop - brdcol_cnt) / brdcol_step;
            unsigned int avail = (CPU::tstates - lastBrdTstate) / (unsigned)brdcol_step + 1;
            int n = (avail < (unsigned)lim) ? (int)avail : lim;
            Update_Border_Span(n);
            lastBrdTstate += (n - 1) * brdcol_step;
            brdcol_cnt += (n - 1) * brdcol_step;
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
            brdcol_cnt = brdcol_start;
            lastBrdTstate += tStatesPerLine - brdcol_end;
            if (brdlin_cnt == (int)lin_end2) {
                // DS80 640×480: lin_end2 == yres — no bottom border rows at all.
                if (brdlin_cnt >= (int)vga.yres) {
                    DrawBorder = &Border_Blank;
                    return;
                }
                brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
                prevBrdptr8 = vga.prevFrameBuffer ? prevRowBorder(brdlin_cnt) : (uint8_t *)brdptr16;
#if !PICO_RP2040
                // DS80: BottomBorder_OSD carve coords are for std-fb layouts —
                // Update_Border_DS80 carves the stats rect itself; use plain bottom.
                DrawBorder = ds80_border_geom ? &BottomBorder : Draw_OSD43;
#else
                DrawBorder = Draw_OSD43;
#endif
                DrawBorder();
                return;
            }
            brdptr16 = (uint16_t *)(vga.frameBuffer[brdlin_cnt]);
            prevBrdptr8 = vga.prevFrameBuffer ? prevRowBorder(brdlin_cnt) : (uint8_t *)brdptr16;
        }
    }
}

IRAM_ATTR void VIDEO::BottomBorder() {
    while (lastBrdTstate <= CPU::tstates) {
        if (brdcol_cnt < brdcol_retrace) {
            int lim = (brdcol_retrace - brdcol_cnt) / brdcol_step;
            unsigned int avail = (CPU::tstates - lastBrdTstate) / (unsigned)brdcol_step + 1;
            int n = (avail < (unsigned)lim) ? (int)avail : lim;
            Update_Border_Span(n);
            lastBrdTstate += (n - 1) * brdcol_step;
            brdcol_cnt += (n - 1) * brdcol_step;
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
            prevBrdptr8 = vga.prevFrameBuffer ? prevRowBorder(brdlin_cnt) : (uint8_t *)brdptr16;
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
            } else {
                const bool inStats = (brdcol_cnt >= osd_x_start && brdcol_cnt < osd_x_end);
                if (!inStats) Update_Border();
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
            prevBrdptr8 = vga.prevFrameBuffer ? prevRowBorder(brdlin_cnt) : (uint8_t *)brdptr16;
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

// Shared by save()/restore_last()/store_ram()/restore_ram(): each opens, uses
// and closes it within a single synchronous call, on the same file, never
// overlapping — so one static FIL (~600 B) replaces four (was one per method).
static FIL s_saveRectFile;

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
        // FatFS f_open calls ff_memalloc() for LFN/VFAT buffers. SDK malloc
        // panics on OOM (no NULL return), so reject before opening when the
        // heap can't satisfy it. Bail to a dummy push — restore will redraw
        // the underlying screen instead of crashing.
        if (getContiguousHeap() < FF_OPEN_HEAP_FLOOR) {
            offsets.push_back(off);
            return;
        }
        // sizeof(FIL) ~= 580 B (FF_MAX_SS=512 sector buf). The OSD runs on a tight
        // ~2 KB core stack and SaveRect::save is reached from deep call chains
        // (e.g. fileDialog -> viewInfo -> showInfoBox -> save) — a FIL on the stack
        // overflows it and corrupts neighbouring memory (timer callbacks etc),
        // crashing later in alarm_pool_irq_handler. save() is synchronous
        // (open/write/close within one call) so a static FIL is safe even nested.
        FIL &f = s_saveRectFile;
        if (f_open(&f, "/tmp/save_rect.tmp", FA_WRITE | FA_OPEN_ALWAYS) != FR_OK) {
            offsets.push_back(off); // open failed — dummy
            return;
        }
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
        if (!w || !h || y >= (uint16_t)VIDEO::vga.yres) return;
        size_t line_end_p = (size_t)y + h;
        if (line_end_p > (size_t)VIDEO::vga.yres) line_end_p = VIDEO::vga.yres;
        for (size_t line = y; line < line_end_p; ++line) {
            readpsram(VIDEO::vga.frameBuffer[line] + x, pos, w);
            pos += w;
        }
        if (offsets.empty()) offsets.push_back(0);
        return;
    }
    if (FileUtils::fsMount) {
        // f_open allocates LFN scratch — gate with heap floor; on shortage the
        // dialog underneath just redraws on close (no restore happens).
        if (getContiguousHeap() < FF_OPEN_HEAP_FLOOR) {
            if (offsets.empty()) offsets.push_back(0);
            return;
        }
        // Static FIL: sizeof(FIL) ~= 580 B would overflow the tight ~2 KB OSD
        // stack from deep call chains (same fix as save()). restore is
        // synchronous (open/read/close in one call) so a static FIL is safe.
        FIL &f = s_saveRectFile;
        if (f_open(&f, "/tmp/save_rect.tmp", FA_READ) != FR_OK) {
            if (offsets.empty()) offsets.push_back(0);
            return;
        }
        f_lseek(&f, off);
        UINT br;
        f_read(&f, &x, 2, &br);
        f_read(&f, &y, 2, &br);
        f_read(&f, &w, 2, &br);
        f_read(&f, &h, 2, &br);
        if (!w || !h || y >= (uint16_t)VIDEO::vga.yres) {
            f_close(&f);
            return;
        }
        size_t line_end = (size_t)y + h;
        if (line_end > (size_t)VIDEO::vga.yres) line_end = VIDEO::vga.yres;
        for (size_t line = y; line < line_end; ++line) {
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
        if (!w || !h || y >= (uint16_t)VIDEO::vga.yres) return;
        size_t line_end_r = (size_t)y + h;
        if (line_end_r > (size_t)VIDEO::vga.yres) line_end_r = VIDEO::vga.yres;
        for (size_t line = y; line < line_end_r; ++line) {
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
    if (offsets.empty()) offsets.push_back(0);
    size_t off = offsets.back();
    // Always push so restore_ram can pop correctly without UB.
    // Use sentinel (size_t)-1 when the write is skipped.
    if (getContiguousHeap() < FF_OPEN_HEAP_FLOOR) {
        offsets.push_back((size_t)-1);
        return;
    }
    // Static FIL: same 580-byte stack-overflow guard as save() / restore_last().
    FIL &f = s_saveRectFile;
    if (f_open(&f, "/tmp/save_rect.tmp", FA_WRITE | FA_OPEN_ALWAYS) != FR_OK) {
        offsets.push_back((size_t)-1);
        return;
    }
    UINT bw;
    f_lseek(&f, off);
    f_write(&f, p, sz, &bw);
    f_close(&f);
    offsets.push_back(off + sz);
}

void SaveRectT::restore_ram(void* p, size_t sz) {
    if (offsets.size() < 2) {
        if (offsets.empty()) offsets.push_back(0);
        return;
    }
    size_t top = offsets.back();
    offsets.pop_back();
    if (top == (size_t)-1) return; // store_ram skipped the write — nothing to restore
    size_t off = offsets.back();   // position where store_ram wrote the data
    if (getContiguousHeap() < FF_OPEN_HEAP_FLOOR) return;
    FIL &f = s_saveRectFile;
    if (f_open(&f, "/tmp/save_rect.tmp", FA_READ) != FR_OK) return;
    f_lseek(&f, off);
    UINT br;
    f_read(&f, p, sz, &br);
    f_close(&f);
}
