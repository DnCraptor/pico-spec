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

#ifndef VIDEO_h
#define VIDEO_h

#include <inttypes.h>
#include "ESPectrum.h"
#include "Config.h"
#include "VGA/VGA8Bit.h"
#include <list>
#include <vector>

#define SPEC_W 256
#define SPEC_H 192

#if !PICO_RP2040
// Free SRAM that must remain after the Gigascreen prev-FB is allocated, so the
// running system keeps working room. Shared by VIDEO::ensurePrevFB (the live
// allocation guard) and the SRAM budget gate (Subsystems::featureMargin for
// FEAT_GIGASCREEN) — they MUST use the same value, or the gate says ALLOW while
// ensurePrevFB silently declines (Gigascreen stays off, no popup).
// History: 24 KB originally, over an empirical "runs fine down to ~20 KB" floor.
// Lowered to 16 KB deliberately, to let Gigascreen coexist with the small audio
// features on a butter-less board (37.7 KB prev-FB out of ~66 KB of heap leaves no
// room for both otherwise). This is BELOW that measured floor: it is the emulator
// itself that keeps running, but what spends this headroom are the file dialogs,
// the browser index, ZIP extraction and network sessions — those are what to
// re-test after touching this number, not gameplay.
static constexpr size_t GIGASCREEN_PREVFB_HEADROOM = 16 * 1024;
#endif

#define TSTATES_PER_LINE 224
#define TSTATES_PER_LINE_128 228
#define TSTATES_PER_LINE_PENTAGON 224
#define TSTATES_PER_LINE_PROFI 224
#define TSTATES_PER_LINE_BYTE 224

#define TS_SCREEN_48           14335  // START OF ULA DRAW PAPER 48K
#define TS_SCREEN_128          14361  // START OF ULA DRAW PAPER 128K
#define TS_SCREEN_PENTAGON     17983  // START OF ULA DRAW PAPER PENTAGON
#define TS_SCREEN_PROFI        12583  // START OF ULA DRAW PAPER PROFI (56*224+39)
#define TS_SCREEN_BYTE         14392  // START OF ULA DRAW PAPER BYTE (64*224+56)

// Profi DS80 (512×240) sync-gen runs 192 T-states/line (not 224!) — ZXMAK2
// ProfiRenderer: c_ulaLineTime=192, c_ulaFirstPaperTact=24, 16T side borders (1T = 4 px).
// First paper line = 48 after INT, NOT ZXMAK2's 72: mcprofi2016's border effect
// is exactly 4608 OUTs × 12T = 55296T = 288 lines starting at T≈4574 after INT —
// it tiles the full 288-line visible window (24+240+24) only with paper at line 48.
// With 72 the effect ends at paper bottom and the 24-row bottom band stays solid.
// ZXMAK2's c_ulaIntBegin=19 (screen tact = CPU tact + 19) is NOT subtracted: real-hw
// photo of mcprofi2016 shows our border pattern ~19T right of the true position with it.
// −2T net calibration on top of that centres the effect around the paper: +1T measured
// from hw screenshot side-strip asymmetry, −3T after FlushOnHalt phase-0 snap (CPU.cpp)
// fixed the per-launch 0..3T wake-up jitter and locked the pattern 3T left of centre.
#define TSTATES_PER_LINE_PROFI_DS80 192
#define TS_SCREEN_PROFI_DS80   9238   // 48*192 + 24 - 2 (calibration)
#define TS_BORDER_PROFI_DS80_240 9226 // 9238 - 16 (left border) + 4 (step=1 correction)
#define TS_BORDER_PROFI_DS80_288 4618 // 9226 - 24*192 (24-row top band incl. blanking rows)

#define TS_BORDER_320x240 8948  // START OF BORDER 48 (+5 correction)
#define TS_BORDER_320x240_128 8878  // START OF BORDER 128 (+5 correction)
#define TS_BORDER_320x240_PENTAGON 12595  // START OF BORDER PENTAGON
#define TS_BORDER_320x240_PROFI 7195      // START OF BORDER PROFI (24 top lines, centred: 12583 - 24*224 - 16 + 4)
#define TS_BORDER_320x240_BYTE 9005       // START OF BORDER BYTE (formula 9000 + 5)

#define TS_BORDER_360x200 13428  // START OF BORDER 48
#define TS_BORDER_360x200_128 13438  // START OF BORDER 128
#define TS_BORDER_360x200_PENTAGON 17075  // START OF BORDER PENTAGON
#define TS_BORDER_360x200_PROFI 11675     // START OF BORDER PROFI (= PENTAGON - 5400)
#define TS_BORDER_360x200_BYTE 13485      // START OF BORDER BYTE (formula 13480 + 5)

#define TS_BORDER_360x288 3564          // START OF BORDER 48 FULL (formula 3559 + 5)
#define TS_BORDER_360x288_128 3398      // START OF BORDER 128 FULL (formula 3393 + 5)
#define TS_BORDER_360x288_PENTAGON 7209 // START OF BORDER PENTAGON FULL (formula 7205 + 4)
#define TS_BORDER_360x288_PROFI 1809    // START OF BORDER PROFI FULL (= PENTAGON - 5400)
#define TS_BORDER_360x288_BYTE 3621     // START OF BORDER BYTE FULL (formula 3616 + 5)

#define TS_BORDER_360x240 8940          // START OF BORDER 48 HALF (formula 8935 + 5)
#define TS_BORDER_360x240_128 8870      // START OF BORDER 128 HALF (formula 8865 + 5)
#define TS_BORDER_360x240_PENTAGON 12585 // START OF BORDER PENTAGON HALF (formula 12581 + 4)
#define TS_BORDER_360x240_PROFI 7185    // START OF BORDER PROFI HALF (24 top lines, centred: 12583 - 24*224 - 26 + 4)
#define TS_BORDER_360x240_BYTE 8997     // START OF BORDER BYTE HALF (formula 8992 + 5)

// Colors as 8-bit palette indices (VGA8 mode)
// Standard Spectrum color order: 0-7 normal, 8-15 bright, 16 orange
#define BLACK       0
#define BLUE        1
#define RED         2
#define MAGENTA     3
#define GREEN       4
#define CYAN        5
#define YELLOW      6
#define WHITE       7
#define BRI_BLACK   8
#define BRI_BLUE    9
#define BRI_RED     10
#define BRI_MAGENTA 11
#define BRI_GREEN   12
#define BRI_CYAN    13
#define BRI_YELLOW  14
#define BRI_WHITE   15
#define ORANGE      16

#define NUM_SPECTRUM_COLORS 17

class SaveRectT {
  std::list<size_t> offsets;
  std::vector<uint8_t> ram_buf; // RAM fallback when no SD card
public:
  SaveRectT() : offsets() {
    offsets.push_back(0);
  }
  void save(int16_t x, int16_t y, int16_t w, int16_t h);
  void restore_last();
  void clear() { offsets.clear(); offsets.push_back(0); ram_buf.clear(); f_unlink("/tmp/save_rect.tmp"); }
  // Release ram_buf capacity (clear() leaves the vector's backing alloc in place).
  // Use before tight-heap operations like framebuffer growth.
  void dropCapacity() { std::vector<uint8_t>().swap(ram_buf); }
  void store_ram(const void* p, size_t sz);
  void restore_ram(void* p, size_t sz);
};

#if !PICO_RP2040
void initGigascreenBlendLUT();
#endif

class VIDEO
{
public:

  // Initialize video
  static void Init();
    
  // Reset video
  static void Reset();

#ifdef VGA_HDMI
  // Hot video mode switch (no reboot)
  static void changeMode();
#endif

  // Video draw functions
  static void EndFrame();
  // Repaint a full frame from the frozen machine state (used while CPU::paused,
  // where the renderer otherwise never runs — e.g. to erase a closed OSD menu).
  static void RedrawPausedFrame();
  static void Blank(unsigned int statestoadd, bool contended);
  static void Blank_Opcode(bool contended);
  static void Blank_Snow(unsigned int statestoadd, bool contended);
  static void Blank_Snow_Opcode(bool contended);
  // 48 / 128
  static void MainScreen_Blank(unsigned int statestoadd, bool contended);
  static void MainScreen_Blank_Opcode(bool contended);
  static void MainScreen(unsigned int statestoadd, bool contended);
  static void MainScreen_OSD(unsigned int statestoadd, bool contended);
  static void MainScreen_Opcode(bool contended);
  static void MainScreen_OSD_Opcode(bool contended);
  static void MainScreen_Blank_Snow(unsigned int statestoadd, bool contended);
  static void MainScreen_Blank_Snow_Opcode(bool contended);
  static void MainScreen_Snow(unsigned int statestoadd, bool contended);
  static void MainScreen_Snow_Opcode(bool contended);
  
  // static void DrawBorderFast();
#if !PICO_RP2040
  static void InitPrevBuffer();

  // Lend the (dormant) Gigascreen prev framebuffer as scratch SRAM for the
  // duration of a paused network session — only on butter-less boards where the
  // TLS/socket working set would otherwise OOM the heap. Detaches prevFrameBuffer
  // so the renderer can't read it while lent; reclaim re-attaches + clears it.
  // Returns true (and fills base/size) only when there's a region to lend.
  static bool gigascreenLendRegion(void*& base, size_t& size);
  static void gigascreenReclaimRegion();

  // Same purpose, for a prev-FB that got split into whole-row chunks: it is not one
  // region, so it cannot be lent — give it up entirely for the session instead and
  // rebuild it afterwards. Returns true when something was released (then the caller
  // MUST pair it with gigascreenRestoreAfterNet). No-op for a single-block prev-FB,
  // which takes the cheaper lending path above.
  static bool gigascreenReleaseForNet();
  static void gigascreenRestoreAfterNet();

  // Byte size the Gigascreen prev-FB needs in the *current* video mode (4-bit
  // packed). Used by the SRAM budget manager to cost the Gigascreen feature.
  static size_t gigascreenPrevFBBytes();

  // Largest SINGLE allocation the prev-FB needs. Smaller than gigascreenPrevFBBytes()
  // because the buffer is only ever addressed row by row (vga.prevFrameBuffer[]) and
  // falls back to whole-row chunks when the heap has no block big enough. This — not
  // the total — is what the budget gate must compare against the largest free block.
  static size_t gigascreenPrevFBBlockBytes();
#endif

  static void Border_Blank();

  // Unified border functions (all models, all resolutions)
  static void TopBorder_Blank();
  static void TopBorder();
  static void MiddleBorder();
  static void BottomBorder();
  static void BottomBorder_OSD();
  
  static void (*Draw)(unsigned int, bool);
  static void (*Draw_Opcode)(bool);
  static void (*Draw_OSD169)(unsigned int, bool);
  static void (*Draw_OSD43)();
  
  static void (*DrawBorder)();

  static void vgataskinit(void *unused);

  static uint8_t* grmem;
  static uint8_t* profi_clrmem;   // Profi hires color attr page (56 or 58), NULL if in SPI PSRAM
#if !PICO_RP2040
#endif
#if !PICO_RP2040
  // pair_lookup[ink][paper] → safe HDMI palette index (avoids sync range 220-244, border 255).
  // Built by init_profi_pair_lookup() in Reset(). Used by rasterizer and passed to HDMI driver.
  static uint8_t profi_pair_lookup[16][16];
  // Live 16-color palette in RGB888 — modifiable by guest via OUT (port_low=0x7E).
  static uint32_t profi_palette_live[16];
  // 3:3:3 (512-color) palette latches — real DS80 hardware feeds the blue LSB (BX0)
  // and PAL_DETECT self-test from separate flip-flops on the palette IC, not from
  // the #7E data byte itself. See Ports::output/Ports::input and profiPaletteWrite.
  static uint8_t profi_bx0_latch;  // last value of port #FE bit7 (BX0, write)
  static uint8_t profi_gx0_latch;  // last GX0 (bit5 of the #7E-style palette byte)
  static volatile bool profi_palette_dirty;      // pending HDMI palette refresh — applied in EndFrame
  static volatile bool profi_ds80_activate_pending;   // deferred off→on mode switch (set in Ports, applied in EndFrame)
  static volatile bool profi_ds80_deactivate_pending; // deferred on→off mode switch (set in Ports, applied in EndFrame)
  // OSD palette override: while OSD is open over a DS80 screen, conv_color is
  // temporarily switched to the standard ZX palette so OSD colour bytes (0..16)
  // render correctly.  HDMI stays in DS80 mode; the frozen Profi background is
  // re-interpreted through the standard palette (accepted tradeoff — OSD always ZX).
  static bool profi_ds80_osd_active;     // true while OSD standard-palette override is in effect
  static void rebuildDS80ColorLut();     // rebuild Graphics8BitPalette::ds80_color_lut from profi_pair_lookup
  static void applyProfiOSDPalette();    // enable Graphics-layer DS80 colour remap (OSD draws correct colours)
  static void restoreProfiLivePalette(); // disable Graphics-layer DS80 colour remap
  static void discardProfiOSDPaletteSnapshot(); // drop saved palette without re-enabling DS80
  static void clearDS80Padding();        // re-blacken DS80 side-padding columns after OSD close
  static void profiPaletteReset();
  // Update palette[index] from a Profi RRRGGGBB color byte; sets dirty flag.
  static void profiPaletteWrite(uint8_t index, uint8_t profi_color);
  // Apply a pending live-palette refresh to the scanout driver. Call ONLY while
  // the display is in vertical blanking (right after the v_sync wait): the
  // conv_color rewrite races active scanout otherwise — visible as a stable
  // palette tear line during guest palette animation (Karabas-Pro tests).
  static void profiPaletteApplyPending();
#endif

  static bool isProfiDS80();
  static void updateBorderBrd(); // set VIDEO::brd correctly for current mode (DS80 or normal)

  static uint16_t spectrum_colors[NUM_SPECTRUM_COLORS];

  static uint16_t offBmp[SPEC_H];
  static uint16_t offAtt[SPEC_H];

  static VGA8Bit vga;

  static uint8_t borderColor;
  static uint32_t border32[8];
  static uint32_t brd;
  static bool brdChange;
  static bool brdnextframe;
  static bool brdGigascreenChange;
  static uint32_t lastBrdTstate;

  static uint8_t tStatesPerLine;
  static int tStatesScreen;
  static int tStatesBorder;  

  static uint8_t flashing;
  static uint8_t flash_ctr;

  static uint8_t att1;
  static uint8_t bmp1;
  static uint8_t att2;
  static uint8_t bmp2;
  // static bool opCodeFetch;

  static uint8_t dispUpdCycle;
  static bool snow_att;
  static bool dbl_att;
  static uint8_t lastbmp;
  static uint8_t lastatt;    
  static uint8_t snowpage;
  static uint8_t snowR;
  static bool snow_toggle;
  
  #ifdef DIRTY_LINES
  static uint8_t dirty_lines[SPEC_H];
  // static uint8_t linecalc[SPEC_H];
  #endif // DIRTY_LINES
 
  static uint8_t OSD;

  static SaveRectT SaveRect;

///  static TaskHandle_t videoTaskHandle;

  static int VsyncFinetune[2];

  static uint32_t framecnt; // Frames elapsed

  static int video_mode;

  // Video mode helper methods
  static uint8_t activeVideoMode() {
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    extern bool SELECT_VGA;
    return SELECT_VGA ? Config::vga_video_mode : Config::hdmi_video_mode;
#else
    return Config::hdmi_video_mode;
#endif
  }
  static bool isFullBorderMode() { return activeVideoMode() >= Config::VM_720x480_60; }
  static bool isFullBorder240()  { return activeVideoMode() == Config::VM_720x480_60; }
  static bool isFullBorder288()  { return activeVideoMode() >= Config::VM_720x576_50; }

  static bool gigascreen_enabled;
  static uint8_t gigascreen_auto_countdown;

  // Profi has no Gigascreen (incompatible video path). Force it off and free
  // the 52 KB prev-FB. Safe to call when arch is not Profi (no-op). Called both
  // at boot (VIDEO::Init) and on a runtime switch into Profi.
  static void disableGigascreenForProfi();

  // Timex SCLD video modes
#if !PICO_RP2040
  static uint8_t timex_port_ff;   // bits 0-5 of port 0xFF
  static uint8_t timex_mode;      // cached (timex_port_ff & 7)
  static uint8_t timex_hires_ink; // mode 6: ink palette index (0-7)
#endif

  // ULA+
#if !PICO_RP2040
  static bool ulaplus_enabled;
  static uint8_t ulaplus_reg;
  static uint8_t ulaplus_palette[64];
  static bool ulaplus_palette_dirty;  // deferred palette flush for HDMI sync
  static bool ulaplus_alubytes_dirty; // deferred AluByte/palette rebuild for HDMI sync
  // AluBytesUlaPlus moved to flash (AluBytesUlaPlus_flash in roms/AluBytesUlaPlus.c)
  static void regenerateUlaPlusAluBytes();
  static void ulaPlusUpdatePaletteEntry(uint8_t entry);
  static void ulaPlusFlushPalette();   // apply pending palette to hardware
  static void ulaPlusUpdateBorder();
  static void ulaPlusDisable();

  // 16col mode (Pentagon, Alone Coder): 4bpp packed, no attributes.
  // Enabled via port #EFF7 bit D0.
  static bool mode16col_enabled;
  static const uint8_t* mode16col_planes[4]; // base ptrs to 4 6144-byte planes
  static void mode16colUpdatePlanes();
  static void ensure16colLut();  // alloc+build the 512 B decode LUT (no-op if present)
  static void free16colLut();    // release the decode LUT — 16col costs 0 SRAM when off
#endif

  // Palette transform (Default, Grayscale, etc.)
  static void applyPalette();

  // Fill 256-entry BMP palette (1024 bytes, BGRA format) matching current VGA palette
  static void getBmpPalette(uint8_t* out);

  // Custom palettes loaded from /palette.nvs
  static void loadCustomPalettes();
  static uint8_t paletteCount();           // built-in + custom
  static const char* paletteName(uint8_t idx); // name for menu display
};

#define zxColor(color,bright) VIDEO::spectrum_colors[bright ? color + 8 : color]

#endif // VIDEO_h
