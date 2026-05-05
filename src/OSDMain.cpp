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
#include <malloc.h>
#include <hardware/watchdog.h>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/vreg.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>

#include "OSDMain.h"
#include "FileUtils.h"
#include "CPU.h"
#include "Video.h"
#include "ESPectrum.h"
#include "messages.h"
#include "Config.h"
#include "Debug.h"
#include "Snapshot.h"
#include "MemESP.h"
#include "Tape.h"
#include "ZipExtract.h"
#include "pwm_audio.h"
#include "Z80_JLS/z80.h"
#include "roms.h"
#include "ff.h"
#include "diskio.h"
#include "psram_spi.h"
#include "Ports.h"
#include "audio.h"
#include "AySound.h"
#include "Midi.h"
#include "MidiSynth.h"
#include "kbd_img.h"
extern "C" void graphics_set_scanlines(bool enabled);
extern "C" void graphics_set_dither(bool enabled);
#if !PICO_RP2040
#include "DivMMC.h"
#include "MB02.h"
#endif

#include <malloc.h>

#include "PinSerialData_595.h"

#include <string>
#include <cstdio>

extern "C" uint8_t TFT_FLAGS;
extern "C" uint8_t TFT_INVERSION;

void fputs(const char* b, FIL& f);

using namespace std;

#define MENU_REDRAW true
#define MENU_UPDATE false
#define OSD_ERROR true
#define OSD_NORMAL false

#define OSD_W 248
#define OSD_H 200
#define OSD_MARGIN 4

extern Font Font6x8;
#ifdef VGA_HDMI
extern bool SELECT_VGA;
#endif

extern int ram_pages, butter_pages, psram_pages, swap_pages;

// Shared buffer for HWInfo/ChipInfo/BoardInfo/EmulatorInfo (never called concurrently)
#define OSD_INFO_BUF_SZ 1536
static char osd_info_buf[OSD_INFO_BUF_SZ];

uint8_t OSD::cols;                     // Maximum columns
uint8_t OSD::tab_col;                  // Tab stop column
uint8_t OSD::max_right;                // Longest right part (hotkeys only)
uint8_t OSD::mf_rows;                  // File menu maximum rows
unsigned short OSD::real_rows;      // Real row count
uint8_t OSD::virtual_rows;             // Virtual maximum rows on screen
uint16_t OSD::w;                        // Width in pixels
uint16_t OSD::h;                        // Height in pixels
uint16_t OSD::x;                        // X vertical position
uint16_t OSD::y;                        // Y horizontal position
uint16_t OSD::prev_y[5];                // Y prev. position
unsigned short OSD::menu_prevopt = 1;
string OSD::menu;                   // Menu string
unsigned short OSD::begin_row = 1;      // First real displayed row
uint8_t OSD::focus = 1;                    // Focused virtual row
uint8_t OSD::last_focus = 0;               // To check for changes
unsigned short OSD::last_begin_row = 0; // To check for changes

uint8_t OSD::menu_level = 0;
bool OSD::menu_saverect = false;
unsigned short OSD::menu_curopt = 1;
bool OSD::menu_del_pressed = false;
bool OSD::menu_rename_pressed = false;
bool OSD::menu_quicksave_pressed = false;
bool OSD::menu_quickload_pressed = false;
string OSD::menu_footer = "";

unsigned short OSD::scrW = 320;
unsigned short OSD::scrH = 240;

char OSD::stats_lin1[25]; // "CPU: 00000 / IDL: 00000 ";
char OSD::stats_lin2[25]; // "FPS:000.00 / FND:000.00 ";

// // X origin to center an element with pixel_width
unsigned short OSD::scrAlignCenterX(unsigned short pixel_width) { return (scrW / 2) - (pixel_width / 2); }

// // Y origin to center an element with pixel_height
unsigned short OSD::scrAlignCenterY(unsigned short pixel_height) { return (scrH / 2) - (pixel_height / 2); }

// Inline text editor — edits text directly at pixel position (ex, ey) in the current window.
// Draws each character individually; cursor shown as highlighted block under current char.
// Returns entered string on Enter, "\x1B" on Escape, "" if Enter pressed with empty field.
// Ignores VK_MENU_* synthetic events to avoid double-fires from kbdExtraMapping.
string OSD::inlineTextEdit(int ex, int ey, int maxlen, const string& initial_text) {
    string text = initial_text;
    auto Kbd = ESPectrum::PS2Controller.keyboard();
    // Drain any keys still in the queue (e.g. the Enter that triggered the save action)
    { fabgl::VirtualKeyItem drain; while (Kbd->virtualKeyAvailable()) Kbd->getNextVirtualKey(&drain); }
    VIDEO::vga.setFont(Font6x8);

    uint8_t blinkCtr = 7; // triggers first draw immediately (++&0x7==0)

    auto redraw = [&](bool cursorOn) {
        string display = text;
        while ((int)display.length() < maxlen) display += ' ';
        int cur = (int)text.length();
        for (int p = 0; p < maxlen; p++) {
            bool isCursor = (p == cur && cur < maxlen) || (p == maxlen - 1 && cur >= maxlen);
            if (isCursor && cursorOn)
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 1));
            else
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
            VIDEO::vga.setCursor(ex + p * OSD_FONT_W, ey);
            char ch[2] = { display[p], 0 };
            VIDEO::vga.print(ch);
        }
    };

    redraw(true);

    while (1) {
        // Blink cursor while waiting for keypress
        while (!Kbd->virtualKeyAvailable()) {
            sleep_ms(5);
            if ((++blinkCtr & 0x7) == 0)
                redraw((blinkCtr & 0x20) == 0);
        }
        blinkCtr = 7; // next tick redraws with cursor on after keypress
        fabgl::VirtualKeyItem ek;
        Kbd->getNextVirtualKey(&ek);
        if (!ek.down) continue;
        // Skip synthetic VK_MENU_* events
        if (ek.vk >= fabgl::VK_MENU_UP && ek.vk <= fabgl::VK_MENU_BS) continue;
        if (ek.vk == fabgl::VK_RETURN || ek.vk == fabgl::VK_KP_ENTER) return text;
        if (ek.vk == fabgl::VK_ESCAPE) return "\x1B";
        if (ek.vk == fabgl::VK_BACKSPACE) {
            if (!text.empty()) { text.pop_back(); redraw(true); }
        } else if (ek.ASCII >= 32 && ek.ASCII < 127) {
            if ((int)text.length() < maxlen) {
                char c = ek.ASCII;
                if (c >= 'A' && c <= 'Z') {
                    bool shift = Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT);
                    bool caps = Kbd->isVKDown(fabgl::VK_CAPSLOCK);
                    if (!shift && !caps) c = c - 'A' + 'a';
                }
                text += c;
                redraw(true);
            }
        }
    }
}

uint8_t OSD::osdMaxRows() { return (OSD_H - (OSD_MARGIN * 2)) / OSD_FONT_H; }
uint8_t OSD::osdMaxCols() { return (OSD_W - (OSD_MARGIN * 2)) / OSD_FONT_W; }
unsigned short OSD::osdInsideX() { return scrAlignCenterX(OSD_W) + OSD_MARGIN; }
unsigned short OSD::osdInsideY() { return scrAlignCenterY(OSD_H) + OSD_MARGIN; }

static const uint8_t click48[12] = { 0,8,32,32,32,32,32,32,32,32,8,0 };

static const uint8_t click128[116] = {   0,8,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                                        32,32,8,0
                                    };

IRAM_ATTR void OSD::click() {
    if (Config::tape_player /*|| Config::real_player*/)
        return; // Disable interface click on tape player mode
    pwm_audio_set_volume(ESP_VOLUME_MAX);
    if (Z80Ops::is48)
        pwm_audio_write((uint8_t*) click48, (uint8_t*) click48, 12, 0, 0);
    else
        pwm_audio_write((uint8_t*) click128, (uint8_t*) click128, 116, 0, 0);
    pwm_audio_set_volume(ESPectrum::aud_volume);
    if (CPU::paused) osdCenteredMsg(OSD_PAUSE[Config::lang], LEVEL_INFO, 0);
}
void close_all(void);
void OSD::esp_hard_reset() {
    if (Config::audio_driver == 3) send_to_595(LOW(AY_Enable));
    close_all();
    watchdog_enable(1, true);
    while (true);
}

static bool confirmReboot(const char* const dlg[2]) {
    return OSD::msgDialog("", dlg[Config::lang]) == DLG_YES;
}

// // Cursor to OSD first row,col
void OSD::osdHome() { VIDEO::vga.setCursor(osdInsideX(), osdInsideY()); }

// // Cursor positioning
void OSD::osdAt(uint8_t row, uint8_t col) {
    if (row > osdMaxRows() - 1)
        row = 0;
    if (col > osdMaxCols() - 1)
        col = 0;
    unsigned short y = (row * OSD_FONT_H) + osdInsideY();
    unsigned short x = (col * OSD_FONT_W) + osdInsideX();
    VIDEO::vga.setCursor(x, y);
}

void OSD::drawOSD(bool bottom_info) {
    unsigned short x = scrAlignCenterX(OSD_W);
    unsigned short y = scrAlignCenterY(OSD_H);
    VIDEO::vga.fillRect(x, y, OSD_W, OSD_H, zxColor(1, 0));
    VIDEO::vga.rect(x, y, OSD_W, OSD_H, zxColor(0, 0));
    VIDEO::vga.rect(x + 1, y + 1, OSD_W - 2, OSD_H - 2, zxColor(7, 0));
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 1));
    VIDEO::vga.setFont(Font6x8);
    osdHome();
    VIDEO::vga.print(OSD_TITLE);
    osdAt(23, 0);
    if (bottom_info) {
        string bottom_line;
#ifdef VGA_HDMI
    {
        uint8_t vm = SELECT_VGA ? Config::vga_video_mode : Config::hdmi_video_mode;
        const char* vmname;
        switch (vm) {
            case Config::VM_640x480_60: vmname = "640x480@60Hz"; break;
            case Config::VM_640x480_50: vmname = "640x480@50Hz"; break;
            case Config::VM_720x480_60: vmname = "720x480@60Hz"; break;
            case Config::VM_720x576_60: vmname = "720x576@60Hz"; break;
            case Config::VM_720x576_50: vmname = "720x576@50Hz"; break;
            default:                    vmname = "unknown";      break;
        }
        char buf2[41];
        snprintf(buf2, sizeof(buf2), " Video: %s %s  ",
                 (SELECT_VGA ? "VGA" : "HDMI"), vmname);
        bottom_line = buf2;
    }
#else
#ifdef TV
        bottom_line = " Video mode: TV RGBI PAL   ";
#endif
#ifdef SOFTTV
        bottom_line = " Video mode: TV-composite  ";
#endif
#ifdef TFT
#ifdef ILI9341
        bottom_line = TFT_INVERSION ? " Video mode: ILI9341I      " : " Video mode: ILI9341       ";
#else 
        bottom_line = TFT_INVERSION ? " Video mode: ST7789I       " : " Video mode: ST7789        ";
#endif
#endif
#endif
        VIDEO::vga.print(bottom_line.append(EMU_VERSION).c_str());
    } else VIDEO::vga.print(OSD_BOTTOM);
    osdHome();
}

void OSD::drawStats() {

    unsigned short x,y;

    if (Config::aspect_16_9) {
        x = 156;
        y = 176;
    } else if (VIDEO::isFullBorder288()) {
        x = 188;
        y = 268;
    } else if (VIDEO::isFullBorder240()) {
        x = 188;
        y = 220;
    } else {
        x = 168;
        y = 220;
    }

    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor( ESPectrum::maxSpeed ? 5 : ESPectrum::multiplicator + 1, 0));
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(x,y);
    VIDEO::vga.print(stats_lin1);
    VIDEO::vga.setCursor(x,y+8);
    VIDEO::vga.print(stats_lin2);
}

void OSD::clearStats() {

    uint16_t brd16 = (uint16_t)VIDEO::brd;

    if (VIDEO::isFullBorder288()) {
        // full border 360x288: stats at x=188, y=268, 144x16px, uint16_t framebuffer
        for (int line = 268; line < 284; line++) {
            uint16_t *ptr = (uint16_t *)(VIDEO::vga.frameBuffer[line]);
            for (int col = 188; col < 332; col++)
                ptr[col ^ 1] = brd16;
        }
    } else if (VIDEO::isFullBorder240()) {
        // half border 360x240: stats at x=188, y=220, 144x16px, uint16_t framebuffer
        for (int line = 220; line < 236; line++) {
            uint16_t *ptr = (uint16_t *)(VIDEO::vga.frameBuffer[line]);
            for (int col = 188; col < 332; col++)
                ptr[col ^ 1] = brd16;
        }
    } else if (Config::aspect_16_9) {
        uint32_t brdColor = VIDEO::brd;
        for (int line = 176; line < 192; line++) {
            uint32_t *ptr = (uint32_t *)(VIDEO::vga.frameBuffer[line]) + 5;
            for (int col = 21; col < 39; col++) {
                ptr[col * 2] = brdColor;
                ptr[col * 2 + 1] = brdColor;
            }
        }
    } else if (Z80Ops::isPentagon) {
        for (int line = 220; line < 236; line++) {
            uint16_t *ptr = (uint16_t *)(VIDEO::vga.frameBuffer[line]);
            for (int col = 84; col < 156; col++)
                ptr[col ^ 1] = brd16;
        }
    } else {
        uint32_t brdColor = VIDEO::brd;
        for (int line = 220; line < 236; line++) {
            uint32_t *ptr = (uint32_t *)(VIDEO::vga.frameBuffer[line]);
            for (int col = 21; col < 39; col++) {
                ptr[col * 2] = brdColor;
                ptr[col * 2 + 1] = brdColor;
            }
        }
    }
}

// Forward-declare the local f_gets wrapper (defined further below)
static void f_gets(char* b, size_t sz, FIL& f);
// Forward-declare slotInlineEdit (defined further below)
static string slotInlineEdit(uint8_t opt2, const string& current);


// Get the base name (no extension) of the currently loaded tape or disk
static string getDefaultSnapshotName() {
    // Try tape first
    if (Tape::tapeFileName != "none" && !Tape::tapeFileName.empty()) {
        string name = Tape::tapeFileName;
        // Strip directory
        size_t sl = name.rfind('/');
        if (sl != string::npos) name = name.substr(sl + 1);
        // Strip extension
        size_t dot = name.rfind('.');
        if (dot != string::npos) name = name.substr(0, dot);
        if (!name.empty()) return name;
    }
    // Try disk drive 0
    if (ESPectrum::fdd.disk[0] && !ESPectrum::fdd.disk[0]->fname.empty()) {
        string name = ESPectrum::fdd.disk[0]->fname;
        size_t sl = name.rfind('/');
        if (sl != string::npos) name = name.substr(sl + 1);
        size_t dot = name.rfind('.');
        if (dot != string::npos) name = name.substr(0, dot);
        if (!name.empty()) return name;
    }
    return "";
}

// Read slot name (3rd line) from .esp info file.
// Returns "" if slot file doesn't exist, "\x01" if file exists but has no name.
static string getSlotName(uint8_t slotnumber) {
    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);
    string finfo = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfinfo;
    FIL* f = fopen2(finfo.c_str(), FA_READ);
    if (!f) return "";
    char buf[64];
    // Skip arch line
    f_gets(buf, sizeof(buf), *f);
    // Skip romset line
    f_gets(buf, sizeof(buf), *f);
    // Read name line
    buf[0] = 0;
    f_gets(buf, sizeof(buf), *f);
    fclose2(f);
    if (buf[0] == 0) return "\x01";  // file exists, no name stored
    return string(buf);
}

// Delete both .sna and .esp files for a slot
static void persistDelete(uint8_t slotnumber) {
    char persistfname[sizeof(DISK_PSNA_FILE) + 7];
    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];
    sprintf(persistfname, DISK_PSNA_FILE "%u.sna", slotnumber);
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);
    string fsna  = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfname;
    string finfo = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfinfo;
    f_unlink(fsna.c_str());
    f_unlink(finfo.c_str());
}

// Confirm and delete slot; returns true if deleted
static bool persistDeleteConfirm(uint8_t slotnumber) {
    string name = getSlotName(slotnumber);
    if (name.empty()) return false;  // already empty, nothing to do
    char buf[8];
    sprintf(buf, "#%02u", slotnumber);
    string displayName = (name == "\x01") ? "[No Name]" : name;
    string msg = string(buf) + " " + displayName + "?";
    if (OSD::msgDialog(Config::lang ? "Borrar ranura" : "Delete slot", msg) != DLG_YES) return false;
    persistDelete(slotnumber);
    return true;
}

// Rename slot: inline-edit the name in the menu row, rewrite .esp file
static void persistRename(uint8_t slotnumber, uint8_t opt2) {
    string name = getSlotName(slotnumber);
    if (name.empty()) return;  // slot is empty, nothing to rename
    string newName = slotInlineEdit(opt2, name == "\x01" ? "" : name);
    if (newName == "\x1B") return;  // Esc = cancel

    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);
    string finfo = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfinfo;

    // Read existing arch + romset lines
    FIL* f = fopen2(finfo.c_str(), FA_READ);
    if (!f) return;
    char arch[64], romset[64];
    f_gets(arch, sizeof(arch), *f);
    f_gets(romset, sizeof(romset), *f);
    fclose2(f);

    // Rewrite with new name
    f = fopen2(finfo.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) return;
    fputs((string(arch) + "\n" + string(romset) + "\n" + newName + "\n").c_str(), *f);
    fclose2(f);
}

// Build slot menu label: "#NN - Name" or "#NN - [Empty]", padded to fixed width
static string slotLabel(uint8_t i) {
    char buf[8];
    sprintf(buf, "#%02u - ", i);
    string name = getSlotName(i);
    if (name.empty()) name = "[Empty]";
    else if (name == "\x01") name = "[No Name]";
    if (name.length() > 20) name = name.substr(0, 20);
    // Pad to fixed 20 chars so menu width never changes between redraws
    while (name.length() < 20) name += ' ';
    return string(buf) + name;
}

// Build a slot menu string for given count of slots (title is first line)
static string buildSlotMenu(const char* title, uint8_t count) {
    string menu = title;
    for (uint8_t i = 1; i <= count; i++) {
        menu += slotLabel(i) + "\n";
    }
    return menu;
}

// Inline-edit the name for slot opt2 directly inside the already-drawn menu row.
// "#NN - " is 6 chars; row margin is 1 space on left. Name field = 20 chars.
// Returns the new name, or "" if cancelled.
static string slotInlineEdit(uint8_t opt2, const string& current) {
    // Virtual row = opt2 - begin_row + 1  (begin_row is 1-based real row of first visible line)
    uint8_t vrow = (uint8_t)(opt2 - OSD::begin_row + 1);
    // Pixel position of the name field: x + 1 space margin + 6 chars of "#NN - "
    int ex = OSD::x + (1 + 6) * OSD_FONT_W;
    int ey = OSD::y + 1 + vrow * OSD_FONT_H;
    // Strip trailing spaces from current name
    string name = current;
    while (!name.empty() && name.back() == ' ') name.pop_back();
    if (name == "[No Name]" || name == "[Empty]") name = "";
    return OSD::inlineTextEdit(ex, ey, 20, name);
}


static bool persistSave(uint8_t slotnumber, uint8_t opt2, bool quicksave = false)
{
    FILINFO stat_buf;
    char persistfname[sizeof(DISK_PSNA_FILE) + 7];
    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];
    sprintf(persistfname, DISK_PSNA_FILE "%u.sna", slotnumber);
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);
    string finfo = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfinfo;

    string slotName;

    // Slot isn't void
    if (f_stat(finfo.c_str(), &stat_buf) == FR_OK) {
        if (!quicksave) {
            string title = OSD_PSNA_SAVE[Config::lang];
            string msg = OSD_PSNA_EXISTS[Config::lang];
            uint8_t res = OSD::msgDialog(title, msg);
            if (res != DLG_YES) return false;
        }
        slotName = getSlotName(slotnumber);
        if (slotName == "\x01") slotName = "";
    } else {
        if (!quicksave) {
            // Empty slot: inline-edit name pre-filled with tape/disk name
            string defaultName = getDefaultSnapshotName();
            slotName = slotInlineEdit(opt2, defaultName);
            if (slotName == "\x1B") return false;  // Esc = cancel save
            if (slotName.empty()) slotName = defaultName;  // Enter on empty = use default
        } else {
            slotName = getDefaultSnapshotName();
        }
    }

    OSD::osdCenteredMsg(OSD_PSNA_SAVING, LEVEL_INFO, 500);

    // Save info file
    FIL* f = fopen2(finfo.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) {
        OSD::osdCenteredMsg(finfo + " - unable to open", LEVEL_ERROR, 5000);
        return false;
    }
    fputs((Config::arch + "\n" + Config::romSet + "\n" + slotName + "\n").c_str(), *f);
    fclose2(f);

    string fsna = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfname;
    if (!FileSNA::save(fsna)) {
        OSD::osdCenteredMsg(OSD_PSNA_SAVE_ERR, LEVEL_ERROR, 5000);
    }
    return true;
}

static void f_gets(char* b, size_t sz, FIL& f) {
    UINT br;
    char* bi = b;
    for (size_t i = 0; i < sz; ++i, ++bi) {
        if ( f_read(&f, bi, 1, &br) != FR_OK || br != 1 ) {
            *bi = 0;
            return;
        }
        if (*bi == '\r') {
            --bi;
            continue;
        }
        if (*bi == '\n') {
            *bi = 0;
            return;
        }
        if (*bi == 0) return;
    }
    b[sz - 1] = 0;
}

static bool persistLoad(uint8_t slotnumber)
{
    char persistfname[sizeof(DISK_PSNA_FILE) + 7];
    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];

    sprintf(persistfname, DISK_PSNA_FILE "%u.sna", slotnumber);
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);

    if (!FileSNA::isPersistAvailable(FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfname)) {
        OSD::osdCenteredMsg(OSD_PSNA_NOT_AVAIL, LEVEL_INFO);
        return false;
    } else {
        // Read info file
        string finfo = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfinfo;
        FIL* f = fopen2(finfo.c_str(), FA_READ);
        if (!f) {
            OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
            // printf("Error opening %s\n",persistfinfo);
            return false;
        }
        char buf[256];
        f_gets(buf, sizeof(buf), *f);
        string persist_arch = buf;
        f_gets(buf, sizeof(buf), *f);
        string persist_romset = buf;
        fclose2(f);

        if (!LoadSnapshot(FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfname, persist_arch, persist_romset)) {
            OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
            return false;
        }
        else
        {
            Config::ram_file = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfname;
            Config::last_ram_file = Config::ram_file;
        }
    }

    return true;

}

#define MADCTL_BGR_PIXEL_ORDER (1<<3)
#define MADCTL_ROW_COLUMN_EXCHANGE (1<<5)
#define MADCTL_COLUMN_ADDRESS_ORDER_SWAP (1<<6)
#define MADCTL_MY  (1 << 7) // Row Address Order (Y flip)
#define MADCTL_MX  (1 << 6) // Column Address Order (X flip)

string getMenuPrefix() {
    if (MEM_PG_CNT <= 64) return "ESPectrum ";
    if (MEM_PG_CNT <= 256) return "Murmuzavr 4M/";
    if (MEM_PG_CNT <= 512) return "Murmuzavr 8M/";
    if (MEM_PG_CNT <= 1024) return "Murmuzavr 16M/";
    return "Murmuzavr 32M/";
}

// Forward declarations for hotkey helpers (defined later in file)
static string hkBindingText(int idx);
static string expandHotkeys(const char* menu);
extern const char* const hkDescEN[];
extern const char* const hkDescES[];

// OSD Main Loop
void OSD::do_OSD(fabgl::VirtualKey KeytoESP, bool ALT, bool CTRL) {

    struct AYGuard {
        AYGuard()  { if (Config::audio_driver == 3) send_to_595(LOW(AY_Enable)); }
        ~AYGuard() { if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable)); }
    } ayGuard;

    static uint8_t last_sna_row = 0;
    fabgl::VirtualKeyItem Nextkey;

    // Find matching configurable hotkey
    int hkIdx = -1;
    for (int i = 0; i < Config::HK_COUNT; i++) {
        if (Config::hotkeys[i].vk != (uint16_t)fabgl::VK_NONE &&
            Config::hotkeys[i].vk == (uint16_t)KeytoESP &&
            Config::hotkeys[i].alt  == ALT &&
            Config::hotkeys[i].ctrl == CTRL) {
            hkIdx = i;
            break;
        }
    }

#ifdef VGA_HDMI
    if (hkIdx == Config::HK_VIDMODE_60) { // HDMI 60Hz
        uint8_t &vm = SELECT_VGA ? Config::vga_video_mode : Config::hdmi_video_mode;
        if (vm == Config::VM_640x480_60) return;
        uint8_t saved_vm = vm;
        vm = Config::VM_640x480_60;
        Config::save();
        VIDEO::changeMode();
        if (!videoModeConfirm(10)) {
            vm = saved_vm;
            Config::save();
            VIDEO::changeMode();
        }
    } else
    if (hkIdx == Config::HK_VIDMODE_50) { // HDMI 50Hz
        uint8_t &vm = SELECT_VGA ? Config::vga_video_mode : Config::hdmi_video_mode;
        if (vm == Config::VM_640x480_50) return;
        uint8_t saved_vm = vm;
        vm = Config::VM_640x480_50;
        Config::save();
        VIDEO::changeMode();
        if (!videoModeConfirm(10)) {
            vm = saved_vm;
            Config::save();
            VIDEO::changeMode();
        }
    } else
#endif
    if (hkIdx == Config::HK_HW_INFO) { // Show mem info
            OSD::HWInfo();
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        } else
        if (hkIdx == Config::HK_TURBO) { // Turbo mode
            ESPectrum::multiplicator += 1;
            if (ESPectrum::multiplicator > 3) {
                ESPectrum::multiplicator = 0;
            }
            CPU::updateStatesInFrame();
        } else
        if (hkIdx == Config::HK_DEBUG) {
            osdDebug();
        }
        else if (hkIdx == Config::HK_BP_LIST) {
            uint16_t bpAddr = BPListDialog();
            if (bpAddr != 0xFFFF) osdDebug(bpAddr);
        } else
        if (hkIdx == Config::HK_JUMP_TO) {
            jumpToDialog();
        } else
        if (hkIdx == Config::HK_POKE) { // Input Poke
            pokeDialog();
        } else
        if (hkIdx == Config::HK_NMI) { // NMI
#if !PICO_RP2040
            if (DivMMC::enabled) {
                // DivMMC NMI: automap at 0x0066 handled by preOpcFetch/postOpcFetch
                Z80::triggerNMI();
            } else
#endif
            if (Z80Ops::isByte) {
                // ZX Byte: NMI menu with COBMECT mode toggle
                menu_level = 0;
                menu_curopt = 1;
                menu_saverect = true;
                string nmi_menu = MENU_NMI_TITLE[Config::lang];
                nmi_menu += "NMI\n";
                nmi_menu += MENU_BYTE_COBMECT_MODE[Config::lang];
                uint8_t nmi_cols = 20;
                uint16_t nmi_w = (nmi_cols * OSD_FONT_W) + 2;
                uint16_t nmi_h = (rowCount(nmi_menu) * OSD_FONT_H) + 2;
                uint8_t opt = simpleMenuRun(nmi_menu,
                    scrAlignCenterX(nmi_w), scrAlignCenterY(nmi_h),
                    rowCount(nmi_menu), nmi_cols);
                if (opt == 1) {
                    Z80::triggerNMI();
                } else if (opt == 2) {
                    Config::byte_cobmect_mode = !Config::byte_cobmect_mode;
                    Config::save();
                    MemESP::rom[0].assign_rom(Config::byte_cobmect_mode ? gb_rom_0_byte_sovmest_48k : gb_rom_0_byte_48k);
                    MemESP::recoverPage0();
                    osdCenteredMsg(Config::byte_cobmect_mode ? OSD_COBMECT_ON[Config::lang] : OSD_COBMECT_OFF[Config::lang], LEVEL_INFO, 500);
                }
            } else if (Z80Ops::isPentagon) {
                menu_level = 0;
                menu_curopt = 1;
                menu_saverect = true;
                string nmi_menu = MENU_NMI_TITLE[Config::lang];
                nmi_menu += MENU_NMI_SEL[Config::lang];
                uint8_t nmi_cols = 20;
                uint16_t nmi_w = (nmi_cols * OSD_FONT_W) + 2;
                uint16_t nmi_h = (rowCount(nmi_menu) * OSD_FONT_H) + 2;
                uint8_t opt = simpleMenuRun(nmi_menu,
                    scrAlignCenterX(nmi_w), scrAlignCenterY(nmi_h),
                    rowCount(nmi_menu), nmi_cols);
                if (opt == 1)
                    Z80::triggerNMI();
                else if (opt == 2)
                    Z80::triggerNMIDOS();
            } else {
                Z80::triggerNMI();
            }
        }
        else
        if (hkIdx == Config::HK_RESET_TO) { // Reset to...
#if !PICO_RP2040
            if (DivMMC::enabled) {
                menu_level = 0;
                menu_curopt = 1;
                menu_saverect = true;
                uint8_t rst_cols = 22;
                uint16_t rst_w = (rst_cols * OSD_FONT_W) + 2;
                uint16_t rst_h = (rowCount(MENU_RESETTO_DIVMMC[Config::lang]) * OSD_FONT_H) + 2;
                uint8_t opt = simpleMenuRun(MENU_RESETTO_DIVMMC[Config::lang],
                    scrAlignCenterX(rst_w), scrAlignCenterY(rst_h),
                    rowCount(MENU_RESETTO_DIVMMC[Config::lang]), rst_cols);
                if (opt == 1) {
                    // Soft Reset: keep DivMMC RAM (ESXDOS sees 0xAA flag, goes to file browser)
                    if (Config::ram_file != NO_RAM_FILE) Config::ram_file = NO_RAM_FILE;
                    Config::last_ram_file = NO_RAM_FILE;
                    DivMMC::reset();
                    ESPectrum::reset();
                } else if (opt == 2) {
                    // Hard Reset: clear DivMMC RAM (ESXDOS re-initializes from scratch)
                    if (Config::ram_file != NO_RAM_FILE) Config::ram_file = NO_RAM_FILE;
                    Config::last_ram_file = NO_RAM_FILE;
                    DivMMC::init();
                    ESPectrum::reset();
                }
            } else
#endif
            if (Z80Ops::is48) {
                // 48K - just reset directly
                if (Config::ram_file != NO_RAM_FILE) Config::ram_file = NO_RAM_FILE;
                Config::last_ram_file = NO_RAM_FILE;
                ESPectrum::reset(0);
            } else {
                // Build machine-dependent menu
                string reset_menu;
                if (Z80Ops::isPentagon) {
                    if (Config::romSet == "128Kpg")
                        reset_menu = MENU_RESETTO_PENTGLUK[Config::lang];
                    else
                        reset_menu = MENU_RESETTO_PENT[Config::lang];
                } else {
                    reset_menu = MENU_RESETTO_128[Config::lang];
                }

                menu_level = 0;
                menu_curopt = 1;
                menu_saverect = true;
                uint8_t rst_cols = 22;
                uint16_t rst_w = (rst_cols * OSD_FONT_W) + 2;
                uint16_t rst_h = (rowCount(reset_menu) * OSD_FONT_H) + 2;
                uint8_t opt = simpleMenuRun(reset_menu,
                    scrAlignCenterX(rst_w), scrAlignCenterY(rst_h),
                    rowCount(reset_menu), rst_cols);

                if (opt > 0) {
                    if (Config::ram_file != NO_RAM_FILE) Config::ram_file = NO_RAM_FILE;
                    Config::last_ram_file = NO_RAM_FILE;

                    if (Z80Ops::isPentagon && Config::romSet == "128Kpg") {
                        // Service (Gluk)=1, TR-DOS=2, 128K=3, 48K=4
                        if (opt == 1) {
                            ESPectrum::reset(3); // Gluk ROM
                        } else if (opt == 2) {
                            ESPectrum::reset(4); // TR-DOS ROM
                            MemESP::romLatch = 1;
                            ESPectrum::trdos = true;
                        } else if (opt == 3) {
                            ESPectrum::reset(0); // 128K
                        } else if (opt == 4) {
                            ESPectrum::reset(1); // 48K BASIC ROM
                            MemESP::pagingLock = 1;
                        }
                    } else if (Z80Ops::isPentagon) {
                        // TR-DOS=1, 128K=2, 48K=3
                        if (opt == 1) {
                            ESPectrum::reset(4); // TR-DOS ROM
                            MemESP::romLatch = 1;
                            ESPectrum::trdos = true;
                        } else if (opt == 2) {
                            ESPectrum::reset(0); // 128K
                        } else if (opt == 3) {
                            ESPectrum::reset(1); // 48K BASIC ROM
                            MemESP::pagingLock = 1;
                        }
                    } else { // 128K
                        // 128K=1, 48K=2
                        if (opt == 1) {
                            ESPectrum::reset(0); // 128K
                        } else if (opt == 2) {
                            ESPectrum::reset(1); // 48K BASIC ROM
                            MemESP::pagingLock = 1;
                        }
                    }
                }
            }
        }
        else if (FileUtils::fsMount && hkIdx == Config::HK_DISK) {
#if !PICO_RP2040
            if (DivMMC::enabled) {
                menu_level = 0;
                menu_saverect = false;
                string mFile = fileDialog(FileUtils::IMG_Path, MENU_IMG_TITLE[Config::lang], DISK_IMGFILE, 51, 22);
                if (mFile != "") {
                    string fname = FileUtils::IMG_Path + mFile.substr(1);
                    if (FileUtils::getLCaseExt(fname) == "zip") {
                        string zipFname = ZipExtract::extract(fname, DISK_IMGFILE);
                        if (zipFname.empty()) OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN);
                        else if (zipFname != "\x1b") fname = zipFname;
                        else fname.clear();
                    }
                    if (!fname.empty()) {
                        diskSlotDialog(IFACE_ESX, 0, fname);
                        Config::save();
                        ESPectrum::reset();
                        return;
                    }
                }
                if (VIDEO::OSD) OSD::drawStats();
            } else
#endif
            while (1) {
                menu_level = 0;
                menu_saverect = false;
                string mFile = fileDialog(FileUtils::DSK_Path, MENU_DSK_TITLE[Config::lang], DISK_DSKFILE, 51, 22);
                if (mFile != "") {
                    string fname = FileUtils::DSK_Path + mFile.substr(1);
                    string fprefix = mFile.substr(0,1);
                    if ( fprefix == "1" || fprefix == "2" || fprefix == "3" || fprefix == "4") {

                        // Create empty trd
                        //Debug::log("Create empty trd. Prefix: %s\n",fprefix.c_str());
                        // FIL *fd = fopen2(fname.c_str(), FA_WRITE);
                        // if (!fd) {
                        //     Debug::led_blink();
                        //     break;
                        // }

                        // // TRD info for 40 tracks 2 sides -> Offset 2274, positions 1 - 4 contains disk type + number of files (0) + number of free sectors
                        // unsigned char trdheader[] = { 0x01, 0x17, 0x00, 0xf0, 0x04, 0x10, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
                        // 0x20, 0x20, 0x20, 0x00, 0x00, 0x42, 0x4c, 0x41, 0x4e, 0x4b }; //, 0x20, 0x20, 0x20 };

                        // char buffer[1024] = {0}; // Bloque de 1 KB lleno de ceros

                        // size_t to_write = 655360; // 640 KB

                        // if (fprefix == "1") {
                        //     // 80/2
                        //     trdheader[1] = 0x16;
                        //     trdheader[4] = 0x09;
                        // } else if (fprefix == "2") {
                        //     // 40/2
                        //     to_write >>= 1; // 320 KB
                        // } else if (fprefix == "3") {
                        //     // 80/1
                        //     to_write >>= 1; // 320 KB
                        //     trdheader[1] = 0x18;
                        // } else if (fprefix == "4") {
                        //     // 40/1
                        //     to_write >>= 2; // 160 KB
                        //     trdheader[1] = 0x19;
                        //     trdheader[3] = 0x70;
                        //     trdheader[4] = 0x02;
                        // }

                        // while (to_write > 0) {
                        //     size_t chunk = (to_write < sizeof(buffer)) ? to_write : sizeof(buffer);
                        //     fwrite(buffer, 1, chunk, fd);
                        //     to_write -= chunk;
                        // }

                        // // Write TRD header
                        // f_lseek(fd, 2274);
                        // fwrite(trdheader, 1, sizeof(trdheader), fd);

                        //  f_close(fd);

                        // continue;

                    }

                    string ext = FileUtils::getLCaseExt(fname);
                    if (ext == "zip") {
                        string zipFname = ZipExtract::extract(fname, DISK_DSKFILE);
                        if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); continue; }
                        if (zipFname == "\x1b") continue;
                        fname = zipFname;
                        ext = FileUtils::getLCaseExt(fname);
                    }
                    if (ext == "trd" || ext == "scl" || ext == "udi" || ext == "fdi") {
                        printf("Insert disk %s\n",fname.c_str());
                        rvmWD1793InsertDisk(&ESPectrum::fdd, 0, fname);
                    }
#if !PICO_RP2040
                    else if (ext == "mbd") {
                        printf("Insert MB-02 disk %s\n",fname.c_str());
                        if (MB02::enabled) {
                            rvmWD1793InsertDisk(&ESPectrum::mb02_fdd, 0, fname);
                            ESPectrum::mb02_fdd.diskLoadedCyl = -1;
                            ESPectrum::mb02_fdd.diskLoadedSide = -1;
                            MB02::signalDiskChange();
                        } else {
                            OSD::osdCenteredMsg("Enable MB-02+ first", LEVEL_WARN);
                        }
                    }
#endif
                    else
                    {
                        Debug::led_blink();
                    }

                    // string fname = FileUtils::DSK_Path + "/" + mFile;
                    // rvmWD1793InsertDisk(&ESPectrum::fdd, 0, fname);
                    Config::save();
                }
                break;
            }
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        }
        else if (hkIdx == Config::HK_USB_BOOT) {
            if (confirmReboot(OSD_DLG_USBBOOT)) {
                reset_usb_boot(0, 0);
                while(1);
            }
        }
        else if (hkIdx == Config::HK_GIGASCREEN) {
            if (Config::gigascreen_enabled)
            {
                Config::gigascreen_onoff = (Config::gigascreen_onoff + 1) % 3; // Off -> On -> Auto -> Off
                if (Config::gigascreen_onoff == 1) {
#if !PICO_RP2040
                    VIDEO::InitPrevBuffer(); // seed prev from current FB to avoid stale-frame flash
#endif
                    VIDEO::gigascreen_enabled = true;
                }
                else {
                    VIDEO::gigascreen_enabled = false;
                    VIDEO::gigascreen_auto_countdown = 0;
                }
                std::string menu = Config::gigascreen_onoff == 1 ? OSD_GIGASCREEN_ON[Config::lang]
                                 : Config::gigascreen_onoff == 2 ? OSD_GIGASCREEN_AUTO[Config::lang]
                                 : OSD_GIGASCREEN_OFF[Config::lang];
                osdCenteredMsg(menu, LEVEL_INFO, 500);
                Config::save();
            }
        } else if (hkIdx == Config::HK_MAX_SPEED || KeytoESP == fabgl::VK_NUMLOCK) {
        ESPectrum::maxSpeed = !ESPectrum::maxSpeed;
        std::string menu = ESPectrum::maxSpeed ? OSD_MAXSPEED_ON[Config::lang] : OSD_MAXSPEED_OFF[Config::lang];
        osdCenteredMsg(menu, LEVEL_INFO, 500);
        click();
        } else if (hkIdx == Config::HK_PAUSE) {
        CPU::paused = !CPU::paused;
        click();
        } else if (FileUtils::fsMount && hkIdx == Config::HK_LOAD_SNA) {
            menu_level = 0;
            menu_saverect = false;
            string mFile = fileDialog(FileUtils::SNA_Path, MENU_SNA_TITLE[Config::lang], DISK_SNAFILE, 51, 22);
            if (mFile != "") {
                Config::save();
                mFile.erase(0, 1);
                string fname = FileUtils::SNA_Path + mFile;
                if (FileUtils::getLCaseExt(fname) == "zip") {
                    string zipFname = ZipExtract::extract(fname, DISK_SNAFILE);
                    if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); }
                    else if (zipFname != "\x1b") fname = zipFname;
                    else fname.clear();
                }
                if (!fname.empty()) {
                    if(!LoadSnapshot(fname, "", "")) {
                        OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
                    }
                    Config::ram_file = fname;
                    Config::last_ram_file = fname;
                }
            }
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        } else if (FileUtils::fsMount && hkIdx == Config::HK_PERSIST_LOAD) {
            menu_level = 0;
            menu_curopt = Config::persist_slot;
            // Persist Load
            while (1) {
                menu_footer = Config::lang ? "F3:Cargar  F6:Renombrar  F8:Borrar" : "F3:Load  F6:Rename  F8:Remove";
                uint8_t opt2 = menuRun(buildSlotMenu(MENU_PERSIST_LOAD[Config::lang], 40));
                if (opt2) {
                    Config::persist_slot = opt2;
                    if (menu_del_pressed) {
                        persistDeleteConfirm(opt2);
                        menu_curopt = opt2;
                        continue;
                    }
                    if (menu_rename_pressed) {
                        persistRename(opt2, opt2);
                        menu_curopt = opt2;
                        continue;
                    }
                    persistLoad(opt2);
                    break;
                } else break;
            }
        } else if (FileUtils::fsMount && hkIdx == Config::HK_PERSIST_SAVE) {
            // Persist Save
            menu_level = 0;
            menu_curopt = Config::persist_slot;
            while (1) {
                menu_footer = Config::lang ? "F4:Guardar  F6:Renombrar  F8:Borrar" : "F4:Save  F6:Rename  F8:Remove";
                uint8_t opt2 = menuRun(buildSlotMenu(MENU_PERSIST_SAVE[Config::lang], 40));
                if (opt2) {
                    Config::persist_slot = opt2;
                    if (menu_del_pressed) {
                        persistDeleteConfirm(opt2);
                        menu_curopt = opt2;
                        continue;
                    }
                    if (menu_rename_pressed) {
                        persistRename(opt2, opt2);
                        menu_curopt = opt2;
                        continue;
                    }
                    if (menu_quicksave_pressed) {
                        persistSave(opt2, opt2, true);
                        return;
                    }
                    if (persistSave(opt2, opt2)) {
                        return;
                    }
                    menu_curopt = opt2;
                } else {
                    break;
                }
            }
        } else if (FileUtils::fsMount && hkIdx == Config::HK_QUICK_LOAD) {
            // Quick Load — load current persist slot without dialog (same as F3 + Enter)
            persistLoad(Config::persist_slot);
        } else if (FileUtils::fsMount && hkIdx == Config::HK_QUICK_SAVE) {
            // Quick Save — save to current persist slot without dialog (same as F4 + F4)
            persistSave(Config::persist_slot, Config::persist_slot, true);
        } else if (FileUtils::fsMount && hkIdx == Config::HK_LOAD_ANY) {
            menu_level = 0;
            menu_saverect = false;
            string mFile;
            string fname;
            string ext;
            bool fromZip = false;

            // Loop to allow re-opening fileDialog after ZIP cancel
            bool forcePopup = false;
            f5_retry:
            mFile = fileDialog(FileUtils::ALL_Path, MENU_ALL_TITLE[Config::lang], DISK_ALLFILE, 52, 22);
            if (mFile != "") {
                // X prefix = extract ZIP to current folder
                if (mFile[0] == 'X') {
                    fname = FileUtils::ALL_Path + mFile.substr(1);
                    OSD::osdCenteredMsg(OSD_ZIP_EXTRACTING[Config::lang], LEVEL_INFO, 0);
                    int count = ZipExtract::extractAll(fname, FileUtils::ALL_Path);
                    if (count > 0) {
                        char msg[40];
                        snprintf(msg, sizeof(msg), " Extracted %d file(s) ", count);
                        OSD::osdCenteredMsg(msg, LEVEL_INFO, 1000);
                    } else {
                        OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN);
                    }
                    goto f5_retry;
                }
                // P prefix = F5 pressed on a disk/image — force the slot picker.
                if (mFile[0] == 'P') forcePopup = true;
                fname = FileUtils::ALL_Path + mFile.substr(1);
                ext = FileUtils::getLCaseExt(fname);
                fromZip = false;

                // ZIP archive — extract and replace fname/ext/mFile
                if (ext == "zip") {
                    string zipFname = ZipExtract::extract(fname, DISK_ALLFILE);
                    if (zipFname.empty()) {
                        OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN);
                        if (VIDEO::OSD) OSD::drawStats();
                        return;
                    }
                    if (zipFname == "\x1b") {
                        // User cancelled ZIP selection — reopen file dialog
                        goto f5_retry;
                    }
                    fname = zipFname;
                    ext = FileUtils::getLCaseExt(fname);
                    // Reconstruct mFile with prefix for Tape::LoadTape compatibility
                    string zipBase = fname.substr(fname.rfind('/') + 1);
                    mFile = mFile.substr(0, 1) + zipBase;
                    fromZip = true;
                }

                if (ext == "tap" || ext == "tzx" || ext == "pzx" || ext == "wav" || ext == "mp3") {
                    // Tape — sync TAP_Path: /tmp/ for ZIP, ALL_Path for normal files
                    FileUtils::TAP_Path = fromZip ? "/tmp/" : FileUtils::ALL_Path;
                    Config::save();
                    Tape::LoadTape(mFile);
                }
                else if (FileUtils::ifaceForExt(ext) == IFACE_BETA) {
                    // TR-DOS disk. Enter in the browser mounts into Drive A;
                    // F5 in the browser opens the slot picker, where Enter mounts
                    // into the focused slot and the popup stays open for more
                    // operations until Esc closes it back to the file dialog.
                    if (Z80Ops::isPentagon || (Z80Ops::is128 && Z80Ops::isByte)) {
                        if (!fromZip) FileUtils::DSK_Path = FileUtils::ALL_Path;
                        if (forcePopup) {
                            Config::driveWP[0] = true;
                            diskSlotDialog(IFACE_BETA, 0, fname);
                            forcePopup = false;
                            goto f5_retry;
                        }
                        rvmWD1793InsertDisk(&ESPectrum::fdd, 0, fname);
                        if (ESPectrum::fdd.disk[0])
                            ESPectrum::fdd.disk[0]->writeprotect = Config::driveWP[0];
                        Config::save();
                    } else {
                        OSD::osdCenteredMsg(OSD_DSK_NEEDS_PENTAGON[Config::lang], LEVEL_WARN);
                    }
                }
#if !PICO_RP2040
                else if (ext == "mbd") {
                    // MB-02+ disk — Enter mounts into Drive 1, F5 opens the popup.
                    if (MB02::enabled) {
                        if (!fromZip) FileUtils::DSK_Path = FileUtils::ALL_Path;
                        if (forcePopup) {
                            Config::mb02WP[0] = true;
                            diskSlotDialog(IFACE_MB02, 0, fname);
                            forcePopup = false;
                            goto f5_retry;
                        }
                        rvmWD1793InsertDisk(&ESPectrum::mb02_fdd, 0, fname);
                        if (ESPectrum::mb02_fdd.disk[0])
                            ESPectrum::mb02_fdd.disk[0]->writeprotect = Config::mb02WP[0];
                        ESPectrum::mb02_fdd.diskLoadedCyl = -1;
                        ESPectrum::mb02_fdd.diskLoadedSide = -1;
                        MB02::signalDiskChange();
                        Config::save();
                    } else {
                        OSD::osdCenteredMsg("Enable MB-02+ first", LEVEL_WARN);
                    }
                }
#endif
                else if (ext == "sna" || ext == "z80" || ext == "p") {
                    // Snapshot
                    if (!fromZip) FileUtils::SNA_Path = FileUtils::ALL_Path;
                    Config::save();
                    if (!LoadSnapshot(fname, "", "")) {
                        OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
                    } else if (!fromZip) {
                        Config::ram_file = fname;
                        Config::last_ram_file = fname;
                    }
                }
#if !PICO_RP2040
                else if (ext == "mmc" || ext == "hdf") {
                    // DivMMC/DivIDE image — Enter loads into hd0 (slot 0); F5 opens
                    // the slot popup which mounts in-place and keeps the popup open.
                    // A full ESPectrum::reset() runs after everything is settled.
                    if (DivMMC::enabled) {
                        FileUtils::IMG_Path = FileUtils::ALL_Path;
                        if (forcePopup) {
                            diskSlotDialog(IFACE_ESX, 0, fname);
                            Config::save();
                            ESPectrum::reset();
                            forcePopup = false;
                            return;
                        }
                        Config::esxdos_hdf_image[0] = fname;
                        DivMMC::init();
                        Config::save();
                        ESPectrum::reset();
                    } else {
                        OSD::osdCenteredMsg(OSD_IMG_NEEDS_ESXDOS[Config::lang], LEVEL_WARN);
                    }
                }
#endif
            }
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        } else if (hkIdx == Config::HK_TAPE_PLAY) {
            // Start / Stop .tap reproduction
            if (Tape::tapeStatus == TAPE_STOPPED) {
                Tape::Play();
            } else {
                Tape::Stop();
            }
            click();
        } else if (hkIdx == Config::HK_TAPE_BROWSER) {
            // Tape Browser
            if (Tape::tapeFileName=="none") {
                OSD::osdCenteredMsg(OSD_TAPE_SELECT_ERR[Config::lang], LEVEL_WARN);
            } else {
                menu_level = 0;
                menu_curopt = 1;
                // int tBlock = menuTape(Tape::tapeFileName.substr(6,28));
                int tBlock = menuTape(Tape::tapeFileName.substr(0,22));
                if (tBlock >= 0) {
                    Tape::tapeCurBlock = tBlock;
                    Tape::Stop();
                }
            }
        } else if (hkIdx == Config::HK_STATS) {
            // Show / hide OnScreen Stats
            {
                uint8_t mode = VIDEO::OSD & 0x03;
                bool hasFdd = (Z80Ops::isPentagon || (Z80Ops::is128 && Z80Ops::isByte)
#if !PICO_RP2040
                                || ((Z80Ops::is48 || Z80Ops::is128) && MB02::enabled))
                        && Tape::tapeStatus != TAPE_LOADING
                    && !DivMMC::enabled
#else
                                )
                        && Tape::tapeStatus != TAPE_LOADING
#endif
                    ;
                uint8_t maxMode = hasFdd ? 3 : 2;

                if (mode == 0)
                    mode = Tape::tapeStatus == TAPE_LOADING ? 1 : 2;
                else
                    mode++;

                if (mode > maxMode) {
                    if ((VIDEO::OSD & 0x04) == 0) {
                        OSD::clearStats();
                        if (Config::aspect_16_9)
                            VIDEO::Draw_OSD169 = VIDEO::MainScreen;
                        else
                            VIDEO::Draw_OSD43 = VIDEO::BottomBorder;
                        VIDEO::brdnextframe = true;
                    }
                    VIDEO::OSD &= 0xfc;
                } else {
                    VIDEO::OSD = (VIDEO::OSD & 0xfc) | mode;
                    if ((VIDEO::OSD & 0x04) == 0) {
                        if (Config::aspect_16_9)
                            VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                        else
                            VIDEO::Draw_OSD43 = VIDEO::BottomBorder_OSD;

                        OSD::drawStats();
                    }
                    ESPectrum::TapeNameScroller = 0;
                }
            }
        } else if (hkIdx == Config::HK_VOL_DOWN) {
            if (VIDEO::OSD == 0) {
                if (Config::aspect_16_9)
                    VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                else
                    VIDEO::Draw_OSD43 = VIDEO::BottomBorder_OSD;
                VIDEO::OSD = 0x04;
            } else
                VIDEO::OSD |= 0x04;
            ESPectrum::totalseconds = 0;
            ESPectrum::totalsecondsnodelay = 0;
            VIDEO::framecnt = 0;
            if (ESPectrum::aud_volume>ESP_VOLUME_MIN) {
                ESPectrum::aud_volume--;
                pwm_audio_set_volume(ESPectrum::aud_volume);
                Config::aud_volume = ESPectrum::aud_volume;
                ESPectrum::vol_changed = true;
            }
            unsigned short x, y;
            if (Config::aspect_16_9) {
                x = 156;
                y = 180;
            } else if (VIDEO::isFullBorder288()) {
                x = 188;
                y = 272;
            } else if (VIDEO::isFullBorder240()) {
                x = 188;
                y = 224;
            } else {
                x = 168;
                y = 224;
            }
            VIDEO::vga.fillRect(x ,y - 4, 24 * 6, 16, zxColor(1, 0));
            VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
            VIDEO::vga.setFont(Font6x8);
            VIDEO::vga.setCursor(x + 4,y + 1);
            VIDEO::vga.print(Config::tape_player ? "TAP" : "VOL");
            for (int i = 0; i < ESPectrum::aud_volume + 16; i++) {
                VIDEO::vga.fillRect(x + 26 + (i * 7) , y + 1, 6, 7, zxColor( 7, 0));
            }
        } else if (hkIdx == Config::HK_VOL_UP) {
            if (VIDEO::OSD == 0) {
                if (Config::aspect_16_9)
                    VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                else
                    VIDEO::Draw_OSD43  = VIDEO::BottomBorder_OSD;
                VIDEO::OSD = 0x04;
            } else
                VIDEO::OSD |= 0x04;
            ESPectrum::totalseconds = 0;
            ESPectrum::totalsecondsnodelay = 0;
            VIDEO::framecnt = 0;
            if (ESPectrum::aud_volume<ESP_VOLUME_MAX) {
                ESPectrum::aud_volume++;
                pwm_audio_set_volume(ESPectrum::aud_volume);
                Config::aud_volume = ESPectrum::aud_volume;
                ESPectrum::vol_changed = true;
            }
            unsigned short x, y;
            if (Config::aspect_16_9) {
                x = 156;
                y = 180;
            } else if (VIDEO::isFullBorder288()) {
                x = 188;
                y = 272;
            } else if (VIDEO::isFullBorder240()) {
                x = 188;
                y = 224;
            } else {
                x = 168;
                y = 224;
            }
            VIDEO::vga.fillRect(x ,y - 4, 24 * 6, 16, zxColor(1, 0));
            VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
            VIDEO::vga.setFont(Font6x8);
            VIDEO::vga.setCursor(x + 4,y + 1);
            VIDEO::vga.print(Config::tape_player ? "TAP" : "VOL");
            for (int i = 0; i < ESPectrum::aud_volume + 16; i++) {
                VIDEO::vga.fillRect(x + 26 + (i * 7) , y + 1, 6, 7, zxColor( 7, 0));
            }
        } else if (hkIdx == Config::HK_HARD_RESET) { // Hard reset
            if (Config::ram_file != NO_RAM_FILE) {
                Config::ram_file = NO_RAM_FILE;
            }
            Config::last_ram_file = NO_RAM_FILE;
            ESPectrum::reset();
        } else if (hkIdx == Config::HK_REBOOT) { // ESP32 reset
            if (confirmReboot(OSD_DLG_REBOOT)) {
                Config::ram_file = NO_RAM_FILE;
                Config::save();
                esp_hard_reset();
            }
        } else if (hkIdx == Config::HK_MAIN_MENU) {
          menu_curopt = 1;
          while(1) {
            // Main menu
            menu_saverect = false;
            menu_level = 0;
            uint8_t opt = menuRun(getMenuPrefix() + Config::arch + "\n" +
                (!FileUtils::fsMount ? MENU_MAIN_NO_SD[Config::lang] : MENU_MAIN[Config::lang])
            );
            if (opt == 1) { // Volume
                if (VIDEO::OSD == 0) {
                    if (Config::aspect_16_9)
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                    else
                        VIDEO::Draw_OSD43 = VIDEO::BottomBorder_OSD;
                    VIDEO::OSD = 0x04;
                } else
                    VIDEO::OSD |= 0x04;
                ESPectrum::totalseconds = 0;
                ESPectrum::totalsecondsnodelay = 0;
                VIDEO::framecnt = 0;
                unsigned short x, y;
                if (Config::aspect_16_9) {
                    x = 156;
                    y = 180;
                } else {
                    x = 168;
                    y = 224;
                }
                VIDEO::vga.fillRect(x ,y - 4, 24 * 6, 16, zxColor(1, 0));
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                VIDEO::vga.setFont(Font6x8);
                VIDEO::vga.setCursor(x + 4,y + 1);
                VIDEO::vga.print(Config::tape_player ? "TAP" : "VOL");
                for (int i = 0; i < ESPectrum::aud_volume + 16; i++) {
                    VIDEO::vga.fillRect(x + 26 + (i * 7) , y + 1, 6, 7, zxColor( 7, 0));
                }
                click();
                return;
            }
            else if (opt == 2) { // Storage
                // ***********************************************************************************
                // STORAGE MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                while(1) {
                    menu_level = 1;
                    uint8_t stor_num = menuRun(FileUtils::fsMount ? MENU_STORAGE_MAIN[Config::lang] : MENU_STORAGE_MAIN_NO_SD[Config::lang]);
                    if (stor_num == 1) { // Tape
                        menu_saverect = true;
                        menu_curopt = 1;
                        while(1) {
                            menu_level = 2;
                            uint8_t tap_num = menuRun(expandHotkeys(FileUtils::fsMount ? MENU_TAPE[Config::lang] : MENU_TAPE_NO_SD[Config::lang]));
                            if (tap_num > 0) {
                                if (!FileUtils::fsMount) ++tap_num;
                                menu_level = 3;
                                menu_saverect = true;
                                if (tap_num == 1) {
                                    // Select TAP File
                                    string mFile = fileDialog(FileUtils::TAP_Path, MENU_TAP_TITLE[Config::lang],DISK_TAPFILE,28,16);
                                    if (mFile != "") {
                                        string fname = FileUtils::TAP_Path + mFile.substr(1);
                                        if (FileUtils::getLCaseExt(fname) == "zip") {
                                            string zipFname = ZipExtract::extract(fname, DISK_TAPFILE);
                                            if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); break; }
                                            if (zipFname == "\x1b") break;
                                            fname = zipFname;
                                            string zipBase = fname.substr(fname.rfind('/') + 1);
                                            mFile = mFile.substr(0, 1) + zipBase;
                                            FileUtils::TAP_Path = "/tmp/";
                                        }
                                        Config::save();
                                        Tape::LoadTape(mFile);
                                        return;
                                    }
                                }
                                else if (tap_num == 2) {
                                    // Start / Stop .tap reproduction
                                    if (Tape::tapeStatus == TAPE_STOPPED) {
                                        Tape::Play();
                                    } else {
                                        Tape::Stop();
                                    }
                                    return;
                                }
                                else if (tap_num == 3) {
                                    // Tape Browser
                                    if (Tape::tapeFileName=="none") {
                                        OSD::osdCenteredMsg(OSD_TAPE_SELECT_ERR[Config::lang], LEVEL_WARN);
                                        menu_curopt = 2;
                                        menu_saverect = false;
                                    } else {
                                        menu_level = 0;
                                        menu_saverect = false;
                                        menu_curopt = 1;
                                        int tBlock = menuTape(Tape::tapeFileName.substr(0,22));
                                        if (tBlock >= 0) {
                                            Tape::tapeCurBlock = tBlock;
                                            Tape::Stop();
                                        }
                                        return;
                                    }
                                }
                                else if (tap_num == 4) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string Mnustr = MENU_TAPEPLAYER[Config::lang];
                                        Mnustr += MENU_YESNO[Config::lang];
                                        bool prev_opt = Config::tape_player;
                                        if (prev_opt) {
                                            Mnustr.replace(Mnustr.find("[Y",0),2,"[*");
                                            Mnustr.replace(Mnustr.find("[N",0),2,"[ ");
                                        } else {
                                            Mnustr.replace(Mnustr.find("[Y",0),2,"[ ");
                                            Mnustr.replace(Mnustr.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(Mnustr);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::tape_player = true;
                                            else
                                                Config::tape_player = false;

                                            if (Config::tape_player != prev_opt) {
                                                if (Config::tape_player) {
                                                    ESPectrum::aud_volume = ESP_VOLUME_MAX;
                                                    pwm_audio_set_volume(ESPectrum::aud_volume);
                                                }
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = FileUtils::fsMount ? 4 : 3;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (tap_num == 5) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string Mnustr = MENU_TAPEPLAYER2[Config::lang];
                                        Mnustr += MENU_YESNO[Config::lang];
                                        bool prev_opt = Config::real_player;
                                        if (prev_opt) {
                                            Mnustr.replace(Mnustr.find("[Y",0),2,"[*");
                                            Mnustr.replace(Mnustr.find("[N",0),2,"[ ");
                                        } else {
                                            Mnustr.replace(Mnustr.find("[Y",0),2,"[ ");
                                            Mnustr.replace(Mnustr.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(Mnustr);
                                        if (opt2) {
                                            Config::real_player = (opt2 == 1);
                                            if (Config::real_player != prev_opt) {
                                                if (Tape::tapeStatus == TAPE_LOADING) {  // W/A
                                                    Tape::Stop();
                                                }
                                                if (Config::real_player) {
                                                    ESPectrum::aud_volume = ESP_VOLUME_MAX;
                                                    pwm_audio_set_volume(ESPectrum::aud_volume);
#if defined(PICO_RP2350) && defined(MIDI_TX_PIN) && defined(LOAD_WAV_PIO) && (LOAD_WAV_PIO == MIDI_TX_PIN)
                                                    if (Config::midi == 1 || Config::midi == 2)
                                                        osdCenteredMsg(MSG_MIDI_PIN_CONFLICT[Config::lang], LEVEL_WARN, 3000);
#endif
                                                } else {
#if LOAD_WAV_PIO
                                                    pcm_audio_in_stop();
#endif
                                                }
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = FileUtils::fsMount ? 5 : 4;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (tap_num == 6) {
                                    // Fast tape load
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string flash_menu = MENU_FLASHLOAD[Config::lang];
                                        flash_menu += MENU_YESNO[Config::lang];
                                        bool prev_flashload = Config::flashload;
                                        if (prev_flashload) {
                                            flash_menu.replace(flash_menu.find("[Y",0),2,"[*");
                                            flash_menu.replace(flash_menu.find("[N",0),2,"[ ");
                                        } else {
                                            flash_menu.replace(flash_menu.find("[Y",0),2,"[ ");
                                            flash_menu.replace(flash_menu.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(flash_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::flashload = true;
                                            else
                                                Config::flashload = false;

                                            if (Config::flashload != prev_flashload) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = FileUtils::fsMount ? 6 : 5;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (tap_num == 7) {
                                    // R.G. ROM timings
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string mnu_str = MENU_RGTIMINGS[Config::lang];
                                        mnu_str += MENU_YESNO[Config::lang];
                                        bool prev_opt = Config::tape_timing_rg;
                                        if (prev_opt) {
                                            mnu_str.replace(mnu_str.find("[Y",0),2,"[*");
                                            mnu_str.replace(mnu_str.find("[N",0),2,"[ ");
                                        } else {
                                            mnu_str.replace(mnu_str.find("[Y",0),2,"[ ");
                                            mnu_str.replace(mnu_str.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(mnu_str);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::tape_timing_rg = true;
                                            else
                                                Config::tape_timing_rg = false;

                                            if (Config::tape_timing_rg != prev_opt) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = FileUtils::fsMount ? 7 : 6;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                            } else {
                                menu_curopt = 1;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
                    else if (FileUtils::fsMount && stor_num == 2) { // Betadisk
                        menu_saverect = true;
                        menu_curopt = 1;
                        bool betaFirstDraw = true;
                        while(1) {
                            menu_level = 2;
                            // Build Betadisk root menu dynamically so drive status refreshes.
                            string betamenu = MENU_BETADISK_TITLE[Config::lang];
                            for (uint8_t i = 0; i < 4; i++) {
                                string label = string("Drive ") + MENU_BETA_DRIVE_LETTERS[i];
                                string fname = ESPectrum::fdd.disk[i] ? ESPectrum::fdd.disk[i]->fname : "";
                                betamenu += formatSlotRow(label, fname, Config::driveWP[i], true);
                                betamenu += "\n";
                            }
                            betamenu += MENU_BETADISK_TAIL[Config::lang];

                            // Save the backing rect only on the very first draw so the
                            // stack has exactly one entry to pop on Esc (otherwise the
                            // extra save'd fragment would show through). Repeat draws
                            // paint over the previous menu — it has fixed height, so
                            // no stale pixels remain.
                            menu_saverect = betaFirstDraw;
                            uint8_t dsk_num = menuRun(betamenu);
                            betaFirstDraw = false;
                            if (dsk_num > 0 && dsk_num < 5) {
                                // Per-drive submenu: Insert / Eject / Write Protect.
                                uint8_t slot = dsk_num - 1;
                                menu_saverect = true;
                                menu_curopt = 1;
                                while (1) {
                                    menu_level = 3;
                                    string drvmenu = MENU_BETADRIVE[Config::lang];
                                    drvmenu.replace(drvmenu.find("#",0),1,(string)" " + char(64 + dsk_num));
                                    // Fill WP toggle marker.
                                    size_t wpPos = drvmenu.rfind("[ ]");
                                    if (wpPos != string::npos && Config::driveWP[slot]) {
                                        drvmenu.replace(wpPos, 3, "[*]");
                                    }
                                    uint8_t opt2 = menuRun(drvmenu);
                                    if (opt2 == 1) {
                                        // Insert disk — no F5-style slot popup; slot is already chosen.
                                        menu_saverect = true;
                                        string mFile = fileDialog(FileUtils::DSK_Path, MENU_DSK_TITLE[Config::lang], DISK_DSKFILE, 26, 15);
                                        if (mFile != "") {
                                            mFile.erase(0, 1);
                                            string fname = FileUtils::DSK_Path + "/" + mFile;
                                            if (FileUtils::getLCaseExt(fname) == "zip") {
                                                string zipFname = ZipExtract::extract(fname, DISK_DSKFILE);
                                                if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); break; }
                                                if (zipFname == "\x1b") break;
                                                fname = zipFname;
                                            }
                                            rvmWD1793InsertDisk(&ESPectrum::fdd, slot, fname);
                                            if (ESPectrum::fdd.disk[slot])
                                                ESPectrum::fdd.disk[slot]->writeprotect = Config::driveWP[slot];
                                            Config::save();
                                        }
                                        // Mirror menuRun's Esc path so the drive submenu
                                        // rect is popped here and the next betamenu redraw
                                        // starts from a clean stack.
                                        VIDEO::SaveRect.restore_last();
                                        menu_saverect = false;
                                        menu_curopt = dsk_num;
                                        break;
                                    } else if (opt2 == 2) {
                                        wdDiskEject(&ESPectrum::fdd, slot);
                                        Config::save();
                                        VIDEO::SaveRect.restore_last();
                                        menu_saverect = false;
                                        menu_curopt = dsk_num;
                                        break;
                                    } else if (opt2 == 3) {
                                        // Toggle per-slot Write Protect.
                                        Config::driveWP[slot] = !Config::driveWP[slot];
                                        if (ESPectrum::fdd.disk[slot])
                                            ESPectrum::fdd.disk[slot]->writeprotect = Config::driveWP[slot];
                                        Config::save();
                                        menu_curopt = 3;
                                        menu_saverect = false;
                                    } else {
                                        // Esc from drive submenu — menuRun already restored
                                        // its backing rect. Reuse prev_y on next betamenu
                                        // redraw so the menu stays in place.
                                        menu_curopt = dsk_num;
                                        menu_saverect = false;
                                        break;
                                    }
                                }
                            }
                            else if (dsk_num == 5) {
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_FASTMODE[Config::lang];
                                    menu += MENU_YESNO[Config::lang];
                                    uint8_t prev = Config::trdosFastMode;
                                    if (prev) {
                                        menu.replace(menu.find("[Y",0),2,"[*");
                                        menu.replace(menu.find("[N",0),2,"[ ");
                                    } else {
                                        menu.replace(menu.find("[Y",0),2,"[ ");
                                        menu.replace(menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        Config::trdosFastMode = (opt2 == 1);
                                        if (Config::trdosFastMode != prev) {
                                            ESPectrum::fdd.fastmode = Config::trdosFastMode;
                                            Config::save();
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 5;
                                        menu_level = 2;
                                        menu_saverect = false;
                                        break;
                                    }
                                }
                            }
                            else if (dsk_num == 6) {
                                // Disk Sound & LED
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_SOUNDLED[Config::lang];
                                    menu += MENU_YESNO[Config::lang];
                                    uint8_t prev = Config::trdosSoundLed;
                                    if (prev) {
                                        menu.replace(menu.find("[Y",0),2,"[*");
                                        menu.replace(menu.find("[N",0),2,"[ ");
                                    } else {
                                        menu.replace(menu.find("[Y",0),2,"[ ");
                                        menu.replace(menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        Config::trdosSoundLed = (opt2 == 1);
                                        if (Config::trdosSoundLed != prev)
                                            Config::save();
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 6;
                                        menu_level = 2;
                                        menu_saverect = false;
                                        break;
                                    }
                                }
                            }
                            else if (dsk_num == 7) {
                                // TR-DOS ROM selector
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_TRDOS_ROM_TITLE[Config::lang];
                                    menu += MENU_TRDOS_ROM_SEL[Config::lang];
                                    int mpos = -1;
                                    int idx = 0;
                                    while ((mpos = menu.find("[ ]", mpos + 1)) != (int)string::npos) {
                                        if (idx == Config::trdosBios)
                                            menu.replace(mpos, 3, "[*]");
                                        idx++;
                                    }
                                    uint8_t prev = Config::trdosBios;
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        Config::trdosBios = opt2 - 1;
                                        if (Config::trdosBios != prev) {
                                            Config::save();
                                            switch (Config::trdosBios) {
                                                case 0: MemESP::rom[4].assign_rom(gb_rom_4_trdos_503); break;
                                                case 1: MemESP::rom[4].assign_rom(gb_rom_4_trdos_504tm); break;
                                                default: MemESP::rom[4].assign_rom(gb_rom_4_trdos_505d); break;
                                            }
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 7;
                                        menu_level = 2;
                                        menu_saverect = false;
                                        break;
                                    }
                                }
                            }
                            else {
                                menu_curopt = 2;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
#if !PICO_RP2040
                    else if (FileUtils::fsMount && stor_num == 3) { // esxDOS
                        static const char* mode_names[] = { "OFF", "DivMMC", "DivIDE", "DivSD" };
                        menu_saverect = true;
                        menu_curopt = 1;
                        while (1) {
                            menu_level = 2;
                            // Root menu: Interface row + optional hd0/hd1 rows.
                            string menu = MENU_ESXDOS_TITLE[Config::lang];
                            menu += string(MENU_ESX_INTERFACE[Config::lang]) + "\t" + mode_names[Config::esxdos] + "\n";
                            bool showHd0 = (Config::esxdos == 1 || Config::esxdos == 2);
                            bool showHd1 = (Config::esxdos == 2);
                            if (showHd0) {
                                menu += formatSlotRow("hd0", Config::esxdos_hdf_image[0], false, false);
                                menu += "\n";
                            }
                            if (showHd1) {
                                menu += formatSlotRow("hd1", Config::esxdos_hdf_image[1], false, false);
                                menu += "\n";
                            }
                            uint8_t opt = menuRun(menu);
                            if (opt == 1) {
                                // Interface submenu.
                                menu_level = 3;
                                menu_curopt = Config::esxdos + 1;
                                menu_saverect = true;
                                while (1) {
                                    string smenu = MENU_ESXDOS_TITLE[Config::lang];
                                    for (int i = 0; i < 4; i++) {
                                        smenu += (i == Config::esxdos) ? "[*] " : "[ ] ";
                                        smenu += mode_names[i];
                                        smenu += "\n";
                                    }
                                    uint8_t sub = menuRun(smenu);
                                    if (sub) {
                                        uint8_t newval = sub - 1;
                                        if (newval != Config::esxdos) {
                                            if (newval && Config::mb02) {
                                                Config::mb02 = 0;
                                                MB02::init();
                                                OSD::osdCenteredMsg("MB-02+ disabled", LEVEL_WARN, 2000);
                                            }
                                            Config::esxdos = newval;
                                            DivMMC::init();
                                            if (DivMMC::enabled && !DivMMC::rom_loaded) {
                                                OSD::osdCenteredMsg("ESXDOS ROM not found", LEVEL_ERROR, 2000);
                                                Config::esxdos = 0;
                                                DivMMC::init();
                                            }
                                            Config::save();
                                            ESPectrum::reset();
                                            return;
                                        }
                                        menu_curopt = sub;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 1;
                                        break;
                                    }
                                }
                            } else if (opt >= 2) {
                                // hd0 / hd1 submenu (Insert / Eject).
                                uint8_t slot = (opt == 2) ? 0 : 1;
                                if (!(slot == 0 && showHd0) && !(slot == 1 && showHd1)) {
                                    menu_curopt = opt;
                                    continue;
                                }
                                menu_saverect = true;
                                menu_curopt = 1;
                                while (1) {
                                    menu_level = 3;
                                    char title[8]; snprintf(title, sizeof(title), "hd%u\n", (unsigned)slot);
                                    string drvmenu = title;
                                    drvmenu += MENU_ESX_INSERT[Config::lang];
                                    drvmenu += MENU_ESX_EJECT[Config::lang];
                                    uint8_t opt2 = menuRun(drvmenu);
                                    if (opt2 == 1) {
                                        menu_saverect = true;
                                        string mFile = fileDialog(FileUtils::IMG_Path, MENU_IMG_TITLE[Config::lang], DISK_IMGFILE, 51, 22);
                                        if (mFile != "") {
                                            string fname = FileUtils::IMG_Path + mFile.substr(1);
                                            if (FileUtils::getLCaseExt(fname) == "zip") {
                                                string zipFname = ZipExtract::extract(fname, DISK_IMGFILE);
                                                if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); break; }
                                                if (zipFname == "\x1b") break;
                                                fname = zipFname;
                                            }
                                            Config::esxdos_hdf_image[slot] = fname;
                                            DivMMC::init();
                                            Config::save();
                                            ESPectrum::reset();
                                            return;
                                        }
                                    } else if (opt2 == 2) {
                                        Config::esxdos_hdf_image[slot].clear();
                                        DivMMC::init();
                                        Config::save();
                                        ESPectrum::reset();
                                        return;
                                    } else {
                                        menu_curopt = opt;
                                        break;
                                    }
                                }
                            } else {
                                menu_curopt = 3;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
                    else if (FileUtils::fsMount && stor_num == 4) { // MB-02+
                        menu_saverect = true;
                        menu_curopt = 1;
                        bool mb02FirstDraw = true;
                        while(1) {
                            menu_level = 2;
                            // Menu has a fixed row count (Mode + 4 drives + Sound & LED)
                            // so the backing-rect size never changes across Mode
                            // toggles. Drive / Sound & LED rows are dimmed (via \x01
                            // prefix) and non-selectable when Mode=Off.
                            string mb02menu = MENU_MB02_TITLE[Config::lang];
                            mb02menu += string(MENU_MB02_MODE[Config::lang]) + "\t"
                                      + (Config::mb02 ? "On" : "Off") + "\n";
                            for (int i = 0; i < 4; i++) {
                                char lab[16]; snprintf(lab, sizeof(lab), "%s %u",
                                    MENU_MB02_DRIVE[Config::lang], (unsigned)(i + 1));
                                string fname = ESPectrum::mb02_fdd.disk[i] ? ESPectrum::mb02_fdd.disk[i]->fname : "";
                                string row = formatSlotRow(lab, fname, Config::mb02WP[i], true);
                                if (!Config::mb02) row = "\x01" + row;
                                mb02menu += row + "\n";
                            }
                            {
                                string snd = MENU_MB02_SNDLED[Config::lang];
                                if (!Config::mb02) snd = "\x01" + snd;
                                mb02menu += snd;
                            }
                            // Save backing rect only on the first draw; repeat draws
                            // paint over the previous fixed-height menu in place.
                            menu_saverect = mb02FirstDraw;
                            uint8_t mb02_num = menuRun(mb02menu);
                            mb02FirstDraw = false;
                            if (mb02_num == 1) {
                                // Mode toggle — stay in the MB-02+ menu so drive rows
                                // show up / disappear in place when the user flips Mode.
                                uint8_t newval = Config::mb02 ? 0 : 1;
                                Config::mb02 = newval;
                                if (Config::mb02 && Config::esxdos) {
                                    Config::esxdos = 0;
                                    DivMMC::init();
                                    OSD::osdCenteredMsg("esxDOS disabled", LEVEL_WARN, 2000);
                                }
                                MB02::init();
                                if (Config::mb02 && !MB02::enabled) {
                                    OSD::osdCenteredMsg("MB-02+: not enough memory", LEVEL_ERROR, 2000);
                                    Config::mb02 = 0;
                                }
                                Config::save();
                                ESPectrum::reset();
                                menu_curopt = 1;
                                continue;
                            }
                            else if (!Config::mb02 && mb02_num >= 2 && mb02_num <= 6) {
                                // Disabled rows — ignore the press and redraw same menu.
                                menu_curopt = mb02_num;
                                continue;
                            }
                            else if (Config::mb02 && mb02_num >= 2 && mb02_num <= 5) {
                                // Per-drive submenu: Insert / Eject / Write Protect.
                                uint8_t slot = mb02_num - 2;
                                menu_saverect = true;
                                menu_curopt = 1;
                                while (1) {
                                    menu_level = 3;
                                    char drvtitle[16];
                                    snprintf(drvtitle, sizeof(drvtitle), "%s %u\n",
                                        MENU_MB02_DRIVE[Config::lang], (unsigned)(slot + 1));
                                    string drvmenu = drvtitle;
                                    drvmenu += MENU_MB02_INSERT[Config::lang];
                                    drvmenu += MENU_MB02_EJECT[Config::lang];
                                    drvmenu += string(MENU_MB02_WP[Config::lang]) + "\t"
                                             + (Config::mb02WP[slot] ? "[*]" : "[ ]") + "\n";
                                    uint8_t opt2 = menuRun(drvmenu);
                                    if (opt2 == 1) {
                                        menu_saverect = true;
                                        string mFile = fileDialog(FileUtils::DSK_Path, "MB-02+ Disk", DISK_DSKFILE, 26, 15);
                                        if (mFile != "") {
                                            mFile.erase(0, 1);
                                            string fname = FileUtils::DSK_Path + "/" + mFile;
                                            if (FileUtils::getLCaseExt(fname) == "zip") {
                                                string zipFname = ZipExtract::extract(fname, DISK_DSKFILE);
                                                if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); break; }
                                                if (zipFname == "\x1b") break;
                                                fname = zipFname;
                                            }
                                            rvmWD1793InsertDisk(&ESPectrum::mb02_fdd, slot, fname);
                                            if (ESPectrum::mb02_fdd.disk[slot])
                                                ESPectrum::mb02_fdd.disk[slot]->writeprotect = Config::mb02WP[slot];
                                            ESPectrum::mb02_fdd.diskLoadedCyl = -1;
                                            ESPectrum::mb02_fdd.diskLoadedSide = -1;
                                            MB02::signalDiskChange();
                                            Config::save();
                                        }
                                        // Pop drive submenu rect here (mirror Esc path).
                                        VIDEO::SaveRect.restore_last();
                                        menu_saverect = false;
                                        menu_curopt = mb02_num;
                                        break;
                                    } else if (opt2 == 2) {
                                        wdDiskEject(&ESPectrum::mb02_fdd, slot);
                                        MB02::signalDiskChange();
                                        Config::save();
                                        VIDEO::SaveRect.restore_last();
                                        menu_saverect = false;
                                        menu_curopt = mb02_num;
                                        break;
                                    } else if (opt2 == 3) {
                                        Config::mb02WP[slot] = !Config::mb02WP[slot];
                                        if (ESPectrum::mb02_fdd.disk[slot])
                                            ESPectrum::mb02_fdd.disk[slot]->writeprotect = Config::mb02WP[slot];
                                        Config::save();
                                        menu_curopt = 3;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = mb02_num;
                                        break;
                                    }
                                }
                            }
                            else if (Config::mb02 && mb02_num == 6) {
                                // Sound & LED toggle.
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_SOUNDLED[Config::lang];
                                    menu += MENU_YESNO[Config::lang];
                                    uint8_t prev = Config::mb02SoundLed;
                                    if (prev) {
                                        menu.replace(menu.find("[Y",0),2,"[*");
                                        menu.replace(menu.find("[N",0),2,"[ ");
                                    } else {
                                        menu.replace(menu.find("[Y",0),2,"[ ");
                                        menu.replace(menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        Config::mb02SoundLed = (opt2 == 1);
                                        if (Config::mb02SoundLed != prev)
                                            Config::save();
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 6;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            }
                            else {
                                menu_curopt = 4;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
#endif
                    else if (FileUtils::fsMount &&
#if !PICO_RP2040
                        stor_num == 5
#else
                        stor_num == 3
#endif
                    ) { // Snapshot
                        menu_saverect = true;
                        menu_curopt = 1;
                        while(1) {
                            menu_level = 2;
                            uint8_t sna_mnu = menuRun(expandHotkeys(MENU_SNA[Config::lang]));
                            if (sna_mnu > 0) {
                                menu_level = 3;
                                menu_saverect = true;
                                if (sna_mnu == 1) {
                                    string mFile = fileDialog(FileUtils::SNA_Path, MENU_SNA_TITLE[Config::lang], DISK_SNAFILE, 28, 16);
                                    if (mFile != "") {
                                        Config::save();
                                        mFile.erase(0, 1);
                                        string fname = FileUtils::SNA_Path + mFile;
                                        if (FileUtils::getLCaseExt(fname) == "zip") {
                                            string zipFname = ZipExtract::extract(fname, DISK_SNAFILE);
                                            if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); break; }
                                            if (zipFname == "\x1b") break;
                                            fname = zipFname;
                                        }
                                        if(!LoadSnapshot(fname, "", "")) {
                                            OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
                                        } else {
                                            Config::ram_file = fname;
                                            Config::last_ram_file = fname;
                                        }
                                        return;
                                    }
                                }
                                else if (sna_mnu == 2) {
                                    // Persist Load
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        menu_footer = Config::lang ? "F3: Cargar  F6: Renombrar  F8: Borrar" : "F3: Load  F6: Rename  F8: Remove";
                uint8_t opt2 = menuRun(buildSlotMenu(MENU_PERSIST_LOAD[Config::lang], 10));
                                        if (opt2) {
                                            if (menu_del_pressed) {
                                                persistDeleteConfirm(opt2);
                                                menu_saverect = false;
                                                menu_curopt = opt2;
                                                continue;
                                            }
                                            if (menu_rename_pressed) {
                                                persistRename(opt2, opt2);
                                                menu_saverect = false;
                                                menu_curopt = opt2;
                                                continue;
                                            }
                                            if (persistLoad(opt2)) {
                                                return;
                                            }
                                            menu_saverect = false;
                                            menu_curopt = opt2;
                                        } else break;
                                    }
                                }
                                else if (sna_mnu == 3) {
                                    // Persist Save
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        menu_footer = Config::lang ? "F6: Renombrar  F8: Borrar" : "F6: Rename  F8: Remove";
                uint8_t opt2 = menuRun(buildSlotMenu(MENU_PERSIST_SAVE[Config::lang], 10));
                                        if (opt2) {
                                            if (menu_del_pressed) {
                                                persistDeleteConfirm(opt2);
                                                menu_saverect = false;
                                                menu_curopt = opt2;
                                                continue;
                                            }
                                            if (menu_rename_pressed) {
                                                persistRename(opt2, opt2);
                                                menu_saverect = false;
                                                menu_curopt = opt2;
                                                continue;
                                            }
                                            if (persistSave(opt2, opt2)) {
                                                return;
                                            }
                                            menu_saverect = false;
                                            menu_curopt = opt2;
                                        } else break;
                                    }
                                }
                                menu_curopt = sna_mnu;
                            } else {
#if !PICO_RP2040
                                menu_curopt = 4;
#else
                                menu_curopt = 3;
#endif
                                menu_level = 1;
                                break;
                            }
                        }
                    }
                    else {
                        menu_curopt = 2;
                        break;
                    }
                }
            }
            else if (opt == 3) { // Audio
                // ***********************************************************************************
                // AUDIO MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                while (1) {
                    menu_level = 1;
                    // Audio: insert General Sound item between MIDI and Audio Driver when GS is available
                    string audio_menu = MENU_AUDIO[Config::lang];
#ifdef USE_GS
                    // GS works on butter XIP (fast) or, as a fallback, on plain SPI PSRAM
                    // (slow path, ~30× slower — best-effort, may glitch on MOD playback).
                    // For SPI fallback, need room for MemESP swap pool + 2 MB GS RAM.
                    bool gs_avail = (butter_psram_size() > 0)
                                    || (psram_size() >= (size_t)MEM_PG_CNT * MEM_PG_SZ + (2u << 20));
                    if (gs_avail) {
                        // Find the last item ("Audio Driver") and insert GS before it
                        size_t last_nl = audio_menu.rfind('\n', audio_menu.size() - 2);
                        if (last_nl != string::npos) {
                            audio_menu.insert(last_nl + 1, MENU_AUDIO_GS_ITEM[Config::lang]);
                        }
                    }
#else
                    bool gs_avail = false;
#endif
                    uint8_t options_num = menuRun(audio_menu);
                    if (options_num > 0) {
                        if (options_num == 1) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string ay_menu = MENU_AY48[Config::lang];
                                ay_menu += MENU_YESNO[Config::lang];
                                bool prev_ay48 = Config::AY48;
                                if (prev_ay48) {
                                    ay_menu.replace(ay_menu.find("[Y",0),2,"[*");
                                    ay_menu.replace(ay_menu.find("[N",0),2,"[ ");
                                } else {
                                    ay_menu.replace(ay_menu.find("[Y",0),2,"[ ");
                                    ay_menu.replace(ay_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(ay_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::AY48 = true;
                                    else
                                        Config::AY48 = false;

                                    if (Config::AY48 != prev_ay48) {
                                        ESPectrum::AY_emu = Config::AY48;
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 1;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 2) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string menu = MENU_AY[Config::lang];
                                uint8_t prev = Config::ayConfig;
                                if (prev == 0) {
                                    menu.replace(menu.find("[B",0),2,"[*");
                                    menu.replace(menu.find("[C",0),2,"[ ");
                                    menu.replace(menu.find("[M",0),2,"[ ");
                                } else if (prev == 1) {
                                    menu.replace(menu.find("[B",0),2,"[ ");
                                    menu.replace(menu.find("[C",0),2,"[*");
                                    menu.replace(menu.find("[M",0),2,"[ ");
                                } else {
                                    menu.replace(menu.find("[B",0),2,"[ ");
                                    menu.replace(menu.find("[C",0),2,"[ ");
                                    menu.replace(menu.find("[M",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(menu);
                                if (opt2) {
                                    Config::ayConfig = opt2 - 1;
                                    if (Config::ayConfig != prev) {
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 2;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 3) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string menu = MENU_TS[Config::lang];
                                uint8_t prev = Config::turbosound;
                                if (prev == 0) {
                                    menu.replace(menu.find("[F",0),2,"[*");
                                    menu.replace(menu.find("[N",0),2,"[ ");
                                    menu.replace(menu.find("[O",0),2,"[ ");
                                    menu.replace(menu.find("[B",0),2,"[ ");
                                } else if (prev == 1) {
                                    menu.replace(menu.find("[F",0),2,"[ ");
                                    menu.replace(menu.find("[N",0),2,"[*");
                                    menu.replace(menu.find("[O",0),2,"[ ");
                                    menu.replace(menu.find("[B",0),2,"[ ");
                                } else if (prev == 2) {
                                    menu.replace(menu.find("[F",0),2,"[ ");
                                    menu.replace(menu.find("[N",0),2,"[ ");
                                    menu.replace(menu.find("[O",0),2,"[*");
                                    menu.replace(menu.find("[B",0),2,"[ ");
                                } else {
                                    menu.replace(menu.find("[F",0),2,"[ ");
                                    menu.replace(menu.find("[N",0),2,"[ ");
                                    menu.replace(menu.find("[O",0),2,"[ ");
                                    menu.replace(menu.find("[B",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(menu);
                                if (opt2) {
                                    Config::turbosound = opt2 - 1;
                                    if (Config::turbosound != prev) {
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 3;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 4) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string menu = MENU_COVOX[Config::lang];
                                uint8_t prev = Config::covox;
                                if (prev == 0) {
                                    menu.replace(menu.find("[N",0),2,"[*");
                                    menu.replace(menu.find("[F",0),2,"[ ");
                                    menu.replace(menu.find("[D",0),2,"[ ");
                                } else if (prev == 1) {
                                    menu.replace(menu.find("[N",0),2,"[ ");
                                    menu.replace(menu.find("[F",0),2,"[*");
                                    menu.replace(menu.find("[D",0),2,"[ ");
                                } else {
                                    menu.replace(menu.find("[N",0),2,"[ ");
                                    menu.replace(menu.find("[F",0),2,"[ ");
                                    menu.replace(menu.find("[D",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(menu);
                                if (opt2) {
                                    Config::covox = opt2 - 1;
                                    if (Config::covox != prev) {
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 4;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
#if !PICO_RP2040
                        else if (options_num == 5) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string saa_menu = MENU_SAA1099[Config::lang];
                                saa_menu += MENU_YESNO[Config::lang];
                                bool prev_saa = Config::SAA1099;
                                if (prev_saa) {
                                    saa_menu.replace(saa_menu.find("[Y",0),2,"[*");
                                    saa_menu.replace(saa_menu.find("[N",0),2,"[ ");
                                } else {
                                    saa_menu.replace(saa_menu.find("[Y",0),2,"[ ");
                                    saa_menu.replace(saa_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(saa_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::SAA1099 = true;
                                    else
                                        Config::SAA1099 = false;

                                    if (Config::SAA1099 != prev_saa) {
                                        ESPectrum::SAA_emu = Config::SAA1099;
                                        if (Config::SAA1099 && Config::timex_video) {
                                            Config::timex_video = false;
                                            VIDEO::timex_port_ff = 0;
                                            VIDEO::timex_mode = 0;
                                            VIDEO::timex_hires_ink = 0;
                                            OSD::osdCenteredMsg("Timex disabled", LEVEL_WARN, 2000);
                                        }
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 5;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 6) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string midi_menu = MENU_MIDI[Config::lang];
                                uint8_t prev_midi = Config::midi;
                                midi_menu.replace(midi_menu.find("[O",0),2, prev_midi == 0 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[A",0),2, prev_midi == 1 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[S",0),2, prev_midi == 2 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[W",0),2, prev_midi == 3 ? "[*" : "[ ");
                                uint8_t opt2 = menuRun(midi_menu);
                                if (opt2 >= 1 && opt2 <= 4) {
                                    Config::midi = opt2 - 1;
                                    if (Config::midi != prev_midi) {
                                        Midi::enabled = prev_midi;
                                        Midi::deinit();
                                        Midi::enabled = Config::midi;
                                        if (Midi::enabled)
                                            Midi::init();
                                        Config::save();
#if defined(MIDI_TX_PIN) && defined(LOAD_WAV_PIO) && (LOAD_WAV_PIO == MIDI_TX_PIN)
                                        if ((Config::midi == 1 || Config::midi == 2) && Config::real_player)
                                            osdCenteredMsg(MSG_MIDI_PIN_CONFLICT[Config::lang], LEVEL_WARN, 3000);
#endif
                                    }
                                    // Software selected — open preset submenu
                                    if (Config::midi == 3) {
                                        menu_level = 3;
                                        menu_curopt = 1;
                                        menu_saverect = true;
                                        string preset_menu = MENU_MIDI_PRESET[Config::lang];
                                        static const char preset_marks[] = "GPCSROMY";
                                        for (int p = 0; p < 8; p++) {
                                            char mark[3] = { '[', preset_marks[p], '\0' };
                                            auto pos = preset_menu.find(mark, 0);
                                            if (pos != string::npos)
                                                preset_menu.replace(pos, 2, Config::midi_synth_preset == p ? "[*" : "[ ");
                                        }
                                        uint8_t opt3 = menuRun(preset_menu);
                                        if (opt3) {
                                            Config::midi_synth_preset = opt3 - 1;
                                            MidiSynth::preset = Config::midi_synth_preset;
                                            Config::save();
                                            VIDEO::SaveRect.restore_last();
                                        }
                                        menu_level = 2;
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 6;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
#ifdef USE_GS
                        else if (gs_avail && options_num == 7) { // General Sound
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string gs_menu = MENU_GS[Config::lang];
                                gs_menu += MENU_YESNO[Config::lang];
                                uint8_t prev_gs = Config::gs_enabled;
                                if (prev_gs) {
                                    gs_menu.replace(gs_menu.find("[Y",0),2,"[*");
                                    gs_menu.replace(gs_menu.find("[N",0),2,"[ ");
                                } else {
                                    gs_menu.replace(gs_menu.find("[Y",0),2,"[ ");
                                    gs_menu.replace(gs_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(gs_menu);
                                if (opt2) {
                                    Config::gs_enabled = (opt2 == 1) ? 1 : 0;
                                    if (Config::gs_enabled != prev_gs) {
                                        if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                            Config::save();
                                            esp_hard_reset();
                                        } else {
                                            Config::gs_enabled = prev_gs;
                                        }
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 7;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
#endif
#endif
                        else if (options_num ==
#if !PICO_RP2040
                            (gs_avail ? 8 : 7)
#else
                            5
#endif
                        ) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string menu = MENU_I2S[Config::lang];
#ifdef ZERO2
                                menu += "PCM5122  \t[5]\n";
#endif
                                uint8_t prev = Config::audio_driver;
                                menu.replace(menu.find("[A",0),2,prev==0 ? "[*" : "[ ");
                                menu.replace(menu.find("[P",0),2,prev==1 ? "[*" : "[ ");
                                menu.replace(menu.find("[I",0),2,prev==2 ? "[*" : "[ ");
                                menu.replace(menu.find("[Y",0),2,prev==3 ? "[*" : "[ ");
                                { auto pos = menu.find("[H",0); if (pos != string::npos) menu.replace(pos,2,prev==4 ? "[*" : "[ "); }
#ifdef ZERO2
                                { auto pos = menu.find("[5",0); if (pos != string::npos) menu.replace(pos,2,prev==5 ? "[*" : "[ "); }
#endif
                                uint8_t opt2 = menuRun(menu);
                                if (opt2) {
                                    // Map menu position to driver value
                                    // (HDMI may be hidden, so opt2-1 doesn't always match)
                                    static const uint8_t driver_map[] = {0, 1, 2, 3,
#ifdef ZERO2
                                        5
#endif
                                    };
                                    static const uint8_t driver_map_size = sizeof(driver_map);
                                    Config::audio_driver = (opt2 <= driver_map_size) ? driver_map[opt2 - 1] : prev;
                                    if (Config::audio_driver != prev) {
                                        if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                            Config::save();
                                            esp_hard_reset();
                                        } else {
                                            Config::audio_driver = prev;
                                        }
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt =
#if !PICO_RP2040
                                        (gs_avail ? 8 : 7);
#else
                                        5;
#endif
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                    } else {
                        menu_curopt = 3;
                        break;
                    }
                }
            }
            else if (opt == 4) { // Video
                // ***********************************************************************************
                // VIDEO MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                while (1) {
                    menu_level = 1;
                    // Video
                    uint8_t options_num = menuRun(MENU_VIDEO[Config::lang]);
                    if (options_num > 0) {
                        if (options_num == 1) { // VIDEO MODE
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = MENU_VIDEO_MODE[Config::lang];
#ifdef VGA_HDMI
                                uint8_t &curVideoMode = SELECT_VGA ? Config::vga_video_mode : Config::hdmi_video_mode;
#else
                                uint8_t dummy_vm = 0;
                                uint8_t &curVideoMode = dummy_vm;
#endif
                                // cur_sel maps directly to VM_* enum: 0=640x480@60, 1=640x480@50, 2=720x480@60, 3=720x576@60, 4=720x576@50
                                uint8_t cur_sel = curVideoMode;
                                opt_menu.replace(opt_menu.find("[6",0),2, cur_sel == 0 ? "[*" : "[ ");
                                opt_menu.replace(opt_menu.find("[5",0),2, cur_sel == 1 ? "[*" : "[ ");
                            #if !PICO_RP2040
                                opt_menu.replace(opt_menu.find("[H",0),2, cur_sel == 2 ? "[*" : "[ ");
                                opt_menu.replace(opt_menu.find("[X",0),2, cur_sel == 3 ? "[*" : "[ ");
                                opt_menu.replace(opt_menu.find("[F",0),2, cur_sel == 4 ? "[*" : "[ ");
                            #endif
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    uint8_t new_vm = opt2 - 1; // opt2 is 1-based, VM_* is 0-based
                                    if (new_vm != curVideoMode) {
                                        uint8_t saved_vm = curVideoMode;
                                        curVideoMode = new_vm;
                                        Config::save();
#ifdef VGA_HDMI
                                        VIDEO::changeMode();
                                        if (!videoModeConfirm(10)) {
                                            // Rollback
                                            curVideoMode = saved_vm;
                                            Config::save();
                                            VIDEO::changeMode();
                                        }
                                        // Exit OSD after mode switch
                                        return;
#else
                                        OSD::esp_hard_reset();
#endif
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 1;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        // Palette selection (option 2 on all platforms)
                        else if (options_num == 2) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                // Build palette menu dynamically (built-in + custom)
                                uint8_t pal_count = VIDEO::paletteCount();
                                uint8_t prev = Config::palette;
                                string pal_menu = Config::lang ? "Paleta\n" : "Palette\n";
                                for (uint8_t i = 0; i < pal_count; i++) {
                                    pal_menu += VIDEO::paletteName(i);
                                    pal_menu += "\t";
                                    pal_menu += (prev == i) ? "[*]\n" : "[ ]\n";
                                }
                                uint8_t opt2 = menuRun(pal_menu);
                                if (opt2) {
                                    Config::palette = opt2 - 1;
                                    if (Config::palette != prev) {
                                        VIDEO::applyPalette();
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 2;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 3) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = MENU_RENDER[Config::lang];
                                uint8_t prev_opt = Config::render;
                                if (prev_opt) {
                                    opt_menu.replace(opt_menu.find("[S",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[A",0),2,"[*");
                                } else {
                                    opt_menu.replace(opt_menu.find("[S",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[A",0),2,"[ ");
                                }
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::render = 0;
                                    else
                                        Config::render = 1;

                                    if (Config::render != prev_opt) {
                                        Config::save();

                                        VIDEO::snow_toggle =
                                            Config::arch != "P1024" && Config::arch != "Pentagon" && Config::arch != "P512"
                                             ? Config::render : false;

                                        if (VIDEO::snow_toggle) {
                                            VIDEO::Draw = &VIDEO::MainScreen_Blank_Snow;
                                            VIDEO::Draw_Opcode = &VIDEO::MainScreen_Blank_Snow_Opcode;
                                        } else {
                                            VIDEO::Draw = &VIDEO::MainScreen_Blank;
                                            VIDEO::Draw_Opcode = &VIDEO::MainScreen_Blank_Opcode;
                                        }

                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 3;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 4) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {

                                // aspect ratio
                                string asp_menu = MENU_ASPECT[Config::lang];
                                bool prev_asp = Config::aspect_16_9;
                                if (prev_asp) {
                                    asp_menu.replace(asp_menu.find("[4",0),2,"[ ");
                                    asp_menu.replace(asp_menu.find("[1",0),2,"[*");
                                } else {
                                    asp_menu.replace(asp_menu.find("[4",0),2,"[*");
                                    asp_menu.replace(asp_menu.find("[1",0),2,"[ ");
                                }
                                uint8_t opt2 = menuRun(asp_menu);
                                if (opt2) {
                                    /***
                                    if (opt2 == 1)
                                        Config::aspect_16_9 = false;
                                    else
                                        Config::aspect_16_9 = true;
                                    */
                                    if (Config::aspect_16_9 != prev_asp) {
                                        Config::ram_file = "none";
                                        Config::save();
                                        esp_hard_reset();
                                    }

                                    menu_curopt = opt2;
                                    menu_saverect = false;

                                } else {
                                    menu_curopt = 4;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 5) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = MENU_SCANLINES[Config::lang];
                                opt_menu += MENU_YESNO[Config::lang];
                                uint8_t prev_opt = Config::scanlines;
                                if (prev_opt) {
                                    opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                } else {
                                    opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::scanlines = 1;
                                    else
                                        Config::scanlines = 0;

                                    if (Config::scanlines != prev_opt) {
                                        Config::save();
                                        graphics_set_scanlines(Config::scanlines);
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 5;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num == 6) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = MENU_VSYNC[Config::lang];
                                opt_menu += MENU_YESNO[Config::lang];
                                bool prev_opt = Config::v_sync_enabled;
                                if (prev_opt) {
                                    opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                } else {
                                    opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::v_sync_enabled = true;
                                    else
                                        Config::v_sync_enabled = false;

                                    if (Config::v_sync_enabled != prev_opt) {
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 6;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        #if !PICO_RP2040
                        else if (options_num == 7) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = MENU_GIGASCREEN[Config::lang];
                                opt_menu += MENU_YESNO[Config::lang];
                                bool prev_opt = Config::gigascreen_enabled;
                                if (prev_opt) {
                                    opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                } else {
                                    opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::gigascreen_enabled = true;
                                    else
                                        Config::gigascreen_enabled = false;

                                    if (Config::gigascreen_enabled != prev_opt) {
                                        if (Config::gigascreen_enabled) {
#if !PICO_RP2040
                                            initGigascreenBlendLUT();
                                            VIDEO::InitPrevBuffer();
                                            if (!VIDEO::vga.prevFrameBuffer) {
                                                Config::gigascreen_enabled = false;
                                                VIDEO::gigascreen_enabled = false;
                                            }
#endif
                                            if (Config::gigascreen_onoff == 0)
                                                Config::gigascreen_onoff = 1; // Off->On when enabling
                                            if (Config::gigascreen_onoff == 1)
                                                VIDEO::gigascreen_enabled = true;
                                            else {
                                                VIDEO::gigascreen_enabled = false;
                                                VIDEO::gigascreen_auto_countdown = 0;
                                            }
                                        } else {
                                            VIDEO::gigascreen_enabled = false;
                                            VIDEO::gigascreen_auto_countdown = 0;
                                        }
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 7;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        // ULA+ ON/OFF
                        else if (options_num == 8) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string ula_menu = MENU_ULAPLUS[Config::lang];
                                ula_menu += MENU_YESNO[Config::lang];
                                bool prev_ula = Config::ulaplus;
                                if (prev_ula) {
                                    ula_menu.replace(ula_menu.find("[Y",0),2,"[*");
                                    ula_menu.replace(ula_menu.find("[N",0),2,"[ ");
                                } else {
                                    ula_menu.replace(ula_menu.find("[Y",0),2,"[ ");
                                    ula_menu.replace(ula_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(ula_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::ulaplus = true;
                                    else
                                        Config::ulaplus = false;

                                    if (Config::ulaplus != prev_ula) {
                                        if (!Config::ulaplus && VIDEO::ulaplus_enabled) {
                                            VIDEO::ulaPlusDisable();
                                        }
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 8;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        // Timex Video ON/OFF
                        else if (options_num == 9) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string tmx_menu = MENU_TIMEX[Config::lang];
                                tmx_menu += MENU_YESNO[Config::lang];
                                bool prev = Config::timex_video;
                                if (prev) {
                                    tmx_menu.replace(tmx_menu.find("[Y",0),2,"[*");
                                    tmx_menu.replace(tmx_menu.find("[N",0),2,"[ ");
                                } else {
                                    tmx_menu.replace(tmx_menu.find("[Y",0),2,"[ ");
                                    tmx_menu.replace(tmx_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(tmx_menu);
                                if (opt2) {
                                    if (opt2 == 1)
                                        Config::timex_video = true;
                                    else {
                                        Config::timex_video = false;
                                        VIDEO::timex_port_ff = 0;
                                        VIDEO::timex_mode = 0;
                                        VIDEO::timex_hires_ink = 0;
                                    }
                                    if (Config::timex_video != prev) {
                                        if (Config::timex_video && Config::SAA1099) {
                                            Config::SAA1099 = false;
                                            ESPectrum::SAA_emu = false;
                                            OSD::osdCenteredMsg("SAA1099 disabled", LEVEL_WARN, 2000);
                                        }
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 9;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        // DMA mode
                        else if (options_num == 10) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string dma_menu = MENU_DMA[Config::lang];
                                uint8_t prev = Config::dma_mode;
                                dma_menu.replace(dma_menu.find("[O",0),2, prev == 0 ? "[*" : "[ ");
                                dma_menu.replace(dma_menu.find("[B",0),2, prev == 1 ? "[*" : "[ ");
                                dma_menu.replace(dma_menu.find("[X",0),2, prev == 2 ? "[*" : "[ ");
                                uint8_t opt2 = menuRun(dma_menu);
                                if (opt2) {
                                    Config::dma_mode = opt2 - 1;
                                    if (Config::dma_mode != prev) {
                                        Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 10;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        // HDMI Dither (visible only on HDMI builds — palette has no extra bits on VGA)
                        else if (options_num == 11) {
#ifdef VGA_HDMI
                            if (SELECT_VGA) {
                                OSD::osdCenteredMsg("HDMI only", LEVEL_WARN, 1500);
                            } else
#endif
                            {
                                menu_level = 2;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string dith_menu = MENU_HDMI_DITHER[Config::lang];
                                    dith_menu += MENU_YESNO[Config::lang];
                                    bool prev = Config::hdmi_dither;
                                    if (prev) {
                                        dith_menu.replace(dith_menu.find("[Y",0),2,"[*");
                                        dith_menu.replace(dith_menu.find("[N",0),2,"[ ");
                                    } else {
                                        dith_menu.replace(dith_menu.find("[Y",0),2,"[ ");
                                        dith_menu.replace(dith_menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(dith_menu);
                                    if (opt2) {
                                        Config::hdmi_dither = (opt2 == 1);
                                        if (Config::hdmi_dither != prev) {
                                            // Only takes effect when ULA+ is active; the HDMI ISR
                                            // OR-masks indices 0..63 with 0x40 to sample palette[64..127].
                                            graphics_set_dither(Config::hdmi_dither && VIDEO::ulaplus_enabled);
                                            Config::save();
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 11;
                                        menu_level = 1;
                                        break;
                                    }
                                }
                            }
                        }
                        #endif
                    } else {
                        menu_curopt = 4;
                        break;
                    }
                }
            }
            else if (opt == 5) { // Machine
                // ***********************************************************************************
                // MACHINE MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                bool ext_ram = butter_psram_size() || FileUtils::fsMount || psram_size() > 0;
                while (1) {
                    menu_level = 1;
                    uint8_t arch_num = menuRun(ext_ram ? MENU_ARCH[Config::lang] : MENU_ARCH_NO_SD[Config::lang]);
                    if (arch_num) {
                        string arch = Config::arch;
                        string romset = Config::romSet;
                        uint8_t opt2 = 0;
                        if (arch_num == 1) { // 48K
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            opt2 = menuRun(MENU_ROMS48[Config::lang]);
                            if (opt2) {
                                arch = "48K";
                                if (opt2 == 1) {
                                    romset = "48K";
                                } else
#if NO_SPAIN_ROM_48k
                                if (opt2 == 2) {
                                    romset = "48Kcs";
                                }
#else
                                if (opt2 == 2) {
                                    romset = "48Kes";
                                } else
                                if (opt2 == 3) {
                                    romset = "48Kcs";
                                }
#endif
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 1;
                                menu_level = 2;
                            }
                        } else if (arch_num == 2) { // 128K
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            opt2 = menuRun(MENU_ROMS128[Config::lang]);
                            if (opt2) {
                                arch = "128K";
                                if (opt2 == 1) {
                                    romset = "128K";
                                } else
#if NO_SPAIN_ROM_128k
                                if (opt2 == 2) {
                                    romset = "128Kcs";
                                }
#else
                                if (opt2 == 2) {
                                    romset = "128Kes";
                                } else
                                if (opt2 == 3) {
                                    romset = "+2";
                                } else
                                if (opt2 == 4) {
                                    romset = "+2es";
                                } else
                                if (opt2 == 5) {
                                    romset = "ZX81+";
                                } else
                                if (opt2 == 6) {
                                    romset = "128Kcs";
                                }
#endif
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 1;
                                menu_level = 2;
                            }
                        } else if (arch_num == 3) { // Pentagon
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            opt2 = menuRun(MENU_ROMS_PENT[Config::lang]);
                            if (opt2) {
                                arch = "Pentagon";
                                if (opt2 == 1) {
                                    romset = "128Kp";
                                } else
                                if (opt2 == 2) {
                                    romset = "128Kpg";
                                } else
                                if (opt2 == 3) {
                                    romset = "128Kcs";
                                }
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 1;
                                menu_level = 2;
                            }
                        } else if (ext_ram && arch_num == 4) { // Pentagon 512K
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            opt2 = menuRun(MENU_ROMS_PENT[Config::lang]);
                            if (opt2) {
                                arch = "P512";
                                if (opt2 == 1) {
                                    romset = "128Kp";
                                } else
                                if (opt2 == 2) {
                                    romset = "128Kpg";
                                } else
                                if (opt2 == 3) {
                                    romset = "128Kcs";
                                }
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 1;
                                menu_level = 2;
                            }
                        } else if (ext_ram && arch_num == 5) { // Pentagon 1024K
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            opt2 = menuRun(MENU_ROMS_PENT[Config::lang]);
                            if (opt2) {
                                arch = "P1024";
                                if (opt2 == 1) {
                                    romset = "128Kp";
                                } else
                                if (opt2 == 2) {
                                    romset = "128Kpg";
                                } else
                                if (opt2 == 3) {
                                    romset = "128Kcs";
                                }
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 1;
                                menu_level = 2;
                            }
                        } else if (ext_ram && arch_num == 6) { // BYTE
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                opt2 = menuRun(MENU_ROMSBYTE[Config::lang]);
                                if (opt2) {
                                    if (opt2 == 1) {
                                        arch = "48K";
                                        romset = "48Kby";
                                        break;
                                    } else
                                    if (opt2 == 2) {
                                        arch = "128K";
                                        romset = "128Kby";
                                        break;
                                    } else
                                    if (opt2 == 3) {
                                        arch = "128K";
                                        romset = "128Kbg";
                                        break;
                                    } else
                                    if (opt2 == 4) {
                                        menu_level = 3;
                                        menu_curopt = 1;
                                        menu_saverect = true;
                                        while (1) {
                                            string opt_menu = MENU_BYTE_COBMECT_MODE[Config::lang];
                                            opt_menu += MENU_YESNO[Config::lang];
                                            bool prev_opt = Config::byte_cobmect_mode;
                                            if (prev_opt) {
                                                opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                                opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                            } else {
                                                opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                                opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                            }
                                            uint8_t opt2 = menuRun(opt_menu);
                                            if (opt2) {
                                                if (opt2 == 1)
                                                    Config::byte_cobmect_mode = true;
                                                else
                                                    Config::byte_cobmect_mode = false;

                                                if (Config::byte_cobmect_mode != prev_opt) {
                                                    Config::save();
                                                    MemESP::rom[0].assign_rom(Config::byte_cobmect_mode ? gb_rom_0_byte_sovmest_48k : gb_rom_0_byte_48k);
                                                    MemESP::recoverPage0();
                                                }
                                                menu_curopt = opt2;
                                                menu_saverect = false;
                                            } else {
                                                menu_curopt = 4;
                                                menu_level = 2;
                                                break;
                                            }
                                        }
                                    }
                                } else {
                                    menu_curopt = 1;
                                    menu_level = 2;
                                    break;
                                }
                            }
                        } else if (ext_ram && arch_num == 7) { // Murmuzavr
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = (FileUtils::fsMount ? MENU_MURMUZAVR : MENU_MURMUZAVR_NONE)[Config::lang];
                                uint32_t new_opt = MEM_PG_CNT, prev_opt = MEM_PG_CNT;
                                if (!FileUtils::fsMount) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[*");
#if PICO_RP2350
                                } else if (prev_opt <= 64) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[1",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[3",0),2,"[ ");
                                } else if (prev_opt <= 256) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[1",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[3",0),2,"[ ");
                                } else if (prev_opt <= 512) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[1",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[3",0),2,"[ ");
                                } else if (prev_opt <= 1024) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[1",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[3",0),2,"[ ");
                                } else {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[1",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[3",0),2,"[*");
                                }
#else
                                } else if (prev_opt <= 64) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[ ");
                                } else if (prev_opt <= 256) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[*");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[ ");
                                } else if (prev_opt <= 512) {
                                    opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[4",0),2,"[ ");
                                    opt_menu.replace(opt_menu.find("[8",0),2,"[*");
                                }
#endif
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    if (opt2 == 1) new_opt = 64;
                                    else if (opt2 == 2) new_opt = 256;
                                    else if (opt2 == 3) new_opt = 512;
                                    else if (opt2 == 4) new_opt = 1024;
                                    else if (opt2 == 5) new_opt = 2048;
                                    if (prev_opt != new_opt) {
                                        if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                            MEM_PG_CNT = new_opt;
                                            Config::save();
                                            OSD::esp_hard_reset();
                                            return;
                                        }
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 1;
                                    menu_level = 2;
                                    break;
                                }
                            }
                        }
#if !NO_ALF
                        else if (arch_num == 8 || !ext_ram) { // ALF TV GAME
                            arch = "ALF";
                            romset = "ALF1";
                            menu_curopt = opt2;
                            menu_saverect = false;
                            Config::romSet = romset;
                            click();
                            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
                            Config::save();
                            Config::requestMachine(arch, romset);
                            ESPectrum::reset();
                            return;
                        }
#endif

                        if (opt2) {
                            if (arch != Config::arch || romset != Config::romSet) {
                                Config::ram_file = "none";
                                if (romset != Config::romSet) {
                                    if (arch == "48K") {
                                        if (Config::pref_romSet_48 == "Last") {
                                            Config::romSet = romset;
                                            Config::romSet48 = romset;
                                        }
                                    } else if (arch == "128K") {
                                        if (Config::pref_romSet_128 == "Last") {
                                            Config::romSet = romset;
                                            Config::romSet128 = romset;
                                        }
                                    } else if (arch == "Pentagon") {
                                        if (Config::pref_romSetPent == "Last") {
                                            Config::romSet = romset;
                                            Config::romSetPent = romset;
                                        }
                                    } else if (arch == "P512") {
                                        if (Config::pref_romSetP512 == "Last") {
                                            Config::romSet = romset;
                                            Config::romSetP512 = romset;
                                        }
                                    } else if (arch == "P1024") {
                                        if (Config::pref_romSetP1M == "Last") {
                                            Config::romSet = romset;
                                            Config::romSetP1M = romset;
                                        }
                                    } else {
                                        Config::romSet = romset;
                                    }
                                }
                                if (arch != Config::arch) {
                                    if (Config::pref_arch == "Last") {
                                        Config::arch = arch;
                                    }
                                }
                                // Mutual exclusivity
#if !PICO_RP2040
                                bool isByte = (romset == "48Kby" || romset == "128Kby");
                                if (Config::mb02 && (arch == "Pentagon" || arch == "P512" || arch == "P1024" ||
                                    isByte)) {
                                    Config::mb02 = 0;
                                    MB02::init();
                                    OSD::osdCenteredMsg("MB-02+ disabled", LEVEL_WARN, 2000);
                                }
                                if (Config::timex_video && isByte) {
                                    Config::timex_video = false;
                                    VIDEO::timex_port_ff = 0;
                                    VIDEO::timex_mode = 0;
                                    OSD::osdCenteredMsg("Timex disabled", LEVEL_WARN, 2000);
                                }
#endif
                                Config::save();
                                Config::requestMachine(arch, romset);
                            }

                            Debug::led_blink();
                            ESPectrum::reset();
                            return;
                        }
                        menu_curopt = arch_num;
                        menu_saverect = false;
                    } else {
                        menu_curopt = 5;
                        break;
                    }
                }
            }
            else if (opt == 6) { // Reset
                // ***********************************************************************************
                // RESET MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                while(1) {
                    menu_level = 1;
                    FILINFO fi;
                    bool mos = f_stat(MOS_FILE, &fi) == FR_OK;
                    // Reset
                    uint8_t opt2 = menuRun(expandHotkeys(mos ? MENU_RESET_MOS[Config::lang] : MENU_RESET[Config::lang]));
                    if (opt2 == 1) {
                        // Soft
                        if (Config::last_ram_file != NO_RAM_FILE) {
                            if(!LoadSnapshot(Config::last_ram_file, "", "")) {
                                OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
                            } else {
                                Config::ram_file = Config::last_ram_file;
                            }
                        } else ESPectrum::reset();
                        return;
                    }
                    else if (opt2 == 2) {
                        // Hard
                        if (Config::ram_file != NO_RAM_FILE) {
                            Config::ram_file = NO_RAM_FILE;
                        }
                        Config::last_ram_file = NO_RAM_FILE;
                        ESPectrum::reset();
                        return;
                    }
                    else if (opt2 == 3) {
                        // ESP host reset
                        if (confirmReboot(OSD_DLG_REBOOT)) {
                            Config::ram_file = NO_RAM_FILE;
                            Config::save();
                            esp_hard_reset();
                        }
                    } else if (mos && opt2 == 4) {
                        if (confirmReboot(OSD_DLG_REBOOT)) {
                            f_unlink(MOS_FILE);
                            esp_hard_reset();
                        }
                    } else if ((mos && opt2 == 5) || (!mos && opt2 == 4)) {
                        if (confirmReboot(OSD_DLG_LOADDEFAULTS)) {
                            f_unlink(MOUNT_POINT_SD STORAGE_NVS);
                            esp_hard_reset();
                        }
                    } else {
                        menu_curopt = 6;
                        break;
                    }
                }
            }
            else if (opt == 7) { // Options
                // ***********************************************************************************
                // OPTIONS MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                while(1) {
                    menu_level = 1;
                    // Options menu
                    uint8_t options_num = menuRun(MENU_OPTIONS[Config::lang]);
                    if (options_num == 1) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            string archprefmenu = MENU_ARCH_PREF[Config::lang];
                            string prev_archpref = Config::pref_arch;
                            if (Config::pref_arch == "48K") {
                                archprefmenu.replace(archprefmenu.find("[4",0),2,"[*");
                                archprefmenu.replace(archprefmenu.find("[1",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[P",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[5",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[L",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[2",0),2,"[ ");
                            } else if (Config::pref_arch == "128K") {
                                archprefmenu.replace(archprefmenu.find("[4",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[1",0),2,"[*");
                                archprefmenu.replace(archprefmenu.find("[P",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[5",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[2",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[L",0),2,"[ ");
                            } else if (Config::pref_arch == "Pentagon") {
                                archprefmenu.replace(archprefmenu.find("[4",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[1",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[P",0),2,"[*");
                                archprefmenu.replace(archprefmenu.find("[5",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[2",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[L",0),2,"[ ");
                            } else if (Config::pref_arch == "P512") {
                                archprefmenu.replace(archprefmenu.find("[4",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[1",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[P",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[5",0),2,"[*");
                                archprefmenu.replace(archprefmenu.find("[2",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[L",0),2,"[ ");
                            } else if (Config::pref_arch == "P1024") {
                                archprefmenu.replace(archprefmenu.find("[4",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[1",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[P",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[5",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[2",0),2,"[*");
                                archprefmenu.replace(archprefmenu.find("[L",0),2,"[ ");
                            } else {
                                archprefmenu.replace(archprefmenu.find("[4",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[1",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[P",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[5",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[2",0),2,"[ ");
                                archprefmenu.replace(archprefmenu.find("[L",0),2,"[*");
                            }
                            uint8_t opt2 = menuRun(archprefmenu);
                            if (opt2) {
                                if (opt2 == 1)
                                    Config::pref_arch = "48K";
                                else
                                if (opt2 == 2)
                                    Config::pref_arch = "128K";
                                else
                                if (opt2 == 3)
                                    Config::pref_arch = "Pentagon";
                                else
                                if (opt2 == 4)
                                    Config::pref_arch = "P512";
                                else
                                if (opt2 == 5)
                                    Config::pref_arch = "P1024";
                                else
                                if (opt2 == 6)
                                    Config::pref_arch = "Last";

                                if (Config::pref_arch != prev_archpref) {
                                    Config::save();
                                }

                                menu_curopt = opt2;
                                menu_saverect = false;

                            } else {
                                menu_curopt = 1;
                                break;
                            }
                        }
                    }
                    else if (options_num == 2) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            uint8_t opt2 = menuRun(MENU_ROM_PREF[Config::lang]);
                            if (opt2) {
                                if (opt2 == 1) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string rpref48_menu = MENU_ROM_PREF_48[Config::lang];
                                        int mpos = -1;
                                        while(1) {
                                            mpos = rpref48_menu.find("[",mpos + 1);
                                            if (mpos == string::npos) break;
                                            string rmenu = rpref48_menu.substr(mpos + 1, 5);
                                            trim(rmenu);
                                            if (rmenu == Config::pref_romSet_48)
                                                rpref48_menu.replace(mpos + 1, 5,"*");
                                            else
                                                rpref48_menu.replace(mpos + 1, 5," ");
                                        }
                                        string prev_rpref48 = Config::pref_romSet_48;
                                        uint8_t opt2 = menuRun(rpref48_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::pref_romSet_48 = "48K";
                                            else
#if NO_SPAIN_ROM_48k
                                            if (opt2 == 2)
                                                Config::pref_romSet_48 = "48Kcs";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSet_48 = "Last";
#else
                                            if (opt2 == 2)
                                                Config::pref_romSet_48 = "48Kes";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSet_48 = "48Kcs";
                                            else
                                            if (opt2 == 4)
                                                Config::pref_romSet_48 = "Last";
#endif
                                            if (Config::pref_romSet_48 != prev_rpref48) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 1;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                } else if (opt2 == 2) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string rpref128_menu = MENU_ROM_PREF_128[Config::lang];
                                        int mpos = -1;
                                        while(1) {
                                            mpos = rpref128_menu.find("[",mpos + 1);
                                            if (mpos == string::npos) break;
                                            string rmenu = rpref128_menu.substr(mpos + 1, 6);
                                            trim(rmenu);
                                            if (rmenu == Config::pref_romSet_128)
                                                rpref128_menu.replace(mpos + 1, 6,"*");
                                            else
                                                rpref128_menu.replace(mpos + 1, 6," ");
                                        }
                                        string prev_rpref128 = Config::pref_romSet_128;
                                        uint8_t opt2 = menuRun(rpref128_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::pref_romSet_128 = "128K";
                                            else
#if NO_SPAIN_ROM_128k
                                            if (opt2 == 2)
                                                Config::pref_romSet_128 = "128Kcs";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSet_128 = "Last";
#else
                                            if (opt2 == 2)
                                                Config::pref_romSet_128 = "128Kes";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSet_128 = "+2";
                                            else
                                            if (opt2 == 4)
                                                Config::pref_romSet_128 = "+2es";
                                            else
                                            if (opt2 == 5)
                                                Config::pref_romSet_128 = "ZX81+";
                                            else
                                            if (opt2 == 6)
                                                Config::pref_romSet_128 = "128Kcs";
                                            else
                                            if (opt2 == 7)
                                                Config::pref_romSet_128 = "Last";
#endif
                                            if (Config::pref_romSet_128 != prev_rpref128) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 1;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                } else if (opt2 == 3) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string rprefPent_menu = MENU_ROM_PREF_PENT[Config::lang];
                                        int mpos = -1;
                                        while(1) {
                                            mpos = rprefPent_menu.find("[",mpos + 1);
                                            if (mpos == string::npos) break;
                                            string rmenu = rprefPent_menu.substr(mpos + 1, 6);
                                            trim(rmenu);
                                            if (rmenu == Config::pref_romSetPent)
                                                rprefPent_menu.replace(mpos + 1, 6,"*");
                                            else
                                                rprefPent_menu.replace(mpos + 1, 6," ");
                                        }
                                        string prev_rprefPent = Config::pref_romSetPent;
                                        uint8_t opt2 = menuRun(rprefPent_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::pref_romSetPent = "128Kp";
                                            else
                                            if (opt2 == 2)
                                                Config::pref_romSetPent = "128Kcs";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSetPent = "Last";
                                            if (Config::pref_romSetPent != prev_rprefPent) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 1;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                } else if (opt2 == 4) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string rprefP512_menu = MENU_ROM_PREF_PENT[Config::lang];
                                        int mpos = -1;
                                        while(1) {
                                            mpos = rprefP512_menu.find("[",mpos + 1);
                                            if (mpos == string::npos) break;
                                            string rmenu = rprefP512_menu.substr(mpos + 1, 6);
                                            trim(rmenu);
                                            if (rmenu == Config::pref_romSetP512)
                                                rprefP512_menu.replace(mpos + 1, 6,"*");
                                            else
                                                rprefP512_menu.replace(mpos + 1, 6," ");
                                        }
                                        string prev_rprefP512 = Config::pref_romSetP512;
                                        uint8_t opt2 = menuRun(rprefP512_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::pref_romSetP512 = "128Kp";
                                            else
                                            if (opt2 == 2)
                                                Config::pref_romSetP512 = "128Kcs";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSetP512 = "Last";
                                            if (Config::pref_romSetP512 != prev_rprefP512) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 1;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                } else if (opt2 == 5) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string rprefP1M_menu = MENU_ROM_PREF_PENT[Config::lang];
                                        int mpos = -1;
                                        while(1) {
                                            mpos = rprefP1M_menu.find("[",mpos + 1);
                                            if (mpos == string::npos) break;
                                            string rmenu = rprefP1M_menu.substr(mpos + 1, 6);
                                            trim(rmenu);
                                            if (rmenu == Config::pref_romSetP1M)
                                                rprefP1M_menu.replace(mpos + 1, 6,"*");
                                            else
                                                rprefP1M_menu.replace(mpos + 1, 6," ");
                                        }
                                        string prev_rprefP1M = Config::pref_romSetP1M;
                                        uint8_t opt2 = menuRun(rprefP1M_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::pref_romSetP1M = "128Kp";
                                            else
                                            if (opt2 == 2)
                                                Config::pref_romSetP1M = "128Kcs";
                                            else
                                            if (opt2 == 3)
                                                Config::pref_romSetP1M = "Last";
                                            if (Config::pref_romSetP1M != prev_rprefP1M) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 1;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 2;
                                break;
                            }
                        }
                    }
                    else if (options_num == 3) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            string joy_menu = MENU_DEFJOY[Config::lang];
                            std::size_t pos = joy_menu.find("[",0);
                            int nfind = 0;
                            while (pos != string::npos) {
                                if (nfind == Config::joystick) {
                                    joy_menu.replace(pos,2,"[*");
                                    break;
                                }
                                pos = joy_menu.find("[",pos + 1);
                                nfind++;
                            }
                            uint8_t optjoy = menuRun(joy_menu);
                            if (optjoy > 0 && optjoy < 6) {
                                Config::joystick = optjoy - 1;
                                Config::setJoyMap(optjoy - 1);
                                Config::save();
                                menu_curopt = optjoy;
                                menu_saverect = false;
                            } else if (optjoy == 6) {
                                joyDialog();
                                if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
                                return;
                            } else {
                                menu_curopt = 3;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
                    else if (options_num == 4) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            // joystick
                            string Mnustr = MENU_JOYPS2[Config::lang];
                            uint8_t opt2 = menuRun(Mnustr);
                            if (opt2 == 1) {
                                // Menu cursor keys as joy
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string csasjoy_menu = MENU_CURSORJOY[Config::lang];
                                    csasjoy_menu += MENU_YESNO[Config::lang];
                                    if (Config::CursorAsJoy) {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[*");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[ ");
                                    } else {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[ ");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(csasjoy_menu);
                                    if (opt2) {
                                        if (opt2 == 1)
                                            Config::CursorAsJoy = true;
                                        else
                                            Config::CursorAsJoy = false;

                                        ESPectrum::PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);
                                        Config::save();

                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 1;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            } else if (opt2 == 2) {
                                // Menu TAB as fire 1
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string csasjoy_menu = MENU_TABASFIRE[Config::lang];
                                    csasjoy_menu += MENU_YESNO[Config::lang];
                                    bool prev_opt = Config::TABasfire1;
                                    if (prev_opt) {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[*");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[ ");
                                    } else {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[ ");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(csasjoy_menu);
                                    if (opt2) {
                                        if (opt2 == 1)
                                            Config::TABasfire1 = true;
                                        else
                                            Config::TABasfire1 = false;

                                        if (Config::TABasfire1 != prev_opt) {

                                            if (Config::TABasfire1) {
                                                ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_TAB;
                                                ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_GRAVEACCENT;
                                                ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_NONE;
                                                ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_NONE;
                                            } else {
                                                ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_NONE;
                                                ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_NONE;
                                                ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_TAB;
                                                ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_GRAVEACCENT;
                                            }

                                            Config::save();
                                        }

                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 2;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            } else if (opt2 == 3) {
                                // Menu Right Enter as Space
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string csasjoy_menu = MENU_ENTERSPACE[Config::lang];
                                    csasjoy_menu += MENU_YESNO[Config::lang];
                                    bool prev_opt = Config::rightSpace;
                                    if (prev_opt) {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[*");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[ ");
                                    } else {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[ ");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(csasjoy_menu);
                                    if (opt2) {
                                        if (opt2 == 1)
                                            Config::rightSpace = true;
                                        else
                                            Config::rightSpace = false;
                                        Config::save();
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 3;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            } else if (opt2 == 4) {
                                // WASD
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string csasjoy_menu = MENU_WASD[Config::lang];
                                    csasjoy_menu += MENU_YESNO[Config::lang];
                                    bool prev_opt = Config::wasd;
                                    if (prev_opt) {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[*");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[ ");
                                    } else {
                                        csasjoy_menu.replace(csasjoy_menu.find("[Y",0),2,"[ ");
                                        csasjoy_menu.replace(csasjoy_menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(csasjoy_menu);
                                    if (opt2) {
                                        if (opt2 == 1)
                                            Config::wasd = true;
                                        else
                                            Config::wasd = false;
                                        Config::save();
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 4;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            } else {
                                menu_curopt = 4;
                                break;
                            }
                        }
                    }
                    else if (options_num == 6) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            // language
                            uint8_t opt2;
                            string Mnustr = MENU_INTERFACE_LANG[Config::lang];
                            std::size_t pos = Mnustr.find("[",0);
                            int nfind = 0;
                            while (pos != string::npos) {
                                if (nfind == Config::lang) {
                                    Mnustr.replace(pos,2,"[*");
                                    break;
                                }
                                pos = Mnustr.find("[",pos + 1);
                                nfind++;
                            }
                            opt2 = menuRun(Mnustr);
                            if (opt2) {
                                if (Config::lang != (opt2 - 1)) {
                                    Config::lang = opt2 - 1;
                                    Config::save();
                                    return;
                                }
                                menu_curopt = opt2;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 6;
                                break;
                            }
                        }
                    }
                    else if (options_num == 5) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            // Other
                            uint8_t options_num = menuRun(MENU_OTHER[Config::lang]);
                            if (options_num > 0) {
                                if (options_num == 1) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string alu_menu = MENU_ALUTIMING[Config::lang];
                                        uint8_t prev_AluTiming = Config::AluTiming;
                                        if (prev_AluTiming == 0) {
                                            alu_menu.replace(alu_menu.find("[E",0),2,"[*");
                                            alu_menu.replace(alu_menu.find("[L",0),2,"[ ");
                                        } else {
                                            alu_menu.replace(alu_menu.find("[E",0),2,"[ ");
                                            alu_menu.replace(alu_menu.find("[L",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(alu_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::AluTiming = 0;
                                            else
                                                Config::AluTiming = 1;

                                            if (Config::AluTiming != prev_AluTiming) {
                                                CPU::latetiming = Config::AluTiming;
                                                CPU::updateStatesInFrame();
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 1;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 2) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string iss_menu = MENU_ISSUE2[Config::lang];
                                        iss_menu += MENU_YESNO[Config::lang];
                                        bool prev_iss = Config::Issue2;
                                        if (prev_iss) {
                                            iss_menu.replace(iss_menu.find("[Y",0),2,"[*");
                                            iss_menu.replace(iss_menu.find("[N",0),2,"[ ");
                                        } else {
                                            iss_menu.replace(iss_menu.find("[Y",0),2,"[ ");
                                            iss_menu.replace(iss_menu.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(iss_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::Issue2 = true;
                                            else
                                                Config::Issue2 = false;

                                            if (Config::Issue2 != prev_iss) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 2;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 3) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string ps2_menu = MENU_KBD2NDPS2[Config::lang];
                                        uint8_t prev_ps2 = Config::joy2cursor;
                                        if (prev_ps2) {
                                            ps2_menu.replace(ps2_menu.find("[N",0),2,"[ ");
                                            ps2_menu.replace(ps2_menu.find("[K",0),2,"[*");
                                        } else {
                                            ps2_menu.replace(ps2_menu.find("[N",0),2,"[*");
                                            ps2_menu.replace(ps2_menu.find("[K",0),2,"[ ");
                                        }
                                        uint8_t opt2 = menuRun(ps2_menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::joy2cursor = false;
                                            else
                                                Config::joy2cursor = true;
                                            Config::save();
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 3;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 4) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string menu = MENU_ALF_JOY[Config::lang];
                                        uint8_t prev = Config::secondJoy;
                                        if (prev == 3) {
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[2",0),2,"[ ");
                                            menu.replace(menu.find("[N",0),2,"[*");
                                        } else if (prev == 2) {
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[2",0),2,"[*");
                                            menu.replace(menu.find("[N",0),2,"[ ");
                                        } else {
                                            menu.replace(menu.find("[1",0),2,"[*");
                                            menu.replace(menu.find("[2",0),2,"[ ");
                                            menu.replace(menu.find("[N",0),2,"[ ");
                                        }
                                        uint8_t opt2 = menuRun(menu);
                                        if (opt2) {
                                            Config::secondJoy = opt2;
                                            if (Config::secondJoy != prev) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 4;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 5) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string menu = MENU_K_JOY[Config::lang];
                                        uint8_t prev = Config::kempstonPort;
                                        if (prev == 0x37) {
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[3",0),2,"[*");
                                            menu.replace(menu.find("[9",0),2,"[ ");
                                        } else if (prev == 0x5F) {
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[3",0),2,"[ ");
                                            menu.replace(menu.find("[9",0),2,"[*");
                                        } else {
                                            menu.replace(menu.find("[1",0),2,"[*");
                                            menu.replace(menu.find("[3",0),2,"[ ");
                                            menu.replace(menu.find("[9",0),2,"[ ");
                                        }
                                        uint8_t opt2 = menuRun(menu);
                                        if (opt2) {
                                            if (opt2 == 1)
                                                Config::kempstonPort = 0x1F;
                                            else if (opt2 == 2)
                                                Config::kempstonPort = 0x37;
                                            else
                                                Config::kempstonPort = 0x5F;

                                            if (Config::kempstonPort != prev) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 5;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 6) {
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string menu = MENU_THROTTLING[Config::lang];
                                        uint8_t prev = Config::throtling;
                                        if (prev == 0) {
                                            menu.replace(menu.find("[N",0),2,"[*");
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[2",0),2,"[ ");
                                            menu.replace(menu.find("[3",0),2,"[ ");
                                        } else if (prev == 1) {
                                            menu.replace(menu.find("[N",0),2,"[ ");
                                            menu.replace(menu.find("[1",0),2,"[*");
                                            menu.replace(menu.find("[2",0),2,"[ ");
                                            menu.replace(menu.find("[3",0),2,"[ ");
                                        } else if (prev == 2) {
                                            menu.replace(menu.find("[N",0),2,"[ ");
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[2",0),2,"[*");
                                            menu.replace(menu.find("[3",0),2,"[ ");
                                        } else if (prev == 3) {
                                            menu.replace(menu.find("[N",0),2,"[ ");
                                            menu.replace(menu.find("[1",0),2,"[ ");
                                            menu.replace(menu.find("[2",0),2,"[ ");
                                            menu.replace(menu.find("[3",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(menu);
                                        if (opt2) {
                                            Config::throtling = opt2 - 1;
                                            if (Config::throtling != prev) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 6;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 7) {
                                    // Hot Keys
                                    OSD::hotkeyDialog();
                                    menu_curopt = 7;
                                    menu_level = 2;
                                    menu_saverect = false;
                                }
                            } else {
                                menu_curopt = 5;
                                break;
                            }
                        }
                    } else if (options_num == 7) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            // Update
                            string Mnustr = expandHotkeys(FileUtils::fsMount ? MENU_UPDATE_FW[Config::lang] : MENU_UPDATE_FW_NO_SD[Config::lang]);
                            uint8_t opt2 = menuRun(Mnustr);
                            if (opt2) {
                                // Update
                                if (opt2 == 1) {
                                    /// TODO: close all files
                                    //close_all()
                                    reset_usb_boot(0, 0);
                                    while(1);
                                } else {
                                    string mFile = fileDialog(FileUtils::ROM_Path, MENU_ROM_TITLE[Config::lang], DISK_ROMFILE, 26, 15);
                                    if (mFile != "") {
                                        mFile.erase(0, 1);
                                        string fname = FileUtils::ROM_Path + mFile;
                                        bool res = updateROM(fname, opt2 - 1);
                                        if (res) {
                                            return;
                                        }
                                    }
                                    menu_curopt = 1;
                                    menu_level = 2;
                                    menu_saverect = false;
                                }
                            } else {
                                menu_curopt = 7;
                                break;
                            }
                        }
                    } else {
                        menu_curopt = 7;
                        break;
                    }
                }
            }
            else if (opt == 8) { // Debug
                // DEBUG MENU
                menu_saverect = true;
                menu_curopt = 1;
                while(1) {
                    menu_level = 1;
                    // Debug
                    uint8_t opt2 = menuRun(expandHotkeys(MENU_DEBUG_EN));
                    if (opt2 == 1) {
                        OSD::osdDebug();
                        return;
                    } else if (opt2 == 2) {
                        BPDialog();
                        return;
                    } else if (opt2 == 3) {
                        { uint16_t bpAddr = BPListDialog();
                        if (bpAddr != 0xFFFF) osdDebug(bpAddr); }
                        return;
                    } else if (opt2 == 4) {
                        jumpToDialog();
                        return;
                    } else if (opt2 == 5) {
                        pokeDialog();
                        return;
                    } else if (opt2 == 6) {
                        Z80::triggerNMI();
                        return;
                    } else {
                        menu_curopt = 8;
                        break;
                    }
                }
            }
            else if (opt == 9) { // Hardware
                // ***********************************************************************************
                // HARDWARE MENU
                // ***********************************************************************************
                menu_saverect = true;
                menu_curopt = 1;
                while (1) {
                    menu_level = 1;
                    uint8_t hw_opt = menuRun(MENU_HARDWARE[Config::lang]);
                    if (hw_opt == 1) {
                        // Chip Info
                        OSD::ChipInfo();
                        menu_curopt = 1;
                        menu_saverect = false;
                    }
                    else if (hw_opt == 2) {
                        // Board Info
                        OSD::BoardInfo();
                        menu_curopt = 2;
                        menu_saverect = false;
                    }
                    else if (hw_opt == 3) {
                        // Emulator Info
                        OSD::EmulatorInfo();
                        menu_curopt = 3;
                        menu_saverect = false;
                    }
                    else if (hw_opt == 4) {
                        // Overclock submenu — warn user
                        osdCenteredMsg(Config::lang ? "Peligroso! Puede no arrancar!" : "Dangerous! Board may not boot!", LEVEL_WARN, 2000);
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                        #if !PICO_RP2040
                            uint8_t oc_opt = menuRun(MENU_OVERCLOCK_VREG[Config::lang]);
                        #else
                            uint8_t oc_opt = menuRun(MENU_OVERCLOCK[Config::lang]);
                        #endif
                            if (oc_opt == 1) {
                                // CPU Freq
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string mhz_menu = MENU_CPU_MHZ;
                                    uint16_t cur = Config::cpu_mhz;
                                    mhz_menu.replace(mhz_menu.find("[2"), 2, cur == 252 ? "[*" : "[ ");
                                    mhz_menu.replace(mhz_menu.find("[3"), 2, cur == 378 ? "[*" : "[ ");
                                #if !PICO_RP2040
                                    mhz_menu.replace(mhz_menu.find("[5"), 2, cur == 504 ? "[*" : "[ ");
                                #endif
                                    uint8_t opt2 = menuRun(mhz_menu);
                                    if (opt2) {
                                        uint16_t new_mhz = 0;
                                        if (opt2 == 1) new_mhz = 252;
                                        else if (opt2 == 2) new_mhz = 378;
                                    #if !PICO_RP2040
                                        else if (opt2 == 3) new_mhz = 504;
                                    #endif
                                        if (new_mhz && new_mhz != cur) {
                                            Config::cpu_mhz = new_mhz;
                                            if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                                Config::save();
                                                esp_hard_reset();
                                            } else {
                                                Config::cpu_mhz = cur;
                                            }
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 1;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            }
                        #if !PICO_RP2040
                            else if (oc_opt == 2) {
                                // VReg Voltage
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                while (1) {
                                    string vreg_menu = MENU_VREG_VOLTAGE;
                                    uint8_t cur = Config::vreq_voltage;
                                    static const uint8_t vreg_vals[] = {
                                        VREG_VOLTAGE_1_15, VREG_VOLTAGE_1_20, VREG_VOLTAGE_1_25,
                                        VREG_VOLTAGE_1_30, VREG_VOLTAGE_1_35, VREG_VOLTAGE_1_40,
                                        VREG_VOLTAGE_1_50, VREG_VOLTAGE_1_60, VREG_VOLTAGE_1_65,
                                        VREG_VOLTAGE_1_70, VREG_VOLTAGE_1_80
                                    };
                                    const char markers[] = "ABCDEFGHIJK";
                                    for (int i = 0; i < 11; i++) {
                                        char mk[3] = { '[', markers[i], '\0' };
                                        vreg_menu.replace(vreg_menu.find(mk), 2, cur == vreg_vals[i] ? "[*" : "[ ");
                                    }
                                    uint8_t opt2 = menuRun(vreg_menu);
                                    if (opt2 && opt2 <= 11) {
                                        uint8_t new_v = vreg_vals[opt2 - 1];
                                        if (new_v != cur) {
                                            Config::vreq_voltage = new_v;
                                            if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                                Config::save();
                                                esp_hard_reset();
                                            } else {
                                                Config::vreq_voltage = cur;
                                            }
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 2;
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            }
                            // Flash Freq (opt 3 on RP2350, opt 2 on RP2040)
                            else if (oc_opt == 3) {
                        #else
                            else if (oc_opt == 2) {
                        #endif
                                // Flash Freq
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                static const uint16_t flash_vals[] = { 33, 66, 84, 100, 133, 166 };
                                while (1) {
                                    string ff_menu = MENU_FLASH_FREQ;
                                    uint16_t cur = Config::max_flash_freq;
                                    const char markers[] = "ABCDEF";
                                    for (int i = 0; i < 6; i++) {
                                        char mk[3] = { '[', markers[i], '\0' };
                                        ff_menu.replace(ff_menu.find(mk), 2, cur == flash_vals[i] ? "[*" : "[ ");
                                    }
                                    uint8_t opt2 = menuRun(ff_menu);
                                    if (opt2 && opt2 <= 6) {
                                        uint16_t new_f = flash_vals[opt2 - 1];
                                        if (new_f != cur) {
                                            Config::max_flash_freq = new_f;
                                            if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                                Config::save();
                                                esp_hard_reset();
                                            } else {
                                                Config::max_flash_freq = cur;
                                            }
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                    #if !PICO_RP2040
                                        menu_curopt = 3;
                                    #else
                                        menu_curopt = 2;
                                    #endif
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            }
                        #if !PICO_RP2040
                            // PSRAM Freq (opt 4 on RP2350, opt 3 on RP2040)
                            else if (oc_opt == 4) {
                        #else
                            else if (oc_opt == 3) {
                        #endif
                                // PSRAM Freq
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                static const uint16_t psram_vals[] = { 66, 84, 100, 133, 166 };
                                while (1) {
                                    string pf_menu = MENU_PSRAM_FREQ;
                                    uint16_t cur = Config::max_psram_freq;
                                    const char markers[] = "ABCDE";
                                    for (int i = 0; i < 5; i++) {
                                        char mk[3] = { '[', markers[i], '\0' };
                                        pf_menu.replace(pf_menu.find(mk), 2, cur == psram_vals[i] ? "[*" : "[ ");
                                    }
                                    uint8_t opt2 = menuRun(pf_menu);
                                    if (opt2 && opt2 <= 5) {
                                        uint16_t new_p = psram_vals[opt2 - 1];
                                        if (new_p != cur) {
                                            Config::max_psram_freq = new_p;
                                            if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                                Config::save();
                                                esp_hard_reset();
                                            } else {
                                                Config::max_psram_freq = cur;
                                            }
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                    #if !PICO_RP2040
                                        menu_curopt = 4;
                                    #else
                                        menu_curopt = 3;
                                    #endif
                                        menu_level = 2;
                                        break;
                                    }
                                }
                            }
                            else {
                                menu_curopt = 3;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
                    else {
                        menu_curopt = 9;
                        break;
                    }
                }
            }
            else if (opt == 10) { // ZX Keyboard — bitmap overlay
                // Protect OSD area from Z80 video renderer overwrite
                bool kbd_osd_enabled = (VIDEO::OSD != 0);
                if (!kbd_osd_enabled) {
                    if (Config::aspect_16_9)
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                    else
                        VIDEO::Draw_OSD43 = VIDEO::BottomBorder_OSD;
                    VIDEO::OSD = 0x04;
                }
                // Wipe the OSD area with ZX paper colour
                int kbd_w = 254, kbd_h = 156;
                int kbd_x = (OSD::scrW - kbd_w) / 2;
                int kbd_y = (OSD::scrH - kbd_h) / 2;
                VIDEO::vga.fillRect(kbd_x - 3, kbd_y - 12, kbd_w + 6, kbd_h + 16, zxColor(0, 0));
                VIDEO::vga.rect(kbd_x - 3, kbd_y - 12, kbd_w + 6, kbd_h + 16, zxColor(7, 0));
                // Header
                VIDEO::vga.fillRect(kbd_x - 2, kbd_y - 11, kbd_w + 4, 9, zxColor(7, 0));
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 0));
                VIDEO::vga.setCursor(kbd_x + 4, kbd_y - 10);
                VIDEO::vga.print(Config::lang ? "Teclado ZX" : "ZX Keyboard");
                // Draw bitmap: byte = ZX palette index (0..15), 0xFF and other non-palette values = transparent
                for (int y = 0; y < kbd_h; y++) {
                    for (int x = 0; x < kbd_w; x++) {
                        uint8_t idx = kbd_img[x + y * kbd_w];
                        if (idx >= 0x10) continue; // skip transparent and out-of-range
                        VIDEO::vga.dotFast(kbd_x + x, kbd_y + y, zxColor(idx & 7, (idx >> 3) & 1));
                    }
                }
                // The Enter used to pick this menu item may still be held — wait for the
                // first key-up event, then accept ESC or Enter as "close".
                bool saw_release = false;
                while (1) {
                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        if (ESPectrum::readKbd(&Nextkey)) {
                            if (!Nextkey.down) { saw_release = true; continue; }
                            if (saw_release && (is_enter(Nextkey.vk) || is_back(Nextkey.vk))) break;
                        }
                    }
                    sleep_ms(20);
                }
                click();
                if (!kbd_osd_enabled) {
                    VIDEO::OSD = 0;
                    if (Config::aspect_16_9)
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen;
                    else
                        VIDEO::Draw_OSD43 = VIDEO::BottomBorder;
                }
                if (VIDEO::OSD) OSD::drawStats();
                return;
            }
            else if (opt == 11) { // Help — dynamic from hotkeys
                // Build index of visible hotkeys (no large buffer needed)
                auto descs = Config::lang ? hkDescES : hkDescEN;
                const int maxCols = osdMaxCols();
                const int descCol = 16;
                // -2 = PrtScr, -1 = ScrollLk, 0..HK_COUNT-1 = hotkey index
                int8_t hkOrder[Config::HK_COUNT + 2];
                int nlines = 0;
                for (int i = 0; i < Config::HK_COUNT; i++) {
                    if (Config::hotkeys[i].vk == (uint16_t)fabgl::VK_NONE) continue;
                    hkOrder[nlines++] = (int8_t)i;
                }
                hkOrder[nlines++] = -2; // PrtScr
                hkOrder[nlines++] = -1; // ScrollLk

                // Format one help line into buf (42 bytes max)
                auto fmtLine = [&](int idx, char *buf) {
                    const char *key, *desc;
                    char keybuf[16];
                    if (idx == -2) { key = "PrtScr"; desc = Config::lang ? "Captura BMP" : "BMP capture"; }
                    else if (idx == -1) { key = "ScrollLk"; desc = Config::lang ? "Cursor=Joy" : "Cursor=Joy"; }
                    else {
                        string b = hkBindingText(idx);
                        snprintf(keybuf, sizeof(keybuf), "%s", b.c_str());
                        key = keybuf;
                        desc = descs[idx];
                    }
                    int pos = snprintf(buf, 42, " [%s]", key);
                    while (pos < descCol) buf[pos++] = ' ';
                    snprintf(buf + pos, 42 - pos, "%s", desc);
                    int len = strlen(buf);
                    if (len < maxCols) { memset(buf + len, ' ', maxCols - len); buf[maxCols] = 0; }
                    else buf[maxCols] = 0;
                };

                // Content area: skip 2 rows top (header), 2 rows bottom (footer)
                const int topRow = 2;
                const int maxVisible = osdMaxRows() - 4;
                int scroll = 0;

                auto drawHelp = [&]() {
                    drawOSD(true);
                    // Content
                    char buf[42];
                    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                    for (int i = 0; i < maxVisible && (scroll + i) < nlines; i++) {
                        fmtLine(hkOrder[scroll + i], buf);
                        osdAt(topRow + i, 0);
                        VIDEO::vga.print(buf);
                    }
                    // Scrollbar on the right edge
                    if (nlines > maxVisible) {
                        int ox = osdInsideX() + (maxCols - 1) * OSD_FONT_W;
                        int oy = osdInsideY() + topRow * OSD_FONT_H;
                        int barH = maxVisible * OSD_FONT_H;
                        // Track
                        VIDEO::vga.fillRect(ox, oy, OSD_FONT_W, barH, zxColor(7, 0));
                        // Thumb
                        int thumbH = (maxVisible * barH) / nlines;
                        if (thumbH < 3) thumbH = 3;
                        int thumbY = (scroll * barH) / nlines;
                        if (thumbY + thumbH > barH) thumbY = barH - thumbH;
                        VIDEO::vga.fillRect(ox + 1, oy + thumbY, OSD_FONT_W - 2, thumbH, zxColor(0, 0));
                        // Arrows
                        osdAt(topRow - 1, maxCols - 1);
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 0));
                        VIDEO::vga.print(scroll > 0 ? "+" : "-");
                        osdAt(topRow + maxVisible, maxCols - 1);
                        VIDEO::vga.print(scroll + maxVisible < nlines ? "+" : "-");
                    }
                };
                drawHelp();
                while (1) {
                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        if (ESPectrum::readKbd(&Nextkey)) {
                            if (!Nextkey.down) continue;
                            if (is_enter(Nextkey.vk) || is_back(Nextkey.vk)) break;
                            if (is_down(Nextkey.vk) && scroll + maxVisible < nlines) {
                                scroll++;
                                drawHelp();
                            }
                            if (is_up(Nextkey.vk) && scroll > 0) {
                                scroll--;
                                drawHelp();
                            }
                        }
                    }
                    sleep_ms(5);
                }
                click();
                if (VIDEO::OSD) OSD::drawStats();
                return;
            }
            else if (opt == 12) { // About
                // About
                // Protect OSD area from Z80 video renderer overwrite
                bool about_osd_enabled = (VIDEO::OSD != 0);
                if (!about_osd_enabled) {
                    if (Config::aspect_16_9)
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                    else
                        VIDEO::Draw_OSD43 = VIDEO::BottomBorder_OSD;
                    VIDEO::OSD = 0x04;
                }
                drawOSD(false);

                int osd_xi = osdInsideX();               // x inside OSD (with margin)
                int osd_y0 = osdInsideY() - OSD_MARGIN; // = scrAlignCenterY(OSD_H)

                VIDEO::vga.fillRect(Config::aspect_16_9 ? osd_xi + 20 : osd_xi,
                                    Config::aspect_16_9 ? osd_y0 - 8 : osd_y0 + 12,
                                    240, 50, zxColor(0, 0));

                // Decode Logo in EBF8 format
                // Logo pixels are stored as ZX Spectrum palette indices (0-15)
                uint8_t *logo = (uint8_t *)ESPectrum_logo;
                int pos_x = Config::aspect_16_9 ? osd_xi + 46 : osd_xi + 26;
                int pos_y = Config::aspect_16_9 ? osd_y0 + 3 : osd_y0 + 23;
                int logo_w = (logo[5] << 8) + logo[4]; // Get Width
                int logo_h = (logo[7] << 8) + logo[6]; // Get Height
                logo+=8; // Skip header
                for (int i=0; i < logo_h; i++)
                    for(int n=0; n<logo_w; n++) {
                        uint8_t zxIdx = logo[n+(i*logo_w)];
                        VIDEO::vga.dotFast(pos_x + n, pos_y + i, zxColor(zxIdx & 7, zxIdx >> 3));
                    }

                // About Page 1
                // osdAt(7, 0);
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                // VIDEO::vga.print(Config::lang ? OSD_ABOUT1_ES : OSD_ABOUT1_EN);

                pos_x = Config::aspect_16_9 ? osd_xi + 26 : osd_xi + 6;
                pos_y = Config::aspect_16_9 ? osd_y0 + 48 : osd_y0 + 68;
                int osdRow = 0; int osdCol = 0;
                int msgIndex = 0; int msgChar = 0;
                int msgDelay = 0; int cursorBlink = 16; int nextChar = 0;
                uint16_t cursorCol = zxColor(7,1);
                uint16_t cursorCol2 = zxColor(1,0);

                while (1) {

                    if (msgDelay == 0) {
                        nextChar = AboutMsg[Config::lang][msgIndex][msgChar];
                        if (nextChar != 13) {
                            if (nextChar == 10) {
                                char fore = AboutMsg[Config::lang][msgIndex][++msgChar];
                                char back = AboutMsg[Config::lang][msgIndex][++msgChar];
                                int foreint = (fore >= 'A') ? (fore - 'A' + 10) : (fore - '0');
                                int backint = (back >= 'A') ? (back - 'A' + 10) : (back - '0');
                                VIDEO::vga.setTextColor(zxColor(foreint & 0x7, foreint >> 3), zxColor(backint & 0x7, backint >> 3));
                                msgChar++;
                                continue;
                            } else {
                                VIDEO::vga.drawChar(pos_x + (osdCol * 6), pos_y + (osdRow * 8), nextChar);
                            }
                            msgChar++;
                        } else {
                            VIDEO::vga.fillRect(pos_x + (osdCol * 6), pos_y + (osdRow * 8), 6,8, zxColor(1, 0) );
                        }
                        osdCol++;
                        if (osdCol == 38) {
                            if (osdRow == 12) {
                                osdCol--;
                                msgDelay = 192;
                            } else {
                                VIDEO::vga.fillRect(pos_x + (osdCol * 6), pos_y + (osdRow * 8), 6,8, zxColor(1, 0) );
                                osdCol = 0;
                                msgChar++;
                                osdRow++;
                            }
                        }
                    } else {
                        msgDelay--;
                        if (msgDelay==0) {
                            VIDEO::vga.fillRect(Config::aspect_16_9 ? osd_xi+20 : osd_xi, Config::aspect_16_9 ? osd_y0+44 : osd_y0+64, 240, 114, zxColor(1, 0));
                            osdCol = 0;
                            osdRow  = 0;
                            msgChar = 0;
                            msgIndex++;
                            if (msgIndex==9) msgIndex = 0;
                        }
                    }

                    if (--cursorBlink == 0) {
                        uint16_t cursorSwap = cursorCol;
                        cursorCol = cursorCol2;
                        cursorCol2 = cursorSwap;
                        cursorBlink = 16;
                    }

                    VIDEO::vga.fillRect(pos_x + ((osdCol + 1) * 6), pos_y + (osdRow * 8), 6,8, cursorCol );

                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        if (ESPectrum::readKbd(&Nextkey)) {
                            if (!Nextkey.down) continue;
                            if (is_enter(Nextkey.vk) || is_back(Nextkey.vk == fabgl::VK_ESCAPE)) break;
                        }
                    }
                    sleep_ms(20);
                }
                click();
                // Restore video renderer if we enabled OSD protection for About
                if (!about_osd_enabled) {
                    VIDEO::OSD = 0;
                    if (Config::aspect_16_9)
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen;
                    else
                        VIDEO::Draw_OSD43 = VIDEO::BottomBorder;
                }
                if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
                return;
            }
#if TFT
            else if (FileUtils::fsMount && opt == 13) { // TFT
                menu_saverect = true;
                menu_curopt = 1;
                while(1) {
                    menu_level = 1;
                    uint8_t opt2 = menuRun(MENU_TFT[Config::lang]);
                    if (opt2 == 1) {
                        // INVERSION
                        uint8_t prev_inv = TFT_INVERSION;
                        TFT_INVERSION = !TFT_INVERSION;
                        if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                            Config::save();
                            esp_hard_reset();
                        } else {
                            TFT_INVERSION = prev_inv;
                        }
                    }
                    else if (opt2 == 2) {
                        // FLAGS
                        menu_level = 2;
                        menu_saverect = true;
                        while (1) {
                            uint8_t opt2 = menuRun(MENU_TFT2[Config::lang]);
                            uint8_t prev_flags = TFT_FLAGS;
                            if (opt2 == 1) {
                                TFT_FLAGS = (TFT_FLAGS & MADCTL_BGR_PIXEL_ORDER) ? (TFT_FLAGS & ~MADCTL_BGR_PIXEL_ORDER) : (TFT_FLAGS | MADCTL_BGR_PIXEL_ORDER);
                                if (confirmReboot(OSD_DLG_APPLYREBOOT)) { Config::save(); esp_hard_reset(); }
                                else TFT_FLAGS = prev_flags;
                            }
                            else if (opt2 == 2) {
                                TFT_FLAGS = (TFT_FLAGS & MADCTL_MX) ? (TFT_FLAGS & ~MADCTL_MX) : (TFT_FLAGS | MADCTL_MX);
                                if (confirmReboot(OSD_DLG_APPLYREBOOT)) { Config::save(); esp_hard_reset(); }
                                else TFT_FLAGS = prev_flags;
                            }
                            else if (opt2 == 3) {
                                TFT_FLAGS = (TFT_FLAGS & MADCTL_MY) ? (TFT_FLAGS & ~MADCTL_MY) : (TFT_FLAGS | MADCTL_MY);
                                if (confirmReboot(OSD_DLG_APPLYREBOOT)) { Config::save(); esp_hard_reset(); }
                                else TFT_FLAGS = prev_flags;
                            }
                            else if (opt2 == 4) {
                                TFT_FLAGS = (TFT_FLAGS & MADCTL_MX) ? (TFT_FLAGS & ~MADCTL_MX) : (TFT_FLAGS | MADCTL_MX);
                                TFT_FLAGS = (TFT_FLAGS & MADCTL_MY) ? (TFT_FLAGS & ~MADCTL_MY) : (TFT_FLAGS | MADCTL_MY);
                                if (confirmReboot(OSD_DLG_APPLYREBOOT)) { Config::save(); esp_hard_reset(); }
                                else TFT_FLAGS = prev_flags;
                            } else {
                                menu_level = 1;
                                menu_curopt = 2;
                                break;
                            }
                        }
                    }
                    else if (opt2 == 3) {
                        uint8_t prev_inv = TFT_INVERSION;
                        uint8_t prev_flags = TFT_FLAGS;
                        TFT_INVERSION = 0;
                        TFT_FLAGS = MADCTL_ROW_COLUMN_EXCHANGE | MADCTL_BGR_PIXEL_ORDER;
                        if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                            Config::save();
                            esp_hard_reset();
                        } else {
                            TFT_INVERSION = prev_inv;
                            TFT_FLAGS = prev_flags;
                        }
                    } else {
                        menu_curopt = 9;
                        break;
                    }
                }
            }
#endif
            else break;
          }
        }
}


// Shows a red panel with error text
void OSD::errorPanel(const string& errormsg) {
    unsigned short x = scrAlignCenterX(OSD_W);
    unsigned short y = scrAlignCenterY(OSD_H);

    if (Config::slog_on)
        printf((errormsg + "\n").c_str());

    VIDEO::vga.fillRect(x, y, OSD_W, OSD_H, zxColor(0, 0));
    VIDEO::vga.rect(x, y, OSD_W, OSD_H, zxColor(7, 0));
    VIDEO::vga.rect(x + 1, y + 1, OSD_W - 2, OSD_H - 2, zxColor(2, 1));
    VIDEO::vga.setFont(Font6x8);
    osdHome();
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(2, 1));
    VIDEO::vga.print(ERROR_TITLE);
    osdAt(2, 0);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.println(errormsg.c_str());
    osdAt(17, 0);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(2, 1));
    VIDEO::vga.print(ERROR_BOTTOM);
}

// Error panel and infinite loop
void OSD::errorHalt(const string& errormsg) {
    errorPanel(errormsg);
    while (1) {
        sleep_ms(5);
    }
}

// W/A
extern "C" void osd_printf(const char* msg, ...) {
    OSD::osdCenteredMsg(msg, LEVEL_WARN, 1000);
}

// Centered message
void OSD::osdCenteredMsg(const string& msg, uint8_t warn_level) {
    osdCenteredMsg(msg,warn_level,1000);
}

void OSD::osdCenteredMsg(const string& msg, uint8_t warn_level, uint16_t millispause) {

    // Count lines and find the longest line for proper sizing
    unsigned short nlines = 1;
    size_t maxlen = 0;
    size_t pos = 0, prev = 0;
    while ((pos = msg.find('\n', prev)) != string::npos) {
        size_t len = pos - prev;
        if (len > maxlen) maxlen = len;
        nlines++;
        prev = pos + 1;
    }
    size_t len = msg.length() - prev;
    if (len > maxlen) maxlen = len;

    size_t maxchars = (scrW / 6) - 10;
    if (maxlen > maxchars) maxlen = maxchars;

    const unsigned short h = OSD_FONT_H * (nlines + 2);
    const unsigned short y = scrAlignCenterY(h);
    unsigned short paper;
    unsigned short ink;
    unsigned int j;

    const unsigned short w = (maxlen + 2) * OSD_FONT_W;
    const unsigned short x = scrAlignCenterX(w);

    switch (warn_level) {
    case LEVEL_OK:
        ink = zxColor(7, 1);
        paper = zxColor(4, 0);
        break;
    case LEVEL_ERROR:
        ink = zxColor(7, 1);
        paper = zxColor(2, 0);
        break;
    case LEVEL_WARN:
        ink = zxColor(0, 0);
        paper = zxColor(6, 0);
        break;
    default:
        ink = zxColor(7, 0);
        paper = zxColor(1, 0);
    }

    if (millispause > 0) {
        // Save backbuffer data
        VIDEO::SaveRect.save(x, y, w, h);
    }

    VIDEO::vga.fillRect(x, y, w, h, paper);
    // VIDEO::vga.rect(x - 1, y - 1, w + 2, h + 2, ink);
    VIDEO::vga.setTextColor(ink, paper);
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(x + OSD_FONT_W, y + OSD_FONT_H);
    VIDEO::vga.print(msg.c_str());

    if (millispause > 0) {
        sleep_ms(millispause); // Pause if needed
        VIDEO::SaveRect.restore_last();
    }
}

const static char* mnem[256] {
    "NOP", // 00
    "LD BC,nn", // 01
    "LD (BC),A", // 02
    "INC BC", // 03
    "INC B", // 04
    "DEC B", // 05
    "LD B,n", // 06
    "RLCA", // 07
    "EX AF,AF'", // 08
    "ADD HL,BC", // 09
    "LD A,(BC)", // 0A
    "DEC BC", // 0B
    "INC C", // 0C
    "DEC C", // 0D
    "LD C,n", // 0E
    "RRCA", // 0F

    "DJNZ d", // 10
    "LD DE,nn", // 11
    "LD (DE),A", // 12
    "INC DE", // 13
    "INC D", // 14
    "DEC D", // 15
    "LD D,n", // 16
    "RLA", // 17
    "JR d", // 18
    "ADD HL,DE", // 19
    "LD A,(DE)", // 1A
    "DEC DE", // 1B
    "INC E", // 1C
    "DEC E", // 1D
    "LD E,n", // 1E
    "RRA", // 1F

    "JR NZ d", // 20
    "LD HL,nn", // 21
    "LD (nn),HL", // 22
    "INC HL", // 23
    "INC H", // 24
    "DEC H", // 25
    "LD H,n", // 26
    "DAA", // 27
    "JR Z,d", // 28
    "ADD HL,HL", // 29
    "LD HL,(nn)", // 2A
    "DEC HL", // 2B
    "INC L", // 2C
    "DEC L", // 2D
    "LD L,n", // 2E
    "CPL", // 2F

    "JR NC d", // 30
    "LD SP,nn", // 31
    "LD (nn),A", // 32
    "INC SP", // 33
    "INC (HL)", // 34
    "DEC (HL)", // 35
    "LD (HL),n", // 36
    "SCF", // 37
    "JR C,d", // 38
    "ADD HL,SP", // 39
    "LD A,(nn)", // 3A
    "DEC SP", // 3B
    "INC A", // 3C
    "DEC A", // 3D
    "LD A,n", // 3E
    "CCF", // 3F

    "LD B,B" , // 40
    "LD B,C", // 41
    "LD B,D", // 42
    "LD B,E", // 43
    "LD B,H", // 44
    "LD B,L", // 45
    "LD B,(HL)", // 46
    "LD B,A", // 47
    "LD C,B", // 48
    "LD C,C", // 49
    "LD C,D", // 4A
    "LD C,E", // 4B
    "LD C,H", // 4C
    "LD C,L", // 4D
    "LD C,(HL)", // 4E
    "LD C,A", // 4F

    "LD D,B", // 50
    "LD D,C", // 51
    "LD D,D", // 52
    "LD D,E", // 53
    "LD D,H", // 54
    "LD D,L", // 55
    "LD D,(HL)", // 56
    "LD D,A", // 57
    "LD E,B", // 58
    "LD E,C", // 59
    "LD E,D", // 5A
    "LD E,E", // 5B
    "LD E,H", // 5C
    "LD E,L", // 5D
    "LD E,(HL)", // 5E
    "LD E,A", // 5F

    "LD H,B", // 60
    "LD H,C", // 61
    "LD H,D", // 62
    "LD H,E", // 63
    "LD H,H", // 64
    "LD H,L", // 65
    "LD H,(HL)", // 66
    "LD H,A", // 67
    "LD L,B", // 68
    "LD L,C", // 69
    "LD L,D", // 6A
    "LD L,E", // 6B
    "LD L,H", // 6C
    "LD L,L", // 6D
    "LD L,(HL)", // 6E
    "LD L,A", // 6F

    "LD (HL),B", // 70
    "LD (HL),C", // 71
    "LD (HL),D", // 72
    "LD (HL),E", // 73
    "LD (HL),H", // 74
    "LD (HL),L", // 75
    "HALT", // 76
    "LD (HL),A", // 77
    "LD A,B", // 78
    "LD A,C", // 79
    "LD A,D", // 7A
    "LD A,E", // 7B
    "LD A,H", // 7C
    "LD A,L", // 7D
    "LD A,(HL)", // 7E
    "LD A,A", // 7F

    "ADD A,B", // 80
    "ADD A,C", // 81
    "ADD A,D", // 82
    "ADD A,E", // 83
    "ADD A,H", // 84
    "ADD A,L", // 85
    "ADD A,(HL)", // 86
    "ADD A,A", // 87
    "ADC A,B", // 88
    "ADC A,C", // 89
    "ADC A,D", // 8A
    "ADC A,E", // 8B
    "ADC A,H", // 8C
    "ADC A,L", // 8D
    "ADC A,(HL)", // 8E
    "ADC A,A", // 8F

    "SUB A,B", // 90
    "SUB A,C", // 91
    "SUB A,D", // 92
    "SUB A,E", // 93
    "SUB A,H", // 94
    "SUB A,L", // 95
    "SUB A,(HL)", // 96
    "SUB A,A", // 97
    "SBC A,B", // 98
    "SBC A,C", // 99
    "SBC A,D", // 9A
    "SBC A,E", // 9B
    "SBC A,H", // 9C
    "SBC A,L", // 9D
    "SBC A,(HL)", // 9E
    "SBC A,A", // 9F

    "AND A,B", // A0
    "AND A,C", // A1
    "AND A,D", // A2
    "AND A,E", // A3
    "AND A,H", // A4
    "AND A,L", // A5
    "AND A,(HL)", // A6
    "AND A,A", // A7
    "XOR A,B", // A8
    "XOR A,C", // A9
    "XOR A,D", // AA
    "XOR A,E", // AB
    "XOR A,H", // AC
    "XOR A,L", // AD
    "XOR A,(HL)", // AE
    "XOR A,A", // AF

    "OR A,B", // B0
    "OR A,C", // B1
    "OR A,D", // B2
    "OR A,E", // B3
    "OR A,H", // B4
    "OR A,L", // B5
    "OR A,(HL)", // B6
    "OR A,A", // B7
    "CP A,B", // B8
    "CP A,C", // B9
    "CP A,D", // BA
    "CP A,E", // BB
    "CP A,H", // BC
    "CP A,L", // BD
    "CP A,(HL)", // BE
    "CP A,A", // BF

    "RET NZ", // C0
    "POP BC", // C1
    "JP NZ,(nn)", // C2
    "JP (nn)", // C3
    "CALL NZ,(nn)", // C4
    "PUSH BC", // C5
    "ADD A,n", // C6
    "RST 0H", // C7
    "RET Z", // C8
    "RET", // C9
    "JP Z,(nn)", // CA
    "bo", // CB
    "CALL Z,(nn)", // CC
    "CALL (nn)", // CD
    "ADC A,n", // CE
    "RST 8H", // CF

    "RET NC", // D0
    "POP DE", // D1
    "JP NC,(nn)", // D2
    "OUT (n),A", // D3
    "CALL NC,(nn)", // D4
    "PUSH DE", // D5
    "SUB A,n", // D6
    "RST 10H", // D7
    "RET C", // D8
    "EXX", // D9
    "JP C,(nn)", // DA
    "IN A,(n)", // DB
    "CALL C,(nn)", // DC
    "op IX:", // DD
    "SBC A,n", // DE
    "RST 18H", // DF

    "RET PO", // E0
    "POP HL", // E1
    "JP PO,(nn)", // E2
    "EX (SP),HL", // E3
    "CALL PO,(nn)", // E4
    "PUSH HL", // E5
    "AND A,n", // E6
    "RST 20H", // E7
    "RET PE", // E8
    "JP (HL)", // E9
    "JP PE,(nn)", // EA
    "EX DE,HL", // EB
    "CALL PE,(nn)", // EC
    "ext:", // ED
    "XOR A,n", // EE
    "RST 28H", // EF

    "RET P", // F0
    "POP AF", // F1
    "JP P,(nn)", // F2
    "DI", // F3
    "CALL P,(nn)", // F4
    "PUSH AF", // F5
    "OR A,n", // F6
    "RST 30H", // F7
    "RET M", // F8
    "LD SP,HL", // F9
    "JP M,(nn)", // FA
    "EI", // FB
    "CALL M,(nn)", // FC
    "op IY:", // FD
    "CP A,n", // FE
    "RST 38H", // FF
};

const char* mnemIX(uint8_t b) {
    switch(b) {
        case 0x09: return "ADD IX,BC";

        case 0x19: return "ADD IX,DE";

        case 0x21: return "LD IX,nn";
        case 0x22: return "LD (nn),IX";
        case 0x23: return "INC IX";
        case 0x24: return "INC IXh";
        case 0x25: return "DEC IXh";
        case 0x26: return "LD IXh,n";

        case 0x29: return "ADD IX,IX";
        case 0x2A: return "LD IX,(nn)";
        case 0x2B: return "DEC IX";
        case 0x2C: return "INC IXl";
        case 0x2D: return "DEC IXl";
        case 0x2E: return "LD IXl,n";

        case 0x34: return "INC (IX+d)";
        case 0x35: return "DEC (IX+d)";
        case 0x36: return "LD (IX+d),n";

        case 0x39: return "ADD IX,SP";

        case 0x44: return "LD B,IXh";
        case 0x45: return "LD B,IXl";
        case 0x46: return "LD B,(IX+d)";

        case 0x4C: return "LD C,IXh";
        case 0x4D: return "LD C,IXl";
        case 0x4E: return "LD C,(IX+d)";

        case 0x54: return "LD D,IXh";
        case 0x55: return "LD D,IXl";
        case 0x56: return "LD D,(IX+d)";

        case 0x5C: return "LD E,IXh";
        case 0x5D: return "LD E,IXl";
        case 0x5E: return "LD E,(IX+d)";

        case 0x60: return "LD IXh,B";
        case 0x61: return "LD IXh,C";
        case 0x62: return "LD IXh,D";
        case 0x63: return "LD IXh,E";
        case 0x64: return "LD IXh,IXh";
        case 0x65: return "LD IXh,IXl";
        case 0x66: return "LD H,(IX+n)";
        case 0x67: return "LD IXh,A";
        case 0x68: return "LD IXl,B";
        case 0x69: return "LD IXl,C";
        case 0x6A: return "LD IXl,D";
        case 0x6B: return "LD IXl,E";
        case 0x6C: return "LD IXl,IXh";
        case 0x6D: return "LD IXl,IXl";
        case 0x6E: return "LD L,(IX+n)";
        case 0x6F: return "LD IXl,A";

        case 0x70: return "LD (IX+d),B";
        case 0x71: return "LD (IX+d),C";
        case 0x72: return "LD (IX+d),D";
        case 0x73: return "LD (IX+d),E";
        case 0x74: return "LD (IX+d),H";
        case 0x75: return "LD (IX+d),L";

        case 0x77: return "LD (IX+d),A";

        case 0x7C: return "LD A,IXh";
        case 0x7D: return "LD A,IXl";
        case 0x7E: return "LD A,(IX+d)";

        case 0x84: return "ADD A,IXh";
        case 0x85: return "ADD A,IXl";
        case 0x86: return "ADD A,(IX+d)";

        case 0x8C: return "ADC A,IXh";
        case 0x8D: return "ADC A,IXl";
        case 0x8E: return "ADC A,(IX+d)";

        case 0x94: return "SUB A,IXh";
        case 0x95: return "SUB A,IXl";
        case 0x96: return "SUB A,(IX+d)";

        case 0x9C: return "SBC A,IXh";
        case 0x9D: return "SBC A,IXl";
        case 0x9E: return "SBC A,(IX+d)";

        case 0xA4: return "AND A,IXh";
        case 0xA5: return "AND A,IXl";
        case 0xA6: return "AND A,(IX+d)";

        case 0xAC: return "XOR A,IXh";
        case 0xAD: return "XOR A,IXl";
        case 0xAE: return "XOR A,(IX+d)";

        case 0xB4: return "OR A,IXh";
        case 0xB5: return "OR A,IXl";
        case 0xB6: return "OR A,(IX+d)";

        case 0xBC: return "CP A,IXh";
        case 0xBD: return "CP A,IXl";
        case 0xBE: return "CP A,(IX+d)";

        case 0xCB: return "IX bits:";

        case 0xE1: return "POP IX";

        case 0xE3: return "EX (SP),IX";

        case 0xE5: return "PUSH IX";

        case 0xE9: return "JP (IX)";

        case 0xF9: return "LD SP,IX";
    }
    return "???";
}

const char* mnemED(uint8_t b) {
    switch(b) {
        case 0x40: return "IN B,(C)";
        case 0x41: return "OUT (C),B";
        case 0x42: return "SBC HL,BC";
        case 0x43: return "LD (nn),BC";
        case 0x44: return "NEG";
        case 0x45: return "RETN";
        case 0x46: return "IM 0";
        case 0x47: return "LD I,A";
        case 0x48: return "IN C,(C)";
        case 0x49: return "OUT (C),C";
        case 0x4A: return "ADC HL,BC";
        case 0x4B: return "LD BC,(nn)";
        case 0x4C: return "NEG";
        case 0x4D: return "RETI";
        case 0x4E: return "IM 0/1";
        case 0x4F: return "LD R,A";

        case 0x50: return "IN D,(C)";
        case 0x51: return "OUT (C),D";
        case 0x52: return "SBC HL,DE";
        case 0x53: return "LD (nn),DE";
        case 0x54: return "NEG";
        case 0x55: return "RETN";
        case 0x56: return "IM 1";
        case 0x57: return "LD A,I";
        case 0x58: return "IN E,(C)";
        case 0x59: return "OUT (C),E";
        case 0x5A: return "ADC HL,DE";
        case 0x5B: return "LD DE,(nn)";
        case 0x5C: return "NEG";
        case 0x5D: return "RETN";
        case 0x5E: return "IM 2";
        case 0x5F: return "LD A,R";

        case 0x60: return "IN H,(C)";
        case 0x61: return "OUT (C),H";
        case 0x62: return "SBC HL,HL";
        case 0x63: return "LD (nn),HL";
        case 0x64: return "NEG";
        case 0x65: return "RETN";
        case 0x66: return "IM 0";
        case 0x67: return "RRD";
        case 0x68: return "IN L,(C)";
        case 0x69: return "OUT (C),L";
        case 0x6A: return "ADC HL,HL";
        case 0x6B: return "LD HL,(nn)";
        case 0x6C: return "NEG";
        case 0x6D: return "RETN";
        case 0x6E: return "IM 0/1";
        case 0x6F: return "RLD";

        case 0x70: return "IN F,(C)";
        case 0x71: return "OUT (C),0";
        case 0x72: return "SBC HL,SP";
        case 0x73: return "LD (nn),SP";
        case 0x74: return "NEG";
        case 0x75: return "RETN";
        case 0x76: return "IM 1";

        case 0x78: return "IN A,(C)";
        case 0x79: return "OUT (C),A";
        case 0x7A: return "ADC HL,SP";
        case 0x7B: return "LD SP,(nn)";
        case 0x7C: return "NEG";
        case 0x7D: return "RETN";
        case 0x7E: return "IM 2";

        case 0xA0: return "LDI";
        case 0xA1: return "CPI";
        case 0xA2: return "INI";
        case 0xA3: return "OUTI";

        case 0xA8: return "LDD";
        case 0xA9: return "CPD";
        case 0xAA: return "IND";
        case 0xAB: return "OUTD";

        case 0xB0: return "LDIR";
        case 0xB1: return "CPIR";
        case 0xB2: return "INIR";
        case 0xB3: return "OUTIR";

        case 0xB8: return "LDDR";
        case 0xB9: return "CPDR";
        case 0xBA: return "INDR";
        case 0xBB: return "OUTDR";
    }
    return "???";
}

const char* mnemCB[256] = {
    "RLC B", // 00
    "RLC C", // 01
    "RLC D", // 02
    "RLC E", // 03
    "RLC H", // 04
    "RLC L", // 05
    "RLC (HL)", // 06
    "RLC A", // 07
    "RRC B", // 08
    "RRC C", // 09
    "RRC D", // 0A
    "RRC E", // 0B
    "RRC H", // 0C
    "RRC L", // 0D
    "RRC (HL)", // 0E
    "RRC A", // 0F

    "RL B", // 10
    "RL C", // 11
    "RL D", // 12
    "RL E", // 13
    "RL H", // 14
    "RL L", // 15
    "RL (HL)", // 16
    "RL A", // 17
    "RR B", // 18
    "RR C", // 19
    "RR D", // 1A
    "RR E", // 1B
    "RR H", // 1C
    "RR L", // 1D
    "RR (HL)", // 1E
    "RR A", // 1F

    "SLA B", // 20
    "SLA C", // 21
    "SLA D", // 22
    "SLA E", // 23
    "SLA H", // 24
    "SLA L", // 25
    "SLA (HL)", // 26
    "SLA A", // 27
    "SRA B", // 28
    "SRA C", // 29
    "SRA D", // 2A
    "SRA E", // 2B
    "SRA H", // 2C
    "SRA L", // 2D
    "SRA (HL)", // 2E
    "SRA A", // 2F

    "SLL B", // 30
    "SLL C", // 31
    "SLL D", // 32
    "SLL E", // 33
    "SLL H", // 34
    "SLL L", // 35
    "SLL (HL)", // 36
    "SLL A", // 37
    "SRL B", // 38
    "SRL C", // 39
    "SRL D", // 3A
    "SRL E", // 3B
    "SRL H", // 3C
    "SRL L", // 3D
    "SRL (HL)", // 3E
    "SRL A", // 3F

    "BIT 0,B", // 40
    "BIT 0,C", // 41
    "BIT 0,D", // 42
    "BIT 0,E", // 43
    "BIT 0,H", // 44
    "BIT 0,L", // 45
    "BIT 0,(HL)", // 46
    "BIT 0,A", // 47
    "BIT 1,B", // 48
    "BIT 1,C", // 49
    "BIT 1,D", // 4A
    "BIT 1,E", // 4B
    "BIT 1,H", // 4C
    "BIT 1,L", // 4D
    "BIT 1,(HL)", // 4E
    "BIT 1,A", // 4F

    "BIT 2,B", // 50
    "BIT 2,C", // 51
    "BIT 2,D", // 52
    "BIT 2,E", // 53
    "BIT 2,H", // 54
    "BIT 2,L", // 55
    "BIT 2,(HL)", // 56
    "BIT 2,A", // 57
    "BIT 3,B", // 58
    "BIT 3,C", // 59
    "BIT 3,D", // 5A
    "BIT 3,E", // 5B
    "BIT 3,H", // 5C
    "BIT 3,L", // 5D
    "BIT 3,(HL)", // 5E
    "BIT 3,A", // 5F

    "BIT 4,B", // 60
    "BIT 4,C", // 61
    "BIT 4,D", // 62
    "BIT 4,E", // 63
    "BIT 4,H", // 64
    "BIT 4,L", // 65
    "BIT 4,(HL)", // 66
    "BIT 4,A", // 67
    "BIT 5,B", // 68
    "BIT 5,C", // 69
    "BIT 5,D", // 6A
    "BIT 5,E", // 6B
    "BIT 5,H", // 6C
    "BIT 5,L", // 6D
    "BIT 5,(HL)", // 6E
    "BIT 5,A", // 6F

    "BIT 6,B", // 70
    "BIT 6,C", // 71
    "BIT 6,D", // 72
    "BIT 6,E", // 73
    "BIT 6,H", // 74
    "BIT 6,L", // 75
    "BIT 6,(HL)", // 76
    "BIT 6,A", // 77
    "BIT 7,B", // 78
    "BIT 7,C", // 79
    "BIT 7,D", // 7A
    "BIT 7,E", // 7B
    "BIT 7,H", // 7C
    "BIT 7,L", // 7D
    "BIT 7,(HL)", // 7E
    "BIT 7,A", // 7F

    "RES 0,B", // 80
    "RES 0,C", // 81
    "RES 0,D", // 82
    "RES 0,E", // 83
    "RES 0,H", // 84
    "RES 0,L", // 85
    "RES 0,(HL)", // 86
    "RES 0,A", // 87
    "RES 1,B", // 88
    "RES 1,C", // 89
    "RES 1,D", // 8A
    "RES 1,E", // 8B
    "RES 1,H", // 8C
    "RES 1,L", // 8D
    "RES 1,(HL)", // 8E
    "RES 1,A", // 8F

    "RES 2,B", // 90
    "RES 2,C", // 91
    "RES 2,D", // 92
    "RES 2,E", // 93
    "RES 2,H", // 94
    "RES 2,L", // 95
    "RES 2,(HL)", // 96
    "RES 2,A", // 97
    "RES 3,B", // 98
    "RES 3,C", // 99
    "RES 3,D", // 9A
    "RES 3,E", // 9B
    "RES 3,H", // 9C
    "RES 3,L", // 9D
    "RES 3,(HL)", // 9E
    "RES 3,A", // 9F

    "RES 4,B", // A0
    "RES 4,C", // A1
    "RES 4,D", // A2
    "RES 4,E", // A3
    "RES 4,H", // A4
    "RES 4,L", // A5
    "RES 4,(HL)", // A6
    "RES 4,A", // A7
    "RES 5,B", // A8
    "RES 5,C", // A9
    "RES 5,D", // AA
    "RES 5,E", // AB
    "RES 5,H", // AC
    "RES 5,L", // AD
    "RES 5,(HL)", // AE
    "RES 5,A", // AF

    "RES 6,B", // B0
    "RES 6,C", // B1
    "RES 6,D", // B2
    "RES 6,E", // B3
    "RES 6,H", // B4
    "RES 6,L", // B5
    "RES 6,(HL)", // B6
    "RES 6,A", // B7
    "RES 7,B", // B8
    "RES 7,C", // B9
    "RES 7,D", // BA
    "RES 7,E", // BB
    "RES 7,H", // BC
    "RES 7,L", // BD
    "RES 7,(HL)", // BE
    "RES 7,A", // BF

    "SET 0,B", // C0
    "SET 0,C", // C1
    "SET 0,D", // C2
    "SET 0,E", // C3
    "SET 0,H", // C4
    "SET 0,L", // C5
    "SET 0,(HL)", // C6
    "SET 0,A", // C7
    "SET 1,B", // C8
    "SET 1,C", // C9
    "SET 1,D", // CA
    "SET 1,E", // CB
    "SET 1,H", // CC
    "SET 1,L", // CD
    "SET 1,(HL)", // CE
    "SET 1,A", // CF

    "SET 2,B", // 90
    "SET 2,C", // D1
    "SET 2,D", // D2
    "SET 2,E", // D3
    "SET 2,H", // D4
    "SET 2,L", // D5
    "SET 2,(HL)", // D6
    "SET 2,A", // D7
    "SET 3,B", // D8
    "SET 3,C", // D9
    "SET 3,D", // DA
    "SET 3,E", // DB
    "SET 3,H", // DC
    "SET 3,L", // DD
    "SET 3,(HL)", // DE
    "SET 3,A", // DF

    "SET 4,B", // E0
    "SET 4,C", // E1
    "SET 4,D", // E2
    "SET 4,E", // E3
    "SET 4,H", // E4
    "SET 4,L", // E5
    "SET 4,(HL)", // E6
    "SET 4,A", // E7
    "SET 5,B", // E8
    "SET 5,C", // E9
    "SET 5,D", // EA
    "SET 5,E", // EB
    "SET 5,H", // EC
    "SET 5,L", // ED
    "SET 5,(HL)", // EE
    "SET 5,A", // EF

    "SET 6,B", // F0
    "SET 6,C", // F1
    "SET 6,D", // F2
    "SET 6,E", // F3
    "SET 6,H", // F4
    "SET 6,L", // F5
    "SET 6,(HL)", // F6
    "SET 6,A", // F7
    "SET 7,B", // F8
    "SET 7,C", // F9
    "SET 7,D", // FA
    "SET 7,E", // FB
    "SET 7,H", // FC
    "SET 7,L", // FD
    "SET 7,(HL)", // FE
    "SET 7,A" // FF
};

#define BNc(x, b) ((x >> b) & 1 ? '1' : '0')

/// TODO:
static uint16_t dump_pc = 0;

static void saveDumpToFile(uint16_t addr_from, uint16_t addr_to) {
    string fname = string(MOUNT_POINT_SD) + "/dump.log";
    FIL* f = fopen2(fname.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) return;

    char line[128];
    UINT bw;

    // Separator
    f_write(f, "========================================\n", 41, &bw);

    // Timestamp
    snprintf(line, sizeof(line), "Dump: #%04X - #%04X\n", addr_from, addr_to);
    f_write(f, line, strlen(line), &bw);

    // Machine info
    snprintf(line, sizeof(line), "Arch: %s  RomSet: %s\n", Config::arch.c_str(), Config::romSet.c_str());
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "ROM in use: %d  romLatch: %d  bankLatch: %d  videoLatch: %d\n",
        MemESP::romInUse, MemESP::romLatch, MemESP::bankLatch, MemESP::videoLatch);
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "pagingLock: %d  page0ram: %d  newSRAM: %d  divmmc: %d\n",
        MemESP::pagingLock, MemESP::page0ram, MemESP::newSRAM,
#if !PICO_RP2040
        MemESP::divmmc_mapped
#else
        0
#endif
    );
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "TR-DOS: %s  TR-DOS BIOS: %d\n",
        ESPectrum::trdos ? "on" : "off", Config::trdosBios);
    f_write(f, line, strlen(line), &bw);

    // Registers
    f_write(f, "--- Registers ---\n", 18, &bw);

    snprintf(line, sizeof(line), "AF=%04X  BC=%04X  DE=%04X  HL=%04X\n",
        Z80::getRegAF(), Z80::getRegBC(), Z80::getRegDE(), Z80::getRegHL());
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "AF'=%04X BC'=%04X DE'=%04X HL'=%04X\n",
        Z80::getRegAFx(), Z80::getRegBCx(), Z80::getRegDEx(), Z80::getRegHLx());
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "IX=%04X  IY=%04X  SP=%04X  PC=%04X\n",
        Z80::getRegIX(), Z80::getRegIY(), Z80::getRegSP(), Z80::getRegPC());
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "I=%02X R=%02X  IM=%d  IFF1=%d IFF2=%d  Halted=%d\n",
        Z80::getRegI(), Z80::getRegR(), Z80::getIntMode(),
        Z80::isIFF1(), Z80::isIFF2(), Z80::isHalted());
    f_write(f, line, strlen(line), &bw);

    // Flags
    uint8_t fl = Z80::getRegAF() & 0xFF;
    snprintf(line, sizeof(line), "Flags: S=%d Z=%d H=%d P=%d N=%d C=%d\n",
        (fl >> 7) & 1, (fl >> 6) & 1, (fl >> 4) & 1,
        (fl >> 2) & 1, (fl >> 1) & 1, fl & 1);
    f_write(f, line, strlen(line), &bw);

    // Stack top 8 words
    f_write(f, "--- Stack (top 8) ---\n", 22, &bw);
    uint16_t sp = Z80::getRegSP();
    for (int i = 0; i < 8; i++) {
        uint16_t addr = sp + i * 2;
        uint16_t val = MemESP::readbyte(addr) | (MemESP::readbyte(addr + 1) << 8);
        snprintf(line, sizeof(line), "  SP+%02X [%04X] = %04X\n", i * 2, addr, val);
        f_write(f, line, strlen(line), &bw);
    }

    snprintf(line, sizeof(line), "CPU T-states: %u  statesInFrame: %u\n",
        CPU::tstates, CPU::statesInFrame);
    f_write(f, line, strlen(line), &bw);

    // TR-DOS / WD1793 state
    f_write(f, "--- TR-DOS / WD1793 ---\n", 24, &bw);
    rvmWD1793 &wd = ESPectrum::fdd;
    snprintf(line, sizeof(line), "TR-DOS: %s  BIOS: %d  fastmode: %d  WP: A=%d B=%d C=%d D=%d\n",
        ESPectrum::trdos ? "on" : "off", Config::trdosBios,
        Config::trdosFastMode,
        Config::driveWP[0], Config::driveWP[1], Config::driveWP[2], Config::driveWP[3]);
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "WD1793: cmd=%02X status=%04X track=%d sector=%d data=%02X\n",
        wd.command, wd.status, wd.track, wd.sector, wd.data);
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "  drive=%d side=%d dsr=%d led=%d retry=%d\n",
        wd.diskS, wd.side, wd.dsr, wd.led, wd.retry);
    f_write(f, line, strlen(line), &bw);

    snprintf(line, sizeof(line), "  state=%u stepState=%u control=%04X\n",
        wd.state, wd.stepState, wd.control);
    f_write(f, line, strlen(line), &bw);

    for (int d = 0; d < 4; d++) {
        if (wd.disk[d]) {
            snprintf(line, sizeof(line), "  Disk%d: trk=%u sides=%d wp=%d %s%s\n",
                d, wd.disk[d]->tracks, wd.disk[d]->sides,
                wd.disk[d]->writeprotect,
                wd.disk[d]->IsSCLFile ? "[SCL] " : "",
                wd.disk[d]->fname.c_str());
            f_write(f, line, strlen(line), &bw);
        }
    }

    // Memory dump
    f_write(f, "--- Memory dump ---\n", 20, &bw);

    uint32_t from = addr_from;
    uint32_t to = addr_to;
    if (to < from) to += 0x10000; // wrap around

    for (uint32_t a = from & 0xFFF0; a <= to; a += 16) {
        uint16_t addr = a & 0xFFFF;
        int n = snprintf(line, sizeof(line),
            "%04X: %02X %02X %02X %02X %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X %02X %02X  |",
            addr,
            MemESP::readbyte(addr+0),  MemESP::readbyte(addr+1),
            MemESP::readbyte(addr+2),  MemESP::readbyte(addr+3),
            MemESP::readbyte(addr+4),  MemESP::readbyte(addr+5),
            MemESP::readbyte(addr+6),  MemESP::readbyte(addr+7),
            MemESP::readbyte(addr+8),  MemESP::readbyte(addr+9),
            MemESP::readbyte(addr+10), MemESP::readbyte(addr+11),
            MemESP::readbyte(addr+12), MemESP::readbyte(addr+13),
            MemESP::readbyte(addr+14), MemESP::readbyte(addr+15));
        // ASCII representation
        for (int j = 0; j < 16; j++) {
            uint8_t ch = MemESP::readbyte((addr + j) & 0xFFFF);
            line[n++] = (ch >= 0x20 && ch < 0x7F) ? ch : '.';
        }
        line[n++] = '|';
        line[n++] = '\n';
        line[n] = 0;
        f_write(f, line, n, &bw);
    }

    f_write(f, "\n", 1, &bw);
    fclose2(f);
}

void OSD::osdDump() {
    const unsigned short h = OSD_FONT_H * 22;
    const unsigned short y = scrAlignCenterY(h);
    const unsigned short w = OSD_FONT_W * 46;
    const unsigned short x = scrAlignCenterX(w);

    VIDEO::SaveRect.save(x - 1, y - 1, w + 2, h + 2);
    char buf[44];
    // Set font
    VIDEO::vga.setFont(Font6x8);

    // Boarder
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print("Dump");

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }

    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));

c:
    int xi = x + 1;

    for (int i = 0; i < 20; ++i) {
        uint16_t pci = (dump_pc + i * 16) & 0b1111111111110000;
        int yi = y + (i + 1) * OSD_FONT_H + 2;
        VIDEO::vga.setCursor(xi, yi);
        snprintf(
            buf, 46, "%04X  %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X %02X%02X",
            pci,
            MemESP::readbyte(pci),
            MemESP::readbyte(pci+1),
            MemESP::readbyte(pci+2),
            MemESP::readbyte(pci+3),

            MemESP::readbyte(pci+4),
            MemESP::readbyte(pci+5),
            MemESP::readbyte(pci+6),
            MemESP::readbyte(pci+7),

            MemESP::readbyte(pci+8),
            MemESP::readbyte(pci+9),
            MemESP::readbyte(pci+10),
            MemESP::readbyte(pci+11),

            MemESP::readbyte(pci+12),
            MemESP::readbyte(pci+13),
            MemESP::readbyte(pci+14),
            MemESP::readbyte(pci+15)
        );
        VIDEO::vga.print(buf);
    }

    fabgl::VirtualKeyItem Nextkey;
    auto Kbd = ESPectrum::PS2Controller.keyboard();
    bool alt = false;
    while (1) {
        sleep_ms(5);
        if (Kbd->virtualKeyAvailable()) {
            Kbd->getNextVirtualKey(&Nextkey);
            if (Nextkey.vk == fabgl::VK_LALT || Nextkey.vk == fabgl::VK_RALT) {
                alt = Nextkey.down;
            }
            if (!Nextkey.down) continue;
            if (Nextkey.vk == fabgl::VK_ESCAPE) {
                break;
            }
            if (alt && Nextkey.vk == fabgl::VK_F2 && FileUtils::fsMount) {
                // Remount SD if needed (card may have been swapped)
                if (!FileUtils::checkSDCard()) FileUtils::remountSD();
                // Save memory dump to file
                uint32_t addr_from = addressDialog(dump_pc, "Dump from");
                if (addr_from > 0xFFFF) goto c;
                uint32_t addr_to = addressDialog((addr_from + 0xFF) & 0xFFFF, "Dump to");
                if (addr_to > 0xFFFF) goto c;
                saveDumpToFile((uint16_t)addr_from, (uint16_t)addr_to);
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_KP_MINUS || Nextkey.vk == fabgl::VK_UP) {
                dump_pc -= 16;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_KP_PLUS || Nextkey.vk == fabgl::VK_DOWN) {
                dump_pc += 16;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_0) {
                dump_pc = 0;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_PAGEUP) {
                dump_pc -= 20 * 16;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_PAGEDOWN) {
                dump_pc += 20 * 16;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F8) {
                uint32_t address = addressDialog(dump_pc, Config::lang ? "Saltar a" : "Jump to");
                if (address <= 0xFFFF) {
                    dump_pc = address;
                }
                goto c;
            }
        }
    }
    VIDEO::SaveRect.restore_last();
}

static uint32_t memSearchResultAddr = 0x10000; // >0xFFFF = no result
static string memSearchHex;
static uint16_t memSearchLastFound = 0;
static uint32_t memDoSearch(uint16_t startAddr);

// Disassemble instruction at addr into out buffer (max maxlen chars)
static void disasmAt(uint16_t addr, char* out, int maxlen) {
    uint8_t b = MemESP::readbyte(addr);
    std::string m;
    int off = 1; // offset to first operand byte
    bool isIY = false;
    if (b == 0xDD || b == 0xFD) {
        isIY = (b == 0xFD);
        uint8_t b1 = MemESP::readbyte(addr + 1);
        if (b1 == 0xCB) {
            m = mnemCB[MemESP::readbyte(addr + 3)];
            auto sp = m.find(" ");
            if (sp != std::string::npos) m.replace(sp, 1, isIY ? " (IY+d)," : " (IX+d),");
            off = 2;
        } else {
            m = mnemIX(b1);
            if (isIY) { auto p = m.find("IX"); if (p != std::string::npos) m.replace(p, 2, "IY"); }
            off = 2;
        }
    } else if (b == 0xED) {
        m = mnemED(MemESP::readbyte(addr + 1));
        off = 2;
    } else if (b == 0xCB) {
        m = mnemCB[MemESP::readbyte(addr + 1)];
        off = 2;
    } else {
        m = mnem[b];
    }
    // Substitute operands
    auto pnn = m.find("nn");
    if (pnn != std::string::npos) {
        uint16_t val = MemESP::readbyte(addr + off) | (MemESP::readbyte(addr + off + 1) << 8);
        char tmp[5]; snprintf(tmp, 5, "%04X", val);
        m.replace(pnn, 2, tmp);
    } else {
        auto pd = m.find("+d");
        if (pd != std::string::npos) {
            int8_t disp = (int8_t)MemESP::readbyte(addr + off);
            char tmp[8]; snprintf(tmp, 8, "%+d", disp);
            m.replace(pd, 2, tmp);
        } else {
            auto pn = m.find("n");
            if (pn != std::string::npos) {
                uint8_t val = MemESP::readbyte(addr + off);
                char tmp[3]; snprintf(tmp, 3, "%02X", val);
                m.replace(pn, 1, tmp);
            } else {
                auto pe = m.find("d");
                if (pe != std::string::npos) {
                    int8_t disp = (int8_t)MemESP::readbyte(addr + off);
                    uint16_t target = addr + 2 + disp; // JR/DJNZ are always 2 bytes
                    char tmp[5]; snprintf(tmp, 5, "%04X", target);
                    m.replace(pe, 1, tmp);
                }
            }
        }
    }
    strncpy(out, m.c_str(), maxlen - 1);
    out[maxlen - 1] = 0;
}

static int instrLen(uint16_t addr) {
    uint8_t b = MemESP::readbyte(addr);
    if (b == 0xDD || b == 0xFD) {
        uint8_t b1 = MemESP::readbyte(addr + 1);
        if (b1 == 0xCB) return 4;
        const char* m = mnemIX(b1);
        if (strstr(m, "nn")) return 4;
        bool has_d = strstr(m, "d") != nullptr;
        bool has_n = strstr(m, "n") != nullptr;
        if (has_d && has_n) return 4;
        if (has_d || has_n) return 3;
        return 2;
    }
    if (b == 0xED) {
        const char* m = mnemED(MemESP::readbyte(addr + 1));
        if (strstr(m, "nn")) return 4;
        return 2;
    }
    if (b == 0xCB) return 2;
    const char* m = mnem[b];
    if (strstr(m, "nn")) return 3;
    if (strstr(m, "n") || strstr(m, "d")) return 2;
    return 1;
}

void OSD::osdDebug(uint16_t gotoAddr) {
    const unsigned short h = OSD_FONT_H * 26;
    const unsigned short y = scrAlignCenterY(h);
    const unsigned short w = OSD_FONT_W * 50;
    const unsigned short x = scrAlignCenterX(w);

    VIDEO::SaveRect.save(x - 1, y - 1, w + 2, h + 2);
    char buf[40];
    int ii = 3;
    int cursor_row = 3; // cursor starts at PC line
    uint32_t t1 = 0;
    uint32_t t2 = 0;
    uint32_t T1 = 0;
    uint32_t T2 = 0;
    int activeSection = 0; // 0=Code, 1=Pages, 2=Memory, 3=Registers
    int regCursorRow = 0;
    bool regAltSet = false;
    int memCursorRow = 0;
    int memCursorCol = 0; // 0-3 = byte index
    uint16_t memViewAddr = Z80::getRegPC();
    int xi_right = x + 32 * OSD_FONT_W;
    int pagesCursorRow = 0; // 0=PAGE0, 1=PAGE3, 2=VIDEO, 3=PAGING LOCK
    bool memAsciiMode = false;
    bool gotoApplied = false;
    bool redrawCode = true;
    bool redrawMem = true;
    bool redrawRight = true;
    bool redrawTitle = true;

c:
    sleep_ms(5);
    // Set font
    VIDEO::vga.setFont(Font6x8);

    if (redrawTitle) {
    // Border + title bar; full content bg only on first draw
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));

    // Title with section indicator
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    const char* secNames[] = {"Code", "Memory", "Pages", "Regs"};
    snprintf(buf, 32, "Debug [%s]", secNames[activeSection]);
    VIDEO::vga.print(buf);

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }
    } // redrawTitle

    uint16_t pc = Z80::getRegPC();
    // Apply gotoAddr: set ii so that gotoAddr appears on current cursor_row
    if (gotoAddr != 0xFFFF && !gotoApplied) {
        ii = (int16_t)(pc - gotoAddr) + cursor_row;
        gotoApplied = true;
    }
    const int CODE_LINES = 18;
    const int MEM_LINES = 4;
    int i = 0;
    int xi = x + 1;
    uint16_t line_addr[CODE_LINES];
    int regStartRow = 0;
  if (redrawCode) {
    uint16_t pci = pc - ii; // starting address for line 0
    std::string mem;
    for (; i < CODE_LINES; ++i) {
        line_addr[i] = pci;
        int len = instrLen(pci);
        uint8_t bytes[4];
        for (int b = 0; b < len && b < 4; b++)
            bytes[b] = MemESP::readbyte(pci + b);
        int yi = y + (i + 1) * OSD_FONT_H + 2;
        // Highlight: red ink for PC, cyan bg for cursor
        bool isCursor = (i == cursor_row && activeSection == 0);
        if (pci == pc && isCursor)
            VIDEO::vga.setTextColor(zxColor(2, 1), zxColor(5, 0));
        else if (pci == pc)
            VIDEO::vga.setTextColor(zxColor(2, 1), zxColor(7, 1));
        else if (isCursor)
            VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
        else
            VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
        if (isCursor)
            VIDEO::vga.fillRect(xi, yi, xi_right - xi - OSD_FONT_W, OSD_FONT_H, zxColor(5, 0));
        else
            VIDEO::vga.fillRect(xi, yi, xi_right - xi - OSD_FONT_W, OSD_FONT_H, zxColor(7, 1));
        VIDEO::vga.setCursor(xi, yi);

        // Build hex bytes string (up to 4 bytes, 8 chars + trailing space)
        char hexbytes[10];
        int hpos = 0;
        for (int b = 0; b < len && b < 4; b++) {
            snprintf(hexbytes + hpos, 3, "%02X", bytes[b]);
            hpos += 2;
        }
        while (hpos < 9) hexbytes[hpos++] = ' ';
        hexbytes[9] = 0;

        // Get mnemonic with resolved operands
        uint8_t bi = bytes[0];
        bool isED = (bi == 0xED);
        bool isCB = (bi == 0xCB);
        bool isIX = (bi == 0xDD);
        bool isIY = (bi == 0xFD);
        if (isIX || isIY) {
            uint8_t b1 = bytes[1];
            if (b1 == 0xCB) {
                mem = mnemCB[bytes[3]];
                auto sp = mem.find(" ");
                if (sp != string::npos) mem.replace(sp, 1, isIY ? " (IY+d)," : " (IX+d),");
            } else {
                mem = mnemIX(b1);
            }
            auto pos = mem.find(",(HL)");
            if (pos != string::npos) mem.replace(pos, 5, " ");
            if (isIY) {
                auto ixp = mem.find("IX");
                if (ixp != string::npos) mem.replace(ixp, 2, "IY");
            }
        } else if (isCB) {
            mem = mnemCB[bytes[1]];
        } else if (isED) {
            mem = mnemED(bytes[1]);
        } else {
            mem = mnem[bi];
        }
        // Replace operand placeholders
        const char* memc = mem.c_str();
        if (strstr(memc, "nn") != 0) {
            int off = (isED || isIX || isIY) ? 2 : 1;
            uint16_t addr = MemESP::readbyte(pci + off) | (MemESP::readbyte(pci + off + 1) << 8);
            char tmp[5]; snprintf(tmp, sizeof(tmp), "%04X", addr);
            auto p = mem.find("nn");
            if (p != string::npos) mem.replace(p, 2, tmp);
        } else if (strstr(memc, "n") != 0) {
            int off = (isED || isIX || isIY) ? 2 : 1;
            uint8_t val = MemESP::readbyte(pci + off);
            char tmp[3]; snprintf(tmp, sizeof(tmp), "%02X", val);
            auto p = mem.find("n");
            if (p != string::npos) mem.replace(p, 1, tmp);
        } else if (strstr(memc, "d") != 0) {
            bool ixiy = isIX || isIY;
            int off = ixiy ? 2 : 1;
            int8_t disp = (int8_t)MemESP::readbyte(pci + off);
            char tmp[8];
            if (ixiy) {
                snprintf(tmp, sizeof(tmp), "%+d", disp);
                auto p = mem.find("+d");
                if (p != string::npos) mem.replace(p, 2, tmp);
            } else {
                uint16_t target = pci + 2 + disp;
                snprintf(tmp, sizeof(tmp), "%04X", target);
                auto p = mem.find("d");
                if (p != string::npos) mem.replace(p, 1, tmp);
            }
        }
        if (mem.length() > 15) mem = mem.substr(0, 15);
        snprintf(buf, 40, "%c%04X %s%-15s", pci == pc ? '*' : ' ', pci, hexbytes, mem.c_str());
        VIDEO::vga.print(buf);
        if (Config::numPcBP > 0 && Config::hasBreakPoint(pci, Config::BP_PC)) {
            VIDEO::vga.circle(xi+3, yi+3, 3, zxColor(2, 0));
        }
        pci += len;
    }

  } // redrawCode

  if (redrawMem) {
    // === MEMORY DUMP (bottom of left panel, 4 rows) ===
    int memBytesPerRow = memAsciiMode ? 20 : 8;
    {
        int memStartRow = CODE_LINES + 2;
        int sy = y + (CODE_LINES + 2) * OSD_FONT_H + 2;
        VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1));
        VIDEO::vga.setCursor(x + 1, sy);
        VIDEO::vga.print(memAsciiMode ? "-Memory (ASCII)---------------" : "-Memory (HEX)-----------------");
        int xi_mem = x + 1 + OSD_FONT_W;
        for (int row = 0; row < MEM_LINES; row++) {
            int yi = y + (memStartRow + 1 + row) * OSD_FONT_H + 2;
            uint16_t addr = (memViewAddr + row * memBytesPerRow) & 0xFFFF;
            bool isRow = (row == memCursorRow && activeSection == 1);
            if (isRow)
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
            else
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            VIDEO::vga.setCursor(xi_mem, yi);
            snprintf(buf, 32, "%04X ", addr);
            VIDEO::vga.print(buf);
            if (memAsciiMode) {
                // ASCII: 20 chars per row (4+1+20 = 25 cols)
                for (int col = 0; col < memBytesPerRow; col++) {
                    uint8_t val = MemESP::readbyte((addr + col) & 0xFFFF);
                    if (isRow && col == memCursorCol)
                        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(2, 0));
                    else if (isRow)
                        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
                    else
                        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                    char ch = (val >= 32 && val < 127) ? val : '.';
                    char s[2] = {ch, 0};
                    VIDEO::vga.print(s);
                }
                // Clear rest
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                VIDEO::vga.print("  ");
            } else {
                // HEX: 8 bytes per row, format: BBBB BBBB BBBB BBBB
                // First pass: build full string, print with row color
                uint8_t vals[8];
                char line[24];
                for (int col = 0; col < 8; col++)
                    vals[col] = MemESP::readbyte((addr + col) & 0xFFFF);
                snprintf(line, 24, "%02X%02X %02X%02X %02X%02X %02X%02X",
                    vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7]);
                // Print char by char, highlighting cursor byte
                VIDEO::vga.setCursor(xi_mem + 5 * OSD_FONT_W, yi);
                int charIdx = 0;
                for (int col = 0; col < 8; col++) {
                    if (isRow && col == memCursorCol)
                        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(2, 0));
                    else if (isRow)
                        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
                    else
                        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                    // Print 2 hex chars
                    char s[3] = {line[charIdx], line[charIdx+1], 0};
                    VIDEO::vga.print(s);
                    charIdx += 2;
                    // Print space separator after every 2 bytes
                    if (col % 2 == 1 && col < 7) {
                        if (isRow)
                            VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
                        else
                            VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                        VIDEO::vga.print(" ");
                        charIdx++; // skip space in line
                    }
                }
                // Clear trailing
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                VIDEO::vga.print("  ");
            }
        }
    }
  } // redrawMem

  if (redrawRight) {
    // === RIGHT PANEL ===
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    i = 0;
    xi = xi_right;
    // regStartRow set for Enter handler

    // --- PAGES header (row 0) + 4 data rows (1-4) ---
    VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("-Pages-----------");
    {
        char pb0[20], pb1[20], pb2[20], pb3[20];
        if (MemESP::ramCurrent[0] < (uint8_t*)0x11000000)
            snprintf(pb0, 20, "PAGE0 -> ROM#%d", MemESP::romInUse);
        else if (MemESP::newSRAM)
            snprintf(pb0, 20, "PAGE0 -> SRAM#%d", MemESP::romLatch);
        else
            snprintf(pb0, 20, "PAGE0 -> RAM#0");
        snprintf(pb1, 20, "PAGE3 -> RAM#%d", MemESP::bankLatch);
        snprintf(pb2, 20, "VIDEO -> RAM#%d", MemESP::videoLatch ? 7 : 5);
        snprintf(pb3, 20, "LOCK %s", MemESP::pagingLock ? "true" : "false");
        const char* pageLabels[] = { pb0, pb1, pb2, pb3 };
        for (int r = 0; r < 4; r++) {
            int yi = y + (i + 1) * OSD_FONT_H + 2;
            bool isCur = (r == pagesCursorRow && activeSection == 2);
            if (isCur)
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
            else
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            VIDEO::vga.setCursor(xi, yi);
            snprintf(buf, 32, "%-17s", pageLabels[r]);
            buf[17] = 0;
            VIDEO::vga.print(buf);
            i++;
        }
    }

    // --- REGS header --- centered between Pages (end row 4) and Flags (row CODE_LINES)
    {
        int regsNeeded = 12; // 1 header + 4 paired + 4 single + IR + Tstates + BP
        int available = CODE_LINES - i; // rows from current i to CODE_LINES
        int pad = (available - regsNeeded) / 2 + 1;
        if (pad > 0) i += pad;
    }
    VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("-Regs------------");
    regStartRow = i; // first data row index
    {
        // Paired registers: show main and alt together
        // Format: "AF 5F88 AF'C011" = 15 chars, fits in 18
        struct PairedReg {
            const char* name; const char* altName;
            uint16_t (*get)(); uint16_t (*getAlt)();
        };
        PairedReg paired[] = {
            {"AF", "AF'", Z80::getRegAF, Z80::getRegAFx},
            {"BC", "BC'", Z80::getRegBC, Z80::getRegBCx},
            {"HL", "HL'", Z80::getRegHL, Z80::getRegHLx},
            {"DE", "DE'", Z80::getRegDE, Z80::getRegDEx},
        };
        for (int r = 0; r < 4; r++) {
            int yi = y + (i + 1) * OSD_FONT_H + 2;
            bool isCur = (regCursorRow == r && activeSection == 3);
            VIDEO::vga.setCursor(xi, yi);
            // Main part
            if (isCur && !regAltSet)
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
            else
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            snprintf(buf, 10, "%-2s %04X ", paired[r].name, paired[r].get());
            VIDEO::vga.print(buf);
            // Alt part
            if (isCur && regAltSet)
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
            else
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            snprintf(buf, 10, "%-3s %04X ", paired[r].altName, paired[r].getAlt());
            VIDEO::vga.print(buf);
            i++;
        }

        // Single registers: IX, IY, SP, PC
        struct SingleReg {
            const char* name;
            uint16_t (*get)();
        };
        SingleReg singles[] = {
            {"IX", Z80::getRegIX},
            {"IY", Z80::getRegIY},
            {"SP", Z80::getRegSP},
            {"PC", (uint16_t(*)())Z80::getRegPC},
        };
        for (int r = 0; r < 4; r++) {
            int yi = y + (i + 1) * OSD_FONT_H + 2;
            int regIdx = 4 + r;
            bool isCur = (regCursorRow == regIdx && activeSection == 3);
            if (isCur)
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
            else
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            VIDEO::vga.setCursor(xi, yi);
            snprintf(buf, 32, "%-2s %04X         ", singles[r].name, singles[r].get());
            buf[17] = 0;
            VIDEO::vga.print(buf);
            i++;
        }

        // I, R, IM row (regCursorRow == 8)
        {
            int yi = y + (i + 1) * OSD_FONT_H + 2;
            bool isCur = (regCursorRow == 8 && activeSection == 3);
            if (isCur)
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(5, 0));
            else
                VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            VIDEO::vga.setCursor(xi, yi);
            snprintf(buf, 32, "IR %02X%02X IM %d    ", Z80::getRegI(), Z80::getRegR(), Z80::getIntMode());
            buf[17] = 0;
            VIDEO::vga.print(buf);
            i++;
        }

        // T-states and BP
        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
        VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
        snprintf(buf, 32, "%dT %dus         ", T2 - T1, t2 - t1);
        buf[17] = 0;
        VIDEO::vga.print(buf);

        if (Config::numBreakPoints > 0) {
            VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
            snprintf(buf, 32, "BP(s):%d          ", Config::numBreakPoints);
            buf[17] = 0;
            VIDEO::vga.print(buf);
        } else ++i;
    }

    // --- FLAGS header (aligned with Memory header = row CODE_LINES+1) ---
    i = CODE_LINES + 1; // force alignment with Memory separator
    VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("-Flags-----------");

    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    {
        uint8_t f = Z80::getRegAF() & 0xFF;
        uint8_t fx = Z80::getRegAFx() & 0xFF;
        snprintf(buf, 32, "%c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c ",
           BNc(f,7), BNc(f,6), BNc(f,5), BNc(f,4), BNc(f,3), BNc(f,2), BNc(f,1), BNc(f,0),
           BNc(fx,7), BNc(fx,6), BNc(fx,5), BNc(fx,4), BNc(fx,3), BNc(fx,2), BNc(fx,1), BNc(fx,0));
        buf[17] = 0;
        VIDEO::vga.print(buf);
    }
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("SZ-H-PNC SZ-H-PNC");

    ++i;
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("F1 - Debug help  ");
  } // redrawRight

    // Reset redraw flags (default: redraw all on next goto c)
    redrawCode = true;
    redrawMem = true;
    redrawRight = true;
    redrawTitle = false; // title/border only on first draw

    // Wait for a key
    fabgl::VirtualKeyItem Nextkey;
    auto Kbd = ESPectrum::PS2Controller.keyboard();
    bool alt = false;
    while (1) {
        sleep_ms(5);
        if (Kbd->virtualKeyAvailable()) {
            Kbd->getNextVirtualKey(&Nextkey);
            if (Config::joystick == JOY_KEMPSTON) {
                Ports::port[Config::kempstonPort] = 0;
                for (int i = fabgl::VK_JOY_RIGHT; i <= fabgl::VK_JOY_C; i++) {
                    if (Kbd->isVKDown((fabgl::VirtualKey) i)) {
                        bitWrite(Ports::port[Config::kempstonPort], i - fabgl::VK_JOY_RIGHT, 1);
                    }
                }
            }

            if (Nextkey.vk == fabgl::VK_LALT || Nextkey.vk == fabgl::VK_RALT) {
                alt = Nextkey.down;
            }

            if (!Nextkey.down) continue;

            if (Nextkey.vk == fabgl::VK_ESCAPE) {
                // Remount SD if card was removed and reinserted during debug session
                if (!FileUtils::checkSDCard()) {
                    FileUtils::remountSD();
                }
                break;
            } else
            if (Nextkey.vk == fabgl::VK_F7) {
                if (alt) {
                    uint16_t bpAddr = BPListDialog();
                    if (bpAddr != 0xFFFF) { gotoAddr = bpAddr; gotoApplied = false; }
                } else {
                    BPDialog();
                }
                redrawTitle = true;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F8) {
                jumpToDialog();
                redrawTitle = true;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F9 && alt) {
                // Fullscreen debug: show Spectrum screen, step with Space/ALT+Space, ESC to return
                VIDEO::SaveRect.restore_last();
                bool fs_alt = false;
                while (1) {
                    sleep_ms(5);
                    if (Kbd->virtualKeyAvailable()) {
                        Kbd->getNextVirtualKey(&Nextkey);
                        if (Nextkey.vk == fabgl::VK_LALT || Nextkey.vk == fabgl::VK_RALT) {
                            fs_alt = Nextkey.down;
                            continue;
                        }
                        if (!Nextkey.down) continue;
                        if (Nextkey.vk == fabgl::VK_ESCAPE) break;
                        if (Nextkey.vk == fabgl::VK_SPACE) {
                            // Step (or step over with ALT)
                            int si = 0;
                            T1 = CPU::tstates;
                            t1 = time_us_32();
                            uint16_t pcs = Z80::getRegPC();
                            pc = pcs;
                            while (si++ < 64*1024 &&
                                (pc == Z80::getRegPC() ||
                                 (fs_alt && pc + 3 != Z80::getRegPC()))
                            ) {
                                CPU::step();
                            }
                            t2 = time_us_32();
                            T2 = CPU::tstates;
                            pc = Z80::getRegPC();
                            ii = 3;
                            // Redraw Spectrum screen after step
                            CPU::loop();
                        }
                    }
                }
                alt = false;
                VIDEO::SaveRect.save(x - 1, y - 1, w + 2, h + 2);
                redrawTitle = true;
                goto c;
            } else
            if (FileUtils::fsMount && Nextkey.vk == fabgl::VK_F11) {
                // Persist Load
                while (1) {
                    menu_footer = Config::lang ? "F3: Cargar  F6: Renombrar  F8: Borrar" : "F3: Load  F6: Rename  F8: Remove";
                uint8_t opt2 = menuRun(buildSlotMenu(MENU_PERSIST_LOAD[Config::lang], 40));
                    if (opt2) {
                        if (menu_del_pressed) {
                            persistDeleteConfirm(opt2);
                            menu_curopt = opt2;
                            continue;
                        }
                        persistLoad(opt2);
                    }
                    break;
                }
                goto c;
            }
            else if (FileUtils::fsMount && Nextkey.vk == fabgl::VK_F12) {
                // Persist Save
                while (1) {
                    menu_footer = Config::lang ? "F6: Renombrar  F8: Borrar" : "F6: Rename  F8: Remove";
                uint8_t opt2 = menuRun(buildSlotMenu(MENU_PERSIST_SAVE[Config::lang], 40));
                    if (opt2) {
                        if (menu_del_pressed) {
                            persistDeleteConfirm(opt2);
                            menu_curopt = opt2;
                            continue;
                        }
                        if (persistSave(opt2, opt2)) break;
                        menu_curopt = opt2;
                    } else break;
                }
                goto c;
            }
            if (Nextkey.vk == fabgl::VK_TAB) {
                activeSection = (activeSection + 1) % 4;
                redrawTitle = true;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_UP) {
                if (activeSection == 0) {
                    if (cursor_row > 0) cursor_row--;
                    else {
                        uint16_t start = pc - ii;
                        int step = 1;
                        for (int k = 1; k <= 4; ++k) {
                            if (instrLen(start - k) == k) { step = k; break; }
                        }
                        ii += step;
                    }
                } else if (activeSection == 2) {
                    if (pagesCursorRow > 0) pagesCursorRow--;
                    redrawCode = false; redrawMem = false;
                } else if (activeSection == 1) {
                    if (memCursorRow > 0) memCursorRow--;
                    else memViewAddr = (memViewAddr - (memAsciiMode ? 20 : 8)) & 0xFFFF;
                    redrawCode = false; redrawRight = false;
                } else if (activeSection == 3) {
                    if (regCursorRow > 0) regCursorRow--;
                    redrawCode = false; redrawMem = false;
                }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_DOWN) {
                if (activeSection == 0) {
                    if (cursor_row < CODE_LINES - 1) cursor_row++;
                    else ii -= instrLen(pc - ii);
                } else if (activeSection == 2) {
                    if (pagesCursorRow < 3) pagesCursorRow++;
                    redrawCode = false; redrawMem = false;
                } else if (activeSection == 1) {
                    if (memCursorRow < MEM_LINES - 1) memCursorRow++;
                    else memViewAddr = (memViewAddr + (memAsciiMode ? 20 : 8)) & 0xFFFF;
                    redrawCode = false; redrawRight = false;
                } else if (activeSection == 3) {
                    if (regCursorRow < 8) regCursorRow++;
                    redrawCode = false; redrawMem = false;
                }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_LEFT) {
                if (activeSection == 2) {
                    // Pages: cycle value left — affects code view (ROM/bank switch)
                    if (pagesCursorRow == 0) { // PAGE0: ROM
                        if (MemESP::romInUse > 0) MemESP::romInUse--;
                        MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse].direct();
                    } else if (pagesCursorRow == 1) { // PAGE3: RAM bank
                        MemESP::bankLatch = (MemESP::bankLatch - 1) & 7;
                        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(MemESP::bankLatch);
                    } else if (pagesCursorRow == 2) { // VIDEO
                        MemESP::videoLatch = MemESP::videoLatch ? 0 : 1;
                    } else if (pagesCursorRow == 3) { // PAGING LOCK
                        MemESP::pagingLock = !MemESP::pagingLock;
                    }
                } else if (activeSection == 1) {
                    if (memCursorCol > 0) memCursorCol--;
                    redrawCode = false; redrawRight = false;
                } else if (activeSection == 3) {
                    regAltSet = !regAltSet;
                    redrawCode = false; redrawMem = false;
                }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_RIGHT) {
                if (activeSection == 2) {
                    // Pages: cycle value right — affects code view (ROM/bank switch)
                    if (pagesCursorRow == 0) {
                        if (MemESP::romInUse < 3) MemESP::romInUse++;
                        MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse].direct();
                    } else if (pagesCursorRow == 1) {
                        MemESP::bankLatch = (MemESP::bankLatch + 1) & 7;
                        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(MemESP::bankLatch);
                    } else if (pagesCursorRow == 2) {
                        MemESP::videoLatch = MemESP::videoLatch ? 0 : 1;
                    } else if (pagesCursorRow == 3) {
                        MemESP::pagingLock = !MemESP::pagingLock;
                    }
                } else if (activeSection == 1) {
                    if (memCursorCol < (memAsciiMode ? 19 : 7)) memCursorCol++;
                    redrawCode = false; redrawRight = false;
                } else if (activeSection == 3) {
                    regAltSet = !regAltSet;
                    redrawCode = false; redrawMem = false;
                }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_0 && activeSection == 0) {
                ii = 3;
                cursor_row = 3;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_PAGEUP) {
                if (activeSection == 0) ii += CODE_LINES;
                else if (activeSection == 1) { memViewAddr = (memViewAddr - MEM_LINES * (memAsciiMode ? 20 : 8)) & 0xFFFF; redrawCode = false; redrawRight = false; }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_PAGEDOWN) {
                if (activeSection == 0) ii -= CODE_LINES;
                else if (activeSection == 1) { memViewAddr = (memViewAddr + MEM_LINES * (memAsciiMode ? 20 : 8)) & 0xFFFF; redrawCode = false; redrawRight = false; }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_T && alt) {
                memAsciiMode = !memAsciiMode;
                memCursorCol = 0; // reset col since byte count per row changes
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F5) {
                uint16_t bp_addr = line_addr[cursor_row];
                if (Config::hasBreakPoint(bp_addr)) {
                    Config::removeBreakPoint(bp_addr);
                } else {
                    Config::addBreakPoint(bp_addr);
                }
                Config::save();
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F2) {
                if (alt && FileUtils::fsMount) {
                    // Remount SD if needed (card may have been swapped)
                    if (!FileUtils::checkSDCard()) FileUtils::remountSD();
                    uint16_t from_addr = line_addr[cursor_row];
                    uint16_t to_addr = (from_addr + 0xFF) & 0xFFFF;
                    if (dumpRangeDialog(from_addr, to_addr)) {
                        saveDumpToFile(from_addr, to_addr);
                        osdCenteredMsg("Dump saved", LEVEL_INFO, 1000);
                    }
                } else {
                    osdDump();
                }
                redrawTitle = true;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F1 && alt) {
                // ALT+F1: Search memory for hex byte sequence
                memSearchDialog();
                if (memSearchResultAddr <= 0xFFFF) {
                    ii = pc - (uint16_t)memSearchResultAddr + cursor_row;
                }
                redrawTitle = true;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F3 && !alt) {
                // F3: Search next (continues from last ALT+F1 search)
                if (memSearchHex.length() >= 2) {
                    uint32_t result = memDoSearch((memSearchLastFound + 1) & 0xFFFF);
                    if (result <= 0xFFFF) {
                        ii = pc - (uint16_t)result + cursor_row;
                    }
                }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F1) {
                drawOSD(true);
                osdAt(2, 0);
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                VIDEO::vga.print(OSD_DBG_HELP_EN);
                while (1) {
                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        if (ESPectrum::readKbd(&Nextkey)) {
                            if(!Nextkey.down) continue;
                            if (is_enter(Nextkey.vk) || is_back(Nextkey.vk)) break;
                        }
                    }
                    sleep_ms(5);
                }
                redrawTitle = true;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_RETURN || Nextkey.vk == fabgl::VK_KP_ENTER) {
                // Inline hex editing — shared helper lambda
                auto inlineHexEdit = [&](int ex, int ey, char* hexbuf, int ndigits) -> bool {
                    int hpos = 0;
                    while (1) {
                        for (int p = 0; p < ndigits; p++) {
                            VIDEO::vga.setTextColor(zxColor(7, 1), p == hpos ? zxColor(1, 1) : zxColor(2, 0));
                            VIDEO::vga.setCursor(ex + p * OSD_FONT_W, ey);
                            char ch[2] = {hexbuf[p], 0};
                            VIDEO::vga.print(ch);
                        }
                        while (!Kbd->virtualKeyAvailable()) sleep_ms(5);
                        fabgl::VirtualKeyItem ek;
                        Kbd->getNextVirtualKey(&ek);
                        if (!ek.down) continue;
                        if (ek.vk >= fabgl::VK_0 && ek.vk <= fabgl::VK_9) {
                            hexbuf[hpos] = '0' + (ek.vk - fabgl::VK_0);
                            if (hpos < ndigits - 1) hpos++;
                        } else if (ek.vk >= fabgl::VK_A && ek.vk <= fabgl::VK_F) {
                            hexbuf[hpos] = 'A' + (ek.vk - fabgl::VK_A);
                            if (hpos < ndigits - 1) hpos++;
                        } else if (ek.vk == fabgl::VK_LEFT) { if (hpos > 0) hpos--; }
                        else if (ek.vk == fabgl::VK_RIGHT) { if (hpos < ndigits - 1) hpos++; }
                        else if (ek.vk == fabgl::VK_BACKSPACE) { if (hpos > 0) { hpos--; hexbuf[hpos] = '0'; } }
                        else if (ek.vk == fabgl::VK_RETURN || ek.vk == fabgl::VK_KP_ENTER) return true;
                        else if (ek.vk == fabgl::VK_ESCAPE) return false;
                    }
                };
                auto parseHex = [](const char* h, int n) -> uint16_t {
                    uint16_t v = 0;
                    for (int i = 0; i < n; i++) {
                        v <<= 4;
                        if (h[i] >= '0' && h[i] <= '9') v += h[i] - '0';
                        else v += h[i] - 'A' + 10;
                    }
                    return v;
                };

                if (activeSection == 0) {
                    // Code: inline address edit — jump to entered address
                    char hexbuf[5];
                    snprintf(hexbuf, 5, "%04X", line_addr[cursor_row]);
                    int addr_x = x + 1 + OSD_FONT_W;
                    int addr_y = y + (cursor_row + 1) * OSD_FONT_H + 2;
                    if (inlineHexEdit(addr_x, addr_y, hexbuf, 4)) {
                        gotoAddr = parseHex(hexbuf, 4);
                        gotoApplied = false;
                    }
                } else if (activeSection == 1) {
                    // Memory: inline byte edit (left panel, below code)
                    int bpr = memAsciiMode ? 20 : 8;
                    uint16_t addr = (memViewAddr + memCursorRow * bpr + memCursorCol) & 0xFFFF;
                    char hexbuf[3];
                    snprintf(hexbuf, 3, "%02X", MemESP::readbyte(addr));
                    int xi_mem = x + 1 + OSD_FONT_W;
                    int memStartRow = CODE_LINES + 2;
                    int ccx = memAsciiMode
                        ? (5 + memCursorCol)
                        : (5 + memCursorCol * 2 + (memCursorCol / 2));
                    int ex = xi_mem + ccx * OSD_FONT_W;
                    int ey = y + (memStartRow + 1 + memCursorRow) * OSD_FONT_H + 2;
                    if (inlineHexEdit(ex, ey, hexbuf, 2)) {
                        MemESP::writebyte(addr, (uint8_t)parseHex(hexbuf, 2));
                    }
                } else if (activeSection == 3) {
                    // Registers: inline edit
                    typedef uint16_t (*RegGetter)();
                    typedef void (*RegSetter)(uint16_t);
                    struct RI { RegGetter get; RegSetter set; bool is8bit; };
                    RI mainRegs[] = {
                        {Z80::getRegAF, Z80::setRegAF, false},
                        {Z80::getRegBC, Z80::setRegBC, false},
                        {Z80::getRegHL, Z80::setRegHL, false},
                        {Z80::getRegDE, Z80::setRegDE, false},
                        {Z80::getRegIX, Z80::setRegIX, false},
                        {Z80::getRegIY, Z80::setRegIY, false},
                        {Z80::getRegSP, Z80::setRegSP, false},
                        {(RegGetter)Z80::getRegPC, Z80::setRegPC, false},
                    };
                    RI altRegs[] = {
                        {Z80::getRegAFx, Z80::setRegAFx, false},
                        {Z80::getRegBCx, Z80::setRegBCx, false},
                        {Z80::getRegHLx, Z80::setRegHLx, false},
                        {Z80::getRegDEx, Z80::setRegDEx, false},
                        {Z80::getRegIX, Z80::setRegIX, false},
                        {Z80::getRegIY, Z80::setRegIY, false},
                        {Z80::getRegSP, Z80::setRegSP, false},
                        {(RegGetter)Z80::getRegPC, Z80::setRegPC, false},
                    };
                    if (regCursorRow < 8) {
                        RI& ri = (regAltSet && regCursorRow < 4) ? altRegs[regCursorRow] : mainRegs[regCursorRow];
                        char hexbuf[5];
                        snprintf(hexbuf, 5, "%04X", ri.get());
                        int rrow = regStartRow + regCursorRow;
                        int ey = y + (rrow + 1) * OSD_FONT_H + 2;
                        // For paired regs (0-3): main "%-2s %04X " (val at col 3), alt "%-3s %04X " (val at col 8+4=12)
                        // For singles (4-7): "%-2s %04X" (val at col 3)
                        int ex;
                        if (regCursorRow < 4 && regAltSet)
                            ex = xi_right + (8 + 4) * OSD_FONT_W; // after main(8) + "AF' "(4)
                        else
                            ex = xi_right + 3 * OSD_FONT_W;  // after "AF "(3)
                        if (inlineHexEdit(ex, ey, hexbuf, 4)) {
                            ri.set(parseHex(hexbuf, 4));
                        }
                    }
                    // IR row (regCursorRow == 8): skip for now, complex
                }
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_SPACE) {
                // Step CPU
                int i = 0;
                T1 = CPU::tstates;
                t1 = time_us_32();
                uint16_t pcs = Z80::getRegPC();
                while (i++ < 64*1024 &&
                    (
                        pc == Z80::getRegPC() ||
                        (alt && pc + 3 != Z80::getRegPC()) // CALL nn case
                    )
                ) {
                    CPU::step();
                }
                if (alt && pc + 3 != Z80::getRegPC() && i >= 64*1024) {
                    Config::addBreakPoint(pcs + 3); // CALL nn case - temp BP at return addr
                    break;
                }
                ii -= (int)pc - Z80::getRegPC();
                if (ii > 16) ii = 4;
                if (ii < 0) ii = 4;
                t2 = time_us_32();
                T2 = CPU::tstates;
                redrawTitle = true;
                goto c;
            }
        }
    }
    VIDEO::SaveRect.restore_last();

}

// // Count NL chars inside a string, useful to count menu rows
unsigned short OSD::rowCount(const string& menu) {
    unsigned short count = 0;
    for (unsigned short i = 0; i < menu.length(); i++) {
        if (menu.at(i) == ASCII_NL) {
            count++;
        }
    }
    return count;
}

// // Get a row text
string OSD::rowGet(const string& menu, unsigned short row) {
    unsigned short count = 0;
    unsigned short last = 0;
    for (unsigned short i = 0; i < menu.length(); i++) {
        if (menu.at(i) == ASCII_NL) {
            if (count == row) {
                return menu.substr(last,i - last);
            }
            count++;
            last = i + 1;
        }
    }
    if (count == row && last < menu.length()) {
        return menu.substr(last);
    }
    return "<Unknown menu row>";
}
// inline static uint32_t get_cpu_flash_size(void) {
//     uint8_t rx[4] = {0};
//     get_cpu_flash_jedec_id(rx);
//     return 1u << rx[3];
// }
extern "C" uint8_t linkVGA01;
extern "C" uint8_t link_i2s_code;
extern bool is_i2s_enabled;

extern char __HeapLimit;
extern "C" void *sbrk(intptr_t incr);

size_t getFreeHeap(void) {
    struct mallinfo mi = mallinfo();
    // fordblks = free blocks in free list + sbrk headroom (total really free memory)
    // Add remaining sbrk space that mallinfo doesn't account for
    char *brk = (char *)sbrk(0);
    size_t sbrk_free = (brk < &__HeapLimit) ? (size_t)(&__HeapLimit - brk) : 0;
    return mi.fordblks + sbrk_free;
}

// Upper bound on a single contiguous allocation that will succeed without
// tripping SDK's check_alloc panic. Ignores fordblks (may be fragmented);
// trusts only sbrk headroom, which is always contiguous.
size_t getContiguousHeap(void) {
    char *brk = (char *)sbrk(0);
    return (brk < &__HeapLimit) ? (size_t)(&__HeapLimit - brk) : 0;
}

// Generic read-only text dialog with vertical scroll
void OSD::showTextDialog(const char* title, const char* text) {
    click();

    unsigned short sx = scrAlignCenterX(OSD_W);
    unsigned short sy = scrAlignCenterY(OSD_H);
    VIDEO::SaveRect.save(sx, sy, OSD_W, OSD_H);

    // Parse text into line pointers (zero-copy: index into original text)
    const int MAX_DLGLINES = 64;
    const char* lineStart[MAX_DLGLINES];
    uint8_t lineLen[MAX_DLGLINES];
    int nlines = 0;
    const char* p = text;
    while (*p && nlines < MAX_DLGLINES) {
        lineStart[nlines] = p;
        const char* eol = p;
        while (*eol && *eol != '\n') eol++;
        int len = eol - p;
        lineLen[nlines] = len > 255 ? 255 : len;
        nlines++;
        p = *eol ? eol + 1 : eol;
    }

    const int visCols = osdMaxCols();
    // rows: 0=OSD_TITLE, 1=title, 2=separator, ... last=OSD_BOTTOM → content rows = maxRows-4
    const int visRows = osdMaxRows() - 4;
    int scroll = 0;
    bool needRedraw = true;

    auto drawContent = [&]() {
        drawOSD(true);
        // Title (row 1)
        osdAt(1, 0);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
        char hdr[42];
        snprintf(hdr, sizeof(hdr), " %s", title);
        int hlen = strlen(hdr);
        while (hlen < visCols) hdr[hlen++] = ' ';
        hdr[visCols] = '\0';
        VIDEO::vga.print(hdr);

        // Separator (row 2)
        osdAt(2, 0);
        VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(1, 0));
        char sep[42];
        memset(sep, '-', visCols);
        sep[visCols] = '\0';
        sep[0] = ' ';
        VIDEO::vga.print(sep);

        // Text lines (rows 3..3+visRows-1)
        char row[42];
        for (int r = 0; r < visRows; r++) {
            osdAt(3 + r, 0);
            int li = scroll + r;
            if (li < nlines) {
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                int len = lineLen[li];
                if (len > visCols) len = visCols;
                memcpy(row, lineStart[li], len);
                memset(row + len, ' ', visCols - len);
            } else {
                VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(1, 0));
                memset(row, ' ', visCols);
            }
            row[visCols] = '\0';
            VIDEO::vga.print(row);
        }

        // Scrollbar on right edge
        if (nlines > visRows) {
            int sbx = osdInsideX() + (visCols - 1) * OSD_FONT_W;
            int sby = osdInsideY() + 3 * OSD_FONT_H;
            int barH = visRows * OSD_FONT_H;
            // Track
            VIDEO::vga.fillRect(sbx, sby, OSD_FONT_W, barH, zxColor(7, 0));
            // Thumb
            int thumbH = (visRows * barH) / nlines;
            if (thumbH < 3) thumbH = 3;
            int thumbY = (scroll * barH) / nlines;
            if (thumbY + thumbH > barH) thumbY = barH - thumbH;
            VIDEO::vga.fillRect(sbx + 1, sby + thumbY, OSD_FONT_W - 2, thumbH, zxColor(0, 0));
        }
    };

    fabgl::VirtualKeyItem Nextkey;

    while (1) {
        if (needRedraw) {
            drawContent();
            needRedraw = false;
        }

        while (!ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable())
            sleep_ms(5);

        ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
        if (!Nextkey.down) continue;

        if (is_enter(Nextkey.vk) || is_back(Nextkey.vk)) {
            click();
            break;
        }
        int maxScroll = nlines > visRows ? nlines - visRows : 0;
        if (Nextkey.vk == fabgl::VK_UP) {
            if (scroll > 0) { scroll--; needRedraw = true; }
        } else if (Nextkey.vk == fabgl::VK_DOWN) {
            if (scroll < maxScroll) { scroll++; needRedraw = true; }
        } else if (Nextkey.vk == fabgl::VK_PAGEUP) {
            scroll -= visRows; if (scroll < 0) scroll = 0; needRedraw = true;
        } else if (Nextkey.vk == fabgl::VK_PAGEDOWN) {
            scroll += visRows; if (scroll > maxScroll) scroll = maxScroll; needRedraw = true;
        }
    }

    VIDEO::SaveRect.restore_last();
}

void OSD::HWInfo() {
    char (&hwtext)[OSD_INFO_BUF_SZ] = osd_info_buf;
    int pos = 0;

    uint32_t cpu_hz = clock_get_hz(clk_sys) / MHZ;
    uint32_t free_heap = getFreeHeap();

#if PICO_RP2350
    {
        static const uint16_t vreg_mv[] = {
            550, 600, 650, 700, 750, 800, 850, 900, 950, 1000,
            1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1500,
            1600, 1650, 1700, 1800, 1900, 2000, 2350, 2500, 2650,
            2800, 3000, 3150, 3300
        };
        int vi = vreg_get_voltage();
        int mv = (vi >= 0 && vi < 32) ? vreg_mv[vi] : 0;
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
            " Chip model     : RP2350%s %d MHz\n"
            " Chip cores     : 2\n"
            " Chip VREG      : %d.%02d V\n"
            " Chip RAM       : 520 KB\n"
            " Free RAM       : %d KB\n",
            rp2350a ? "A" : "B", (int)cpu_hz,
            mv / 1000, (mv % 1000) / 10,
            (int)(free_heap / 1024));
    }
#else
    pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
        " Chip model     : RP2040 %d MHz\n"
        " Chip cores     : 2\n"
        " Chip RAM       : 264 KB\n"
        " Free RAM       : %d KB\n",
        (int)cpu_hz, (int)(free_heap / 1024));
#endif

    {
        uint32_t flash_size = (1 << rx[3]);
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
            " Flash size     : %d MB\n"
            " Flash JEDEC ID : %02X-%02X-%02X-%02X\n",
            (int)(flash_size >> 20), rx[0], rx[1], rx[2], rx[3]);
    }

#ifndef MURM2
    {
        uint32_t psram32 = psram_size();
        if (psram32) {
            uint8_t rx8[8];
            psram_id(rx8);
            pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
                " PSRAM size     : %d MB\n"
                " PSRAM MF ID/KGD: %02X/%02X\n"
                " PSRAM EID      : %02X%02X-%02X%02X-%02X%02X\n",
                (int)(psram32 >> 20), rx8[0], rx8[1], rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7]);
        }
    }
#endif
#ifdef BUTTER_PSRAM_GPIO
    if (butter_psram_size()) {
        uint32_t psram32 = butter_psram_size();
        if (psram32)
            pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
                "+PSRAM on GP%02d  : %d MB (QSPI)\n", psram_pin, (int)(psram32 >> 20));
        else
            pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
                " PSRAM on GP%02d  : Not found\n", psram_pin);
    }
#endif

    if (Config::audio_driver == 4)
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " Audio mode     : HDMI\n");
    else if (Config::audio_driver == 3)
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " Audio mode     : AY-3-8910\n");
    else
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " Audio mode     : %s [%02Xh] %s\n",
            (is_i2s_enabled ? "i2s" : "PWM"), link_i2s_code,
            (Config::audio_driver == 0 ? " (auto)" : "(overriden)"));

#ifdef VGA_HDMI
    pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " VGA/HDMI detect: %02Xh\n", linkVGA01);
#endif

    if (!psram_pages)
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " 16K RAM pages  : %d[s%d:b%d:v%d]\n",
            ram_pages + butter_pages + swap_pages, ram_pages, butter_pages, swap_pages);
    else if (!butter_pages)
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " 16K RAM pages  : %d[s%d:p%d:v%d]\n",
            ram_pages + psram_pages + swap_pages, ram_pages, psram_pages, swap_pages);
    else if (!swap_pages)
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " 16K RAM pages  : %d[s%d:b%d:p%d]\n",
            ram_pages + butter_pages + psram_pages, ram_pages, butter_pages, psram_pages);
    else
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " 16K RAM pages  : %d[s%d:b%d:p%d:v%d]\n",
            ram_pages + butter_pages + psram_pages + swap_pages, ram_pages, butter_pages, psram_pages, swap_pages);

#if !PICO_RP2040
    if (DivMMC::enabled) {
        const char* mode_names[] = { "OFF", "DivMMC", "DivIDE", "DivSD" };
        const char* mem_type = DivMMC::use_psram ? "PSRAM" : "swap";
        pos += snprintf(hwtext + pos, sizeof(hwtext) - pos, " %-15s: 128K+8K [%s]\n",
            mode_names[Config::esxdos], mem_type);
    }
#endif

    pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
        "\n"
        " Built at %s %s\n"
        " branch '%s'\n"
        " commit [%s]\n"
        " %s\n",
        __DATE__, __TIME__, PICO_GIT_BRANCH, PICO_GIT_COMMIT, PICO_BUILD_NAME);

    showTextDialog("Hardware info", hwtext);
}

void OSD::ChipInfo() {
    char (&buf)[OSD_INFO_BUF_SZ] = osd_info_buf;
    int pos = 0;
    uint32_t cpu_hz = clock_get_hz(clk_sys) / MHZ;
    uint32_t free_heap = getFreeHeap();

#if PICO_RP2350
    {
        static const uint16_t vreg_mv[] = {
            550, 600, 650, 700, 750, 800, 850, 900, 950, 1000,
            1050, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1500,
            1600, 1650, 1700, 1800, 1900, 2000, 2350, 2500, 2650,
            2800, 3000, 3150, 3300
        };
        int vi = vreg_get_voltage();
        int mv = (vi >= 0 && vi < 32) ? vreg_mv[vi] : 0;
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Chip model     : RP2350%s\n"
            " Chip cores     : 2\n"
            " CPU frequency  : %d MHz\n"
            " VREG voltage   : %d.%02d V\n"
            " Chip RAM       : 520 KB\n"
            " Free RAM       : %d KB\n",
            rp2350a ? "A" : "B",
            (int)cpu_hz,
            mv / 1000, (mv % 1000) / 10,
            (int)(free_heap / 1024));
    }
#else
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        " Chip model     : RP2040\n"
        " Chip cores     : 2\n"
        " CPU frequency  : %d MHz\n"
        " Chip RAM       : 264 KB\n"
        " Free RAM       : %d KB\n",
        (int)cpu_hz, (int)(free_heap / 1024));
#endif

    {
        uint32_t flash_size = (1 << rx[3]);
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Flash size     : %d MB\n"
            " Flash JEDEC ID : %02X-%02X-%02X-%02X\n",
            (int)(flash_size >> 20), rx[0], rx[1], rx[2], rx[3]);
    }

#ifndef MURM2
    {
        uint32_t psram32 = psram_size();
        if (psram32) {
            uint8_t rx8[8];
            psram_id(rx8);
            size_t psram_used = (size_t)psram_pages * MEM_PG_SZ;
            size_t psram_free = psram32 > psram_used ? psram32 - psram_used : 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " PSRAM size     : %d MB\n"
                " PSRAM MF ID/KGD: %02X/%02X\n"
                " PSRAM EID      : %02X%02X-%02X%02X-%02X%02X\n"
                " Free PSRAM     : %d KB\n",
                (int)(psram32 >> 20), rx8[0], rx8[1], rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7],
                (int)(psram_free / 1024));
        }
    }
#endif
#ifdef BUTTER_PSRAM_GPIO
    {
        uint32_t psram32 = butter_psram_size();
        if (psram32) {
            size_t butter_used = (size_t)butter_pages * MEM_PG_SZ;
            size_t butter_free = psram32 > butter_used ? psram32 - butter_used : 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "+PSRAM on GP%02d  : %d MB (QSPI)\n"
                " Free PSRAM     : %d KB\n", psram_pin, (int)(psram32 >> 20), (int)(butter_free / 1024));
        }
    }
#endif

    // On-chip temperature sensor via ADC
    {
        // Register bases differ between chips
    #if PICO_RP2350
        volatile uint32_t *resets     = (volatile uint32_t *)0x40020000;
        volatile uint32_t *adc_cs     = (volatile uint32_t *)0x400a0000;
        volatile uint32_t *adc_result = (volatile uint32_t *)0x400a0004;
        uint32_t ts_ch = rp2350a ? 4 : 8;
    #else // RP2040
        volatile uint32_t *resets     = (volatile uint32_t *)0x4000c000;
        volatile uint32_t *adc_cs     = (volatile uint32_t *)0x4004c000;
        volatile uint32_t *adc_result = (volatile uint32_t *)0x4004c004;
        uint32_t ts_ch = 4;
    #endif

        // Unreset ADC block: clear bit 0 in RESETS_RESET, wait bit 0 in RESET_DONE
        resets[0] &= ~1u;                      // RESET: clear ADC bit
        while (!(resets[2] & 1u)) {}            // RESET_DONE: wait ADC ready

        *adc_cs = 1; // EN=1
        while (!(*adc_cs & (1 << 8))) {} // wait READY
        *adc_cs = (ts_ch << 12) | (1 << 1) | 1; // AINSEL=ch, TS_EN=1, EN=1
        sleep_ms(1); // let temp sensor stabilize
        *adc_cs = (ts_ch << 12) | (1 << 2) | (1 << 1) | 1; // + START_ONCE
        while (!(*adc_cs & (1 << 8))) {} // wait READY
        uint16_t raw = *adc_result & 0xFFF;
        *adc_cs = 0; // disable ADC

        // T = 27 - (V - 0.706) / 0.001721, V = raw * 3.3 / 4096
        // Integer: T*10 = 270 - (raw*33000/4096 - 7060) * 100 / 1721
        int uv10 = (int)raw * 33000 / 4096; // voltage * 10000 (0..33000)
        int temp_x10 = 270 - (uv10 - 7060) * 100 / 1721;
        int t_int = temp_x10 / 10;
        int t_frac = (temp_x10 < 0 ? -temp_x10 : temp_x10) % 10;
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Temperature    : %d.%d C\n", t_int, t_frac);
    }

    showTextDialog("Chip Info", buf);
}

void OSD::BoardInfo() {
    char (&buf)[OSD_INFO_BUF_SZ] = osd_info_buf;
    int pos = 0;

    // SD Card
    if (FileUtils::fsMount) {
        FATFS* fsp;
        DWORD fre_clust;
        if (f_getfree("", &fre_clust, &fsp) == FR_OK) {
            uint32_t tot_mb = (uint32_t)((fsp->n_fatent - 2) * fsp->csize / 2048);
            uint32_t fre_mb = (uint32_t)(fre_clust * fsp->csize / 2048);
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " SD Card        : %d/%d MB free\n", (int)fre_mb, (int)tot_mb);

            const char* fs_name = "?";
            switch (fsp->fs_type) {
                case FS_FAT12: fs_name = "FAT12"; break;
                case FS_FAT16: fs_name = "FAT16"; break;
                case FS_FAT32: fs_name = "FAT32"; break;
                case FS_EXFAT: fs_name = "exFAT"; break;
            }
            uint32_t cluster_kb = (uint32_t)fsp->csize / 2;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "  FS / cluster  : %s / %u KB\n", fs_name, (unsigned)cluster_kb);

            char label[34] = {0};
            DWORD vsn = 0;
            if (f_getlabel("", label, &vsn) == FR_OK) {
                pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "  Label / VSN   : '%s' / %04lX-%04lX\n",
                    label[0] ? label : "(none)",
                    (unsigned long)((vsn >> 16) & 0xFFFF),
                    (unsigned long)(vsn & 0xFFFF));
            }
        } else {
            pos += snprintf(buf + pos, sizeof(buf) - pos, " SD Card        : mounted\n");
        }

        // Card type via MMC_GET_TYPE (CT_SD1=0x02, CT_SD2=0x04, CT_MMC=0x01, CT_BLOCK=0x08)
        BYTE ct = 0;
        if (disk_ioctl(0, MMC_GET_TYPE, &ct) == RES_OK) {
            const char* ct_name = "?";
            if (ct & 0x01) ct_name = "MMCv3";
            else if (ct & 0x02) ct_name = "SDv1";
            else if (ct & 0x04) ct_name = (ct & 0x08) ? "SDHC/SDXC" : "SDv2";
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "  Card type     : %s\n", ct_name);
        }

        // CID: manufacturer + product name (5 ASCII) + serial
        BYTE cid[16];
        if (disk_ioctl(0, MMC_GET_CID, cid) == RES_OK) {
            char pname[6];
            for (int i = 0; i < 5; i++) {
                char c = (char)cid[3 + i];
                pname[i] = (c >= 0x20 && c < 0x7F) ? c : ' ';
            }
            pname[5] = 0;
            uint32_t serial = ((uint32_t)cid[9] << 24) | ((uint32_t)cid[10] << 16)
                            | ((uint32_t)cid[11] << 8) | (uint32_t)cid[12];
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "  CID name/sn   : '%s' / %08lX\n", pname, (unsigned long)serial);
        }
    } else {
        pos += snprintf(buf + pos, sizeof(buf) - pos, " SD Card        : not mounted\n");
    }

    // Audio
    if (Config::audio_driver == 4)
        pos += snprintf(buf + pos, sizeof(buf) - pos, " Audio          : HDMI\n");
    else if (Config::audio_driver == 3)
        pos += snprintf(buf + pos, sizeof(buf) - pos, " Audio          : AY-3-8910\n");
    else
        pos += snprintf(buf + pos, sizeof(buf) - pos, " Audio          : %s [%02Xh]%s\n",
            (is_i2s_enabled ? "i2s" : "PWM"), link_i2s_code,
            (Config::audio_driver == 0 ? " auto" : ""));

#ifdef VGA_HDMI
    pos += snprintf(buf + pos, sizeof(buf) - pos, " VGA/HDMI detect: %02Xh\n", linkVGA01);
#endif

    // RAM pages
    if (!psram_pages)
        pos += snprintf(buf + pos, sizeof(buf) - pos, " 16K RAM pages  : %d[s%d:b%d:v%d]\n",
            ram_pages + butter_pages + swap_pages, ram_pages, butter_pages, swap_pages);
    else if (!butter_pages)
        pos += snprintf(buf + pos, sizeof(buf) - pos, " 16K RAM pages  : %d[s%d:p%d:v%d]\n",
            ram_pages + psram_pages + swap_pages, ram_pages, psram_pages, swap_pages);
    else if (!swap_pages)
        pos += snprintf(buf + pos, sizeof(buf) - pos, " 16K RAM pages  : %d[s%d:b%d:p%d]\n",
            ram_pages + butter_pages + psram_pages, ram_pages, butter_pages, psram_pages);
    else
        pos += snprintf(buf + pos, sizeof(buf) - pos, " 16K RAM pages  : %d[s%d:b%d:p%d:v%d]\n",
            ram_pages + butter_pages + psram_pages + swap_pages, ram_pages, butter_pages, psram_pages, swap_pages);

#if !PICO_RP2040
    if (DivMMC::enabled) {
        const char* mode_names[] = { "OFF", "DivMMC", "DivIDE", "DivSD" };
        const char* mem_type = DivMMC::use_psram ? "PSRAM" : "swap";
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %-15s: 128K+8K [%s]\n",
            mode_names[Config::esxdos], mem_type);
    }
#endif

    // GPIO pins (all labels 16 chars after "  " prefix, colon at col 18)
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\n GPIO pins:\n");
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  Kbd CLK/DATA  : %d/%d\n", KBD_CLOCK_PIN, KBD_DATA_PIN);
#ifdef VGA_BASE_PIN
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  VGA base      : %d\n", VGA_BASE_PIN);
#endif
#ifdef HDMI_BASE_PIN
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  HDMI base     : %d\n", HDMI_BASE_PIN);
#endif
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  PWM L/R       : %d/%d\n", PWM_PIN0, PWM_PIN1);
#ifdef BEEPER_PIN
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  Beeper        : %d\n", BEEPER_PIN);
#endif
#if defined(I2S_DATA_PIO) && defined(I2S_BCK_PIO) && defined(I2S_LCK_PIO)
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  I2S D/BCK/LCK : %d/%d/%d\n", I2S_DATA_PIO, I2S_BCK_PIO, I2S_LCK_PIO);
#endif
#ifdef PCM5122_I2S_DATA
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  PCM5122 I2S   : %d/%d/%d\n", PCM5122_I2S_DATA, PCM5122_I2S_BCK, PCM5122_I2S_LCK);
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  PCM5122 I2C   : %d/%d\n", PCM5122_I2C_SDA, PCM5122_I2C_SCL);
#endif
#ifdef LATCH_595_PIN
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  AY 595 L/C/D  : %d/%d/%d\n", LATCH_595_PIN, CLK_595_PIN, DATA_595_PIN);
#endif
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  SD MI/MO/CK/CS: %d/%d/%d/%d\n",
        SDCARD_PIN_SPI0_MISO, SDCARD_PIN_SPI0_MOSI, SDCARD_PIN_SPI0_SCK, SDCARD_PIN_SPI0_CS);
#ifdef PSRAM_PIN_CS
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  PSRAM CS/CK/IO: %d/%d/%d/%d\n",
        PSRAM_PIN_CS, PSRAM_PIN_SCK, PSRAM_PIN_MOSI, PSRAM_PIN_MISO);
#endif
#ifdef BUTTER_PSRAM_GPIO
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  Butter PSRAM  : %d\n", BUTTER_PSRAM_GPIO);
#endif
#ifdef MIDI_TX_PIN
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  MIDI TX       : %d\n", MIDI_TX_PIN);
#endif
#ifdef LOAD_WAV_PIO
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  LOAD WAV      : %d\n", LOAD_WAV_PIO);
#endif
#ifdef USE_NESPAD
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  NES CLK/LAT/D : %d/%d/%d\n", NES_GPIO_CLK, NES_GPIO_LAT, NES_GPIO_DATA);
#endif
#ifdef PICO_DEFAULT_LED_PIN
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "  LED           : %d\n", PICO_DEFAULT_LED_PIN);
#endif

    // Build info
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        "\n %s\n"
        " Built %s %s\n"
        " branch '%s'\n"
        " commit [%s]\n",
        PICO_BUILD_NAME, __DATE__, __TIME__, PICO_GIT_BRANCH, PICO_GIT_COMMIT);

    showTextDialog("Board Info", buf);
}

// Helper: append just the filename part of a path, truncated to maxlen chars
static int appendFilename(char* buf, int pos, int bufsize, const string& path, int maxlen) {
    if (path.empty()) return snprintf(buf + pos, bufsize - pos, "(none)");
    size_t slash = path.find_last_of("/");
    const char* fn = (slash != string::npos) ? path.c_str() + slash + 1 : path.c_str();
    int len = strlen(fn);
    if (len <= maxlen)
        return snprintf(buf + pos, bufsize - pos, "%s", fn);
    else
        return snprintf(buf + pos, bufsize - pos, "..%s", fn + len - (maxlen - 2));
}

void OSD::EmulatorInfo() {
    char (&buf)[OSD_INFO_BUF_SZ] = osd_info_buf;
    int pos = 0;

    // --- Machine ---
    pos += snprintf(buf + pos, sizeof(buf) - pos,
        " --- Machine ---\n"
        " Architecture   : %s\n"
        " ROM set        : %s\n"
        " Issue 2        : %s\n"
        " ALU Timing     : %s\n",
        Config::arch.c_str(),
        Config::romSet.c_str(),
        Config::Issue2 ? "On" : "Off",
        Config::AluTiming == 0 ? "Early" : "Late");

    // --- Video ---
    {
        const char* vmname;
        uint8_t vm = VIDEO::activeVideoMode();
        switch (vm) {
            case Config::VM_640x480_60: vmname = "640x480@60"; break;
            case Config::VM_640x480_50: vmname = "640x480@50"; break;
            case Config::VM_720x480_60: vmname = "720x480@60"; break;
            case Config::VM_720x576_60: vmname = "720x576@60"; break;
            case Config::VM_720x576_50: vmname = "720x576@50"; break;
            default:                    vmname = "unknown";    break;
        }
        pos += snprintf(buf + pos, sizeof(buf) - pos, "\n --- Video ---\n");
#ifdef VGA_HDMI
        extern bool SELECT_VGA;
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Video output   : %s %s\n",
            SELECT_VGA ? "VGA" : "HDMI", vmname);
#else
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Video mode     : %s\n", vmname);
#endif
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Scanlines      : %s\n"
            " Render         : %s\n"
            " Palette        : %s\n",
            Config::scanlines ? "On" : "Off",
            Config::render ? "Snow effect" : "Standard",
            VIDEO::paletteName(Config::palette));
#if !PICO_RP2040
        {
            const char* gs;
            if (!Config::gigascreen_enabled || Config::gigascreen_onoff == 0) gs = "Off";
            else if (Config::gigascreen_onoff == 1) gs = "On";
            else gs = "Auto";
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " Gigascreen     : %s\n"
                " ULA+           : %s\n"
                " Timex video    : %s\n",
                gs,
                Config::ulaplus ? "On" : "Off",
                Config::timex_video ? "On (#FF)" : "Off");
        }
#endif
    }

    // --- Sound ---
    {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "\n --- Sound ---\n");

        // AY chip
        if (Config::AY48) {
            static const char* stereo[] = { "ABC", "ACB", "Mono" };
            int si = Config::ayConfig;
            if (si > 2) si = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " AY-3-8912      : On (%s) #FFFD\n", stereo[si]);
        } else {
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " AY-3-8912      : Off\n");
        }

        // TurboSound
        {
            static const char* ts[] = { "Off", "NedoPC", "old-TC", "Both" };
            int ti = Config::turbosound;
            if (ti > 3) ti = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " TurboSound     : %s\n", ts[ti]);
        }

        // Covox
        if (Config::covox == 1)
            pos += snprintf(buf + pos, sizeof(buf) - pos, " Covox          : On (#FB)\n");
        else if (Config::covox == 2)
            pos += snprintf(buf + pos, sizeof(buf) - pos, " Covox          : On (#DD)\n");
        else
            pos += snprintf(buf + pos, sizeof(buf) - pos, " Covox          : Off\n");

#if !PICO_RP2040
        // SAA1099
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " SAA1099        : %s\n",
            Config::SAA1099 ? "On (#FF)" : "Off");

        // MIDI
        if (Config::midi == 0) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, " MIDI           : Off\n");
        } else if (Config::midi == 3) {
            static const char* presets[] = {
                "GM", "Piano", "Chiptune", "Strings",
                "Rock", "Organ", "MusicBox", "Synth"
            };
            int pi = Config::midi_synth_preset;
            if (pi > 7) pi = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " MIDI           : Synth (%s)\n", presets[pi]);
        } else {
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " MIDI           : %s\n",
                Config::midi == 1 ? "AY bitbang" : "ShamaZX");
        }
#endif

        // Audio driver
        if (Config::audio_driver == 4)
            pos += snprintf(buf + pos, sizeof(buf) - pos, " Audio driver   : HDMI\n");
        else if (Config::audio_driver == 3)
            pos += snprintf(buf + pos, sizeof(buf) - pos, " Audio driver   : AY-3-8910\n");
        else
            pos += snprintf(buf + pos, sizeof(buf) - pos, " Audio driver   : %s%s\n",
                (is_i2s_enabled ? "i2s" : "PWM"),
                (Config::audio_driver == 0 ? " (auto)" : ""));
    }

    // --- Input ---
    {
        static const char* jnames[] = {
            "Cursor", "Kempston", "Sinclair 1",
            "Sinclair 2", "Fuller", "Custom", "None"
        };
        int ji = Config::joystick;
        if (ji > 6) ji = 6;
        pos += snprintf(buf + pos, sizeof(buf) - pos,
            "\n --- Input ---\n");

        // Joystick with port number
        if (ji == JOY_KEMPSTON)
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " Joystick       : Kempston (#%02X)\n", Config::kempstonPort);
        else if (ji == JOY_FULLER)
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " Joystick       : Fuller (#7F)\n");
        else
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " Joystick       : %s\n", jnames[ji]);

        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " TAB as fire    : %s\n",
            Config::TABasfire1 ? "On" : "Off");
    }

    // --- Storage ---
    {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "\n --- Storage ---\n");

#if !PICO_RP2040
        // esxDOS
        {
            static const char* esx[] = { "Off", "DivMMC", "DivIDE", "DivSD" };
            int ei = Config::esxdos;
            if (ei > 3) ei = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " esxDOS         : %s\n", esx[ei]);

            // Show image names depending on mode (hd0/hd1 are shared slots).
            if (ei == 1) {
                // DivMMC — shows hd0 only.
                pos += snprintf(buf + pos, sizeof(buf) - pos, "  hd0           : ");
                pos += appendFilename(buf, pos, sizeof(buf), Config::esxdos_hdf_image[0], 19);
                pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
            } else if (ei == 2) {
                // DivIDE — both slots.
                pos += snprintf(buf + pos, sizeof(buf) - pos, "  hd0           : ");
                pos += appendFilename(buf, pos, sizeof(buf), Config::esxdos_hdf_image[0], 19);
                pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
                pos += snprintf(buf + pos, sizeof(buf) - pos, "  hd1           : ");
                pos += appendFilename(buf, pos, sizeof(buf), Config::esxdos_hdf_image[1], 19);
                pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
            }
        }
#endif

        // TR-DOS — available for Pentagon or Byte 128K
        {
            bool trdos_available = Z80Ops::isPentagon || (Z80Ops::is128 && Z80Ops::isByte);
            if (trdos_available) {
                pos += snprintf(buf + pos, sizeof(buf) - pos, " TR-DOS         : On");
                if (Config::trdosFastMode) {
                    pos += snprintf(buf + pos, sizeof(buf) - pos, " (fast)");
                }
                pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");

                // Show disk drives A:-D:
                static const char drive_letter[] = "ABCD";
                for (int i = 0; i < 4; i++) {
                    pos += snprintf(buf + pos, sizeof(buf) - pos, "  %c:            : ", drive_letter[i]);
                    if (ESPectrum::fdd.disk[i] && !ESPectrum::fdd.disk[i]->fname.empty()) {
                        pos += appendFilename(buf, pos, sizeof(buf), ESPectrum::fdd.disk[i]->fname, 19);
                        if (Config::driveWP[i])
                            pos += snprintf(buf + pos, sizeof(buf) - pos, " WP");
                    } else
                        pos += snprintf(buf + pos, sizeof(buf) - pos, "(empty)");
                    pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
                }
            } else {
                pos += snprintf(buf + pos, sizeof(buf) - pos, " TR-DOS         : Off\n");
            }
        }

#if !PICO_RP2040
        // MB-02+
        if (MB02::enabled) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, " MB-02+         : On\n");
            for (int i = 0; i < 4; i++) {
                pos += snprintf(buf + pos, sizeof(buf) - pos, "  %02d            : ", i + 1);
                if (ESPectrum::mb02_fdd.disk[i] && !ESPectrum::mb02_fdd.disk[i]->fname.empty()) {
                    pos += appendFilename(buf, pos, sizeof(buf), ESPectrum::mb02_fdd.disk[i]->fname, 19);
                    if (Config::mb02WP[i])
                        pos += snprintf(buf + pos, sizeof(buf) - pos, " WP");
                } else
                    pos += snprintf(buf + pos, sizeof(buf) - pos, "(empty)");
                pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
            }
        } else if (Config::mb02) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, " MB-02+         : No PSRAM\n");
        }
#endif

#if !PICO_RP2040
        // DMA
        if (Config::dma_mode == 1)
            pos += snprintf(buf + pos, sizeof(buf) - pos, " DMA            : Z80 DMA (#0B)\n");
        else if (Config::dma_mode == 2)
            pos += snprintf(buf + pos, sizeof(buf) - pos, " DMA            : zxnDMA (#6B)\n");
        else
            pos += snprintf(buf + pos, sizeof(buf) - pos, " DMA            : Off\n");
#endif

        // Tape
        pos += snprintf(buf + pos, sizeof(buf) - pos, " Tape           : ");
        pos += appendFilename(buf, pos, sizeof(buf), Tape::tapeFileName, 19);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");

        pos += snprintf(buf + pos, sizeof(buf) - pos,
            " Flash load     : %s\n",
            Config::flashload ? "On" : "Off");
    }

    showTextDialog(Config::lang ? "Info emulador" : "Emulator Info", buf);
}

static void __not_in_flash_func(flash_block)(const uint8_t* buffer, size_t flash_target_offset) {
    // ensure it is required to write block (may be, it is already the same)
    for (size_t i = 0; i < 512; ++i) {
        if (buffer[i] != *(uint8_t*)(XIP_BASE + flash_target_offset + i)) {
            goto flash_it;
        }
    }
    return;
flash_it:
    #ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, flash_target_offset % (FLASH_SECTOR_SIZE << 2) == 0);
    #endif
    multicore_lockout_start_blocking();
    const uint32_t ints = save_and_disable_interrupts();
    if (flash_target_offset % FLASH_SECTOR_SIZE == 0) { // cleanup_block
        flash_range_erase(flash_target_offset, FLASH_SECTOR_SIZE);
    }
    flash_range_program(flash_target_offset, buffer, 512);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
    #ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    #endif
}

bool OSD::updateROM(const string& fname, uint8_t arch) {
    FIL* f = fopen2(fname.c_str(), FA_READ);
    if (!f) {
        osdCenteredMsg(OSD_NOROMFILE_ERR[Config::lang], LEVEL_WARN, 2000);
        return false;
    }
    FSIZE_t bytesfirmware = f_size(f);
    const uint8_t* rom;
    FSIZE_t max_rom_size = 0;
    string dlgTitle = OSD_ROM[Config::lang];
    // Flash custom ROM 48K
    if ( arch == 1 ) {
#if !CARTRIDGE_AS_CUSTOM || NO_ALF
        if( bytesfirmware > 0x4000 ) {
            osdCenteredMsg("Too long file", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
#if NO_SEPARATE_48K_CUSTOM
        rom = gb_rom_0_128k_custom;
#else
        rom = gb_rom_0_48k_custom;
#endif
        max_rom_size = 16 << 10;
#else
        if( bytesfirmware > (1ul << 20) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_Alf_cart;
        max_rom_size = 1ul << 20;
#endif
        dlgTitle += " 48K   ";
        Config::arch = "48K";
        Config::romSet = "48Kcs";
        Config::romSet48 = "48Kcs";
        Config::pref_arch = "48K";
        Config::pref_romSet_48 = "48Kcs";
    }
    // Flash custom ROM 128K
    else if ( arch == 2 ) {
#if !CARTRIDGE_AS_CUSTOM || NO_ALF
        if( bytesfirmware > 0x8000 ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_0_128k_custom;
        if (bytesfirmware <= (16 << 10)) {
            max_rom_size = 16 << 10;
        } else {
            max_rom_size = 32 << 10;
        }
#else
        if( bytesfirmware > (1ul << 20) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_Alf_cart;
        if (bytesfirmware <= (16 << 10)) {
            max_rom_size = 16 << 10;
        } else if (bytesfirmware <= (32 << 10)) {
            max_rom_size = 32 << 10;
        } else {
            max_rom_size = 1ul << 20;
        }
#endif
        dlgTitle += " 128K  ";
        Config::arch = "128K";
        Config::romSet = "128Kcs";
        Config::romSet128 = "128Kcs";
        Config::pref_arch = "128K";
        Config::pref_romSet_128 = "128Kcs";
    }
    else if ( arch == 3 ) {
#if !CARTRIDGE_AS_CUSTOM || NO_ALF
        if( bytesfirmware > 0x8000 ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_0_128k_custom;
        if (bytesfirmware <= (16 << 10)) {
            max_rom_size = 16 << 10;
        } else {
            max_rom_size = 32 << 10;
        }
#else
        if( bytesfirmware > (1ul << 20) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_Alf_cart;
        if (bytesfirmware <= (16 << 10)) {
            max_rom_size = 16 << 10;
        } else if (bytesfirmware <= (32 << 10)) {
            max_rom_size = 32 << 10;
        } else {
            max_rom_size = 1ul << 20;
        }
#endif
        dlgTitle += " Pentagon ";
        Config::arch = "Pentagon";
        Config::romSet = "128Kcs";
        Config::romSetPent = "128Kcs";
        Config::pref_arch = "Pentagon";
        Config::pref_romSetPent = "128Kcs";
    }
#if !NO_ALF
    else if ( arch == 4 ) {
        if( bytesfirmware > (256ul << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_Alf;
        max_rom_size = 256ul << 10;
        dlgTitle += " ALF ROM ";
        Config::arch = "ALF";
        Config::romSet = "ALF";
        Config::pref_arch = "ALF";
    }
    else if ( arch == 5 ) {
        if( (size_t)((bytesfirmware >> 10) & 0xFFFFFFFF) > (1ul << 10) ) {
            char b[40];
            snprintf(b, 40, "Unsupported file (by size: %d KB)", (size_t)((bytesfirmware >> 10) & 0xFFFFFFFF));
            osdCenteredMsg(b, LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_Alf_cart;
        max_rom_size = 1ul << 20;
        dlgTitle += " ALF Cartridge ";
        Config::arch = "ALF";
    }
#endif
    else if ( arch == 6 ) {
        if( bytesfirmware > (16ul << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        switch (Config::trdosBios) {
            case 0: rom = gb_rom_4_trdos_503; break;
            case 1: rom = gb_rom_4_trdos_504tm; break;
            default: rom = gb_rom_4_trdos_505d; break;
        }
        max_rom_size = 16ul << 10;
        dlgTitle += " TRDOS ";
        Config::arch = "Pentagon";
    }
    else if ( arch == 7 ) {
        if( bytesfirmware > (32ul << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_pentagon_128k;
        max_rom_size = bytesfirmware > (16ul << 10) ? (32ul << 10) : (16ul << 10);
        dlgTitle += " Pentagon#0 ";
        Config::arch = "Pentagon";
        Config::romSet = "128Kp";
        Config::romSetPent = "128Kp";
        Config::pref_arch = "Pentagon";
        Config::pref_romSetPent = "128Kp";
    }
    else if ( arch == 8 ) {
        if( bytesfirmware > (16 << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_pentagon_128k;
        max_rom_size = 16 << 10;
        dlgTitle += " Pentagon#1 ";
        Config::arch = "Pentagon";
        Config::romSet = "128Kp";
        Config::romSetPent = "128Kp";
        Config::pref_arch = "Pentagon";
        Config::pref_romSetPent = "128Kp";
    }
    else {
        osdCenteredMsg("Unexpected ROM type: " + to_string(arch), LEVEL_WARN, 2000);
        fclose2(f);
        return false;
    }
    size_t flash_target_offset = (size_t)rom - XIP_BASE;
    UINT br;
    const size_t sz = 512;
    uint8_t* buffer = (uint8_t*)malloc(sz);
    FSIZE_t i = 0;
    for (; i < bytesfirmware && i < max_rom_size; i += sz) {
        memset(buffer, 0, sz);
        if ( f_read(f, buffer, sz, &br) != FR_OK) {
            osdCenteredMsg(fname + " - unable to read", LEVEL_ERROR, 5000);
            free(buffer);
            fclose2(f);
            return false;
        }
        flash_block(buffer, flash_target_offset + (size_t)(i & 0xFFFFFFFF));
    }
    fclose2(f);
    memset(buffer, 0, sz);
    for (; i < max_rom_size; i += sz) {
        flash_block(buffer, flash_target_offset + (size_t)(i & 0xFFFFFFFF));
    }
    free(buffer);
    Config::save();
///    Config::requestMachine(Config::arch, Config::romSet);
    // Firmware written: reboot
///    ESPectrum::reset();
    OSD::esp_hard_reset();
    return true;
}

bool OSD::updateFirmware(FIL* firmware) {
    /**
    char ota_write_data[FWBUFFSIZE + 1] = { 0 };
    // get the currently running partition
    const esp_partition_t *partition = esp_ota_get_running_partition();
    if (partition == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

// Grab next update target
// const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
string splabel;
if (strcmp(partition->label,"esp0")==0) splabel = "esp1"; else splabel= "esp0";
const esp_partition_t *target = esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_ANY,splabel.c_str());
if (target == NULL) {
    return ESP_ERR_NOT_FOUND;
}

// printf("Running partition %s type %d subtype %d at offset 0x%x.\n", partition->label, partition->type, partition->subtype, partition->address);
// printf("Target  partition %s type %d subtype %d at offset 0x%x.\n", target->label, target->type, target->subtype, target->address);

// osdCenteredMsg(OSD_FIRMW_BEGIN[Config::lang], LEVEL_INFO,0);

progressDialog(OSD_FIRMW[Config::lang],OSD_FIRMW_BEGIN[Config::lang],0,0);

// Fake erase progress bar ;D
delay(100);
for(int n=0; n <= 100; n += 10) {
    progressDialog("","",n,1);
    delay(100);
}

esp_ota_handle_t ota_handle;
esp_err_t result = esp_ota_begin(target, OTA_SIZE_UNKNOWN, &ota_handle);
if (result != ESP_OK) {
    progressDialog("","",0,2);
    return result;
}

size_t bytesread;
uint32_t byteswritten = 0;

// osdCenteredMsg(OSD_FIRMW_WRITE[Config::lang], LEVEL_INFO,0);
progressDialog(OSD_FIRMW[Config::lang],OSD_FIRMW_WRITE[Config::lang],0,1);

// Get firmware size
fseek(firmware, 0, SEEK_END);
long bytesfirmware = ftell(firmware);
rewind(firmware);

while (1) {
    bytesread = fread(ota_write_data, 1, 0x1000 , firmware);
    result = esp_ota_write(ota_handle,(const void *) ota_write_data, bytesread);
    if (result != ESP_OK) {
        progressDialog("","",0,2);
        return result;
    }
    byteswritten += bytesread;
    progressDialog("","",(float) 100 / ((float) bytesfirmware / (float) byteswritten),1);
    // printf("Bytes written: %d\n",byteswritten);
    if (feof(firmware)) break;
}

result = esp_ota_end(ota_handle);
if (result != ESP_OK)
{
    // printf("esp_ota_end failed, err=0x%x.\n", result);
    progressDialog("","",0,2);
    return result;
}

result = esp_ota_set_boot_partition(target);
if (result != ESP_OK) {
    // printf("esp_ota_set_boot_partition failed, err=0x%x.\n", result);
    progressDialog("","",0,2);
    return result;
}

// osdCenteredMsg(OSD_FIRMW_END[Config::lang], LEVEL_INFO, 0);
progressDialog(OSD_FIRMW[Config::lang],OSD_FIRMW_END[Config::lang],100,1);

*/
    // Enable StartMsg
    Config::StartMsg = true;
    Config::save();
    delay(5000);
    // Firmware written: reboot
    OSD::esp_hard_reset();
    return true;
}

void OSD::progressDialog(const string& title, const string& msg, int percent, int action) {

    static unsigned short h;
    static unsigned short y;
    static unsigned short w;
    static unsigned short x;
    static unsigned short progress_x;
    static unsigned short progress_y;
    static unsigned int j;

    if (action == 0 ) { // SHOW

        h = (OSD_FONT_H * 6) + 2;
        y = scrAlignCenterY(h);

        size_t maxchars = (scrW / 6) - 4;
        string tmsg = msg.length() > maxchars ? msg.substr(0, maxchars) : msg;
        string ttitle = title.length() > maxchars ? title.substr(0, maxchars) : title;

        w = (((tmsg.length() > ttitle.length() + 6 ? tmsg.length(): ttitle.length() + 6) + 2) * OSD_FONT_W) + 2;
        x = scrAlignCenterX(w);

        // Save backbuffer data
        VIDEO::SaveRect.save(x, y, w, h);

        // Set font
        VIDEO::vga.setFont(Font6x8);

        // Menu border
        VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

        VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
        VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

        // Title
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
        VIDEO::vga.print(ttitle.c_str());

        // Msg
        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
        VIDEO::vga.setCursor(scrAlignCenterX(tmsg.length() * OSD_FONT_W), y + 1 + (OSD_FONT_H * 2));
        VIDEO::vga.print(tmsg.c_str());

        // Rainbow
        unsigned short rb_y = y + 8;
        unsigned short rb_paint_x = x + w - 30;
        uint8_t rb_colors[] = {2, 6, 4, 5};
        for (uint8_t c = 0; c < 4; c++) {
            for (uint8_t i = 0; i < 5; i++) {
                VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
            }
            rb_paint_x += 5;
        }

        // Progress bar frame
        progress_x = scrAlignCenterX(72);
        progress_y = y + (OSD_FONT_H * 4);
        VIDEO::vga.rect(progress_x, progress_y, 72, OSD_FONT_H + 2, zxColor(0, 0));
        progress_x++;
        progress_y++;

    } else if (action == 1 ) { // UPDATE

        // Msg
        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
        VIDEO::vga.setCursor(scrAlignCenterX(msg.length() * OSD_FONT_W), y + 1 + (OSD_FONT_H * 2));
        VIDEO::vga.print(msg.c_str());

        // Progress bar
        int barsize = (70 * percent) / 100;
        VIDEO::vga.fillRect(progress_x, progress_y, barsize, OSD_FONT_H, zxColor(5,1));
        VIDEO::vga.fillRect(progress_x + barsize, progress_y, 70 - barsize, OSD_FONT_H, zxColor(7,1));
    } else if (action == 2) { // CLOSE
        // Restore backbuffer data
        VIDEO::SaveRect.restore_last();
    }
}

uint8_t OSD::msgDialog(const string& title_, const string& msg_) {

    const unsigned short h = (OSD_FONT_H * 6) + 2;
    const unsigned short y = scrAlignCenterY(h);
    uint8_t res = DLG_NO;

    string msg = msg_, title = title_;
    if (msg.length() > (scrW / 6) - 4) msg = msg.substr(0,(scrW / 6) - 4);
    if (title.length() > (scrW / 6) - 4) title = title.substr(0,(scrW / 6) - 4);

    const unsigned short w = (((msg.length() > title.length() + 6 ? msg.length() : title.length() + 6) + 2) * OSD_FONT_W) + 2;
    const unsigned short x = scrAlignCenterX(w);

    // Save backbuffer data
    VIDEO::SaveRect.save(x, y, w, h);

    // Set font
    VIDEO::vga.setFont(Font6x8);

    // Menu border
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(title.c_str());

    // Msg
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(msg.length() * OSD_FONT_W), y + 1 + (OSD_FONT_H * 2));
    VIDEO::vga.print(msg.c_str());

    // Yes
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) - (w >> 2), y + 1 + (OSD_FONT_H * 4));
    VIDEO::vga.print(Config::lang ? "  Si  " : " Yes  ");

    // // Ruler
    // VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    // VIDEO::vga.setCursor(x + 1, y + 1 + (OSD_FONT_H * 3));
    // VIDEO::vga.print("123456789012345678901234567");

    // No
    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) + (w >> 2), y + 1 + (OSD_FONT_H * 4));
    VIDEO::vga.print("  No  ");

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }

    // Keyboard loop
    fabgl::VirtualKeyItem Menukey;
    while (1) {
        // Process external keyboard
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPectrum::readKbd(&Menukey)) {
                if (!Menukey.down) continue;
                if (is_left(Menukey.vk)) {
                    // Yes
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) - (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print(Config::lang ? "  Si  " : " Yes  ");
                    // No
                    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) + (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print("  No  ");
                    click();
                    res = DLG_YES;
                } else if (is_right(Menukey.vk)) {
                    // Yes
                    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) - (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print(Config::lang ? "  Si  " : " Yes  ");
                    // No
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) + (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print("  No  ");
                    click();
                    res = DLG_NO;
                } else if (is_enter(Menukey.vk)) {
                    break;
                } else if (Menukey.vk == fabgl::VK_ESCAPE) {
                    res = DLG_CANCEL;
                    break;
                }
            }
        }
        sleep_ms(5);
    }

    click();

    // Restore backbuffer data
    VIDEO::SaveRect.restore_last();

    return res;

}

bool OSD::videoModeConfirm(int timeout_sec) {

    string title = "Video Mode";
    string msg = Config::lang ? "Mantener este modo?" : "Keep this video mode?";

    const unsigned short h = (OSD_FONT_H * 6) + 2;
    const unsigned short y = scrAlignCenterY(h);
    bool confirmed = false;

    const unsigned short w = (((msg.length() + 2) * OSD_FONT_W) + 2);
    const unsigned short x = scrAlignCenterX(w);

    VIDEO::SaveRect.save(x, y, w, h);

    VIDEO::vga.setFont(Font6x8);

    // Border
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    // Title bar
    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(title.c_str());

    // Message
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(msg.length() * OSD_FONT_W), y + 1 + (OSD_FONT_H * 2));
    VIDEO::vga.print(msg.c_str());

    // Countdown area (row 3) — will be updated each second
    unsigned short count_y = y + 1 + (OSD_FONT_H * 3);

    // Yes button (initially not selected)
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) - (w >> 2), y + 1 + (OSD_FONT_H * 4));
    VIDEO::vga.print(Config::lang ? "  Si  " : " Yes  ");

    // No button (initially selected/highlighted)
    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) + (w >> 2), y + 1 + (OSD_FONT_H * 4));
    VIDEO::vga.print("  No  ");

    // Rainbow decoration
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }

    int remaining = timeout_sec;
    int tick_count = 0;

    // Show initial countdown
    char countbuf[8];
    snprintf(countbuf, sizeof(countbuf), " [%2d] ", remaining);
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W), count_y);
    VIDEO::vga.print(countbuf);

    // Keyboard loop with countdown
    fabgl::VirtualKeyItem Menukey;
    while (remaining > 0) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPectrum::readKbd(&Menukey)) {
                if (!Menukey.down) continue;
                if (is_left(Menukey.vk)) {
                    // Highlight Yes
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) - (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print(Config::lang ? "  Si  " : " Yes  ");
                    // Unhighlight No
                    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) + (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print("  No  ");
                    click();
                    confirmed = true;
                } else if (is_right(Menukey.vk)) {
                    // Unhighlight Yes
                    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) - (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print(Config::lang ? "  Si  " : " Yes  ");
                    // Highlight No
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W) + (w >> 2), y + 1 + (OSD_FONT_H * 4));
                    VIDEO::vga.print("  No  ");
                    click();
                    confirmed = false;
                } else if (is_enter(Menukey.vk)) {
                    break;
                } else if (Menukey.vk == fabgl::VK_ESCAPE) {
                    confirmed = false;
                    break;
                }
            }
        }

        sleep_ms(5);
        tick_count++;
        if (tick_count >= 200) { // 200 * 5ms = 1 second
            tick_count = 0;
            remaining--;
            snprintf(countbuf, sizeof(countbuf), " [%2d] ", remaining);
            VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
            VIDEO::vga.setCursor(scrAlignCenterX(6 * OSD_FONT_W), count_y);
            VIDEO::vga.print(countbuf);
        }
    }

    click();

    VIDEO::SaveRect.restore_last();

    return confirmed;
}

string OSD::inputBox(int x, int y, const string& text) {

return text;

}

#define MENU_JOYSELKEY_EN "Key      \n"\
    "A-Z      \n"\
    "0-9      \n"\
    "Special  \n"\
    "PS/2     \n"\
    "Joystick \n"
#define MENU_JOYSELKEY_ES "Tecla    \n"\
    "A-Z      \n"\
    "0-9      \n"\
    "Especial \n"\
    "PS/2     \n"\
    "Joystick \n"
static const char *MENU_JOYSELKEY[2] = { MENU_JOYSELKEY_EN, MENU_JOYSELKEY_ES };

#define MENU_JOY_AZ "A-Z\n"\
    "A\n"\
    "B\n"\
    "C\n"\
    "D\n"\
    "E\n"\
    "F\n"\
    "G\n"\
	"H\n"\
	"I\n"\
	"J\n"\
	"K\n"\
    "L\n"\
    "M\n"\
    "N\n"\
    "O\n"\
    "P\n"\
    "Q\n"\
    "R\n"\
	"S\n"\
	"T\n"\
	"U\n"\
	"V\n"\
    "W\n"\
    "X\n"\
    "Y\n"\
    "Z\n"

#define MENU_JOY_09 "0-9\n"\
	"0\n"\
    "1\n"\
    "2\n"\
    "3\n"\
    "4\n"\
    "5\n"\
    "6\n"\
    "7\n"\
	"8\n"\
	"9\n"

#define MENU_JOY_SPECIAL\
    "Enter\n"\
    "Caps\n"\
    "SymbShift\n"\
    "Brk/Space\n"\
    "Backspace\n"\
    "KP 0/Ins\n"\
    "KP ./Del\n"\
    "None\n"

#define MENU_JOY_PS2 "PS/2\n"\
    "F1\n"\
    "F2\n"\
    "F3\n"\
    "F4\n"\
    "F5\n"\
	"F6\n"\
	"F7\n"\
    "F8\n"\
    "F9\n"\
    "F10\n"\
    "F11\n"\
    "F12\n"\
    "Pause\n"\
    "PrtScr\n"\
    "Left\n"\
    "Right\n"\
    "Up\n"\
    "Down\n"

#define MENU_JOY_KEMPSTON "DPAD\n"\
    "Left\n"\
    "Right\n"\
    "Up\n"\
    "Down\n"\
    "A\n"\
    "B\n"\
    "Start\n"\
    "Select\n"

string vkToText(int key) {

fabgl::VirtualKey vk = (fabgl::VirtualKey) key;

switch (vk)
{
case fabgl::VK_0:
    return "    0    ";
case fabgl::VK_1:
    return "    1    ";
case fabgl::VK_2:
    return "    2    ";
case fabgl::VK_3:
    return "    3    ";
case fabgl::VK_4:
    return "    4    ";
case fabgl::VK_5:
    return "    5    ";
case fabgl::VK_6:
    return "    6    ";
case fabgl::VK_7:
    return "    7    ";
case fabgl::VK_8:
    return "    8    ";
case fabgl::VK_9:
    return "    9    ";
case fabgl::VK_A:
    return "    A    ";
case fabgl::VK_B:
    return "    B    ";
case fabgl::VK_C:
    return "    C    ";
case fabgl::VK_D:
    return "    D    ";
case fabgl::VK_E:
    return "    E    ";
case fabgl::VK_F:
    return "    F    ";
case fabgl::VK_G:
    return "    G    ";
case fabgl::VK_H:
    return "    H    ";
case fabgl::VK_I:
    return "    I    ";
case fabgl::VK_J:
    return "    J    ";
case fabgl::VK_K:
    return "    K    ";
case fabgl::VK_L:
    return "    L    ";
case fabgl::VK_M:
    return "    M    ";
case fabgl::VK_N:
    return "    N    ";
case fabgl::VK_O:
    return "    O    ";
case fabgl::VK_P:
    return "    P    ";
case fabgl::VK_Q:
    return "    Q    ";
case fabgl::VK_R:
    return "    R    ";
case fabgl::VK_S:
    return "    S    ";
case fabgl::VK_T:
    return "    T    ";
case fabgl::VK_U:
    return "    U    ";
case fabgl::VK_V:
    return "    V    ";
case fabgl::VK_W:
    return "    W    ";
case fabgl::VK_X:
    return "    X    ";
case fabgl::VK_Y:
    return "    Y    ";
case fabgl::VK_Z:
    return "    Z    ";
case fabgl::VK_RETURN:
    return "  Enter  ";
case fabgl::VK_BACKSPACE:
    return "Backspace";
case fabgl::VK_KP_0:
    return "KP 0/Ins ";
case fabgl::VK_KP_PERIOD:
    return "KP ./Del ";
case fabgl::VK_SPACE:
    return "Brk/Space";
case fabgl::VK_LSHIFT:
    return "  Caps   ";
case fabgl::VK_RSHIFT:
    return " RShift  ";
case fabgl::VK_LCTRL:
    return "  LCtrl  ";
case fabgl::VK_RCTRL:
    return "  RCtrl  ";
case fabgl::VK_F1:
    return "   F1    ";
case fabgl::VK_F2:
    return "   F2    ";
case fabgl::VK_F3:
    return "   F3    ";
case fabgl::VK_F4:
    return "   F4    ";
case fabgl::VK_F5:
    return "   F5    ";
case fabgl::VK_F6:
    return "   F6    ";
case fabgl::VK_F7:
    return "   F7    ";
case fabgl::VK_F8:
    return "   F8    ";
case fabgl::VK_F9:
    return "   F9    ";
case fabgl::VK_F10:
    return "   F10   ";
case fabgl::VK_F11:
    return "   F11   ";
case fabgl::VK_F12:
    return "   F12   ";
case fabgl::VK_PAUSE:
    return "  Pause  ";
case fabgl::VK_PRINTSCREEN:
    return " PrtScr  ";
case fabgl::VK_LEFT:
    return "  Left   ";
case fabgl::VK_RIGHT:
    return "  Right  ";
case fabgl::VK_UP:
    return "   Up    ";
case fabgl::VK_DOWN:
    return "  Down   ";
case fabgl::VK_DPAD_LEFT:
    return "Joy.Left ";
case fabgl::VK_DPAD_RIGHT:
    return "Joy.Right";
case fabgl::VK_DPAD_UP:
    return " Joy.Up  ";
case fabgl::VK_DPAD_DOWN:
    return "Joy.Down ";
case fabgl::VK_DPAD_FIRE:
    return "  Joy.A  ";
case fabgl::VK_DPAD_ALTFIRE:
    return "  Joy.B  ";
case fabgl::VK_DPAD_SELECT:
    return " Joy.Sel ";
case fabgl::VK_DPAD_START:
    return "Joy.Start";
case fabgl::VK_HOME:
    return "  Home   ";
case fabgl::VK_END:
    return "   End   ";
case fabgl::VK_PAGEUP:
    return "  PgUp   ";
case fabgl::VK_PAGEDOWN:
    return "  PgDn   ";
case fabgl::VK_INSERT:
    return " Insert  ";
case fabgl::VK_DELETE:
    return " Delete  ";
case fabgl::VK_NUMLOCK:
    return " NumLock ";
case fabgl::VK_TAB:
    return "   Tab   ";
case fabgl::VK_TILDE:
    return "    ~    ";
case fabgl::VK_GRAVEACCENT:
    return "    `    ";
case fabgl::VK_SLASH:
    return "    /    ";
case fabgl::VK_BACKSLASH:
    return "    \\    ";
case fabgl::VK_SEMICOLON:
    return "    ;    ";
case fabgl::VK_QUOTE:
    return "    '    ";
case fabgl::VK_COMMA:
    return "    ,    ";
case fabgl::VK_PERIOD:
    return "    .    ";
case fabgl::VK_MINUS:
    return "    -    ";
case fabgl::VK_EQUALS:
    return "    =    ";
case fabgl::VK_LEFTBRACKET:
    return "    [    ";
case fabgl::VK_RIGHTBRACKET:
    return "    ]    ";
case fabgl::VK_VOLUMEUP:
    return "  Vol+   ";
case fabgl::VK_VOLUMEDOWN:
    return "  Vol-   ";
case fabgl::VK_VOLUMEMUTE:
    return "  Mute   ";
default:
    return "  None   ";
}

}

unsigned int joyControl[12][3]={
    {34,55,zxColor(0,0)}, // Left
    {87,55,zxColor(0,0)}, // Right
    {63,30,zxColor(0,0)}, // Up
    {63,78,zxColor(0,0)}, // Down
    {49,109,zxColor(0,0)}, // Start
    {136,109,zxColor(0,0)}, // Mode
    {145,69,zxColor(0,0)}, // A
    {205,69,zxColor(0,0)}, // B
    {265,69,zxColor(0,0)}, // C
    {145,37,zxColor(0,0)}, // X
    {205,37,zxColor(0,0)}, // Y
    {265,37,zxColor(0,0)} // Z
};

void DrawjoyControls(unsigned short x, unsigned short y) {

    // Draw joy controls

    // Left arrow
    for (int i = 0; i <= 5; i++) {
        VIDEO::vga.line(x + joyControl[0][0] + i, y + joyControl[0][1] - i, x + joyControl[0][0] + i, y + joyControl[0][1] + i, joyControl[0][2]);
    }

    // Right arrow
    for (int i = 0; i <= 5; i++) {
        VIDEO::vga.line(x + joyControl[1][0] + i, y + joyControl[1][1] - ( 5 - i), x + joyControl[1][0] + i, y + joyControl[1][1] + ( 5 - i), joyControl[1][2]);
    }

    // Up arrow
    for (int i = 0; i <= 6; i++) {
        VIDEO::vga.line(x + joyControl[2][0] - i, y + joyControl[2][1] + i, x + joyControl[2][0] + i, y + joyControl[2][1] + i, joyControl[2][2]);
    }

    // Down arrow
    for (int i = 0; i <= 6; i++) {
        VIDEO::vga.line(x + joyControl[3][0] - (6 - i), y + joyControl[3][1] + i, x + joyControl[3][0] + ( 6 - i), y + joyControl[3][1] + i, joyControl[3][2]);
    }

    // START text
    VIDEO::vga.setTextColor(joyControl[4][2], zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[4][0], y + joyControl[4][1]);
    VIDEO::vga.print("START");

    // MODE text
    VIDEO::vga.setTextColor(joyControl[5][2], zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[5][0], y + joyControl[5][1]);
    VIDEO::vga.print("MODE");

    // Text A
    VIDEO::vga.setTextColor( joyControl[6][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[6][0], y + joyControl[6][1]);
    VIDEO::vga.circle(x + joyControl[6][0] + 3, y + joyControl[6][1] + 3, 6,  joyControl[6][2]);
    VIDEO::vga.print("A");

    // Text B
    VIDEO::vga.setTextColor(joyControl[7][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[7][0], y + joyControl[7][1]);
    VIDEO::vga.circle(x + joyControl[7][0] + 3, y + joyControl[7][1] + 3, 6,  joyControl[7][2]);
    VIDEO::vga.print("B");

    // Text C
    VIDEO::vga.setTextColor(joyControl[8][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[8][0], y + joyControl[8][1]);
    VIDEO::vga.circle(x + joyControl[8][0] + 3, y + joyControl[8][1] + 3, 6, joyControl[8][2]);
    VIDEO::vga.print("C");

    // Text X
    VIDEO::vga.setTextColor(joyControl[9][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[9][0], y + joyControl[9][1]);
    VIDEO::vga.circle(x + joyControl[9][0] + 3, y + joyControl[9][1] + 3, 6, joyControl[9][2]);
    VIDEO::vga.print("X");

    // Text Y
    VIDEO::vga.setTextColor(joyControl[10][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[10][0], y + joyControl[10][1]);
    VIDEO::vga.circle(x + joyControl[10][0] + 3, y + joyControl[10][1] + 3, 6, joyControl[10][2]);
    VIDEO::vga.print("Y");

    // Text Z
    VIDEO::vga.setTextColor(joyControl[11][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[11][0], y + joyControl[11][1]);
    VIDEO::vga.circle(x + joyControl[11][0] + 3, y + joyControl[11][1] + 3, 6, joyControl[11][2]);
    VIDEO::vga.print("Z");

}

// Returns a short display string for a hotkey binding, left-aligned, e.g. "ALT+F1", "F2", "None"
static string hkBindingText(int idx) {
    const Config::HotkeyBinding &b = Config::hotkeys[idx];
    if (b.vk == (uint16_t)fabgl::VK_NONE) return "None";
    // vkToText returns a 9-char padded string; trim it
    string kname = vkToText(b.vk);
    size_t s = kname.find_first_not_of(' ');
    size_t e = kname.find_last_not_of(' ');
    string trimmed = (s == string::npos) ? "None" : kname.substr(s, e - s + 1);
    string text;
    if (b.ctrl && b.alt) text = "C+A+" + trimmed;
    else if (b.ctrl)      text = "C+" + trimmed;
    else if (b.alt)      text = "A+" + trimmed;
    else                 text = trimmed;
    return text;
}

static const char* hkIdNames[Config::HK_COUNT] = {
    "HK_MAIN_MENU", "HK_LOAD_SNA", "HK_PERSIST_LOAD", "HK_PERSIST_SAVE",
    "HK_LOAD_ANY", "HK_TAPE_PLAY", "HK_TAPE_BROWSER", "HK_STATS",
    "HK_VOL_DOWN", "HK_VOL_UP", "HK_HARD_RESET", "HK_REBOOT",
    "HK_MAX_SPEED", "HK_PAUSE", "HK_HW_INFO", "HK_TURBO",
    "HK_DEBUG", "HK_DISK", "HK_NMI", "HK_RESET_TO",
    "HK_USB_BOOT", "HK_GIGASCREEN", "HK_BP_LIST", "HK_JUMP_TO",
    "HK_POKE", "HK_VIDMODE_60", "HK_VIDMODE_50",
    "HK_QUICK_LOAD", "HK_QUICK_SAVE"
};

static string expandHotkeys(const char* menu) {
    string s(menu);
    size_t pos = 0;
    while ((pos = s.find('{', pos)) != string::npos) {
        size_t end = s.find('}', pos);
        if (end == string::npos) break;
        string token = s.substr(pos + 1, end - pos - 1);
        string replacement;
        for (int i = 0; i < Config::HK_COUNT; i++) {
            if (token == hkIdNames[i]) {
                string b = hkBindingText(i);
                if (b != "None") replacement = b + " ";
                break;
            }
        }
        s.replace(pos, end - pos + 1, replacement);
        pos += replacement.length();
    }
    return s;
}

// EN
const char* const hkDescEN[Config::HK_COUNT] = {
    "Main menu",            // HK_MAIN_MENU
    "Load (SNA,Z80,P)",     // HK_LOAD_SNA
    "Load snapshot",        // HK_PERSIST_LOAD
    "Save snapshot",        // HK_PERSIST_SAVE
    "Open file",            // HK_LOAD_ANY
    "Play/Stop tape",       // HK_TAPE_PLAY
    "Tape browser",         // HK_TAPE_BROWSER
    "CPU/Tape stats",       // HK_STATS
    "Volume down",          // HK_VOL_DOWN
    "Volume up",            // HK_VOL_UP
    "Hard reset",           // HK_HARD_RESET
#if PICO_RP2040
    "Reboot RP2040",        // HK_REBOOT
#else
    "Reboot RP2350",        // HK_REBOOT
#endif
    "Max speed toggle",     // HK_MAX_SPEED
    "Pause",                // HK_PAUSE
    "Hardware info",        // HK_HW_INFO
    "Turbo mode",           // HK_TURBO
    "Debug",                // HK_DEBUG
    "Insert disk",          // HK_DISK
    "NMI",                  // HK_NMI
    "Reset to...",          // HK_RESET_TO
    "USB Boot mode",        // HK_USB_BOOT
    "Gigascreen toggle",    // HK_GIGASCREEN
    "Breakpoint list",      // HK_BP_LIST
    "Jump to address",      // HK_JUMP_TO
    "Input poke",           // HK_POKE
    "HDMI 60Hz mode",       // HK_VIDMODE_60
    "HDMI 50Hz mode",       // HK_VIDMODE_50
    "Quick Load snapshot",  // HK_QUICK_LOAD
    "Quick Save snapshot",  // HK_QUICK_SAVE
};
// ES
const char* const hkDescES[Config::HK_COUNT] = {
    "Menu principal",        // HK_MAIN_MENU
    "Cargar (SNA,Z80,P)",    // HK_LOAD_SNA
    "Cargar snapshot",  // HK_PERSIST_LOAD
    "Guardar snapshot", // HK_PERSIST_SAVE
    "Abrir fichero",         // HK_LOAD_ANY
    "Play/Stop cinta",       // HK_TAPE_PLAY
    "Explorador cinta",      // HK_TAPE_BROWSER
    "Status CPU/Carga",      // HK_STATS
    "Bajar volumen",         // HK_VOL_DOWN
    "Subir volumen",         // HK_VOL_UP
    "Reset completo",        // HK_HARD_RESET
#if PICO_RP2040
    "Resetear RP2040",       // HK_REBOOT
#else
    "Resetear RP2350",       // HK_REBOOT
#endif
    "Velocidad maxima",      // HK_MAX_SPEED
    "Pausa",                 // HK_PAUSE
    "Info hardware",         // HK_HW_INFO
    "Modo turbo",            // HK_TURBO
    "Depurar",               // HK_DEBUG
    "Insertar disco",        // HK_DISK
    "NMI",                   // HK_NMI
    "Resetear a...",         // HK_RESET_TO
    "Modo USB Boot",         // HK_USB_BOOT
    "Gigascreen",            // HK_GIGASCREEN
    "Lista breakpoints",     // HK_BP_LIST
    "Ir a direccion",        // HK_JUMP_TO
    "Introducir poke",       // HK_POKE
    "Modo HDMI 60Hz",        // HK_VIDMODE_60
    "Modo HDMI 50Hz",        // HK_VIDMODE_50
    "Carga rapida snapshot", // HK_QUICK_LOAD
    "Guardado rapido snapshot", // HK_QUICK_SAVE
};

static const int HK_MENU_WIDTH = 32; // usable cols for hotkey menu

static string buildHotkeyMenu() {
    auto descs = Config::lang ? hkDescES : hkDescEN;
    string menu = Config::lang ? "Teclas rapidas\n" : "Hot Keys\n";
    for (int i = 0; i < Config::HK_COUNT; i++) {
        string left = descs[i];
        string right = hkBindingText(i);
        // Pad between left and right so right column is flush right
        int pad = HK_MENU_WIDTH - (int)left.length() - (int)right.length();
        if (pad < 1) pad = 1;
        // Prefix readonly entries with \x01 marker for dimmed rendering
        if (Config::hotkeys[i].readonly) menu += '\x01';
        menu += left + string(pad, ' ') + right + '\n';
    }
    return menu;
}

void OSD::hotkeyDialog() {
    auto Kbd = ESPectrum::PS2Controller.keyboard();
    fabgl::VirtualKeyItem Nextkey;
    bool changed = false;

    // Disable joystick mapping so LALT/cursors aren't remapped to DPAD
    bool savedCursorAsJoy = Config::CursorAsJoy;
    Config::CursorAsJoy = false;

    menu_level = 3;
    menu_curopt = 1;
    menu_saverect = true;

    while (1) {
        menu_footer = Config::lang
            ? "F6:Defaults F8:Borrar "
            : "F6:Defaults F8:Clear";
        string hmenu = buildHotkeyMenu();
        uint8_t opt = menuRun(hmenu);

        if (opt == 0) {
            // Escape: close dialog, save if changed
            menu_footer = "";
            if (changed) Config::save();
            Config::CursorAsJoy = savedCursorAsJoy;
            return;
        }

        int idx = opt - 1; // row 1 = title, so opt=1 → first entry
        if (idx < 0 || idx >= Config::HK_COUNT) continue;

        // Readonly entries: cannot be edited or cleared
        if (Config::hotkeys[idx].readonly && !menu_rename_pressed) {
            osdCenteredMsg(
                Config::lang ? " Solo lectura " : " Read only ",
                LEVEL_WARN, 800);
            menu_curopt = opt;
            menu_saverect = false;
            continue;
        }

        // Delete/F8: clear binding for selected entry
        if (menu_del_pressed) {
            if (!Config::hotkeys[idx].readonly) {
                Config::hotkeys[idx] = { (uint16_t)fabgl::VK_NONE, false, false };
                changed = true;
            }
            menu_curopt = opt;
            menu_saverect = false;
            continue;
        }

        // F6: reset all to defaults
        if (menu_rename_pressed) {
            uint8_t res = msgDialog(
                Config::lang ? "Teclas rapidas" : "Hot Keys",
                Config::lang ? "Restaurar por defecto?" : "Reset to defaults?");
            if (res == DLG_YES) {
                Config::initHotkeys();
                Config::save();
                changed = false;
            }
            menu_curopt = opt;
            menu_saverect = false;
            continue;
        }

        // Enter pressed: capture new key
        // Save background, show message, restore after capture
        {
            string msg = Config::lang ? " Pulsa tecla... (Esc=cancelar) " : " Press key... (Esc=cancel) ";
            const unsigned short mh = OSD_FONT_H * 3;
            const unsigned short my = scrAlignCenterY(mh);
            const unsigned short mw = (msg.length() + 2) * OSD_FONT_W;
            const unsigned short mx = scrAlignCenterX(mw);
            VIDEO::SaveRect.save(mx, my, mw, mh);
            VIDEO::vga.fillRect(mx, my, mw, mh, zxColor(1, 0));
            VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
            VIDEO::vga.setFont(Font6x8);
            VIDEO::vga.setCursor(mx + OSD_FONT_W, my + OSD_FONT_H);
            VIDEO::vga.print(msg.c_str());
        }

        // Drain queue (Enter release etc.)
        while (Kbd->virtualKeyAvailable())
            Kbd->getNextVirtualKey(&Nextkey);

        // Wait for a real key-down event
        bool alt = false, ctrl = false;
        while (1) {
            sleep_ms(5);
            if (Kbd->virtualKeyAvailable()) {
                Kbd->getNextVirtualKey(&Nextkey);
                if (Nextkey.vk == fabgl::VK_LALT || Nextkey.vk == fabgl::VK_RALT)
                    alt = Nextkey.down;
                if (Nextkey.vk == fabgl::VK_LCTRL || Nextkey.vk == fabgl::VK_RCTRL)
                    ctrl = Nextkey.down;
                if (!Nextkey.down) continue;
                fabgl::VirtualKey vk = Nextkey.vk;
                // Ignore modifier, joystick and menu-navigation keys
                if (vk == fabgl::VK_LALT || vk == fabgl::VK_RALT ||
                    vk == fabgl::VK_LCTRL || vk == fabgl::VK_RCTRL ||
                    vk == fabgl::VK_LSHIFT || vk == fabgl::VK_RSHIFT ||
                    vk == fabgl::VK_DPAD_FIRE || vk == fabgl::VK_DPAD_ALTFIRE ||
                    vk == fabgl::VK_DPAD_LEFT || vk == fabgl::VK_DPAD_RIGHT ||
                    vk == fabgl::VK_DPAD_UP || vk == fabgl::VK_DPAD_DOWN ||
                    vk == fabgl::VK_DPAD_SELECT || vk == fabgl::VK_DPAD_START ||
                    vk == fabgl::VK_MENU_LEFT || vk == fabgl::VK_MENU_RIGHT ||
                    vk == fabgl::VK_MENU_UP || vk == fabgl::VK_MENU_DOWN ||
                    vk == fabgl::VK_MENU_ENTER || vk == fabgl::VK_MENU_BS ||
                    vk == fabgl::VK_MENU_HOME)
                    continue;

                if (vk == fabgl::VK_ESCAPE) {
                    break;
                }

                // Only allow non-Spectrum keys (F-keys, navigation, special)
                if (!((vk >= fabgl::VK_F1 && vk <= fabgl::VK_F12) ||
                      vk == fabgl::VK_PAUSE || vk == fabgl::VK_PRINTSCREEN ||
                      vk == fabgl::VK_SCROLLLOCK || vk == fabgl::VK_NUMLOCK ||
                      vk == fabgl::VK_INSERT ||
                      vk == fabgl::VK_HOME || vk == fabgl::VK_END ||
                      vk == fabgl::VK_PAGEUP || vk == fabgl::VK_PAGEDOWN ||
                      vk == fabgl::VK_TILDE || vk == fabgl::VK_GRAVEACCENT ||
                      vk == fabgl::VK_VOLUMEUP || vk == fabgl::VK_VOLUMEDOWN ||
                      vk == fabgl::VK_VOLUMEMUTE ||
                      vk == fabgl::VK_DELETE || vk == fabgl::VK_BACKSPACE)) {
                    osdCenteredMsg(
                        Config::lang ? " Tecla no permitida " : " Key not allowed ",
                        LEVEL_WARN, 800);
                    continue;
                }

                // Conflict check
                bool conflict = false;
                for (int i = 0; i < Config::HK_COUNT; i++) {
                    if (i == idx) continue;
                    if (Config::hotkeys[i].vk == (uint16_t)vk &&
                        Config::hotkeys[i].alt  == alt &&
                        Config::hotkeys[i].ctrl == ctrl) {
                        conflict = true;
                        break;
                    }
                }
                if (conflict) {
                    osdCenteredMsg(
                        Config::lang ? " Ya asignado! " : " Already assigned! ",
                        LEVEL_WARN, 1000);
                    continue;
                }

                Config::hotkeys[idx] = { (uint16_t)vk, alt, ctrl };
                changed = true;
                break;
            }
        }

        // Restore background behind "Press key..." message
        VIDEO::SaveRect.restore_last();

        menu_curopt = opt;
        menu_saverect = false;
    }
}

void OSD::joyDialog(void) {

    int joyDropdown[14][7]={
        {7,65,-1,1,2,3,0}, // Left
        {67,65,0,-1,2,3,0}, // Right
        {37,17,-1,9,-1,0,0}, // Up
        {37,89,-1,6,0,4,0}, // Down
        {37,121,-1,5,3,-1,0}, // Start
        {121,121,4,12,6,-1,0}, // Mode
        {121,89,3,7,9,5,0}, // A
        {181,89,6,8,10,12,0}, // B
        {241,89,7,-1,11,13,0}, // C
        {121,17,2,10,-1,6,0}, // X
        {181,17,9,11,-1,7,0}, // Y
        {241,17,10,-1,-1,8,0}, // Z
        {181,121,5,13,7,-1,0}, // Ok
        {241,121,12,-1,8,-1,0} // Test
    };

    string keymenu = MENU_JOYSELKEY[Config::lang];
    int joytype = Config::joystick;

    string selkeymenu[5] = {
        MENU_JOY_AZ,
        MENU_JOY_09,
        "",
        MENU_JOY_PS2,
        "Joystick"
    };

    selkeymenu[2] = (Config::lang ? "Especial\n" : "Special\n");
    selkeymenu[2] += MENU_JOY_SPECIAL;
    selkeymenu[4] = MENU_JOY_KEMPSTON;
    int curDropDown = 2;
    uint8_t joyDialogMode = 0; // 0 -> Define, 1 -> Test

    const unsigned short h = (OSD_FONT_H * 18) + 2;
    const unsigned short y = scrAlignCenterY(h) - 8;

    const unsigned short w = (50 * OSD_FONT_W) + 2;
    const unsigned short x = scrAlignCenterX(w) - 3;

    // Set font
    VIDEO::vga.setFont(Font6x8);

    // Menu border
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print("Joystick");

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }

    // Read joy definition into joyDropdown
    for (int n = 0; n < 12; ++n)
        joyDropdown[n][6] = Config::joydef[n];

    // Draw Joy DropDowns
    for (int n = 0; n < 12; ++n) {
        VIDEO::vga.rect(x + joyDropdown[n][0] - 2, y + joyDropdown[n][1] - 2, 58, 12, zxColor(0, 0));
        if (n == curDropDown)
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        else
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        VIDEO::vga.setCursor(x + joyDropdown[n][0], y + joyDropdown[n][1]);
        VIDEO::vga.print(vkToText(joyDropdown[n][6]).c_str());
    }

    // Draw dialog buttons
    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyDropdown[12][0], y + joyDropdown[12][1]);
    VIDEO::vga.print("   Ok    ");
    VIDEO::vga.setCursor(x + joyDropdown[13][0], y + joyDropdown[13][1]);
    VIDEO::vga.print(" JoyTest ");
    DrawjoyControls(x, y);

    // Wait for key
    fabgl::VirtualKeyItem Nextkey;
    int joyTestExitCount = 0;
    while (1) {
        if (joyDialogMode) {
            DrawjoyControls(x,y);
        }
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if(!Nextkey.down) continue;
            if (is_left(Nextkey.vk)) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][2] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][2];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_right(Nextkey.vk)) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][3] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][3];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_up(Nextkey.vk)) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][4] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][4];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_down(Nextkey.vk)) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][5] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][5];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 12)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 12 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_enter(Nextkey.vk)) {
                if (joyDialogMode == 0) {
                    if (curDropDown>=0 && curDropDown<12) {
                        click();
                        // Launch assign menu
                        menu_curopt = 1;
                        while (1) {
                            menu_level = 0;
                            menu_saverect = true;
                            uint8_t opt = simpleMenuRun(keymenu,x + joyDropdown[curDropDown][0],y + joyDropdown[curDropDown][1],6,11);
                            if(opt!=0) {
                                // Key select menu
                                menu_saverect = true;
                                menu_level = 0;
                                menu_curopt = 1;
                                uint8_t opt2 = simpleMenuRun(selkeymenu[opt - 1],x + joyDropdown[curDropDown][0],y + joyDropdown[curDropDown][1],6,11);
                                if(opt2!=0) {
                                    if (opt == 1) {// A-Z
                                        joyDropdown[curDropDown][6] = (fabgl::VirtualKey) 47 + opt2;
                                    } else
                                    if (opt == 2) {// 0-9
                                        joyDropdown[curDropDown][6] = (fabgl::VirtualKey) 1 + opt2;
                                    } else
                                    if (opt == 3) {// Special
                                        if (opt2 == 1) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_RETURN;
                                        } else
                                        if (opt2 == 2) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_LSHIFT;
                                        } else
                                        if (opt2 == 3) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_LCTRL;
                                        } else
                                        if (opt2 == 4) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_SPACE;
                                        } else
                                        if (opt2 == 5) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_NONE;
                                        }
                                    } else
                                    if (opt == 4) {// PS/2
                                        if (opt2 < 13) {
                                            joyDropdown[curDropDown][6] = (fabgl::VirtualKey) 158 + opt2;
                                        } else
                                        if (opt2 == 13) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_PAUSE;
                                        } else
                                        if (opt2 == 14) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_PRINTSCREEN;
                                        } else
                                        if (opt2 == 15) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_LEFT;
                                        } else
                                        if (opt2 == 16) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_RIGHT;
                                        } else
                                        if (opt2 == 17) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_UP;
                                        } else
                                        if (opt2 == 18) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DOWN;
                                        }
                                    } else
                                    if (opt == 5) {// Kempston / Fuller
                                        if (opt2 == 1) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_LEFT;
                                        } else
                                        if (opt2 == 2) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_RIGHT;
                                        } else
                                        if (opt2 == 3) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_UP;
                                        } else
                                        if (opt2 == 4) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_DOWN;
                                        } else
                                        if (opt2 == 5) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_FIRE;
                                        } else
                                        if (opt2 == 6) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_ALTFIRE;
                                        } else
                                        if (opt2 == 7) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_START;
                                        } else
                                        if (opt2 == 8) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_SELECT;
                                        }
                                        if (joytype == JOY_FULLER)
                                            joyDropdown[curDropDown][6] += 6;
                                    }
                                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                                    VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                                    break;
                                }
                            } else break;
                            menu_curopt = opt;
                        }
                    } else
                    if (curDropDown == 12) {
                        // Ok button

                        // Check if there are changes and ask to save them
                        bool changed = false;
                        for (int n = 0; n < 12; ++n) {
                            if (Config::joydef[n] != joyDropdown[n][6]) {
                                changed = true;
                                break;
                            }
                        }
                        // Ask to save changes
                        if (changed) {
                            string title = "Joystick";
                            string msg = OSD_DLG_JOYSAVE[Config::lang];
                            uint8_t res = OSD::msgDialog(title,msg);
                            if (res == DLG_YES) {
                                for (int n = 0; n < 12; ++n) {
                                    Config::joydef[n] = joyDropdown[n][6];
                                }
                                Config::save();
                                click();
                                break;

                            } else
                            if (res == DLG_NO) {
                                click();
                                break;
                            }
                        } else {
                            click();
                            break;
                        }
                    } else
                    if (curDropDown == 13) {
                        // Enable joyTest
                        joyDialogMode = 1;
                        for (int n = 0; n < 12; ++n) {
                            Config::joydef[n] = joyDropdown[n][6];
                        }
                        Config::save(); /// TODO: revert support
                        joyTestExitCount = 0;
                        VIDEO::vga.setTextColor(zxColor(4, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + joyDropdown[13][0], y + joyDropdown[13][1]);
                        VIDEO::vga.print(" JoyTest ");
                        click();
                    }
                }
            } else
            if (Nextkey.vk == fabgl::VK_ESCAPE) {
                if (joyDialogMode) {
                    if (Nextkey.vk == fabgl::VK_ESCAPE) {
                        // Disable joyTest
                        joyDialogMode = 0;
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + joyDropdown[13][0], y + joyDropdown[13][1]);
                        VIDEO::vga.print(" JoyTest ");
                        for (int n = 0; n < 12; n++)
                            joyControl[n][2] = zxColor(0,0);
                        DrawjoyControls(x,y);
                        click();
                    }
                } else {
                    // Ask to discard changes
                    string title = "Joystick";
                    string msg = OSD_DLG_JOYDISCARD[Config::lang];
                    uint8_t res = OSD::msgDialog(title,msg);
                    if (res == DLG_YES) {
                        click();
                        break;
                    }
                }
            }
        }

        // Joy Test Mode: Check joy status and color controls
        if (joyDialogMode) {
            for (int n = fabgl::VK_JOY_RIGHT; n <= fabgl::VK_JOY_Z; n++) {
                int m = 0; // index in joyControl
                switch (n) {
                    case fabgl::VK_JOY_RIGHT: m = 1; break;
                    case fabgl::VK_JOY_LEFT: m = 0; break;
                    case fabgl::VK_JOY_DOWN: m = 3; break;
                    case fabgl::VK_JOY_UP: m = 2; break;
                    case fabgl::VK_JOY_A: m = 6; break;
                    case fabgl::VK_JOY_B: m = 7; break;
                    case fabgl::VK_JOY_START: m = 4; break;
                    case fabgl::VK_JOY_MODE: m = 5; break;
                    case fabgl::VK_JOY_C: m = 8; break;
                    case fabgl::VK_JOY_X: m = 9; break;
                    case fabgl::VK_JOY_Y: m = 10; break;
                    case fabgl::VK_JOY_Z: m = 11; break;
                }
                if (ESPectrum::PS2Controller.keyboard()->isVKDown((fabgl::VirtualKey) n))
                    joyControl[m][2] = zxColor(4,1);
                else
                    joyControl[m][2] = zxColor(0,0);
            }
            if (ESPectrum::PS2Controller.keyboard()->isVKDown(fabgl::VK_JOY_B)) {
                joyTestExitCount++;
                if (joyTestExitCount == 5)
                    ESPectrum::PS2Controller.keyboard()->injectVirtualKey(fabgl::VK_ESCAPE, true, false);
            } else {
                joyTestExitCount = 0;
            }
        }
        sleep_ms(50);
    }
}

// POKE DIALOG

#define DLG_OBJ_BUTTON 0
#define DLG_OBJ_INPUT 1
#define DLG_OBJ_COMBO 2

struct dlgObject {
    string Name;
    unsigned short int posx;
    unsigned short int posy;
    int objLeft;
    int objRight;
    int objTop;
    int objDown;
    unsigned char objType;
    string Label[2];
};

const dlgObject dlg_Objects[5] = {
    {"Bank",70,16,-1,-1, 4, 1, DLG_OBJ_COMBO , {"RAM Bank  ","Banco RAM "}},
    {"Address",70,32,-1,-1, 0, 2, DLG_OBJ_INPUT , {"Address   ","Direccion "}},
    {"Value",70,48,-1,-1, 1, 4, DLG_OBJ_INPUT , {"Value     ","Valor     "}},
    {"Ok",7,65,-1, 4, 2, 0, DLG_OBJ_BUTTON,  {"  Ok  "  ,"  Ok  "  }},
    {"Cancel",52,65, 3,-1, 2, 0, DLG_OBJ_BUTTON, {"  Cancel  "," Cancelar "}}
};

const string BankCombo[9] = { "   -   ", "   0   ", "   1   ", "   2   ", "   3   ", "   4   ", "   5   ", "   6   ", "   7   " };

void OSD::jumpToDialog() {
    uint32_t address = addressDialog(Z80::getRegPC(), Config::lang ? "Saltar a" : "Jump to");
    if (Z80::getRegPC() != address && address <= 0xFFFF) {
        Z80::setRegPC(address);
    }
}

void OSD::pokeDialog() {
    char tmp1[8];
    uint16_t address = Z80::getRegPC();
    snprintf(tmp1, 8, "%04X", address);
    char* tmp2 = tmp1 + 5;
    uint8_t page = address >> 14;
    snprintf(tmp2, 8, "%02X", MemESP::ramCurrent[page][address & 0x3fff]);

    string dlgValues[5] = {
        "   -   ", // Bank
        tmp1, // Address
        tmp2, // Value
        "",
        ""
    };

    string Bankmenu = (Config::lang ? " Banco \n" : " Bank  \n");
    int i=0;
    for (;i<9;i++) Bankmenu += BankCombo[i] + "\n";

    int curObject = 0;
    uint8_t dlgMode = 0; // 0 -> Move, 1 -> Input

    const unsigned short h = (OSD_FONT_H * 10) + 2;
    const unsigned short y = scrAlignCenterY(h) - 8;

    const unsigned short w = (OSD_FONT_W * 20) + 2;
    const unsigned short x = scrAlignCenterX(w) - 3;

    click();

    // Set font
    VIDEO::vga.setFont(Font6x8);

    // Menu border
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(Config::lang ? "A" "\xA4" "adir Poke" : "Input Poke");

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }

    // Draw objects
    for (int n = 0; n < 5; n++) {
        if (dlg_Objects[n].Label[Config::lang] != "" && dlg_Objects[n].objType != DLG_OBJ_BUTTON) {
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
            VIDEO::vga.setCursor(x + dlg_Objects[n].posx - 63, y + dlg_Objects[n].posy);
            VIDEO::vga.print(dlg_Objects[n].Label[Config::lang].c_str());
            VIDEO::vga.rect(x + dlg_Objects[n].posx - 2, y + dlg_Objects[n].posy - 2, 46, 12, zxColor(0, 0));
        }
        if (n == curObject)
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        else
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        VIDEO::vga.setCursor(x + dlg_Objects[n].posx, y + dlg_Objects[n].posy);
        if (dlg_Objects[n].objType == DLG_OBJ_BUTTON) {
            VIDEO::vga.print(dlg_Objects[n].Label[Config::lang].c_str());
        } else {
            VIDEO::vga.print(dlgValues[n].c_str());
        }
    }
    // Wait for key
    fabgl::VirtualKeyItem Nextkey;
    uint8_t CursorFlash = 0;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if(!Nextkey.down) continue;
            if ((Nextkey.vk >= fabgl::VK_0) && (Nextkey.vk <= fabgl::VK_9)) {
                if (dlg_Objects[curObject].objType == DLG_OBJ_INPUT) {
                    if (dlgValues[curObject].length() < (curObject == 1 ? 4 : 2)) {
                        dlgValues[curObject] += char(Nextkey.vk + 46);
                    }
                }
                click();
            } else
            if ((Nextkey.vk >= fabgl::VK_A) && (Nextkey.vk <= fabgl::VK_F)) {
                if (dlg_Objects[curObject].objType == DLG_OBJ_INPUT) {
                    if (dlgValues[curObject].length() < (curObject == 1 ? 4 : 2)) {
                        dlgValues[curObject] += char(Nextkey.vk - fabgl::VK_A) + 'A';
                    }
                }
                click();
            } else
            if (is_left(Nextkey.vk)) {
                if (dlg_Objects[curObject].objLeft >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                    if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    curObject = dlg_Objects[curObject].objLeft;
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                    if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    click();
                }
            } else
            if (is_right(Nextkey.vk)) {
                if (dlg_Objects[curObject].objRight >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                    if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    curObject = dlg_Objects[curObject].objRight;
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                    if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    click();
                }
            } else
            if (is_up(Nextkey.vk)) {
                if (dlg_Objects[curObject].objTop >= 0) {
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                        if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                            if (dlg_Objects[curObject].objType == DLG_OBJ_INPUT) VIDEO::vga.print(" "); // Clear K cursor
                        }
                        curObject = dlg_Objects[curObject].objTop;
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                        if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                        }
                        click();
                }
            } else
            if (is_down(Nextkey.vk)) {
                if (dlg_Objects[curObject].objDown >= 0) {
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                        if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                            if (dlg_Objects[curObject].objType == DLG_OBJ_INPUT) VIDEO::vga.print(" "); // Clear K cursor
                        }
                        curObject = dlg_Objects[curObject].objDown;
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                        if (dlg_Objects[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                        }
                        click();
                }
            } else
            if (Nextkey.vk == fabgl::VK_BACKSPACE) {
                if (dlg_Objects[curObject].objType == DLG_OBJ_INPUT) {
                    if (dlgValues[curObject] != "") dlgValues[curObject].pop_back();
                }
                click();
            } else
            if (is_enter(Nextkey.vk)) {
                if (dlg_Objects[curObject].Name == "Bank" && !Z80Ops::is48) {
                    click();
                    // Launch bank menu
                    menu_curopt = 1;
                    while (1) {
                        menu_level = 0;
                        menu_saverect = true;
                        uint8_t opt = simpleMenuRun( Bankmenu, x + dlg_Objects[curObject].posx,y + dlg_Objects[curObject].posy, 10, 9);
                        if(opt != 0) {
                            if (BankCombo[opt -1] != dlgValues[curObject]) {
                                dlgValues[curObject] = BankCombo[opt - 1];
                                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                                VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                                VIDEO::vga.print(dlgValues[curObject].c_str());

                                if (dlgValues[curObject]==BankCombo[0]) {
                                    dlgValues[1] = tmp1;
                                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                                    VIDEO::vga.setCursor(x + dlg_Objects[1].posx, y + dlg_Objects[1].posy);
                                    VIDEO::vga.print(tmp1);
                                } else {
                                    string val = dlgValues[1];
                                    trim(val);
                                    if(stoi(val) > 16383) {
                                        dlgValues[1] = tmp1;
                                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                                        VIDEO::vga.setCursor(x + dlg_Objects[1].posx, y + dlg_Objects[1].posy);
                                        VIDEO::vga.print(tmp1);
                                    }
                                }

                            }
                            break;
                        } else {
                            break;
                        }
                        menu_curopt = opt;
                    }
                } else
                if (dlg_Objects[curObject].Name == "Ok") {
                    string addr = dlgValues[1];
                    string val = dlgValues[2];
                    trim(addr);
                    trim(val);
                    address = stoul(addr, nullptr, 16);
                    uint8_t value = stoul(val, nullptr, 16);
                    // Apply poke
                    if (dlgValues[0] == "   -   ") {
                        // Poke address between 16384 and 65535
                        uint8_t page = address >> 14;
                        MemESP::ramCurrent[page][address & 0x3fff] = value;
                    } else {
                        // Poke address in bank
                        string bank = dlgValues[0];
                        trim(bank);
                        MemESP::ram[stoi(bank)].write(address, value);
                    }
                    click();
                    break;
                } else if (dlg_Objects[curObject].Name == "Cancel") {
                    click();
                    break;
                }
            } else if (is_back(Nextkey.vk)) {
                click();
                break;
            }
        }
        if (dlg_Objects[curObject].objType == DLG_OBJ_INPUT) {
            if ((++CursorFlash & 0xF) == 0) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                VIDEO::vga.setCursor(x + dlg_Objects[curObject].posx, y + dlg_Objects[curObject].posy);
                VIDEO::vga.print(dlgValues[curObject].c_str());
                if (CursorFlash > 63) {
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 1));
                    if (CursorFlash == 128) CursorFlash = 0;
                } else {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                }
                VIDEO::vga.print("K");
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                VIDEO::vga.print(" ");
            }
        }
        sleep_ms(5);
    }
    flushKbd();
}

void flushKbd() {
    auto kbd = ESPectrum::PS2Controller.keyboard();
    while (kbd->virtualKeyAvailable()) {
        fabgl::VirtualKeyItem dummy;
        kbd->getNextVirtualKey(&dummy);
    }
}

const dlgObject dlg_Objects2[3] = {
    {"Address",70,32,-1,-1, 0, 1, DLG_OBJ_INPUT , {"Address   ","Direccion "}},
    {"Ok",     7, 65, 2, 2, 0, 2, DLG_OBJ_BUTTON, {"  Ok  "  , "  Ok  "  }},
    {"Cancel", 52,65, 2, 2, 1, 0, DLG_OBJ_BUTTON, {"  Cancel  "," Cancelar "}}
};

uint32_t OSD::addressDialog(uint16_t addr, const char* title) {
    char tmp[8];
    snprintf(tmp, 8, "%04X", addr);
    string dlgValues[3]={
        tmp, // Address
        "",
        ""
    };

    int curObject = 0;
    uint8_t dlgMode = 0; // 0 -> Move, 1 -> Input

    const unsigned short h = (OSD_FONT_H * 10) + 2;
    const unsigned short y = scrAlignCenterY(h) - 8;

    const unsigned short w = (OSD_FONT_W * 20) + 2;
    const unsigned short x = scrAlignCenterX(w) - 3;

    click();

    // Set font
    VIDEO::vga.setFont(Font6x8);

    // Menu border
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(title);

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++) {
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        }
        rb_paint_x += 5;
    }

    // Draw objects
    for (int n = 0; n < 3; n++) {
        if (dlg_Objects2[n].Label[Config::lang] != "" && dlg_Objects2[n].objType != DLG_OBJ_BUTTON) {
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
            VIDEO::vga.setCursor(x + dlg_Objects2[n].posx - 63, y + dlg_Objects2[n].posy);
            VIDEO::vga.print(dlg_Objects2[n].Label[Config::lang].c_str());
            VIDEO::vga.rect(x + dlg_Objects2[n].posx - 2, y + dlg_Objects2[n].posy - 2, 46, 12, zxColor(0, 0));
        }
        if (n == curObject)
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        else
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));

        VIDEO::vga.setCursor(x + dlg_Objects2[n].posx, y + dlg_Objects2[n].posy);
        if (dlg_Objects2[n].objType == DLG_OBJ_BUTTON) {
            VIDEO::vga.print(dlg_Objects2[n].Label[Config::lang].c_str());
        } else {
            VIDEO::vga.print(dlgValues[n].c_str());
        }
    }
    // Wait for key
    fabgl::VirtualKeyItem Nextkey;
    uint8_t CursorFlash = 0;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if(!Nextkey.down) continue;
            if (Nextkey.vk >= fabgl::VK_0 && Nextkey.vk <= fabgl::VK_9) {
                if (dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) {
                    if (dlgValues[curObject].length() < 4) {
                        dlgValues[curObject] += char(Nextkey.vk + 46);
                    }
                }
                click();
            } else
            if (Nextkey.vk >= fabgl::VK_A && Nextkey.vk <= fabgl::VK_F) {
                if (dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) {
                    if (dlgValues[curObject].length() < 4) {
                        dlgValues[curObject] += char(Nextkey.vk - fabgl::VK_A) + 'A';
                    }
                }
                click();
            } else
            if (is_left(Nextkey.vk) || Nextkey.vk == fabgl::VK_TAB) {
                if (dlg_Objects2[curObject].objLeft >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                    if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    curObject = dlg_Objects2[curObject].objLeft;
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                    if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    click();
                }
            } else
            if (is_right(Nextkey.vk)) {
                if (dlg_Objects2[curObject].objRight >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                    if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    curObject = dlg_Objects2[curObject].objRight;
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                    if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                        VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                    } else {
                        VIDEO::vga.print(dlgValues[curObject].c_str());
                    }
                    click();
                }
            } else
            if (is_up(Nextkey.vk)) {
                if (dlg_Objects2[curObject].objTop >= 0) {
                    // Input values validation
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                        if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                            if (dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) VIDEO::vga.print(" "); // Clear K cursor
                        }
                        curObject = dlg_Objects2[curObject].objTop;
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                        if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                        }
                        click();
                }
            } else
            if (is_down(Nextkey.vk)) {
                if (dlg_Objects2[curObject].objDown >= 0) {
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                        if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                            if (dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) VIDEO::vga.print(" "); // Clear K cursor
                        }
                        curObject = dlg_Objects2[curObject].objDown;
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                        if (dlg_Objects2[curObject].objType == DLG_OBJ_BUTTON) {
                            VIDEO::vga.print(dlg_Objects2[curObject].Label[Config::lang].c_str());
                        } else {
                            VIDEO::vga.print(dlgValues[curObject].c_str());
                        }
                        click();
                }
            } else
            if (Nextkey.vk == fabgl::VK_BACKSPACE) {
                if (dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) {
                    if (dlgValues[curObject] != "") dlgValues[curObject].pop_back();
                }
                click();
            } else
            if (is_enter(Nextkey.vk) || Nextkey.vk == fabgl::VK_RETURN || Nextkey.vk == fabgl::VK_KP_ENTER) {
                if (dlg_Objects2[curObject].Name == "Ok" || dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) {
                    string s = dlgValues[0];
                    trim(s);
                    if (s.empty()) {
                        click();
                        flushKbd();
                        return 0x00010001;
                    }
                    addr = stoul(s, nullptr, 16);
                    click();
                    flushKbd();
                    return addr;
                } else if (dlg_Objects2[curObject].Name == "Cancel") {
                    click();
                    flushKbd();
                    return 0x00010001;
                }
            } else if (is_back(Nextkey.vk)) {
                click();
                flushKbd();
                return 0x00010000;
            }
        }

        if (dlg_Objects2[curObject].objType == DLG_OBJ_INPUT) {
            if ((++CursorFlash & 0xF) == 0) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                VIDEO::vga.setCursor(x + dlg_Objects2[curObject].posx, y + dlg_Objects2[curObject].posy);
                VIDEO::vga.print(dlgValues[curObject].c_str());
                if (CursorFlash > 63) {
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 1));
                    if (CursorFlash == 128) CursorFlash = 0;
                } else {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                }
                VIDEO::vga.print("K");
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                VIDEO::vga.print(" ");
            }
        }
        sleep_ms(5);
    }
    return 0x00010000;
}

uint16_t OSD::BPListDialog() {
    // Collect active breakpoint indices
    int bpIdx[Config::MAX_BREAKPOINTS];
    int count = 0;
    auto rebuild = [&]() {
        count = 0;
        for (int i = 0; i < Config::MAX_BREAKPOINTS; i++)
            if (Config::breakPoints[i].type != Config::BP_NONE)
                bpIdx[count++] = i;
        // Sort by type then address
        for (int i = 0; i < count - 1; i++)
            for (int j = i + 1; j < count; j++) {
                auto &a = Config::breakPoints[bpIdx[i]], &b = Config::breakPoints[bpIdx[j]];
                if (a.type > b.type || (a.type == b.type && a.addr > b.addr)) {
                    int t = bpIdx[i]; bpIdx[i] = bpIdx[j]; bpIdx[j] = t;
                }
            }
    };
    rebuild();
    if (count == 0) return 0xFFFF;

    int maxVisible = 10;
    int visible = count < maxVisible ? count : maxVisible;
    const unsigned short h = (visible + 2) * OSD_FONT_H + 4;
    const unsigned short w = OSD_FONT_W * 28 + 2;
    const unsigned short x = scrAlignCenterX(w);
    const unsigned short y = scrAlignCenterY(h);

    int cursor = 0;
    int scroll = 0;
    char buf[34];

    VIDEO::vga.setFont(Font6x8);
    VIDEO::SaveRect.save(x - 1, y - 1, w + 2, h + 2);

    while (1) {
        VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
        VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
        VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));

        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
        snprintf(buf, sizeof(buf), "BP(s) [%d/%d] DEL=rm", count, Config::MAX_BREAKPOINTS);
        VIDEO::vga.print(buf);

        for (int i = 0; i < visible; i++) {
            int idx = bpIdx[scroll + i];
            auto &bp = Config::breakPoints[idx];
            int yi = y + (i + 2) * OSD_FONT_H;
            if (i == cursor)
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
            else
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
            VIDEO::vga.setCursor(x + OSD_FONT_W, yi);
            if (bp.type == Config::BP_PC) {
                char mn[13];
                disasmAt(bp.addr, mn, 13);
                snprintf(buf, sizeof(buf), "#%02d %s %04X %-12s", scroll + i + 1, Config::bpTypeName(bp.type), bp.addr, mn);
            } else {
                snprintf(buf, sizeof(buf), "#%02d %s %04X             ", scroll + i + 1, Config::bpTypeName(bp.type), bp.addr);
            }
            buf[28] = 0;
            VIDEO::vga.print(buf);
        }

        auto Kbd = ESPectrum::PS2Controller.keyboard();
        while (!Kbd->virtualKeyAvailable()) sleep_ms(5);
        fabgl::VirtualKeyItem key;
        Kbd->getNextVirtualKey(&key);
        if (!key.down) continue;

        if (key.vk == fabgl::VK_UP) {
            if (cursor > 0) cursor--;
            else if (scroll > 0) scroll--;
        } else if (key.vk == fabgl::VK_DOWN) {
            if (cursor < visible - 1 && cursor < count - scroll - 1) cursor++;
            else if (scroll + visible < count) scroll++;
        } else if (key.vk == fabgl::VK_DELETE || key.vk == fabgl::VK_BACKSPACE) {
            Config::removeBreakPointAt(bpIdx[scroll + cursor]);
            rebuild();
            if (count == 0) break;
            visible = count < maxVisible ? count : maxVisible;
            if (scroll + cursor >= count) {
                if (cursor > 0) cursor--;
                else if (scroll > 0) scroll--;
            }
        } else if (key.vk == fabgl::VK_RETURN) {
            auto &bp = Config::breakPoints[bpIdx[scroll + cursor]];
            if (bp.type == Config::BP_PC) {
                Config::save();
                VIDEO::SaveRect.restore_last();
                return bp.addr;
            }
            break;
        } else if (key.vk == fabgl::VK_ESCAPE) {
            break;
        }
    }
    Config::save();
    VIDEO::SaveRect.restore_last();
    return 0xFFFF;
}

void OSD::BPDialog() {
    const char* items[] = { "PC address", "Port read", "Port write", "Mem write", "Mem read" };
    Config::BPType types[] = { Config::BP_PC, Config::BP_PORT_READ, Config::BP_PORT_WRITE, Config::BP_MEM_WRITE, Config::BP_MEM_READ };
    const char* titles[] = { "PC breakpoint", "Port read BP", "Port write BP", "Mem write BP", "Mem read BP" };
    const int nItems = 5;
    int sel = 0;

    const unsigned short h = (nItems + 2) * OSD_FONT_H + 4;
    const unsigned short w = OSD_FONT_W * 18 + 2;
    const unsigned short x = scrAlignCenterX(w);
    const unsigned short y = scrAlignCenterY(h);
    char buf[24];

    VIDEO::vga.setFont(Font6x8);
    VIDEO::SaveRect.save(x - 1, y - 1, w + 2, h + 2);

    while (1) {
        VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
        VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
        VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
        VIDEO::vga.print("Breakpoint type");

        for (int i = 0; i < nItems; i++) {
            int yi = y + (i + 2) * OSD_FONT_H;
            if (i == sel)
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
            else
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
            VIDEO::vga.setCursor(x + OSD_FONT_W, yi);
            snprintf(buf, 24, " %-15s", items[i]);
            buf[16] = 0;
            VIDEO::vga.print(buf);
        }

        auto Kbd = ESPectrum::PS2Controller.keyboard();
        while (!Kbd->virtualKeyAvailable()) sleep_ms(5);
        fabgl::VirtualKeyItem key;
        Kbd->getNextVirtualKey(&key);
        if (!key.down) continue;
        if (key.vk == fabgl::VK_UP) { if (sel > 0) sel--; }
        else if (key.vk == fabgl::VK_DOWN) { if (sel < nItems - 1) sel++; }
        else if (key.vk == fabgl::VK_RETURN || key.vk == fabgl::VK_KP_ENTER) {
            VIDEO::SaveRect.restore_last();
            uint32_t address = addressDialog(Z80::getRegPC(), titles[sel]);
            if (address == 0x00010000 || address == 0x00010001) return;
            Config::addBreakPoint(address, types[sel]);
            Config::save();
            return;
        }
        else if (key.vk == fabgl::VK_ESCAPE) {
            VIDEO::SaveRect.restore_last();
            return;
        }
    }
}

bool OSD::dumpRangeDialog(uint16_t &from, uint16_t &to) {
    char tmp0[8], tmp1[8];
    snprintf(tmp0, 8, "%04X", from);
    snprintf(tmp1, 8, "%04X", to);
    string vals[2] = { tmp0, tmp1 };
    const char* labels[2] = { "From ", "To   " };
    int curField = 0;
    uint8_t dlgMode = 0;

    const unsigned short h = (OSD_FONT_H * 8) + 2;
    const unsigned short w = (OSD_FONT_W * 20) + 2;
    const unsigned short x = scrAlignCenterX(w) - 3;
    const unsigned short y = scrAlignCenterY(h) - 8;

    click();
    VIDEO::vga.setFont(Font6x8);

    // Border & background
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print("Dump range");

    // Rainbow
    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++)
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        rb_paint_x += 5;
    }

    // Field positions
    int fx = x + 10 * OSD_FONT_W; // input x
    int fy[2] = { y + 24, y + 36 };
    // Button positions
    int by = y + 52;
    int bx_ok = x + 7, bx_cancel = x + 52;

    // Draw labels
    for (int f = 0; f < 2; f++) {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        VIDEO::vga.setCursor(x + OSD_FONT_W, fy[f]);
        VIDEO::vga.print(labels[f]);
        VIDEO::vga.rect(fx - 2, fy[f] - 2, 6 * OSD_FONT_W + 4, 12, zxColor(0, 0));
    }

    int curObject = 0; // 0=from, 1=to, 2=ok, 3=cancel
    int CursorFlash = 0;
    fabgl::VirtualKeyItem Nextkey;

    auto drawFields = [&]() {
        for (int f = 0; f < 2; f++) {
            if (f == curObject)
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
            else
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
            VIDEO::vga.setCursor(fx, fy[f]);
            VIDEO::vga.print(vals[f].c_str());
            VIDEO::vga.print(" ");
        }
        // Buttons
        VIDEO::vga.setCursor(bx_ok, by);
        VIDEO::vga.setTextColor(curObject == 2 ? zxColor(0, 1) : zxColor(0, 1),
                                curObject == 2 ? zxColor(5, 1) : zxColor(7, 1));
        VIDEO::vga.print("  Ok  ");
        VIDEO::vga.setCursor(bx_cancel, by);
        VIDEO::vga.setTextColor(curObject == 3 ? zxColor(0, 1) : zxColor(0, 1),
                                curObject == 3 ? zxColor(5, 1) : zxColor(7, 1));
        VIDEO::vga.print(Config::lang ? " Cancelar " : "  Cancel  ");
    };

    drawFields();

    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if (!Nextkey.down) continue;

            if (Nextkey.vk >= fabgl::VK_0 && Nextkey.vk <= fabgl::VK_9) {
                if (curObject < 2 && vals[curObject].length() < 4) {
                    vals[curObject] += char(Nextkey.vk + 46);
                    drawFields();
                }
            } else if (Nextkey.vk >= fabgl::VK_A && Nextkey.vk <= fabgl::VK_F) {
                if (curObject < 2 && vals[curObject].length() < 4) {
                    vals[curObject] += char('A' + (Nextkey.vk - fabgl::VK_A));
                    drawFields();
                }
            } else if (Nextkey.vk == fabgl::VK_BACKSPACE) {
                if (curObject < 2 && vals[curObject].length() > 0) {
                    vals[curObject].pop_back();
                    drawFields();
                }
            } else if (Nextkey.vk == fabgl::VK_DOWN || Nextkey.vk == fabgl::VK_TAB) {
                curObject = (curObject + 1) % 4;
                CursorFlash = 0;
                drawFields();
            } else if (Nextkey.vk == fabgl::VK_UP) {
                curObject = (curObject + 3) % 4;
                CursorFlash = 0;
                drawFields();
            } else if (Nextkey.vk == fabgl::VK_LEFT) {
                if (curObject >= 2) { curObject = (curObject == 2) ? 3 : 2; drawFields(); }
            } else if (Nextkey.vk == fabgl::VK_RIGHT) {
                if (curObject >= 2) { curObject = (curObject == 2) ? 3 : 2; drawFields(); }
            } else if (Nextkey.vk == fabgl::VK_RETURN || Nextkey.vk == fabgl::VK_KP_ENTER) {
                if (curObject == 3) { click(); return false; } // Cancel
                if (curObject == 2 || curObject < 2) {
                    // Ok or Enter on field
                    string s0 = vals[0], s1 = vals[1];
                    while (s0.length() < 4) s0 = "0" + s0;
                    while (s1.length() < 4) s1 = "0" + s1;
                    from = stoul(s0, nullptr, 16);
                    to = stoul(s1, nullptr, 16);
                    click();
                    return true;
                }
            } else if (Nextkey.vk == fabgl::VK_ESCAPE) {
                click();
                return false;
            }
        }
        // Cursor blink
        if (curObject < 2) {
            if ((++CursorFlash & 0xF) == 0) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                VIDEO::vga.setCursor(fx, fy[curObject]);
                VIDEO::vga.print(vals[curObject].c_str());
                if (CursorFlash > 63)
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 1));
                else
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                VIDEO::vga.print("K");
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                VIDEO::vga.print(" ");
                if (CursorFlash == 128) CursorFlash = 0;
            }
        }
        sleep_ms(5);
    }
}

// Search memory for memSearchHex pattern starting at startAddr, return found address or 0x10000
static uint32_t memDoSearch(uint16_t startAddr) {
    int len = memSearchHex.length();
    if (len < 2 || (len & 1)) return 0x10000;
    int nBytes = len / 2;
    uint8_t pattern[8];
    if (nBytes > 8) nBytes = 8;
    for (int i = 0; i < nBytes; i++) {
        char hi = memSearchHex[i * 2], lo = memSearchHex[i * 2 + 1];
        auto hexVal = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        int hv = hexVal(hi), lv = hexVal(lo);
        if (hv < 0 || lv < 0) return 0x10000;
        pattern[i] = (hv << 4) | lv;
    }
    for (uint32_t off = 0; off < 0x10000; off++) {
        uint16_t addr = (startAddr + off) & 0xFFFF;
        bool match = true;
        for (int i = 0; i < nBytes; i++) {
            if (MemESP::readbyte((addr + i) & 0xFFFF) != pattern[i]) { match = false; break; }
        }
        if (match) {
            memSearchLastFound = addr;
            return addr;
        }
    }
    return 0x10000;
}

void OSD::memSearchDialog() {

    const unsigned short h = (OSD_FONT_H * 6) + 2;
    const unsigned short w = (OSD_FONT_W * 22) + 2;
    const unsigned short x = scrAlignCenterX(w) - 3;
    const unsigned short y = scrAlignCenterY(h) - 8;

    click();
    VIDEO::vga.setFont(Font6x8);

    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));

    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print("Search memory");

    unsigned short rb_y = y + 8;
    unsigned short rb_paint_x = x + w - 30;
    uint8_t rb_colors[] = {2, 6, 4, 5};
    for (uint8_t c = 0; c < 4; c++) {
        for (uint8_t i = 0; i < 5; i++)
            VIDEO::vga.line(rb_paint_x + i, rb_y, rb_paint_x + 8 + i, rb_y - 8, zxColor(rb_colors[c], 1));
        rb_paint_x += 5;
    }

    int iy = y + 22;
    int ry = y + 34;
    int maxHexChars = 16;
    char buf[32];
    int CursorFlash = 0;
    memSearchResultAddr = 0x10000;

    auto drawInput = [&]() {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        VIDEO::vga.setCursor(x + OSD_FONT_W, iy);
        VIDEO::vga.print("Hex:");
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        VIDEO::vga.setCursor(x + 5 * OSD_FONT_W, iy);
        snprintf(buf, 32, "%-16s", memSearchHex.c_str());
        buf[16] = 0;
        VIDEO::vga.print(buf);
    };

    auto drawResult = [&](const char* msg) {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        VIDEO::vga.setCursor(x + OSD_FONT_W, ry);
        snprintf(buf, 32, "%-20s", msg);
        buf[20] = 0;
        VIDEO::vga.print(buf);
    };

    auto doSearch = [&](uint16_t startAddr) {
        uint32_t result = memDoSearch(startAddr);
        memSearchResultAddr = result;
        if (result <= 0xFFFF) {
            snprintf(buf, 32, "Found at %04X", (uint16_t)result);
            drawResult(buf);
        } else {
            int len = memSearchHex.length();
            if (len < 2 || (len & 1))
                drawResult("Need even hex digits");
            else
                drawResult("Not found");
        }
    };

    drawInput();
    drawResult("Enter hex, F3=next");

    fabgl::VirtualKeyItem Nextkey;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if (!Nextkey.down) continue;

            if (Nextkey.vk >= fabgl::VK_0 && Nextkey.vk <= fabgl::VK_9) {
                if ((int)memSearchHex.length() < maxHexChars) {
                    memSearchHex += char(Nextkey.vk + 46);
                    drawInput();
                }
            } else if (Nextkey.vk >= fabgl::VK_A && Nextkey.vk <= fabgl::VK_F) {
                if ((int)memSearchHex.length() < maxHexChars) {
                    memSearchHex += char('A' + (Nextkey.vk - fabgl::VK_A));
                    drawInput();
                }
            } else if (Nextkey.vk == fabgl::VK_BACKSPACE) {
                if (memSearchHex.length() > 0) {
                    memSearchHex.pop_back();
                    drawInput();
                }
            } else if (Nextkey.vk == fabgl::VK_RETURN || Nextkey.vk == fabgl::VK_KP_ENTER) {
                doSearch(0);
            } else if (Nextkey.vk == fabgl::VK_F3) {
                doSearch((memSearchLastFound + 1) & 0xFFFF);
            } else if (Nextkey.vk == fabgl::VK_ESCAPE) {
                break;
            }
        }
        if ((++CursorFlash & 0xF) == 0) {
            int cx = x + 5 * OSD_FONT_W + (int)memSearchHex.length() * OSD_FONT_W;
            if (CursorFlash > 63)
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 1));
            else
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
            VIDEO::vga.setCursor(cx, iy);
            VIDEO::vga.print("K");
            if (CursorFlash == 128) CursorFlash = 0;
        }
        sleep_ms(5);
    }
}

