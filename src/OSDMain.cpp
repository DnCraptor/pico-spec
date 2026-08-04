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
#include <hardware/uart.h>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/vreg.h>
#include <pico/bootrom.h>
#include <pico/multicore.h>

#include "OSDMain.h"
#include "FileUtils.h"
#include "Subsystem.h"
#include "CPU.h"
#include "Video.h"
#include "Z80DMA.h"
#include "ESPectrum.h"
#include "messages.h"
#include "Config.h"
#include "Debug.h"
#include "Snapshot.h"
#include "MemESP.h"
#include "AlfCart.h"
#include "Buffer.h"
#include "Tape.h"
#include "LEDIndicators.h"
#include "sdcard.h"
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
#include "dls_conv.h"
#include "SoftSynth.h"
#include "ZiFi.h"
#include "ZiFiAT.h"
#include "UsbMsc.h"
#include "BoardPins.h"
#if ZIFI_NET_CLIENT
#include "ZiFiSock.h"
#include "RemoteFs.h"
#include "Ftp.h"
#include "Sftp.h"
#include "Ssh.h"
#include "HttpCatalogFs.h"
#include "HttpsGet.h"
#include "Ftpd.h"
#include "HttpCatalogFs.h"
#include "HttpsGet.h"
#endif
#include "kbd_img.h"

#if !PICO_RP2040
// GM.DLS on-device .dls -> bank conversion progress: throttle OSD redraws to whole
// percent steps so the long (~tens of s) encode shows life without flickering.
static int s_dlsConvLastPct = -1;
static void osdDlsConvProgress(int pct, void* /*user*/) {
    if (pct == s_dlsConvLastPct) return;
    s_dlsConvLastPct = pct;
    OSD::osdCenteredMsg(string(MSG_MIDI_CONVERTING[Config::lang]) + " " + std::to_string(pct) + "%",
                        LEVEL_INFO, 0);
}

// Convert a .dls (full SD path) to a GMWB bank named <stem>.bin in CONFIG_DIR, with
// on-screen progress. Returns the bank path on success, or "" on failure / too-large-
// for-flash (shows the matching message itself). Does NOT touch Config — the caller
// selects the bank + drives provisioning. Shared by the MIDI menu and the F5 browser.
static string osdConvertDlsToBank(const string& dlsPath) {
    string base = dlsPath;
    size_t slash = base.find_last_of('/');
    if (slash != string::npos) base.erase(0, slash + 1);   // basename
    size_t dot = base.rfind('.');
    if (dot != string::npos) base.erase(dot);              // strip extension
    string outBin = string(CONFIG_DIR) + "/" + base + ".bin";

    s_dlsConvLastPct = -1;
    OSD::osdCenteredMsg(string(MSG_MIDI_CONVERTING[Config::lang]) + " 0%", LEVEL_INFO, 0);
    if (!DlsConv::convert(dlsPath.c_str(), outBin.c_str(), 31250, osdDlsConvProgress, nullptr)) {
        OSD::osdCenteredMsg(MSG_MIDI_CONVERT_FAIL[Config::lang], LEVEL_WARN, 3000);
        return "";
    }
    // A bank larger than the flash partition is written to SD but can never be
    // installed (scanBanks/provision reject it) — delete it and warn, don't select it.
    size_t bankSz = 0;
    FIL* bf = fopen2(outBin.c_str(), FA_READ);
    if (bf) { bankSz = (size_t)f_size(bf); fclose2(bf); }
    size_t cap = MidiSynth::flashBankCapacity();
    if (bankSz > cap) {
        f_unlink(outBin.c_str());
        OSD::osdCenteredMsg(string(MSG_MIDI_BANK_TOOBIG[Config::lang]) + " (" +
                            std::to_string(bankSz >> 10) + " > " + std::to_string(cap >> 10) + " KB)",
                            LEVEL_WARN, 4000);
        return "";
    }
    OSD::osdCenteredMsg(MSG_MIDI_CONVERT_OK[Config::lang], LEVEL_OK, 1500);
    return outBin;
}
#endif

extern "C" void graphics_set_scanlines(uint8_t level);
extern "C" void graphics_set_dither(bool enabled);
#if !PICO_RP2040
extern "C" volatile bool profi_ds80_active;
extern "C" const uint32_t profi_default_palette16[16];
#endif
#if !PICO_RP2040
#include "DivMMC.h"
#include "IDE.h"
#include "MB02.h"
#include "GS/GS.h"
#endif

#include <malloc.h>

#include "PinSerialData_595.h"

#include <string>
#include <cstdio>

#if USE_NESPAD
#include "nespad.h"
#else
static const uint32_t nespad_state = 0, nespad_state2 = 0;
#endif
struct input_bits_t {
  bool a: true;
  bool b: true;
  bool select: true;
  bool start: true;
  bool right: true;
  bool left: true;
  bool up: true;
  bool down: true;
};
extern input_bits_t gamepad1_bits;
extern input_bits_t gamepad2_bits;

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
extern Font Font6x8Cyr;   // CP1251 Cyrillic face (online-catalog names)
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
extern bool SELECT_VGA;
#endif

extern int ram_pages, butter_pages, psram_pages, swap_pages;

// Shared buffer for HWInfo/ChipInfo/BoardInfo/EmulatorInfo (never called concurrently).
// RP2040 keeps only HWInfo (Alt+F1) — the Hardware menu (and its other, larger info
// screens) is removed there — and HWInfo fills only ~500 B, so a 640 B buffer suffices
// and saves ~900 B of the tight SRAM. RP2350 keeps 1536 for the bigger screens.
#if PICO_RP2040
#define OSD_INFO_BUF_SZ 640
#else
#define OSD_INFO_BUF_SZ 1536
#endif
static char osd_info_buf[OSD_INFO_BUF_SZ] __attribute__((aligned(4)));

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
bool OSD::net_launch_close = false;
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
// US-layout shifted form of a symbol/digit. map_key() returns only unshifted symbol
// VKs (e.g. VK_MINUS), so Shift+key arrives as the base char — translate it here so
// '_', '!', '+', '?', etc. can be typed. Letters are handled separately (case).
static char shiftSymUS(char c) {
    switch (c) {
        case '-': return '_'; case '=': return '+';
        case '[': return '{'; case ']': return '}'; case '\\': return '|';
        case ';': return ':'; case '\'': return '"'; case '`': return '~';
        case ',': return '<'; case '.': return '>'; case '/': return '?';
        case '1': return '!'; case '2': return '@'; case '3': return '#';
        case '4': return '$'; case '5': return '%'; case '6': return '^';
        case '7': return '&'; case '8': return '*'; case '9': return '('; case '0': return ')';
        default:  return c;   // already shifted, or a non-shiftable char
    }
}

string OSD::inlineTextEdit(int ex, int ey, int maxlen, const string& initial_text, bool mask, int viscols) {
    if (viscols <= 0 || viscols > maxlen) viscols = maxlen; // visible window
    string text = initial_text;
    bool reveal = false; // password mask: false → show '*', toggled with TAB
    auto Kbd = ESPectrum::PS2Controller.keyboard();
    // Drain any keys still in the queue (e.g. the Enter that triggered the save action)
    { fabgl::VirtualKeyItem drain; while (Kbd->virtualKeyAvailable()) Kbd->getNextVirtualKey(&drain); }
    VIDEO::vga.setFont(Font6x8);

    uint8_t blinkCtr = 7; // triggers first draw immediately (++&0x7==0)

    auto redraw = [&](bool cursorOn) {
        int cur = (int)text.length();
        bool full = (cur >= maxlen);
        // Scroll so the cursor cell stays inside the visible window.
        int scroll = (cur >= viscols) ? (cur - viscols + 1) : 0;
        if (scroll > maxlen - viscols) scroll = maxlen - viscols;
        for (int p = 0; p < viscols; p++) {
            int idx = scroll + p;
            bool isCursor = (!full && idx == cur) || (full && p == viscols - 1);
            if (isCursor && cursorOn)
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 1));
            else
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
            VIDEO::vga.setCursor(ex + p * OSD_FONT_W, ey);
            // Masked password: render '*' for the typed characters until revealed.
            char dch = (idx < cur) ? text[idx] : ' ';
            if (mask && !reveal && idx < cur) dch = '*';
            char ch[2] = { dch, 0 };
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
        // TAB toggles password reveal (only relevant in masked fields).
        if (ek.vk == fabgl::VK_TAB) { if (mask) { reveal = !reveal; redraw(true); } continue; }
        if (ek.vk == fabgl::VK_RETURN || ek.vk == fabgl::VK_KP_ENTER) return text;
        if (ek.vk == fabgl::VK_ESCAPE) return "\x1B";
        if (ek.vk == fabgl::VK_BACKSPACE) {
            if (!text.empty()) { text.pop_back(); redraw(true); }
        } else if (ek.ASCII >= 32 && ek.ASCII < 127) {
            if ((int)text.length() < maxlen) {
                char c = ek.ASCII;
                bool shift = Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT);
                if (c >= 'A' && c <= 'Z') {
                    bool caps = Kbd->isVKDown(fabgl::VK_CAPSLOCK);
                    if (!shift && !caps) c = c - 'A' + 'a';
                } else if (shift) {
                    c = shiftSymUS(c);   // '-'→'_', '1'→'!', … (map_key gives no shift variant)
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
    Debug::log("esp_hard_reset called from %p", __builtin_return_address(0));
    if (Config::audio_driver == 3) send_to_595(LOW(AY_Enable));
    close_all();
    Debug::log("ehr: close_all done, arming watchdog");
#if defined(DBG_UART_ENABLED) && defined(PICO_DEFAULT_UART)
    uart_tx_wait_blocking(uart_default);   // drain FIFO — 32B @115200 ≈ 3ms > watchdog delay
#endif
    watchdog_enable(1, true);
    while (true);
}

static bool confirmReboot(const char* const dlg[2]) {
    return OSD::msgDialog("", dlg[Config::lang]) == DLG_YES;
}

#if !PICO_RP2040
// SRAM budget gate. See OSDMain.h. Asks Subsystems::budgetCheck whether `f` fits;
// if not, either refuses (DENY) or pops up the freeable features so the user can
// turn some off → Config + reboot. Returns true only when the caller may proceed
// to enable `f` itself (it fits live); on a freeing reboot this never returns.
bool OSD::featureBudgetGate(int featureId) {
    using namespace Subsystems;
    FeatureId f = (FeatureId)featureId;

    FeatureId cand[FEAT_COUNT];
    int nCand = 0;
    size_t deficit = 0;
    BudgetResult r = budgetCheck(f, cand, &nCand, &deficit);

    if (r == BUDGET_ALLOW) return true;

    // Enough total SRAM, just not in one block: nothing to give up, only a reboot to
    // take (setup() allocates from an unfragmented heap). Persist the enable and go —
    // declining leaves the feature off, exactly like Esc out of the free-list.
    if (r == BUDGET_NEEDS_REBOOT) {
        const string dlg = (string)featureName(f) + ": " + OSD_DLG_APPLYREBOOT[Config::lang];
        if (OSD::msgDialog("", dlg.c_str()) != DLG_YES) return false;
        featureSetEnabled(f, true);
        Config::save();
        esp_hard_reset();   // never returns
    }

    if (r == BUDGET_DENY || nCand == 0) {
        osdCenteredMsg((string)featureName(f) + ":\n" + MSG_BUDGET_DENY[Config::lang],
                       LEVEL_WARN, 4000);
        return false;
    }

    // BUDGET_NEEDS_FREE: multi-select popup of the freeable features. Run it as a
    // CHILD dialog (menu_level+1) so it uses its own prev_y slot — at the caller's
    // level its menu_saverect draw would overwrite prev_y[level] with the popup's
    // (lower) Y, and the parent menu's next saverect=false redraw (y=prev_y[level])
    // would then draw shifted down, compounding each cycle.
    int savedMenuLevel = menu_level;
    if (menu_level < 4) menu_level++;
    bool sel[FEAT_COUNT] = { false };
    menu_saverect = true;
    menu_curopt = 1;   // start focus at row 1 — the caller's menu_curopt (e.g. 5 for
                       // GM.DLS, the 5th MIDI row) would be out of range for this small
                       // popup → "Unknown menu row" + corrupted geometry/SaveRect → hang.
    bool result = false;
    while (1) {
        size_t freed = 0;
        for (int i = 0; i < nCand; i++) if (sel[i]) freed += featureCost(cand[i]);

        string menu = (string)MSG_BUDGET_FREE_HINT[Config::lang] + "\n";
        for (int i = 0; i < nCand; i++) {
            char row[48];
            snprintf(row, sizeof(row), "%s (%uK)\t[%c]\n",
                     featureName(cand[i]), (unsigned)((featureCost(cand[i]) + 1023) / 1024),
                     sel[i] ? '*' : ' ');
            menu += row;
        }
        menu += (string)MSG_BUDGET_APPLY[Config::lang] + "\t\n";

        char foot[40];
        snprintf(foot, sizeof(foot), "free %uK / need %uK",
                 (unsigned)(freed / 1024), (unsigned)((deficit + 1023) / 1024));
        menu_footer = foot;

        uint8_t opt = menuRun(menu);
        if (opt == 0) { result = false; break; }            // Esc → not enabled
        if (opt <= nCand) {                                  // toggle a candidate
            sel[opt - 1] = !sel[opt - 1];
            menu_curopt = opt;
            menu_saverect = false;
            continue;
        }
        // Apply & reboot row.
        if (freed < deficit) {
            osdCenteredMsg(MSG_BUDGET_INSUFFICIENT[Config::lang], LEVEL_WARN, 2500);
            menu_curopt = opt;
            menu_saverect = false;
            continue;
        }
        for (int i = 0; i < nCand; i++) if (sel[i]) featureSetEnabled(cand[i], false);
        featureSetEnabled(f, true);
        Config::save();
        esp_hard_reset();   // never returns
        result = true; break;
    }
    // NOTE: do NOT restore_last() here — menuRun() already pops the popup's saved rect
    // on its Esc/back path (OSDMenu.cpp, is_back → SaveRect.restore_last when level!=0).
    // A manual restore here is a double-pop → SaveRect stack underflow → shifted menus
    // and eventually a wild-pointer SIGBUS.
    menu_level = savedMenuLevel;   // restore parent level (popup ran one level deeper)
    menu_saverect = false;
    return result;
}
#endif

#if !PICO_RP2040
// Centred dialog to type a WiFi password (title = SSID). Saves/restores its own
// screen rect. Returns the typed text, or "\x1B" if the user pressed Esc.
static string wifiAskPassword(const string& ssid) {
    const int field = 32; // visible password length
    const int label = strlen(MSG_WIFI_PASS_LABEL[Config::lang]);
    const unsigned short cols = label + field + 2;
    const unsigned short w = (cols * OSD_FONT_W) + 2;
    const unsigned short h = (OSD_FONT_H * 2) + 3;
    const unsigned short x = OSD::scrAlignCenterX(w);
    const unsigned short y = OSD::scrAlignCenterY(h);
    VIDEO::SaveRect.save(x, y, w, h);
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));                       // title bar
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));  // body
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(ssid.substr(0, cols - 2).c_str());
    // TAB-reveal hint, right-aligned in the title bar.
    const char* hint = MSG_PASS_TAB[Config::lang];
    VIDEO::vga.setCursor(x + w - 1 - (int)(strlen(hint) + 1) * OSD_FONT_W, y + 1);
    VIDEO::vga.print(hint);
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    VIDEO::vga.setCursor(x + OSD_FONT_W, y + 1 + OSD_FONT_H + 1);
    VIDEO::vga.print(MSG_WIFI_PASS_LABEL[Config::lang]);
    string pass = OSD::inlineTextEdit(x + OSD_FONT_W * (label + 1), y + 1 + OSD_FONT_H + 1, field, "", true);
    VIDEO::SaveRect.restore_last();
    return pass;
}

// ── Shared on-screen line log (WiFi connect window + FTP server terminal) ────
// Scrolling line ring: fixed char[][] (no heap churn), holding the most recent
// lines; every renderer shows the tail (auto-scroll). Deliberately OUTSIDE
// #if ZIFI_NET_CLIENT — the Network menu's WiFi scan/connect flow logs the live
// ESP-01 exchange through it in builds without the FTP/SFTP/SSH client too.
#define FTPD_LOG_LINES 40
#define FTPD_LOG_COLS  72
// Lazy-heaped (2880 B) — costs 0 SRAM until the FTP server / WiFi-connect window
// actually logs a line, so it never starves the razor-thin Profi heap at
// VIDEO::Init (Profi forces ~80 KB of SRAM pages before the framebuffer alloc).
// Readers are gated by `li < ftpd_log_count`, and ftpd_log_count only advances
// after a successful alloc here, so they never touch a null buffer.
static char (*ftpd_log)[FTPD_LOG_COLS] = nullptr;
extern "C" size_t getLargestAllocatable(void);   // defined at the bottom of this file
static int  ftpd_log_count = 0;  // total lines pushed (monotonic)
static bool ftpd_log_dirty = true;
static void ftpdLogLine(const char* s) {
    if (!ftpd_log) {
        // Gate, don't null-check: pico_malloc wraps calloc too and PANICs on OOM
        // instead of returning NULL, so "drop the line rather than crash" only
        // works if we never make an ask that can fail (see Buffer::palloc).
        if (getLargestAllocatable() < FTPD_LOG_LINES * FTPD_LOG_COLS + 8192) return;
        ftpd_log = (char(*)[FTPD_LOG_COLS])calloc(FTPD_LOG_LINES, FTPD_LOG_COLS);
        if (!ftpd_log) return;   // OOM — drop the line rather than crash
    }
    int slot = ftpd_log_count % FTPD_LOG_LINES;
    strncpy(ftpd_log[slot], s ? s : "", FTPD_LOG_COLS - 1);
    ftpd_log[slot][FTPD_LOG_COLS - 1] = '\0';
    ftpd_log_count++;
    ftpd_log_dirty = true;
}

// ── Live ESP-01 AT-log window (shown during a blocking WiFi connect) ──────────
// ZiFiAT::log_cb → wifiLogLine appends each tx/rx line to the shared ftpd_log ring
// and redraws the tail immediately, so the ESP-01 exchange scrolls live while
// connect() blocks. Reuses ftpd_log[] (a generic line ring) — the WiFi connect
// window and the FTP server are never up at the same time.
static string s_wifilog_sub;
static void wifiLogDraw() {
    OSD::drawOSD(true);
    int visCols = OSD::osdMaxCols(); if (visCols > 40) visCols = 40;
    int visRows = OSD::osdMaxRows() - 4;
    char row[42];
    OSD::osdAt(1, 0);                                   // subtitle: "Connecting <ssid>"
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
    snprintf(row, sizeof(row), " %s", s_wifilog_sub.c_str());
    int hl = strlen(row); while (hl < visCols) row[hl++] = ' '; row[visCols] = '\0';
    VIDEO::vga.print(row);
    OSD::osdAt(2, 0);                                   // separator
    VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(1, 0));
    memset(row, '-', visCols); row[0] = ' '; row[visCols] = '\0';
    VIDEO::vga.print(row);
    int first = ftpd_log_count > visRows ? ftpd_log_count - visRows : 0; // tail
    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
    for (int r = 0; r < visRows; r++) {
        OSD::osdAt(3 + r, 0);
        int li = first + r;
        if (li < ftpd_log_count) {
            const char* s = ftpd_log[li % FTPD_LOG_LINES];
            int len = strlen(s); if (len > visCols) len = visCols;
            memcpy(row, s, len); memset(row + len, ' ', visCols - len);
        } else memset(row, ' ', visCols);
        row[visCols] = '\0';
        VIDEO::vga.print(row);
    }
}
static void wifiLogLine(const char* s) { ftpdLogLine(s); wifiLogDraw(); }
// Append a final line (e.g. an error) and hold the window open until the user
// presses a key — so a failure stays readable instead of auto-closing.
static void wifiLogHold(const char* msg) {
    if (msg && msg[0]) ftpdLogLine(msg);
    ftpdLogLine(Config::lang ? " -- pulsa una tecla --" : " -- press a key --");
    wifiLogDraw();
    auto* kb = ESPectrum::PS2Controller.keyboard();
    while (kb->virtualKeyAvailable()) { fabgl::VirtualKeyItem k; kb->getNextVirtualKey(&k); } // drain
    for (;;) {
        if (kb->virtualKeyAvailable()) {
            fabgl::VirtualKeyItem k;
            if (kb->getNextVirtualKey(&k) && k.down) break;
        }
        sleep_ms(5);
    }
}

#if ZIFI_NET_CLIENT
// ─── Network file-transfer client (FTP / SFTP) ──────────────────────────────
// A centred single-line text-entry dialog (host / user / port). Returns the
// typed text, or "\x1B" if Esc was pressed. Generalises wifiAskPassword().
static string netAskField(const string& label, const string& initial, bool mask = false, int field = 30, int maxlen = 0) {
    if (maxlen <= 0 || maxlen < field) maxlen = field; // input cap (>= visible width)
    const int lbl = (int)label.size();
    const unsigned short cols = lbl + field + 2;
    const unsigned short w = (cols * OSD_FONT_W) + 2;
    const unsigned short h = (OSD_FONT_H * 2) + 3;
    const unsigned short x = OSD::scrAlignCenterX(w);
    const unsigned short y = OSD::scrAlignCenterY(h);
    VIDEO::SaveRect.save(x, y, w, h);
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(label.c_str());
    if (mask) { // TAB-reveal hint, right-aligned in the title bar
        const char* hint = MSG_PASS_TAB[Config::lang];
        VIDEO::vga.setCursor(x + w - 1 - (int)(strlen(hint) + 1) * OSD_FONT_W, y + 1);
        VIDEO::vga.print(hint);
    }
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
    string r = OSD::inlineTextEdit(x + OSD_FONT_W, y + 1 + OSD_FONT_H + 1, maxlen, initial, mask, field);
    VIDEO::SaveRect.restore_last();
    return r;
}

#define NET_KNOWN_HOSTS CONFIG_DIR "/known_hosts"

// Look up a stored SHA-256 fingerprint for `host` (returns "" if unknown).
static string knownHostsLookup(const string& host) {
    FIL* f = fopen2(NET_KNOWN_HOSTS, FA_READ);
    if (!f) return "";
    string line, result;
    UINT br; char c;
    while (!f_eof(f)) {
        if (f_read(f, &c, 1, &br) != FR_OK || br == 0) break;
        if (c == '\n') {
            size_t sp = line.find(' ');
            if (sp != string::npos && line.substr(0, sp) == host) { result = line.substr(sp + 1); break; }
            line.clear();
        } else if (c != '\r') line += c;
    }
    fclose2(f);
    return result;
}

static void knownHostsSave(const string& host, const string& fp) {
    FileUtils::mkdirParents(CONFIG_DIR);
    FIL* f = fopen2(NET_KNOWN_HOSTS, FA_OPEN_APPEND | FA_WRITE);
    if (!f) return;
    string line = host + " " + fp + "\n";
    UINT bw; f_write(f, line.data(), line.size(), &bw);
    fclose2(f);
}

// Host-key callback: trust-on-first-use against /known_hosts, refuse on mismatch.
static string g_net_host_for_cb;
static Ssh::TrustResult netHostKeyCb(const char* host, const char* keytype,
                                     const char* fp, bool sig_verified) {
    (void)host; (void)sig_verified;
    string stored = knownHostsLookup(g_net_host_for_cb);
    if (!stored.empty()) {
        if (stored == fp) return Ssh::TRUST;
        OSD::msgDialog(MSG_NET_HOSTKEY_BAD[Config::lang], fp); // mismatch → refuse
        return Ssh::REJECT;
    }
    // First contact — show fingerprint and ask to trust.
    string body = string(keytype) + "\nSHA256:" + fp + "\n" + MSG_NET_TRUST_Q[Config::lang];
    if (OSD::msgDialog(g_net_host_for_cb, body) == DLG_YES) {
        knownHostsSave(g_net_host_for_cb, fp);
        return Ssh::TRUST;
    }
    return Ssh::REJECT;
}

// The SSH/SFTP crypto path (mbedTLS ECDH/ECP/bignum) needs far more than the
// 4 KB core-0 stack (PICO_STACK_SIZE=0x1000), especially nested under do_OSD —
// it overflows and faults (observed: SIGBUS, stackOvf=1). So we run the whole
// network session on a large heap-allocated stack via a switch trampoline.
// ARMv8-M (RP2350/Cortex-M33): also clear MSPLIM during the window so the
// hardware stack-limit check doesn't fault on the alternate stack.
struct NetSessCtx { string host, user, pass, restorePath; uint16_t port; uint8_t proto; };

// Record the global "last F5 location" (Config::last_loc) — persisted so F5 reopens
// where you left off. Writes wifi.cfg only when the value actually changes.
static void lastLocSet(const string& v) {
    if (Config::last_loc != v) { Config::last_loc = v; Config::saveWifiConfig(); }
}

static void netSessionRun(void* p) {
    NetSessCtx* c = (NetSessCtx*)p;
    // progressDialog saves/restores its background (osdCenteredMsg with 0 does not),
    // so the notice is cleanly removed before the browser menu draws. The host-key
    // trust msgDialog (during SSH connect) saves/restores its own rect over this.
    OSD::progressDialog(MSG_NET_CONNECTING[Config::lang], c->host, 0, 0);
    RemoteFs* fs = nullptr;
    if (c->proto == 0) {
        Ftp* ftp = new Ftp();
        if (ftp->connect(c->host.c_str(), c->port, c->user.c_str(), c->pass.c_str())) fs = ftp;
        else delete ftp;
    } else {
        g_net_host_for_cb = c->host;
        Ssh::setHostKeyCb(netHostKeyCb);
        Sftp* sftp = new Sftp();
        if (sftp->connect(c->host.c_str(), c->port, c->user.c_str(), c->pass.c_str())) fs = sftp;
        else delete sftp;
    }
    OSD::progressDialog("", "", 0, 2); // close the "Connecting..." notice
    if (!fs) { OSD::osdCenteredMsg(MSG_NET_CONN_ERR[Config::lang], LEVEL_WARN, 2500); return; }
    if (!c->restorePath.empty() && c->restorePath != fs->cwdPath())
        fs->cwd(c->restorePath);               // reopen at the last folder for this remote
    OSD::remoteFileDialog(fs); // SD-indexed browser (bounded RAM); runs on this alt-stack
    fs->disconnect();
    delete fs;
    // Remember this remote + the folder we left off in, as the global last F5 location.
    char b[32]; snprintf(b, sizeof(b), "%u\t%u", (unsigned)c->port, (unsigned)c->proto);
    lastLocSet("R\t" + c->host + "\t" + b + "\t" + c->user + "\t" + OSD::net_last_path);
}

// Call fn(arg) with MSP switched to new_top (8-byte aligned highest address of a
// scratch buffer). Saves/restores SP, MSPLIM and LR. RP2350/ARM only (this whole
// file region is gated to RP2350 via ZIFI_NET_CLIENT).
extern "C" __attribute__((naked, noinline))
void net_call_on_stack(void* new_top, void (*fn)(void*), void* arg) {
    __asm volatile(
        "mrs  r12, msplim       \n" // save old MSPLIM
        "movs r3, #0            \n"
        "msr  msplim, r3        \n" // disable stack-limit check on the alt stack
        "mov  r3, sp            \n" // r3 = old SP
        "mov  sp, r0            \n" // SP = new_top
        "push {r2, r3, r12, lr} \n" // 16 bytes → keeps 8-byte alignment
        "mov  r0, r2            \n" // r0 = arg
        "blx  r1                \n" // fn(arg)
        "pop  {r2, r3, r12, lr} \n"
        "mov  sp, r3            \n" // restore SP
        "msr  msplim, r12       \n" // restore MSPLIM
        "bx   lr                \n"
    );
}

// RAII: for the duration of a (paused) network session, lend the dormant
// Gigascreen prev framebuffer to the Buffer pool so the alt-stack + TLS/socket
// working set draws from those ~52 KB instead of the scarce heap. No-op unless
// there's something to lend (Gigascreen on AND a butter-less board); on butter
// boards palloc routes the TLS set to XIP PSRAM instead, so no lease is needed.
struct NetArenaLease {
    bool held = false;
    bool released = false;
    NetArenaLease() {
        void* base; size_t size;
        if (VIDEO::gigascreenLendRegion(base, size)) {
            if (Buffer::lendArena(base, size)) held = true;
            else VIDEO::gigascreenReclaimRegion();   // lend rejected → undo the detach
        } else {
            // Nothing lendable can still mean there IS a prev-FB — just a chunked one,
            // which is not one region. Then it is released outright for the session
            // (VIDEO::gigascreenReleaseForNet) so the heap, not the arena, gets those
            // ~38 KB. No-op in every other case.
            released = VIDEO::gigascreenReleaseForNet();
        }
    }
    ~NetArenaLease() {
        if (held) { Buffer::reclaimArena(); VIDEO::gigascreenReclaimRegion(); }
        if (released) VIDEO::gigascreenRestoreAfterNet();
    }
};

// Alloc/free the network alt-stack via the Buffer pool so it can come from the
// lent arena (USE_NET_ARENA) when one is active, else heap. Returns nullptr on OOM.
static inline uint8_t* netAltStackAlloc(size_t& stksz) {
    uint8_t* p = (uint8_t*)Buffer::palloc(stksz, Buffer::NEED_POINTER | Buffer::USE_NET_ARENA);
    if (!p) { stksz = 8 * 1024; p = (uint8_t*)Buffer::palloc(stksz, Buffer::NEED_POINTER | Buffer::USE_NET_ARENA); }
    return p;
}

// Connect to a saved remote (prompting the password if it wasn't stored) and run
// the connect + browse session on a large heap stack (see net_call_on_stack). The
// crypto/listing path can't live on the 4 KB core stack.
static void netConnectRemote(const Config::Remote& r, const string& restorePath = "") {
    string pass = r.pass;
    if (!r.savepass) {
        pass = netAskField(MSG_NET_PASS_LABEL[Config::lang], "", true); // masked
        if (pass == "\x1B") return;
    }
    // Remember as the "last used" defaults (prefill the Add-Remote form next time).
    Config::net_host = r.host; Config::net_user = r.user;
    Config::net_port = r.port; Config::net_proto = r.proto;
    Config::saveWifiConfig();

    NetSessCtx ctx; ctx.host = r.host; ctx.user = r.user; ctx.pass = pass;
    ctx.port = r.port; ctx.proto = r.proto; ctx.restorePath = restorePath;

    // Modest alt stack: mbedTLS curve25519/P256/RSA-verify peaks at only a few KB,
    // and the heap must keep enough for the handshake + SSH objects + the listing.
    NetArenaLease lease;   // borrow the dormant Gigascreen prevFB if available
    size_t stksz = 12 * 1024;
    uint8_t* stk = netAltStackAlloc(stksz);
    if (!stk) { OSD::osdCenteredMsg(MSG_NET_CONN_ERR[Config::lang], LEVEL_WARN, 2500); return; }
    void* top = (void*)(((uintptr_t)stk + stksz) & ~(uintptr_t)7); // 8-byte aligned top
    net_call_on_stack(top, netSessionRun, &ctx);
    Buffer::pfree(stk);
}

// Shared scratch for the remotes list (avoids a big array on the 4 KB core stack;
// one network UI runs at a time). Lazy-heaped — costs 0 SRAM until a network UI
// actually opens, so it never starves the razor-thin Profi heap at VIDEO::Init.
// (Profi forces ~80 KB of SRAM pages before the framebuffer alloc; every byte of
// permanent BSS here pushes it into OOM.) ~2 KB once allocated; never freed (the
// one allocation persists for the session — it's small and reused).
static Config::Remote* remotesBuf() {
    static Config::Remote* p = nullptr;
    if (!p) p = new (std::nothrow) Config::Remote[Config::MAX_REMOTES];
    return p;
}
#define g_remotes remotesBuf()

// Add Remote: prompt protocol / host / user / port / password (+ "save password")
// and append the connection to remotes.tsv. Does not connect.
static void addRemoteForm() {
    string ssid, ip;
    if (!ZiFiAT::getStatus(ssid, ip)) { OSD::osdCenteredMsg(MSG_NET_FT_NOWIFI[Config::lang], LEVEL_WARN, 2200); return; }

    OSD::menu_level = 2; OSD::menu_saverect = true; OSD::menu_curopt = Config::net_proto + 1;
    uint8_t proto = OSD::menuRun(MENU_NET_PROTO[Config::lang]); // 1=FTP, 2=SFTP
    if (proto == 0) return;
    VIDEO::SaveRect.restore_last();
    uint8_t pr = proto - 1;

    // Field order: Host → Port → User → Pass (→ save-password → Alias below).
    string host = netAskField(MSG_NET_HOST_LABEL[Config::lang], Config::net_host);
    if (host == "\x1B" || host.empty()) return;
    uint16_t defport = pr ? 22 : 21;
    char pbuf[8]; snprintf(pbuf, sizeof(pbuf), "%u", Config::net_port ? Config::net_port : defport);
    string ports = netAskField(MSG_NET_PORT_LABEL[Config::lang], pbuf);
    if (ports == "\x1B") return;
    uint16_t port = (uint16_t)atoi(ports.c_str()); if (!port) port = defport;
    string user = netAskField(MSG_NET_USER_LABEL[Config::lang], Config::net_user);
    if (user == "\x1B") return;
    string pass = netAskField(MSG_NET_PASS_LABEL[Config::lang], "", true); // masked
    if (pass == "\x1B") return;

    OSD::menu_level = 2; OSD::menu_saverect = true; OSD::menu_curopt = 1;
    uint8_t sp = OSD::menuRun(MENU_REMOTE_SAVEPASS[Config::lang]); // 1=No, 2=Yes
    if (sp == 0) return;
    VIDEO::SaveRect.restore_last();
    bool savepass = (sp == 2);

    // Optional start directory, then display name — Enter to leave empty, Esc cancels.
    string path = netAskField(MSG_REMOTE_PATH_LABEL[Config::lang], "");
    if (path == "\x1B") return;
    string alias = netAskField(MSG_REMOTE_ALIAS_LABEL[Config::lang], "");
    if (alias == "\x1B") return;

    if (!g_remotes) return;   // alloc failed (OOM) — bail out of the form
    int n = Config::loadRemotes(g_remotes, Config::MAX_REMOTES);
    if (n >= Config::MAX_REMOTES) { OSD::osdCenteredMsg(MSG_REMOTE_FULL[Config::lang], LEVEL_WARN, 2000); return; }
    g_remotes[n].host = host; g_remotes[n].user = user; g_remotes[n].port = port;
    g_remotes[n].proto = pr; g_remotes[n].savepass = savepass;
    g_remotes[n].pass = savepass ? pass : ""; g_remotes[n].alias = alias; g_remotes[n].path = path;
    Config::saveRemotes(g_remotes, n + 1);

    Config::net_host = host; Config::net_user = user; Config::net_port = port; Config::net_proto = pr;
    Config::saveWifiConfig();
}

// Remote (FTP/SFTP): list saved connections; Enter connects (prompting the
// password if not stored), F8 forgets a connection, and a trailing "[Add Remote]"
// row opens the Add-Remote form.
// Remote (FTP/SFTP) saved-connection list — rendered in the file-browser window
// (FD_SIDE_HOSTS sidebar) so it matches SD. Enter connects (alt-stack browse),
// F8 forgets, the trailing [Add Remote] row opens the form.
static void remoteHostsBrowse() {
    string ssid, ip;
    if (!ZiFiAT::getStatus(ssid, ip)) { OSD::osdCenteredMsg(MSG_NET_FT_NOWIFI[Config::lang], LEVEL_WARN, 2200); return; }
    if (!g_remotes) return;   // alloc failed (OOM) — can't list remotes
    int hf = 2, hb = 2;                               // keep the cursor on the chosen host
    while (1) {
        int n = Config::loadRemotes(g_remotes, Config::MAX_REMOTES);
        std::vector<string> rows;
        rows.push_back(string(2, (char)DIR_MARKER) + "..");  // row 0: ".." → locations root
        for (int i = 0; i < n; i++) {
            if (!g_remotes[i].alias.empty()) {        // show the alias when set
                rows.push_back(g_remotes[i].alias);
            } else {
                char b[96];
                snprintf(b, sizeof(b), "%s@%s:%u (%s)", g_remotes[i].user.c_str(),
                         g_remotes[i].host.c_str(), (unsigned)g_remotes[i].port,
                         g_remotes[i].proto ? "sftp" : "ftp");
                rows.push_back(b);                    // rows 1..n: a plain row (no <DIR>)
            }
        }
        rows.push_back(MSG_REMOTE_ADD_ROW[Config::lang]);   // row n+1
        int key;
        int sel = OSD::fdChromeList(rows, MENU_ALL_TITLE[Config::lang],
                                    MENU_REMOTE_TITLE[Config::lang],
                                    OSD::FD_SIDE_HOSTS, false, &key, &hf, &hb);
        if (sel <= 0) {                               // "..", Backspace, or Esc → leave list
            if (sel < 0 && key == OSD::FDK_ESC) OSD::net_close_all = true; // Esc → close OSD
            return;                                   // ".."/Backspace → climb to locations
        }
        if (sel == n + 1) { addRemoteForm(); continue; }    // [Add Remote] row
        int hi = sel - 1;                             // host index (row 0 is "..")
        if (hi < 0 || hi >= n) continue;
        if (key == OSD::FDK_F8) {                      // F8 → forget this connection
            if (OSD::msgDialog(g_remotes[hi].host, MSG_REMOTE_FORGET_Q[Config::lang]) == DLG_YES) {
                for (int j = hi; j < n - 1; j++) g_remotes[j] = g_remotes[j + 1];
                Config::saveRemotes(g_remotes, n - 1);
            }
            continue;
        }
        netConnectRemote(g_remotes[hi], g_remotes[hi].path); // connect → cd into saved Path
        if (OSD::net_launch_close || OSD::net_close_all) return;  // launched or Esc → unwind
    }
}

static void netDownloadArchive();   // Web Archives entry (defined below)

// Set by the F5 handler on a fresh F5 press → f5Locations restores Config::last_loc
// once (so F5 reopens where you left off). Cleared after use; the SD-back-out re-entry
// (goto) does NOT set it, so backing out always lands on the chooser.
static bool g_f5_restore = false;

// F5 last-location restore for the catalog (set by f5Locations before the alt-stack
// archSessionRun entry): open this site at this path directly, skipping the list once.
static string g_web_rsite, g_web_rpath;
static bool   g_web_restore = false;

// Split a tab-separated string into fields.
static void splitTabs(const string& s, std::vector<string>& out) {
    out.clear();
    size_t start = 0;
    for (size_t i = 0; i <= s.size(); ++i)
        if (i == s.size() || s[i] == '\t') { out.push_back(s.substr(start, i - start)); start = i + 1; }
}

// F5 location level — rendered IN the file-browser window (Open File + sidebar) as
// rows: Local (SD) / [USB Drive] / Remote (FTP/SFTP) / Web Archives / Add Remote.
// The USB row appears only while a mass-storage stick is enumerated. Returns true
// when the user chose a LOCAL source — SD or USB (caller then opens the browser at
// FileUtils::ALL_Path, which this function points at the chosen volume); false for
// Esc or any network action (caller closes the OSD).

// The chooser is reachable when there's any location besides the SD card: a WiFi
// link (or saved SSID) or a USB stick. With the stick AS the root volume (no SD
// at boot) "Local" already browses it, so it adds no extra location.
static inline bool f5HasChooser() {
    return ZiFiAT::connected || !Config::wifi_ssid.empty() ||
           (UsbMsc::ready() && !FileUtils::usbRoot);
}

// Last browse dir per local source, so switching SD⇄USB in the chooser returns to
// where you were on each side. ALL_Path holds whichever is active (and is the one
// persisted to NVS; a stale "USB:/..." with no stick self-heals in fileDialog).
static string s_f5_sd_dir  = "/";
static string s_f5_usb_dir = "USB:/";

static bool f5Locations() {
    // One-time restore of the last browse location (across all sources), so F5 reopens
    // where you left off — Local stays at ALL_Path, Web/Remote reopen their last folder.
    if (g_f5_restore) {
        g_f5_restore = false;
        const string& L = Config::last_loc;
        if (!L.empty() && (L[0] == 'W' || L[0] == 'R')) {
            // Only auto-reopen a network location when the WiFi link is actually up.
            // Right after a (power-cycle) reboot the ESP may not be associated yet;
            // diving into a blocking HTTPS revalidate/fetch then would freeze the UI
            // ("F5 hangs hard"). Not connected → skip the dive and fall through to the
            // chooser below, so F5 stays responsive.
            std::vector<string> f; splitTabs(L, f);
            if (!ZiFiAT::connected) {
                /* fall through to the chooser */
            } else if (L[0] == 'W' && f.size() >= 3) {       // W \t site \t path
                g_web_rsite = f[1]; g_web_rpath = f[2]; g_web_restore = true;
                netDownloadArchive();
                g_web_restore = false;    // clear even if netDownloadArchive bailed early
                if (OSD::net_launch_close || OSD::net_close_all) return false;
            } else if (L[0] == 'R' && f.size() >= 6 && g_remotes) {  // R \t host \t port \t proto \t user \t path
                int n = Config::loadRemotes(g_remotes, Config::MAX_REMOTES);
                uint16_t port = (uint16_t)atoi(f[2].c_str());
                uint8_t  proto = (uint8_t)atoi(f[3].c_str());
                for (int i = 0; i < n; i++)
                    if (g_remotes[i].host == f[1] && g_remotes[i].port == port &&
                        g_remotes[i].proto == proto && g_remotes[i].user == f[4]) {
                        netConnectRemote(g_remotes[i], f[5]); break;
                    }
                if (OSD::net_launch_close || OSD::net_close_all) return false;
            }
            // after the restored browse → fall through to the chooser below
        } else {
            return true;   // "L" or empty → Local (SD) at ALL_Path
        }
    }
    int lf = 2, lb = 2;                               // keep the cursor on the chosen location
    while (1) {
        if (OSD::net_launch_close) return false;      // a quick-start launched → close OSD
        // Hide the USB row when the stick is the root volume — "Local" IS the stick.
        const bool usb = UsbMsc::ready() && !FileUtils::usbRoot;
        std::vector<string> rows = {
            // USB-as-root: "Local" is the stick, not the (absent) SD card.
            string(1, (char)DIR_MARKER) + (FileUtils::usbRoot ? MSG_F5_USB[Config::lang]
                                                              : MSG_F5_LOCAL[Config::lang]),
            string(1, (char)DIR_MARKER) + MSG_F5_REMOTE[Config::lang],
            string(1, (char)DIR_MARKER) + MSG_F5_WEB[Config::lang],
            MSG_F5_ADD_REMOTE[Config::lang],
        };
        if (usb)
            rows.insert(rows.begin() + 1, string(1, (char)DIR_MARKER) + MSG_F5_USB[Config::lang]);
        OSD::menu_level = 0;
        int key;
        int loc = OSD::fdChromeList(rows, MENU_ALL_TITLE[Config::lang],
                                    MENU_F5_LOCATION[Config::lang],
                                    OSD::FD_SIDE_LOCATIONS, false, &key, &lf, &lb);
        if (loc < 0) return false;                    // Esc → close OSD
        if (usb && loc == 1) {                        // USB Drive → browse the stick
            if (FileUtils::ALL_Path.compare(0, 4, "USB:") != 0) {
                s_f5_sd_dir = FileUtils::ALL_Path;
                FileUtils::ALL_Path = s_f5_usb_dir;
            }
            return true;
        }
        if (usb && loc > 1) loc--;                    // fold the USB row out of the indices
        if (loc == 0) {                               // Local (SD) → caller opens SD browser
            if (FileUtils::ALL_Path.compare(0, 4, "USB:") == 0) {
                s_f5_usb_dir = FileUtils::ALL_Path;
                FileUtils::ALL_Path = s_f5_sd_dir;
            }
            return true;
        }
        else if (loc == 1) remoteHostsBrowse();
        else if (loc == 2) netDownloadArchive();      // Web Archives (built-in catalog)
        else if (loc == 3) addRemoteForm();
        if (OSD::net_launch_close || OSD::net_close_all) return false;
    }
}

// ── FTP server: share the SD card to the LAN ─────────────────────────────────
struct FtpdCtx { const char* ip; };

// Run the server session (details panel + live log terminal) in a blocking loop
// until ESC. The Z80 stays frozen (we never call CPU::loop). Invoked on a heap
// alt-stack via net_call_on_stack — FatFS + the OSD draw chain are too deep for
// the 4 KB core stack when nested under do_OSD (same reason as the net client).
static void ftpdSessionRun(void* arg) {
    const char* ip = ((FtpdCtx*)arg)->ip;

    ftpd_log_count = 0; ftpd_log_dirty = true;
    if (!Ftpd::begin(21, ftpdLogLine)) {
        OSD::osdCenteredMsg(Config::lang ? "Error al iniciar FTP" : "FTP server start failed", LEVEL_WARN, 2500);
        Ftpd::stop();
        return;
    }
    ftpdLogLine(Config::lang ? "Servidor FTP iniciado" : "FTP server started");
    char det[FTPD_LOG_COLS];
    snprintf(det, sizeof(det), "ftp://%s:21  user: anonymous", ip);
    ftpdLogLine(det);
    ftpdLogLine(Config::lang ? "Modo ACTIVO. ESC para parar." : "Use ACTIVE mode. ESC to stop.");

    unsigned short sx = OSD::scrAlignCenterX(OSD_W);
    unsigned short sy = OSD::scrAlignCenterY(OSD_H);
    VIDEO::SaveRect.save(sx, sy, OSD_W, OSD_H);

    string sub = string("ftp://") + ip + ":21  anon  ESC:stop";

    auto draw = [&]() {
        OSD::drawOSD(true);
        int visCols = OSD::osdMaxCols(); if (visCols > 40) visCols = 40;
        int visRows = OSD::osdMaxRows() - 4;
        char row[42];
        // Subtitle (row 1): connection string + hint.
        OSD::osdAt(1, 0);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
        snprintf(row, sizeof(row), " %s", sub.c_str());
        int hl = strlen(row); while (hl < visCols) row[hl++] = ' '; row[visCols] = '\0';
        VIDEO::vga.print(row);
        // Separator (row 2).
        OSD::osdAt(2, 0);
        VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(1, 0));
        memset(row, '-', visCols); row[0] = ' '; row[visCols] = '\0';
        VIDEO::vga.print(row);
        // Log tail (rows 3..): the most recent visRows lines.
        int first = ftpd_log_count > visRows ? ftpd_log_count - visRows : 0;
        VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
        for (int r = 0; r < visRows; r++) {
            OSD::osdAt(3 + r, 0);
            int li = first + r;
            if (li < ftpd_log_count) {
                const char* s = ftpd_log[li % FTPD_LOG_LINES];
                int len = strlen(s); if (len > visCols) len = visCols;
                memcpy(row, s, len); memset(row + len, ' ', visCols - len);
            } else memset(row, ' ', visCols);
            row[visCols] = '\0';
            VIDEO::vga.print(row);
        }
    };
    draw();

    for (;;) {
        Ftpd::poll();
        if (ftpd_log_dirty) { draw(); ftpd_log_dirty = false; }
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            fabgl::VirtualKeyItem k;
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&k);
            if (k.down && is_back(k.vk)) break;
        }
        sleep_ms(2);
    }

    Ftpd::stop();
    OSD::osdCenteredMsg(Config::lang ? "Servidor FTP detenido" : "FTP server stopped", LEVEL_WARN, 1200);
    VIDEO::SaveRect.restore_last();
}

// Entry: require WiFi, then run the session on a heap alt-stack.
static void ftpServerRun() {
    string ssid, ip;
    if (!ZiFiAT::getStatus(ssid, ip)) {
        OSD::osdCenteredMsg(MSG_NET_FT_NOWIFI[Config::lang], LEVEL_WARN, 2200);
        return;
    }
    FtpdCtx ctx; ctx.ip = ip.c_str(); // ip outlives the (synchronous) call below
    NetArenaLease lease;   // borrow the dormant Gigascreen prevFB if available
    size_t stksz = 8 * 1024;
    uint8_t* stk = netAltStackAlloc(stksz);
    if (!stk) { OSD::osdCenteredMsg(MSG_NET_CONN_ERR[Config::lang], LEVEL_WARN, 2500); return; }
    void* top = (void*)(((uintptr_t)stk + stksz) & ~(uintptr_t)7); // 8-byte aligned top
    net_call_on_stack(top, ftpdSessionRun, &ctx);
    Buffer::pfree(stk);
}

// ── Download archive (catalog server over plain HTTP) ───────────────────────
// Browses the pico-spec catalog server (HttpCatalogFs) with the same generic
// remoteFileDialog used for FTP/SFTP. Runs on the large heap stack like the
// FTP/SFTP session — no crypto here, but the SD-indexed browser is the heavy
// part, so we keep off the 4 KB core stack for safety + consistency.
// Open one catalog site at `path`, browse, and record it as the global last location.
static void archOpenSite(const char* siteId, const string& path) {
    HttpCatalogFs* fs = new HttpCatalogFs(siteId);
    if (!path.empty() && path != fs->cwdPath()) fs->cwd(path);
    OSD::remoteFileDialog(fs);                  // catalog in the same chrome
    fs->disconnect();
    delete fs;
    lastLocSet(string("W\t") + siteId + "\t" + OSD::net_last_path);
}

static void archSessionRun(void* p) {
    (void)p;
    // Restore: open the last catalog site+path directly (then fall to the site list).
    if (g_web_restore) {
        g_web_restore = false;
        archOpenSite(g_web_rsite.c_str(), g_web_rpath);
        if (OSD::net_launch_close || OSD::net_close_all) return; // launched or Esc → unwind
    }
    OSD::progressDialog(MSG_NET_CONNECTING[Config::lang], "", 0, 0); // no URL (built-in catalog)
    static string site_ids[12];
    static string site_names[12];
    int n = HttpCatalogFs::fetchSites(site_ids, site_names, 12);
    OSD::progressDialog("", "", 0, 2);
    if (n <= 0) { OSD::osdCenteredMsg(MSG_ARCH_SITES_ERR[Config::lang], LEVEL_WARN, 2200); return; }

    // Site list rendered IN the browser window (not a popup) — ".." row returns to the
    // locations level; selecting a source opens its catalog in the same chrome.
    int sf = 2, sb = 2;                                  // keep the cursor on the chosen site
    while (1) {
        std::vector<string> rows;
        rows.push_back(string(2, (char)DIR_MARKER) + "..");                 // → locations
        for (int i = 0; i < n; i++) rows.push_back(string(1, (char)DIR_MARKER) + site_names[i]);
        int key;
        int sel = OSD::fdChromeList(rows, MENU_ALL_TITLE[Config::lang],
                                    MENU_ARCH_SITE_TITLE[Config::lang],
                                    OSD::FD_SIDE_LOCATIONS, false, &key, &sf, &sb);
        if (sel <= 0) {                             // "..", Backspace, or Esc → leave list
            if (sel < 0 && key == OSD::FDK_ESC) OSD::net_close_all = true; // Esc → close OSD
            return;                                 // ".."/Backspace → climb to locations
        }
        int si = sel - 1;                           // site index (row 0 is "..")
        if (si < 0 || si >= n) continue;
        archOpenSite(site_ids[si].c_str(), "");     // open at root
        if (OSD::net_launch_close || OSD::net_close_all) return;  // launched or Esc → unwind
    }
}

// Top-level entry: require an active WiFi link, then browse the built-in online
// catalog. The catalog URL is hardcoded (HttpCatalogFs) — nothing to configure.
static void netDownloadArchive() {
    string ssid, ip;
    if (!ZiFiAT::getStatus(ssid, ip)) {
        OSD::osdCenteredMsg(MSG_NET_FT_NOWIFI[Config::lang], LEVEL_WARN, 2200);
        return;
    }

    NetArenaLease lease;   // borrow the dormant Gigascreen prevFB if available
    size_t stksz = 12 * 1024;
    uint8_t* stk = netAltStackAlloc(stksz);
    if (!stk) { OSD::osdCenteredMsg(MSG_NET_CONN_ERR[Config::lang], LEVEL_WARN, 2500); return; }
    void* top = (void*)(((uintptr_t)stk + stksz) & ~(uintptr_t)7);
    net_call_on_stack(top, archSessionRun, nullptr);
    Buffer::pfree(stk);
}

// ── HTTP test ("curl") ──────────────────────────────────────────────────────
// A diagnostic: GET an arbitrary URL and show the result. Exercises HttpsGet ->
// TlsSock (host-TLS over the ESP plain-TCP pipe) on real hardware — the Step-0
// spike that can't be validated off-target. Output is a summary only (no SD file).
extern size_t getFreeHeap(void);

namespace {
struct HttpTestCtx { std::string url; };
// Preview accumulator passed to the sink as ctx (lives on the alt-stack, no BSS).
struct HttpPreview { char* buf; size_t cap; size_t n; };

// Collects a printable preview of the first bytes; drains the rest (return true)
// so the whole body still streams (and Content-Length is satisfied).
bool httpTestSink(void* ctx, const uint8_t* data, size_t len) {
    HttpPreview* pv = (HttpPreview*)ctx;
    while (pv->n + 1 < pv->cap && len) {
        uint8_t b = *data++; len--;
        bool printable = (b >= 32 && b < 127) || b == '\n' || b == '\t';
        pv->buf[pv->n++] = printable ? (char)b : '.';
    }
    return true;
}

void httpTestRun(void* p) {
    HttpTestCtx* c = (HttpTestCtx*)p;
    // Buffers live on this alt-stack (12 KB) — no permanent SRAM/BSS. Kept modest
    // so they don't crowd the mbedTLS handshake sharing the same stack.
    char preview[512];
    char summary[1024];
    HttpPreview pv = { preview, sizeof(preview), 0 };

    OSD::progressDialog(MSG_HTTP_TESTING[Config::lang], c->url, 0, 0);
    size_t heap0 = getFreeHeap();
    HttpsGet::Result r = HttpsGet::get(c->url.c_str(), httpTestSink, &pv,
                                       CONFIG_DIR "/cacert.pem");
    size_t heap1 = getFreeHeap();
    OSD::progressDialog("", "", 0, 2);
    preview[pv.n] = '\0';
    Debug::log("netHttpTest: status=%d len=%lu recv=%lu ok=%d heap before/after=%u/%u",
               r.status, (unsigned long)r.length, (unsigned long)r.received, r.ok,
               (unsigned)heap0, (unsigned)heap1);
    snprintf(summary, sizeof(summary),
             "%s\n\nstatus: %d\nContent-Length: %lu\nreceived: %lu\nok: %s\n"
             "free heap: %u -> %u\n\n--- body ---\n%s",
             c->url.c_str(), r.status, (unsigned long)r.length,
             (unsigned long)r.received, r.ok ? "yes" : "no",
             (unsigned)heap0, (unsigned)heap1, preview);
    OSD::showTextDialog(MSG_HTTP_TEST_TITLE[Config::lang], summary);
}
} // namespace

static void netHttpTest() {
    Buffer::selfTest();   // exercise the tiered allocator (logs to serial) before the GET
    string ssid, ip;
    if (!ZiFiAT::getStatus(ssid, ip)) {
        OSD::osdCenteredMsg(MSG_NET_FT_NOWIFI[Config::lang], LEVEL_WARN, 2200);
        return;
    }
    // Scheme.
    OSD::menu_level = 2; OSD::menu_saverect = true; OSD::menu_curopt = 1;
    uint8_t sc = OSD::menuRun(MENU_HTTP_SCHEME[Config::lang]);
    if (sc == 0) return;
    VIDEO::SaveRect.restore_last();
    const char* scheme = (sc == 1) ? "https" : "http";

    // Session-only memory of the last host/path (RAM only — not persisted to NVS).
    // Survives until the board reboots; no Config default is substituted.
    static string lastHost;          // empty on first use
    static string lastPath = "/";

    // Host[:port] — prefilled with whatever was last typed this session.
    string host = netAskField(MSG_HTTP_HOST_LABEL[Config::lang], lastHost);
    if (host == "\x1B" || host.empty()) return;
    lastHost = host;

    // Path — visible width clamped to the screen, but the input cap is larger so
    // long paths can be typed (the field scrolls horizontally past the window).
    int pathField = (int)OSD::osdMaxCols() - (int)strlen(MSG_HTTP_PATH_LABEL[Config::lang]) - 4;
    if (pathField < 20) pathField = 20;
    if (pathField > 64) pathField = 64;
    string path = netAskField(MSG_HTTP_PATH_LABEL[Config::lang], lastPath, false, pathField, 255);
    if (path == "\x1B") return;
    if (path.empty() || path[0] != '/') path = "/" + path;
    lastPath = path;

    HttpTestCtx ctx;
    ctx.url = string(scheme) + "://" + host + path;

    // Run the GET (and its TLS handshake) on a large heap stack, like the FTP/
    // archive sessions — the 4 KB core stack can't hold the mbedTLS handshake.
    NetArenaLease lease;   // borrow the dormant Gigascreen prevFB if available
    size_t stksz = 12 * 1024;
    uint8_t* stk = netAltStackAlloc(stksz);
    if (!stk) { OSD::osdCenteredMsg(MSG_NET_CONN_ERR[Config::lang], LEVEL_WARN, 2500); return; }
    void* top = (void*)(((uintptr_t)stk + stksz) & ~(uintptr_t)7);
    net_call_on_stack(top, httpTestRun, &ctx);
    Buffer::pfree(stk);
}
#endif // ZIFI_NET_CLIENT
#endif

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
        uint8_t vm = 
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
        SELECT_VGA ? Config::vga_video_mode :
#endif
         Config::hdmi_video_mode;
        const char* vmname;
        switch (vm) {
            case Config::VM_640x480_60: vmname = "640x480@60Hz"; break;
            case Config::VM_640x480_50: vmname = "640x480@50Hz"; break;
            case Config::VM_720x480_60: vmname = "720x480@60Hz"; break;
            case Config::VM_720x576_50: vmname = "720x576@50Hz"; break;
            default:                    vmname = "unknown";      break;
        }
        char buf2[41];
        snprintf(buf2, sizeof(buf2), " Video: %s %s  ",
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
                SELECT_VGA ? "VGA" : 
#endif
                "HDMI", vmname);
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
    } else if ((Z80Ops::isPentagon || Z80Ops::isProfi)) {
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

void OSD::drawVolumeBox() {

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
    VIDEO::vga.fillRect(x, y - 4, 24 * 6, 16, zxColor(1, 0));
    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(x + 4, y + 1);
    VIDEO::vga.print(Config::tape_player ? "TAP" : "VOL");
    for (int i = 0; i < ESPectrum::aud_volume + 16; i++) {
        VIDEO::vga.fillRect(x + 26 + (i * 7), y + 1, 6, 7, zxColor(7, 0));
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
    string finfo = string(DISK_PSNA_DIR) + "/" + persistfinfo;
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
    string fsna  = string(DISK_PSNA_DIR) + "/" + persistfname;
    string finfo = string(DISK_PSNA_DIR) + "/" + persistfinfo;
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
    string finfo = string(DISK_PSNA_DIR) + "/" + persistfinfo;

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

#if !PICO_RP2040
// Throttled progress overlay for IDE::createImage (zero-fill can take seconds).
static void ide_create_progress(uint32_t done, uint32_t total) {
    static int lastpct = -1;
    int pct = total ? (int)((uint64_t)done * 100 / total) : 100;
    if (pct == lastpct) return;
    lastpct = pct;
    char msg[32];
    snprintf(msg, sizeof(msg), "Creating HDD... %d%%", pct);
    OSD::osdCenteredMsg(msg, LEVEL_INFO, 0);
}
#endif


static bool persistSave(uint8_t slotnumber, uint8_t opt2, bool quicksave = false)
{
    FILINFO stat_buf;
    char persistfname[sizeof(DISK_PSNA_FILE) + 7];
    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];
    sprintf(persistfname, DISK_PSNA_FILE "%u.sna", slotnumber);
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);
    string finfo = string(DISK_PSNA_DIR) + "/" + persistfinfo;

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

    string fsna = string(DISK_PSNA_DIR) + "/" + persistfname;
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

    if (!FileSNA::isPersistAvailable(string(DISK_PSNA_DIR) + "/" + persistfname)) {
        OSD::osdCenteredMsg(OSD_PSNA_NOT_AVAIL, LEVEL_INFO);
        return false;
    } else {
        // Read info file
        string finfo = string(DISK_PSNA_DIR) + "/" + persistfinfo;
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

        if (!LoadSnapshot(string(DISK_PSNA_DIR) + "/" + persistfname, persist_arch, persist_romset)) {
            OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
            return false;
        }
        else
        {
            Config::ram_file = string(DISK_PSNA_DIR) + "/" + persistfname;
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

// Cold-boot into TR-DOS for the current architecture. Mirrors the per-arch
// "Reset to… → TR-DOS" logic (HK_RESET_TO): Profi boots bank1 + SYSEN-style
// TR-DOS, Pentagon/Profi (incl. Gluk 128Kpg) boots the TR-DOS ROM (bank4) with
// romLatch + trdos asserted. 48K/128K have no dedicated TR-DOS cold-boot bank
// (entry is only via the BASIC trap), so they get a plain reset with the disk
// left mounted. Used by the Alt+Enter "download a disk to /tmp and run" path.
void OSD::bootTrdos() {
    Config::ram_file = NO_RAM_FILE;
    Config::last_ram_file = NO_RAM_FILE;
    if (Config::arch == "Profi") {
        ESPectrum::reset(1);
        MemESP::romLatch = 1;
        ESPectrum::trdos = true;
    } else if (Z80Ops::isPentagon || Z80Ops::isProfi) {
        ESPectrum::reset(4); // TR-DOS ROM
        MemESP::romLatch = 1;
        ESPectrum::trdos = true;
    } else {
        ESPectrum::reset(0); // 48K/128K: no TR-DOS cold bank — restart, disk stays mounted
    }
}

// OSD Main Loop
void OSD::nmiAction() {
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
            // BYTE and BYTE-compat are both overlays over the Sinclair 48K base.
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
            MemESP::registerOverlay(gb_rom_0_sinclair_48k,
                Config::byte_cobmect_mode ? gb_overlay_48k_byte_sovmest : gb_overlay_48k_byte);
            MemESP::recoverPage0();
            osdCenteredMsg(Config::byte_cobmect_mode ? OSD_COBMECT_ON[Config::lang] : OSD_COBMECT_OFF[Config::lang], LEVEL_INFO, 500);
        }
    } else if ((Z80Ops::isPentagon || Z80Ops::isProfi)) {
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

void OSD::do_OSD(fabgl::VirtualKey KeytoESP, bool ALT, bool CTRL) {

    struct AYGuard {
        AYGuard()  { if (Config::audio_driver == 3) send_to_595(LOW(AY_Enable)); }
        ~AYGuard() { if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable)); }
    } ayGuard;

#if !PICO_RP2040
    // DS80 OSD palette guard.
    //
    // While OSD is open, the Z80/main loop is paused (suspended inside do_OSD()),
    // so EndFrame() never fires — the deferred profi_ds80_*_pending flags cannot
    // be applied here.  The HDMI ISR on core1 keeps scanning the (now frozen) DS80
    // framebuffer, so the Profi screen stays visible behind the dialog.
    //
    // Problem: OSD draws standard ZX colour bytes (indices 0..16) into the
    // framebuffer, but in DS80 mode a framebuffer byte indexes the DS80 packed-pair
    // conv_color table, not the standard ZX palette — so OSD bytes render as garbled
    // striped colours (the reported bug).
    //
    // Fix: applyProfiOSDPalette() makes the OSD render correctly, in one of two modes
    // chosen by the "OSD palette" menu toggle (applied live, no reboot):
    //   STD : restore the standard ZX conv_color snapshot → menu in TRUE ZX colours,
    //         Profi background re-interpreted through std palette (sacrificed).
    //   DS80: enable the Graphics-layer remap (Graphics8BitPalette::ds80_active +
    //         ds80_color_lut[]) so each ZX index → profi_pair_lookup[c][c] (solid Profi
    //         colour) → menu in Profi-palette colours, background fully correct.
    // Either way HDMI stays in DS80 mode and the change reverts on OSD close.
    //
    // Edge case: if a machine reset fires during OSD, ESPectrum::reset() clears DS80
    // state directly.  The dtor re-arms activation only if the machine is still DS80.
    struct DS80Guard {
        bool was_active;
        DS80Guard() : was_active(profi_ds80_active) {
            // Cancel any pending activation that arrived just before OSD opened
            // (avoids a spurious framebuffer-zeroing + geometry reset under the dialog).
            VIDEO::profi_ds80_activate_pending = false;
            if (was_active) {
                VIDEO::profi_ds80_osd_active = true;
                VIDEO::applyProfiOSDPalette();   // STD swap or DS80 remap (per toggle)
            }
        }
        ~DS80Guard() {
            if (VIDEO::profi_ds80_osd_active) {
                VIDEO::profi_ds80_osd_active = false;
                // CRITICAL: only restore the DS80 palette if HDMI is STILL in DS80 mode.
                // If a machine reset fired during OSD (e.g. switching to 48K/128K from a
                // menu), ESPectrum::reset() already called hdmi_set_profi_ds80_mode(false).
                // restoreProfiLivePalette() would call hdmi_set_profi_ds80_mode(true,…),
                // RE-ENABLING DS80 packed-pair scanout over a non-DS80 framebuffer →
                // shifted/garbled screen.  Skip it when we've left DS80.
                if (profi_ds80_active) {
                    VIDEO::restoreProfiLivePalette(); // back to live DS80 pair palette
                    // DS80 renderer only rewrites the 256 content bytes per row, so any
                    // side-padding the dialog drew over stays dirty — re-blacken it.
                    VIDEO::clearDS80Padding();
                } else {
                    // Left DS80 during OSD: drop the saved-palette flag without re-arming
                    // DS80, so a later DS80 session starts clean.
                    VIDEO::discardProfiOSDPaletteSnapshot();
                }
            }
            // Cancel any stray deferred flags from transitions during OSD.
            VIDEO::profi_ds80_activate_pending   = false;
            VIDEO::profi_ds80_deactivate_pending = false;
            // Reset-during-OSD: profi_ds80_active was cleared by ESPectrum::reset()
            // directly.  Re-arm activation only if the machine is still DS80.
            if (was_active && !profi_ds80_active && VIDEO::isProfiDS80()) {
                VIDEO::profi_ds80_activate_pending = true;
            }
        }
    } ds80Guard;
#endif

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

#if !PICO_RP2040
    // Alt+` (grave/tilde) or plain PrtScr (the Karabas-Pro hardware combo) —
    // toggle Profi extended keyboard mode (only in Profi arch)
    if (Config::arch == "Profi" && !CTRL &&
        ((ALT && (KeytoESP == fabgl::VK_GRAVEACCENT || KeytoESP == fabgl::VK_TILDE)) ||
         (!ALT && KeytoESP == fabgl::VK_PRINTSCREEN))) {
        Config::profi_ext_keys = !Config::profi_ext_keys;
        Config::save();
        osdCenteredMsg(Config::profi_ext_keys ? " XT keyboard ON  " : " XT keyboard OFF ", LEVEL_INFO, 500);
        return;
    }
#endif

#ifdef VGA_HDMI
    // Mode switches require a hard reset — heap fragmentation breaks runtime
    // framebuffer grow. Save the *old* vm to pending (loaded after reboot for
    // the rollback confirmation), write the new vm to main config, then reset.
    if (hkIdx == Config::HK_VIDMODE_60) { // HDMI 60Hz
        uint8_t &vm = 
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
        SELECT_VGA ? Config::vga_video_mode : 
#endif
        Config::hdmi_video_mode;
        if (vm == Config::VM_640x480_60) return;
        Config::savePendingVideoMode(); // captures old vm
        vm = Config::VM_640x480_60;
        Config::save();
        esp_hard_reset();
    } else
    if (hkIdx == Config::HK_VIDMODE_50) { // HDMI 50Hz
        uint8_t &vm =
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
         SELECT_VGA ? Config::vga_video_mode :
#endif
         Config::hdmi_video_mode;
        if (vm == Config::VM_640x480_50) return;
        Config::savePendingVideoMode(); // captures old vm
        vm = Config::VM_640x480_50;
        Config::save();
        esp_hard_reset();
    } else
#endif
    if (hkIdx == Config::HK_HW_INFO) { // Show mem info (Alt+F1)
            // Kept on RP2040 too (useful for debugging the tight heap); uses the
            // 640 B osd_info_buf there. The rest of the Hardware menu is still
            // removed on RP2040 — HWInfo is the only remaining osd_info_buf user.
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
        else if (hkIdx == Config::HK_LED_TOGGLE) { // Toggle LED indicators
            Config::ledIndicators = !Config::ledIndicators;
            if (!Config::ledIndicators) LED::clear();
            Config::save();
            osdCenteredMsg(Config::ledIndicators ? " LED indicators ON  " : " LED indicators OFF ", LEVEL_INFO, 500);
        } else
        if (hkIdx == Config::HK_POKE) { // Input Poke
            pokeDialog();
        } else
        if (hkIdx == Config::HK_NMI) { // NMI
            nmiAction();
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
                if (Config::arch == "Profi") {
                    reset_menu = MENU_RESETTO_PROFI[Config::lang];
                } else if ((Z80Ops::isPentagon || Z80Ops::isProfi)) {
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

                    if (Config::arch == "Profi") {
                        // Service ROM=1, TR-DOS=2, 128K=3, 48K=4
                        if (opt == 1) {
                            // Service ROM: boot SYS ROM (bank0), SYSEN=true (set by reset(0) for Profi).
                            ESPectrum::reset(0);
                        } else if (opt == 2) {
                            // TR-DOS: per UnrealSpeccy memory.cpp set_mode(RM_DOS) —
                            //   comp.flags |= CF_TRDOS;  comp.p7FFD |= 0x10;
                            // set_banks() then maps bank0 = (CF_TRDOS && p7FFD&0x10)
                            // ? DOS_ROM : ... → the TR-DOS ROM (bank1). The CPU is
                            // reset to PC=0 and TR-DOS cold-starts from its own ROM
                            // vector. Mirror exactly: reset into bank1, then assert
                            // trdos (CF_TRDOS) + romLatch=1 (p7FFD bit4).
                            ESPectrum::reset(1);
                            MemESP::romLatch = 1;
                            ESPectrum::trdos = true;
                        } else if (opt == 3) {
                            // 128K ROM: trdos=false, romLatch=0 → bank2
                            ESPectrum::reset(2);
                        } else if (opt == 4) {
                            // 48K/SOS ROM: trdos=false, romLatch=1 → bank3
                            ESPectrum::reset(3);
                            MemESP::romLatch = 1;
                        }
                    } else if ((Z80Ops::isPentagon || Z80Ops::isProfi) && Config::romSet == "128Kpg") {
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
                    } else if ((Z80Ops::isPentagon || Z80Ops::isProfi)) {
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
                    if (ext == "trd" || ext == "scl" || ext == "udi" || ext == "fdi" || ext == "td0" || ext == "pro") {
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
#if !PICO_RP2040
            // Profi is incompatible with Gigascreen (renderer geometry never
            // touches the prev-FB coherently) — same guard as the Video menu;
            // without it the Alt+PgUp toggle enabled it mid-Profi and the
            // render path SIGBUS-stormed (hw, PICO_DV).
            if (Z80Ops::isProfi) {
                osdCenteredMsg("Gigascreen: not available on Profi", LEVEL_WARN, 1500);
                return;
            }
            Config::gigascreen_onoff = (Config::gigascreen_onoff + 1) % 3; // Off -> On -> Auto -> Off
            bool want_on = (Config::gigascreen_onoff != 0);
            if (want_on && !Config::gigascreen_enabled &&
                !OSD::featureBudgetGate(Subsystems::FEAT_GIGASCREEN)) {
                // Denied / cancelled (a freeing reboot never returns) — stay Off.
                Config::gigascreen_onoff = 0;
                want_on = false;
            }
            if (want_on) {
                if (!Config::gigascreen_enabled) {
                    initGigascreenBlendLUT();
                    Config::gigascreen_enabled = true;
                    GsSubsys::request(true);
                    GsSubsys::apply();
                    if (!VIDEO::vga.prevFrameBuffer) {
                        // OOM — fall back to Off.
                        Config::gigascreen_enabled = false;
                        Config::gigascreen_onoff = 0;
                        VIDEO::gigascreen_enabled = false;
                        VIDEO::gigascreen_auto_countdown = 0;
                    } else {
                        VIDEO::InitPrevBuffer();
                    }
                }
                if (Config::gigascreen_enabled) {
                    VIDEO::gigascreen_enabled = (Config::gigascreen_onoff == 1);
                    VIDEO::gigascreen_auto_countdown = 0;
                }
            } else {
                // Off — release the 52 KB prev framebuffer.
                Config::gigascreen_enabled = false;
                VIDEO::gigascreen_enabled = false;
                VIDEO::gigascreen_auto_countdown = 0;
                GsSubsys::request(false);
            }
            std::string menu = Config::gigascreen_onoff == 1 ? OSD_GIGASCREEN_ON[Config::lang]
                             : Config::gigascreen_onoff == 2 ? OSD_GIGASCREEN_AUTO[Config::lang]
                             : OSD_GIGASCREEN_OFF[Config::lang];
            osdCenteredMsg(menu, LEVEL_INFO, 500);
            Config::save();
#endif
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
#if !PICO_RP2040 && ZIFI_NET_CLIENT
            // When networking is available, F5 first offers a location picker IN the
            // browser window (Local / Remote / Web Archives / Add Remote). The gate is
            // cheap (no blocking ESP round-trip); the network actions check the WiFi link
            // themselves. With networking off, fall straight through to the SD browser.
            // The label is the "root": backing out of the SD browser returns here (below)
            // rather than closing the OSD, so the locations chooser is always reachable.
            // On a fresh F5 press, restore the last browse location across all sources
            // (g_f5_restore); the goto re-entry skips it so back-out lands on the chooser.
            if (f5HasChooser()) g_f5_restore = true;
            f5_locations:
            if (f5HasChooser()) {
                if (!f5Locations()) {                // chose a non-Local action or cancelled
                    if (OSD::net_launch_close) OSD::net_launch_close = false;
                    OSD::net_close_all = false;      // Esc-close consumed → reset
                    if (VIDEO::OSD) OSD::drawStats();
                    return;
                }
                lastLocSet("L");                     // Local (SD) chosen → record as last location
                // fall through to the SD browser below.
            }
#endif
            menu_level = 0;
            menu_saverect = false;
            string mFile;
            string fname;
            string ext;
            bool fromZip = false;

            // Loop to allow re-opening fileDialog after ZIP cancel
            bool forcePopup = false;
            f5_retry:
#if !PICO_RP2040 && ZIFI_NET_CLIENT
            // From locations: show a ".." even at the SD root → returns "" → locations.
            OSD::fd_root_parent = f5HasChooser();
#endif
            mFile = fileDialog(FileUtils::ALL_Path, MENU_ALL_TITLE[Config::lang], DISK_ALLFILE, 52, 22);
#if !PICO_RP2040 && ZIFI_NET_CLIENT
            OSD::fd_root_parent = false;             // don't leak into other fileDialog uses
            // ".." at the SD root → locations chooser. Esc ("") just closes the browser
            // (as before) — it does NOT climb a level.
            if (mFile == "\x02UP") goto f5_locations;
#endif
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
                    // Lend the dormant Gigascreen prevFB to the extract's inflate
                    // buffers on butter-less boards (no-op on butter — palloc routes
                    // them to XIP PSRAM). Released right after the extract. The lease
                    // helper only exists where Gigascreen + the net arena do (RP2350).
#if !PICO_RP2040 && ZIFI_NET_CLIENT
                    NetArenaLease zipLease;
#endif
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
                    // Respect the Auto-start toggle: with auto-start off, force the
                    // "L" (load-only) key so flashload never runs the program — the
                    // machine lands at BASIC. WAV/MP3 always play inside LoadTape.
                    // Don't press Play here: leaving the tape STOPPED until the guest
                    // polls keeps F8 stats out of tape mode while it's merely mounted.
                    if (!Config::tape_autostart && mFile.size() > 0)
                        mFile[0] = 'L';
                    Tape::LoadTape(mFile);
                }
                else if (FileUtils::ifaceForExt(ext) == IFACE_BETA) {
                    // TR-DOS disk. Enter in the browser mounts into Drive A;
                    // F5 in the browser opens the slot picker, where Enter mounts
                    // into the focused slot and the popup stays open for more
                    // operations until Esc closes it back to the file dialog.
                    if (!fromZip) FileUtils::DSK_Path = FileUtils::ALL_Path;
                    if (forcePopup) {
                        Config::driveWP[0] = true;
                        diskSlotDialog(IFACE_BETA, 0, fname);
                        forcePopup = false;
                        goto f5_retry;
                    }
                    rvmWD1793InsertDisk(&ESPectrum::fdd, 0, fname);
                    if (ESPectrum::fdd.disk[0])
                        // TD0 is read-only (no write-back) → always WP regardless of the slot flag.
                        ESPectrum::fdd.disk[0]->writeprotect =
                            Config::driveWP[0]
#if !PICO_RP2040
                            || ESPectrum::fdd.disk[0]->IsTD0File
#endif
                            ;
                    Config::save();
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
                else if (ext == "rom" || ext == "bin") {
                    // ALF cartridge — lazy-mount from SD (no flash) and switch into ALF in place.
                    if (loadAlfCart(fname)) return;   // clean exit into the running machine
                }
#endif
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
#if !PICO_RP2040
                else if (ext == "dls") {
                    // GM.DLS soundbank — convert to a GMWB bank in CONFIG_DIR and flash
                    // it, the same pipeline as Audio→MIDI→GM.DLS→"Convert a .dls".
                    if (!fromZip) FileUtils::DLS_Path = FileUtils::ALL_Path;
                    string outBin = osdConvertDlsToBank(fname);
                    if (!outBin.empty()) {
                        // Enabling GM.DLS (mode 4) is the RAM-heavy engine → gate it
                        // through the SRAM budget manager, like the MIDI menu does.
                        if (Config::midi == 4 || OSD::featureBudgetGate(Subsystems::FEAT_MIDI)) {
                            uint8_t prev_midi = Config::midi;
                            Config::midi = 4;
                            Config::midi_bank = outBin;
                            Midi::enabled = prev_midi; Midi::deinit();
                            Midi::enabled = 4; Midi::init();
                            MidiSubsys::request(true);
                            Config::save();
                            // applyBankLive() loads THIS freshly-converted bank: live on PSRAM
                            // (no reboot), else false → it must be written to flash at early boot.
                            if (MidiSynth::applyBankLive()) {
                                osdCenteredMsg(MSG_MIDI_BANK_OK[Config::lang], LEVEL_OK, 2000);
                            } else if (OSD::msgDialog("DLS Wavetable", MSG_MIDI_BANK_INSTALL_Q[Config::lang]) == DLG_YES) {
                                osdCenteredMsg(MSG_MIDI_BANK_FLASHING[Config::lang], LEVEL_INFO, 3000);
                                OSD::esp_hard_reset();   // no PSRAM: provisionAtBoot writes it pre-video
                            }
                        }
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
                bool hasFdd = ((Z80Ops::isPentagon || Z80Ops::isProfi) || (Z80Ops::is128 && Z80Ops::isByte)
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
            OSD::drawVolumeBox();
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
            OSD::drawVolumeBox();
        } else if (hkIdx == Config::HK_HARD_RESET) { // Hard reset
            if (Config::ram_file != NO_RAM_FILE) {
                Config::ram_file = NO_RAM_FILE;
            }
            Config::last_ram_file = NO_RAM_FILE;
            if (Config::arch == "Profi") {
                // Profi hard reset = boot Service ROM (bank0, SYSEN), same as
                // Alt-F11 → "Service ROM". reset(0) sets trdos=true to hold SYSEN.
                ESPectrum::reset(0);
            } else
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
            // An online-archive file was downloaded to /tmp and launched: tear the
            // whole menu stack down so the freshly loaded program runs immediately.
            if (OSD::net_launch_close) { OSD::net_launch_close = false; if (VIDEO::OSD) OSD::drawStats(); return; }
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
                OSD::drawVolumeBox();
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
                                        // Auto-start off: force the "L" (load-only) key so
                                        // flashload never runs the program — land at BASIC,
                                        // same as the F5 file manager.
                                        if (!Config::tape_autostart && mFile.size() > 0)
                                            mFile[0] = 'L';
                                        Tape::LoadTape(mFile);
                                        // Auto-start: press Play automatically after a manual
                                        // mount. Skipped when flashload is on — there the loader
                                        // has already run the program during LoadTape, and the
                                        // runtime auto-start heuristic drives real-time loaders.
                                        if (Config::tape_autostart && !Config::flashload &&
                                            Tape::tapeStatus == TAPE_STOPPED &&
                                            Tape::tapeFileName != "none")
                                            Tape::Play();
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
                                else if (tap_num == 8) {
                                    // Auto-start (remember tape + auto-play on load/restore)
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string mnu_str = MENU_TAPE_AUTOSTART[Config::lang];
                                        mnu_str += MENU_YESNO[Config::lang];
                                        bool prev_opt = Config::tape_autostart;
                                        if (prev_opt) {
                                            mnu_str.replace(mnu_str.find("[Y",0),2,"[*");
                                            mnu_str.replace(mnu_str.find("[N",0),2,"[ ");
                                        } else {
                                            mnu_str.replace(mnu_str.find("[Y",0),2,"[ ");
                                            mnu_str.replace(mnu_str.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(mnu_str);
                                        if (opt2) {
                                            Config::tape_autostart = (opt2 == 1);
                                            if (Config::tape_autostart != prev_opt) {
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = FileUtils::fsMount ? 8 : 7;
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
                            betamenu += string(MENU_BETADISK_MODE[Config::lang]) + "\t"
                                      + (Config::betadisk ? "On" : "Off") + "\n";
                            for (uint8_t i = 0; i < 4; i++) {
                                string label = string("Drive ") + MENU_BETA_DRIVE_LETTERS[i];
                                string fname = ESPectrum::fdd.disk[i] ? ESPectrum::fdd.disk[i]->fname : "";
                                string row = formatSlotRow(label, fname, Config::driveWP[i], true);
                                if (!Config::betadisk) row = "\x01" + row;
                                betamenu += row + "\n";
                            }
                            {
                                string fm = MENU_BETADISK_FASTMODE[Config::lang];
                                string sl = MENU_BETADISK_SNDLED[Config::lang];
                                string rm = MENU_BETADISK_ROM[Config::lang];
                                string ab = MENU_BETADISK_AUTOBOOT[Config::lang];
                                if (!Config::betadisk) { fm = "\x01" + fm; sl = "\x01" + sl; rm = "\x01" + rm; ab = "\x01" + ab; }
                                betamenu += fm + sl + rm + ab;
                            }

                            // Save the backing rect only on the very first draw so the
                            // stack has exactly one entry to pop on Esc (otherwise the
                            // extra save'd fragment would show through). Repeat draws
                            // paint over the previous menu — it has fixed height, so
                            // no stale pixels remain.
                            menu_saverect = betaFirstDraw;
                            uint8_t dsk_num = menuRun(betamenu);
                            betaFirstDraw = false;
                            if (dsk_num == 1) {
                                // Mode toggle
                                Config::betadisk = !Config::betadisk;
                                if (!Config::betadisk && ESPectrum::trdos) {
                                    ESPectrum::trdos = false;
                                    MemESP::recoverPage0();
                                }
                                Config::save();
                                menu_curopt = 1;
                                continue;
                            }
                            else if (!Config::betadisk && dsk_num >= 2 && dsk_num <= 8) {
                                // Dimmed rows — ignore
                                menu_curopt = dsk_num;
                                continue;
                            }
                            else if (dsk_num >= 2 && dsk_num <= 5) {
                                // Per-drive submenu: Insert / Eject / Write Protect.
                                uint8_t slot = dsk_num - 2;
                                menu_saverect = true;
                                menu_curopt = 1;
                                while (1) {
                                    menu_level = 3;
                                    string drvmenu = MENU_BETADRIVE[Config::lang];
                                    drvmenu.replace(drvmenu.find("#",0),1,(string)" " + MENU_BETA_DRIVE_LETTERS[slot]);
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
                                                // TD0 is read-only (no write-back) → always WP.
                                                ESPectrum::fdd.disk[slot]->writeprotect =
                                                    Config::driveWP[slot]
#if !PICO_RP2040
                                                    || ESPectrum::fdd.disk[slot]->IsTD0File
#endif
                                                    ;
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
                                            // TD0 is read-only (no write-back) → stays WP even if toggled off.
                                            ESPectrum::fdd.disk[slot]->writeprotect =
                                                Config::driveWP[slot]
#if !PICO_RP2040
                                                || ESPectrum::fdd.disk[slot]->IsTD0File
#endif
                                                ;
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
                            else if (dsk_num == 6) {
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
                                            rvmWD1793UpdateFastmode(&ESPectrum::fdd);
                                            Config::save();
                                        }
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
                                // Disk Sound & LED
                                menu_level = 3;
                                menu_curopt = Config::trdosSoundLed + 1;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_SOUNDLED[Config::lang];
                                    menu += MENU_SOUNDLED_SEL[Config::lang];
                                    int mpos = -1;
                                    int idx = 0;
                                    while ((mpos = menu.find("[ ]", mpos + 1)) != (int)string::npos) {
                                        if (idx == Config::trdosSoundLed)
                                            menu.replace(mpos, 3, "[*]");
                                        idx++;
                                    }
                                    uint8_t prev = Config::trdosSoundLed;
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        Config::trdosSoundLed = opt2 - 1;
                                        if (Config::trdosSoundLed != prev)
                                            Config::save();
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
                            else if (dsk_num == 8) {
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
                                            // 5.03 / 5.04TM are read-only overlays over 5.05D applied on
                                            // the fly by MemESP (RomOverlay.h) — bind immediately, no
                                            // reboot, on every board.
                                            const uint8_t* base = gb_rom_4_trdos_505d;
                                            const uint8_t* ov = nullptr;
                                            switch (Config::trdosBios) {
                                                case 0: ov = gb_overlay_trdos_503;   break;
                                                case 1: ov = gb_overlay_trdos_504tm; break;
                                                case 3: base = gb_rom_4_trdos_custom; break;
                                                default: break;
                                            }
                                            MemESP::rom[4].assign_rom(base);
                                            MemESP::registerOverlay(gb_rom_4_trdos_505d, ov);
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 8;
                                        menu_level = 2;
                                        menu_saverect = false;
                                        break;
                                    }
                                }
                            }
                            else if (dsk_num == 9) {
                                // Auto-boot: inject a "boot" file into TRD/SCL images
                                // that lack one so TR-DOS autostarts after mounting.
                                menu_level = 3;
                                menu_curopt = Config::trdosAutoBoot ? 1 : 2;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_AUTOBOOT[Config::lang];
                                    menu += MENU_YESNO[Config::lang];
                                    bool prev = Config::trdosAutoBoot;
                                    if (prev) {
                                        menu.replace(menu.find("[Y",0),2,"[*");
                                        menu.replace(menu.find("[N",0),2,"[ ");
                                    } else {
                                        menu.replace(menu.find("[Y",0),2,"[ ");
                                        menu.replace(menu.find("[N",0),2,"[*");
                                    }
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        bool nv = (opt2 == 1);
                                        if (nv != prev) {
                                            Config::trdosAutoBoot = nv;
                                            Config::save();
                                        }
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 9;
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
#if !PICO_RP2040
                                            // Enabling DivMMC/DivIDE/DivSD from Off — check SRAM budget.
                                            // Set esxdos to the chosen variant first so a freeing reboot
                                            // preserves it; restore on ALLOW so the side-effects below run.
                                            if (newval && Config::esxdos == 0) {
                                                uint8_t oldEsx = Config::esxdos;
                                                Config::esxdos = newval;
                                                bool ok = OSD::featureBudgetGate(Subsystems::FEAT_DIVMMC);
                                                Config::esxdos = oldEsx;
                                                if (!ok) { menu_curopt = sub; menu_saverect = false; continue; }
                                            }
#endif
                                            if (newval && Config::mb02) {
                                                Config::mb02 = 0;
                                                MB02::init();
                                                OSD::osdCenteredMsg("MB-02+ disabled", LEVEL_WARN, 2000);
                                            }
#if !PICO_RP2040
                                            if (newval && Config::zcontroller) {
                                                Config::zcontroller = false;
                                                DivMMC::zc_shutdown();
                                                OSD::osdCenteredMsg("Z-Controller disabled", LEVEL_WARN, 2000);
                                            }
#ifdef USE_GS
                                            // DivIDE conflicts with General Sound on ports 0xB3/0xBB
                                            if (newval == 2 && Config::gs_enabled) {
                                                Config::gs_enabled = 0;
                                                OSD::osdCenteredMsg("General Sound disabled", LEVEL_WARN, 2000);
                                            }
#endif
#endif
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
                                // MB-02+ and Profi both claim the upper MemESP RAM
                                // pages (Profi forces ~96 KB of SRAM pages for its
                                // 1024K/DS80 working set); enabling MB-02+ on Profi
                                // corrupts that and the machine fails to boot. Refuse.
                                if (newval && Config::arch == "Profi") {
                                    OSD::osdCenteredMsg("MB-02+ not available on Profi", LEVEL_WARN, 2000);
                                    menu_curopt = 1;
                                    continue;
                                }
                                Config::mb02 = newval;
                                if (Config::mb02 && Config::esxdos) {
                                    Config::esxdos = 0;
                                    DivMMC::init();
                                    OSD::osdCenteredMsg("esxDOS disabled", LEVEL_WARN, 2000);
                                }
#if !PICO_RP2040
                                if (Config::mb02 && Config::zcontroller) {
                                    Config::zcontroller = false;
                                    DivMMC::zc_shutdown();
                                    OSD::osdCenteredMsg("Z-Controller disabled", LEVEL_WARN, 2000);
                                }
#endif
                                MB02::init();
                                if (Config::mb02 && !MB02::enabled) {
                                    OSD::osdCenteredMsg("MB-02+: not enough memory", LEVEL_ERROR, 2000);
                                    Config::mb02 = 0;
                                }
                                // Restore the last-used disks remembered in the
                                // mounts file. loadDiskMounts() skips MB-02 disks
                                // while the interface is off, so after a reboot
                                // taken with MB-02 disabled (e.g. switched to Profi
                                // and back) mb02_fdd is empty — re-mount them now.
                                if (MB02::enabled)
                                    Config::loadMb02DiskMounts();
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
                                // Sound & LED selector: Off / LED / Sound / Sound+LED.
                                menu_level = 3;
                                menu_curopt = Config::mb02SoundLed + 1;
                                menu_saverect = true;
                                while (1) {
                                    string menu = MENU_SOUNDLED[Config::lang];
                                    menu += MENU_SOUNDLED_SEL[Config::lang];
                                    int mpos = -1;
                                    int idx = 0;
                                    while ((mpos = menu.find("[ ]", mpos + 1)) != (int)string::npos) {
                                        if (idx == Config::mb02SoundLed)
                                            menu.replace(mpos, 3, "[*]");
                                        idx++;
                                    }
                                    uint8_t prev = Config::mb02SoundLed;
                                    uint8_t opt2 = menuRun(menu);
                                    if (opt2) {
                                        Config::mb02SoundLed = opt2 - 1;
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
#if !PICO_RP2040
                    else if (FileUtils::fsMount && stor_num == 5) { // Z-Controller
                        menu_saverect = true;
                        menu_curopt = 1;
                        while (1) {
                            menu_level = 2;
                            string zmenu = "Z-Controller\n";
                            zmenu += string(MENU_YESNO[Config::lang]);
                            if (Config::zcontroller) {
                                zmenu.replace(zmenu.find("[Y",0),2,"[*");
                                zmenu.replace(zmenu.find("[N",0),2,"[ ");
                            } else {
                                zmenu.replace(zmenu.find("[Y",0),2,"[ ");
                                zmenu.replace(zmenu.find("[N",0),2,"[*");
                            }
                            uint8_t opt = menuRun(zmenu);
                            if (opt == 0) {
                                menu_curopt = 5;
                                menu_level = 1;
                                break;
                            }
                            bool newval = (opt == 1);
                            if (newval == Config::zcontroller) {
                                menu_curopt = opt;
                                menu_saverect = false;
                                continue;
                            }
                            // Budget-gate when turning Z-Controller on (~0.5 KB sector buffer).
                            if (newval && !OSD::featureBudgetGate(Subsystems::FEAT_ZCONTROLLER)) {
                                menu_curopt = opt;
                                menu_saverect = false;
                                continue;   // declined to free → leave off
                            }
                            Config::zcontroller = newval;
                            if (newval) {
                                if (Config::esxdos) {
                                    Config::esxdos = 0;
                                    DivMMC::init();
                                    OSD::osdCenteredMsg("esxDOS disabled", LEVEL_WARN, 2000);
                                }
                                if (Config::mb02) {
                                    Config::mb02 = 0;
                                    MB02::init();
                                    OSD::osdCenteredMsg("MB-02+ disabled", LEVEL_WARN, 2000);
                                }
                                DivMMC::zc_init();
                            } else {
                                DivMMC::zc_shutdown();
                            }
                            Config::save();
                            ESPectrum::reset();
                            return;
                        }
                    }
                    else if (FileUtils::fsMount && stor_num == 7) { // Snapshot
#else
                    else if (FileUtils::fsMount && stor_num == 3) { // Snapshot
#endif
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
                                menu_curopt = 7;
#else
                                menu_curopt = 3;
#endif
                                menu_level = 1;
                                break;
                            }
                        }
                    }
#if !PICO_RP2040
                    else if (FileUtils::fsMount && stor_num == 6) { // IDE/HDD
                        static const char* ide_modes[] = { "OFF", "NEMO", "PROFI" };
                        menu_saverect = true;
                        menu_curopt = 1;
                        while (1) {
                            menu_level = 2;
                            // Root: Scheme row + hd0/hd1 rows (shown when a scheme is active).
                            string menu = MENU_IDE_TITLE[Config::lang];
                            menu += string(MENU_IDE_SCHEME[Config::lang]) + "\t" + ide_modes[Config::ide_scheme <= 2 ? Config::ide_scheme : 0] + "\n";
                            bool showSlots = (Config::ide_scheme != 0);
                            if (showSlots) {
                                menu += formatSlotRow("hd0", Config::ide_image[0], false, false);
                                menu += "\n";
                                menu += formatSlotRow("hd1", Config::ide_image[1], false, false);
                                menu += "\n";
                                menu += MENU_IDE_CREATE[Config::lang];
                            }
                            uint8_t opt = menuRun(menu);
                            if (opt == 1) {
                                // Scheme submenu (OFF / NEMO / PROFI).
                                menu_level = 3;
                                menu_curopt = Config::ide_scheme + 1;
                                menu_saverect = true;
                                while (1) {
                                    string smenu = MENU_IDE_TITLE[Config::lang];
                                    for (int i = 0; i < 3; i++) {
                                        smenu += (i == Config::ide_scheme) ? "[*] " : "[ ] ";
                                        smenu += ide_modes[i];
                                        smenu += "\n";
                                    }
                                    uint8_t sub = menuRun(smenu);
                                    if (sub) {
                                        uint8_t newval = sub - 1;
                                        if (newval != Config::ide_scheme) {
                                            uint8_t prevScheme = Config::ide_scheme;
                                            // Mutually exclusive with esxDOS DivMMC/DivIDE
                                            // (shared ports 0xEB/0xE7/0xA3 etc.).
                                            if (newval && Config::esxdos) {
                                                Config::esxdos = 0;
                                                DivMMC::init();
                                                OSD::osdCenteredMsg("esxDOS disabled", LEVEL_WARN, 2000);
                                            }
                                            Config::ide_scheme = newval;
                                            // Budget-gate when turning IDE on from off (~3.4 KB buffers).
                                            // Set scheme first so the user's NEMO/PROFI choice survives
                                            // a free-and-reboot inside the gate.
                                            bool ok = true;
                                            if (prevScheme == 0 && newval != 0)
                                                ok = OSD::featureBudgetGate(Subsystems::FEAT_IDE);
                                            if (!ok) {
                                                Config::ide_scheme = prevScheme;   // declined → stay off
                                            } else {
                                                IDE::init();
                                                IdeSubsys::syncFromState();
                                                Config::save();
                                            }
                                            menu_curopt = sub;
                                            menu_saverect = false;
                                        }
                                        menu_curopt = sub;
                                        menu_saverect = false;
                                    } else {
                                        menu_curopt = 1;
                                        break;
                                    }
                                }
                            } else if ((opt == 2 || opt == 3) && showSlots) {
                                // hd0 / hd1 submenu (Insert / Eject).
                                uint8_t slot = (opt == 2) ? 0 : 1;
                                menu_saverect = true;
                                menu_curopt = 1;
                                while (1) {
                                    menu_level = 3;
                                    char title[8]; snprintf(title, sizeof(title), "hd%u\n", (unsigned)slot);
                                    string drvmenu = title;
                                    drvmenu += MENU_ESX_INSERT[Config::lang];
                                    drvmenu += MENU_ESX_EJECT[Config::lang];
                                    // Geometry row: shows effective C/H/S, LBA and size; "auto" when no override.
                                    {
                                        char geo[48];
                                        if (IDE::isCD(slot)) {
                                            // ATAPI CD-ROM: no CHS geometry; show type + size.
                                            uint32_t mb = (uint32_t)(((uint64_t)IDE::sizeBytes(slot)) / (1024*1024));
                                            snprintf(geo, sizeof(geo), "Type\tCD-ROM\n");
                                            drvmenu += geo;
                                            char szr[48];
                                            snprintf(szr, sizeof(szr), "Size\t%u MB (ISO)\n", (unsigned)mb);
                                            drvmenu += szr;
                                        } else {
                                        uint16_t oc=Config::ide_chs[slot][0], oh=Config::ide_chs[slot][1], os=Config::ide_chs[slot][2];
                                        uint16_t C=IDE::geomC(slot), H=IDE::geomH(slot), S=IDE::geomS(slot);
                                        bool over = (oc&&oh&&os);
                                        bool profi = (IDE::scheme == IDE::PROFI);
                                        if (C && H && S)
                                            // Profi locks H=16/S=16 (BIOS CHS addressing); only C is user-editable.
                                            snprintf(geo, sizeof(geo), "CHS\t%u/%u/%u%s\n", C, H, S,
                                                     profi ? " (Profi: C only)" : (over?"":" (auto)"));
                                        else
                                            snprintf(geo, sizeof(geo), "CHS\t<empty>\n");
                                        drvmenu += geo;
                                        uint32_t lba = IDE::geomLBA(slot);
                                        char lbar[48];
                                        snprintf(lbar, sizeof(lbar), "LBA\t%u (%u MB)\n", (unsigned)lba, (unsigned)(((uint64_t)lba*512)/(1024*1024)));
                                        drvmenu += lbar;
                                        }
                                    }
                                    uint8_t opt2 = menuRun(drvmenu);
                                    if (opt2 == 1) {
                                        menu_saverect = true;
                                        string mFile = fileDialog(FileUtils::IMG_Path, MENU_IDE_IMG_TITLE[Config::lang], DISK_IMGFILE, 26, 15);
                                        if (mFile != "") {
                                            string fname = FileUtils::IMG_Path + mFile.substr(1);
                                            if (FileUtils::getLCaseExt(fname) == "zip") {
                                                string zipFname = ZipExtract::extract(fname, DISK_IMGFILE);
                                                if (zipFname.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); break; }
                                                if (zipFname == "\x1b") break;
                                                fname = zipFname;
                                            }
                                            Config::ide_image[slot] = fname;
                                            IDE::init();
                                            Config::save();
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        }
                                    } else if (opt2 == 2) {
                                        Config::ide_image[slot].clear();
                                        IDE::init();
                                        Config::save();
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    } else if (opt2 == 3 && !IDE::isCD(slot)) {
                                        // Edit CHS override. Empty input = auto-detect (0/0/0).
                                        // (Skipped for ATAPI CD-ROM — geometry N/A.)
                                        // Inline-edit on the CHS row (row index 3 within submenu).
                                        int ex = OSD::x + (1 + 4) * OSD_FONT_W;     // after "CHS\t"
                                        int ey = OSD::y + 1 + 3 * OSD_FONT_H;
                                        if (IDE::scheme == IDE::PROFI) {
                                            // Profi locks H=16/S=16 for BIOS CHS addressing; only the
                                            // cylinder count is meaningful. Edit C alone (empty = auto).
                                            char cur[8];
                                            snprintf(cur, sizeof(cur), "%u", IDE::geomC(slot));
                                            string in = OSD::inlineTextEdit(ex, ey, 6, string(cur));
                                            if (in != "\x1B") {
                                                unsigned c=0;
                                                if (in.empty()) { c=0; }   // auto
                                                if (in.empty() || sscanf(in.c_str(), "%u", &c)==1) {
                                                    if (c==0) {            // auto-detect
                                                        Config::ide_chs[slot][0]=0;
                                                        Config::ide_chs[slot][1]=0;
                                                        Config::ide_chs[slot][2]=0;
                                                    } else {               // keep H=16 S=16
                                                        Config::ide_chs[slot][0]=(uint16_t)c;
                                                        Config::ide_chs[slot][1]=16;
                                                        Config::ide_chs[slot][2]=16;
                                                    }
                                                    IDE::init();
                                                    Config::save();
                                                } else {
                                                    OSD::osdCenteredMsg("Cylinders: number or empty", LEVEL_WARN, 2000);
                                                }
                                            }
                                        } else {
                                        char cur[20];
                                        if (Config::ide_chs[slot][0] && Config::ide_chs[slot][1] && Config::ide_chs[slot][2])
                                            snprintf(cur, sizeof(cur), "%u/%u/%u",
                                                     Config::ide_chs[slot][0], Config::ide_chs[slot][1], Config::ide_chs[slot][2]);
                                        else
                                            snprintf(cur, sizeof(cur), "%u/%u/%u",
                                                     IDE::geomC(slot), IDE::geomH(slot), IDE::geomS(slot));
                                        string in = OSD::inlineTextEdit(ex, ey, 14, string(cur));
                                        if (in != "\x1B") {
                                            unsigned c=0,h=0,s=0;
                                            if (in.empty()) { c=h=s=0; }   // auto
                                            if (in.empty() || sscanf(in.c_str(), "%u/%u/%u", &c,&h,&s)==3) {
                                                // Sanity: H<=16, S<=63 (ATA); 0/0/0 allowed (=auto)
                                                if ((c==0&&h==0&&s==0) || (h>=1&&h<=16&&s>=1&&s<=63&&c>=1)) {
                                                    Config::ide_chs[slot][0]=(uint16_t)c;
                                                    Config::ide_chs[slot][1]=(uint16_t)h;
                                                    Config::ide_chs[slot][2]=(uint16_t)s;
                                                    IDE::init();
                                                    Config::save();
                                                } else {
                                                    OSD::osdCenteredMsg("Invalid CHS (H<=16 S<=63)", LEVEL_WARN, 2000);
                                                }
                                            } else {
                                                OSD::osdCenteredMsg("Format: C/H/S", LEVEL_WARN, 2000);
                                            }
                                        }
                                        }
                                        menu_saverect = false;
                                        menu_curopt = 3;
                                    } else if (opt2 == 4) {
                                        // LBA row is informational — Enter does nothing,
                                        // stay on this hd submenu with the cursor on LBA.
                                        menu_saverect = false;
                                        menu_curopt = 4;
                                    } else {
                                        menu_curopt = opt;
                                        break;
                                    }
                                }
                            } else if (opt == 4 && showSlots) {
                                // "Create empty image": pick slot → size → name, then create+mount.
                                // Nested Esc-driven loops mirror the Scheme submenu so each level
                                // backs out to its parent (slot picker → IDE/HDD root) on Esc.
                                static const struct { const char* label; uint32_t mb; } ide_presets[] = {
                                    { "10 MB\n", 10 }, { "32 MB\n", 32 }, { "64 MB\n", 64 }, { "128 MB\n", 128 }
                                };
                                menu_level = 3;
                                menu_curopt = 1;
                                menu_saverect = true;
                                bool created = false;
                                while (!created) {
                                    // Level 3 — target slot.
                                    string slmenu = MENU_IDE_CREATE[Config::lang];
                                    slmenu += "hd0\n";
                                    slmenu += "hd1\n";
                                    uint8_t slsel = menuRun(slmenu);
                                    if (slsel != 1 && slsel != 2) { menu_curopt = 4; break; }  // Esc → IDE/HDD
                                    uint8_t slot = slsel - 1;
                                    // Level 4 — size preset.
                                    menu_level = 4;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string szmenu = MENU_IDE_CREATE_SIZE[Config::lang];
                                        for (auto &p : ide_presets) szmenu += p.label;
                                        uint8_t sz = menuRun(szmenu);
                                        if (sz < 1 || sz > 4) break;   // Esc → back to slot picker
                                        uint32_t mb = ide_presets[sz - 1].mb;
                                        // Name — inline-edit over the selected size row.
                                        uint8_t vrow = (uint8_t)(sz - OSD::begin_row + 1);
                                        int ex = OSD::x + 1 * OSD_FONT_W;
                                        int ey = OSD::y + 1 + vrow * OSD_FONT_H;
                                        string name = OSD::inlineTextEdit(ex, ey, 18, "new");
                                        menu_saverect = false;
                                        menu_curopt = sz;
                                        if (name == "\x1B") continue;   // Esc on name → stay in size menu
                                        while (!name.empty() && name.back()  == ' ') name.pop_back();
                                        while (!name.empty() && name.front() == ' ') name.erase(name.begin());
                                        if (name.empty()) continue;
                                        if (FileUtils::getLCaseExt(name) != "hdd") name += ".hdd";
                                        string path = FileUtils::IMG_Path + name;
                                        FILINFO fno;
                                        if (f_stat(path.c_str(), &fno) == FR_OK) {
                                            OSD::osdCenteredMsg("File already exists", LEVEL_WARN, 2000);
                                            continue;
                                        }
                                        OSD::osdCenteredMsg("Creating HDD...", LEVEL_INFO, 0);
                                        bool ok = IDE::createImage(path.c_str(), mb, ide_create_progress);
                                        if (!ok) { OSD::osdCenteredMsg("Create failed", LEVEL_WARN, 2000); continue; }
                                        Config::ide_image[slot] = path;
                                        IDE::init();
                                        Config::save();
                                        OSD::osdCenteredMsg("HDD image created", LEVEL_INFO, 1500);
                                        // Success: unwind size + slot menus back to the IDE/HDD root.
                                        VIDEO::SaveRect.restore_last();   // pop size-menu rect
                                        VIDEO::SaveRect.restore_last();   // pop slot-menu rect
                                        created = true;
                                        menu_curopt = 4;
                                        break;
                                    }
                                    // Back at the slot-picker level (Esc from size, or after create).
                                    menu_level = 3;
                                    menu_saverect = false;
                                }
                                menu_saverect = false;
                            } else {
                                menu_curopt = 6;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
#endif
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
                    // Buffer owns the test: the SPI fallback needs GS's region + the
                    // minimum arena + a usable swap pool, and a large Murmuzavr page
                    // count must NOT count against it (those pages yield to GS via
                    // Buffer::pageBudgetSpi) — measuring MEM_PG_CNT * MEM_PG_SZ against
                    // the chip hid the whole GS row at 8 MB+ of extended RAM.
                    bool gs_avail = Buffer::gsPsramAvailable();
                    if (gs_avail) {
                        // Insert GS before "Audio Driver" (second-to-last item, before Volume Boost)
                        size_t last_nl = audio_menu.rfind('\n', audio_menu.size() - 2);
                        if (last_nl != string::npos && last_nl > 0) {
                            size_t insert_pos = audio_menu.rfind('\n', last_nl - 1);
                            if (insert_pos != string::npos) {
                                audio_menu.insert(insert_pos + 1, MENU_AUDIO_GS_ITEM[Config::lang]);
                            }
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
                                        TurboSubsys::request(Config::turbosound != 0);
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
                                static const char covox_tags[3] = {'N','F','D'};
                                for (uint8_t i = 0; i < 3; i++) {
                                    const char tag[3] = {'[', covox_tags[i], 0};
                                    menu.replace(menu.find(tag,0),2, i == prev ? "[*" : "[ ");
                                }
                                uint8_t opt2 = menuRun(menu);
                                if (opt2) {
                                    bool wasOn = (prev != 0 || Config::soundriveEnabled());
                                    Config::covox = opt2 - 1;
                                    bool nowOn = (Config::covox != 0 || Config::soundriveEnabled());
                                    // Budget-gate when the shared Covox/SounDrive buffer (~2 KB)
                                    // transitions from not-needed to needed. No budget
                                    // manager on RP2040 — enable directly.
#if !PICO_RP2040
                                    if (!wasOn && nowOn &&
                                        !OSD::featureBudgetGate(Subsystems::FEAT_COVOX)) {
                                        Config::covox = prev;   // declined → revert
                                    } else
#endif
                                    if (Config::covox != prev) {
                                        Config::save();
                                        CovoxSubsys::request(Config::covox != 0 || Config::soundriveEnabled());
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
                        else if (options_num == 5) { // SounDrive
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string menu = MENU_SOUNDRIVE[Config::lang];
                                uint8_t prev = Config::soundrive;
                                menu.replace(menu.find("[A",0),2, prev == 2 ? "[*" : "[ ");
                                menu.replace(menu.find("[O",0),2, prev == 1 ? "[*" : "[ ");
                                menu.replace(menu.find("[F",0),2, prev == 0 ? "[*" : "[ ");
                                uint8_t opt2 = menuRun(menu);
                                if (opt2) {
                                    static const uint8_t sd_map[3] = {2, 1, 0}; // menu row → mode
                                    bool wasOn = (Config::covox != 0 || Config::soundriveEnabled());
                                    Config::soundrive = (opt2 <= 3) ? sd_map[opt2 - 1] : prev;
                                    bool nowOn = (Config::covox != 0 || Config::soundriveEnabled());
                                    // Shared Covox/SounDrive buffer (~2 KB): gate on off→on.
                                    // No budget manager on RP2040 — enable directly.
#if !PICO_RP2040
                                    if (!wasOn && nowOn &&
                                        !OSD::featureBudgetGate(Subsystems::FEAT_COVOX)) {
                                        Config::soundrive = prev;   // declined → revert
                                    } else
#endif
                                    if (Config::soundrive != prev) {
                                        Config::save();
                                        CovoxSubsys::request(Config::covox != 0 || Config::soundriveEnabled());
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
#if !PICO_RP2040
                        else if (options_num == 6) {
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
                                    bool want_saa = (opt2 == 1);
                                    // Budget-gate when turning SAA1099 on from off (~2 KB).
                                    if (want_saa && !prev_saa &&
                                        !OSD::featureBudgetGate(Subsystems::FEAT_SAA)) {
                                        // declined to free → leave SAA off
                                    } else {
                                        Config::SAA1099 = want_saa;
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
                                            SaaSubsys::request(Config::SAA1099);
                                        }
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
                        else if (options_num == 7) {
                            menu_level = 2;
                            menu_curopt = 1;
                            bool midiFirstDraw = true;   // save the rect only on the first draw;
                            while (1) {                  // redraws (after gate/submenu) must NOT re-save → no duplicate menu
                                menu_level = 2;
                                string midi_menu = MENU_MIDI[Config::lang];
                                uint8_t prev_midi = Config::midi;
                                midi_menu.replace(midi_menu.find("[O",0),2, prev_midi == 0 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[A",0),2, prev_midi == 1 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[S",0),2, prev_midi == 2 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[W",0),2, prev_midi == 3 ? "[*" : "[ ");
                                midi_menu.replace(midi_menu.find("[G",0),2, prev_midi == 4 ? "[*" : "[ ");
                                menu_saverect = midiFirstDraw;
                                uint8_t opt2 = menuRun(midi_menu);
                                midiFirstDraw = false;
                                if (opt2 >= 1 && opt2 <= 5) {
#if !PICO_RP2040
                                    // GM.DLS (mode 4) is the RAM-heavy MIDI engine: gate it through
                                    // the SRAM budget manager so a tight machine (Profi/m1p2) offers
                                    // heavy features to free — including Profi itself (→ Pentagon) —
                                    // instead of OOMing at allocation time. ALLOW → true (fits as-is);
                                    // user frees + applies → reboots (never returns); decline → false.
                                    if ((opt2 - 1) == 4 && Config::midi != 4 &&
                                        !OSD::featureBudgetGate(Subsystems::FEAT_MIDI)) {
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                        continue;   // declined / not enough — leave MIDI unchanged
                                    }
#endif
                                    Config::midi = opt2 - 1;
                                    if (Config::midi != prev_midi) {
                                        Midi::enabled = prev_midi;
                                        Midi::deinit();
                                        Midi::enabled = Config::midi;
                                        if (Midi::enabled)
                                            Midi::init();
                                        Config::save();
                                        MidiSubsys::request(Config::midi != 0);
#if defined(MIDI_TX_PIN) && defined(LOAD_WAV_PIO) && (LOAD_WAV_PIO == MIDI_TX_PIN)
                                        if ((Config::midi == 1 || Config::midi == 2) && Config::real_player)
                                            osdCenteredMsg(MSG_MIDI_PIN_CONFLICT[Config::lang], LEVEL_WARN, 3000);
#endif
                                    }
                                    // Software MIDI selected — open preset submenu
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
                                            SoftSynth::preset = Config::midi_synth_preset;
                                            Config::save();
                                            VIDEO::SaveRect.restore_last();
                                        }
                                        menu_level = 2;
                                        menu_curopt = opt2;
                                        menu_saverect = false;
                                    }
                                    // GM.DLS wavetable selected. The bank is written to
                                    // flash at EARLY BOOT (single core, no video) — never
                                    // here, where core1/HDMI would freeze. So any flash
                                    // write is triggered by REBOOT; the boot path does it.
                                    if (Config::midi == 4) {
                                        // ALF cartridges stream from SD (AlfCart) and no longer occupy
                                        // this flash region, so GM.DLS and a loaded cart coexist — no
                                        // unload prompt needed.
                                        {
                                            // Let the user pick which instrument set (.bin bank) to use
                                            // when SD holds more than one. The choice is applied to
                                            // Config::midi_bank only TENTATIVELY: it is committed (saved)
                                            // only if the user confirms the flash below, and reverted to
                                            // `prevBank` otherwise — so declining never changes the bank
                                            // (and never triggers a flash on the next boot).
                                            std::vector<std::string> bankPaths, bankNames;
                                            size_t nBanks = MidiSynth::scanBanks(bankPaths, bankNames);
                                            string prevBank = Config::midi_bank;
                                            // Always show the picker: row 1 converts any .dls on the card
                                            // into a bank (gm_bank.bin in CONFIG_DIR), the rest are the
                                            // banks already present. The choice is applied to
                                            // Config::midi_bank only TENTATIVELY: committed on flash-confirm
                                            // below, reverted to `prevBank` otherwise.
                                            menu_level = 3;
                                            menu_curopt = 1;
                                            menu_saverect = true;
                                            string bankMenu = MENU_MIDI_BANK_TITLE[Config::lang];
                                            bankMenu += string(MENU_MIDI_CONVERT_DLS[Config::lang]) + "\n";  // row 1
                                            for (size_t b = 0; b < nBanks; b++) {
                                                bool cur = (Config::midi_bank == bankPaths[b]);
                                                bankMenu += string(cur ? "[*] " : "[ ] ") + bankNames[b] + "\n";
                                            }
                                            uint8_t optB = menuRun(bankMenu);
                                            menu_level = 2;
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                            VIDEO::SaveRect.restore_last();
                                            if (optB == 1) {
                                                // Convert a .dls -> <stem>.bin in CONFIG_DIR, then select it.
                                                // Streams the .dls from SD (no big RAM buffer); runs on core0
                                                // at runtime — the actual flash install still happens at the
                                                // next boot via MidiSynth::provisionAtBoot().
                                                string mFile = fileDialog(FileUtils::DLS_Path, MENU_DLS_TITLE[Config::lang], DISK_DLSFILE, 51, 22);
                                                if (mFile != "") {
                                                    mFile.erase(0, 1);
                                                    string outBin = osdConvertDlsToBank(FileUtils::DLS_Path + mFile);
                                                    if (!outBin.empty())
                                                        Config::midi_bank = outBin;   // tentative; committed on flash-confirm below
                                                }
                                            } else if (optB >= 2 && (size_t)(optB - 1) <= nBanks) {
                                                Config::midi_bank = bankPaths[optB - 2];   // tentative
                                            }

                                            bool haveSd = MidiSynth::sdBankAvailable();
                                            if (!haveSd) {
                                                // No SD bank: keep an already-bound (flash) bank, else nothing.
                                                if (MidiSynth::bankReady()) {
                                                    if (Config::midi_bank != prevBank) Config::save();
                                                    osdCenteredMsg(MSG_MIDI_BANK_OK[Config::lang], LEVEL_OK, 2000);
                                                } else {
                                                    Config::midi_bank = prevBank;
                                                    osdCenteredMsg(MSG_MIDI_BANK_MISSING[Config::lang], LEVEL_WARN, 3000);
                                                }
                                            } else if (MidiSynth::applyBankLive()) {
                                                // Applied without a reboot: PSRAM boards load the bank live,
                                                // and a flash bank that is already current just rebinds.
                                                Config::save();
                                                osdCenteredMsg(MSG_MIDI_BANK_OK[Config::lang], LEVEL_OK, 2000);
                                            } else if (OSD::msgDialog("DLS Wavetable",
                                                                      MSG_MIDI_BANK_INSTALL_Q[Config::lang]) == DLG_YES) {
                                                // No PSRAM and the flash bank differs → it must be written at
                                                // EARLY BOOT (single core, pre-video). Commit + reboot.
                                                Config::save();
                                                osdCenteredMsg(MSG_MIDI_BANK_FLASHING[Config::lang], LEVEL_INFO, 3000);
                                                OSD::esp_hard_reset();
                                            } else {
                                                Config::midi_bank = prevBank;   // declined -> revert + restore
                                                MidiSynth::init();
                                            }
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
#ifdef USE_GS
                        else if (gs_avail && options_num == 8) { // General Sound
                            static const char* GS_CLOCK_NAMES[5] = {"12 MHz", "13 MHz", "14 MHz", "20 MHz", "24 MHz"};
                            menu_level = 2;
                            menu_curopt = 1;
                            bool gsFirstDraw = true;
                            while (1) {
                                string gsmenu = MENU_GS_TITLE[Config::lang];
                                gsmenu += string(MENU_GS_MODE[Config::lang]) + "\t"
                                        + (Config::gs_enabled ? "On" : "Off") + "\n";
                                uint8_t ci = Config::gs_clock < 5 ? Config::gs_clock : 1;
                                string clockRow = string("Clock") + "\t"
                                               + GS_CLOCK_NAMES[ci] + " >\n";
                                if (!Config::gs_enabled) clockRow = "\x01" + clockRow;
                                gsmenu += clockRow;
                                menu_saverect = gsFirstDraw;
                                uint8_t gs_num = menuRun(gsmenu);
                                gsFirstDraw = false;
                                if (gs_num == 1) {
                                    // Mode toggle — requires reboot
                                    uint8_t prev = Config::gs_enabled;
                                    Config::gs_enabled = prev ? 0 : 1;
                                    if (Config::gs_enabled != prev) {
                                        if (Config::gs_enabled && !OSD::featureBudgetGate(Subsystems::FEAT_GENERAL_SOUND)) {
                                            // Denied / cancelled (a freeing reboot never returns).
                                            Config::gs_enabled = prev;
                                        } else if (Config::gs_enabled && Config::esxdos == 2) {
                                            if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                                Config::esxdos = 0;
                                                Config::save();
                                                esp_hard_reset();
                                            } else {
                                                Config::gs_enabled = prev;
                                            }
                                        } else if (confirmReboot(OSD_DLG_APPLYREBOOT)) {
                                            Config::save();
                                            esp_hard_reset();
                                        } else {
                                            Config::gs_enabled = prev;
                                        }
                                    }
                                    menu_curopt = 1;
                                    continue;
                                } else if (gs_num == 2 && Config::gs_enabled) {
                                    // Clock submenu — radio button list
                                    menu_level = 3;
                                    menu_curopt = ci + 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string cmenu = MENU_GS_CLOCK[Config::lang];
                                        cmenu += MENU_GS_CLOCK_SEL[Config::lang];
                                        int mpos = -1; int idx = 0;
                                        while ((mpos = cmenu.find("[ ]", mpos + 1)) != (int)string::npos) {
                                            if (idx == ci) cmenu.replace(mpos, 3, "[*]");
                                            idx++;
                                        }
                                        uint8_t prev_clock = Config::gs_clock;
                                        uint8_t opt2 = menuRun(cmenu);
                                        if (opt2) {
                                            uint8_t newclock = opt2 - 1;
                                            if (newclock != prev_clock) {
                                                // Clock only feeds pump()/step() timing constants
                                                // (no allocation) — apply live, no reboot needed.
                                                Config::gs_clock = newclock;
                                                GS::setClock();
                                                Config::save();
                                            }
                                            ci = Config::gs_clock < 5 ? Config::gs_clock : 1;
                                            menu_curopt = ci + 1;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 2;
                                            menu_level = 2;
                                            menu_saverect = false;
                                            break;
                                        }
                                    }
                                    continue;
                                } else if (gs_num == 2 && !Config::gs_enabled) {
                                    // Disabled row — ignore
                                    menu_curopt = 2;
                                    continue;
                                } else {
                                    menu_curopt = 8;
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
#endif
#endif
                        else if (options_num ==
#if !PICO_RP2040
                            (gs_avail ? 9 : 8)
#else
                            6
#endif
                        ) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string menu = MENU_I2S[Config::lang];
#if PICO_RP2040 || !defined(VGA_HDMI)
                                // HDMI audio needs RP2350 + HDMI video — hide the entry
                                {
                                    auto pos = menu.find("HDMI");
                                    if (pos != string::npos) menu.erase(pos, menu.find('\n', pos) - pos + 1);
                                }
#endif
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
                                    // (HDMI is hidden on RP2040/non-HDMI builds, so
                                    // opt2-1 doesn't always match the driver number)
                                    static const uint8_t driver_map[] = {0, 1, 2, 3,
#if !PICO_RP2040 && defined(VGA_HDMI)
                                        4,
#endif
#ifdef ZERO2
                                        5
#endif
                                    };
                                    static const uint8_t driver_map_size = sizeof(driver_map);
                                    Config::audio_driver = (opt2 <= driver_map_size) ? driver_map[opt2 - 1] : prev;
                                    if (Config::audio_driver != prev) {
#if !PICO_RP2040 && defined(VGA_HDMI)
                                        // Budget-gate when switching to HDMI audio (~8.5 KB ring/queue).
                                        // The gate reboots itself if the user frees features; otherwise
                                        // fall through to the normal confirm+reboot.
                                        if (Config::audio_driver == 4 &&
                                            !OSD::featureBudgetGate(Subsystems::FEAT_HDMI_AUDIO)) {
                                            Config::audio_driver = prev;   // declined → keep current driver
                                        } else
#endif
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
                                        (gs_avail ? 9 : 8);
#else
                                        6;
#endif
                                    menu_level = 1;
                                    break;
                                }
                            }
                        }
                        else if (options_num ==
#if !PICO_RP2040
                            (gs_avail ? 10 : 9)
#else
                            7
#endif
                        ) { // Volume Boost
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string boost_menu = MENU_AUDIO_BOOST[Config::lang];
                                uint8_t cur = Config::audio_boost;
                                int cur_idx = 0;
                                for (int i = 0; i < (int)(sizeof(AUDIO_BOOST_VALS)); i++)
                                    if (AUDIO_BOOST_VALS[i] == cur) { cur_idx = i; break; }
                                static const char boost_marks[] = "ABCDEFG";
                                for (int i = 0; i < (int)(sizeof(AUDIO_BOOST_VALS)); i++) {
                                    char mark[3] = { '[', boost_marks[i], '\0' };
                                    auto pos = boost_menu.find(mark, 0);
                                    if (pos != string::npos)
                                        boost_menu.replace(pos, 2, cur_idx == i ? "[*" : "[ ");
                                }
                                uint8_t opt2 = menuRun(boost_menu);
                                if (opt2 >= 1 && opt2 <= (uint8_t)(sizeof(AUDIO_BOOST_VALS))) {
                                    Config::audio_boost = AUDIO_BOOST_VALS[opt2 - 1];
                                    Config::save();
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = options_num;
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
                                uint8_t &curVideoMode = 
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
                                SELECT_VGA ? Config::vga_video_mode :
#endif
                                 Config::hdmi_video_mode;
#else
                                uint8_t dummy_vm = 0;
                                uint8_t &curVideoMode = dummy_vm;
#endif
                                // cur_sel maps directly to VM_* enum: 0=640x480@60, 1=640x480@50, 2=720x480@60, 3=720x576@50
                                uint8_t cur_sel = curVideoMode;
                                opt_menu.replace(opt_menu.find("[6",0),2, cur_sel == 0 ? "[*" : "[ ");
                                opt_menu.replace(opt_menu.find("[5",0),2, cur_sel == 1 ? "[*" : "[ ");
                            #if !PICO_RP2040
                                opt_menu.replace(opt_menu.find("[H",0),2, cur_sel == 2 ? "[*" : "[ ");
                                opt_menu.replace(opt_menu.find("[F",0),2, cur_sel == 3 ? "[*" : "[ ");
                            #endif
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    uint8_t new_vm = opt2 - 1; // opt2 is 1-based, VM_* is 0-based
                                    if (new_vm != curVideoMode) {
                                        // Save old vm as pending for post-reboot rollback confirmation,
                                        // then commit new vm and hard reset — runtime FB resize is
                                        // unreliable due to heap fragmentation.
                                        Config::savePendingVideoMode();
                                        curVideoMode = new_vm;
                                        Config::save();
                                        OSD::esp_hard_reset();
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
                                opt_menu += MENU_SCANLINES_SEL[Config::lang];
                                uint8_t prev_opt = Config::scanlines; // 0=Off, 1..4=level
                                // Mark the active level: rows are [0]..[4], matching the value
                                for (uint8_t lv = 0; lv <= 4; ++lv) {
                                    char tag[3] = { '[', (char)('0' + lv), 0 };
                                    size_t at = opt_menu.find(tag, 0);
                                    if (at != string::npos)
                                        opt_menu.replace(at, 2, (lv == prev_opt) ? "[*" : "[ ");
                                }
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    // Rows 1..5 map to levels 0..4
                                    Config::scanlines = opt2 - 1;

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
                            menu_curopt = Config::gigascreen_onoff + 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = MENU_GIGASCREEN[Config::lang];
                                opt_menu += MENU_GIGASCREEN_SEL[Config::lang];
                                uint8_t prev_onoff = Config::gigascreen_onoff;
                                int mpos = -1;
                                int idx = 0;
                                while ((mpos = opt_menu.find("[ ]", mpos + 1)) != (int)string::npos) {
                                    if (idx == prev_onoff)
                                        opt_menu.replace(mpos, 3, "[*]");
                                    idx++;
                                }
                                uint8_t opt2 = menuRun(opt_menu);
                                if (opt2) {
                                    Config::gigascreen_onoff = opt2 - 1; // 0=Off, 1=On, 2=Auto
                                    if (Config::gigascreen_onoff != prev_onoff) {
                                        bool want_on = (Config::gigascreen_onoff != 0);
                                        // Profi is incompatible with Gigascreen (its renderer
                                        // geometry never touches the prev-FB coherently) —
                                        // arch switches INTO Profi already force it off
                                        // (disableGigascreenForProfi), but enabling it FROM
                                        // this menu while Profi is running crashed with a
                                        // SIGBUS storm in the render path (hw, PICO_DV).
                                        if (want_on && Z80Ops::isProfi) {
                                            OSD::osdCenteredMsg("Gigascreen: not available on Profi", LEVEL_WARN, 1500);
                                            Config::gigascreen_onoff = prev_onoff;
                                            menu_curopt = prev_onoff + 1;
                                            menu_saverect = false;
                                            continue;
                                        }
                                        if (want_on && !OSD::featureBudgetGate(Subsystems::FEAT_GIGASCREEN)) {
                                            // Denied / cancelled (a freeing reboot never returns here).
                                            Config::gigascreen_onoff = prev_onoff;
                                            menu_curopt = prev_onoff + 1;
                                            menu_saverect = false;
                                            continue;
                                        }
                                        if (want_on) {
                                            initGigascreenBlendLUT();
                                            Config::gigascreen_enabled = true;
                                            GsSubsys::request(true);
                                            // Allocate immediately so InitPrevBuffer can seed it.
                                            GsSubsys::apply();
                                            if (!VIDEO::vga.prevFrameBuffer) {
                                                // OOM — fall back to Off.
                                                Config::gigascreen_enabled = false;
                                                Config::gigascreen_onoff = 0;
                                                VIDEO::gigascreen_enabled = false;
                                                VIDEO::gigascreen_auto_countdown = 0;
                                            } else {
                                                VIDEO::InitPrevBuffer();
                                                VIDEO::gigascreen_enabled = (Config::gigascreen_onoff == 1);
                                                VIDEO::gigascreen_auto_countdown = 0;
                                            }
                                        } else {
                                            // Off — release the 52 KB prev framebuffer.
                                            Config::gigascreen_enabled = false;
                                            VIDEO::gigascreen_enabled = false;
                                            VIDEO::gigascreen_auto_countdown = 0;
                                            GsSubsys::request(false);
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
                                    bool want_ula = (opt2 == 1);
                                    // Budget-gate for uniformity (ULA+ costs 0 SRAM — table in
                                    // flash — so this always allows; keeps the toggle shape identical).
                                    if (want_ula && !prev_ula &&
                                        !OSD::featureBudgetGate(Subsystems::FEAT_ULAPLUS)) {
                                        // declined → leave off
                                    } else {
                                        Config::ulaplus = want_ula;
                                        if (Config::ulaplus != prev_ula) {
                                            if (!Config::ulaplus && VIDEO::ulaplus_enabled) {
                                                VIDEO::ulaPlusDisable();
                                            }
                                            Config::save();
                                        }
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
                                    bool want_tmx = (opt2 == 1);
                                    // Budget-gate for uniformity (Timex costs 0 SRAM → always allows).
                                    if (want_tmx && !prev &&
                                        !OSD::featureBudgetGate(Subsystems::FEAT_TIMEX)) {
                                        // declined → leave off
                                    } else {
                                        if (want_tmx)
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
#if !PICO_RP2040
                                        // Budget-gate when turning DMA on from off (~7 KB attr shadow).
                                        if (prev == 0 && Config::dma_mode != 0 &&
                                            !OSD::featureBudgetGate(Subsystems::FEAT_DMA)) {
                                            Config::dma_mode = prev;   // declined to free → stay off
                                        } else {
                                            // Apply now: emulation is paused in the OSD, so the attr
                                            // shadow alloc/free is consistent for the renderer.
                                            DmaSubsys::request(Config::dma_mode != 0);
                                            DmaSubsys::apply();
                                            Config::save();
                                        }
#else
                                        Config::save();
#endif
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
                        // 16col (Pentagon)
                        else if (options_num == 12) {
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string mc_menu = MENU_16COL[Config::lang];
                                mc_menu += MENU_YESNO[Config::lang];
                                bool prev = Config::mode16col_onoff;
                                if (prev) {
                                    mc_menu.replace(mc_menu.find("[Y",0),2,"[*");
                                    mc_menu.replace(mc_menu.find("[N",0),2,"[ ");
                                } else {
                                    mc_menu.replace(mc_menu.find("[Y",0),2,"[ ");
                                    mc_menu.replace(mc_menu.find("[N",0),2,"[*");
                                }
                                uint8_t opt2 = menuRun(mc_menu);
                                if (opt2) {
                                    bool want = (opt2 == 1);
                                    if (want && !(Z80Ops::isPentagon || Z80Ops::isProfi)) {
                                        OSD::osdCenteredMsg(OSD_16COL_NEEDS_PENTAGON[Config::lang], LEVEL_WARN, 1500);
                                    } else if (want && !prev &&
                                               !OSD::featureBudgetGate(Subsystems::FEAT_16COL)) {
                                        // declined to free → leave 16col off (~0.5 KB LUT)
                                    } else {
                                        Config::mode16col_onoff = want;
                                        if (Config::mode16col_onoff) {
                                            // Build the decode LUT now so the rasterizer is ready
                                            // even before the next machine reset.
                                            VIDEO::ensure16colLut();
                                        } else if (VIDEO::mode16col_enabled) {
                                            // Disabling globally also drops the runtime latch.
                                            VIDEO::mode16col_enabled = false;
                                        }
                                        if (!Config::mode16col_onoff) VIDEO::free16colLut();
                                        if (Config::mode16col_onoff != prev) Config::save();
                                    }
                                    menu_curopt = opt2;
                                    menu_saverect = false;
                                } else {
                                    menu_curopt = 12;
                                    menu_level = 1;
                                    break;
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
                bool has_psram = butter_psram_size() || psram_size() > 0;
                bool ext_ram = has_psram || FileUtils::fsMount;
                // Profi needs PSRAM (DS80 hires framebuffer in PSRAM); without it
                // the emulation is pointless, so hide the entry on SD-only boards.
                // It also needs RP2350 (RP2040 has no DS80 hires support), so it is
                // hidden on RP2040 regardless of PSRAM.  The DS80 hires mode lives
                // entirely in the VGA/HDMI drivers, so it is also hidden on non-VGA/HDMI
                // builds (TFT/SOFTTV/TV).
                // Stripping "Profi" keeps indices of the items before it stable;
                // any trailing item (ALF) shifts up by one and is keyed off the
                // computed last index below.  show_profi gates both the menu entry
                // and the arch_num==8 branch so the indices stay consistent.
#if PICO_RP2040 || !defined(VGA_HDMI)
                // RP2040 has no DS80 hires support; non-VGA/HDMI builds (TFT/SOFTTV/TV)
                // do not link the DS80 packed-pair display mode (the set_profi_ds80_mode
                // entry points are no-op stubs there — see Video.cpp), so Profi would
                // run without its defining hires mode. Hide it on those builds.
                const bool show_profi = false;
#else
                const bool show_profi = has_psram;
#endif
                string arch_menu = ext_ram ? MENU_ARCH[Config::lang] : MENU_ARCH_NO_SD[Config::lang];
                if (ext_ram && !show_profi) {
                    const string profi_line = "Profi\t>\n";
                    size_t p = arch_menu.find(profi_line);
                    if (p != string::npos) arch_menu.erase(p, profi_line.size());
                }
                while (1) {
                    menu_level = 1;
                    uint8_t arch_num = menuRun(arch_menu.c_str());
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
                                                    // BYTE and BYTE-compat are both overlays over Sinclair 48K.
                                                    MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
                                                    MemESP::registerOverlay(gb_rom_0_sinclair_48k,
                                                        Config::byte_cobmect_mode ? gb_overlay_48k_byte_sovmest : gb_overlay_48k_byte);
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
                            // Murmuzavr extends Pentagon paging (#AFF7 planes over #7FFD),
                            // and ESPectrum::setup drops the count at boot on anything else
                            // — say so here instead of letting the pick vanish silently.
                            if (!(Config::arch == "Pentagon" || Config::arch == "P512" ||
                                  Config::arch == "P1024")) {
                                osdCenteredMsg(Config::lang
                                    ? " Murmuzavr: solo Pentagon 128/512/1024 "
                                    : " Murmuzavr needs Pentagon 128/512/1024 ", LEVEL_WARN, 3000);
                                menu_curopt = 7;
                                menu_level = 2;
                                break;
                            }
                            menu_level = 2;
                            menu_curopt = 1;
                            menu_saverect = true;
                            while (1) {
                                string opt_menu = (FileUtils::fsMount ? MENU_MURMUZAVR : MENU_MURMUZAVR_NONE)[Config::lang];
                                // The PICK, not the live count: a machine that boots
                                // clamped (Profi, see ESPectrum::setup) would otherwise
                                // show "None" and re-select the same size forever.
                                uint32_t new_opt = Config::mem_pg_cnt, prev_opt = Config::mem_pg_cnt;
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
                                            // The pick is what gets persisted; the live
                                            // count is re-derived from it in setup()
                                            // after the reboot below.
                                            Config::mem_pg_cnt = (uint16_t)new_opt;
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
                        } else if (show_profi && arch_num == 8) { // Profi (needs PSRAM+RP2350 for DS80)
                            menu_curopt = 1;
                            menu_saverect = true;
                            opt2 = 0;
                            while (1) {
                                // menu_level set INSIDE the loop (canonical pattern) so the
                                // outer Profi submenu level stays stable across iterations,
                                // even after an inner submenu (level 3) returns.
                                menu_level = 2;
                                // Profi submenu: ROM selection + XT keyboard + OSD palette
                                string profi_sub =
                                    string(Config::lang ? "Profi\n" : "Profi\n") +
                                    "1024K (Original)\n" +
                                    "1024K (Karabas)\n" +
                                    "1024K (Karabas+PQDOS)\n" +
                                    "1024K (Karabas+FlashTool)\n" +
                                    "1024K (Karabas+FDImage)\n" +
                                    string("XT keyboard [") +
                                    (Config::profi_ext_keys ? "ON" : "OFF") + "]\n" +
                                    string("OSD palette [") +
                                    (Config::profi_ds80_std_palette_osd ? "STD" : "DS80") + "]\n";
                                uint8_t opt_p = menuRun(profi_sub);
                                if (opt_p >= 1 && opt_p <= 5) {
                                    // ROM selected — mirrors the real Karabas-Pro ROMSET
                                    // slots: 1=stock "Original", 2="Karabas" (ROMain,
                                    // ROMSET 0), 3=PQDOS BIOS (ROMSET 1), 4=Flash Tool
                                    // (ROMSET 2), 5=FDImage (ROMSET 3)
                                    static const char* profi_romsets[5] = {
                                        "Profi", "ProfiKarabas", "ProfiPQ",
                                        "ProfiKarabasFT", "ProfiKarabasFDI" };
                                    arch = "Profi";
                                    romset = profi_romsets[opt_p - 1];
                                    opt2 = 1; // signal machine switch
                                    menu_curopt = 1;
                                    menu_saverect = false;
                                    break;
                                } else if (opt_p == 6) {
                                    // XT keyboard toggle (Yes/No submenu) — level 3
                                    // Remind the user it can also be toggled live via the hotkey.
                                    osdCenteredMsg(Config::lang
                                        ? " Atajo: Alt+~ alterna teclado XT "
                                        : " Hotkey Alt+~ toggles XT keyboard ", LEVEL_WARN, 3000);
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string ext_menu = string(Config::lang ? "Teclado XT\n" : "XT keyboard\n");
                                        ext_menu += MENU_YESNO[Config::lang];
                                        bool prev_ext = Config::profi_ext_keys;
                                        if (prev_ext) {
                                            ext_menu.replace(ext_menu.find("[Y",0), 2, "[*");
                                            ext_menu.replace(ext_menu.find("[N",0), 2, "[ ");
                                        } else {
                                            ext_menu.replace(ext_menu.find("[Y",0), 2, "[ ");
                                            ext_menu.replace(ext_menu.find("[N",0), 2, "[*");
                                        }
                                        uint8_t opt3 = menuRun(ext_menu);
                                        if (opt3) {
                                            Config::profi_ext_keys = (opt3 == 1);
                                            if (Config::profi_ext_keys != prev_ext) Config::save();
                                            menu_curopt = opt3;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 6;
                                            menu_level = 2;
                                            break; // back to Profi submenu
                                        }
                                    }
                                } else if (opt_p == 7) {
                                    // OSD palette toggle (STD / DS80) submenu — level 3
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        // Two explicit options: STD (standard ZX palette) / DS80
                                        // (live Profi palette).  Active one marked with [*].
                                        string osd_pal_menu = string(Config::lang ? "Paleta OSD\n" : "OSD palette\n");
                                        bool prev_pal = Config::profi_ds80_std_palette_osd;
                                        osd_pal_menu += string("STD\t[")  + (prev_pal  ? "*" : " ") + "]\n";
                                        osd_pal_menu += string("DS80\t[") + (!prev_pal ? "*" : " ") + "]\n";
                                        uint8_t opt3 = menuRun(osd_pal_menu);
                                        if (opt3) {
                                            Config::profi_ds80_std_palette_osd = (opt3 == 1); // 1=STD, 2=DS80
                                            if (Config::profi_ds80_std_palette_osd != prev_pal) {
                                                Config::save();
                                                // Re-apply immediately — OSD blocks the main loop
                                                // so EndFrame won't fire to do it for us.
                                                // applyProfiOSDPalette() reads the new flag and
                                                // picks STD (conv_color swap) or DS80 (Graphics remap).
#if !PICO_RP2040
                                                if (profi_ds80_active) {
                                                    VIDEO::restoreProfiLivePalette(); // clean slate
                                                    VIDEO::profi_ds80_osd_active = true;
                                                    VIDEO::applyProfiOSDPalette();
                                                }
#endif
                                            }
                                            menu_curopt = opt3;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 7;
                                            menu_level = 2;
                                            break; // back to Profi submenu
                                        }
                                    }
                                } else {
                                    // ESC — exit Profi submenu
                                    opt2 = 0;
                                    menu_curopt = 1;
                                    menu_level = 2;
                                    break;
                                }
                            }
                        }
#if !PICO_RP2040
                        else if (arch_num == 9 || (ext_ram && !show_profi && arch_num == 8) || !ext_ram) { // ALF TV GAME (shifts to 8 when Profi hidden)
                            arch = "ALF";
                            romset = "ALF1";
                            menu_curopt = opt2;
                            menu_saverect = false;
                            Config::romSet = romset;
                            // (ALF carts stream from SD now, not the GM.DLS flash region, so
                            // GM.DLS may stay enabled when entering ALF — no disable needed.)
                            click();
                            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
#if !PICO_RP2040
                            // Leaving Profi's forced-SRAM layout (no butter PSRAM) MUST reboot:
                            // ESPectrum::reset() does NOT free the ~96 KB Profi pages + DS80
                            // framebuffer (only setup() re-lays out memory), so a soft reset
                            // here leaves them allocated and ALF OOMs (SPI-PSRAM thrash → panic).
                            // Persist arch=ALF so the reboot lands on ALF, not the old Profi arch.
                            if (butter_psram_size() == 0 &&
                                MemESP::ram[56].memType() == mem_type_t::POINTER) {
                                Config::arch = "ALF";
                                if (Config::pref_arch == "Profi") Config::pref_arch = "Last";
                                Config::save();
                                OSD::esp_hard_reset();   // never returns; setup() re-lays out for ALF
                                return;
                            }
#endif
                            Config::save();
                            Config::requestMachine(arch, romset);
                            ESPectrum::reset();
                            return;
                        }
#endif

                        if (opt2) {
#if !PICO_RP2040
                            // Leaving ALF for another machine. loadAlfCart() pinned
                            // pref_arch="ALF" so the cart machine survives the flash-reboot; that
                            // pin also makes setup() force ALF back at every boot and blocks the
                            // menu from committing a new Config::arch (it only does so when
                            // pref_arch=="Last"). Detect the switch-away here, BEFORE the
                            // "did anything change?" gate below — because ALF entered via this
                            // menu (the early branch) runs requestMachine() without writing
                            // Config::arch, so arch==Config::arch can look unchanged and the gate
                            // would skip the un-pin + save, leaving pref_arch="ALF" → F12 = ALF.
                            bool leavingAlf = (arch != "ALF" &&
                                (Config::pref_arch == "ALF" || Config::alfCartBanks > 0));
#else
                            bool leavingAlf = false;
#endif
                            if (arch != Config::arch || romset != Config::romSet || leavingAlf) {
#if !PICO_RP2040
                                if (leavingAlf) {
                                    if (Config::pref_arch == "ALF") Config::pref_arch = "Last";
                                    Config::alfCartBanks = 0;   // unmount cart (empty drive)
                                    Config::alfCartPath  = "";
                                    AlfCart::unmount();         // close the SD cart file
                                }
                                // Entering Profi needs ~96 KB SRAM on butter-less boards.
                                // Gate it: the popup frees room (Gigascreen/ZiFi/DivMMC
                                // auto-handled by the code below; offers GS) or refuses. A freeing
                                // reboot never returns; denied/cancelled stays on current arch.
                                if (arch == "Profi" && Config::arch != "Profi" &&
                                    !OSD::featureBudgetGate(Subsystems::FEAT_PROFI)) {
                                    menu_curopt = arch_num;
                                    menu_saverect = false;
                                    continue;
                                }
#endif
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
                                    } else if (arch == "Profi") {
                                        if (Config::pref_romSetProfi == "Last") {
                                            Config::romSet = romset;
                                            Config::romSetProfi = romset;
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
                                // The per-arch romset slot above is only written when the
                                // romset itself changed. Switching arch while the same romset
                                // stays active (e.g. P512+Gluk → P1024 — Config::romSet is
                                // already "128Kpg") leaves the destination arch's slot at its
                                // default, so a cold boot reloads the wrong ROM: Gluk works
                                // while running (romSet carries over across the switch) but
                                // reverts to the 128 menu after a restart. Sync the active
                                // arch's slot to the running romSet so it survives reboot
                                // (mirrors the pref=="Last" gating of the romset block above;
                                // when a pref is pinned, cold boot loads from that pref instead).
                                if (Config::arch == "Pentagon" && Config::pref_romSetPent == "Last") Config::romSetPent = Config::romSet;
                                else if (Config::arch == "P512" && Config::pref_romSetP512 == "Last") Config::romSetP512 = Config::romSet;
                                else if (Config::arch == "P1024" && Config::pref_romSetP1M == "Last") Config::romSetP1M = Config::romSet;
                                else if (Config::arch == "128K" && Config::pref_romSet_128 == "Last") Config::romSet128 = Config::romSet;
                                else if (Config::arch == "48K" && Config::pref_romSet_48 == "Last") Config::romSet48 = Config::romSet;
                                else if (Config::arch == "Profi" && Config::pref_romSetProfi == "Last") Config::romSetProfi = Config::romSet;
                                // Mutual exclusivity
#if !PICO_RP2040
                                bool isByte = (romset == "48Kby" || romset == "128Kby");
                                if (Config::mb02 && (arch == "Pentagon" || arch == "P512" || arch == "P1024" ||
                                    arch == "Profi" || isByte)) {
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
                                // TR-DOS is mandatory on Pentagon / Profi
                                if ((arch == "Pentagon" || arch == "P512" || arch == "P1024" || arch == "Profi") && !Config::betadisk) {
                                    Config::betadisk = true;
                                    OSD::osdCenteredMsg("Betadisk enabled", LEVEL_WARN, 1500);
                                }
                                // Byte 48K has no Beta Disk interface — force it off on entry.
                                if (romset == "48Kby" && Config::betadisk) {
                                    Config::betadisk = false;
                                    if (ESPectrum::trdos) {
                                        ESPectrum::trdos = false;
                                        MemESP::recoverPage0();
                                    }
                                    OSD::osdCenteredMsg("Betadisk disabled", LEVEL_WARN, 1500);
                                }
#if !PICO_RP2040
                                // Switching into Profi: Gigascreen is incompatible —
                                // turn it off and free its 52 KB prev-FB before saving
                                // so the Off-state persists across reboots. Config::arch
                                // is already committed above (pref_arch=="Last" path).
                                if (Config::arch == "Profi" && Config::gigascreen_enabled) {
                                    VIDEO::disableGigascreenForProfi();
                                    OSD::osdCenteredMsg("Gigascreen disabled", LEVEL_WARN, 1500);
                                }
                                // Switching into Profi: turn the ZiFi NIC off and free its
                                // ~12 KB of heap rings — Profi forces ~80 KB of SRAM pages
                                // and OOMs at VIDEO::Init otherwise. Persist Off so the boot
                                // into Profi has the room (boot also skips ZiFi on Profi).
                                if (Config::arch == "Profi" && Config::zifi_enabled) {
                                    Config::zifi_enabled = 0;
                                    ZiFi::enabled = 0;
                                    if (!ZiFiAT::connected) {
#if ZIFI_NET_CLIENT
                                        ZiFiSock::end();
#endif
                                        ZiFi::deinit();
                                    }
                                    OSD::osdCenteredMsg("ZiFi NIC disabled", LEVEL_WARN, 1500);
                                }
                                // Switching into Profi: turn DivMMC off and free its
                                // sector/IDE buffers — same mutual exclusion as MB-02+
                                // (Profi forces ~80 KB of SRAM pages and OOMs otherwise).
                                if (Config::arch == "Profi" && Config::esxdos) {
                                    Config::esxdos = 0;
                                    DivMMC::init();   // teardown path frees buffers
                                    OSD::osdCenteredMsg("DivMMC disabled", LEVEL_WARN, 1500);
                                }
                                // NOTE: GM.DLS MIDI is NOT auto-disabled on Profi entry anymore.
                                // On tight butter-less boards the featureBudgetGate popup offers
                                // MIDI as a manual free candidate instead (user decides).
                                // Karabas-Pro romsets (everything but the stock "Profi"
                                // Original) boot from SD through the real board's
                                // Z-Controller (ROMain "Loading boot from SD" = FATALL
                                // via ZC) — auto-enable it on entry so SD boot works
                                // out of the box. esxDOS/MB-02+ are already forced off
                                // above; skipped silently if the budget gate declines.
                                if (arch == "Profi" && romset != "Profi" &&
                                    !Config::zcontroller && FileUtils::fsMount &&
                                    OSD::featureBudgetGate(Subsystems::FEAT_ZCONTROLLER)) {
                                    Config::zcontroller = true;
                                    DivMMC::zc_init();
                                    OSD::osdCenteredMsg("Z-Controller enabled", LEVEL_INFO, 1500);
                                }
#endif
                                Config::save();
#if !PICO_RP2040
                                // Profi on SPI-PSRAM boards (no butter PSRAM) needs its
                                // hires colour/working pages 56/58/61/60/40 backed by
                                // SRAM (see assign_ram()) — ~80 KB total.  That backing
                                // is decided once in setup() from the boot arch, and
                                // ESPectrum::reset() does NOT re-assign or free it.  So
                                // ANY switch that crosses the Profi boundary must reboot
                                // so setup() re-lays out memory:
                                //  - entering Profi: 56/58/61/60/40 still in SPI PSRAM
                                //    (direct()==null) → DS80 colour memory invalid →
                                //    screen never switches.
                                //  - leaving Profi: the 80 KB of forced-SRAM page buffers
                                //    would stay allocated and unused on the new machine.
                                // ram[56].memType()==POINTER iff the Profi forced-SRAM
                                // layout is currently in effect, so reboot whenever that
                                // disagrees with the desired arch.  Config is already
                                // saved above, so setup() re-backs pages for the new arch.
                                bool profiSramLayout =
                                    (MemESP::ram[56].memType() == mem_type_t::POINTER);
                                if (butter_psram_size() == 0
                                    && (arch == "Profi") != profiSramLayout) {
                                    OSD::esp_hard_reset();
                                    return;
                                }
#endif
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
                        // Declined: redraw same submenu in place, same item focused
                        menu_curopt = opt2;
                        menu_saverect = false;
                    } else if (mos && opt2 == 4) {
                        if (confirmReboot(OSD_DLG_REBOOT)) {
                            f_unlink(MOS_FILE);
                            esp_hard_reset();
                        }
                        menu_curopt = opt2;
                        menu_saverect = false;
                    } else if ((mos && opt2 == 5) || (!mos && opt2 == 4)) {
                        // True factory reset: wipe storage.nvs AND skip the
                        // user's saved default.nvs on the next load() (see
                        // SKIP_DEFAULT_FLAG in Config::load()).
                        if (confirmReboot(OSD_DLG_LOADDEFAULTS)) {
                            FIL* flag = fopen2(SKIP_DEFAULT_FLAG, FA_WRITE | FA_CREATE_ALWAYS);
                            if (flag) fclose2(flag);
                            f_unlink(STORAGE_NVS);
                            esp_hard_reset();
                        }
                        menu_curopt = opt2;
                        menu_saverect = false;
                    } else if ((mos && opt2 == 6) || (!mos && opt2 == 5)) {
                        // Save as Default: snapshot current live config as the
                        // fallback used whenever no storage.nvs exists yet for
                        // this board (fresh firmware version, or "My Default"
                        // reset below). Per-board on purpose — never crosses
                        // to a different board (video wiring / RAM budget).
                        if (confirmReboot(OSD_DLG_SAVEDEFAULT)) {
                            Config::save(DEFAULT_NVS);
                            osdCenteredMsg(MSG_DEFAULT_SAVED[Config::lang], LEVEL_INFO, 500);
                        }
                        // Redraw same submenu in place either way, same item focused
                        menu_curopt = opt2;
                        menu_saverect = false;
                    } else if ((mos && opt2 == 7) || (!mos && opt2 == 6)) {
                        // My Default: wipe storage.nvs only — default.nvs is
                        // left intact, so the next load() falls back to it.
                        if (confirmReboot(OSD_DLG_LOADMYDEFAULT)) {
                            f_unlink(STORAGE_NVS);
                            esp_hard_reset();
                        }
                        menu_curopt = opt2;
                        menu_saverect = false;
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
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            // menu_level set INSIDE the loop (canonical pattern): inner
                            // ROM-pref submenus run at level 3 and restore 2 on Esc, but
                            // setting it here keeps the outer level stable across iterations.
                            menu_level = 2;
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
                            string other_menu = MENU_OTHER[Config::lang];
#if !PICO_RP2040
                            other_menu += MENU_OTHER_RTC[Config::lang]; // RP2350-only RTC toggle
#endif
                            uint8_t options_num = menuRun(other_menu);
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
                                else if (options_num == 8) {
                                    // LED indicators ON/OFF + Legend
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string opt_menu = MENU_LEDINDICATORS[Config::lang];
                                        opt_menu += MENU_YESNO[Config::lang];
                                        opt_menu += Config::lang ? "Leyenda\t>\n" : "Legend\t>\n";
                                        bool prev_opt = Config::ledIndicators;
                                        if (prev_opt) {
                                            opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                            opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                        } else {
                                            opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                            opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(opt_menu);
                                        if (opt2 == 1 || opt2 == 2) {
                                            Config::ledIndicators = (opt2 == 1);
                                            if (Config::ledIndicators != prev_opt) {
                                                if (!Config::ledIndicators) LED::clear();
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else if (opt2 == 3) {
                                            // showLedLegend() does its own SaveRect save/restore,
                                            // so the LED menu background is already intact on return.
                                            // menu_saverect MUST be false here: a true would re-save
                                            // and recalculate y (shifts the menu) and imbalance the
                                            // SaveRect stack → SIGBUS.
                                            showLedLegend();
                                            menu_curopt = 3;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 8;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 9) {
                                    // SD card LED (onboard GPIO 25) ON/OFF
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string opt_menu = MENU_SDLEDBLINK[Config::lang];
                                        opt_menu += MENU_YESNO[Config::lang];
                                        bool prev_opt = Config::sdLedBlink;
                                        if (prev_opt) {
                                            opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                            opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                        } else {
                                            opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                            opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(opt_menu);
                                        if (opt2 == 1 || opt2 == 2) {
                                            Config::sdLedBlink = (opt2 == 1);
                                            if (Config::sdLedBlink != prev_opt) {
                                                sdcard_set_led_blink(Config::sdLedBlink);
                                                Config::save();
                                            }
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 9;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
#if !PICO_RP2040
                                else if (options_num == 10) {
                                    // RTC + NVRAM (Pentagon/Profi Mr Gluk MC146818) ON/OFF
                                    menu_level = 3;
                                    menu_curopt = 1;
                                    menu_saverect = true;
                                    while (1) {
                                        string opt_menu = MENU_RTC[Config::lang];
                                        opt_menu += MENU_YESNO[Config::lang];
                                        bool prev_opt = Config::rtc_enabled;
                                        if (prev_opt) {
                                            opt_menu.replace(opt_menu.find("[Y",0),2,"[*");
                                            opt_menu.replace(opt_menu.find("[N",0),2,"[ ");
                                        } else {
                                            opt_menu.replace(opt_menu.find("[Y",0),2,"[ ");
                                            opt_menu.replace(opt_menu.find("[N",0),2,"[*");
                                        }
                                        uint8_t opt2 = menuRun(opt_menu);
                                        if (opt2 == 1 || opt2 == 2) {
                                            Config::rtc_enabled = (opt2 == 1);
                                            if (Config::rtc_enabled != prev_opt) Config::save();
                                            menu_curopt = opt2;
                                            menu_saverect = false;
                                        } else {
                                            menu_curopt = 10;
                                            menu_level = 2;
                                            break;
                                        }
                                    }
                                }
#endif
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
                                }
                                else {
                                    string mFile = fileDialog(FileUtils::ROM_Path, MENU_ROM_TITLE[Config::lang], DISK_ROMFILE, 26, 15);
                                    if (mFile != "") {
                                        mFile.erase(0, 1);
                                        string fname = FileUtils::ROM_Path + mFile;
                                        // ALF cartridges (and ROMs) ship zipped (zxbyte.org) — extract
                                        // the .rom/.bin inside before flashing.
                                        if (FileUtils::getLCaseExt(fname) == "zip") {
                                            string zf = ZipExtract::extract(fname, DISK_ROMFILE);
                                            if (zf.empty()) { OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN); fname.clear(); }
                                            else if (zf != "\x1b") fname = zf;
                                            else fname.clear();
                                        }
                                        if (!fname.empty()) {
                                            bool res = updateROM(fname, opt2 - 1);
                                            if (res) {
                                                return;
                                            }
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
                    } else if (opt2 == 7) {
                        // Debug Log toggle
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            string dl_menu = MENU_DEBUG_LOG[Config::lang];
                            dl_menu += MENU_YESNO[Config::lang];
                            bool prev_dl = Debug::log_enabled;
                            if (prev_dl) {
                                dl_menu.replace(dl_menu.find("[Y",0),2,"[*");
                                dl_menu.replace(dl_menu.find("[N",0),2,"[ ");
                            } else {
                                dl_menu.replace(dl_menu.find("[Y",0),2,"[ ");
                                dl_menu.replace(dl_menu.find("[N",0),2,"[*");
                            }
                            uint8_t opt3 = menuRun(dl_menu);
                            if (opt3) {
                                Debug::log_enabled = (opt3 == 1);
                                if (Debug::log_enabled != prev_dl) {
                                    Config::save();
                                }
                                menu_curopt = opt3;
                                menu_saverect = false;
                            } else {
                                menu_curopt = 7;
                                menu_level = 1;
                                break;
                            }
                        }
                    } else {
                        menu_curopt = 8;
                        break;
                    }
                }
            }
#if !PICO_RP2040
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
                        // Memory Info
                        OSD::MemoryInfo();
                        menu_curopt = 3;
                        menu_saverect = false;
                    }
                    else if (hw_opt == 4) {
                        // Emulator Info
                        OSD::EmulatorInfo();
                        menu_curopt = 4;
                        menu_saverect = false;
                    }
                    else if (hw_opt == 5) {
                        // HID devices
                        OSD::HIDDevices();
                        menu_curopt = 5;
                        menu_saverect = false;
                    }
                    else if (hw_opt == 7) {
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
                                menu_curopt = 6;
                                menu_level = 1;
                                break;
                            }
                        }
                    }
                    else if (hw_opt == 6) {
                        // Speed Test
                        OSD::SpeedTest();
                        menu_curopt = 6;
                        menu_saverect = false;
                    }
                    else {
                        menu_curopt = 9;
                        break;
                    }
                }
            }
#endif
#if !PICO_RP2040
            else if (opt == 10) { // Network
                menu_saverect = true;
                menu_curopt = 1;
                // Live WiFi status (getStatus is a blocking AT round-trip) — cached,
                // refreshed on entry and after connect/disconnect/NIC toggle.
                bool   st_conn = false;
                string st_ssid, st_ip;
                auto refreshNetStatus = [&]() {
                    st_ssid.clear(); st_ip.clear();
                    // Query when WiFi is in use — WiFi enabled, or already connected (e.g.
                    // the boot SNTP auto-connect still settling). Keyed on WiFi, not the
                    // NIC: the NIC is a layer on top and never the reason networking is up.
                    // Avoids powering up the ESP just by opening the menu when WiFi is off.
                    st_conn = (Config::wifi_enabled || ZiFiAT::connected)
                              && ZiFiAT::getStatus(st_ssid, st_ip);
                };
                // After a transport/baud change re-established the UART link, make sure WiFi
                // is associated again. Normally the ESP keeps its association across a host
                // UART reconfigure and getStatus() above confirms it — but if the change
                // dropped the link, re-associate from the saved credentials. We're in menu
                // context with the Z80 paused, so a blocking connect is safe.
                auto reconnectWifiIfNeeded = [&]() {
                    if (Config::wifi_enabled && !ZiFiAT::connected && !Config::wifi_ssid.empty()) {
                        ZiFiAT::connect(Config::wifi_ssid, Config::wifi_pass);
                        refreshNetStatus();
                    }
                };

                // ── ESP-01S hardware/config pickers (level 3, under the ESP01 submenu) ──
                // ESP-01 transport picker: Off / USB (CH340) / per-board GPIO pairs.
                // Merges the old GPIO picker with the USB-CDC choice. The USB row only
                // exists in host-stack builds (#ifdef KBDUSB) — without it the host
                // never enumerates a dongle, so USB transport can't work. usbRow shifts
                // the dense GPIO-pair indices (zifiPair()/zifiPairCount() are already a
                // contiguous, package-filtered view) by one when the USB row is present.
                auto pickTransport = [&]() {
                    menu_level = 3; menu_saverect = true;
                    int usbRow = 0;
#if defined(KBDUSB)
                    usbRow = 1;
#endif
                    string m = string(MENU_ZIFI_TRANSPORT_TITLE[Config::lang]) + "\n";
                    m += "Off\n";
                    if (usbRow) m += string(MENU_ZIFI_USB_LABEL[Config::lang]) + "\n";
                    int nopt = BoardPins::zifiPairCount();
                    for (int i = 0; i < nopt; i++) {
                        const BoardPins::UartPair* p = BoardPins::zifiPair(i);
                        char b[48];
                        snprintf(b, sizeof(b), "GPIO %u/%u%s%s\n", p->tx, p->rx, p->note[0] ? "  " : "", p->note);
                        m += b;
                    }
                    bool usbT = usbRow && (Config::zifi_transport == 1);
                    if (usbT) menu_curopt = 2;
                    else if (Config::zifi_tx_pin == BoardPins::PIN_OFF) menu_curopt = 1;
                    else {
                        uint8_t rtx, rrx;
                        BoardPins::resolveZifiPins(Config::zifi_tx_pin, Config::zifi_rx_pin, rtx, rrx);
                        menu_curopt = 2 + usbRow;
                        for (int i = 0; i < nopt; i++)
                            if (BoardPins::zifiPair(i)->tx == rtx) { menu_curopt = i + 2 + usbRow; break; }
                    }
                    uint8_t sel = menuRun(m);
                    if (sel > 0) {
                        VIDEO::SaveRect.restore_last();
                        bool conflict = false;
                        bool wasUsb = (Config::zifi_transport == 1);
                        if (sel == 1) {                       // Off
                            Config::zifi_transport = 0;
                            Config::zifi_tx_pin = Config::zifi_rx_pin = BoardPins::PIN_OFF;
                        } else if (usbRow && sel == 2) {      // USB (CH340)
                            Config::zifi_transport = 1;
                        } else {                              // a GPIO pair
                            const BoardPins::UartPair* p = BoardPins::zifiPair(sel - 2 - usbRow);
                            if (p) { Config::zifi_transport = 0; Config::zifi_tx_pin = p->tx; Config::zifi_rx_pin = p->rx; conflict = p->note[0] != 0; }
                        }
                        Config::save();
                        // Re-apply now whenever the link is up — for the NIC *or* WiFi
                        // (independent users of the shared transport).
                        if (ZiFi::linkUp()) { ZiFi::deinit(); ZiFi::init(); }
                        refreshNetStatus();
                        // A conflicting GPIO pair, or any switch into/out of USB, wants
                        // a reboot so displaced peripherals release/grab their pins.
                        bool nowUsb = (Config::zifi_transport == 1);
                        if ((conflict || wasUsb != nowUsb) &&
                            OSD::msgDialog(MENU_ZIFI_TRANSPORT_TITLE[Config::lang],
                                           OSD_DLG_APPLYREBOOT[Config::lang]) == DLG_YES)
                            OSD::esp_hard_reset();  // never returns
                        // No reboot (or declined) → recover the WiFi link on the new transport.
                        reconnectWifiIfNeeded();
                    }
                };
                auto pickBaud = [&]() {
                    menu_level = 3; menu_saverect = true;
                    static const uint32_t bauds[] = { 115200, 230400, 460800, 921600 };
                    const int NB = 4;
                    string m = string(MENU_BAUD_TITLE[Config::lang]) + "\n";
                    for (int i = 0; i < NB; i++) { char b[16]; snprintf(b, sizeof(b), "%u\n", (unsigned)bauds[i]); m += b; }
                    menu_curopt = 1;
                    for (int i = 0; i < NB; i++) if (bauds[i] == Config::zifi_baud) { menu_curopt = i + 1; break; }
                    uint8_t sel = menuRun(m);
                    if (sel > 0) {
                        VIDEO::SaveRect.restore_last();
                        uint32_t nb = bauds[sel - 1];
                        if (nb != Config::zifi_baud) {
                            Config::zifi_baud = nb;
                            Config::saveWifiConfig();
                            // deinit resets the ESP to 115200, init re-handshakes at the new rate.
                            // Apply now whenever the link is up (NIC or WiFi), not just for the NIC.
                            if (ZiFi::linkUp()) { ZiFi::deinit(); ZiFi::init(); }
                            refreshNetStatus();
                            reconnectWifiIfNeeded();  // recover the link if the baud switch dropped it
                        }
                    }
                };
                auto pickTz = [&]() {
                    menu_level = 3; menu_saverect = true;
                    string m = string(MENU_TZ_TITLE[Config::lang]) + "\n";
                    for (int tz = -12; tz <= 14; tz++) { char b[16]; snprintf(b, sizeof(b), "UTC%+d\n", tz); m += b; }
                    menu_curopt = Config::wifi_tz + 13;
                    uint8_t sel = menuRun(m);
                    if (sel > 0) {
                        Config::wifi_tz = (signed char)((int)sel - 13);
                        Config::saveWifiConfig();
                        VIDEO::SaveRect.restore_last();
                    }
                };
                auto doSync = [&]() {
                    OSD::osdCenteredMsg(MSG_RTC_SYNCING[Config::lang], LEVEL_INFO, 0);
                    string when;
                    if (ZiFiAT::syncTime(Config::wifi_tz, when) == ZiFiAT::OK)
                        OSD::osdCenteredMsg(string(MSG_RTC_SYNCED[Config::lang]) + "\n" + when, LEVEL_INFO, 3000);
                    else
                        OSD::osdCenteredMsg(MSG_RTC_SYNC_ERR[Config::lang], LEVEL_WARN, 3000);
                };
                // ── ESP01 submenu (level 2): Transport / Baud / Time zone / Sync time ──
                auto esp01Menu = [&]() {
                    menu_saverect = true; menu_curopt = 1;
                    while (1) {
                        menu_level = 2;
                        // First row shows the current transport: USB, Off, or GPIO pins.
                        char gpio[44];
                        bool usbT = false;
#if defined(KBDUSB)
                        usbT = (Config::zifi_transport == 1);
#endif
                        if (usbT)
                            snprintf(gpio, sizeof(gpio), "Transport: %s\t>", MENU_ZIFI_USB_LABEL[Config::lang]);
                        else if (Config::zifi_tx_pin == BoardPins::PIN_OFF)
                            snprintf(gpio, sizeof(gpio), "Transport: Off\t>");
                        else {
                            uint8_t dtx, drx;
                            BoardPins::resolveZifiPins(Config::zifi_tx_pin, Config::zifi_rx_pin, dtx, drx);
                            snprintf(gpio, sizeof(gpio), "Transport: %u/%u%s\t>", dtx, drx,
                                     Config::zifi_tx_pin == BoardPins::PIN_DEFAULT ? " (def)" : "");
                        }
                        char baud[32]; snprintf(baud, sizeof(baud), "Baud %u\t>", (unsigned)Config::zifi_baud);
                        string m = string(MENU_ESP01_TITLE[Config::lang]) + "\n"
                                 + gpio + "\n" + baud + "\n"
                                 + (Config::lang ? "Zona horaria\t>\n" : "Time zone\t>\n")
                                 + (Config::lang ? "Sincronizar hora\n" : "Sync time (SNTP)\n");
                        uint8_t e = menuRun(m);
                        if (e == 0) break; // Esc → back to Network (menuRun popped our rect)
                        if      (e == 1) pickTransport();
                        else if (e == 2) pickBaud();
                        else if (e == 3) pickTz();
                        else if (e == 4) doSync();
                        menu_curopt = e;
                        menu_saverect = false; // only the first ESP01 menuRun saves/positions
                    }
                };
                // ── WiFi connect/disconnect (level 2 list) ──
                auto doWifi = [&]() {
                    if (Config::wifi_enabled) {
                        // WiFi is on → this row turns it off. Show the live IP when the ESP
                        // is associated; otherwise just confirm the switch-off (enabled but
                        // offline — e.g. autoconnect hasn't finished / AP out of range).
                        string title = st_conn ? st_ssid : string("WiFi");
                        string msg   = (st_conn ? st_ip + "  " : string())
                                     + string(MSG_WIFI_DISCONNECT_Q[Config::lang]);
                        if (OSD::msgDialog(title, msg) == DLG_YES) {
                            ZiFiAT::disconnect();
                            Config::wifi_enabled = false;
                            // The NIC is purely a layer on top of WiFi — it cannot stay on
                            // once WiFi is off. Drop it too (NVS-persisted separately).
                            if (Config::zifi_enabled) {
                                Config::zifi_enabled = 0;
                                ZiFi::enabled = 0;
                                Config::save();
                            }
                            // Nobody left using the shared UART link → tear it down.
                            if (!ZiFiAT::connected) ZiFi::deinit();
                            Config::saveWifiConfig();
                            OSD::osdCenteredMsg(MSG_WIFI_DISCONNECTED[Config::lang], LEVEL_INFO, 1500);
                        }
                    } else {
                        // ONE live ESP-01 log window for the whole flow: scan → SSID pick
                        // → password → connect → SNTP. It stays up the entire time (the
                        // SSID menu and password box draw over it and restore their own
                        // regions). On error it does NOT auto-close — it holds until a
                        // keypress so the failure stays readable. SaveRect: the window is
                        // pushed once here and popped on every exit path. menuRun, on a
                        // selection, leaves its own save for us to pop (→ window reappears);
                        // on Esc it restores itself, so we pop only the window.
                        ftpd_log_count = 0;
                        s_wifilog_sub = MSG_WIFI_SCANNING[Config::lang];
                        unsigned short wlx = OSD::scrAlignCenterX(OSD_W);
                        unsigned short wly = OSD::scrAlignCenterY(OSD_H);
                        VIDEO::SaveRect.save(wlx, wly, OSD_W, OSD_H);   // [window]
                        wifiLogDraw();
                        ZiFiAT::log_cb = wifiLogLine;
                        // static, not on the 4 KB core stack: 24 std::strings (~768 B)
                        // plus the nested connect+SNTP-sync call chain under do_OSD
                        // overflowed the stack (SIGBUS/stackOvf). Single-use, non-reentrant.
                        static string nets[24];
                        int n = ZiFiAT::scan(nets, 24);
                        ZiFiAT::log_cb = nullptr;             // no AT traffic during menu/password
                        if (n <= 0) {
                            wifiLogHold(MSG_WIFI_NO_NETS[Config::lang]); // error → hold open
                            VIDEO::SaveRect.restore_last();
                            return;
                        }
                        string m = string(MENU_WIFI_LIST_TITLE[Config::lang]) + "\n";
                        for (int i = 0; i < n; i++) m += nets[i] + "\n";
                        menu_level = 2; menu_saverect = true; menu_curopt = 1;
                        uint8_t sel = menuRun(m);            // draws over the window
                        if (sel <= 0) {                      // Esc: menuRun restored its own save
                            VIDEO::SaveRect.restore_last();  // pop [window]
                            return;
                        }
                        VIDEO::SaveRect.restore_last();      // pop menuRun's save → window reappears
                        string chosen = nets[sel - 1];
                        s_wifilog_sub = chosen;
                        wifiLogDraw();
                        string pass = wifiAskPassword(chosen); // box draws over the window
                        if (pass == "\x1B") {                // password cancelled
                            VIDEO::SaveRect.restore_last();  // pop [window]
                            return;
                        }
                        s_wifilog_sub = string(MSG_WIFI_CONNECTING[Config::lang]) + " " + chosen;
                        wifiLogDraw();                       // repaint over the box region
                        ZiFiAT::log_cb = wifiLogLine;
                        ZiFiAT::Status cst = ZiFiAT::connect(chosen, pass);
                        string when;
                        if (cst == ZiFiAT::OK && Config::rtc_enabled)
                            ZiFiAT::syncTime(Config::wifi_tz, when); // streamed too
                        ZiFiAT::log_cb = nullptr;
                        if (cst == ZiFiAT::OK) {
                            Config::wifi_ssid = chosen; Config::wifi_pass = pass; Config::wifi_enabled = true;
                            Config::saveWifiConfig();
                            wifiLogLine(MSG_WIFI_CONNECTED[Config::lang]);
                            sleep_ms(900);                   // brief, then auto-close on success
                            VIDEO::SaveRect.restore_last();  // pop [window]
                            string msg = string(MSG_WIFI_CONNECTED[Config::lang]) + "\n" + ZiFiAT::current_ip;
                            if (!when.empty()) msg += "\n" + when;
                            OSD::osdCenteredMsg(msg, LEVEL_INFO, 2500);
                        } else {
                            wifiLogHold(MSG_WIFI_CONNECT_ERR[Config::lang]); // error → hold open
                            VIDEO::SaveRect.restore_last();  // pop [window]
                        }
                    }
                };
                // ── ZiFi NIC on/off (level 2) ──
                auto doNic = [&]() {
                    // The NIC is only the guest-port (0xEF/16550) emulation layered on top
                    // of WiFi — it can't be enabled while WiFi is off (and WiFi-off already
                    // forces it off), so there's nothing to toggle. Point the user at WiFi.
                    if (!Config::wifi_enabled) {
                        OSD::osdCenteredMsg(Config::lang ? "Active WiFi primero"
                                                         : "Enable WiFi first", LEVEL_WARN, 2500);
                        return;
                    }
                    menu_level = 2; menu_saverect = true; menu_curopt = Config::zifi_enabled + 1;
                    uint8_t zn = menuRun(MENU_ZIFI_NIC[Config::lang]);
                    if (zn > 0) {
                        uint8_t prevZ = Config::zifi_enabled;
                        Config::zifi_enabled = zn - 1;
#if !PICO_RP2040
                        // Turning the NIC on claims ~12 KB heap rings — check the SRAM budget.
                        // Freeing reboot never returns; denied/cancelled reverts.
                        if (Config::zifi_enabled && !prevZ &&
                            !OSD::featureBudgetGate(Subsystems::FEAT_ZIFI)) {
                            Config::zifi_enabled = prevZ;
                            ZiFi::enabled = Config::zifi_enabled;
                            VIDEO::SaveRect.restore_last();
                            return;
                        }
#endif
                        ZiFi::enabled = Config::zifi_enabled; // runtime mirror gates tick()+ports
                        // The NIC toggle ONLY gates the 0xEF port emulation — it is
                        // independent of WiFi. Bring the shared UART link up when the NIC
                        // goes on; when it goes off only tear the link down if WiFi isn't
                        // using it, otherwise we'd disconnect WiFi as a side effect.
                        if (Config::zifi_enabled) ZiFi::init();
                        else if (!ZiFiAT::connected) ZiFi::deinit();
                        Config::save();
                        VIDEO::SaveRect.restore_last();
                        refreshNetStatus();
                        if (Config::zifi_enabled && BoardPins::zifiActiveNote()[0] &&
                            OSD::msgDialog("", OSD_DLG_APPLYREBOOT[Config::lang]) == DLG_YES)
                            OSD::esp_hard_reset();
                    }
                };

                refreshNetStatus();
                while (1) {
                    if (OSD::net_launch_close) break;   // remoteFileDialog launched a /tmp download → close OSD
                    menu_level = 1;
                    // Row 2: live WiFi status, padded to a fixed width so geometry is stable.
                    string st;
                    if (st_conn) {
                        const char* pre = "WiFi On ";
                        string ssid = st_ssid, ip = st_ip;
                        int avail = 32 - (int)strlen(pre) - 1 - (int)ip.size();
                        if (avail < 0) avail = 0;
                        if ((int)ssid.size() > avail) ssid.resize(avail);
                        st = string(pre) + ssid + " " + ip;
                    } else if (ZiFiAT::autoSyncBusy()) {
                        // Boot auto-connect (CWJAP→SNTP) still running — don't claim "Off".
                        st = "WiFi connecting...";
                    } else if (Config::wifi_enabled) {
                        // Enabled by the user but not currently associated (out of range /
                        // autoconnect not run yet). Still "on" — selecting the row turns it off.
                        st = "WiFi On (offline)";
                    } else st = "WiFi Off";
                    if (st.size() < 32) st.append(32 - st.size(), ' '); else st.resize(32);

                    string nm = string(Config::lang ? "Red\n" : "Network\n")
                              + string(MENU_ESP01_TITLE[Config::lang]) + "\t>\n"  // 1 ESP01
                              + st + "\n";                                        // 2 WiFi
                    // File transfer (Remote) and Online archives now live under F5
                    // (location picker) — only the server/diagnostic rows remain here.
#if ZIFI_NET_CLIENT
                    nm += (Config::lang ? "Servidor FTP\t>\n" : "FTP Server\t>\n");           // 3
                    nm += string(MENU_HTTP_TEST_ITEM[Config::lang]);              // 4
#endif
                    nm += "ZiFi NIC\t>\n";                                        // 3 or 5

                    uint8_t net_opt = menuRun(nm);
                    if (net_opt == 0) break;

                    if      (net_opt == 1) esp01Menu();
                    else if (net_opt == 2) doWifi();
#if ZIFI_NET_CLIENT
                    else if (net_opt == 3) { ftpServerRun(); }
                    else if (net_opt == 4) { netHttpTest(); }
                    else if (net_opt == 5) doNic();
#else
                    else if (net_opt == 3) doNic();
#endif
                    refreshNetStatus();
                    menu_curopt = net_opt;
                    // Only the FIRST Network menuRun may save/position (see existing note):
                    // keep menu_saverect false on re-entry so the window doesn't march down.
                    menu_saverect = false;
                }
                menu_curopt = 10;
            }
#endif
#if !PICO_RP2040
            else if (opt == 11) { // ZX Keyboard — bitmap overlay
#else
            else if (opt == 9) { // ZX Keyboard (RP2040: Network+Hardware removed)
#endif
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
                // Draw bitmap: packed 4 bits/pixel, palette index 1..7 (0 = skip).
                // Black (idx 0) and transparent pixels are not drawn — the area is
                // already filled with paper black above, so the result is identical.
                for (int y = 0; y < kbd_h; y++) {
                    for (int x = 0; x < kbd_w; x++) {
                        int i = x + y * kbd_w;
                        uint8_t idx = (kbd_img[i >> 1] >> ((i & 1) << 2)) & 0x0F;
                        if (!idx) continue;
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
#if !PICO_RP2040
            else if (opt == 12) { // Help — dynamic from hotkeys
#else
            else if (opt == 10) { // Help — dynamic from hotkeys (RP2040 shifted)
#endif
                // Build index of visible hotkeys (no large buffer needed)
                auto descs = Config::lang ? hkDescES : hkDescEN;
                const int maxCols = osdMaxCols();
                const int descCol = 16;
                // Profi/Karabas-Pro "Menu"-key (Win/GUI) combos — not configurable
                // hotkeys (handled in ESPectrum::processKeyboard), so they're
                // listed here as a static section when a Profi machine is active.
                static const char* const profiKeys[] = {
                    "Menu+F1-F4", "Menu+F5", "Menu+F7", "Menu+F11",
                    "Menu+F12", "Menu+Tab", "Menu+J", "Menu+Esc",
                };
                static const char* const profiDescEN[] = {
                    "ROMSET 0-3 select", "Turbo FDC", "AY stereo mode", "CPU speed",
                    "NMI", "Swap drives A/B", "Joystick type", "Main menu",
                };
                static const char* const profiDescES[] = {
                    "Sel. ROMSET 0-3", "Turbo FDC", "Estereo AY", "Velocidad CPU",
                    "NMI", "Discos A/B", "Tipo joystick", "Menu principal",
                };
                const int profiN = (int)(sizeof(profiKeys) / sizeof(profiKeys[0]));
                const bool showProfi = Z80Ops::isProfi;
                // -3 = Profi section header, -(4+p) = Profi line p,
                // -2 = PrtScr, -1 = ScrollLk, 0..HK_COUNT-1 = hotkey index
                int8_t hkOrder[Config::HK_COUNT + 2 + 16];
                int nlines = 0;
                for (int i = 0; i < Config::HK_COUNT; i++) {
                    if (Config::hotkeys[i].vk == (uint16_t)fabgl::VK_NONE) continue;
                    hkOrder[nlines++] = (int8_t)i;
                }
                hkOrder[nlines++] = -2; // PrtScr
                hkOrder[nlines++] = -1; // ScrollLk
                if (showProfi) {
                    hkOrder[nlines++] = -3; // section header
                    for (int p = 0; p < profiN; p++) hkOrder[nlines++] = (int8_t)(-(4 + p));
                }

                // Format one help line into buf (42 bytes max)
                auto fmtLine = [&](int idx, char *buf) {
                    const char *key, *desc;
                    char keybuf[16];
                    if (idx == -3) { // Profi/Karabas section header (no key column)
                        snprintf(buf, 42, " Karabas (Menu=Win):");
                        int len = strlen(buf);
                        if (len < maxCols) { memset(buf + len, ' ', maxCols - len); buf[maxCols] = 0; }
                        else buf[maxCols] = 0;
                        return;
                    }
                    if (idx <= -4) { int p = -4 - idx; key = profiKeys[p]; desc = Config::lang ? profiDescES[p] : profiDescEN[p]; }
                    // On Profi plain PrtScr toggles the XT keyboard; BMP capture
                    // moves to Alt+PrtScr (see ESPectrum::processKeyboard).
                    else if (idx == -2) {
                        if (showProfi) { key = "PrtScr"; desc = Config::lang ? "Teclado XT" : "XT keyboard"; }
                        else { key = "PrtScr"; desc = Config::lang ? "Captura BMP" : "BMP capture"; }
                    }
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
#if !PICO_RP2040
            else if (opt == 13) { // About
#else
            else if (opt == 11) { // About (RP2040 shifted)
#endif
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
            // TFT: RP2350 opt 13; RP2040-TFT opt 12 (Hardware removed, no Network row).
#if PICO_RP2040
            else if (FileUtils::fsMount && opt == 12) { // TFT
#else
            else if (FileUtils::fsMount && opt == 13) { // TFT
#endif
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

// LED indicators legend panel: draws all glyphs with labels in a single column
void OSD::showLedLegend() {
    struct Entry { LED::Id id; const char* label; };
    static const Entry entries[] = {
        { LED::TAPE,      "Tape (EAR)"      },
        { LED::FDD,       "Floppy/TR-DOS"   },
        { LED::SD,        "DivMMC/esxDOS"   },
        { LED::ZCTRL,     "Z-Controller"    },
        { LED::IDE,       "IDE/HDD"         },
        { LED::BEEPER,    "Beeper"          },
        { LED::AY,        "AY-3-8912"       },
        { LED::COVOX,     "Covox DAC"       },
        { LED::SAA,       "SAA1099"         },
        { LED::MIDI,      "MIDI"            },
        { LED::GS,        "General Sound"   },
        { LED::ULAPLUS,    "ULA+"            },
        { LED::TIMEX,      "Timex SCLD"     },
        { LED::GIGASCREEN, "Gigascreen"     },
        { LED::RAM,        "RAM paging"     },
        { LED::DMA,       "Z80 DMA"         },
        { LED::KEMPJOY,   "Kempston joy"    },
        { LED::KEMPMOUSE, "Kempston mouse"  },
        { LED::NET,       "Network (ZiFi)"  },
    };
    static constexpr int N = sizeof(entries) / sizeof(entries[0]);

    static constexpr int row_h = 11;  // 8px sprite + 3px gap
    static constexpr int W = 160;
    static constexpr int H = 2 + OSD_FONT_H + 4 + N * row_h + 4;  // title + rows + padding

    unsigned short ox = scrAlignCenterX(W);
    unsigned short oy = scrAlignCenterY(H);
    uint8_t paper      = zxColor(1, 0);
    uint8_t ink        = zxColor(7, 1);
    uint8_t title_paper = zxColor(5, 1);
    uint8_t title_ink   = zxColor(0, 0);

    VIDEO::SaveRect.save(ox, oy, W, H);
    VIDEO::vga.fillRect(ox, oy, W, H, paper);
    VIDEO::vga.rect(ox, oy, W, H, zxColor(0, 0));
    VIDEO::vga.rect(ox + 1, oy + 1, W - 2, H - 2, zxColor(7, 0));

    // Title bar
    VIDEO::vga.fillRect(ox + 2, oy + 2, W - 4, OSD_FONT_H, title_paper);
    VIDEO::vga.setTextColor(title_ink, title_paper);
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(ox + OSD_FONT_W, oy + 2);
    VIDEO::vga.print(Config::lang ? "Leyenda" : "LED legend");

    VIDEO::vga.setTextColor(ink, paper);

    int top_y = oy + 2 + OSD_FONT_H + 4;

    for (int i = 0; i < N; i++) {
        int gx = ox + 6;
        int gy = top_y + i * row_h;
        LED::drawGlyph(entries[i].id, gx, gy + 1, ink - 1, paper);
        VIDEO::vga.setCursor(gx + 10, gy + 1);
        VIDEO::vga.print(entries[i].label);
    }

    // Drain any pending keys (e.g. Enter from menu selection), then wait for new keypress
    auto Kbd = ESPectrum::PS2Controller.keyboard();
    fabgl::VirtualKeyItem item;
    while (Kbd->virtualKeyAvailable()) Kbd->getNextVirtualKey(&item);
    while (1) {
        if (Kbd->virtualKeyAvailable()) {
            Kbd->getNextVirtualKey(&item);
            if (item.down) break;
        }
    }

    VIDEO::SaveRect.restore_last();
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

const char* const mnemCB[256] = {
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
    static const char* fname = DUMP_LOG_PATH;
    FIL* f = fopen2(fname, FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) {
        FileUtils::mkdirParents(CONFIG_DIR);
        f = fopen2(fname, FA_WRITE | FA_CREATE_ALWAYS);
        if (!f) return;
    }

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

    snprintf(line, sizeof(line), "portDFFD: %02X  (CPM=%d ROM14=%d DS80=%d NOROM=%d)\n",
        Ports::portDFFD, (Ports::portDFFD & 0x20) != 0, MemESP::romLatch,
        (Ports::portDFFD & 0x80) != 0, (Ports::portDFFD & 0x10) != 0);
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

    snprintf(line, sizeof(line), "  drive=%d side=%d dsr=%d retry=%d\n",
        wd.diskS, wd.side, wd.dsr, wd.retry);
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
        if (MemESP::ramCurrent[0] && MemESP::ramCurrent[0] < (uint8_t*)0x11000000)
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
            if (Nextkey.vk == fabgl::VK_F9 && !alt) {
                pokeDialog();
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
                        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(3); // slot 3, not bankLatch
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
                        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(3); // slot 3, not bankLatch
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
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
extern "C" uint8_t linkVGA01;
#endif
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

// Largest single block that malloc() can actually satisfy RIGHT NOW, without
// triggering the SDK's panic-on-OOM. getContiguousHeap() only measures the sbrk
// top gap and is blind to freed blocks in the allocator's free-list (e.g. a 38 KB
// prevFB freed when Gigascreen was turned off), so it badly under-reports after a
// disable→re-enable. getFreeHeap() (total free) over-reports on a fragmented heap.
// This probes the real (non-panicking) allocator by binary search: the only metric
// that reflects what a single allocation can really get.
extern "C" void* __real_malloc(size_t);
extern "C" void  __real_free(void*);
extern "C" size_t getLargestAllocatable(void) {
    extern size_t getFreeHeap(void);
    size_t hi = getFreeHeap();          // a single block can't exceed total free
    if (hi == 0) return 0;
    size_t lo = 0;
    while (lo < hi) {
        size_t mid = lo + (hi - lo + 1) / 2;
        void* p = __real_malloc(mid);
        if (p) { __real_free(p); lo = mid; } // fits → search higher
        else   { hi = mid - 1; }             // too big → search lower
    }
    return lo;
}

// Generic read-only text dialog with vertical scroll
void OSD::showTextDialog(const char* title, const char* text, bool blocking, int* scroll_state) {
    if (blocking) {
        click();
        unsigned short sx = scrAlignCenterX(OSD_W);
        unsigned short sy = scrAlignCenterY(OSD_H);
        VIDEO::SaveRect.save(sx, sy, OSD_W, OSD_H);
    }

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
    int scroll_local = 0;
    int& scroll = scroll_state ? *scroll_state : scroll_local;
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
    do {
        if (needRedraw) {
            drawContent();
            needRedraw = false;
        }
        if (!ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (blocking) { sleep_ms(5); continue; }
            else break;  // очередь пуста — выходим, caller сам перерисует позже
        }
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
    } while(blocking);
    if (blocking) VIDEO::SaveRect.restore_last();
}

// 0-based line index of the Uptime row in the info text — lets HWInfo's live
// loop repaint just that one row between full redraws.
static int s_hwinfo_uptime_line = -1;

extern "C" uint32_t uptime_seconds(void); // main.cpp — survives F12/watchdog reboots

static int formatUptimeLine(char* out, int outsz) {
    // Seconds since POWER-ON: the hardware timer resets on every watchdog
    // reboot (F12 / video-mode switch), so main.cpp accumulates the running
    // total across those in a watchdog scratch register.
    uint32_t up_s = uptime_seconds();
    return snprintf(out, outsz, " Uptime         : %dd %02d:%02d:%02d",
        (int)(up_s / 86400), (int)(up_s / 3600 % 24),
        (int)(up_s / 60 % 60), (int)(up_s % 60));
}

static void buildHWInfoText() {
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
            chip_is_rp2350a() ? "A" : "B", (int)cpu_hz,
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
#if !PICO_RP2040
        if (flash_qe) {
            pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
                " Flash QE bit   : %s\n", flash_qe_text());
            if (flash_qe >= 4)
                pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
                    " Flash QE diag  : %02X %02X %02X %02X %02X %02X\n",
                    flash_qe_diag[0], flash_qe_diag[1], flash_qe_diag[2],
                    flash_qe_diag[3], flash_qe_diag[4], flash_qe_diag[5]);
        }
#endif
    }

#ifndef MURM2
    {
        uint32_t psram32 = psram_size();
        if (psram32) {
            // ID is immutable — read once, not on every 1 Hz refresh (SPI
            // transaction under PSRAM_SPINLOCK, GS may be running on core1)
            static uint8_t rx8[8];
            static bool rx8_valid = false;
            if (!rx8_valid) { psram_id(rx8); rx8_valid = true; }
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

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
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

    {
        int ln = 0;
        for (int i = 0; i < pos; i++) if (hwtext[i] == '\n') ln++;
        s_hwinfo_uptime_line = ln;
        pos += formatUptimeLine(hwtext + pos, sizeof(hwtext) - pos);
        if (pos < (int)sizeof(hwtext) - 1) { hwtext[pos++] = '\n'; hwtext[pos] = '\0'; }
    }

    pos += snprintf(hwtext + pos, sizeof(hwtext) - pos,
        "\n"
        " Built at %s %s\n"
        " branch '%s'\n"
        " commit [%s]\n"
        " %s\n",
        __DATE__, __TIME__, PICO_GIT_BRANCH, PICO_GIT_COMMIT, PICO_BUILD_NAME);

}

void OSD::HWInfo() {
    // Live variant (same pattern as HIDDevices): rebuild text at 1 Hz so
    // Uptime and Free RAM tick while the dialog is open.
    unsigned short sx = scrAlignCenterX(OSD_W);
    unsigned short sy = scrAlignCenterY(OSD_H);
    VIDEO::SaveRect.save(sx, sy, OSD_W, OSD_H);

    fabgl::VirtualKeyItem Nextkey;
    uint32_t last_draw = 0;
    int scroll = 0;

    bool down = false;
    bool full = true; // full redraw (initial + after scroll keys), else Uptime row only
    while (true) {
        // Non-blocking key check
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if (down && !Nextkey.down && (is_enter(Nextkey.vk) || is_back(Nextkey.vk))) {
                click();
                break;
            }
            if (Nextkey.down) {
                down = (is_enter(Nextkey.vk) || is_back(Nextkey.vk));
                if (Nextkey.vk == fabgl::VK_UP || Nextkey.vk == fabgl::VK_DOWN || Nextkey.vk == fabgl::VK_PAGEUP || Nextkey.vk == fabgl::VK_PAGEDOWN) {
                    // Re-inject for showTextDialog's own scroll handling
                    ESPectrum::PS2Controller.keyboard()->injectVirtualKey(Nextkey.vk, true);
                }
                last_draw = 0; // force redraw
                full = true;
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_draw < 1000) {
            sleep_ms(5);
            continue;
        }
        last_draw = now;

        if (full) {
            buildHWInfoText();
            showTextDialog("Hardware info", osd_info_buf, false, &scroll);
            full = false;
        } else if (s_hwinfo_uptime_line >= 0) {
            // Repaint only the Uptime row in place — no full-dialog redraw
            int r = s_hwinfo_uptime_line - scroll;
            const int visRows = osdMaxRows() - 4;
            if (r >= 0 && r < visRows) {
                const int visCols = osdMaxCols();
                char row[42];
                int len = formatUptimeLine(row, sizeof(row));
                int w = visCols - 1; // keep last column intact (scrollbar lives there)
                if (len > w) len = w;
                memset(row + len, ' ', w - len);
                row[w] = '\0';
                osdAt(3 + r, 0);
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                VIDEO::vga.print(row);
            }
        }
    }

    VIDEO::SaveRect.restore_last();
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
            chip_is_rp2350a() ? "A" : "B",
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
#if !PICO_RP2040
        if (flash_qe) {
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " Flash QE bit   : %s\n", flash_qe_text());
            if (flash_qe >= 4)
                pos += snprintf(buf + pos, sizeof(buf) - pos,
                    " Flash QE diag  : %02X %02X %02X %02X %02X %02X\n",
                    flash_qe_diag[0], flash_qe_diag[1], flash_qe_diag[2],
                    flash_qe_diag[3], flash_qe_diag[4], flash_qe_diag[5]);
        }
#endif
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
        uint32_t ts_ch = chip_is_rp2350a() ? 4 : 8;
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

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
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
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
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

// Memory Info — overall FLASH/SRAM/PSRAM occupancy plus the Buffer tier pools and the
// SRAM cost of the features currently enabled via the Subsystem budget manager. Data
// sources: linker symbols (firmware/static/heap extents), getFreeHeap/getLargestAllocatable,
// butter_psram_size/psram_size + page counters, Buffer::poolStat() and Subsystems::feature*.
void OSD::MemoryInfo() {
    extern char __flash_binary_start, __flash_binary_end;  // pico-sdk linker symbols
    extern char end, __HeapLimit;                          // heap arena [end, __HeapLimit)

    char (&buf)[OSD_INFO_BUF_SZ] = osd_info_buf;
    int pos = 0;
    const int KB = 1024;

    // ── SRAM ───────────────────────────────────────────────────────────────────
    size_t sram_total  = (size_t)((uintptr_t)&__HeapLimit - SRAM_BASE);  // up to stack top
    size_t sram_static = (size_t)((uintptr_t)&end - SRAM_BASE);          // data + bss
    size_t heap_total  = (size_t)((uintptr_t)&__HeapLimit - (uintptr_t)&end);
    size_t heap_free   = getFreeHeap();
    size_t heap_used   = heap_total > heap_free ? heap_total - heap_free : 0;
    pos += snprintf(buf + pos, sizeof(buf) - pos, " SRAM (%d KB usable):\n", (int)(sram_total / KB));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  Static bss+data: %d KB\n", (int)(sram_static / KB));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  Heap used/total: %d/%d KB\n", (int)(heap_used / KB), (int)(heap_total / KB));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  Heap free      : %d KB\n", (int)(heap_free / KB));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  Largest block  : %d KB\n", (int)(getLargestAllocatable() / KB));

    // ── FLASH ──────────────────────────────────────────────────────────────────
    size_t fw = (size_t)((uintptr_t)&__flash_binary_end - (uintptr_t)&__flash_binary_start);
    uint32_t flash_total = (1u << rx[3]);
    pos += snprintf(buf + pos, sizeof(buf) - pos, " FLASH (%d MB):\n", (int)(flash_total >> 20));
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  Firmware       : %d KB\n", (int)(fw / KB));
    Buffer::PoolStat fp = Buffer::poolStat(Buffer::TIER_FLASH);
    if (fp.total)
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  Buffer pool    : %d/%d KB\n", (int)(fp.used / KB), (int)(fp.total / KB));

    // ── PSRAM ──────────────────────────────────────────────────────────────────
#ifdef BUTTER_PSRAM_GPIO
    if (butter_psram_size()) {
        uint32_t bsz = butter_psram_size();
        size_t emu = (size_t)butter_pages * MEM_PG_SZ;
        Buffer::PoolStat bp = Buffer::poolStat(Buffer::TIER_BUTTER);
        pos += snprintf(buf + pos, sizeof(buf) - pos, " Butter PSRAM (%d.%d MB):\n",
            (int)(bsz >> 20), (int)(((bsz & 0xFFFFF) * 10) >> 20));
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  Emu RAM pages  : %d KB\n", (int)(emu / KB));
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  Buffer arena   : %d/%d KB\n", (int)(bp.used / KB), (int)(bp.total / KB));
    }
#endif
    if (psram_size()) {
        uint32_t psz = psram_size();
        size_t emu = (size_t)psram_pages * MEM_PG_SZ;
        Buffer::PoolStat sp = Buffer::poolStat(Buffer::TIER_SPI);
        pos += snprintf(buf + pos, sizeof(buf) - pos, " SPI PSRAM (%d.%d MB):\n",
            (int)(psz >> 20), (int)(((psz & 0xFFFFF) * 10) >> 20));
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  Emu RAM pages  : %d KB\n", (int)(emu / KB));
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  Buffer arena   : %d/%d KB\n", (int)(sp.used / KB), (int)(sp.total / KB));
    }
    Buffer::PoolStat swp = Buffer::poolStat(Buffer::TIER_SWAP);
    if (swp.total)
        pos += snprintf(buf + pos, sizeof(buf) - pos, " SD swap pool   : %d/%d KB\n", (int)(swp.used / KB), (int)(swp.total / KB));

#if !PICO_RP2040
    // ── Enabled features (Subsystem SRAM budget) ────────────────────────────────
    using namespace Subsystems;
    pos += snprintf(buf + pos, sizeof(buf) - pos, "\n Enabled features (SRAM):\n");
    size_t feat_total = 0;
    for (int i = 0; i < FEAT_COUNT; i++) {
        FeatureId f = (FeatureId)i;
        if (!featureEnabled(f)) continue;
        size_t c = featureCost(f);
        feat_total += c;
        // Round up so a sub-KB feature (e.g. 512 B Z-Controller) isn't shown as 0 KB;
        // a genuinely-zero cost (Gigascreen on butter, ULA+/Timex) stays 0.
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  %-14s : %d KB\n", featureName(f), (int)((c + KB - 1) / KB));
    }
    pos += snprintf(buf + pos, sizeof(buf) - pos, "  %-14s : %d KB\n", "TOTAL", (int)((feat_total + KB - 1) / KB));

    // ── PSRAM by feature ────────────────────────────────────────────────────────
    // The big tiered buffers (GM.DLS bank, GS sample RAM, prevFB, DivMMC banks) live
    // in PSRAM, not the heap — invisible in the SRAM list above. List the PSRAM users.
    size_t psram_feat_total = 0;
    int psram_feat_n = 0;
    for (int i = 0; i < FEAT_COUNT; i++) {
        FeatureId f = (FeatureId)i;
        if (!featureEnabled(f)) continue;
        size_t pc = featurePsramCost(f);
        if (!pc) continue;
        if (!psram_feat_n++)
            pos += snprintf(buf + pos, sizeof(buf) - pos, "\n PSRAM by feature:\n");
        psram_feat_total += pc;
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  %-14s : %d KB\n", featureName(f), (int)((pc + KB - 1) / KB));
    }
    if (psram_feat_n)
        pos += snprintf(buf + pos, sizeof(buf) - pos, "  %-14s : %d KB\n", "TOTAL", (int)((psram_feat_total + KB - 1) / KB));
#endif

    showTextDialog("Memory Info", buf);
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
            Config::scanlines ? (Config::scanlines == 1 ? "Lvl 1" :
                                 Config::scanlines == 2 ? "Lvl 2" :
                                 Config::scanlines == 3 ? "Lvl 3" : "Lvl 4") : "Off",
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

        // SounDrive
        pos += snprintf(buf + pos, sizeof(buf) - pos, " SounDrive      : %s\n",
            Config::soundrive == 2 ? (Config::soundriveEnabled() ? "Auto (On)" : "Auto (Off)")
                                   : (Config::soundrive == 1 ? "On" : "Off"));

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
                " MIDI           : Software (%s)\n", presets[pi]);
        } else if (Config::midi == 4) {
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                " MIDI           : DLS (%s)\n",
                MidiSynth::bankReady() ? "bank OK" : "no bank");
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

        // IDE/HDD (NEMO/PROFI)
        if (Config::ide_scheme != 0) {
            static const char* idesc[] = { "Off", "NEMO", "PROFI" };
            int si = Config::ide_scheme; if (si > 2) si = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos, " IDE/HDD        : %s\n", idesc[si]);
            pos += snprintf(buf + pos, sizeof(buf) - pos, "  hd0           : ");
            pos += appendFilename(buf, pos, sizeof(buf), Config::ide_image[0], 19);
            pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
            pos += snprintf(buf + pos, sizeof(buf) - pos, "  hd1           : ");
            pos += appendFilename(buf, pos, sizeof(buf), Config::ide_image[1], 19);
            pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
        }
#endif

        // TR-DOS — available for Pentagon or Byte 128K
        {
            bool trdos_available = (Z80Ops::isPentagon || Z80Ops::isProfi) || (Z80Ops::is128 && Z80Ops::isByte);
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

extern "C" int hid_app_format_devices_info(char* buf, int bufsz);
extern "C" int xinput_app_format_devices_info(char* buf, int bufsz);

void OSD::HIDDevices() {
    extern int hid_app_format_devices_info(char* buf, int bufsz);
    extern int xinput_app_format_devices_info(char* buf, int bufsz);

    char (&buf)[OSD_INFO_BUF_SZ] = osd_info_buf;

    unsigned short sx = scrAlignCenterX(OSD_W);
    unsigned short sy = scrAlignCenterY(OSD_H);
    VIDEO::SaveRect.save(sx, sy, OSD_W, OSD_H);

    fabgl::VirtualKeyItem Nextkey;
    uint32_t last_draw = 0;
    int hid_scroll = 0;

    bool down = false;
    while (true) {
        // Non-blocking key check
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if (down && !Nextkey.down && (is_enter(Nextkey.vk) || is_back(Nextkey.vk))) {
                click();
                break;
            }
            if (Nextkey.down) {
                down = (is_enter(Nextkey.vk) || is_back(Nextkey.vk));
                if (Nextkey.vk == fabgl::VK_UP || Nextkey.vk == fabgl::VK_DOWN || Nextkey.vk == fabgl::VK_PAGEUP || Nextkey.vk == fabgl::VK_PAGEDOWN) {
                    ESPectrum::PS2Controller.keyboard()->injectVirtualKey(Nextkey.vk, true);
                }
                last_draw = 0; // forse redraw
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_draw < 1000) {
            sleep_ms(5);
            continue;
        }
        last_draw = now;

        buf[0] = '\0';
        int xpos = xinput_app_format_devices_info(buf, sizeof(buf));
        if (xpos < 0) xpos = 0;
        buf[xpos] = '\0';
        int hpos = hid_app_format_devices_info(buf + xpos, sizeof(buf) - xpos);
        if (hpos < 0) hpos = 0;
        buf[xpos + hpos] = '\0';
        if (xpos == 0 && hpos == 0)
            snprintf(buf, sizeof(buf), "No HID/XInput devices.\n");
#ifdef USE_NESPAD
        int used = (xpos + hpos > 0) ? xpos + hpos : (int)strlen(buf);
        snprintf(buf + used, sizeof(buf) - used,
            "NES1: %c%c%c%c %c%c%c%c %c%c%c%c\n"
            "NES2: %c%c%c%c %c%c%c%c %c%c%c%c\n"
            "joy1: U%d D%d L%d R%d  A%d B%d St%d Se%d\n"
            "joy2: U%d D%d L%d R%d  A%d B%d St%d Se%d\n",
            // NES1
            (nespad_state & DPAD_UP)     ? 'U' : '.',
            (nespad_state & DPAD_DOWN)   ? 'D' : '.',
            (nespad_state & DPAD_LEFT)   ? 'L' : '.',
            (nespad_state & DPAD_RIGHT)  ? 'R' : '.',
            (nespad_state & DPAD_A)      ? 'A' : '.',
            (nespad_state & DPAD_B)      ? 'B' : '.',
            (nespad_state & DPAD_X)      ? 'X' : '.',
            (nespad_state & DPAD_Y)      ? 'Y' : '.',
            (nespad_state & DPAD_LT)     ? '<' : '.',
            (nespad_state & DPAD_RT)     ? '>' : '.',
            (nespad_state & DPAD_START)  ? 'S' : '.',
            (nespad_state & DPAD_SELECT) ? 's' : '.',
            // NES2
            (nespad_state2 & DPAD_UP)     ? 'U' : '.',
            (nespad_state2 & DPAD_DOWN)   ? 'D' : '.',
            (nespad_state2 & DPAD_LEFT)   ? 'L' : '.',
            (nespad_state2 & DPAD_RIGHT)  ? 'R' : '.',
            (nespad_state2 & DPAD_A)      ? 'A' : '.',
            (nespad_state2 & DPAD_B)      ? 'B' : '.',
            (nespad_state2 & DPAD_X)      ? 'X' : '.',
            (nespad_state2 & DPAD_Y)      ? 'Y' : '.',
            (nespad_state2 & DPAD_LT)     ? '<' : '.',
            (nespad_state2 & DPAD_RT)     ? '>' : '.',
            (nespad_state2 & DPAD_START)  ? 'S' : '.',
            (nespad_state2 & DPAD_SELECT) ? 's' : '.',
            // gamepad1_bits
            (int)gamepad1_bits.up,    (int)gamepad1_bits.down,
            (int)gamepad1_bits.left,  (int)gamepad1_bits.right,
            (int)gamepad1_bits.a,     (int)gamepad1_bits.b,
            (int)gamepad1_bits.start, (int)gamepad1_bits.select,
            // gamepad2_bits
            (int)gamepad2_bits.up,    (int)gamepad2_bits.down,
            (int)gamepad2_bits.left,  (int)gamepad2_bits.right,
            (int)gamepad2_bits.a,     (int)gamepad2_bits.b,
            (int)gamepad2_bits.start, (int)gamepad2_bits.select
        );
#endif
        showTextDialog(Config::lang ? "Disp. HID live" : "HID devices (live)", buf, false, &hid_scroll);
    }

    VIDEO::SaveRect.restore_last();
}

#if !PICO_RP2040
extern "C" uint8_t __gm_bank_start[];   // shared flash region (RP2350) — ALF cart load target
#endif

static void __not_in_flash_func(flash_block)(const uint8_t* buffer, size_t flash_target_offset) {
    // ensure it is required to write block (may be, it is already the same)
    for (size_t i = 0; i < 512; ++i) {
        if (buffer[i] != *(uint8_t*)(XIP_BASE + flash_target_offset + i)) {
            goto flash_it;
        }
    }
    return;
flash_it:
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
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
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    #endif
}

#if !PICO_RP2040
bool OSD::loadAlfCart(const string& fname) {
    FIL* f = fopen2(fname.c_str(), FA_READ);
    if (!f) { OSD::osdCenteredMsg(OSD_NOROMFILE_ERR[Config::lang], LEVEL_WARN, 2000); return false; }
    size_t size = (size_t)f_size(f);
    fclose2(f);
    if (size == 0) {
        OSD::osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
        return false;
    }
    // Some distributions append a small text footer (e.g. "filename=.../size=.../crc32=...")
    // after the 1MB cart image. Clamp to 1MB; only the cart data is served, the tail ignored.
    if (size > (1ul << 20)) size = (1ul << 20);
    // Lazy load: the cart is served from SD on demand (like a wd1793 disk) — NO 1MB
    // flash write; banks fault in via #5F as the guest pages them (see AlfCart / Ports).
    // Switch into ALF IN PLACE (no reboot), the same way the Hardware menu and snapshot
    // loads switch machines: mount the cart, requestMachine + reset, then return so the
    // OSD closes cleanly into the running machine. GM.DLS no longer conflicts (the cart
    // does not use the shared flash region), so it is left untouched.
    Config::alfCartBanks = (uint8_t)((size + (16ul << 10) - 1) >> 14);   // ceil to 16K banks
    Config::alfCartPath  = fname;
    Config::arch = "ALF"; Config::romSet = "ALF1"; Config::pref_arch = "ALF";
    Config::save();
    // Mount FRESH every load — open a new FIL on the file as it is NOW. ZIP carts always
    // extract to the same temp path (/tmp/.zip_extract.rom), so reloading would otherwise
    // keep a stale handle open on a file that was unlinked+rewritten underneath it
    // (progressively empty catalog until a reboot). mount() closes any previous handle.
    if (!AlfCart::mount(fname)) {
        OSD::osdCenteredMsg("ALF cart mount failed", LEVEL_WARN, 2000);
        return false;
    }
    Config::alfCartBanks = (uint8_t)AlfCart::bankCount();
#if !PICO_RP2040
    // ALF uses neither General Sound nor Gigascreen — free their SRAM so the cart +
    // machine have headroom. Gigascreen frees live (prevFB via GsSubsys); General
    // Sound only frees across a reboot (no live deinit), so if it was on we reboot
    // into ALF clean (the cart path is persisted → boot re-mounts it).
    bool needGsReboot = (Config::gs_enabled != 0);
    Config::gs_enabled = 0;
    Config::gigascreen_enabled = false;
    Config::gigascreen_onoff   = 0;
    GsSubsys::request(false);     // release prevFB (applied at reset/reboot below)
    Config::save();
    // Reboot when GS was on (to actually reclaim it) OR when leaving Profi's forced-
    // SRAM layout (no butter PSRAM) — an in-place reset() frees neither the ~96 KB
    // Profi pages nor GS, and ALF would OOM. Boot re-mounts the cart from SD.
    if (needGsReboot ||
        (butter_psram_size() == 0 &&
         MemESP::ram[56].memType() == mem_type_t::POINTER)) {
        OSD::esp_hard_reset();   // never returns
        return true;
    }
#endif
    Config::requestMachine("ALF", "ALF1");   // in-place machine switch (no reboot)
    ESPectrum::reset();
    return true;
}
#endif

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
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
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
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
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
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
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
#if !PICO_RP2040
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
        // Load an ALF cartridge (up to 1MB) served lazily from SD on demand (like a
        // wd1793 disk — see AlfCart). No 1MB flash write; the file stays on SD and 16K
        // banks fault in as the guest pages them. Switches into ALF in place (no reboot).
        fclose2(f);
        return loadAlfCart(fname);   // lazy-mount from SD + switch into ALF in place
    }
#endif
    else if ( arch == 6 ) {
        if( bytesfirmware > (16ul << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        // Always flash into the dedicated custom slot and select it, so the
        // factory 5.03/5.04TM/5.05D images are never overwritten and the new
        // ROM is active after reboot (mirrors 48K -> "48Kcs").
        rom = gb_rom_4_trdos_custom;
        Config::trdosBios = 3;
        max_rom_size = 16ul << 10;
        dlgTitle += " TRDOS ";
        // TR-DOS ROM (rom[4]) is independent of the active machine and now
        // works with any of them, so we must NOT force Config::arch here —
        // doing so would yank the user out of their current machine into
        // Pentagon after the post-flash reboot. The custom slot is re-bound
        // unconditionally by Config::requestMachine() regardless of arch.
    }
    else if ( arch == 7 ) {
        if( bytesfirmware > (32ul << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        // Custom Pentagon ROM: the factory Pentagon is now a 101-byte overlay over the
        // Sinclair 128K base (no 32K blob), so a user ROM flashes into the shared 128K
        // custom slot and the machine switches to the "128Kcs" romset.
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
        rom = gb_rom_0_128k_custom;
#else
        rom = gb_rom_Alf_cart;
#endif
        max_rom_size = bytesfirmware > (16ul << 10) ? (32ul << 10) : (16ul << 10);
        dlgTitle += " Pentagon#0 ";
        Config::arch = "Pentagon";
        Config::romSet = "128Kcs";
        Config::romSetPent = "128Kcs";
        Config::pref_arch = "Pentagon";
        Config::pref_romSetPent = "128Kcs";
    }
    else if ( arch == 8 ) {
        if( bytesfirmware > (16 << 10) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
        rom = gb_rom_0_128k_custom;
#else
        rom = gb_rom_Alf_cart;
#endif
        max_rom_size = 16 << 10;
        dlgTitle += " Pentagon#1 ";
        Config::arch = "Pentagon";
        Config::romSet = "128Kcs";
        Config::romSetPent = "128Kcs";
        Config::pref_arch = "Pentagon";
        Config::pref_romSetPent = "128Kcs";
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

// ---------------------------------------------------------------------------
// Speed Test
// ---------------------------------------------------------------------------

// Sequential file R/W benchmark on one FatFs volume — measures the same
// FatFs+diskio path the emulator's own file I/O uses. benchPath selects the
// volume ("/bench.tmp" = SD, "USB:/..." = stick).
//
// NOTE on USB: block size does NOT change USB throughput. The RP2350 native
// USB host transfers bulk data at ~1 packet (64 B) per 1 ms SOF frame, so USB
// MSC is hard-capped near ~64 KB/s regardless of transfer size — measured a
// single 32 KB (64-sector) read10 at 558 ms, identical to 64 single-sector
// reads. It's a host-controller limit, not this code. See CLAUDE.md.
static bool benchFsSpeed(const char* benchPath, const char* volName,
                         const char* title, float& rd, float& wr) {
    static uint8_t io_buf[512];
    static FIL f;    // 2KB core stack — never put a FIL on it
    UINT bw, br;
    bool ok = false;
    char msg[24];

    snprintf(msg, sizeof(msg), "%s write...", volName);
    OSD::progressDialog(title, msg, 0, 0);
    memset(io_buf, 0x55, sizeof(io_buf));
    if (f_open(&f, benchPath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        uint64_t t0 = time_us_64();
        uint32_t total = 0;
        while (time_us_64() - t0 < 2000000ULL && total < 512u * 1024u) {
            if (f_write(&f, io_buf, sizeof(io_buf), &bw) != FR_OK) break;
            total += bw;
        }
        uint64_t elapsed = time_us_64() - t0;
        f_close(&f);
        if (elapsed > 0 && total > 0) {
            wr = (float)total / (float)elapsed;
            ok = true;
        }
    }

    snprintf(msg, sizeof(msg), "%s read...", volName);
    OSD::progressDialog(title, msg, 50, 1);
    if (f_open(&f, benchPath, FA_READ) == FR_OK) {
        uint64_t t0 = time_us_64();
        uint32_t total = 0;
        while (f_read(&f, io_buf, sizeof(io_buf), &br) == FR_OK && br > 0)
            total += br;
        uint64_t elapsed = time_us_64() - t0;
        f_close(&f);
        if (elapsed > 0 && total > 0)
            rd = (float)total / (float)elapsed;
    }
    f_unlink(benchPath);

    OSD::progressDialog(title, "", 100, 1);
    OSD::progressDialog("", "", 0, 2);
    return ok;
}

#if !PICO_RP2040 && ZIFI_NET_CLIENT
// NET download benchmark — GET the catalog's speedtest blob (512 KB of
// incompressible bytes published by gen_static.py) and count body bytes.
// Always the built-in Pages URL: a Config::catalog_host override may point at
// a dynamic /v1 proxy that doesn't serve the blob, and the point is to measure
// the device's real HTTPS-download path, not a LAN server.
// TLS handshake time and body throughput are reported separately — on a
// ~100 KB/s link the multi-second mbedTLS handshake would swamp the KB/s
// figure otherwise. Nothing is written to SD. ZiFiSock::sock_open already
// boostBaud()s the link, same as FTP/archive sessions.
#define SPEEDTEST_NET_URL "https://drewpo28.github.io/pico-spec-catalog/speedtest.bin"

struct NetBenchCtx {
    uint64_t t_start;      // just before HttpsGet::get
    uint64_t t_first;      // first body byte
    uint64_t t_end;        // last body byte
    uint64_t ui_last;      // progress-redraw rate limit
    uint32_t bytes;
    HttpsGet::Result r;
};

// Body cap: stop after this much measured time — a slow link (115200 baud)
// would otherwise take ~45 s for the full 512 KB. Aborting via the sink is
// fine: speed is computed from what arrived.
static const uint64_t NET_BENCH_BODY_US = 8 * 1000000ULL;

static bool netBenchSink(void* ctx, const uint8_t* data, size_t len) {
    (void)data;
    NetBenchCtx* c = (NetBenchCtx*)ctx;
    uint64_t now = time_us_64();
    if (!c->bytes) c->t_first = now;
    c->bytes += len;
    c->t_end = now;
    return now - c->t_first < NET_BENCH_BODY_US;
}

// Progress by bytes, rate-limited: the redraw runs between recv chunks and
// would otherwise eat into the measured throughput.
static bool netBenchProgress(void* ctx, uint32_t done, uint32_t total) {
    NetBenchCtx* c = (NetBenchCtx*)ctx;
    uint64_t now = time_us_64();
    if (total > 0 && now - c->ui_last > 250000ULL) {
        c->ui_last = now;
        OSD::progressDialog("", "", (int)((uint64_t)done * 100 / total), 1);
    }
    return true;
}

static void netBenchRun(void* p) {
    NetBenchCtx* c = (NetBenchCtx*)p;
    c->t_start = time_us_64();
    c->r = HttpsGet::get(SPEEDTEST_NET_URL, netBenchSink, c,
                         CONFIG_DIR "/cacert.pem", netBenchProgress, c);
    if (!c->t_end) c->t_end = time_us_64();
}

// Runs the GET on the big heap alt-stack (mbedTLS handshake doesn't fit the
// core stack). Returns false when no alt-stack could be allocated; success of
// the transfer itself is judged from ctx (2xx + bytes received — an abort by
// the time cap clears r.ok, but the sample is still valid).
static bool benchNetSpeed(NetBenchCtx& ctx, const char* title) {
    memset(&ctx, 0, sizeof(ctx));
    OSD::progressDialog(title, "NET download...", 0, 0);
    NetArenaLease lease;   // borrow the dormant Gigascreen prevFB if available
    size_t stksz = 12 * 1024;
    uint8_t* stk = netAltStackAlloc(stksz);
    if (!stk) {
        OSD::progressDialog("", "", 0, 2);
        return false;
    }
    void* top = (void*)(((uintptr_t)stk + stksz) & ~(uintptr_t)7);
    net_call_on_stack(top, netBenchRun, &ctx);
    Buffer::pfree(stk);
    OSD::progressDialog(title, "", 100, 1);
    OSD::progressDialog("", "", 0, 2);
    Debug::log("SpeedTest NET: status=%d bytes=%lu hs_us=%lu body_us=%lu",
               ctx.r.status, (unsigned long)ctx.bytes,
               (unsigned long)(ctx.bytes ? ctx.t_first - ctx.t_start : 0),
               (unsigned long)(ctx.bytes ? ctx.t_end - ctx.t_first : 0));
    return true;
}
#endif // !PICO_RP2040 && ZIFI_NET_CLIENT

void OSD::SpeedTest() {
    menu_level = 2;
    menu_curopt = 1;
    menu_saverect = true;

    while (1) {
        uint8_t st_opt = menuRun(MENU_SPEEDTEST[Config::lang]);
        if (st_opt == 0) break;

        // With the net client built in, row 6 is NET and "All tests" shifts to 7.
#if !PICO_RP2040 && ZIFI_NET_CLIENT
        const uint8_t all_opt = 7;
        const bool do_net   = (st_opt == 6 || st_opt == all_opt);
#else
        const uint8_t all_opt = 6;
#endif
        const bool do_cpu   = (st_opt == 1 || st_opt == all_opt);
        const bool do_sram  = (st_opt == 2 || st_opt == all_opt);
        const bool do_psram = (st_opt == 3 || st_opt == all_opt);
        const bool do_sd    = (st_opt == 4 || st_opt == all_opt);
        const bool do_usb   = (st_opt == 5 || st_opt == all_opt);

        const char* title = Config::lang ? "Test velocidad" : "Speed Test";

        float cpu_mips = 0.0f;
        float sram_rd = 0.0f, sram_wr = 0.0f;
        float spi_rd = 0.0f, spi_wr = 0.0f;
        float qspi_rd = 0.0f, qspi_wr = 0.0f;
        float sd_rd = 0.0f, sd_wr = 0.0f;
        float usb_rd = 0.0f, usb_wr = 0.0f;
        bool sd_ok = false, usb_ok = false;
        // usbRoot: there is no SD — the unprefixed volume IS the stick, so the
        // SD test would silently benchmark USB; report "No card" instead.
        const bool sd_present  = FileUtils::fsMount && !FileUtils::usbRoot;
        const bool usb_present = UsbMsc::ready();

        const bool has_spi  = psram_size() > 0;
        const bool has_qspi = butter_psram_size() > 0;

        // --- CPU MIPS ---
        if (do_cpu) {
            progressDialog(title, "CPU MIPS...", 0, 0);
            const int BATCH = 10000;
            uint32_t acc = 1;
            uint32_t iters = 0;
            uint64_t t0 = time_us_64();
            do {
                for (int i = 0; i < BATCH; i++)
                    acc = acc * 1664525u + 1013904223u;
                iters += BATCH;
            } while (time_us_64() - t0 < 500000ULL);
            volatile uint32_t _sink = acc; (void)_sink;
            uint64_t elapsed = time_us_64() - t0;
            // 2 integer ops per iteration (mul + add)
            cpu_mips = (float)iters * 2.0f / (float)elapsed;
            progressDialog(title, "", 100, 1);
            progressDialog("", "", 0, 2);
        }

        // --- SRAM R/W ---
        // osd_info_buf used as scratch; results saved in floats before we refill it
        if (do_sram) {
            char (&scratch)[OSD_INFO_BUF_SZ] = osd_info_buf;
            uint64_t t0, elapsed;
            uint32_t total;

            typedef uint32_t __attribute__((may_alias)) u32a;

            progressDialog(title, "SRAM write...", 0, 0);
            total = 0;
            t0 = time_us_64();
            do {
                char *p = scratch;
                for (uint32_t a = 0; a < OSD_INFO_BUF_SZ; a += 4)
                    *(u32a *)(p + a) = a;
                total += OSD_INFO_BUF_SZ;
            } while (time_us_64() - t0 < 300000ULL);
            elapsed = time_us_64() - t0;
            sram_wr = (float)total / (float)elapsed;

            progressDialog(title, "SRAM read...", 50, 1);
            uint32_t acc = 0;
            total = 0;
            t0 = time_us_64();
            do {
                const char *p = scratch;
                for (uint32_t a = 0; a < OSD_INFO_BUF_SZ; a += 4)
                    acc ^= *(const u32a *)(p + a);
                total += OSD_INFO_BUF_SZ;
            } while (time_us_64() - t0 < 300000ULL);
            volatile uint32_t _sink = acc; (void)_sink;
            elapsed = time_us_64() - t0;
            sram_rd = (float)total / (float)elapsed;

            progressDialog(title, "", 100, 1);
            progressDialog("", "", 0, 2);
        }

        // --- PSRAM ---
        if (do_psram) {
            // SPI PSRAM — burst via psram_write_range / psram_read_range
            if (has_spi) {
                char (&burst)[OSD_INFO_BUF_SZ] = osd_info_buf;
                uint64_t t0, elapsed;
                uint32_t total;

                progressDialog(title, "SPI PSRAM wr...", 0, 0);
                total = 0;
                t0 = time_us_64();
                do {
                    psram_write_range(0, (const uint8_t*)burst, OSD_INFO_BUF_SZ);
                    total += OSD_INFO_BUF_SZ;
                } while (time_us_64() - t0 < 500000ULL);
                elapsed = time_us_64() - t0;
                spi_wr = (float)total / (float)elapsed;

                progressDialog(title, "SPI PSRAM rd...", 50, 1);
                total = 0;
                t0 = time_us_64();
                do {
                    psram_read_range(0, (uint8_t*)burst, OSD_INFO_BUF_SZ);
                    total += OSD_INFO_BUF_SZ;
                } while (time_us_64() - t0 < 500000ULL);
                elapsed = time_us_64() - t0;
                spi_rd = (float)total / (float)elapsed;

                progressDialog(title, "", 100, 1);
                progressDialog("", "", 0, 2);
            }
            // QSPI/butter PSRAM — memory-mapped, use memset + sum loop
            if (has_qspi) {
                extern uint8_t* PSRAM_DATA;
                const uint32_t TEST_SZ = 0x10000u; // 64KB
                uint32_t sz = (uint32_t)butter_psram_size() < TEST_SZ
                            ? (uint32_t)butter_psram_size() : TEST_SZ;
                uint64_t t0, elapsed;
                uint32_t total;

                progressDialog(title, "QSPI PSRAM wr...", 0, 0);
                total = 0;
                t0 = time_us_64();
                do {
                    memset(PSRAM_DATA, 0xAA, sz);
                    total += sz;
                } while (time_us_64() - t0 < 300000ULL);
                elapsed = time_us_64() - t0;
                qspi_wr = (float)total / (float)elapsed;

                progressDialog(title, "QSPI PSRAM rd...", 50, 1);
                total = 0;
                t0 = time_us_64();
                do {
                    const uint32_t *p32 = (const uint32_t *)PSRAM_DATA;
                    uint32_t a;
                    for (a = 0; a < sz / 4; a++) {
                        if (p32[a] != 0xAAAAAAAAu) break;
                    }
                    total += a * 4;
                } while (time_us_64() - t0 < 300000ULL);
                elapsed = time_us_64() - t0;
                qspi_rd = (float)total / (float)elapsed;

                progressDialog(title, "", 100, 1);
                progressDialog("", "", 0, 2);
            }
        }

        // --- SD Card ---
        if (do_sd && sd_present)
            sd_ok = benchFsSpeed("/bench.tmp", "SD", title, sd_rd, sd_wr);

        // --- USB flash stick ---
        if (do_usb && usb_present)
            usb_ok = benchFsSpeed("USB:/bench.tmp", "USB", title, usb_rd, usb_wr);

        // --- NET (HTTPS download from the catalog Pages) ---
#if !PICO_RP2040 && ZIFI_NET_CLIENT
        NetBenchCtx net_ctx = {};
        bool net_wifi = false, net_ran = false;
        if (do_net) {
            string ssid, ip;
            net_wifi = ZiFiAT::getStatus(ssid, ip);
            if (net_wifi)
                net_ran = benchNetSpeed(net_ctx, title);
        }
#endif

        // --- Build result text ---
        char (&buf)[OSD_INFO_BUF_SZ] = osd_info_buf;
        int pos = 0;

        if (do_cpu) {
            uint32_t cpu_mhz = clock_get_hz(clk_sys) / 1000000u;
            pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                " CPU     : %u MHz\n"
                " MIPS    : %.1f\n\n",
                (unsigned)cpu_mhz, cpu_mips);
        }
        if (do_sram) {
            pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                " SRAM rd : %.1f MB/s\n"
                " SRAM wr : %.1f MB/s\n\n",
                sram_rd, sram_wr);
        }
        if (do_psram) {
            if (!has_spi && !has_qspi) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " PSRAM   : N/A\n\n");
            }
            if (has_spi) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " SPI PSRAM rd: %.1f MB/s\n"
                    " SPI PSRAM wr: %.1f MB/s\n\n",
                    spi_rd, spi_wr);
            }
            if (has_qspi) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " QSPI PSRAM rd: %.2f MB/s\n"
                    " QSPI PSRAM wr: %.2f MB/s\n\n",
                    qspi_rd, qspi_wr);
            }
        }
        if (do_sd) {
            if (!sd_present) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " SD      : No card\n\n");
            } else if (sd_ok) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " SD rd   : %.2f MB/s\n"
                    " SD wr   : %.2f MB/s\n\n",
                    sd_rd, sd_wr);
            } else {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " SD      : Error\n\n");
            }
        }
        if (do_usb) {
            if (!usb_present) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " USB     : No stick\n\n");
            } else if (usb_ok) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " USB rd  : %.2f MB/s\n"
                    " USB wr  : %.2f MB/s\n\n",
                    usb_rd, usb_wr);
            } else {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " USB     : Error\n\n");
            }
        }
#if !PICO_RP2040 && ZIFI_NET_CLIENT
        if (do_net) {
            // 2xx + bytes counts as success even when r.ok was cleared by the
            // sink's time cap — the sample is what we measure.
            if (!net_wifi) {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " NET     : No WiFi\n\n");
            } else if (net_ran && net_ctx.r.status >= 200 && net_ctx.r.status < 300 &&
                       net_ctx.bytes > 0 && net_ctx.t_end > net_ctx.t_first) {
                float kbs = (float)net_ctx.bytes * (1000000.0f / 1024.0f)
                          / (float)(net_ctx.t_end - net_ctx.t_first);
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " NET rd  : %.1f KB/s\n"
                    " TLS hshk: %lu ms\n\n",
                    kbs, (unsigned long)((net_ctx.t_first - net_ctx.t_start) / 1000));
            } else {
                pos += snprintf(buf + pos, OSD_INFO_BUF_SZ - pos,
                    " NET     : Error (%d)\n\n", net_ctx.r.status);
            }
        }
#endif

        showTextDialog(title, buf);
        menu_curopt = st_opt;
        menu_saverect = false;
    }
}

void OSD::progressDialog(const string& title, const string& msg, int percent, int action, bool cyrillic) {

    static unsigned short h;
    static unsigned short y;
    static unsigned short w;
    static unsigned short x;
    static unsigned short progress_x;
    static unsigned short progress_y;
    static unsigned int j;
    static bool cyr;   // remembered from SHOW so UPDATE/CLOSE keep the same face

    if (action == 0 ) { // SHOW

        cyr = cyrillic;

        h = (OSD_FONT_H * 6) + 2;
        y = scrAlignCenterY(h);

        // Transcode Cyrillic (online-catalog names) to CP1251 before measuring/truncating,
        // so widths are right and we never cut a multibyte UTF-8 sequence.
        string cmsg = cyr ? FileUtils::utf8ToCp1251(msg) : msg;
        string ctitle = cyr ? FileUtils::utf8ToCp1251(title) : title;

        size_t maxchars = (scrW / 6) - 4;
        string tmsg = cmsg.length() > maxchars ? cmsg.substr(0, maxchars) : cmsg;
        string ttitle = ctitle.length() > maxchars ? ctitle.substr(0, maxchars) : ctitle;

        w = (((tmsg.length() > ttitle.length() + 6 ? tmsg.length(): ttitle.length() + 6) + 2) * OSD_FONT_W) + 2;
        x = scrAlignCenterX(w);

        // Save backbuffer data
        VIDEO::SaveRect.save(x, y, w, h);

        // Set font
        VIDEO::vga.setFont(cyr ? Font6x8Cyr : Font6x8);

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
        VIDEO::vga.setFont(cyr ? Font6x8Cyr : Font6x8);
        string umsg = cyr ? FileUtils::utf8ToCp1251(msg) : msg;
        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));
        VIDEO::vga.setCursor(scrAlignCenterX(umsg.length() * OSD_FONT_W), y + 1 + (OSD_FONT_H * 2));
        VIDEO::vga.print(umsg.c_str());

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
    "None     \n"\
    "A-Z      \n"\
    "0-9      \n"\
    "Special  \n"\
    "Joystick \n"
#define MENU_JOYSELKEY_ES "Tecla    \n"\
    "None     \n"\
    "A-Z      \n"\
    "0-9      \n"\
    "Especial \n"\
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
    "Down\n"\
    "Enter\n"\
    "Caps\n"\
    "SymbShift\n"\
    "Brk/Space\n"\
    "Backspace\n"\
    "KP 0/Ins\n"\
    "KP ./Del\n"

#define MENU_JOY_KEMPSTON "DPAD\n"\
    "Left\n"\
    "Right\n"\
    "Up\n"\
    "Down\n"\
    "A\n"\
    "B\n"\
    "C\n"\
    "X\n"\
    "Y\n"\
    "Z\n"\
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
case fabgl::VK_JOY_C:
    return "  Joy.C  ";
case fabgl::VK_JOY_X:
    return "  Joy.X  ";
case fabgl::VK_JOY_Y:
    return "  Joy.Y  ";
case fabgl::VK_JOY_Z:
    return "  Joy.Z  ";
case fabgl::VK_JOY_L2:
    return "  Joy.L2 ";
case fabgl::VK_JOY_R2:
    return "  Joy.R2 ";
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

unsigned int joyControl[14][3]={
    {34,55,zxColor(0,0)},   // Left
    {87,55,zxColor(0,0)},   // Right
    {63,30,zxColor(0,0)},   // Up
    {63,78,zxColor(0,0)},   // Down
    {49,109,zxColor(0,0)},  // Start
    {130,109,zxColor(0,0)}, // Select
    {142,17,zxColor(0,0)},  // A (circle left of A cell)
    {222,17,zxColor(0,0)},  // B
    {142,65,zxColor(0,0)},  // C
    {142,41,zxColor(0,0)},  // X
    {222,41,zxColor(0,0)},  // Y
    {222,65,zxColor(0,0)},  // Z
    {142,89,zxColor(0,0)},  // L2 (circle left of L2 cell)
    {222,89,zxColor(0,0)}   // R2
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

    // SELECT text
    VIDEO::vga.setTextColor(joyControl[5][2], zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[5][0], y + joyControl[5][1]);
    VIDEO::vga.print("SELECT");

    // Text A
    VIDEO::vga.setTextColor( joyControl[6][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[6][0], y + joyControl[6][1]);
    VIDEO::vga.circle(x + joyControl[6][0] + 3, y + joyControl[6][1] + 3, 8,  joyControl[6][2]);
    VIDEO::vga.print("A");

    // Text B
    VIDEO::vga.setTextColor(joyControl[7][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[7][0], y + joyControl[7][1]);
    VIDEO::vga.circle(x + joyControl[7][0] + 3, y + joyControl[7][1] + 3, 8,  joyControl[7][2]);
    VIDEO::vga.print("B");

    // Text C
    VIDEO::vga.setTextColor(joyControl[8][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[8][0], y + joyControl[8][1]);
    VIDEO::vga.circle(x + joyControl[8][0] + 3, y + joyControl[8][1] + 3, 8, joyControl[8][2]);
    VIDEO::vga.print("C");

    // Text X
    VIDEO::vga.setTextColor(joyControl[9][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[9][0], y + joyControl[9][1]);
    VIDEO::vga.circle(x + joyControl[9][0] + 3, y + joyControl[9][1] + 3, 8, joyControl[9][2]);
    VIDEO::vga.print("X");

    // Text Y
    VIDEO::vga.setTextColor(joyControl[10][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[10][0], y + joyControl[10][1]);
    VIDEO::vga.circle(x + joyControl[10][0] + 3, y + joyControl[10][1] + 3, 8, joyControl[10][2]);
    VIDEO::vga.print("Y");

    // Text Z
    VIDEO::vga.setTextColor(joyControl[11][2],zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[11][0], y + joyControl[11][1]);
    VIDEO::vga.circle(x + joyControl[11][0] + 3, y + joyControl[11][1] + 3, 8, joyControl[11][2]);
    VIDEO::vga.print("Z");

    // Text L2
    VIDEO::vga.setTextColor(joyControl[12][2], zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[12][0] - 3, y + joyControl[12][1]);
    VIDEO::vga.circle(x + joyControl[12][0] + 3, y + joyControl[12][1] + 3, 8, joyControl[12][2]);
    VIDEO::vga.print("L2");

    // Text R2
    VIDEO::vga.setTextColor(joyControl[13][2], zxColor(7, 1));
    VIDEO::vga.setCursor(x + joyControl[13][0] - 3, y + joyControl[13][1]);
    VIDEO::vga.circle(x + joyControl[13][0] + 3, y + joyControl[13][1] + 3, 8, joyControl[13][2]);
    VIDEO::vga.print("R2");

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
    "HK_USB_BOOT", "HK_GIGASCREEN", "HK_LED_TOGGLE",
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
    "LED indicators",       // HK_LED_TOGGLE
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
    "Indicadores LED",       // HK_LED_TOGGLE
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

    int joyDropdown[16][7]={
        {7, 65,-1, 1, 2, 3, 0},   // 0=Left   (right→Right, up→Up, down→Down)
        {67,65, 0, 9, 2, 3, 0},   // 1=Right  (left→Left, right→X, up→Up, down→Down)
        {37,17,-1, 6,-1, 0, 0},   // 2=Up     (right→A, down→Left)
        {37,89,-1, 8, 0, 4, 0},   // 3=Down   (right→C, up→Left, down→Start)
        {37,121,-1, 5, 3,-1, 0},  // 4=Start  (right→Mode, up→Down)
        {121,121,4,14, 1,-1, 0},  // 5=Mode   (left→Start, right→Ok, up→Right)
        {157,17, 2, 7,-1, 9, 0},  // 6=A      (left→Up, right→B, down→X)
        {237,17, 6,-1,-1,10, 0},  // 7=B      (left→A, down→Y)
        {157,65, 3,11, 9,12, 0},  // 8=C      (left→Down, right→Z, up→X, down→L2)
        {157,41, 1,10, 6, 8, 0},  // 9=X      (left→Right, right→Y, up→A, down→C)
        {237,41, 9,-1, 7,11, 0},  // 10=Y     (left→X, up→B, down→Z)
        {237,65, 8,-1,10,13, 0},  // 11=Z     (left→C, up→Y, down→R2)
        {157,89, 3,13, 8,14, 0},  // 12=L2    (left→Down, right→R2, up→C, down→Ok)
        {237,89,12,-1,11,15, 0},  // 13=R2    (left→L2, up→Z, down→Test)
        {181,121, 5,15,12,-1, 0}, // 14=Ok    (left→Mode, right→Test, up→L2)
        {241,121,14,-1,13,-1, 0}  // 15=Test  (left→Ok, up→R2)
    };

    string keymenu = MENU_JOYSELKEY[Config::lang];
    int joytype = Config::joystick;

    string selkeymenu[5] = {
        "",             // None (no submenu)
        MENU_JOY_AZ,
        MENU_JOY_09,
        "",
        "Joystick"
    };

    selkeymenu[3] = (Config::lang ? "Especial\n" : "Special\n");
    selkeymenu[3] += MENU_JOY_SPECIAL;
    selkeymenu[4] = MENU_JOY_KEMPSTON;
    int curDropDown = 2;
    uint8_t joyDialogMode = 0; // 0 -> Define, 1 -> Test

    const unsigned short h = (OSD_FONT_H * 17) + 2;
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
    for (int n = 0; n < 14; ++n)
        joyDropdown[n][6] = Config::joydef[n];

    // Draw Joy DropDowns
    for (int n = 0; n < 14; ++n) {
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
    VIDEO::vga.setCursor(x + joyDropdown[14][0], y + joyDropdown[14][1]);
    VIDEO::vga.print("   Ok    ");
    VIDEO::vga.setCursor(x + joyDropdown[15][0], y + joyDropdown[15][1]);
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
            if (is_left(Nextkey.vk) || Nextkey.vk == fabgl::VK_F9) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][2] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][2];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_right(Nextkey.vk) || Nextkey.vk == fabgl::VK_F10) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][3] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][3];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_up(Nextkey.vk)) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][4] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][4];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_down(Nextkey.vk)) {
                if (joyDialogMode == 0 && joyDropdown[curDropDown][5] >= 0) {
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    curDropDown = joyDropdown[curDropDown][5];
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    if (curDropDown < 14)
                        VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                    else
                        VIDEO::vga.print(curDropDown == 14 ? "   Ok    " : " JoyTest ");
                    click();
                }
            } else
            if (is_enter(Nextkey.vk)) {
                if (joyDialogMode == 0) {
                    if (curDropDown>=0 && curDropDown<14) {
                        click();
                        // Launch assign menu
                        menu_curopt = 1;
                        while (1) {
                            menu_level = 0;
                            menu_saverect = true;
                            uint8_t opt = simpleMenuRun(keymenu,x + joyDropdown[curDropDown][0],y + joyDropdown[curDropDown][1],6,11);
                            if(opt!=0) {
                                if (opt == 1) {// None: assign directly, no submenu
                                    joyDropdown[curDropDown][6] = fabgl::VK_NONE;
                                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                                    VIDEO::vga.print(vkToText(joyDropdown[curDropDown][6]).c_str());
                                    break;
                                }
                                // Key select menu
                                menu_saverect = true;
                                menu_level = 0;
                                menu_curopt = 1;
                                uint8_t opt2 = simpleMenuRun(selkeymenu[opt - 1],x + joyDropdown[curDropDown][0],y + joyDropdown[curDropDown][1],6,11);
                                if(opt2!=0) {
                                    if (opt == 2) {// A-Z
                                        joyDropdown[curDropDown][6] = (fabgl::VirtualKey) 47 + opt2;
                                    } else
                                    if (opt == 3) {// 0-9
                                        joyDropdown[curDropDown][6] = (fabgl::VirtualKey) 1 + opt2;
                                    } else
                                    if (opt == 4) {// Special
                                        if (opt2 < 13) {// F1-F12
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
                                        } else
                                        if (opt2 == 19) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_RETURN;
                                        } else
                                        if (opt2 == 20) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_LSHIFT;
                                        } else
                                        if (opt2 == 21) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_LCTRL;
                                        } else
                                        if (opt2 == 22) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_SPACE;
                                        } else
                                        if (opt2 == 23) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_BACKSPACE;
                                        } else
                                        if (opt2 == 24) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_KP_0;
                                        } else
                                        if (opt2 == 25) {
                                            joyDropdown[curDropDown][6] = fabgl::VK_KP_PERIOD;
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
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_JOY_C;
                                        } else
                                        if (opt2 == 8) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_JOY_X;
                                        } else
                                        if (opt2 == 9) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_JOY_Y;
                                        } else
                                        if (opt2 == 10) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_JOY_Z;
                                        } else
                                        if (opt2 == 11) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_START;
                                        } else
                                        if (opt2 == 12) {
                                            joyDropdown[curDropDown][6] = fabgl::VirtualKey::VK_DPAD_SELECT;
                                        }
                                        if (joytype == JOY_FULLER && opt2 <= 6)
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
                    if (curDropDown == 14) {
                        // Ok button

                        // Check if there are changes and ask to save them
                        bool changed = false;
                        for (int n = 0; n < 14; ++n) {
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
                                for (int n = 0; n < 14; ++n) {
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
                    if (curDropDown == 15) {
                        // Enable joyTest
                        joyDialogMode = 1;
                        for (int n = 0; n < 14; ++n) {
                            Config::joydef[n] = joyDropdown[n][6];
                        }
                        Config::save(); /// TODO: revert support
                        joyTestExitCount = 0;
                        VIDEO::vga.setTextColor(zxColor(4, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + joyDropdown[15][0], y + joyDropdown[15][1]);
                        VIDEO::vga.print(" JoyTest ");
                        click();
                    }
                }
            } else
            if (Nextkey.vk == fabgl::VK_DELETE) {
                if (joyDialogMode == 0 && curDropDown < 14) {
                    click();
                    joyDropdown[curDropDown][6] = fabgl::VK_NONE;
                    VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                    VIDEO::vga.setCursor(x + joyDropdown[curDropDown][0], y + joyDropdown[curDropDown][1]);
                    VIDEO::vga.print(vkToText(fabgl::VK_NONE).c_str());
                }
            } else
            if (Nextkey.vk == fabgl::VK_ESCAPE) {
                if (joyDialogMode) {
                    if (Nextkey.vk == fabgl::VK_ESCAPE) {
                        // Disable joyTest
                        joyDialogMode = 0;
                        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        VIDEO::vga.setCursor(x + joyDropdown[15][0], y + joyDropdown[15][1]);
                        VIDEO::vga.print(" JoyTest ");
                        for (int n = 0; n < 14; n++)
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
            for (int n = fabgl::VK_JOY_RIGHT; n <= fabgl::VK_JOY_R2; n++) {
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
                    case fabgl::VK_JOY_L2: m = 12; break;
                    case fabgl::VK_JOY_R2: m = 13; break;
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
    snprintf(tmp2, 8, "%02X", MemESP::romPeek(page, MemESP::ramCurrent[page], address & 0x3fff));

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
                        MemESP::ensureResident(page); // accessor bank → real frame
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

