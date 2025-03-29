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
#include <hardware/watchdog.h>
#include <hardware/sync.h>
#include <pico/bootrom.h>

#include "OSDMain.h"
#include "FileUtils.h"
#include "CPU.h"
#include "Video.h"
#include "ESPectrum.h"
#include "messages.h"
#include "Config.h"
#include "Snapshot.h"
#include "MemESP.h"
#include "Tape.h"
#include "pwm_audio.h"
#include "Z80_JLS/z80.h"
#include "roms.h"
#include "ff.h"
#include "psram_spi.h"
#include "Ports.h"

/**
#ifndef ESP32_SDL2_WRAPPER
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_efuse.h"
#include "soc/efuse_reg.h"

#include "fabgl.h"

#include "soc/rtc_wdt.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
*/
#include <string>
#include <cstdio>

void fputs(const char* b, FIL& f);

using namespace std;

#define MENU_REDRAW true
#define MENU_UPDATE false
#define OSD_ERROR true
#define OSD_NORMAL false

#define OSD_W 248
#define OSD_H 184
#define OSD_MARGIN 4

extern Font Font6x8;

uint8_t OSD::cols;                     // Maximum columns
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

unsigned short OSD::scrW = 320;
unsigned short OSD::scrH = 240;

char OSD::stats_lin1[25]; // "CPU: 00000 / IDL: 00000 ";
char OSD::stats_lin2[25]; // "FPS:000.00 / FND:000.00 ";

// // X origin to center an element with pixel_width
unsigned short OSD::scrAlignCenterX(unsigned short pixel_width) { return (scrW / 2) - (pixel_width / 2); }

// // Y origin to center an element with pixel_height
unsigned short OSD::scrAlignCenterY(unsigned short pixel_height) { return (scrH / 2) - (pixel_height / 2); }

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
    if (CPU::paused) osdCenteredMsg(OSD_PAUSE[Config::lang], LEVEL_INFO, 500);
}

void OSD::esp_hard_reset() {
    watchdog_enable(1, true);
    while (true);
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
    osdAt(21, 0);
    if (bottom_info) {
        string bottom_line;
#ifdef VGA_DRV
        bottom_line = " Video mode: VGA 60 Hz      ";
#else
#ifdef HDMI
        bottom_line = " Video mode: HDMI 75 Hz     ";
#endif
#ifdef TV
        bottom_line = " Video mode: TV RGBI PAL    ";
#endif
#ifdef TVSOFT
        bottom_line = " Video mode: TV-composite   ";
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
    } else {
        x = 168;
        y = 220;
    }

    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor( ESPectrum::multiplicator + 1, 0));
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(x,y);
    VIDEO::vga.print(stats_lin1);
    VIDEO::vga.setCursor(x,y+8);
    VIDEO::vga.print(stats_lin2);

}

static bool persistSave(uint8_t slotnumber)
{
    FILINFO stat_buf;
    char persistfname[sizeof(DISK_PSNA_FILE) + 7];
    char persistfinfo[sizeof(DISK_PSNA_FILE) + 7];    
    sprintf(persistfname, DISK_PSNA_FILE "%u.sna", slotnumber);
    sprintf(persistfinfo, DISK_PSNA_FILE "%u.esp", slotnumber);
    string finfo = FileUtils::MountPoint + DISK_PSNA_DIR + "/" + persistfinfo;

    // Slot isn't void
    if (f_stat(finfo.c_str(), &stat_buf) == FR_OK) {
        string title = OSD_PSNA_SAVE[Config::lang];
        string msg = OSD_PSNA_EXISTS[Config::lang];
        uint8_t res = OSD::msgDialog(title, msg);
        if (res != DLG_YES) return false;
    }

    OSD::osdCenteredMsg(OSD_PSNA_SAVING, LEVEL_INFO, 500);

    // Save info file
    FIL* f = fopen2(finfo.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) {
        OSD::osdCenteredMsg(finfo + " - unable to open", LEVEL_ERROR, 5000);
        return false;
    }
    fputs((Config::arch + "\n" + Config::romSet + "\n").c_str(), *f);    // Put architecture and romset on info file
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

// OSD Main Loop
void OSD::do_OSD(fabgl::VirtualKey KeytoESP, bool ALT) {

    static uint8_t last_sna_row = 0;
    fabgl::VirtualKeyItem Nextkey;

    if (ALT) {
        if (KeytoESP == fabgl::VK_F1) { // Show mem info
            OSD::HWInfo();
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        } else
        if (KeytoESP == fabgl::VK_F2) { // Turbo mode
            ESPectrum::multiplicator += 1;
            if (ESPectrum::multiplicator > 3) {
                ESPectrum::multiplicator = 0;
            }
            CPU::updateStatesInFrame();
        } else 
        if (KeytoESP == fabgl::VK_F3) {
            portReadBPDialog();
        } else 
        if (KeytoESP == fabgl::VK_F4) {
            portWriteBPDialog();
        }
        else if (KeytoESP == fabgl::VK_F5) {
            osdDebug();
        }
        else if (KeytoESP == fabgl::VK_F7) {
            BPDialog();
        } else 
        if (KeytoESP == fabgl::VK_F8) {
            jumpToDialog();
        } else 
        if (KeytoESP == fabgl::VK_F9) { // Input Poke
            pokeDialog();
        } else 
        if (KeytoESP == fabgl::VK_F10) { // NMI
            Z80::triggerNMI();
        }
        else if (KeytoESP == fabgl::VK_F12) {
            /// TODO: close all files
            //close_all()
            reset_usb_boot(0, 0);
            while(1);
        }
    } else {
        if (KeytoESP == fabgl::VK_PAUSE) {
            CPU::paused = !CPU::paused;
            click();
        }
        else if (FileUtils::fsMount && KeytoESP == fabgl::VK_F2) {
            menu_level = 0;
            menu_saverect = false;
            string mFile = fileDialog(FileUtils::SNA_Path, MENU_SNA_TITLE[Config::lang], DISK_SNAFILE, 51, 22);
            if (mFile != "") {
                Config::save();
                mFile.erase(0, 1);
                string fname = FileUtils::SNA_Path + mFile;
                if(!LoadSnapshot(fname, "", "")) {
                    OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
                }
                Config::ram_file = fname;
                Config::last_ram_file = fname;
            }
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        }
        else if (FileUtils::fsMount && KeytoESP == fabgl::VK_F3) {
            menu_level = 0;
            menu_curopt = 1;
            // Persist Load
            string menuload = MENU_PERSIST_LOAD[Config::lang];
            for(int i = 1; i <= 40; ++i) {
                menuload += (Config::lang ? "Ranura " : "Slot ") + to_string(i) + "\n";
            }
            uint8_t opt2 = menuRun(menuload);
            if (opt2) {
                persistLoad(opt2);
            }
        }
        else if (FileUtils::fsMount && KeytoESP == fabgl::VK_F4) {
            // Persist Save
            menu_level = 0;
            menu_curopt = 1;
            while (1) {
                string menusave = MENU_PERSIST_SAVE[Config::lang];
                for(int i = 1; i <= 40; ++i) {
                    menusave += (Config::lang ? "Ranura " : "Slot ") + to_string(i) + "\n";
                }
                uint8_t opt2 = menuRun(menusave);
                if (opt2) {
                    if (persistSave(opt2)) return;
                    menu_curopt = opt2;
                } else break;
            }
        }
        else if (FileUtils::fsMount && KeytoESP == fabgl::VK_F5) {
            menu_level = 0; 
            menu_saverect = false;  
            string mFile = fileDialog(FileUtils::TAP_Path, MENU_TAP_TITLE[Config::lang],DISK_TAPFILE,51,22);
            if (mFile != "") {
                Config::save();
                Tape::LoadTape(mFile);
            }
            if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes
        }
        else if (KeytoESP == fabgl::VK_F6) {
            // Start / Stop .tap reproduction
            if (Tape::tapeStatus == TAPE_STOPPED)
                Tape::Play();
            else
                Tape::Stop();
            click();
        }
        else if (KeytoESP == fabgl::VK_F7) {
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
        }
        else if (KeytoESP == fabgl::VK_F8) {
            // Show / hide OnScreen Stats
            if ((VIDEO::OSD & 0x03) == 0)
                VIDEO::OSD |= Tape::tapeStatus == TAPE_LOADING ? 1 : 2;
            else
                VIDEO::OSD++;

            if ((VIDEO::OSD & 0x03) > 2) {
                if ((VIDEO::OSD & 0x04) == 0) {
                    if (Config::aspect_16_9) 
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen;
                    else
                        VIDEO::Draw_OSD43 = Z80Ops::isPentagon ? VIDEO::BottomBorder_Pentagon :  VIDEO::BottomBorder;
                }
                VIDEO::OSD &= 0xfc;
            } else {
                if ((VIDEO::OSD & 0x04) == 0) {
                    if (Config::aspect_16_9) 
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                    else
                        VIDEO::Draw_OSD43  = Z80Ops::isPentagon ? VIDEO::BottomBorder_OSD_Pentagon : VIDEO::BottomBorder_OSD;

                    OSD::drawStats();
                }
                ESPectrum::TapeNameScroller = 0;
            }
            click();
        }
        else if (KeytoESP == fabgl::VK_F9 || KeytoESP == fabgl::VK_VOLUMEDOWN) { 
            if (VIDEO::OSD == 0) {
                if (Config::aspect_16_9) 
                    VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                else
                    VIDEO::Draw_OSD43  = Z80Ops::isPentagon ? VIDEO::BottomBorder_OSD_Pentagon : VIDEO::BottomBorder_OSD;
                VIDEO::OSD = 0x04;
            } else
                VIDEO::OSD |= 0x04;
            ESPectrum::totalseconds = 0;
            ESPectrum::totalsecondsnodelay = 0;
            VIDEO::framecnt = 0;
            if (ESPectrum::aud_volume>ESP_VOLUME_MIN) {
                ESPectrum::aud_volume--;
                pwm_audio_set_volume(ESPectrum::aud_volume);
            }
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
        }
        else if (KeytoESP == fabgl::VK_F10 || KeytoESP == fabgl::VK_VOLUMEUP) { 
            if (VIDEO::OSD == 0) {
                if (Config::aspect_16_9) 
                    VIDEO::Draw_OSD169 = VIDEO::MainScreen_OSD;
                else
                    VIDEO::Draw_OSD43  = Z80Ops::isPentagon ? VIDEO::BottomBorder_OSD_Pentagon : VIDEO::BottomBorder_OSD;
                VIDEO::OSD = 0x04;
            } else
                VIDEO::OSD |= 0x04;
            ESPectrum::totalseconds = 0;
            ESPectrum::totalsecondsnodelay = 0;
            VIDEO::framecnt = 0;
            if (ESPectrum::aud_volume<ESP_VOLUME_MAX) {
                ESPectrum::aud_volume++;
                pwm_audio_set_volume(ESPectrum::aud_volume);
            }
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
        }
        else if (KeytoESP == fabgl::VK_F11) { // Hard reset
            if (Config::ram_file != NO_RAM_FILE) {
                Config::ram_file = NO_RAM_FILE;
            }
            Config::last_ram_file = NO_RAM_FILE;
            ESPectrum::reset();
        }
        else if (KeytoESP == fabgl::VK_F12) { // ESP32 reset
            // ESP host reset
            Config::ram_file = NO_RAM_FILE;
            Config::save();
            esp_hard_reset();
        }
        else if (KeytoESP == fabgl::VK_F1) {
        menu_curopt = 1;
        while(1) {
            // Main menu
            menu_saverect = false;
            menu_level = 0;
            uint8_t opt = menuRun("ESPectrum " + Config::arch + "\n" +
                (!FileUtils::fsMount ? MENU_MAIN_NO_SD[Config::lang] : MENU_MAIN[Config::lang])
            );
            if (FileUtils::fsMount && opt == 1) {
                // ***********************************************************************************
                // SNAPSHOTS MENU
                // ***********************************************************************************
                menu_curopt = 1;
                menu_saverect = true;
                while(1) {
                    menu_level = 1;
                    // Snapshot menu
                    uint8_t sna_mnu = menuRun(MENU_SNA[Config::lang]);
                    if (sna_mnu > 0) {
                        menu_level = 2;
                        menu_saverect = true;
                        if (sna_mnu == 1) {
                            string mFile = fileDialog(FileUtils::SNA_Path, MENU_SNA_TITLE[Config::lang], DISK_SNAFILE, 28, 16);
                            if (mFile != "") {
                                Config::save();
                                mFile.erase(0, 1);
                                string fname = FileUtils::SNA_Path + mFile;
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
                                string menuload = MENU_PERSIST_LOAD[Config::lang];
                                for(int i=1; i <= 10; i++) {
                                    menuload += (Config::lang ? "Ranura " : "Slot ") + to_string(i) + "\n";
                                }
                                uint8_t opt2 = menuRun(menuload);
                                if (opt2) {
                                    if (persistLoad(opt2)) return;
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
                                string menusave = MENU_PERSIST_SAVE[Config::lang];
                                for(int i=1; i <= 10; i++) {
                                    menusave += (Config::lang ? "Ranura " : "Slot ") + to_string(i) + "\n";
                                }
                                uint8_t opt2 = menuRun(menusave);
                                if (opt2) {
                                    if (persistSave(opt2)) return;
                                    menu_saverect = false;
                                    menu_curopt = opt2;
                                } else break;
                            }
                        }
                        menu_curopt = sna_mnu;
                    } else {
                        menu_curopt = 1;
                        break;
                    }
                }
            } 
            else if (FileUtils::fsMount && opt == 2 || opt == 1) {
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
                    uint8_t opt2 = menuRun(mos ? MENU_RESET_MOS[Config::lang] : MENU_RESET[Config::lang]);
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
                        Config::ram_file = NO_RAM_FILE;
                        Config::save();
                        esp_hard_reset();
                    } else if (mos && opt2 == 4) {
                        f_unlink(MOS_FILE);
                        esp_hard_reset();
                    } else if ((mos && opt2 == 5) || (!mos && opt2 == 4)) {
                        f_unlink(MOUNT_POINT_SD STORAGE_NVS);
                        esp_hard_reset();
                    } else {
                        menu_curopt = FileUtils::fsMount ? 2 : 1;
                        break;
                    }
                }
            }
            else if (FileUtils::fsMount && opt == 3 || opt == 2) {
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
                            string stor_menu = MENU_STORAGE[Config::lang];
                            uint8_t opt2 = menuRun(stor_menu);
                            if (opt2) {
                                if (opt2 == 1) {
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
                                            menu_curopt = 1;
                                            menu_level = 2;                                       
                                            break;
                                        }
                                    }
                                }
                                else if (opt2 == 2) {
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
                                            menu_curopt = 1;
                                            menu_level = 2;                                       
                                            break;
                                        }
                                    }
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
                            } else {
                                menu_curopt = 3;
                                break;
                            }
                        }
                    }
                    else if (options_num == 4) {
                        menu_level = 2;
                        menu_curopt = 1;                    
                        menu_saverect = true;
                        while (1) {
                            // Video
                            uint8_t options_num = menuRun(MENU_VIDEO[Config::lang]);
                            if (options_num > 0) {
                                if (options_num == 1) {
                                    menu_level = 3;
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
                                                Config::ram_file = "none";
                                                Config::save();
                                                Config::save();
                                                // Reset to apply if mode != CRT
                                                esp_hard_reset();
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
                            } else {
                                menu_curopt = 4;
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
                                            menu_curopt = 6;
                                            menu_level = 2;                                       
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 7) {
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
                                            menu_curopt = 7;
                                            menu_level = 2;                                       
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 8) {
                                    menu_level = 3;
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
                                            menu_curopt = 8;
                                            menu_level = 2;                                       
                                            break;
                                        }
                                    }
                                }
                                else if (options_num == 9) {
                                    menu_level = 3;
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
                                            menu_curopt = 9;
                                            menu_level = 2;                                       
                                            break;
                                        }
                                    }
                                }
                            } else {
                                menu_curopt = 5;
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
                    else if (options_num == 7) {
                        menu_level = 2;
                        menu_curopt = 1;
                        menu_saverect = true;
                        while (1) {
                            // Update
                            string Mnustr = FileUtils::fsMount ? MENU_UPDATE_FW[Config::lang] : MENU_UPDATE_FW_NO_SD[Config::lang];
                            uint8_t opt2 = menuRun(Mnustr);
                            if (opt2) {
                                // Update
                                if (opt2 == 1) {
                                    /// TODO: close all files
                                    //close_all()
                                    reset_usb_boot(0, 0);
                                    while(1);
                                } else {
                                    menu_saverect = true;
                                    string mFile = fileDialog(FileUtils::ROM_Path, MENU_ROM_TITLE[Config::lang], DISK_ROMFILE, 26, 15);
                                    if (mFile != "") {
                                        mFile.erase(0, 1);
                                        string fname = FileUtils::ROM_Path + mFile;
                                        bool res = updateROM(fname, opt2 - 1);
                                        if (res) return;
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
                        menu_curopt = FileUtils::fsMount ? 3 : 2;
                        break;
                    }
                }
            }
            else if (FileUtils::fsMount && opt == 4 || opt == 3) {
                // DEBUG MENU
                menu_saverect = true;
                menu_curopt = 1;            
                while(1) {
                    menu_level = 1;
                    // Debug
                    uint8_t opt2 = menuRun(MENU_DEBUG_EN);
                    if (opt2 == 1) {
                        portReadBPDialog();
                        return;
                    }
                    else if (opt2 == 2) {
                        portWriteBPDialog();
                        return;
                    }
                    else if (opt2 == 3) {
                        OSD::osdDebug();
                        return;
                    } else if (opt2 == 4) {
                        BPDialog();
                        return;
                    } else if (opt2 == 5) {
                        jumpToDialog();
                        return;
                    } else if (opt2 == 6) {
                        pokeDialog();
                        return;
                    } else if (opt2 == 7) {
                        Z80::triggerNMI();
                        return;
                    } else {
                        menu_curopt = FileUtils::fsMount ? 4 : 3;
                        break;
                    }
                }
            }
            else if (FileUtils::fsMount && opt == 5 || opt == 4) {
                // Help
                drawOSD(true);
                osdAt(2, 0);
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                VIDEO::vga.print(Config::lang ? OSD_HELP_ES : OSD_HELP_EN);
                while (1) {
                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        if (ESPectrum::readKbd(&Nextkey)) {
                            if(!Nextkey.down) continue;
                            if (is_enter(Nextkey.vk) || is_back(Nextkey.vk)) break;
                        }
                    }
                    sleep_ms(5);
                }
                click();
                if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes                
                return;
            }        
            else if (FileUtils::fsMount && opt == 6 || opt == 5) {
                // About
                drawOSD(false);
                
                VIDEO::vga.fillRect(Config::aspect_16_9 ? 60 : 40,Config::aspect_16_9 ? 12 : 32,240,50,zxColor(0, 0));            

                // Decode Logo in EBF8 format
                uint8_t *logo = (uint8_t *)ESPectrum_logo;
                int pos_x = Config::aspect_16_9 ? 86 : 66;
                int pos_y = Config::aspect_16_9 ? 23 : 43;
                int logo_w = (logo[5] << 8) + logo[4]; // Get Width
                int logo_h = (logo[7] << 8) + logo[6]; // Get Height
                logo+=8; // Skip header
                for (int i=0; i < logo_h; i++)
                    for(int n=0; n<logo_w; n++)
                        VIDEO::vga.dotFast(pos_x + n,pos_y + i,logo[n+(i*logo_w)]);

                // About Page 1
                // osdAt(7, 0);
                VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
                // VIDEO::vga.print(Config::lang ? OSD_ABOUT1_ES : OSD_ABOUT1_EN);
                
                pos_x = Config::aspect_16_9 ? 66 : 46;
                pos_y = Config::aspect_16_9 ? 68 : 88;            
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
                            VIDEO::vga.fillRect(Config::aspect_16_9 ? 60 : 40,Config::aspect_16_9 ? 64 : 84,240,114,zxColor(1, 0));
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
                if (VIDEO::OSD) OSD::drawStats(); // Redraw stats for 16:9 modes                
                return;            
            }
            else break;
        }        
        }
    }
}

// Shows a red panel with error text
void OSD::errorPanel(string errormsg) {
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
void OSD::errorHalt(string errormsg) {
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
void OSD::osdCenteredMsg(string msg, uint8_t warn_level) {
    osdCenteredMsg(msg,warn_level,1000);
}

void OSD::osdCenteredMsg(string msg, uint8_t warn_level, uint16_t millispause) {

    const unsigned short h = OSD_FONT_H * 3;
    const unsigned short y = scrAlignCenterY(h);
    unsigned short paper;
    unsigned short ink;
    unsigned int j;

    if (msg.length() > (scrW / 6) - 10) msg = msg.substr(0,(scrW / 6) - 10);

    const unsigned short w = (msg.length() + 2) * OSD_FONT_W;
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
    while (1) {
        sleep_ms(5);
        if (Kbd->virtualKeyAvailable()) {
            Kbd->getNextVirtualKey(&Nextkey);
            if (!Nextkey.down) continue;
            if (Nextkey.vk == fabgl::VK_ESCAPE) {
                break;
            }
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

void OSD::osdDebug() {
    const unsigned short h = OSD_FONT_H * 22;
    const unsigned short y = scrAlignCenterY(h);
    const unsigned short w = OSD_FONT_W * 40;
    const unsigned short x = scrAlignCenterX(w);

    VIDEO::SaveRect.save(x - 1, y - 1, w + 2, h + 2);
    char buf[32];
    int ii = 3;
    uint32_t t1 = 0;
    uint32_t t2 = 0;
    uint32_t T1 = 0;
    uint32_t T2 = 0;

c:
    sleep_ms(5);
    // Set font
    VIDEO::vga.setFont(Font6x8);

    // Boarder
    VIDEO::vga.rect(x, y, w, h, zxColor(0, 0));

    VIDEO::vga.fillRect(x + 1, y + 1, w - 2, OSD_FONT_H, zxColor(0,0));
    VIDEO::vga.fillRect(x + 1, y + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7,1));

    // Title
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));        
    VIDEO::vga.setCursor(x + OSD_FONT_W + 1, y + 1);
    VIDEO::vga.print(Config::lang ? "Depurar" : "Debug");

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
    uint16_t pc = Z80::getRegPC();
    int i = 0;
    int xi = x + 1;
    int nn = 0;
    int n = 0;
    int d = 0;
    bool ED = false;
    bool CB = false;
    bool IX = false;
    bool IY = false;
    bool ICB = false;
    std::string mem;
    for (; i < 20; ++i) {
        uint16_t pci = pc + i - ii;
        uint8_t bi = MemESP::readbyte(pci);
        uint8_t b1 = MemESP::readbyte(pci + 1);
        int yi = y + (i + 1) * OSD_FONT_H + 2;
        VIDEO::vga.setCursor(xi, yi);
        if (ICB) {
            int8_t b1b = b1;
            snprintf(buf, 32, b1b >= 0 ? " %04X %02X +%d(d)" : " %04X %02X %d(d)", pci, bi, b1b);
            d = 0;
            nn = 1;
            ICB = false;
        }
        else if (IX || IY) {
            snprintf(buf, 32, " %04X %02X", pci, bi);
            IX = false;
            IY = false;
        }
        else if (!CB && !ED && nn == 0 && n == 0 && d == 0 || pci == pc) {
            ED = bi == 0xED;
            CB = bi == 0xCB;
            IX = bi == 0xDD;
            IY = bi == 0xFD;
            if (IX || IY) {
                ICB = b1 == 0xCB;
                if (ICB) {
                    mem = mnemCB[MemESP::readbyte(pci + 3)];
                    mem.replace(mem.find(" ", 0), 1, " (IX+d),");
                } else {
                    mem = mnemIX(b1);
                }
                auto pos = mem.find(",(HL)", 0);
                if (pos != string::npos)
                    mem.replace(pos, 4, " ");
                if (mem.length() > 12)
                    mem = mem.substr(0, 12);
                if (IY) {
                    mem.replace(mem.find("IX",0),2,"IY");
                }
            } else if (CB) {
                mem = mnemCB[b1];
            } else if (ED) {
                mem = mnemED(b1);
            } else {
                mem = mnem[bi];
            }
            const char* memc = mem.c_str();
            if (strstr(memc, "nn") != 0 || strstr(memc, "(nn)") != 0) {
                d = 0;
                n = 0;
                nn = 2;
            } else if (strstr(memc, "n") != 0 || strstr(memc, "(n)") != 0) {
                d = 0;
                n = 1;
                nn = 0;
            } else if (strstr(memc, "d") != 0) {
                d = 1;
                n = 0;
                nn = 0;
            } else {
                d = 0;
                n = 0;
                nn = 0;
            }
            snprintf(buf, 32, "%c%04X %02X %s", pci == pc ? '*' : ' ', pci, bi, memc);
        } else if (d == 1) {
            int8_t bib = bi;
            snprintf(buf, 32, bib >= 0 ? " %04X %02X +%d(d)" : " %04X %02X %d(d)", pci, bi, bib);
            --d;
        } else if (n == 1) {
            snprintf(buf, 32, " %04X    %02X", pci, bi);
            --n;
        } else if (nn == 2 && !ED) {
            snprintf(buf, 32, " %04X %02X %02X%02X", pci, bi, b1, bi);
            --nn;
        } else {
            snprintf(buf, 32, " %04X %02X", pci, bi);
            if (ED) ED = false;
            else if (CB) CB = false;
            else --nn;
        }
        VIDEO::vga.print(buf);
        if (Config::enableBreakPoint && Config::breakPoint == pci) {
            VIDEO::vga.circle(xi+3, yi+3, 3, zxColor(2, 0));
        }
    }
    i = 0;
    xi = x + 22 * OSD_FONT_W;
    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    if (MemESP::newSRAM)
        snprintf(buf, 32, "PAGE0 -> SRAM#%d", MemESP::sramLatch);
    else if (MemESP::ramCurrent[0] < (uint8_t*)0x20000000)
        snprintf(buf, 32, "PAGE0 -> ROM#%d", MemESP::romInUse);
    else
        snprintf(buf, 32, "PAGE0 -> RAM#0");
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "PAGE3 -> RAM#%d", MemESP::bankLatch);
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "VIDEO -> RAM#%d", MemESP::videoLatch ? 7 : 5);
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "PAGING LOCK %s", MemESP::pagingLock ? "true" : "false");
    VIDEO::vga.print(buf);

    ++i;

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "AF %04X AFx%04X", Z80::getRegAF(), Z80::getRegAFx());
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "BC %04X BCx%04X", Z80::getRegBC(), Z80::getRegBCx());
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "HL %04X HLx%04X", Z80::getRegHL(), Z80::getRegHLx());
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "DE %04X DEx%04X", Z80::getRegDE(), Z80::getRegDEx());
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "IX %04X IY %04X", Z80::getRegIX(), Z80::getRegIY());
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "IR %02X%02X IM %d", Z80::getRegI(), Z80::getRegR(), Z80::getIntMode());
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "SP %04X %dT", Z80::getRegSP(), T2 - T1);
    VIDEO::vga.print(buf);

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    snprintf(buf, 32, "PC %04X %dus", Z80::getRegPC(), t2 - t1);
    VIDEO::vga.print(buf);

    if (!Config::enableBreakPoint)
        ++i;
    else {
        VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
        snprintf(buf, 32, "BP %04X", Config::breakPoint);
        VIDEO::vga.print(buf);
    }
    if (Config::enablePortReadBP && Config::enablePortWriteBP) {
        VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
        snprintf(buf, 32, "BPP R%04X W%04X", Config::portReadBP, Config::portWriteBP);
        VIDEO::vga.print(buf);
    } else if (Config::enablePortWriteBP) {
        VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
        snprintf(buf, 32, "BPP       W%04X", Config::portWriteBP);
        VIDEO::vga.print(buf);
    } else if (Config::enablePortReadBP) {
        VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
        snprintf(buf, 32, "BPP R%04X      ", Config::portReadBP);
        VIDEO::vga.print(buf);
    } else {
        ++i;
    }

    ++i;

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("SZxHxPAC SZxHxPAC");

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    uint8_t f = Z80::getRegAF() & 0xFF;
    uint8_t fx = Z80::getRegAFx() & 0xFF;
    snprintf(buf, 32, "%c%c%c%c%c%c%c%c %c%c%c%c%c%c%c%c",
       BNc(f , 7), BNc(f , 6), BNc(f , 5), BNc(f , 4), BNc(f , 3), BNc(f , 2), BNc(f , 1), BNc(f , 0),
       BNc(fx, 7), BNc(fx, 6), BNc(fx, 5), BNc(fx, 4), BNc(fx, 3), BNc(fx, 2), BNc(fx, 1), BNc(fx, 0)
    );
    VIDEO::vga.print(buf);

    ++i;

    VIDEO::vga.setCursor(xi, y + (i++ + 1) * OSD_FONT_H + 2);
    VIDEO::vga.print("F1 - Debug help");

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
                break;
            } else
            if (Nextkey.vk == fabgl::VK_F3) {
                portReadBPDialog();
                goto c;
            } else 
            if (Nextkey.vk == fabgl::VK_F4) {
                portWriteBPDialog();
                goto c;
            } else 
            if (Nextkey.vk == fabgl::VK_F7) {
                BPDialog();
                goto c;
            } else 
            if (Nextkey.vk == fabgl::VK_F8) {
                jumpToDialog();
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F9) {
                pokeDialog();
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F10) {
                Z80::triggerNMI();
                goto c;
            } else
            if (FileUtils::fsMount && Nextkey.vk == fabgl::VK_F11) {
                // Persist Load
                string menuload = MENU_PERSIST_LOAD[Config::lang];
                for(int i = 1; i <= 40; ++i) {
                    menuload += (Config::lang ? "Ranura " : "Slot ") + to_string(i) + "\n";
                }
                uint8_t opt2 = menuRun(menuload);
                if (opt2) {
                    persistLoad(opt2);
                }
                goto c;
            }
            else if (FileUtils::fsMount && Nextkey.vk == fabgl::VK_F12) {
                // Persist Save
                string menusave = MENU_PERSIST_SAVE[Config::lang];
                for(int i = 1; i <= 40; ++i) {
                    menusave += (Config::lang ? "Ranura " : "Slot ") + to_string(i) + "\n";
                }
                uint8_t opt2 = menuRun(menusave);
                if (opt2) {
                    persistSave(opt2);
                }
                goto c;
            }
            if (Nextkey.vk == fabgl::VK_KP_PLUS || Nextkey.vk == fabgl::VK_UP) {
                ++ii;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_KP_MINUS || Nextkey.vk == fabgl::VK_DOWN) {
                --ii;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_0) {
                ii = 0;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_PAGEUP) {
                ii += 20;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_PAGEDOWN) {
                ii -= 20;
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F5) {
                if (Config::enableBreakPoint && Config::breakPoint == pc) {
                    Config::enableBreakPoint = false;
                } else {
                    Config::enableBreakPoint = true;
                    Config::breakPoint = pc;
                }
                Config::save();
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_F2) {
                osdDump();
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
                goto c;
            } else
            if (Nextkey.vk == fabgl::VK_SPACE) {
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
                    Config::enableBreakPoint = true;
                    Config::breakPoint = pcs + 3; // CALL nn case
                    break;
                }
                ii -= (int)pc - Z80::getRegPC();
                if (ii > 16) ii = 4;
                if (ii < 0) ii = 4;
                t2 = time_us_32();
                T2 = CPU::tstates;
                goto c;
            }
        }
    }
    VIDEO::SaveRect.restore_last();

}

// // Count NL chars inside a string, useful to count menu rows
unsigned short OSD::rowCount(string menu) {
    unsigned short count = 0;
    for (unsigned short i = 0; i < menu.length(); i++) {
        if (menu.at(i) == ASCII_NL) {
            count++;
        }
    }
    return count;
}

// // Get a row text
string OSD::rowGet(string menu, unsigned short row) {
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
    return "<Unknown menu row>";
}

#include <hardware/flash.h>
#include <pico/multicore.h>

static void get_cpu_flash_jedec_id(uint8_t _rx[4]) {
    static uint8_t rx[4] = {0};
    if (rx[0] == 0) {
        uint8_t tx[4] = {0x9f};
        multicore_lockout_start_blocking();
        const uint32_t ints = save_and_disable_interrupts();
        flash_do_cmd(tx, rx, 4);
        restore_interrupts(ints);
        multicore_lockout_end_blocking();
    }
    *(unsigned*)_rx = *(unsigned*)rx;
}

inline static uint32_t get_cpu_flash_size(void) {
    uint8_t rx[4] = {0};
    get_cpu_flash_jedec_id(rx);
    return 1u << rx[3];
}

void OSD::HWInfo() {
    fabgl::VirtualKeyItem Nextkey;
    click();
    // Draw Hardware and memory info
    drawOSD(true);
    osdAt(2, 0);

    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));

    VIDEO::vga.print(" Hardware info\n");
    VIDEO::vga.print(" --------------------------------------\n");

#if !PICO_RP2040
    string textout =
        " Chip model     : RP2350 " + to_string(CPU_MHZ) + " MHz\n"
        " Chip cores     : 2\n"
        " Chip RAM       : 520 KB\n"
    ;
#else
    string textout =
        " Chip model     : RP2040 " + to_string(CPU_MHZ) + " MHz\n"
        " Chip cores     : 2\n"
        " Chip RAM       : 264 KB\n"
    ;
#endif
    VIDEO::vga.print(textout.c_str());    

    char buf[128] = { 0 };
    uint8_t rx[4];
    get_cpu_flash_jedec_id(rx);
    uint32_t flash_size = (1 << rx[3]);
    snprintf(buf, 128,
             " Flash size     : %d MB\n"
             " Flash JEDEC ID : %02X-%02X-%02X-%02X\n",
             flash_size >> 20, rx[0], rx[1], rx[2], rx[3]
    );
    VIDEO::vga.print(buf);

    uint32_t psram32 = psram_size();
    if (psram32) {
        uint8_t rx8[8];
        psram_id(rx8);
        if (psram32) {
            snprintf(buf, 128,
                     " PSRAM size     : %d MB\n"\
                     " PSRAM MF ID    : %02X\n"\
                     " PSRAM KGD      : %02X\n"\
                     " PSRAM EID      : %02X%02X-%02X%02X-%02X%02X\n",
                     psram32 >> 20, rx8[0], rx8[1], rx8[2], rx8[3], rx8[4], rx8[5], rx8[6], rx8[7]
            );
        }
        VIDEO::vga.print(buf);
    }
    // Wait for key
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&Nextkey);
            if (!Nextkey.down) continue;
            if (is_enter(Nextkey.vk) || is_back(Nextkey.vk)) {
                click();
                break;
            }
        }
        sleep_ms(5);
    }
}

static void __not_in_flash_func(flash_block)(const uint8_t* buffer, size_t flash_target_offset) {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    multicore_lockout_start_blocking();
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_program(flash_target_offset, buffer, 512);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}

static void __not_in_flash_func(cleanup_block)(size_t flash_target_offset) {
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    multicore_lockout_start_blocking();
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(flash_target_offset, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    multicore_lockout_end_blocking();
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}

/**
 * arch:
 * 1 - ROM Custom ALF
 * 2 - ROM Cartridge ALF
 */
bool OSD::updateROM(const string& fname, uint8_t arch) {
    FIL* f = fopen2(fname.c_str(), FA_READ);
    if (!f) {
        osdCenteredMsg(OSD_NOROMFILE_ERR[Config::lang], LEVEL_WARN, 2000);
        return false;
    }
    FSIZE_t bytesfirmware = f_size(f); 
    const uint8_t* rom;
    size_t max_rom_size = 0;
    string dlgTitle = OSD_ROM[Config::lang];
    if ( arch == 1 ) {
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
    else if ( arch == 2 ) {
        if( bytesfirmware > (1ul << 20) ) {
            osdCenteredMsg("Unsupported file (by size)", LEVEL_WARN, 2000);
            fclose2(f);
            return false;
        }
        rom = gb_rom_Alf_cart;
        max_rom_size = 1ul << 20;
        dlgTitle += " ALF Cartridge ";
        Config::arch = "ALF";
    }
    else {
        osdCenteredMsg("Unexpected ROM type: " + to_string(arch), LEVEL_WARN, 2000);
        fclose2(f);
        return false;
    }
    size_t flash_target_offset = (size_t)rom - XIP_BASE;
    size_t max_flash_target_offset = flash_target_offset + max_rom_size;
    for (size_t i = flash_target_offset; i < max_flash_target_offset; i += FLASH_SECTOR_SIZE) {
        cleanup_block(i);
    }

    UINT br;
    const size_t sz = 512;
    uint8_t* buffer = (uint8_t*)malloc(sz);
    for (FSIZE_t i = 0; i < bytesfirmware; i += sz) {
        if ( f_read(f, buffer, sz, &br) != FR_OK) {
            osdCenteredMsg(fname + " - unable to read", LEVEL_ERROR, 5000);
            fclose2(f);
            return false;
        }
        flash_block(buffer, flash_target_offset + (size_t)(i & 0xFFFFFFFF));
    }
    fclose2(f);
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

void OSD::progressDialog(string title, string msg, int percent, int action) {

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

        if (msg.length() > (scrW / 6) - 4) msg = msg.substr(0,(scrW / 6) - 4);
        if (title.length() > (scrW / 6) - 4) title = title.substr(0,(scrW / 6) - 4);

        w = (((msg.length() > title.length() + 6 ? msg.length(): title.length() + 6) + 2) * OSD_FONT_W) + 2;
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
        VIDEO::vga.print(title.c_str());
        
        // Msg
        VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));        
        VIDEO::vga.setCursor(scrAlignCenterX(msg.length() * OSD_FONT_W), y + 1 + (OSD_FONT_H * 2));
        VIDEO::vga.print(msg.c_str());

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

uint8_t OSD::msgDialog(string title, string msg) {

    const unsigned short h = (OSD_FONT_H * 6) + 2;
    const unsigned short y = scrAlignCenterY(h);
    uint8_t res = DLG_NO;

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

string OSD::inputBox(int x, int y, string text) {

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

#define MENU_JOY_SPECIAL "Enter\n"\
    "Caps\n"\
    "SymbShift\n"\
    "Brk/Space\n"\
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
case fabgl::VK_SPACE:
    return "Brk/Space";
case fabgl::VK_LSHIFT:
    return "  Caps   ";
case fabgl::VK_LCTRL:
    return "SymbShift";
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
            if (is_enter(Nextkey.vk)) {
                if (dlg_Objects2[curObject].Name == "Ok") {
                    string s = dlgValues[0];
                    trim(s);
                    addr = stoul(s, nullptr, 16);
                    click();
                    return addr;
                } else if (dlg_Objects2[curObject].Name == "Cancel") {
                    click();
                    return 0x00010001;
                }
            } else if (is_back(Nextkey.vk)) {
                click();
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

void OSD::portReadBPDialog() {
    uint32_t address = addressDialog(Config::portReadBP, "Port read BP");
    if (address == 0x00010000) {
        return;
    }
    if (address == 0x00010001) {
        Config::enablePortReadBP = false;
    } else {
        Config::enablePortReadBP = true;
        Config::portReadBP = address;
    }
    Config::save();
}

void OSD::portWriteBPDialog() {
    uint32_t address = addressDialog(Config::portWriteBP, "Port write BP");
    if (address == 0x00010000) {
        return;
    }
    if (address == 0x00010001) {
        Config::enablePortWriteBP = false;
    } else {
        Config::enablePortWriteBP = true;
        Config::portWriteBP = address;
    }
    Config::save();
}

void OSD::BPDialog() {
    uint32_t address = addressDialog(Config::breakPoint, Config::lang ? "Punto de interr." : "Breakpoint");
    if (address == 0x00010000) {
        return;
    }
    if (address == 0x00010001) {
        Config::enableBreakPoint = false;
    } else {
        Config::enableBreakPoint = true;
        Config::breakPoint = address;
    }
    Config::save();
}
