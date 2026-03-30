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

#ifndef Config_h
#define Config_h

#include <stdio.h>
#include <inttypes.h>
#include <string>
#include "Debug.h"

using namespace std;

#define JOY_CURSOR 0
#define JOY_KEMPSTON 1
#define JOY_SINCLAIR1 2
#define JOY_SINCLAIR2 3
#define JOY_FULLER 4
#define JOY_CUSTOM 5
#define JOY_NONE 6

class Config
{
public:

    static void load();  // load before initialized
    static void load2(); // load after initialized
    static void save();
    static bool loaded;  // true after successful load() from file/RAM

    static void requestMachine(string newArch, string newRomSet);

    static void setJoyMap(uint8_t joy_type);

    static string   arch;
    static string   romSet;
    static string   romSet48;
    static string   romSet128;
    static string   romSetPent;
    static string   romSetP512;
    static string   romSetP1M;
    static string   pref_arch;
    static string   pref_romSet_48;
    static string   pref_romSet_128;
    static string   pref_romSetPent;
    static string   pref_romSetP512;
    static string   pref_romSetP1M;
    static string   ram_file;
    static string   last_ram_file;
    static uint8_t  esp32rev;
    static bool     slog_on;
    const static bool     aspect_16_9; /// TODO:
    static uint8_t  lang;
    static bool     AY48;
#if !PICO_RP2040
    static bool     SAA1099;
    static uint8_t  midi;  // 0=Off, 1=AY bitbang, 2=ShamaZX, 3=Soft Synth
    static uint8_t  midi_synth_preset; // 0=GM,1=Piano,2=Chiptune,3=Strings,4=Rock,5=Organ,6=MusicBox,7=Synth
    static bool     timex_video;  // Timex SCLD video modes (port 0xFF)
#endif
    static uint16_t cpu_mhz;   // 252, 378 (RP2040/RP2350), 504 (RP2350 only)
    static uint16_t max_flash_freq; // MHz, default 66
    static uint16_t max_psram_freq; // MHz, default 166
#if !PICO_RP2040
    static uint8_t  vreq_voltage;  // vreg_voltage_t enum value, default VREG_VOLTAGE_1_60
#endif
    static bool     Issue2;
    static bool     flashload;    
    static bool     tape_player;
    static volatile bool real_player;
    static bool     tape_timing_rg;
    static bool     rightSpace;
    static bool     wasd;
    enum BPType : uint8_t { BP_PC=0, BP_PORT_READ=1, BP_PORT_WRITE=2, BP_MEM_WRITE=3, BP_MEM_READ=4, BP_NONE=0xFF };
    struct BreakPoint { uint16_t addr = 0xFFFF; BPType type = BP_NONE; };
    static constexpr int MAX_BREAKPOINTS = 20;
    static BreakPoint breakPoints[MAX_BREAKPOINTS];
    static int numBreakPoints;
    // Per-type cached counts for fast-path skip
    static int numPcBP;
    static int numPortReadBP;
    static int numPortWriteBP;
    static int numMemWriteBP;
    static int numMemReadBP;
    static void recountBP() {
        numBreakPoints = numPcBP = numPortReadBP = numPortWriteBP = numMemWriteBP = numMemReadBP = 0;
        for (int i = 0; i < MAX_BREAKPOINTS; i++) {
            if (breakPoints[i].type == BP_NONE) continue;
            numBreakPoints++;
            switch (breakPoints[i].type) {
                case BP_PC: numPcBP++; break;
                case BP_PORT_READ: numPortReadBP++; break;
                case BP_PORT_WRITE: numPortWriteBP++; break;
                case BP_MEM_WRITE: numMemWriteBP++; break;
                case BP_MEM_READ: numMemReadBP++; break;
                default: break;
            }
        }
    }
    static bool hasBreakPoint(uint16_t addr, BPType type) {
        for (int i = 0; i < MAX_BREAKPOINTS; i++)
            if (breakPoints[i].addr == addr && breakPoints[i].type == type) return true;
        return false;
    }
    // Legacy: check any BP_PC at addr
    static bool hasBreakPoint(uint16_t addr) { return hasBreakPoint(addr, BP_PC); }
    static bool addBreakPoint(uint16_t addr, BPType type) {
        if (hasBreakPoint(addr, type)) return false;
        for (int i = 0; i < MAX_BREAKPOINTS; i++) {
            if (breakPoints[i].type == BP_NONE) {
                breakPoints[i] = {addr, type};
                recountBP();
                return true;
            }
        }
        return false;
    }
    static bool addBreakPoint(uint16_t addr) { return addBreakPoint(addr, BP_PC); }
    static bool removeBreakPoint(uint16_t addr, BPType type) {
        for (int i = 0; i < MAX_BREAKPOINTS; i++) {
            if (breakPoints[i].addr == addr && breakPoints[i].type == type) {
                breakPoints[i] = {0xFFFF, BP_NONE};
                recountBP();
                return true;
            }
        }
        return false;
    }
    static bool removeBreakPoint(uint16_t addr) { return removeBreakPoint(addr, BP_PC); }
    static void removeBreakPointAt(int idx) {
        if (idx >= 0 && idx < MAX_BREAKPOINTS) {
            breakPoints[idx] = {0xFFFF, BP_NONE};
            recountBP();
        }
    }
    static const char* bpTypeName(BPType t) {
        switch(t) {
            case BP_PC: return "PC";
            case BP_PORT_READ: return "PR";
            case BP_PORT_WRITE: return "PW";
            case BP_MEM_WRITE: return "MW";
            case BP_MEM_READ: return "MR";
            default: return "??";
        }
    }
    static uint8_t  joystick;
    static uint16_t joydef[12];
    static uint8_t  AluTiming;
    static uint8_t  ayConfig;
    static uint8_t  turbosound;
    static uint8_t  covox;
    static uint8_t  joy2cursor;
    static uint8_t  secondJoy;
    static uint8_t  kempstonPort;
    static uint8_t  throtling;
    static bool CursorAsJoy;
    static uint8_t scanlines;
    static uint8_t render;    

    static bool TABasfire1; 

    static bool StartMsg;    

    static bool trdosFastMode;
    static bool trdosWriteProtect;
    static bool trdosSoundLed;
    static uint8_t trdosBios; // 0=5.03, 1=5.04TM, 2=5.05D
#if !PICO_RP2040
    static uint8_t esxdos;   // 0=OFF 1=DivMMC 2=DivIDE 3=DivSD
    static string esxdos_mmc_image; // Full path to .mmc image (empty = /esxdos.mmc)
    static string esxdos_hdf_image[2]; // HDF images: [0]=master/hd0, [1]=slave/hd1
#endif
    
    static signed char aud_volume;

    // Video mode enum
    enum {
        VM_640x480_60  = 0,  // 640x480@60Hz (default)
        VM_640x480_50  = 1,  // 640x480@50Hz (arch-dependent timing)
        VM_720x480_60  = 2,  // 720x480@60Hz half border
        VM_720x576_60  = 3,  // 720x576@60Hz full border (non-standard)
        VM_720x576_50  = 4,  // 720x576@50Hz full border
    };

    static uint8_t hdmi_video_mode;
    static uint8_t vga_video_mode;

    static bool v_sync_enabled;
    static bool gigascreen_enabled;
    static uint8_t gigascreen_onoff; // 0=Off, 1=On, 2=Auto
#if !PICO_RP2040
    static bool ulaplus;
#endif
    // Palette: 0=Default, 1=Grayscale
    static uint8_t palette;
    static uint8_t audio_driver;
    static bool byte_cobmect_mode;

    static void savePendingVideoMode();
    static bool loadPendingVideoMode(uint8_t &hdmi_vm, uint8_t &vga_vm);
    static void clearPendingVideoMode();

    // Hotkey indices
    enum HotkeyId {
        HK_MAIN_MENU    =  0,
        HK_LOAD_SNA     =  1,
        HK_PERSIST_LOAD =  2,
        HK_PERSIST_SAVE =  3,
        HK_LOAD_ANY     =  4,
        HK_TAPE_PLAY    =  5,
        HK_TAPE_BROWSER =  6,
        HK_STATS        =  7,
        HK_VOL_DOWN     =  8,
        HK_VOL_UP       =  9,
        HK_HARD_RESET   = 10,
        HK_REBOOT       = 11,
        HK_MAX_SPEED    = 12,
        HK_PAUSE        = 13,
        HK_HW_INFO      = 14,
        HK_TURBO        = 15,
        HK_DEBUG        = 16,
        HK_DISK         = 17,
        HK_NMI          = 18,
        HK_RESET_TO     = 19,
        HK_USB_BOOT     = 20,
        HK_GIGASCREEN   = 21,
        HK_BP_LIST      = 22,
        HK_JUMP_TO      = 23,
        HK_POKE         = 24,
        HK_VIDMODE_60   = 25,
        HK_VIDMODE_50   = 26,
        HK_COUNT        = 27
    };

    struct HotkeyBinding {
        uint16_t vk;       // fabgl::VirtualKey cast to uint16_t; 0 = unassigned (VK_NONE)
        bool     alt;
        bool     ctrl;
        bool     readonly; // true = shown in dialog but not editable
    };
    static HotkeyBinding hotkeys[HK_COUNT];

    static void initHotkeys();  // fill hotkeys[] with compiled-in defaults
};

#endif // Config.h