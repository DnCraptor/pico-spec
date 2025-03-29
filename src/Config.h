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

    static void load();
    static void save();

    static void requestMachine(string newArch, string newRomSet);

    static void setJoyMap(uint8_t joy_type);

    static string   arch;
    static string   romSet;
    static string   pref_arch;
    static string   ram_file;
    static string   last_ram_file;
    static uint8_t  esp32rev;
    static bool     slog_on;
    const static bool     aspect_16_9; /// TODO:
    static uint8_t  lang;
    static bool     AY48;
    static bool     Issue2;    
    static bool     flashload;    
    static bool     tape_player;
    static volatile bool real_player;
    static bool     tape_timing_rg;
    static bool     rightSpace;
    static uint16_t breakPoint;
    static uint16_t portReadBP;
    static uint16_t portWriteBP;
    static bool     enableBreakPoint;
    static bool     enablePortReadBP;
    static bool     enablePortWriteBP;
    static uint8_t  joystick;
    static uint16_t joydef[12];
    static uint8_t  AluTiming;
    static uint8_t  ayConfig;
    static uint8_t  turbosound;
    static uint8_t  joy2cursor;
    static uint8_t  secondJoy;
    static uint8_t  kempstonPort;
    static uint8_t  throtling;
    static bool CursorAsJoy;
    static uint8_t scanlines;
    static uint8_t render;    

    static bool TABasfire1; 

    static bool StartMsg;    

};

#endif // Config.h