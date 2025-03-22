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

#ifndef ESPectrum_h
#define ESPectrum_h
#include <string>
#include "hardpins.h"
#include "CaptureBMP.h"
///#include "fabgl.h"
#include "wd1793.h"
#include <pico/time.h>
#include "fabutils.h"

using namespace std;

#define ESP_AUDIO_OVERSAMPLES_48 4368
#define ESP_AUDIO_FREQ_48 31250 // In 48K calcs are perfect :) -> ESP_AUDIO_SAMPLES_48 * 50,0801282 frames per second = 31250 Hz
#define ESP_AUDIO_SAMPLES_48  624
#define ESP_AUDIO_SAMPLES_DIV_48  7

#define ESP_AUDIO_OVERSAMPLES_128 3732
#define ESP_AUDIO_FREQ_128 31112 // ESP_AUDIO_SAMPLES_128 * 50,020008 fps = 31112,445 Hz. 
#define ESP_AUDIO_SAMPLES_128 622
#define ESP_AUDIO_SAMPLES_DIV_128  6

#define ESP_AUDIO_OVERSAMPLES_PENTAGON 4480
#define ESP_AUDIO_FREQ_PENTAGON 31250 // ESP_AUDIO_SAMPLES_PENTAGON * 48,828125 frames per second = 31250 Hz
#define ESP_AUDIO_SAMPLES_PENTAGON  640
#define ESP_AUDIO_SAMPLES_DIV_PENTAGON  7

#define ESP_VOLUME_DEFAULT -8
#define ESP_VOLUME_MAX 0
#define ESP_VOLUME_MIN -16

#include "keyboard.h"

#define esp_timer_get_time() time_us_64()

namespace fabgl {
    class PS2Controller {
        Keyboard kbd;
    public:
        Keyboard* keyboard() { return &kbd; }
    };
};

class ESPectrum
{
public:

    static void setup();
    static void loop();
    static void reset();

    // Kbd
    static void processKeyboard();
    static void bootKeyboard();
    static bool readKbd(fabgl::VirtualKeyItem *Nextkey);
    static fabgl::PS2Controller PS2Controller;
    static fabgl::VirtualKey VK_ESPECTRUM_FIRE1;
    static fabgl::VirtualKey VK_ESPECTRUM_FIRE2;
    static fabgl::VirtualKey VK_ESPECTRUM_TAB;
    static fabgl::VirtualKey VK_ESPECTRUM_GRAVEACCENT;

    // Audio
    static void BeeperGetSample();
    static void AYGetSample();
    static uint8_t audioBuffer_L[ESP_AUDIO_SAMPLES_PENTAGON];
    static uint8_t audioBuffer_R[ESP_AUDIO_SAMPLES_PENTAGON];
    static uint32_t overSamplebuf[ESP_AUDIO_SAMPLES_PENTAGON];
    static unsigned char audioSampleDivider;
    static signed char aud_volume;
    static uint32_t audbufcnt;
    static uint32_t audbufcntover;    
    static uint32_t audbufcntAY;
    static uint32_t faudbufcnt;
    static uint32_t faudbufcntAY;
    static int lastaudioBit;
    static int faudioBit;
    static int samplesPerFrame;
    static bool AY_emu;
    static int Audio_freq;

    static uint8_t multiplicator;
    static int sync_cnt;    

    static int TapeNameScroller;

    static int64_t ts_start;
    static int64_t target;
    static double totalseconds;
    static double totalsecondsnodelay;
    static int64_t elapsed;
    static int64_t idle;
    static int ESPoffset;

    static int ESPtestvar;
    static int ESPtestvar1;
    static int ESPtestvar2;        

    static volatile bool vsync;

    static bool trdos;
    static WD1793 Betadisk;

    static int32_t mouseX;
    static int32_t mouseY;
    static bool mouseButtonL;
    static bool mouseButtonR;
};

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

inline void delay(uint32_t ms) {
    sleep_ms(ms);
}

inline void delayMicroseconds(int64_t us) {
    sleep_us(us);
}

void kbdPushData(fabgl::VirtualKey virtualKey, bool down);
void joyPushData(fabgl::VirtualKey virtualKey, bool down);

#endif