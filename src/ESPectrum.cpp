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
#include <stdio.h>
#include <string>

#include "ESPectrum.h"
#include "Snapshot.h"
#include "Config.h"
#include "FileUtils.h"
#include "OSDMain.h"
#include "Ports.h"
#include "MemESP.h"
#include "CPU.h"
#include "Video.h"
#include "messages.h"
#include "AySound.h"
#include "Tape.h"
#include "Z80_JLS/z80.h"
#include "pwm_audio.h"
#include "wd1793.h"

#ifndef ESP32_SDL2_WRAPPER
#include "ZXKeyb.h"
#endif

#include "psram_spi.h"
#include "ps2.h"

using namespace std;

//=======================================================================================
// KEYBOARD
//=======================================================================================
fabgl::PS2Controller ESPectrum::PS2Controller;
bool ESPectrum::ps2kbd2 = false;

void joyPushData(fabgl::VirtualKey virtualKey, bool down) {
    fabgl::Keyboard* kbd = ESPectrum::PS2Controller.keyboard();
    if ( kbd ) {
        kbd->injectVirtualKey(virtualKey, down);
    }
}

volatile static uint32_t tickKbdRep = 0;
volatile static fabgl::VirtualKey last_key_pressed = fabgl::VirtualKey::VK_NONE;

fabgl::VirtualKey get_last_key_pressed(void) {
    return last_key_pressed;
}

void kbdPushData(fabgl::VirtualKey virtualKey, bool down) {
    static bool ctrlPressed = false;
    static bool altPressed = false;
    static bool delPressed = false;
    if (virtualKey == fabgl::VirtualKey::VK_LCTRL || virtualKey == fabgl::VirtualKey::VK_RCTRL) ctrlPressed = down;
    else if (virtualKey == fabgl::VirtualKey::VK_LALT || virtualKey == fabgl::VirtualKey::VK_RALT) altPressed = down;
    else if (virtualKey == fabgl::VirtualKey::VK_DELETE || virtualKey == fabgl::VirtualKey::VK_KP_PERIOD) delPressed = down;
    if (ctrlPressed && altPressed && delPressed) {
        f_unlink(MOS_FILE);
        watchdog_enable(1, true);
        while (true);
    }
    if (down) {
        if (ctrlPressed && virtualKey == fabgl::VirtualKey::VK_J) {
            Config::CursorAsJoy = !Config::CursorAsJoy;
        }
        if (last_key_pressed != virtualKey) {
            last_key_pressed = virtualKey;
            tickKbdRep = time_us_32();
        }
        switch (virtualKey) {
            case fabgl::VirtualKey::VK_NUMLOCK: keyboard_toggle_led(PS2_LED_NUM_LOCK); break;
            case fabgl::VirtualKey::VK_SCROLLLOCK: keyboard_toggle_led(PS2_LED_SCROLL_LOCK); break;
            case fabgl::VirtualKey::VK_CAPSLOCK: keyboard_toggle_led(PS2_LED_CAPS_LOCK); break;
        }
    } else {
        last_key_pressed = fabgl::VirtualKey::VK_NONE;
        tickKbdRep = 0;
    }
    fabgl::Keyboard* kbd = ESPectrum::PS2Controller.keyboard();
    fabgl::KeybJoystick* kbdj = ESPectrum::PS2Controller.keybjoystick();
    if ( kbd ) {
        switch (virtualKey) {
            case fabgl::VirtualKey::VK_KP_1: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_FIRE; break;
            case fabgl::VirtualKey::VK_KP_2: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_DOWN; break;
            case fabgl::VirtualKey::VK_KP_3: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE; break;
            case fabgl::VirtualKey::VK_KP_4: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_LEFT; break;
            case fabgl::VirtualKey::VK_KP_5: virtualKey = fabgl::VirtualKey::VK_SPACE; break;
            case fabgl::VirtualKey::VK_KP_6: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_RIGHT; break;
            case fabgl::VirtualKey::VK_KP_7: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_FIRE; break;
            case fabgl::VirtualKey::VK_KP_8: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_UP; break;
            case fabgl::VirtualKey::VK_KP_9: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE; break;
            case fabgl::VirtualKey::VK_KP_0: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE; break;
            case fabgl::VirtualKey::VK_KP_PERIOD: virtualKey = fabgl::VirtualKey::VK_KEMPSTON_FIRE; break;
            case fabgl::VirtualKey::VK_KP_ENTER:
                if(Config::rightSpace) virtualKey = fabgl::VirtualKey::VK_SPACE;
                else virtualKey = fabgl::VirtualKey::VK_RETURN;
            break;
        }
        if (virtualKey != fabgl::VirtualKey::VK_NONE) {
            virtualKey = kbd->manageCAPSLOCK(virtualKey);
        }
        kbd->injectVirtualKey(virtualKey, down);
    }
    if ( ESPectrum::ps2kbd2 && kbdj ) {
        kbdj->injectVirtualKey(virtualKey, down);
    }
}

void repeat_handler(void) {
    fabgl::VirtualKey v = last_key_pressed;
    if (v != fabgl::VirtualKey::VK_NONE) {
        if (tickKbdRep == 0) {
            kbdPushData(v, true);
        } else {
            uint32_t t2 = time_us_32();
            if (t2 - tickKbdRep > 500000) {
                tickKbdRep = 0;
            }
        }
    }
}

//=======================================================================================
// AUDIO
//=======================================================================================
uint8_t ESPectrum::audioBuffer_L[ESP_AUDIO_SAMPLES_PENTAGON] = { 0 };
uint8_t ESPectrum::audioBuffer_R[ESP_AUDIO_SAMPLES_PENTAGON] = { 0 };
uint32_t ESPectrum::overSamplebuf[ESP_AUDIO_SAMPLES_PENTAGON] = { 0 };
signed char ESPectrum::aud_volume = ESP_VOLUME_DEFAULT;
// signed char ESPectrum::aud_volume = ESP_VOLUME_MAX; // For .tap player test

uint32_t ESPectrum::audbufcnt = 0;
uint32_t ESPectrum::audbufcntover = 0;
uint32_t ESPectrum::faudbufcnt = 0;
uint32_t ESPectrum::audbufcntAY = 0;
uint32_t ESPectrum::faudbufcntAY = 0;
int ESPectrum::lastaudioBit = 0;
int ESPectrum::faudioBit = 0;
int ESPectrum::samplesPerFrame;
bool ESPectrum::AY_emu = false;
int ESPectrum::Audio_freq = 44100;
static int prevAudio_freq = 44100;
unsigned char ESPectrum::audioSampleDivider;
static int audioBitBuf = 0;
static unsigned char audioBitbufCount = 0;
///QueueHandle_t audioTaskQueue;
///TaskHandle_t ESPectrum::audioTaskHandle;
uint8_t *param;

//=======================================================================================
// TAPE OSD
//=======================================================================================

int ESPectrum::TapeNameScroller = 0;

//=======================================================================================
// BETADISK
//=======================================================================================

bool ESPectrum::trdos = false;
WD1793 ESPectrum::Betadisk;

//=======================================================================================
// ARDUINO FUNCTIONS
//=======================================================================================
/**
#ifndef ESP32_SDL2_WRAPPER
#define NOP() asm volatile ("nop")
#else
#define NOP() {for(int i=0;i<1000;i++){}}
#endif

IRAM_ATTR unsigned long millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

IRAM_ATTR void delayMicroseconds(int64_t us)
{
    int64_t m = esp_timer_get_time();
    if(us){
        int64_t e = (m + us);
        if(m > e){ //overflow
            while(esp_timer_get_time() > e){
                NOP();
            }
        }
        while(esp_timer_get_time() < e){
            NOP();
        }
    }
}
*/
//=======================================================================================
// TIMING / SYNC
//=======================================================================================

double ESPectrum::totalseconds = 0;
double ESPectrum::totalsecondsnodelay = 0;
int64_t ESPectrum::target;
int ESPectrum::sync_cnt = 0;
volatile bool ESPectrum::vsync = false;
int64_t ESPectrum::ts_start;
int64_t ESPectrum::elapsed;
int64_t ESPectrum::idle;
bool ESPectrum::ESP_delay = true;
int ESPectrum::ESPoffset = 0;

//=======================================================================================
// LOGGING / TESTING
//=======================================================================================

int ESPectrum::ESPtestvar = 0;
int ESPectrum::ESPtestvar1 = 0;
int ESPectrum::ESPtestvar2 = 0;

void ShowStartMsg() {
    
    fabgl::VirtualKeyItem Nextkey;

    VIDEO::vga.clear(zxColor(7,0));

    OSD::drawOSD(false);

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

    OSD::osdAt(7, 1);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
    VIDEO::vga.print(Config::lang ? StartMsg[0] : StartMsg[1]);

    VIDEO::vga.setTextColor(zxColor(16,0), zxColor(1, 0));
    OSD::osdAt(7, Config::lang ? 28 : 25);          
    VIDEO::vga.print("ESP");
    OSD::osdAt(9, 1);          
    VIDEO::vga.print("ESP");
    OSD::osdAt(13, 13);          
    VIDEO::vga.print("ESP");

    OSD::osdAt(17, 4);          
    VIDEO::vga.setTextColor(zxColor(3, 1), zxColor(1, 0));
    VIDEO::vga.print("https://patreon.com/ESPectrum");

    char msg[38];
    
    for (int i=20; i >= 0; i--) {
        OSD::osdAt(19, 1);          
        sprintf(msg,Config::lang ? "Este mensaje se cerrar" "\xA0" " en %02d segundos" : "This message will close in %02d seconds",i);
        VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
        VIDEO::vga.print(msg);
        sleep_ms(1);
    }

    VIDEO::vga.clear(zxColor(7,0));

    // Disable StartMsg
    Config::StartMsg = false;
    Config::save("StartMsg");
}

/**
void showMemInfo(const char* caption = "ZX-ESPectrum-IDF") {

#ifndef ESP32_SDL2_WRAPPER

multi_heap_info_t info;

heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal RAM, memory capable to store data or to create new task
printf("=========================================================================\n");
printf(" %s - Mem info:\n",caption);
printf("-------------------------------------------------------------------------\n");
printf("Total currently free in all non-continues blocks: %d\n", info.total_free_bytes);
printf("Minimum free ever: %d\n", info.minimum_free_bytes);
printf("Largest continues block to allocate big array: %d\n", info.largest_free_block);
printf("Heap caps get free size (MALLOC_CAP_8BIT): %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
printf("Heap caps get free size (MALLOC_CAP_32BIT): %d\n", heap_caps_get_free_size(MALLOC_CAP_32BIT));
printf("Heap caps get free size (MALLOC_CAP_INTERNAL): %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
printf("=========================================================================\n\n");

// heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);

// printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_8BIT);            

// printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_32BIT);                        

// printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);

// printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_DMA);            

// printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_EXEC);            

// printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_IRAM_8BIT);            

// printf("=========================================================================\n");
// heap_caps_dump_all();

// printf("=========================================================================\n");

// UBaseType_t wm;
// wm = uxTaskGetStackHighWaterMark(audioTaskHandle);
// printf("Audio Task Stack HWM: %u\n", wm);
// // wm = uxTaskGetStackHighWaterMark(loopTaskHandle);
// // printf("Loop Task Stack HWM: %u\n", wm);
// wm = uxTaskGetStackHighWaterMark(VIDEO::videoTaskHandle);
// printf("Video Task Stack HWM: %u\n", wm);

#endif

}
*/
//=======================================================================================
// BOOT KEYBOARD
//=======================================================================================
void ESPectrum::bootKeyboard() {

    auto Kbd = PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;
    int i = 0;
    string s = "00";

    // printf("Boot kbd!\n");

    for (; i < 200; i++) {

        if (ZXKeyb::Exists) {

            // Process physical keyboard
            ZXKeyb::process();
            
            // Detect and process physical kbd menu key combinations
            if (!bitRead(ZXKeyb::ZXcols[3], 0)) { // 1
                s[0]='1';
            } else
            if (!bitRead(ZXKeyb::ZXcols[3], 1)) { // 2
                s[0]='2';
            } else
            if (!bitRead(ZXKeyb::ZXcols[3], 2)) { // 3
                s[0]='3';
            }

            if (!bitRead(ZXKeyb::ZXcols[2], 0)) { // Q
                s[1]='Q';
            } else 
            if (!bitRead(ZXKeyb::ZXcols[2], 1)) { // W
                s[1]='W';
            }

        }

        while (Kbd->virtualKeyAvailable()) {

            bool r = Kbd->getNextVirtualKey(&NextKey);

            if (r && NextKey.down) {

                // Check keyboard status
                switch (NextKey.vk) {
                    case fabgl::VK_1:
                        s[0] = '1';
                        break;
                    case fabgl::VK_2:
                        s[0] = '2';
                        break;
                    case fabgl::VK_3:
                        s[0] = '3';
                        break;
                    case fabgl::VK_Q:
                    case fabgl::VK_q:    
                        s[1] = 'Q';
                        break;
                    case fabgl::VK_W:
                    case fabgl::VK_w:    
                        s[1] = 'W';
                        break;
                }

            }

        }

        if (s.find('0') == std::string::npos) break;

        delayMicroseconds(1000);

    }

    // printf("Boot kbd end!\n");

    if (i < 200) {
///        Config::videomode = (s[0] == '1') ? 0 : (s[0] == '2') ? 1 : 2;
///        Config::aspect_16_9 = (s[1] == 'Q') ? false : true;
        Config::ram_file="none";
        Config::save();
        // printf("%s\n", s.c_str());
    }

}

//=======================================================================================
// SETUP
//=======================================================================================
// TaskHandle_t ESPectrum::loopTaskHandle;

// uint32_t ESPectrum::sessid;

#if !PICO_RP2040
    #ifdef HDMI
    static unsigned char MemESP_ram[(112ul + 256ul) << 10];
    #else
    static unsigned char MemESP_ram[(128ul + 256ul) << 10];
    #endif
#else
    #ifdef HDMI
    static unsigned char MemESP_ram[112ul << 10];
    #else
    static unsigned char MemESP_ram[128ul << 10];
    #endif
#endif

static unsigned char *MemESP_ram0 = MemESP_ram;
static unsigned char *MemESP_ram1 = MemESP_ram + 0x4000;
static unsigned char *MemESP_ram2 = MemESP_ram + 0x4000 + 0x8000;

static unsigned char *MemESP_ram4 = MemESP_ram + 4 * 0x4000;
static unsigned char *MemESP_ram5 = MemESP_ram + 5 * 0x4000;
#if PICO_RP2040
    #ifdef HDMI
    static unsigned char *MemESP_ram7 = MemESP_ram + 6 * 0x4000;
    #else
    static unsigned char *MemESP_ram6 = MemESP_ram + 6 * 0x4000;
    static unsigned char *MemESP_ram7 = MemESP_ram + 7 * 0x4000;
    #endif
#else
    static unsigned char *MemESP_ram6 = MemESP_ram + 6 * 0x4000;
    static unsigned char *MemESP_ram7 = MemESP_ram + 7 * 0x4000;
#endif

void ESPectrum::setup() 
{
    //=======================================================================================
    // PHYSICAL KEYBOARD (SINCLAIR 8 + 5 MEMBRANE KEYBOARD)
    //=======================================================================================
    ZXKeyb::setup();

    //=======================================================================================
    // INIT FILESYSTEM
    //=======================================================================================
    FileUtils::initFileSystem();

    mem_desc_t::reset();

    //=======================================================================================
    // LOAD CONFIG
    //=======================================================================================
    Config::load();
    
    // Set arch if there's no snapshot to load
    if (Config::ram_file == NO_RAM_FILE) {
        if (Config::pref_arch.substr(Config::pref_arch.length()-1) == "R") {
            Config::pref_arch.pop_back();
            Config::save("pref_arch");
        } else {
            if (Config::pref_arch != "Last") Config::arch = Config::pref_arch;

            if (Config::arch == "48K") {
                if (Config::pref_romSet_48 != "Last") 
                    Config::romSet = Config::pref_romSet_48;
                else
                    Config::romSet = Config::romSet48;            
            } else if (Config::arch == "128K") {
                if (Config::pref_romSet_128 != "Last")
                    Config::romSet = Config::pref_romSet_128;
                else
                    Config::romSet = Config::romSet128;
            } else if (Config::arch == "P512") {
                if (Config::pref_romSetP512 != "Last")
                    Config::romSet = Config::pref_romSetP512;
                else
                    Config::romSet = Config::romSetP512;
            } else if (Config::arch == "P1024") {
                if (Config::pref_romSetP1M != "Last")
                    Config::romSet = Config::pref_romSetP1M;
                else
                    Config::romSet = Config::romSetP1M;
            } else {
                if (Config::pref_romSetPent != "Last")
                    Config::romSet = Config::pref_romSetPent;
                else
                    Config::romSet = Config::romSetPent;
            }
        }
    }

    //=======================================================================================
    // INIT PS/2 KEYBOARD
    //=======================================================================================

    ESPectrum::ps2kbd2 = (Config::ps2_dev2 != 0);
    
    if (ZXKeyb::Exists) {
///        PS2Controller.begin(ps2kbd2 ? PS2Preset::KeyboardPort0 : PS2Preset::zxKeyb, KbdMode::CreateVirtualKeysQueue);
    } else {
///        PS2Controller.begin(ps2kbd2 ? PS2Preset::KeyboardPort0_KeybJoystickPort1 : PS2Preset::KeyboardPort0, KbdMode::CreateVirtualKeysQueue);
    }

    ps2kbd2 &= !ZXKeyb::Exists;

    // Set Scroll Lock Led as current CursorAsJoy value
    PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);
    if(ps2kbd2)
        PS2Controller.keybjoystick()->setLEDs(false, false, Config::CursorAsJoy);

    // Set TAB and GRAVEACCENT behaviour
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

    //=======================================================================================
    // BOOTKEYS: Read keyboard for 200 ms. checking boot keys
    //=======================================================================================

    // printf("Waiting boot keys\n");
    bootKeyboard();
    // printf("End Waiting boot keys\n");

    //=======================================================================================
    // MEMORY SETUP
    //=======================================================================================
    MemESP::ram[5].assign_ram(MemESP_ram5, 5, true);
    MemESP::ram[0].assign_ram(MemESP_ram0, 0, false);

    MemESP::ram[2].assign_ram(MemESP_ram2, 2, false);
    MemESP::ram[7].assign_ram(MemESP_ram7, 7, true);

    MemESP::ram[1].assign_ram(MemESP_ram1, 1, true);
    MemESP::ram[3].assign_ram(MemESP_ram1 + 0x4000, 3, true); /// why?

    MemESP::ram[4].assign_ram(MemESP_ram4, 4, false);
#if PICO_RP2040
    #ifdef HDMI
    MemESP::ram[6].assign_vram(6);
    #else
    MemESP::ram[6].assign_ram(MemESP_ram6, 6, false);
    #endif
#else
    MemESP::ram[6].assign_ram(MemESP_ram6, 6, false);
#endif

#if !PICO_RP2040
    #ifdef HDMI
    for (size_t i = 8; i < 23; ++i) MemESP::ram[i].assign_ram(MemESP_ram + 0x4000 * i, i, false);
    for (size_t i = 23; i < 64; ++i) MemESP::ram[i].assign_vram(i);
    #else
    for (size_t i = 8; i < 24; ++i) MemESP::ram[i].assign_ram(MemESP_ram + 0x4000 * i, i, false);
    for (size_t i = 24; i < 64; ++i) MemESP::ram[i].assign_vram(i);
    #endif
#else
    for (size_t i = 8; i < 64; ++i) MemESP::ram[i].assign_vram(i);
#endif

    // Load romset
    Config::requestMachine(Config::arch, Config::romSet);

    MemESP::page0ram = 0;
    MemESP::page128 = 0;
    MemESP::romInUse = 0;
    MemESP::bankLatch = 0;
    MemESP::videoLatch = 0;
    MemESP::romLatch = 0;

    MemESP::ramCurrent[0] = MemESP::page0ram ? MemESP::ram[0] : MemESP::rom[MemESP::romInUse];
    MemESP::ramCurrent[1] = MemESP::ram[5];
    MemESP::ramCurrent[2] = MemESP::ram[2];
    MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];

    MemESP::ramContended[0] = false;
    MemESP::ramContended[1] = Config::arch == "P1024" || Config::arch == "P512" || Config::arch == "Pentagon" ? false : true;
    MemESP::ramContended[2] = false;
    MemESP::ramContended[3] = false;

    // if (Config::arch == "48K") MemESP::pagingLock = 1; else MemESP::pagingLock = 0;
    MemESP::pagingLock = Config::arch == "48K" ? 1 : 0;

///    if (Config::slog_on) showMemInfo("RAM Initialized");

    //=======================================================================================
    // VIDEO
    //=======================================================================================

    VIDEO::Init();
    VIDEO::Reset();
    
///    if (Config::slog_on) showMemInfo("VGA started");

    if (Config::StartMsg) ShowStartMsg(); // Show welcome message

    //=======================================================================================
    // AUDIO
    //=======================================================================================
    // Set samples per frame and AY_emu flag depending on arch
    if (Config::arch == "48K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_48; 
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
        AY_emu = Config::AY48;
        Audio_freq = ESP_AUDIO_FREQ_48;
    } else if (Config::arch == "128K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_128;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_128;
        AY_emu = true;        
        Audio_freq = ESP_AUDIO_FREQ_128;
    } else { /// if (Config::arch == "P512" || Config::arch == "Pentagon") {
        samplesPerFrame = ESP_AUDIO_SAMPLES_PENTAGON;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_PENTAGON;
        AY_emu = true;        
        Audio_freq = ESP_AUDIO_FREQ_PENTAGON;
    }

    if (Config::tape_player) {
        AY_emu = false; // Disable AY emulation if tape player mode is set
        ESPectrum::aud_volume = ESP_VOLUME_MAX;
    } else
        ESPectrum::aud_volume = ESP_VOLUME_DEFAULT;

    ESPoffset = 0;

    // Create Audio task
///    audioTaskQueue = xQueueCreate(1, sizeof(uint8_t *));
    // Latest parameter = Core. In ESPIF, main task runs on core 0 by default. In Arduino, loop() runs on core 1.
///    xTaskCreatePinnedToCore(&ESPectrum::audioTask, "audioTask", 1024 /*1536*/, NULL, configMAX_PRIORITIES - 1, &audioTaskHandle, 1);
    pwm_audio_set_volume(aud_volume);
    pcm_setup(Audio_freq, samplesPerFrame << 1);
    prevAudio_freq = Audio_freq;

    // AY Sound
    chip0.init();
    chip0.set_sound_format(Audio_freq,1,8);
    chip0.set_stereo(AYEMU_MONO,NULL);
    chip0.reset();
    chip1.init();
    chip1.set_sound_format(Audio_freq,1,8);
    chip1.set_stereo(AYEMU_MONO,NULL);
    chip1.reset();

    // Init tape
    Tape::Init();
    Tape::tapeFileName = "none";
    Tape::tapeStatus = TAPE_STOPPED;
    Tape::SaveStatus = SAVE_STOPPED;
    Tape::romLoading = false;

    // Init CPU
    Z80::create();

    // Set Ports starting values
    for (int i = 0; i < 128; i++) Ports::port[i] = 0xBF;
    if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0; // Kempston
    if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff; // Fuller

    // Read joystick default definition
    for (int n = 0; n < 24; n++)
        ESPectrum::JoyVKTranslation[n] = (fabgl::VirtualKey) Config::joydef[n];

    // Init disk controller
    Betadisk.Init();

    // Reset cpu
    CPU::reset();

    // Load snapshot if present in Config::
    if (Config::ram_file != NO_RAM_FILE) {
/**
        FileUtils::SNA_Path = Config::SNA_Path;
        FileUtils::fileTypes[DISK_SNAFILE].begin_row = Config::SNA_begin_row;
        FileUtils::fileTypes[DISK_SNAFILE].focus = Config::SNA_focus;
        FileUtils::fileTypes[DISK_SNAFILE].fdMode = Config::SNA_fdMode;
        FileUtils::fileTypes[DISK_SNAFILE].fileSearch = Config::SNA_fileSearch;

        FileUtils::TAP_Path = Config::TAP_Path;
        FileUtils::fileTypes[DISK_TAPFILE].begin_row = Config::TAP_begin_row;
        FileUtils::fileTypes[DISK_TAPFILE].focus = Config::TAP_focus;
        FileUtils::fileTypes[DISK_TAPFILE].fdMode = Config::TAP_fdMode;
        FileUtils::fileTypes[DISK_TAPFILE].fileSearch = Config::TAP_fileSearch;

        FileUtils::DSK_Path = Config::DSK_Path;
        FileUtils::fileTypes[DISK_DSKFILE].begin_row = Config::DSK_begin_row;
        FileUtils::fileTypes[DISK_DSKFILE].focus = Config::DSK_focus;
        FileUtils::fileTypes[DISK_DSKFILE].fdMode = Config::DSK_fdMode;
        FileUtils::fileTypes[DISK_DSKFILE].fileSearch = Config::DSK_fileSearch;
*/
        LoadSnapshot(Config::ram_file, "", "");

        Config::last_ram_file = Config::ram_file;
        Config::ram_file = NO_RAM_FILE;
        Config::save("ram");
    }
///    if (Config::slog_on) showMemInfo("ZX-ESPectrum-IDF setup finished.");

    // Create loop function as task: it doesn't seem better than calling from main.cpp and increases RAM consumption (4096 bytes for stack).
    // xTaskCreatePinnedToCore(&ESPectrum::loop, "loopTask", 4096, NULL, 1, &loopTaskHandle, 0);
}

//=======================================================================================
// RESET
//=======================================================================================
void ESPectrum::reset()
{
    // Ports
    for (int i = 0; i < 128; i++) Ports::port[i] = 0xBF;
    if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0; // Kempston
    if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff; // Fuller

    // Read joystick default definition
    for (int n = 0; n < 24; n++)
        ESPectrum::JoyVKTranslation[n] = (fabgl::VirtualKey) Config::joydef[n];

    // Memory
    MemESP::page0ram = 0;
    MemESP::page128 = 0;
    MemESP::romInUse = 0;
    MemESP::bankLatch = 0;
    MemESP::videoLatch = 0;
    MemESP::romLatch = 0;

    MemESP::ramCurrent[0] = MemESP::page0ram ? MemESP::ram[0] : MemESP::rom[MemESP::romInUse];
    MemESP::ramCurrent[1] = MemESP::ram[5];
    MemESP::ramCurrent[2] = MemESP::ram[2];
    MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch];

    MemESP::ramContended[0] = false;
    MemESP::ramContended[1] = Config::arch == "P1024" || Config::arch == "P512" || Config::arch == "Pentagon" ? false : true;
    MemESP::ramContended[2] = false;
    MemESP::ramContended[3] = false;

    MemESP::pagingLock = Config::arch == "48K" ? 1 : 0;

    VIDEO::Reset();

    // Reinit disk controller
    // Betadisk.ShutDown();
    // Betadisk.Init();
    Betadisk.EnterIdle();

    Tape::tapeFileName = "none";
    if (Tape::tape.obj.fs != NULL) {
        f_close(&Tape::tape);
    }
    Tape::tapeStatus = TAPE_STOPPED;
    Tape::tapePhase = TAPE_PHASE_STOPPED;
    Tape::SaveStatus = SAVE_STOPPED;
    Tape::romLoading = false;

    // Empty audio buffers
    memset(overSamplebuf, 0, sizeof(overSamplebuf));
    memset(audioBuffer_L, 0, sizeof(audioBuffer_L));
    memset(audioBuffer_R, 0, sizeof(audioBuffer_R));
    memset(chip0.SamplebufAY_L, 0, sizeof(chip0.SamplebufAY_L));
    memset(chip0.SamplebufAY_R, 0, sizeof(chip0.SamplebufAY_R));
    memset(chip1.SamplebufAY_L, 0, sizeof(chip1.SamplebufAY_L));
    memset(chip1.SamplebufAY_R, 0, sizeof(chip1.SamplebufAY_R));
    lastaudioBit=0;

    // Set samples per frame and AY_emu flag depending on arch
    prevAudio_freq = Audio_freq;
    if (Config::arch == "48K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_48; 
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
        AY_emu = Config::AY48;
        Audio_freq = ESP_AUDIO_FREQ_48;
    } else if (Config::arch == "128K") {
        samplesPerFrame=ESP_AUDIO_SAMPLES_128;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_128;
        AY_emu = true;        
        Audio_freq = ESP_AUDIO_FREQ_128;
    } else { /// if (Config::arch == "Pentagon") {
        samplesPerFrame = ESP_AUDIO_SAMPLES_PENTAGON;
        audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_PENTAGON;
        AY_emu = true;        
        Audio_freq = ESP_AUDIO_FREQ_PENTAGON;
    }

    if (Config::tape_player) AY_emu = false; // Disable AY emulation if tape player mode is set

    ESPoffset = 0;

    // Readjust output pwmaudio frequency if needed
    if (prevAudio_freq != Audio_freq) {
        
        // printf("Resetting pwmaudio to freq: %d\n",Audio_freq);
        pcm_setup(Audio_freq, samplesPerFrame << 1);
/***
        esp_err_t res;
        res = pwm_audio_set_sample_rate(Audio_freq);
        if (res != ESP_OK) {
            printf("Can't set sample rate\n");
        }
*/
    }
    
    // Reset AY emulation
    chip0.init();
    chip0.set_sound_format(Audio_freq,1,8);
    chip0.set_stereo(AYEMU_MONO,NULL);
    chip0.reset();
    chip1.init();
    chip1.set_sound_format(Audio_freq,1,8);
    chip1.set_stereo(AYEMU_MONO,NULL);
    chip1.reset();

    CPU::reset();
}

//=======================================================================================
// KEYBOARD / KEMPSTON
//=======================================================================================
IRAM_ATTR bool ESPectrum::readKbd(fabgl::VirtualKeyItem *Nextkey) {
    
    bool r = PS2Controller.keyboard()->getNextVirtualKey(Nextkey);
    // Global keys
    if (Nextkey->down) {
        if (Nextkey->vk == fabgl::VK_PRINTSCREEN) { // Capture framebuffer to BMP file in SD Card (thx @dcrespo3d!)
            CaptureToBmp();
            r = false;
        } else
        if (Nextkey->vk == fabgl::VK_SCROLLLOCK) { // Change CursorAsJoy setting
            Config::CursorAsJoy = !Config::CursorAsJoy;
            PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);
            if(ps2kbd2)
                PS2Controller.keybjoystick()->setLEDs(false, false, Config::CursorAsJoy);
            Config::save("CursorAsJoy");
            r = false;
        }
    }

    return r;
}

//
// Read second ps/2 port and inject on first queue
//
IRAM_ATTR void ESPectrum::readKbdJoy() {
    if (ps2kbd2) {
        fabgl::VirtualKeyItem NextKey;
        auto KbdJoy = PS2Controller.keybjoystick();    
        while (KbdJoy->virtualKeyAvailable()) {
            PS2Controller.keybjoystick()->getNextVirtualKey(&NextKey);
            ESPectrum::PS2Controller.keyboard()->injectVirtualKey(NextKey.vk, NextKey.down, false);
        }
    }
}

fabgl::VirtualKey ESPectrum::JoyVKTranslation[24];
//     fabgl::VK_FULLER_LEFT, // Left
//     fabgl::VK_FULLER_RIGHT, // Right
//     fabgl::VK_FULLER_UP, // Up
//     fabgl::VK_FULLER_DOWN, // Down
//     fabgl::VK_S, // Start
//     fabgl::VK_M, // Mode
//     fabgl::VK_FULLER_FIRE, // A
//     fabgl::VK_9, // B
//     fabgl::VK_SPACE, // C
//     fabgl::VK_X, // X
//     fabgl::VK_Y, // Y
//     fabgl::VK_Z, // Z

fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_NONE;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_NONE;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_TAB;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_GRAVEACCENT;

IRAM_ATTR void ESPectrum::processKeyboard() {

    static uint8_t PS2cols[8] = { 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf };    
    static int zxDelay = 0;
    auto Kbd = PS2Controller.keyboard();
    fabgl::VirtualKeyItem NextKey;
    fabgl::VirtualKey KeytoESP;
    bool Kdown;
    bool r = false;
    bool j[10] = { true, true, true, true, true, true, true, true, true, true };
    // bool j1 =  true;
    // bool j2 = true;
    // bool j3 = true;
    // bool j4 = true;
    // bool j5 = true;
    // bool j6 = true;
    // bool j7 = true;
    // bool j8 = true;
    // bool j9 = true;
    // bool j0 = true;
    bool jShift = true;

    readKbdJoy();

    while (Kbd->virtualKeyAvailable()) {

        r = readKbd(&NextKey);

        if (r) {

            KeytoESP = NextKey.vk;
            Kdown = NextKey.down;
          
            if (KeytoESP >= fabgl::VK_JOY1LEFT && KeytoESP <= fabgl::VK_JOY2Z) {
                // printf("KeytoESP: %d\n",KeytoESP);
                ESPectrum::PS2Controller.keyboard()->injectVirtualKey(JoyVKTranslation[KeytoESP - 248], Kdown, false);
                continue;
            }

            if ((Kdown) && ((KeytoESP >= fabgl::VK_F1 && KeytoESP <= fabgl::VK_F12) || KeytoESP == fabgl::VK_PAUSE || KeytoESP == fabgl::VK_VOLUMEUP || KeytoESP == fabgl::VK_VOLUMEDOWN || KeytoESP == fabgl::VK_VOLUMEMUTE)) {

                int64_t osd_start = esp_timer_get_time();

                OSD::do_OSD(KeytoESP, Kbd->isVKDown(fabgl::VK_LCTRL) || Kbd->isVKDown(fabgl::VK_RCTRL));

                Kbd->emptyVirtualKeyQueue();
                
                // Set all zx keys as not pressed
                for (uint8_t i = 0; i < 8; i++) ZXKeyb::ZXcols[i] = 0xbf;
                zxDelay = 15;
                
                // totalseconds = 0;
                // totalsecondsnodelay = 0;
                // VIDEO::framecnt = 0;

                #ifdef DIRTY_LINES
                for (int i=0; i < SPEC_H; i++) VIDEO::dirty_lines[i] |= 0x01;
                #endif // DIRTY_LINES

                // Refresh border
                VIDEO::brdnextframe = true;

                ESPectrum::ts_start += esp_timer_get_time() - osd_start;

                return;

            }

            // Reset keys
            if (Kdown && NextKey.LALT) {
                if (NextKey.CTRL) {
                    if (KeytoESP == fabgl::VK_DELETE) {
                        // printf("Ctrl + Alt + Supr!\n");
                        // ESP host reset
                        Config::ram_file = NO_RAM_FILE;
                        Config::save("ram");
                        OSD::esp_hard_reset();
                    } else if (KeytoESP == fabgl::VK_BACKSPACE) {
                        // printf("Ctrl + Alt + backSpace!\n");
                        // Hard
                        if (Config::ram_file != NO_RAM_FILE) {
                            Config::ram_file = NO_RAM_FILE;
                        }
                        Config::last_ram_file = NO_RAM_FILE;
                        ESPectrum::reset();
                        return;
                    }
                } else if (KeytoESP == fabgl::VK_BACKSPACE) {
                    // printf("Alt + backSpace!\n");
                    // Soft reset
                    if (Config::last_ram_file != NO_RAM_FILE) {
                        LoadSnapshot(Config::last_ram_file, "", "");
                        Config::ram_file = Config::last_ram_file;
                    }
                    else
                        ESPectrum::reset();
                    return;
                }
            }

            if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON || Config::joyPS2 == JOYPS2_KEMPSTON) Ports::port[0x1f] = 0;
            if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER || Config::joyPS2 == JOYPS2_FULLER) Ports::port[0x7f] = 0xff;

            if (Config::joystick1 == JOY_KEMPSTON || Config::joystick2 == JOY_KEMPSTON) {

                for (int i = fabgl::VK_KEMPSTON_RIGHT; i <= fabgl::VK_KEMPSTON_ALTFIRE; i++)
                    if (Kbd->isVKDown((fabgl::VirtualKey) i))
                        bitWrite(Ports::port[0x1f], i - fabgl::VK_KEMPSTON_RIGHT, 1);

            }

            if (Config::joystick1 == JOY_FULLER || Config::joystick2 == JOY_FULLER) {

                // Fuller
                if (Kbd->isVKDown(fabgl::VK_FULLER_RIGHT)) {
                    bitWrite(Ports::port[0x7f], 3, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_LEFT)) {
                    bitWrite(Ports::port[0x7f], 2, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_DOWN)) {
                    bitWrite(Ports::port[0x7f], 1, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_UP)) {
                    bitWrite(Ports::port[0x7f], 0, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_FULLER_FIRE)) {
                    bitWrite(Ports::port[0x7f], 7, 0);
                }

            }

            jShift = !(Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT));
           
            if (Config::CursorAsJoy) {
                // Kempston Joystick emulation
                if (Config::joyPS2 == JOYPS2_KEMPSTON) {

                    if (Kbd->isVKDown(fabgl::VK_RIGHT)) {
                        j[8] = jShift;
                        bitWrite(Ports::port[0x1f], 0, j[8]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_LEFT)) {
                        j[5] = jShift;
                        bitWrite(Ports::port[0x1f], 1, j[5]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_DOWN)) {
                        j[6] = jShift;
                        bitWrite(Ports::port[0x1f], 2, j[6]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_UP)) {
                        j[7] = jShift;
                        bitWrite(Ports::port[0x1f], 3, j[7]);
                    }

                // Fuller Joystick emulation
                } else if (Config::joyPS2 == JOYPS2_FULLER) {

                    if (Kbd->isVKDown(fabgl::VK_RIGHT)) {
                        j[8] = jShift;
                        bitWrite(Ports::port[0x7f], 3, !j[8]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_LEFT)) {
                        j[5] = jShift;
                        bitWrite(Ports::port[0x7f], 2, !j[5]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_DOWN)) {
                        j[6] = jShift;
                        bitWrite(Ports::port[0x7f], 1, !j[6]);
                    }

                    if (Kbd->isVKDown(fabgl::VK_UP)) {
                        j[7] = jShift;
                        bitWrite(Ports::port[0x7f], 0, !j[7]);
                    }

                } else if (Config::joyPS2 == JOYPS2_CURSOR) {

                    j[5] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                    j[8] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                    j[7] =  !Kbd->isVKDown(fabgl::VK_UP);
                    j[6] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                  
                } else if (Config::joyPS2 == JOYPS2_SINCLAIR1) { // Right Sinclair

                    if (jShift) {
                        j[9] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[8] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                        j[7] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[6] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                    } else {
                        j[5] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                        j[8] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[7] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[6] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                    }

                } else if (Config::joyPS2 == JOYPS2_SINCLAIR2) { // Left Sinclair

                    if (jShift) {
                        j[4] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[3] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                        j[2] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[1] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                    } else {
                        j[5] =  !Kbd->isVKDown(fabgl::VK_LEFT);
                        j[8] =  !Kbd->isVKDown(fabgl::VK_RIGHT);
                        j[7] =  !Kbd->isVKDown(fabgl::VK_UP);
                        j[6] =  !Kbd->isVKDown(fabgl::VK_DOWN);
                    }

                }
            } else {
                // Cursor Keys
                if (Kbd->isVKDown(fabgl::VK_RIGHT)) {
                    jShift = false;
                    j[8] = jShift;
                }

                if (Kbd->isVKDown(fabgl::VK_LEFT)) {
                    jShift = false;
                    j[5] = jShift;
                }

                if (Kbd->isVKDown(fabgl::VK_DOWN)) {
                    jShift = false;                
                    j[6] = jShift;
                }

                if (Kbd->isVKDown(fabgl::VK_UP)) {
                    jShift = false;
                    j[7] = jShift;
                }

            }

            // Keypad PS/2 Joystick emulation
            if (Config::joyPS2 == JOYPS2_KEMPSTON) {

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    bitWrite(Ports::port[0x1f], 0, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    bitWrite(Ports::port[0x1f], 1, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    bitWrite(Ports::port[0x1f], 2, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    bitWrite(Ports::port[0x1f], 3, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    bitWrite(Ports::port[0x1f], 4, 1);
                }

                if (Kbd->isVKDown(fabgl::VK_SLASH) ||
                 //Kbd->isVKDown(fabgl::VK_QUESTION) ||
                 Kbd->isVKDown(fabgl::VK_RGUI) || Kbd->isVKDown(fabgl::VK_APPLICATION) || Kbd->isVKDown(VK_ESPECTRUM_FIRE2)) {
                    bitWrite(Ports::port[0x1f], 5, 1);
                }

            } else if (Config::joyPS2 == JOYPS2_FULLER) {

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    bitWrite(Ports::port[0x7f], 3, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    bitWrite(Ports::port[0x7f], 2, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    bitWrite(Ports::port[0x7f], 1, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    bitWrite(Ports::port[0x7f], 0, 0);
                }

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    bitWrite(Ports::port[0x7f], 7, 0);
                }

            } else if (Config::joyPS2 == JOYPS2_CURSOR) {

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    jShift = true;
                    j[5] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    jShift = true;
                    j[8] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    jShift = true;
                    j[7] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    jShift = true;
                    j[6] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    jShift = true;
                    j[0] = false;
                };
                
            } else if (Config::joyPS2 == JOYPS2_SINCLAIR1) { // Right Sinclair

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    jShift = true;
                    j[6] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    jShift = true;
                    j[7] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    jShift = true;
                    j[9] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    jShift = true;
                    j[8] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    jShift = true;
                    j[0] = false;
                };

            } else if (Config::joyPS2 == JOYPS2_SINCLAIR2) { // Left Sinclair

                if (Kbd->isVKDown(fabgl::VK_KP_LEFT)) {
                    jShift = true;
                    j[1] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_RIGHT)) {
                    jShift = true;
                    j[2] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_UP)) {
                    jShift = true;
                    j[4] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_KP_DOWN) || Kbd->isVKDown(fabgl::VK_KP_CENTER)) {
                    jShift = true;
                    j[3] = false;
                };

                if (Kbd->isVKDown(fabgl::VK_RALT) || Kbd->isVKDown(VK_ESPECTRUM_FIRE1)) {
                    jShift = true;
                    j[5] = false;
                };

            }

            // Check keyboard status and map it to Spectrum Ports
            
            bitWrite(PS2cols[0], 0, (jShift) 
                & (!Kbd->isVKDown(fabgl::VK_BACKSPACE))
                & (!Kbd->isVKDown(fabgl::VK_CAPSLOCK)) // Caps lock   
                &   (!Kbd->isVKDown(VK_ESPECTRUM_GRAVEACCENT)) // Edit
                &   (!Kbd->isVKDown(VK_ESPECTRUM_TAB)) // Extended mode                                      
                &   (!Kbd->isVKDown(fabgl::VK_ESCAPE)) // Break                             
                ); // CAPS SHIFT
            bitWrite(PS2cols[0], 1, (!Kbd->isVKDown(fabgl::VK_Z)) & (!Kbd->isVKDown(fabgl::VK_z)));
            bitWrite(PS2cols[0], 2, (!Kbd->isVKDown(fabgl::VK_X)) & (!Kbd->isVKDown(fabgl::VK_x)));
            bitWrite(PS2cols[0], 3, (!Kbd->isVKDown(fabgl::VK_C)) & (!Kbd->isVKDown(fabgl::VK_c)));
            bitWrite(PS2cols[0], 4, (!Kbd->isVKDown(fabgl::VK_V)) & (!Kbd->isVKDown(fabgl::VK_v)));

            bitWrite(PS2cols[1], 0, (!Kbd->isVKDown(fabgl::VK_A)) & (!Kbd->isVKDown(fabgl::VK_a)));    
            bitWrite(PS2cols[1], 1, (!Kbd->isVKDown(fabgl::VK_S)) & (!Kbd->isVKDown(fabgl::VK_s)));
            bitWrite(PS2cols[1], 2, (!Kbd->isVKDown(fabgl::VK_D)) & (!Kbd->isVKDown(fabgl::VK_d)));
            bitWrite(PS2cols[1], 3, (!Kbd->isVKDown(fabgl::VK_F)) & (!Kbd->isVKDown(fabgl::VK_f)));
            bitWrite(PS2cols[1], 4, (!Kbd->isVKDown(fabgl::VK_G)) & (!Kbd->isVKDown(fabgl::VK_g)));

            bitWrite(PS2cols[2], 0, (!Kbd->isVKDown(fabgl::VK_Q)) & (!Kbd->isVKDown(fabgl::VK_q)));
            bitWrite(PS2cols[2], 1, (!Kbd->isVKDown(fabgl::VK_W)) & (!Kbd->isVKDown(fabgl::VK_w)));
            bitWrite(PS2cols[2], 2, (!Kbd->isVKDown(fabgl::VK_E)) & (!Kbd->isVKDown(fabgl::VK_e)));
            bitWrite(PS2cols[2], 3, (!Kbd->isVKDown(fabgl::VK_R)) & (!Kbd->isVKDown(fabgl::VK_r)));
            bitWrite(PS2cols[2], 4, (!Kbd->isVKDown(fabgl::VK_T)) & (!Kbd->isVKDown(fabgl::VK_t)));

            bitWrite(PS2cols[3], 0, (!Kbd->isVKDown(fabgl::VK_1)) & (!Kbd->isVKDown(fabgl::VK_EXCLAIM)) 
                                &   (!Kbd->isVKDown(VK_ESPECTRUM_GRAVEACCENT)) // Edit
                                & (j[1]));
            bitWrite(PS2cols[3], 1, (!Kbd->isVKDown(fabgl::VK_2)) & (!Kbd->isVKDown(fabgl::VK_AT)) 
                                &   (!Kbd->isVKDown(fabgl::VK_CAPSLOCK)) // Caps lock            
                                & (j[2])                                
                                );
            bitWrite(PS2cols[3], 2, (!Kbd->isVKDown(fabgl::VK_3)) & (!Kbd->isVKDown(fabgl::VK_HASH)) & (j[3]));
            bitWrite(PS2cols[3], 3, (!Kbd->isVKDown(fabgl::VK_4)) & (!Kbd->isVKDown(fabgl::VK_DOLLAR)) & (j[4]));
            bitWrite(PS2cols[3], 4, (!Kbd->isVKDown(fabgl::VK_5)) & (!Kbd->isVKDown(fabgl::VK_PERCENT)) & (j[5]));

            bitWrite(PS2cols[4], 0, (!Kbd->isVKDown(fabgl::VK_0)) & (!Kbd->isVKDown(fabgl::VK_RIGHTPAREN)) & (!Kbd->isVKDown(fabgl::VK_BACKSPACE)) & (j[0]));
            bitWrite(PS2cols[4], 1, !Kbd->isVKDown(fabgl::VK_9) & (!Kbd->isVKDown(fabgl::VK_LEFTPAREN)) & (j[9]));
            bitWrite(PS2cols[4], 2, (!Kbd->isVKDown(fabgl::VK_8)) & (!Kbd->isVKDown(fabgl::VK_ASTERISK)) & (j[8]));
            bitWrite(PS2cols[4], 3, (!Kbd->isVKDown(fabgl::VK_7)) & (!Kbd->isVKDown(fabgl::VK_AMPERSAND)) & (j[7]));
            bitWrite(PS2cols[4], 4, (!Kbd->isVKDown(fabgl::VK_6)) & (!Kbd->isVKDown(fabgl::VK_CARET)) & (j[6]));

            bitWrite(PS2cols[5], 0, (!Kbd->isVKDown(fabgl::VK_P)) & (!Kbd->isVKDown(fabgl::VK_p))
                                &   (!Kbd->isVKDown(fabgl::VK_QUOTE)) // Double quote            
                                );
            bitWrite(PS2cols[5], 1, (!Kbd->isVKDown(fabgl::VK_O)) & (!Kbd->isVKDown(fabgl::VK_o))
                                &   (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
                                );
            bitWrite(PS2cols[5], 2, (!Kbd->isVKDown(fabgl::VK_I)) & (!Kbd->isVKDown(fabgl::VK_i)));
            bitWrite(PS2cols[5], 3, (!Kbd->isVKDown(fabgl::VK_U)) & (!Kbd->isVKDown(fabgl::VK_u)));
            bitWrite(PS2cols[5], 4, (!Kbd->isVKDown(fabgl::VK_Y)) & (!Kbd->isVKDown(fabgl::VK_y)));

            bitWrite(PS2cols[6], 0, !Kbd->isVKDown(fabgl::VK_RETURN));
            bitWrite(PS2cols[6], 1, (!Kbd->isVKDown(fabgl::VK_L)) & (!Kbd->isVKDown(fabgl::VK_l)));
            bitWrite(PS2cols[6], 2, (!Kbd->isVKDown(fabgl::VK_K)) & (!Kbd->isVKDown(fabgl::VK_k)));
            bitWrite(PS2cols[6], 3, (!Kbd->isVKDown(fabgl::VK_J)) & (!Kbd->isVKDown(fabgl::VK_j)));
            bitWrite(PS2cols[6], 4, (!Kbd->isVKDown(fabgl::VK_H)) & (!Kbd->isVKDown(fabgl::VK_h)));

            bitWrite(PS2cols[7], 0, !Kbd->isVKDown(fabgl::VK_SPACE)
                            &   (!Kbd->isVKDown(fabgl::VK_ESCAPE)) // Break                             
            );
            bitWrite(PS2cols[7], 1, (!Kbd->isVKDown(fabgl::VK_LCTRL)) // SYMBOL SHIFT
                                &   (!Kbd->isVKDown(fabgl::VK_RCTRL))
                                &   (!Kbd->isVKDown(fabgl::VK_COMMA)) // Comma
                                &   (!Kbd->isVKDown(fabgl::VK_PERIOD)) // Period
                                &   (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
                                &   (!Kbd->isVKDown(fabgl::VK_QUOTE)) // Double quote
                                &   (!Kbd->isVKDown(VK_ESPECTRUM_TAB)) // Extended mode                                                                      
                                ); // SYMBOL SHIFT
            bitWrite(PS2cols[7], 2, (!Kbd->isVKDown(fabgl::VK_M)) & (!Kbd->isVKDown(fabgl::VK_m))
                                &   (!Kbd->isVKDown(fabgl::VK_PERIOD)) // Period
                                );
            bitWrite(PS2cols[7], 3, (!Kbd->isVKDown(fabgl::VK_N)) & (!Kbd->isVKDown(fabgl::VK_n))
                                &   (!Kbd->isVKDown(fabgl::VK_COMMA)) // Comma
                                );
            bitWrite(PS2cols[7], 4, (!Kbd->isVKDown(fabgl::VK_B)) & (!Kbd->isVKDown(fabgl::VK_b)));

        }

    }

    if (ZXKeyb::Exists) { // START - ZXKeyb Exists

        if (zxDelay > 0)
            zxDelay--;
        else
            // Process physical keyboard
            ZXKeyb::process();

        // Detect and process physical kbd menu key combinations
        // CS+SS+<1..0> -> F1..F10 Keys, CS+SS+Q -> F11, CS+SS+W -> F12, CS+SS+S -> Capture screen
        if ((!bitRead(ZXKeyb::ZXcols[0],0)) && (!bitRead(ZXKeyb::ZXcols[7],1))) {

            zxDelay = 15;

            int64_t osd_start = esp_timer_get_time();
            if (!bitRead(ZXKeyb::ZXcols[3],0)) {
                OSD::do_OSD(fabgl::VK_F1,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],1)) {
                OSD::do_OSD(fabgl::VK_F2,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],2)) {
                OSD::do_OSD(fabgl::VK_F3,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],3)) {
                OSD::do_OSD(fabgl::VK_F4,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[3],4)) {
                OSD::do_OSD(fabgl::VK_F5,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],4)) {
                OSD::do_OSD(fabgl::VK_F6,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],3)) {
                OSD::do_OSD(fabgl::VK_F7,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],2)) {
                OSD::do_OSD(fabgl::VK_F8,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],1)) {
                OSD::do_OSD(fabgl::VK_F9,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[4],0)) {
                OSD::do_OSD(fabgl::VK_F10,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],0)) {
                OSD::do_OSD(fabgl::VK_F11,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],1)) {
                OSD::do_OSD(fabgl::VK_F12,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],0)) { // P -> Pause
                OSD::do_OSD(fabgl::VK_PAUSE,0);
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],2)) { // I -> Info
                OSD::do_OSD(fabgl::VK_F1,true);
            } else
            if (!bitRead(ZXKeyb::ZXcols[2],4)) { // T -> Turbo
                OSD::do_OSD(fabgl::VK_F2,true);
            } else
            if (!bitRead(ZXKeyb::ZXcols[1],1)) { // S -> Screen capture
                CaptureToBmp();
            } else
            if (!bitRead(ZXKeyb::ZXcols[5],1)) { // O -> Poke
                OSD::pokeDialog();
            } else
            if (!bitRead(ZXKeyb::ZXcols[7],3)) { // N -> NMI
                Z80::triggerNMI();
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],1)) { // Z -> CenterH
                if (Config::CenterH > -16) Config::CenterH--;
                Config::save("CenterH");
                OSD::osdCenteredMsg("Horiz. center: " + to_string(Config::CenterH), LEVEL_INFO, 375);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],2)) { // X -> CenterH
                if (Config::CenterH < 16) Config::CenterH++;
                Config::save("CenterH");
                OSD::osdCenteredMsg("Horiz. center: " + to_string(Config::CenterH), LEVEL_INFO, 375);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],3)) { // C -> CenterV
                if (Config::CenterV > -16) Config::CenterV--;
                Config::save("CenterV");
                OSD::osdCenteredMsg("Vert. center: " + to_string(Config::CenterV), LEVEL_INFO, 375);
            } else
            if (!bitRead(ZXKeyb::ZXcols[0],4)) { // V -> CenterV
                if (Config::CenterV < 16) Config::CenterV++;
                Config::save("CenterV");
                OSD::osdCenteredMsg("Vert. center: " + to_string(Config::CenterV), LEVEL_INFO, 375);
            } else
                zxDelay = 0;

            if (zxDelay) {
                // Set all keys as not pressed
                for (uint8_t i = 0; i < 8; i++) ZXKeyb::ZXcols[i] = 0xbf;
                // Refresh border
                VIDEO::brdnextframe = true;                
                ESPectrum::ts_start += esp_timer_get_time() - osd_start;
                return;
            }
        }

        // Combine both keyboards
        for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
            Ports::port[rowidx] = PS2cols[rowidx] & ZXKeyb::ZXcols[rowidx];
        }
    } else {
        if (r) {
            for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
                Ports::port[rowidx] = PS2cols[rowidx];
            }
        }
    }
}

IRAM_ATTR void ESPectrum::BeeperGetSample() {

    // Beeper audiobuffer generation (oversample)
    uint32_t audbufpos = Z80Ops::is128 ? CPU::tstates / 19 : CPU::tstates >> 4;
    for (;audbufcnt < audbufpos; audbufcnt++) {
        audioBitBuf += lastaudioBit;
        if(++audioBitbufCount == audioSampleDivider) {
            overSamplebuf[audbufcntover++] = audioBitBuf;
            audioBitBuf = 0;
            audioBitbufCount = 0;
        }
    }

}

IRAM_ATTR void ESPectrum::AYGetSample() {
    // AY audiobuffer generation (oversample)
    uint32_t audbufpos = CPU::tstates / (Z80Ops::is128 ? 114 : 112);
    if (audbufpos > audbufcntAY) {
        chip0.gen_sound(audbufpos - audbufcntAY, audbufcntAY);
        chip1.gen_sound(audbufpos - audbufcntAY, audbufcntAY);
        audbufcntAY = audbufpos;
    }
}

//=======================================================================================
// MAIN LOOP
//=======================================================================================
void ESPectrum::loop() {    
  for(;;) {
    ts_start = time_us_64();

    // Send audioBuffer to pwmaudio
    audbufcnt = 0;
    audbufcntover = 0;
    audbufcntAY = 0;

    CPU::loop();

    // Process audio buffer
    faudbufcnt = audbufcnt;
    faudioBit = lastaudioBit;
    faudbufcntAY = audbufcntAY;
    if (ESP_delay) {
///        if (Config::real_player) {
///            pwm_audio_in_frame_started();
///        }

        ///xQueueSend(audioTaskQueue, &param, portMAX_DELAY);
        // Finish fill of beeper oversampled audio buffers
        for (;faudbufcnt < (samplesPerFrame * audioSampleDivider); ++faudbufcnt) {
            audioBitBuf += faudioBit;
            if(++audioBitbufCount == audioSampleDivider) {
                overSamplebuf[audbufcntover++] = audioBitBuf;
                audioBitBuf = 0;
                audioBitbufCount = 0; 
            }
        }
        if (AY_emu) {
            if (faudbufcntAY < samplesPerFrame) {
                chip0.gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);
                chip1.gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);
            }
            for (int i = 0; i < samplesPerFrame; ++i) {
                auto os = overSamplebuf[i] / audioSampleDivider;
                int l = os + chip0.SamplebufAY_L[i]+ chip1.SamplebufAY_L[i];
                int r = os + chip0.SamplebufAY_R[i]+ chip1.SamplebufAY_R[i];
                // Clamp
                audioBuffer_L[i] = l > 255 ? 255 : l;
                audioBuffer_R[i] = r > 255 ? 255 : r;
            }
        } else {
            for (int i = 0; i < samplesPerFrame; ++i) {
                auto v = overSamplebuf[i] / audioSampleDivider;
                audioBuffer_L[i] = v;
                audioBuffer_R[i] = v;
            }
        }
        pwm_audio_write(audioBuffer_L, audioBuffer_R, samplesPerFrame);
    }
    processKeyboard();
    // Update stats every 50 frames
    if (VIDEO::OSD && VIDEO::framecnt >= 50) {
        if (VIDEO::OSD & 0x04) {
            // printf("Vol. OSD out -> Framecnt: %d\n", VIDEO::framecnt);
            if (VIDEO::framecnt >= 100) {
                VIDEO::OSD &= 0xfb;
                if (VIDEO::OSD == 0) {
                    if (Config::aspect_16_9) 
                        VIDEO::Draw_OSD169 = VIDEO::MainScreen;
                    else
                        VIDEO::Draw_OSD43 = Z80Ops::isPentagon ? VIDEO::BottomBorder_Pentagon :  VIDEO::BottomBorder;
                    VIDEO::brdnextframe = true;
                }
            }
        }
        if ((VIDEO::OSD & 0x04) == 0) {
            if (VIDEO::OSD == 1 && Tape::tapeStatus==TAPE_LOADING) {
                snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1), " %-12s %04d/%04d ", Tape::tapeFileName.substr(0 + ESPectrum::TapeNameScroller, 12).c_str(), Tape::tapeCurBlock + 1, Tape::tapeNumBlocks);
                float percent = (float)((Tape::tapebufByteCount + Tape::tapePlayOffset) * 100) / (float)Tape::tapeFileSize;
                snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2), " %05.2f%% %07d%s%07d ", percent, Tape::tapebufByteCount + Tape::tapePlayOffset, "/" , Tape::tapeFileSize);
                if ((++ESPectrum::TapeNameScroller + 12) > Tape::tapeFileName.length()) ESPectrum::TapeNameScroller = 0;
                OSD::drawStats();

            } else if (VIDEO::OSD == 2) {
                snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1), "CPU: %05d / IDL: %05d ", (int)(ESPectrum::elapsed), (int)(ESPectrum::idle));
                snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2), "FPS:%6.2f / FND:%6.2f ", VIDEO::framecnt / (ESPectrum::totalseconds / 1000000), VIDEO::framecnt / (ESPectrum::totalsecondsnodelay / 1000000));
                OSD::drawStats();
            }
            totalseconds = 0;
            totalsecondsnodelay = 0;
            VIDEO::framecnt = 0;
        }
    }
    // Flashing flag change
    if (!(VIDEO::flash_ctr++ & 0x0f)) VIDEO::flashing ^= 0x80;

    elapsed = time_us_64() - ts_start;
    idle = target - elapsed - ESPoffset;

    totalsecondsnodelay += elapsed;

    if (ESP_delay == false) {
        totalseconds += elapsed;
        continue;
    }

        if (idle > 0)
            delayMicroseconds(idle);
        // else
        //     printf("Elapsed: %d\n",(int)elapsed);

        // Audio sync
        if (++sync_cnt & 0x10) {
///            ESPoffset = 128 - pwm_audio_rbstats();
            sync_cnt = 0;
        } 
    totalseconds += time_us_64() - ts_start;
 }
}

