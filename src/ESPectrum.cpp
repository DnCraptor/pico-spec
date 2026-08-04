/*
ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo
[dcrespo3d] https://github.com/EremusOne/ZX-ESPectrum-IDF

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

#include "AySound.h"
#include "SAASound.h"
#include "Subsystem.h"
#include "CPU.h"
#include "Config.h"
#include "ESPectrum.h"
#include "FileUtils.h"
#include "sdcard.h"
#include "MemESP.h"
#include "Buffer.h"
#include "LEDIndicators.h"
#include "OSDMain.h"
#include "Ports.h"
#include "Snapshot.h"
#include "Tape.h"
#include "Video.h"
#include "Z80_JLS/z80.h"
#include "messages.h"
#include "pwm_audio.h"
#include "wd1793.h"

#include "psram_spi.h"

#ifdef KBDUSB
#include "ps2kbd_mrmltr.h"
#else
#include "ps2.h"
#endif

#include "PinSerialData_595.h"
#include "Debug.h"
#if !PICO_RP2040
#include "DivMMC.h"
#include "IDE.h"
#include "MB02.h"
#endif
#include "Midi.h"
#include "MidiSynth.h"
#include "SoftSynth.h"
#include "ZiFi.h"
#include "ZiFiAT.h"
#include "BoardPins.h"
#include "RTC.h"
#include "Z80DMA.h"
#ifdef USE_GS
#include "GS/GS.h"
#endif

using namespace std;

extern size_t getFreeHeap(void);

#if !PICO_RP2040
extern "C" void hdmi_set_profi_ds80_mode(bool active, const uint32_t *palette16, const uint8_t *pair_lut);
extern "C" volatile bool profi_ds80_active;
#endif
#ifdef KBDUSB
// C-linkage query from hid_app.cpp; declared at file scope because a linkage
// specification ("C") is not permitted at block scope. ALL platforms — the
// factory-reset probe uses it on RP2040 (MURM/ZERO) KBDUSB builds too.
extern "C" bool usb_keyboard_mounted(void);
extern "C" size_t getLargestAllocatable(void);  // defined in OSDMain.cpp
#endif

//=======================================================================================
// KEYBOARD
//=======================================================================================
fabgl::PS2Controller ESPectrum::PS2Controller;

void joyPushData(fabgl::VirtualKey virtualKey, bool down) {
  fabgl::Keyboard *kbd = ESPectrum::PS2Controller.keyboard();
  if (kbd) {
    kbd->injectVirtualKey(virtualKey, down);
  }
}

// Last pwm_audio_write() duration — [NEG2] attribution (it can block waiting
// for DMA-ring space, which shows up as el−cpu time).
volatile uint32_t g_aud_write_us = 0;
volatile uint32_t g_kbd_us = 0;        // processKeyboard() per frame ([NEG2])
volatile uint32_t g_mix_us = 0;        // audio synth+mix block per frame ([NEG2])

volatile static uint32_t tickKbdRep = 0;
volatile static fabgl::VirtualKey last_key_pressed = fabgl::VirtualKey::VK_NONE;

fabgl::VirtualKey get_last_key_pressed(void) { return last_key_pressed; }

void close_all(void) {
#ifdef BUTTER_PSRAM_GPIO
  if (butter_psram_size()) {
    memset((void *)PSRAM_DATA, 0, butter_psram_size());
  }
  if (butter_psram_size()) {
    gpio_init(psram_pin);
    gpio_set_dir(psram_pin, GPIO_OUT);
    gpio_put(psram_pin, true);
  }
#endif
}

void kbdPushData(fabgl::VirtualKey virtualKey, bool down) {
  static bool ctrlPressed = false;
  static bool altPressed = false;
  static bool delPressed = false;
  if (virtualKey == fabgl::VirtualKey::VK_LCTRL ||
      virtualKey == fabgl::VirtualKey::VK_RCTRL)
    ctrlPressed = down;
  else if (virtualKey == fabgl::VirtualKey::VK_LALT ||
           virtualKey == fabgl::VirtualKey::VK_RALT)
    altPressed = down;
  else if (virtualKey == fabgl::VirtualKey::VK_DELETE ||
           virtualKey == fabgl::VirtualKey::VK_KP_PERIOD)
    delPressed = down;
  if (ctrlPressed && altPressed && delPressed) {
    // Single reboot choke point (logs the caller, handles the 595 latch).
    OSD::esp_hard_reset();
  }
  if (down) {
    if (ctrlPressed && virtualKey == fabgl::VirtualKey::VK_J) {
      Config::CursorAsJoy = !Config::CursorAsJoy;
    }
    if (last_key_pressed != virtualKey &&
        last_key_pressed != fabgl::VirtualKey::VK_MENU_UP &&
        last_key_pressed != fabgl::VirtualKey::VK_MENU_DOWN) {
      last_key_pressed = virtualKey;
      tickKbdRep = time_us_32();
    }
  } else {
    ///        switch (virtualKey) {
    ///            case fabgl::VirtualKey::VK_NUMLOCK   :
    ///            keyboard_toggle_led(PS2_LED_NUM_LOCK); break; case
    ///            fabgl::VirtualKey::VK_SCROLLLOCK:
    ///            keyboard_toggle_led(PS2_LED_SCROLL_LOCK); break; case
    ///            fabgl::VirtualKey::VK_CAPSLOCK  :
    ///            keyboard_toggle_led(PS2_LED_CAPS_LOCK); break;
    ///        }
    last_key_pressed = fabgl::VirtualKey::VK_NONE;
    tickKbdRep = 0;
  }
  fabgl::Keyboard *kbd = ESPectrum::PS2Controller.keyboard();
  if (kbd) {
    if (virtualKey != fabgl::VirtualKey::VK_NONE) {
      virtualKey = kbd->manageCAPSLOCK(virtualKey);
    }
    kbd->injectVirtualKey(virtualKey, down);
  }
}

void repeat_handler(void) {
  fabgl::VirtualKey v = last_key_pressed;
  if (v != fabgl::VirtualKey::VK_NONE) {
    if (tickKbdRep == 0) {
      if (v == fabgl::VirtualKey::VK_UP) {
        kbdPushData(fabgl::VirtualKey::VK_MENU_UP, true);
      } else if (v == fabgl::VirtualKey::VK_DOWN) {
        kbdPushData(fabgl::VirtualKey::VK_MENU_DOWN, true);
      }
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
uint8_t ESPectrum::audioBuffer_L[ESP_AUDIO_SAMPLES_PENTAGON] = {0};
uint8_t ESPectrum::audioBuffer_R[ESP_AUDIO_SAMPLES_PENTAGON] = {0};
uint8_t* ESPectrum::audioBufferCovoxL = nullptr;
uint8_t* ESPectrum::audioBufferCovoxR = nullptr;
uint8_t ESPectrum::overSamplebuf[ESP_AUDIO_SAMPLES_PENTAGON] = {0};
signed char ESPectrum::aud_volume = ESP_VOLUME_DEFAULT;
bool ESPectrum::vol_changed = false;
// signed char ESPectrum::aud_volume = ESP_VOLUME_MAX; // For .tap player test

uint32_t ESPectrum::audbufcnt = 0;
uint32_t ESPectrum::audbufcntover = 0;
uint32_t ESPectrum::faudbufcnt = 0;
uint32_t ESPectrum::audbufcntAY = 0;
uint32_t ESPectrum::faudbufcntAY = 0;
uint32_t ESPectrum::audbufcntCovox = 0;
uint32_t ESPectrum::faudbufcntCovox = 0;

#if !PICO_RP2040
uint8_t* ESPectrum::audioBufferPIT = nullptr;
uint8_t* ESPectrum::audioBufferMIDI_L = nullptr;
uint8_t* ESPectrum::audioBufferMIDI_R = nullptr;
uint32_t ESPectrum::audbufcntPIT = 0;
uint32_t ESPectrum::faudbufcntPIT = 0;
uint32_t ESPectrum::audbufcntSAA = 0;
uint32_t ESPectrum::faudbufcntSAA = 0;
bool ESPectrum::SAA_emu = false;
#endif

ESPectrum::FDDSound ESPectrum::fddSound = {{}, 0xACE1, 0, false, 0, 12};
const uint8_t ESPectrum::fdd_click_decay[12] = {48,36,27,20,15,11,8,6,4,3,2,1};
int ESPectrum::lastaudioBit = 0;
int ESPectrum::lastCovoxVal = 0;
int ESPectrum::lastCovoxValR = 0;
int ESPectrum::faudioBit = 0;
int ESPectrum::samplesPerFrame;
bool ESPectrum::AY_emu = false;
int ESPectrum::Audio_freq = 44000;
unsigned char ESPectrum::audioSampleDivider;
unsigned char ESPectrum::audioAYDivider;
unsigned char ESPectrum::audioCOVOXDivider;
unsigned char ESPectrum::audioOverSampleDivider;
/// QueueHandle_t audioTaskQueue;
/// TaskHandle_t ESPectrum::audioTaskHandle;
uint8_t *param;

//=======================================================================================
// TAPE OSD
//=======================================================================================

int ESPectrum::TapeNameScroller = 0;

//=======================================================================================
// BETADISK
//=======================================================================================

bool ESPectrum::trdos = false;
rvmWD1793 ESPectrum::fdd;
#if !PICO_RP2040
rvmWD1793 ESPectrum::mb02_fdd;
#endif

/// @brief  Mouse support
int32_t ESPectrum::mouseX = 0;
int32_t ESPectrum::mouseY = 0;
bool ESPectrum::mouseButtonL = 0;
bool ESPectrum::mouseButtonR = 0;
bool ESPectrum::mouseButtonM = 0;
uint8_t ESPectrum::mouseWheel = 0;
bool ESPectrum::mouseSeen = false;
int32_t ESPectrum::mouseDX = 0;
int32_t ESPectrum::mouseDY = 0;

bool ESPectrum::maxSpeed = false;

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
volatile bool ESPectrum::v_sync = false;
int64_t ESPectrum::ts_start;
int64_t ESPectrum::elapsed;
int64_t ESPectrum::idle;
uint8_t ESPectrum::multiplicator = 0;
uint32_t ESPectrum::lastBeeperTstates = 0;
uint32_t ESPectrum::accumulatorFP = 0;
uint32_t ESPectrum::tstatesPerSampleFP = 0;
uint32_t ESPectrum::beeperSampleAccum = 0;
uint32_t ESPectrum::beeperTstatesInSample = 0;

// Reciprocal LUT: recip[n] = (uint16_t)((1<<16)/n) for fast division in BeeperGetSample
static const uint16_t beeper_recip[256] = {
        0,    0,32768,21845,16384,13107,10922, 9362, 8192, 7281, 6553, 5957, 5461, 5041, 4681, 4369,
     4096, 3855, 3640, 3449, 3276, 3120, 2978, 2849, 2730, 2621, 2520, 2427, 2340, 2259, 2184, 2114,
     2048, 1985, 1927, 1872, 1820, 1771, 1724, 1680, 1638, 1598, 1560, 1524, 1489, 1456, 1424, 1394,
     1365, 1337, 1310, 1285, 1260, 1236, 1213, 1191, 1170, 1149, 1129, 1110, 1092, 1074, 1057, 1040,
     1024, 1008,  992,  978,  963,  949,  936,  923,  910,  897,  885,  873,  862,  851,  840,  829,
      819,  809,  799,  789,  780,  771,  762,  753,  744,  736,  728,  720,  712,  704,  697,  689,
      682,  675,  668,  661,  655,  648,  642,  636,  630,  624,  618,  612,  606,  601,  595,  590,
      585,  579,  574,  569,  564,  560,  555,  550,  546,  541,  537,  532,  528,  524,  520,  516,
      512,  508,  504,  500,  496,  492,  489,  485,  481,  478,  474,  471,  468,  464,  461,  458,
      455,  451,  448,  445,  442,  439,  436,  434,  431,  428,  425,  422,  420,  417,  414,  412,
      409,  407,  404,  402,  399,  397,  394,  392,  390,  387,  385,  383,  381,  378,  376,  374,
      372,  370,  368,  366,  364,  362,  360,  358,  356,  354,  352,  350,  348,  346,  344,  343,
      341,  339,  337,  336,  334,  332,  330,  329,  327,  326,  324,  322,  321,  319,  318,  316,
      315,  313,  312,  310,  309,  307,  306,  304,  303,  302,  300,  299,  297,  296,  295,  293,
      292,  291,  289,  288,  287,  286,  284,  283,  282,  281,  280,  278,  277,  276,  275,  274,
      273,  271,  270,  269,  268,  267,  266,  265,  264,  263,  262,  261,  260,  259,  258,  257
};

//=======================================================================================
// LOGGING / TESTING
//=======================================================================================

int ESPectrum::ESPtestvar = 0;
int ESPectrum::ESPtestvar1 = 0;
int ESPectrum::ESPtestvar2 = 0;

void ShowStartMsg() {

  fabgl::VirtualKeyItem Nextkey;

  VIDEO::vga.clear(zxColor(7, 0));

  OSD::drawOSD(false);

  VIDEO::vga.fillRect(Config::aspect_16_9 ? 60 : 40,
                      Config::aspect_16_9 ? 12 : 32, 240, 50, zxColor(0, 0));

  // Decode Logo in EBF8 format
  // Logo pixels are stored as ZX Spectrum palette indices (0-15)
  uint8_t *logo = (uint8_t *)ESPectrum_logo;
  int pos_x = Config::aspect_16_9 ? 86 : 66;
  int pos_y = Config::aspect_16_9 ? 23 : 43;
  int logo_w = (logo[5] << 8) + logo[4]; // Get Width
  int logo_h = (logo[7] << 8) + logo[6]; // Get Height
  logo += 8;                             // Skip header
  for (int i = 0; i < logo_h; i++)
    for (int n = 0; n < logo_w; n++) {
      uint8_t zxIdx = logo[n + (i * logo_w)];
      VIDEO::vga.dotFast(pos_x + n, pos_y + i, zxColor(zxIdx & 7, zxIdx >> 3));
    }

  OSD::osdAt(7, 1);
  VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 0));
  VIDEO::vga.print(Config::lang ? StartMsg[0] : StartMsg[1]);

  VIDEO::vga.setTextColor(zxColor(16, 0), zxColor(1, 0));
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

  for (int i = 20; i >= 0; i--) {
    OSD::osdAt(19, 1);
    sprintf(msg,
            Config::lang ? "Este mensaje se cerrar"
                           "\xA0"
                           " en %02d segundos"
                         : "This message will close in %02d seconds",
            i);
    VIDEO::vga.setTextColor(zxColor(7, 0), zxColor(1, 0));
    VIDEO::vga.print(msg);
    sleep_ms(1);
  }

  VIDEO::vga.clear(zxColor(7, 0));

  // Disable StartMsg
  Config::StartMsg = false;
  Config::save();
}

/**
void showMemInfo(const char* caption = "ZX-ESPectrum-IDF") {

#ifndef ESP32_SDL2_WRAPPER

multi_heap_info_t info;

heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); // internal
RAM, memory capable to store data or to create new task
printf("=========================================================================\n");
printf(" %s - Mem info:\n",caption);
printf("-------------------------------------------------------------------------\n");
printf("Total currently free in all non-continues blocks: %d\n",
info.total_free_bytes); printf("Minimum free ever: %d\n",
info.minimum_free_bytes); printf("Largest continues block to allocate big array:
%d\n", info.largest_free_block); printf("Heap caps get free size
(MALLOC_CAP_8BIT): %d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT));
printf("Heap caps get free size (MALLOC_CAP_32BIT): %d\n",
heap_caps_get_free_size(MALLOC_CAP_32BIT)); printf("Heap caps get free size
(MALLOC_CAP_INTERNAL): %d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
printf("=========================================================================\n\n");

// heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);

//
printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_8BIT);

//
printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_32BIT);

//
printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);

//
printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_DMA);

//
printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_EXEC);

//
printf("=========================================================================\n");
// heap_caps_print_heap_info(MALLOC_CAP_IRAM_8BIT);

//
printf("=========================================================================\n");
// heap_caps_dump_all();

//
printf("=========================================================================\n");

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
  /***
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
  */
}

//=======================================================================================
// SETUP
//=======================================================================================
extern int ram_pages, butter_pages, psram_pages, swap_pages;

static void assign_ram(int i) {
  // Not the raw chip size: Buffer::pageBudget* holds back what GS, DivMMC and the
  // Buffer arena need above the pages. Without that a large Murmuzavr page count ate
  // the whole chip and everything that belongs in PSRAM landed on the heap instead
  // (see the note in Buffer.h). Pages past the budget go to SD swap.
  static size_t butter_remains = Buffer::pageBudgetButter();
  static size_t spi_budget     = Buffer::pageBudgetSpi();
  static size_t butter_idx = 0;
  // Profi DS80 hires color attr pages (56/58) + CP/M's hot working page (61):
  // on SPI-PSRAM boards the Profi BIOS selects bankLatch=56..63 (portDFFD[2:0]=7)
  // causing sync()/swaps 50-100×/frame → ~1 FPS.  Fix: allocate as SRAM-backed
  // (POINTER) so sync() returns the SRAM pointer instantly without any SPI DMA.
  // Page 61 left UNPINNED so it stays in the evictable pool ({56,58,61}).
  // 1024K BIOS test stays correct (the earlier 160K was the buggy 32-bit
  // write_page truncating writes, not force_sram).
  // butter/QSPI-XIP boards are excluded: direct XIP pointers serve all pages
  // there, and per the 2026-07-07 A/B ([NEG2] cpu, HC idle + CP/M disk) the
  // pool/accessor layout brought no cpu gain once the platform-independent
  // costs were fixed — the real killers were AY/SAA synth, WD flash path and
  // per-poll FDC stepping, all fixed separately.
  //
  // +1 extra LRU pool buffer (60 = 16KB): HC's CP/M bank-switch trampoline
  // (OUT 0x7FFD/0xDFFD) ping-pongs read-only code banks; historically that
  // needed +2..3 forced SRAM buffers here (40/41, ~70 SPI reloads/frame with
  // fewer).  The ACCESSOR-MODE bank window (mem_desc_t::sync/MemESP::accessor*)
  // now serves those short bank visits per-byte over SPI without any 16KB
  // load — hw showed 85-100% of trampoline visits touch <128 bytes — so pages
  // 40/41 went back to the heap (+32KB).  Every page's data is preserved in
  // PSRAM on evict → no loss of the 1024K capacity.  Page indices must be
  // MID-range: forcing the TOP pages (62/63) corrupts Profi-1024K boot
  // (system data lives there); 60 is safe.  The initial index is otherwise
  // irrelevant — the LRU repurposes the buffer for whatever working set is hot.
  //
  // Pages 56 and 58 are the DS80 color-attribute pages (videoLatch=0 → 56,
  // videoLatch=1 → 58).  They must be permanently SRAM-resident (locked=true,
  // never in the evictable pool) so profi_clrmem is always a valid direct()
  // pointer regardless of what the Z80 banks into the 56-63 range via DFFD
  // group 7.  The previous approach (pin one, leave the other in the pool)
  // let the LRU evict the unpinned color page when the game (e.g. SINGLEWAR)
  // used DFFD group 7 for banking → profi_clrmem → null → mono/garbage frame.
  // With locked=true: no pin/unpin/preload needed in the render path → the
  // HDMI/VGA renderer never stalls on SPI DMA → no sync loss.
  //
  // Profi CP/M pool layout (RP2350, SPI-PSRAM only):
  //   Locked SRAM (never evicted): pages 56, 58 — DS80 colour-attribute data
  //   LRU pool (evictable):        pages 1, 2, 3, 60, 61 — CP/M working set
  //   All other pages:             SPI PSRAM, on demand via accessor window
  //
  // RP2040 (ZERO/MURM): DS80 not available; heap budget ~181KB with MEM_REMAIN=96KB
  // reserved for framebuffer. Use 3-page set {56,58,61} on RP2040.
#if PICO_RP2040
  bool force_sram_locked = false; // DS80 not available on RP2040
  bool force_sram = (Config::arch == "Profi")
                    && (i == 56 || i == 58 || i == 61)
                    && (butter_psram_size() == 0);
#else
  // butter/QSPI boards keep the direct-XIP-pointer scheme for ALL Profi pages
  // (56/58 included — the DS80 renderer reads them via the ds80_clr_sram
  // vblank snapshot, see Video.cpp).  An A/B test (2026-07-07, [NEG2] cpu on
  // HC idle + CP/M disk activity) showed the pool/accessor layout gave no cpu
  // gain on butter once the platform-independent costs were fixed (AY/SAA
  // silent paths, WD step path in SRAM, FDDStep fast exit, strcmp removal,
  // idle-window track loads) — the forced-SRAM pages only spent ~64 KB heap.
  bool force_sram_locked = (Config::arch == "Profi")
                           && (i == 56 || i == 58)
                           && (butter_psram_size() == 0);
  bool force_sram = (Config::arch == "Profi")
                    && (i == 61 || i == 60)
                    && (butter_psram_size() == 0);
#endif
  if (force_sram_locked) {
    MemESP::ram[i].assign_ram(new unsigned char[MEM_PG_SZ], i, true); // LOCKED: permanent SRAM
    ++ram_pages;
  } else if (force_sram) {
    MemESP::ram[i].assign_ram(new unsigned char[MEM_PG_SZ], i, false); // unlocked → in pool
    ++ram_pages;
  } else if (getFreeHeap() >= MEM_PG_SZ + MEM_REMAIN) {
    MemESP::ram[i].assign_ram(new unsigned char[MEM_PG_SZ], i, false);
    ++ram_pages;
  } else {
    if (butter_remains >= MEM_PG_SZ) {
      MemESP::ram[i].assign_ram(
          (uint8_t *)PSRAM_DATA + (butter_idx++) * MEM_PG_SZ, i, false);
      butter_remains -= MEM_PG_SZ;
      ++butter_pages;
    } else if (spi_budget >= ((size_t)MEM_PG_SZ * (i + 1))) {
      MemESP::ram[i].assign_vram(i, mem_type_t::PSRAM_SPI);
      ++psram_pages;
    } else {
      MemESP::ram[i].assign_vram(i, mem_type_t::SWAP);
      ++swap_pages;
    }
  }
}

void ESPectrum::setup() {
  //=======================================================================================
  // INIT FILESYSTEM
  //=======================================================================================
  Debug::log("setup: initFileSystem begin");
  FileUtils::initFileSystem();
  Debug::log("setup: initFileSystem done, fsMount=%d", FileUtils::fsMount);
  Debug::log2SD("setup: initFileSystem done, fsMount=%d", FileUtils::fsMount);

  mem_desc_t::reset();
  Ports::portAFF7 = 0;
  Ports::portDFFD = 0;
  Ports::serialMouseReset();
  //=======================================================================================
  // LOAD CONFIG
  //=======================================================================================
  Config::initHotkeys(); // fill hotkey defaults even without SD
  if (FileUtils::fsMount)
    Config::load();
#if !PICO_RP2040
  // Mount the ALF cartridge from SD (served lazily on demand like a wd1793 disk),
  // per Config::alfCartPath. Empty drive if none is set or the SD file is missing —
  // there is no built-in cart. Must run before ALF banking can read it.
  { extern void alfBindCart(); alfBindCart(); }
  // NOTE: the GM.DLS bank (MidiSynth::provisionAtBoot) is set up later — after
  // Buffer::initPools() so the butter PSRAM arena exists — but still before
  // VIDEO::Init() (flash erase must precede the live HDMI DMA over XIP).
#endif
  sdcard_set_led_blink(Config::sdLedBlink); // onboard LED blink on SD access
  VIDEO::loadCustomPalettes();
  Debug::log("setup: Config loaded");
  Debug::log2SD("setup: Config loaded, arch=%s romSet=%s", Config::arch.c_str(), Config::romSet.c_str());
  bool ext_ram_exist = butter_psram_size() >= (16 << 10) ||
                       psram_size() >= (16 << 10) || FileUtils::fsMount;
  Debug::log("setup: ext_ram_exist=%d, freeHeap=%u", ext_ram_exist, getFreeHeap());
  Debug::log2SD("setup: ext_ram_exist=%d, butter=%u spi=%u freeHeap=%u",
                ext_ram_exist, (unsigned)butter_psram_size(),
                (unsigned)psram_size(), (unsigned)getFreeHeap());

  // Set arch if there's no snapshot to load
  if (Config::ram_file == NO_RAM_FILE) {
    if (Config::pref_arch.substr(Config::pref_arch.length() - 1) == "R") {
      Config::pref_arch.pop_back();
      Config::save();
    } else {
      if (Config::pref_arch != "Last")
        Config::arch = Config::pref_arch;

      if (Config::arch == "48K") {
        if (Config::pref_romSet_48 != "Last")
          Config::romSet = Config::pref_romSet_48;
        else
          Config::romSet = Config::romSet48;
      }
#if !PICO_RP2040
      else if (Config::arch == "ALF") {
        Config::romSet = "ALF";
      }
#endif
      else if (Config::arch == "128K") {
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
      } else if (Config::arch == "Profi") {
        if (Config::pref_romSetProfi != "Last")
          Config::romSet = Config::pref_romSetProfi;
        else
          Config::romSet = Config::romSetProfi;
      } else {
        if (Config::pref_romSetPent != "Last")
          Config::romSet = Config::pref_romSetPent;
        else
          Config::romSet = Config::romSetPent;
      }
    }
  }

  // The live page count is derived from the persisted pick HERE and nowhere else: the arch
  // for this boot is final and nothing has sized a page strip yet. Murmuzavr extended RAM
  // is Pentagon hardware, and it is expensive — 2048 pages cost ~32 KB of SRAM bookkeeping
  // (descriptors + ram[]) plus their share of the PSRAM page budget. The #AFF7 plane latch
  // itself is not arch-gated (Ports.cpp), so without this clamp a "Profi + Murmuzavr
  // 32 MB" config OOM-panicked in setup(): Profi spends another ~80 KB of its own (incl.
  // the 16 KB DS80 colour SRAM), so the WD1793 track buffer no longer fit — with no way to
  // reach the menu and undo it. Config::mem_pg_cnt keeps the pick, so coming back to a
  // Pentagon does not need it re-entered.
  MEM_PG_CNT = Config::mem_pg_cnt;
  if (MEM_PG_CNT > 64 && !(Config::arch == "Pentagon" || Config::arch == "P512" ||
                           Config::arch == "P1024")) {
    Debug::log("setup: Murmuzavr %u pages dropped — %s is not Pentagon",
               (unsigned)MEM_PG_CNT, Config::arch.c_str());
    Debug::log2SD("setup: MEM_PG_CNT %u -> 64 (arch=%s, Murmuzavr is Pentagon-only)",
                  (unsigned)MEM_PG_CNT, Config::arch.c_str());
    MEM_PG_CNT = 64;
  }

  //=======================================================================================
  // INIT PS/2 KEYBOARD
  //=======================================================================================

  // Set Scroll Lock Led as current CursorAsJoy value
  PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);

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
  Debug::log("setup: MEMORY SETUP begin, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: MEMORY SETUP begin, freeHeap=%u", (unsigned)getFreeHeap());
  if (ext_ram_exist) {
    mem_desc_t *temp = MemESP::ram;
    MemESP::ram = new mem_desc_t[MEM_PG_CNT + 2];
    memcpy(MemESP::ram, temp, sizeof(mem_desc_t) * 8);
    Debug::log("setup: after memcpy: ram5=%p ram7=%p", MemESP::ram[5].direct(), MemESP::ram[7].direct());
#if PICO_RP2040
    // RP2040: page 0 goes to external backing to save heap for framebuffer.
    // Actual data location at access time is chosen by psram_size() check in
    // mem_desc_t ops, so label the page with its real backing for honest stats.
    if (psram_size() >= MEM_PG_SZ) {
        MemESP::ram[0].assign_vram(0, mem_type_t::PSRAM_SPI);
        ++psram_pages;
    } else {
        MemESP::ram[0].assign_vram(0, mem_type_t::SWAP);
        ++swap_pages;
    }
    MemESP::ram[1].assign_ram(new unsigned char[MEM_PG_SZ], 1, false);
    MemESP::ram[2].assign_ram(new unsigned char[MEM_PG_SZ], 2, false);
    MemESP::ram[3].assign_ram(new unsigned char[MEM_PG_SZ], 3, false);
    ram_pages += 3;
    // pages 4 and 6 — assign_ram falls back to butter/PSRAM/swap when heap
    // is below MEM_REMAIN (reserved for framebuffer). 5 and 7 are static
    // SRAM (pages57). On RP2040 the static pages46 buffer doesn't exist.
    assign_ram(4);
    assign_ram(6);
#else
    // RP2350: pages 0-3 pre-bound to static `pages0123`, pages 4,6 to
    // `pages46`, pages 5,7 to `pages57` (MemESP.cpp). Skip assign_ram so we
    // don't overwrite the static buffers. Pages 5,7 historically not
    // counted in ram_pages — keep that behaviour.
    ram_pages += 4;
    ram_pages += 2;
    // Add pages 1,2,3 to the LRU pool using their existing static SRAM buffers.
    // Without this the pool is empty on SPI-PSRAM and SWAP-only boards (pages 8+
    // are assign_vram/PSRAM_SPI, never pooled), so _sync() can never find an
    // eviction victim → MB-02 SRAM sync(0) returns NULL → applyMapping() falls
    // back to ROM → BS-DOS crashes on every page switch.
    MemESP::ram[1].assign_ram(MemESP::ram[1].direct(), 1, false);
    MemESP::ram[2].assign_ram(MemESP::ram[2].direct(), 2, false);
    MemESP::ram[3].assign_ram(MemESP::ram[3].direct(), 3, false);
#endif
    Debug::log("setup: ext_ram: pages 0-7 done, freeHeap=%u", getFreeHeap());
    for (size_t i = 8; i < (MEM_PG_CNT + 2); ++i) {
      assign_ram(i);
    }
    Debug::log("setup: ext_ram: all pages done, freeHeap=%u", getFreeHeap());
    Debug::log("setup: ram5=%p ram7=%p diff=%d", MemESP::ram[5].direct(), MemESP::ram[7].direct(),
               (int)((uint8_t*)MemESP::ram[7].direct() - (uint8_t*)MemESP::ram[5].direct()));
  } else {
    Debug::log("setup: no ext_ram path, freeHeap=%u", getFreeHeap());
#if PICO_RP2350
    // RP2350: pages 0-3 pre-bound to static `pages0123`, pages 4,6 to
    // `pages46` (MemESP.cpp). Pages 5,7 historically not counted.
    ram_pages += 4;
    ram_pages += 2;
#else
    // RP2040: page 0 not supported without virtual memory; pages 1-3 essential.
    MemESP::ram[1].assign_ram(new unsigned char[MEM_PG_SZ], 1, false);
    MemESP::ram[2].assign_ram(new unsigned char[MEM_PG_SZ], 2, false);
    MemESP::ram[3].assign_ram(new unsigned char[MEM_PG_SZ], 3, false);
    ram_pages += 3;
    // Pages 4,6 only if enough heap remains for framebuffer (MEM_REMAIN).
    // No ext_ram and 16col mode is disabled on RP2040, so if heap is too
    // tight these pages stay unbacked (machine effectively 48K).
    if (getFreeHeap() >= MEM_PG_SZ + MEM_REMAIN) {
        MemESP::ram[4].assign_ram(new unsigned char[MEM_PG_SZ], 4, false);
        ++ram_pages;
    }
    if (getFreeHeap() >= MEM_PG_SZ + MEM_REMAIN) {
        MemESP::ram[6].assign_ram(new unsigned char[MEM_PG_SZ], 6, false);
        ++ram_pages;
    }
#endif
    Debug::log("setup: no ext_ram: pages done, freeHeap=%u", getFreeHeap());
  }
  // Initialise every SRAM/butter-backed ZX RAM page to a defined state (0).
  // Without this the emulated RAM starts with build-dependent garbage:
  //  - RP2350 pages 0-7 live in the .ram_128k linker section, declared (NOLOAD),
  //    so the C runtime never zeroes them.
  //  - heap pages from `new unsigned char[]` are not zero-initialised either.
  // The leftover content is stable across power cycles for a given binary but
  // differs between builds (layout/boot writes change), which made halt2int's
  // floating-bus probe return a flaky Early/Unknown verdict that flipped from
  // build to build with no source change.  A defined power-on state (matching a
  // real machine after the ROM clears the screen) makes the result reproducible.
  // PSRAM_SPI/SWAP pages are skipped (extended pages, not used by 48K; butter is
  // already cleared at boot).  Runs once at cold setup, before romset/snapshot load.
  for (size_t i = 0; i < MEM_PG_CNT; ++i) {
    if (MemESP::ram[i].memType() == mem_type_t::POINTER) {
      uint8_t *p = MemESP::ram[i].direct();
      if (p && p >= (uint8_t *)0x11000000) memset(p, 0, MEM_PG_SZ);
    }
  }
  Debug::log("setup: ZX RAM pages zeroed, freeHeap=%u", getFreeHeap());
  // Load romset
  Debug::log("setup: requestMachine begin, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: requestMachine begin arch=%s romSet=%s freeHeap=%u",
                Config::arch.c_str(), Config::romSet.c_str(), (unsigned)getFreeHeap());
  Config::requestMachine(Config::arch, Config::romSet);
  Debug::log("setup: requestMachine done, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: requestMachine done, freeHeap=%u", (unsigned)getFreeHeap());

  MemESP::page0ram = 0;
  ESPectrum::trdos = false;
  // Pentagon+Gluk: boot with Gluk ROM to install service monitor at 0xDB00
  // Profi: boot with SYS ROM (bank0), SYSEN=true — per ZXMAK2 BusReset() spec
  if (Config::romSet == "128Kpg" || Config::romSet == "128Kbg")
      MemESP::romInUse = 3;
  else
      MemESP::romInUse = 0;
  if (Config::arch == "Profi") ESPectrum::trdos = true; // SYSEN
  // Profi CP/M: clear physical page 1 (ram[1]) so BDOS.BIN loading via the INI
  // driver lands in a known-zero state. The BOOTFDD self-install clears ram[5/6/58]
  // but deliberately skips ram[1] (it holds BOOTFDD.COM from TR-DOS). On butter-PSRAM
  // systems, stale data from a previous corrupted INI overflow (which can overwrite
  // page 1 if INTRQ never fires) causes page 1 code to be garbage on the next boot,
  // breaking the BDOS loading. Zeroing on every reset costs ~10ms but guarantees
  // that page 1 starts clean — the BIOS BDOS load then populates it correctly.
  if (Config::arch == "Profi") {
    uint8_t *p1 = MemESP::ram[1].direct();
    if (p1 && p1 >= (uint8_t*)0x11000000) {
      memset(p1, 0, 16384);
      Debug::log("[setup] Profi: cleared ram[1] (page1 for CP/M BDOS)");
    }
  }
  MemESP::bankLatch = 0;
  MemESP::videoLatch = 0;
  MemESP::romLatch = 0;
  MemESP::newSRAM = false;
  Debug::log("[setup] arch=%s romInUse=%d", Config::arch.c_str(), MemESP::romInUse);

  MemESP::ramCurrent[0] = MemESP::rom[MemESP::romInUse].direct();
  MemESP::ramCurrent[1] = MemESP::ram[5].direct();
  MemESP::ramCurrent[2] = MemESP::ram[2].sync(2);
  MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(3);

  // Pre-load STS 7.5 into RAM bank 7 at offset 0x1B00 (= 0xDB00 - 0xC000)
  // Gluk ROM writes 0x47 to port 0x7FFD selecting bank 7 for page 3, then
  // installs its service monitor there. We pre-populate it so STS is ready.
  // ALASM (0x8000) is loaded by Gluk after boot — ram[2] must stay clean.
  if (Config::romSet == "128Kpg" || Config::romSet == "128Kbg") {
      uint8_t* page3 = MemESP::ram[7].direct();
      if (page3) memcpy(page3 + 0x1B00, gb_rom_sts75, sizeof(gb_rom_sts75));
  }

  MemESP::ramContended[0] = false;
  MemESP::ramContended[1] = Config::arch == "P1024" || Config::arch == "P512" ||
                                    Config::arch == "Pentagon" || Config::arch == "Profi"
                                ? false
                                : true;
  MemESP::ramContended[2] = false;
  MemESP::ramContended[3] = false;

  // if (Config::arch == "48K") MemESP::pagingLock = 1; else MemESP::pagingLock
  // = 0;
  MemESP::pagingLock = Config::arch == "48K" ? 1 : 0;

  ///    if (Config::slog_on) showMemInfo("RAM Initialized");

#if !PICO_RP2040
  // Always init DivMMC (load ROM) so it's ready if user enables from OSD later
  Debug::log2SD("setup: DivMMC::init begin, freeHeap=%u", (unsigned)getFreeHeap());
  DivMMC::init();
  Debug::log2SD("setup: DivMMC::init done, freeHeap=%u", (unsigned)getFreeHeap());
  // IDE/HDD (NEMO/PROFI schemes) — independent of DivMMC.
  IDE::init();
  // MC146818 RTC (Pentagon/Profi Mr Gluk TimeKeeper) — set register defaults.
  RTC::init();
  // MB-02+ disk interface (allocates SRAM in butter PSRAM after DivMMC)
  Debug::log2SD("setup: MB02::init begin");
  MB02::init();
  Debug::log2SD("setup: MB02::init done, enabled=%d freeHeap=%u",
                (int)MB02::enabled, (unsigned)getFreeHeap());
  // Z-Controller raw SD on ports 0x77/0x57 (mutually exclusive with esxDOS)
  if (Config::zcontroller && !Config::esxdos && !Config::mb02) {
    Debug::log2SD("setup: DivMMC::zc_init");
    DivMMC::zc_init();
  }
#endif
  // Tiered buffer pools: carve PSRAM/SD-swap arenas from whatever the existing
  // consumers above (MemESP/Profi pages, DivMMC) have NOT claimed, reserving GS's
  // sample-RAM region via GS::configuredRamBytes(). Must run after those so the
  // boundaries are final, and BEFORE GS::init so GS's work/ring buffers can draw
  // from the butter arena. See Buffer.cpp.
  Buffer::initPools();

#ifdef USE_GS
  // AFTER initPools: GS::init allocates its work RAM + DAC rings from the butter
  // arena (NEED_POINTER|PREFER_PSRAM), freeing ~32 KB SRAM on PSRAM boards. The
  // sample-RAM region it claims at the PSRAM top was already excluded from the arena.
  if (Config::gs_enabled) {
    uint32_t gs_ram = GS::configuredRamBytes();
    Debug::log2SD("setup: GS::init ram=%u", (unsigned)gs_ram);
    GS::init(gs_ram);
    Debug::log2SD("setup: GS::init done, freeHeap=%u", (unsigned)getFreeHeap());
  }
#endif

#if !PICO_RP2040
  // GM.DLS MIDI bank: load into butter PSRAM (preferred) or provision the flash
  // partition from SD. MUST be here — AFTER initPools() (the PSRAM arena is now
  // final) and BEFORE VIDEO::Init(): a flash erase disables XIP for the whole QMI
  // (flash CS0 + PSRAM CS1), so once the HDMI engine streams the framebuffer out of
  // XIP-PSRAM it would stall the bus and hang. Still single core (core1 launches
  // later in main()). No-op unless GM.DLS mode (Config::midi==4) is selected.
  if (FileUtils::fsMount) MidiSynth::provisionAtBoot();
#endif

  //=======================================================================================
  // VIDEO
  //=======================================================================================
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
  {
    extern bool SELECT_VGA;
    extern uint8_t linkVGA01;
    extern uint8_t video_driver;
    if (video_driver == 0) {
        #if defined(ZERO) || defined(ZERO2) || defined(PICO_DV)
            SELECT_VGA = linkVGA01 == 0x1F;
        #else
            SELECT_VGA = (linkVGA01 == 0) || (linkVGA01 == 0x1F);
        #endif
    } else {
        SELECT_VGA = video_driver == 1;
    }
  }
#endif
  Debug::log("setup: VIDEO::Init begin, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: VIDEO::Init begin, freeHeap=%u", getFreeHeap());
  VIDEO::Init();
  Debug::log("setup: VIDEO::Init done, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: VIDEO::Init done, freeHeap=%u", getFreeHeap());
  VIDEO::Reset();
  Debug::log("setup: VIDEO::Reset done");
  Debug::log2SD("setup: VIDEO::Reset done");

  // if (Config::StartMsg) ShowStartMsg(); // Show welcome message

  Debug::log("setup: AUDIO section begin, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: AUDIO section begin, freeHeap=%u", (unsigned)getFreeHeap());
  //=======================================================================================
  // AUDIO
  //=======================================================================================
  // Set samples per frame and AY_emu flag depending on arch
    AY_emu = Config::AY48;
#if !PICO_RP2040
    SAA_emu = Config::SAA1099;
    Midi::enabled = Config::midi;
#if !PICO_RP2040 && defined(MIDI_TX_PIN)
    // Yield the MIDI TX pin to ZiFi when it owns it (boot-time, after Config).
    if (BoardPins::zifiOwnsPin(MIDI_TX_PIN)) Midi::enabled = 0;
#endif
    if (Midi::enabled) {
        Debug::log2SD("setup: Midi::init mode=%d", (int)Midi::enabled);
        Midi::init();
    }
    if (Config::dma_mode) {
        Debug::log2SD("setup: Z80DMA::reset");
        Z80DMA::reset();
    }
    // Profi forces ~80 KB of SRAM pages and OOMs at VIDEO::Init if the NIC's heap
    // rings (~12 KB) are also up — so never bring ZiFi up on Profi, regardless of a
    // stale zifi_enabled. (The menu also turns the NIC off when switching to Profi.)
    // The NIC also requires WiFi to be enabled — it is purely the guest-port
    // emulation layer on top of WiFi, never a standalone networking switch.
    ZiFi::enabled = Config::zifi_enabled && Config::wifi_enabled && Config::arch != "Profi";
    if (ZiFi::enabled)
        ZiFi::init();
#endif

  if (Config::arch == "48K" || Config::arch == "Profi") {
    samplesPerFrame = ESP_AUDIO_SAMPLES_48;
    audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_48;
    audioAYDivider = ESP_AUDIO_AY_DIV_48;
    audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;

    Audio_freq = ESP_AUDIO_FREQ_48;
    tstatesPerSampleFP = (TSTATES_PER_FRAME_48 << 8) / ESP_AUDIO_SAMPLES_48;
  } else if (Config::arch == "128K" || Config::arch == "ALF") {
    samplesPerFrame = ESP_AUDIO_SAMPLES_128;
    audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_128;
    audioAYDivider = ESP_AUDIO_AY_DIV_128;
    audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_128;
    Audio_freq = ESP_AUDIO_FREQ_128;
    tstatesPerSampleFP = (TSTATES_PER_FRAME_128 << 8) / ESP_AUDIO_SAMPLES_128;
  } else { /// if (Config::arch == "P512" || Config::arch == "Pentagon") {
    samplesPerFrame = ESP_AUDIO_SAMPLES_PENTAGON;
    audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_PENTAGON;
    audioAYDivider = ESP_AUDIO_AY_DIV_PENTAGON;
    audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_PENTAGON;
    Audio_freq = ESP_AUDIO_FREQ_PENTAGON;
    tstatesPerSampleFP = (TSTATES_PER_FRAME_PENTAGON << 8) / ESP_AUDIO_SAMPLES_PENTAGON;
  }

  audioCOVOXDivider = audioAYDivider;

  Debug::log("setup: init_sound begin");
  Debug::log2SD("setup: init_sound begin arch=%s freq=%d", Config::arch.c_str(), (int)Audio_freq);
  init_sound();
  pcm_setup(Audio_freq);
  Debug::log("setup: audio init done, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: audio init done, freeHeap=%u", (unsigned)getFreeHeap());

  if (Config::tape_player) {
    AY_emu = false; // Disable AY emulation if tape player mode is set
#if !PICO_RP2040
    SAA_emu = false;
#endif
    ESPectrum::aud_volume = ESP_VOLUME_MAX;
  } else
    ESPectrum::aud_volume = Config::aud_volume;

  pwm_audio_set_volume(aud_volume);

  // AY Sound — chip0 is always-on (required by 128K/Pentagon).
  // chip1 (TurboSound second AY) is allocated on demand by TurboSubsys.
  Debug::log("setup: AY init begin");
  Debug::log2SD("setup: AY init begin");
  chip0.init();
  chip0.set_sound_format(Audio_freq, 1, 8);
  chip0.set_stereo(AYEMU_MONO, NULL);
  chip0.reset();
  Debug::log("setup: AY init done");
  Debug::log2SD("setup: AY init done");

  // Optional audio subsystems: alloc memory only if their Config flag is set.
  // MB02 / DivMMC were already initialized above (they need MemESP ready) —
  // sync our subsystem flags to reflect that state.
  // tape_player mode disables AY/SAA emulation regardless of Config (see below).
  Debug::log2SD("setup: Subsystems::request begin tape_player=%d turbo=%d covox=%d SAA=%d midi=%d freeHeap=%u",
                (int)Config::tape_player, (int)Config::turbosound, (int)Config::covox,
#if !PICO_RP2040
                (int)Config::SAA1099, (int)Config::midi,
#else
                0, 0,
#endif
                (unsigned)getFreeHeap());
  TurboSubsys::request(!Config::tape_player && Config::turbosound != 0);
  CovoxSubsys::request(Config::covox != 0 || Config::soundriveEnabled());
#if !PICO_RP2040
  PitSubsys::request(Z80Ops::isByte);
  SaaSubsys::request(!Config::tape_player && Config::SAA1099);
  MidiSubsys::request(Config::midi != 0);
  DmaSubsys::request(Config::dma_mode != 0);
  Mb02Subsys::syncFromState();
  DivMmcSubsys::syncFromState();
  IdeSubsys::syncFromState();   // IDE::init() already ran above
#endif
  Debug::log2SD("setup: Subsystems::applyPending begin, freeHeap=%u", (unsigned)getFreeHeap());
  Subsystems::applyPending();
  Debug::log2SD("setup: Subsystems::applyPending done, freeHeap=%u", (unsigned)getFreeHeap());

  // Init tape
  Debug::log("setup: Tape init begin");
  Debug::log2SD("setup: Tape init begin");
  Tape::Init();
  Tape::tapeFileName = "none";
  Tape::tapeStatus = TAPE_STOPPED;
  Tape::SaveStatus = SAVE_STOPPED;
  Tape::romLoading = false;
  Debug::log("setup: Tape init done");
  Debug::log2SD("setup: Tape init done");

  // Init CPU
  Debug::log("setup: Z80 create begin");
  Debug::log2SD("setup: Z80 create begin");
  Z80::create();

  // Set Ports starting values
  for (int i = 0; i < 128; i++)
    Ports::port[i] = 0xBF;
  if (Config::joystick == JOY_KEMPSTON)
    Ports::port[Config::kempstonPort] = 0; // Kempston
  if (Config::joystick == JOY_FULLER)
    Ports::port[0x7f] = 0xff; // Fuller

  // Init disk controller
  Debug::log("setup: WD1793 reset begin");
  Debug::log2SD("setup: WD1793 reset begin");
#if !PICO_RP2040
  // Primary Betadisk drive always needs its track buffer. The MB-02 drive's
  // buffer is allocated/freed on demand via Mb02Subsys (saves ~12.5 KB SRAM
  // while MB-02 is disabled, which is the default).
  rvmWD1793AllocTrackBuf(&fdd);
#endif
  rvmWD1793Reset(&fdd);
  Debug::log("setup: WD1793 reset done");
  Debug::log2SD("setup: WD1793 reset done");

  // Reset cpu
  Debug::log("setup: CPU reset begin");
  Debug::log2SD("setup: CPU reset begin");
  CPU::reset();
  VIDEO::Reset(); // Re-run after CPU::reset() so Z80Ops flags are correct

#if !PICO_RP2040
  // KR580VI53 (8253 PIT) — reset to silent state
  if (Z80Ops::isByte && audioBufferPIT) {
    memset(audioBufferPIT, 0, ESP_AUDIO_SAMPLES_PENTAGON);
    memset(Ports::pitChannels, 0, sizeof(Ports::pitChannels));
  }
#endif

  Debug::log("setup: CPU reset done");
  Debug::log2SD("setup: CPU reset done");
  Debug::log("setup: Config::loadDiskMounts begin");
  Debug::log2SD("setup: Config::loadDiskMounts begin, freeHeap=%u", (unsigned)getFreeHeap());
  if (FileUtils::fsMount) {
    Config::loadDiskMounts();
  }
  Debug::log("setup: Config::loadDiskMounts done");
  Debug::log2SD("setup: Config::loadDiskMounts done, freeHeap=%u", (unsigned)getFreeHeap());

#if !PICO_RP2040
  // Re-reset MB-02 after disk mounts so boot EPROM starts with disks already inserted
  if (MB02::enabled && mb02_fdd.disk[0]) {
    Debug::log2SD("setup: MB02::reset (post-mount)");
    MB02::reset();
    Z80::reset();
  }
#endif

  // Profi first boot: run the full reset(0) path so the machine starts in the
  // SYS ROM (bank0) Service menu — identical to the F11 hard-reset / Alt-F11
  // "Service ROM" behaviour. The inline paging above sets the right banks but
  // not all the per-reset state, which left first boot dropping into 48K.
  // Skip when a snapshot is queued (LoadSnapshot below sets up its own state).
  if (Config::arch == "Profi" && Config::ram_file == NO_RAM_FILE) {
    ESPectrum::reset(0);
  }

  // Load snapshot if present in Config::
  Debug::log("setup: ram_file='%s'", Config::ram_file.c_str());
  Debug::log2SD("setup: ram_file='%s'", Config::ram_file.c_str());
  if (Config::ram_file != NO_RAM_FILE) {
    if (FileUtils::fsMount) {
      Debug::log2SD("setup: LoadSnapshot begin");
      LoadSnapshot(Config::ram_file, "", "");
      Debug::log2SD("setup: LoadSnapshot done");
    }
    Config::last_ram_file = Config::ram_file;
    Config::ram_file = NO_RAM_FILE;
    if (FileUtils::fsMount)
      Config::save();
  }

  // Re-mount the tape remembered from a previous session (NVS) so it is present
  // at cold boot, the same way disk mounts are restored by loadDiskMounts above.
  Tape::LoadRemembered();

  Debug::log("setup: COMPLETE, freeHeap=%u", getFreeHeap());
  Debug::log2SD("setup: COMPLETE, freeHeap=%u", (unsigned)getFreeHeap());

  // Create loop function as task: it doesn't seem better than calling from
  // main.cpp and increases RAM consumption (4096 bytes for stack).
  // xTaskCreatePinnedToCore(&ESPectrum::loop, "loopTask", 4096, NULL, 1,
  // &loopTaskHandle, 0);
}

//=======================================================================================
// RESET
//=======================================================================================
void ESPectrum::reset() {
  // Pentagon+Gluk: boot with Gluk ROM so it installs service monitor at 0xDB00
  // This matches real Pentagon hardware where Gluk always boots first
  uint8_t romInUse = 0;
  if (Config::romSet == "128Kpg" || Config::romSet == "128Kbg")
      romInUse = 3;
  ESPectrum::reset(romInUse);
}

void ESPectrum::reset(uint8_t romInUse) {
  // Ports
  for (int i = 0; i < 128; i++)
    Ports::port[i] = 0xBF;
  if (Config::joystick == JOY_KEMPSTON)
    Ports::port[Config::kempstonPort] = 0; // Kempston
  else if (Config::joystick == JOY_FULLER)
    Ports::port[0x7f] = 0xff; // Fuller
  Ports::portAFF7 = 0;
#if !PICO_RP2040
  // If DS80 packed-pair HDMI mode was active before reset, disable it before
  // clearing DFFD — otherwise HDMI ISR keeps expanding bytes as pairs and the
  // normal-mode framebuffer renders as vertical scanline garbage.
  if (Ports::portDFFD & 0x80) {
    hdmi_set_profi_ds80_mode(false, nullptr, nullptr);
    Graphics8BitPalette::ds80_active = false; // leaving DS80 → raw ZX indices again
    // Clear framebuffer immediately after switching HDMI back to standard mode.
    // DS80 packed-pair slot values (0..254) in the framebuffer look like garbage
    // when re-read through the restored standard conv_color table.
    // VIDEO::Reset() won't clear it (profi_ds80_active is now false), so we
    // must do it here.  Fill with 0 = palette index BLACK in both standard and
    // DS80 modes.
    if (VIDEO::vga.frameBuffer) {
      for (int _y = 0; _y < (int)VIDEO::vga.yres; _y++)
        if (VIDEO::vga.frameBuffer[_y]) memset(VIDEO::vga.frameBuffer[_y], 0, VIDEO::vga.xres);
    }
    Debug::log("[RESET] DS80 off + FB cleared");
  }
#endif
  Ports::portDFFD = 0;
  Ports::serialMouseReset();
  // Profi SYSEN: boot into SYS ROM (bank0) with trdos=true to protect page0
  ESPectrum::trdos = (Config::arch == "Profi" && romInUse == 0);

  Debug::log("[reset] arch=%s romInUse=%d trdos=%d", Config::arch.c_str(), romInUse, (int)ESPectrum::trdos);
#if !PICO_RP2040 && FDD_PORT_TRACE
  // g_fdcCmdCount gates the [WR→DIR]/[WR→RSTVEC]/[WR→CBIOS] write-count caps
  // in MemESP.h (only trace once real disk activity starts, skipping the
  // SYS-ROM self-test's memtest/buffer-clear noise). It's a plain static, so
  // without this it carries over across a soft reset within the same power
  // session — on a SECOND boot attempt it's already >0 from the FIRST one,
  // so the gate does nothing and the caps burn on self-test again. Reset it
  // here so every [RESET] gets a fresh budget for its own disk activity.
  extern uint32_t g_fdcCmdCount;
  g_fdcCmdCount = 0;
#endif
  // Memory
  MemESP::page0ram = 0;
  MemESP::romInUse = romInUse;
  MemESP::bankLatch = 0;
  MemESP::videoLatch = 0;
  MemESP::romLatch = 0;
  MemESP::newSRAM = false;

  MemESP::ramCurrent[0] = MemESP::rom[romInUse].direct();
  MemESP::ramCurrent[1] = MemESP::ram[5].direct();
  MemESP::ramCurrent[2] = MemESP::ram[2].sync(2);
  MemESP::ramCurrent[3] = MemESP::ram[0].sync(3);

  // Re-load STS 7.5 into RAM bank 7 at offset 0x1B00 (= 0xDB00 - 0xC000)
  if (Config::romSet == "128Kpg" || Config::romSet == "128Kbg") {
      uint8_t* page3 = MemESP::ram[7].direct();
      if (page3) memcpy(page3 + 0x1B00, gb_rom_sts75, sizeof(gb_rom_sts75));
  }

  MemESP::ramContended[0] = false;
  MemESP::ramContended[1] = Config::arch == "P1024" || Config::arch == "P512" ||
                                    Config::arch == "Pentagon" || Config::arch == "Profi"
                                ? false
                                : true;
  MemESP::ramContended[2] = false;
  MemESP::ramContended[3] = false;

  MemESP::pagingLock = Config::arch == "48K" ? 1 : 0;

  // Init disk controller
  rvmWD1793Reset(&fdd);
#if !PICO_RP2040
  if (MB02::enabled) MB02::reset();
#endif
#ifdef USE_GS
  // Without this, GS-Z80 keeps running (still streaming previous module's
  // samples from PSRAM) when ZX side reboots — leftover state collides with
  // the new player's load, producing random garbled audio.
  if (GS::enabled) GS::reset();
#endif

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
  if (audioBufferCovoxL) memset(audioBufferCovoxL, 0, 2 * ESP_AUDIO_SAMPLES_PENTAGON); // L+R halves
  memset(chip0.SamplebufAY_L, 0, sizeof(chip0.SamplebufAY_L));
  if (chip1) memset(chip1->SamplebufAY_R, 0, sizeof(chip1->SamplebufAY_R));
#if !PICO_RP2040
  if (saaChip) {
    memset(saaChip->SamplebufSAA_L, 0, sizeof(saaChip->SamplebufSAA_L));
    memset(saaChip->SamplebufSAA_R, 0, sizeof(saaChip->SamplebufSAA_R));
  }
#endif
  lastCovoxVal = lastCovoxValR = lastaudioBit = 0;
  memset(Ports::sndriveLatch, 0, sizeof(Ports::sndriveLatch));
  Ports::sndriveUsed = 0;

  AY_emu = Config::AY48;
#if !PICO_RP2040
    SAA_emu = Config::SAA1099;
    Midi::enabled = Config::midi;
    if (Midi::enabled) Midi::init();
    if (Config::dma_mode) Z80DMA::reset();
#endif

  // Set samples per frame and AY_emu flag depending on arch
  // Profi shares the 48K frame timing (69888 T / 624 samples) — it MUST take the
  // 48K branch here, exactly like setup() does. Otherwise it falls through to the
  // Pentagon branch (640 samples/frame) and over-feeds the 31250 Hz DAC by ~2.6%
  // (640*50.08fps = 32051 > 31250), causing ring overrun → periodic clicks/buzz.
  if (Config::arch == "48K" || Config::arch == "Profi") {
    samplesPerFrame = ESP_AUDIO_SAMPLES_48;
    audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_48;
    audioAYDivider = ESP_AUDIO_AY_DIV_48;
    audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_48;
    Audio_freq = ESP_AUDIO_FREQ_48;
    tstatesPerSampleFP = (TSTATES_PER_FRAME_48 << 8) / ESP_AUDIO_SAMPLES_48;
  } else if (Config::arch == "128K" || Config::arch == "ALF") {
    samplesPerFrame = ESP_AUDIO_SAMPLES_128;
    audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_128;
    audioAYDivider = ESP_AUDIO_AY_DIV_128;
    audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_128;
    Audio_freq = ESP_AUDIO_FREQ_128;
    tstatesPerSampleFP = (TSTATES_PER_FRAME_128 << 8) / ESP_AUDIO_SAMPLES_128;
  } else { /// if (Config::arch == "P512" || Config::arch == "Pentagon") {
    samplesPerFrame = ESP_AUDIO_SAMPLES_PENTAGON;
    audioOverSampleDivider = ESP_AUDIO_OVERSAMPLES_DIV_PENTAGON;
    audioAYDivider = ESP_AUDIO_AY_DIV_PENTAGON;
    audioSampleDivider = ESP_AUDIO_SAMPLES_DIV_PENTAGON;
    Audio_freq = ESP_AUDIO_FREQ_PENTAGON;
    tstatesPerSampleFP = (TSTATES_PER_FRAME_PENTAGON << 8) / ESP_AUDIO_SAMPLES_PENTAGON;
  }

  audioCOVOXDivider = audioAYDivider;

  init_sound();
  pcm_setup(Audio_freq);
#if !PICO_RP2040
  // A reset re-runs init_sound()/pcm_setup(), which can re-claim the shared audio
  // output pins (GP26/27 on MURM1_P2). If a live ESP/WiFi link is using them for
  // its UART, re-assert ownership so WiFi survives a machine reset (F11) instead of
  // dying until a full reboot.
  if (ZiFi::linkUp()) ZiFi::reclaimPins();
#endif

  if (Config::tape_player) {
    AY_emu = false; // Disable AY emulation if tape player mode is set
#if !PICO_RP2040
    SAA_emu = false;
#endif
  }

  // Reconfigure optional subsystems against current Config and apply
  // synchronously here (we're outside the main loop's frame boundary).
  TurboSubsys::request(!Config::tape_player && Config::turbosound != 0);
  CovoxSubsys::request(Config::covox != 0 || Config::soundriveEnabled());
#if !PICO_RP2040
  PitSubsys::request(Z80Ops::isByte);
  SaaSubsys::request(!Config::tape_player && Config::SAA1099);
  MidiSubsys::request(Config::midi != 0);
  DmaSubsys::request(Config::dma_mode != 0);
#endif
  Subsystems::applyPending();

  // Reset AY emulation
  chip0.init();
  chip0.set_sound_format(Audio_freq, 1, 8);
  chip0.set_stereo(AYEMU_MONO, NULL);
  chip0.reset();
  if (chip1) {
    chip1->init();
    chip1->set_sound_format(Audio_freq, 1, 8);
    chip1->set_stereo(AYEMU_MONO, NULL);
    chip1->reset();
  }

#if !PICO_RP2040
  // Reset SAA1099 emulation
  if (saaChip) {
    saaChip->init();
    saaChip->set_sound_format(Audio_freq, 1, 8);
    saaChip->reset();
  }

  // Silence the MIDI synth on machine reset — otherwise notes that were sounding
  // when F11 is pressed keep ringing. reset() = all-notes-off for the active engine.
  if (Midi::enabled == 3)      SoftSynth::reset();
  else if (Midi::enabled == 4) MidiSynth::reset();
#endif

  CPU::reset();

  VIDEO::Reset();

#if !PICO_RP2040
  // KR580VI53 (8253 PIT) — reset to silent state
  if (Z80Ops::isByte && audioBufferPIT) {
    memset(audioBufferPIT, 0, ESP_AUDIO_SAMPLES_PENTAGON);
    memset(Ports::pitChannels, 0, sizeof(Ports::pitChannels));
  }
#endif

  // Re-mount the remembered tape (reset() wiped it above) so a tape survives an
  // F11 reset like a mounted disk does. No-op if no tape is remembered.
  Tape::LoadRemembered();
}

//=======================================================================================
// KEYBOARD / KEMPSTON
//=======================================================================================
IRAM_ATTR bool ESPectrum::readKbd(fabgl::VirtualKeyItem *Nextkey) {

  bool r = PS2Controller.keyboard()->getNextVirtualKey(Nextkey);
  // Global keys
  if (Nextkey->down) {
    if (Nextkey->vk ==
        fabgl::VK_PRINTSCREEN) { // Capture framebuffer to BMP file in SD Card
                                 // (thx @dcrespo3d!)
      // On Profi plain PrtScr is the Karabas-Pro XT-keyboard toggle (handled
      // in do_OSD) — there the BMP capture moves to Alt+PrtScr; other archs
      // keep the plain-PrtScr capture.
      bool xtToggle = Z80Ops::isProfi &&
                      !PS2Controller.keyboard()->isVKDown(fabgl::VK_LALT) &&
                      !PS2Controller.keyboard()->isVKDown(fabgl::VK_RALT);
      if (!xtToggle) {
        CaptureToBmp();
        r = false;
      }
    } else if (Nextkey->vk ==
               fabgl::VK_SCROLLLOCK) { // Change CursorAsJoy setting
      Config::CursorAsJoy = !Config::CursorAsJoy;
      PS2Controller.keyboard()->setLEDs(false, false, Config::CursorAsJoy);
      Config::save();
      r = false;
    }
  }

  return r;
}

fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_FIRE1 = fabgl::VK_NONE;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_FIRE2 = fabgl::VK_NONE;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_TAB = fabgl::VK_TAB;
fabgl::VirtualKey ESPectrum::VK_ESPECTRUM_GRAVEACCENT = fabgl::VK_GRAVEACCENT;

#if !PICO_RP2040
// Map a fabgl VirtualKey to a PQ-DOS serial-keyboard scancode. Table verified
// against the 0.41h1 BIOS translate routine (ROM 0x2363: `LD HL,0x2429` —
// 0x2429 holds 'B'): base layout is "BHY65TGVNJU74RFCMKI83EDX?LO92WSZ <cr>P01QA?./"
// — scan 0='B', 1='H', 2='Y'… The Ctrl layer at ROM 0x2401 confirms the base:
// index 0 = 0x02 (Ctrl+B/STX), 1 = 0x08 (Ctrl+H/BS), 2 = 0x19 (Ctrl+Y). The
// same "BH"-prefixed table exists in every build (h1/h2 bank0+bank3, the old
// profi64k.rom, and QDOS.SYS on pqdos1.fdi) — an earlier extraction matched
// the table from its 'Y' byte, which shifted every code by -2 and left B/H
// "missing". Slots 0x18/0x27 are genuinely NUL. Returns 0xFF for unmapped.
// PQDOS reads keys ONLY via ports #F3/#D3 (Ports::pushKey), never the #FE matrix.
static uint8_t pqdosScancode(fabgl::VirtualKey vk) {
  switch (vk) {
    case fabgl::VK_A: case fabgl::VK_a: return 0x26;
    case fabgl::VK_B: case fabgl::VK_b: return 0x00;
    case fabgl::VK_C: case fabgl::VK_c: return 0x0F;
    case fabgl::VK_D: case fabgl::VK_d: return 0x16;
    case fabgl::VK_E: case fabgl::VK_e: return 0x15;
    case fabgl::VK_F: case fabgl::VK_f: return 0x0E;
    case fabgl::VK_G: case fabgl::VK_g: return 0x06;
    case fabgl::VK_H: case fabgl::VK_h: return 0x01;
    case fabgl::VK_I: case fabgl::VK_i: return 0x12;
    case fabgl::VK_J: case fabgl::VK_j: return 0x09;
    case fabgl::VK_K: case fabgl::VK_k: return 0x11;
    case fabgl::VK_L: case fabgl::VK_l: return 0x19;
    case fabgl::VK_M: case fabgl::VK_m: return 0x10;
    case fabgl::VK_N: case fabgl::VK_n: return 0x08;
    case fabgl::VK_O: case fabgl::VK_o: return 0x1A;
    case fabgl::VK_P: case fabgl::VK_p: return 0x22;
    case fabgl::VK_Q: case fabgl::VK_q: return 0x25;
    case fabgl::VK_R: case fabgl::VK_r: return 0x0D;
    case fabgl::VK_S: case fabgl::VK_s: return 0x1E;
    case fabgl::VK_T: case fabgl::VK_t: return 0x05;
    case fabgl::VK_U: case fabgl::VK_u: return 0x0A;
    case fabgl::VK_V: case fabgl::VK_v: return 0x07;
    case fabgl::VK_W: case fabgl::VK_w: return 0x1D;
    case fabgl::VK_X: case fabgl::VK_x: return 0x17;
    case fabgl::VK_Y: case fabgl::VK_y: return 0x02;
    case fabgl::VK_Z: case fabgl::VK_z: return 0x1F;
    case fabgl::VK_0: case fabgl::VK_KP_0: return 0x23;
    case fabgl::VK_1: case fabgl::VK_KP_1: return 0x24;
    case fabgl::VK_2: case fabgl::VK_KP_2: return 0x1C;
    case fabgl::VK_3: case fabgl::VK_KP_3: return 0x14;
    case fabgl::VK_4: case fabgl::VK_KP_4: return 0x0C;
    case fabgl::VK_5: case fabgl::VK_KP_5: return 0x04;
    case fabgl::VK_6: case fabgl::VK_KP_6: return 0x03;
    case fabgl::VK_7: case fabgl::VK_KP_7: return 0x0B;
    case fabgl::VK_8: case fabgl::VK_KP_8: return 0x13;
    case fabgl::VK_9: case fabgl::VK_KP_9: return 0x1B;
    case fabgl::VK_SPACE: return 0x20;
    case fabgl::VK_RETURN: case fabgl::VK_KP_ENTER: return 0x21;
    case fabgl::VK_PERIOD: return 0x28;
    case fabgl::VK_SLASH: return 0x29;
    default: return 0xFF;
  }
}
#endif

IRAM_ATTR void ESPectrum::processKeyboard() {
  static uint8_t PS2cols[8] = {0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf, 0xbf};
  auto Kbd = PS2Controller.keyboard();
  fabgl::VirtualKeyItem NextKey;
  fabgl::VirtualKey KeytoESP;
  bool Kdown;
  bool r = false;
  bool j[10] = {true, true, true, true, true, true, true, true, true, true};
  bool jShift = true;

  if ((Config::numPcBP > 0 && Config::hasBreakPoint(Z80::getRegPC(), Config::BP_PC)) ||
      CPU::portBasedBP) {
    int64_t osd_start = esp_timer_get_time();
    OSD::osdDebug();
    VIDEO::brdnextframe = true;
    ESPectrum::ts_start += esp_timer_get_time() - osd_start;
    CPU::portBasedBP = false;
    return;
  }

  while (Kbd->virtualKeyAvailable()) {
    r = readKbd(&NextKey);
    if (r) {
      KeytoESP = NextKey.vk;
      Kdown = NextKey.down;
#if !PICO_RP2040
      // PQ-DOS serial keyboard: feed key-downs to the #F3/#D3 queue (Profi only).
      // Harmless on non-PQDOS software — those ports are ignored unless polled.
#if FDD_PORT_TRACE
      if (Kdown)
        Debug::log("[PQKBD KEYDN] vk=%d profi=%d sc=%02X", (int)KeytoESP,
                   (int)Z80Ops::isProfi, (int)pqdosScancode(KeytoESP));
#endif
      if (Kdown && Z80Ops::isProfi) {
        uint8_t sc = pqdosScancode(KeytoESP);
        if (sc != 0xFF) Ports::pushKey(sc);
      }
      // Real Karabas-Pro "Menu"-key ROMSET hotkeys: Menu(Win/GUI)+F1..F4 pick
      // ROMSET 0..3 (ROMain / PQDOS / Flash Tool / FDImage) and reset into its
      // bank0 — mirrors the hardware combos from the Karabas-Pro manual.
      // Active only while a Karabas* romset is already selected ("1024K
      // (Original)" is not part of the real Karabas flash, so plain GUI+F1
      // there still means nothing / OSD keeps its normal F-key behavior).
      if (Kdown && Z80Ops::isProfi &&
          KeytoESP >= fabgl::VK_F1 && KeytoESP <= fabgl::VK_F4 &&
          (Kbd->isVKDown(fabgl::VK_LGUI) || Kbd->isVKDown(fabgl::VK_RGUI))) {
        static const char* karabasRomsets[4] = {
            "ProfiKarabas", "ProfiPQ", "ProfiKarabasFT", "ProfiKarabasFDI" };
        bool inKarabas = false;
        for (int i = 0; i < 4; i++)
          if (Config::romSet == karabasRomsets[i]) { inKarabas = true; break; }
        if (inKarabas) {
          const char* target = karabasRomsets[KeytoESP - fabgl::VK_F1];
          if (Config::romSet != target) {
            Config::romSet = target;
            if (Config::pref_romSetProfi == "Last") Config::romSetProfi = target;
            Config::save();
            Config::requestMachine("Profi", target);
          }
          ESPectrum::reset(); // hardware combo resets into the ROMSET's bank0
          return;
        }
      }
#endif
      // The rest of the Karabas-Pro "Menu"-key combos from the user manual.
      // Unlike Menu+F1..F4 they map onto machine-independent emulator settings,
      // so they work on any arch. osdCenteredMsg blocks for the toast duration —
      // compensate ts_start like the do_OSD dispatch below does.
      if (Kdown && (Kbd->isVKDown(fabgl::VK_LGUI) || Kbd->isVKDown(fabgl::VK_RGUI))) {
        auto menuToast = [](const char *msg) {
          int64_t t = esp_timer_get_time();
          OSD::osdCenteredMsg(msg, LEVEL_INFO, 500);
          ESPectrum::ts_start += esp_timer_get_time() - t;
        };
        if (KeytoESP == fabgl::VK_F5) { // TurboFDC = our TR-DOS fast mode
          Config::trdosFastMode = !Config::trdosFastMode;
          rvmWD1793UpdateFastmode(&ESPectrum::fdd);
          Config::save();
          menuToast(Config::trdosFastMode ? " Turbo FDC: On  " : " Turbo FDC: Off " );
          return;
        }
        if (KeytoESP == fabgl::VK_F7) { // SSG stereo mode
          static const char* const ayModes[3] =
              { " AY stereo: ABC  ", " AY stereo: ACB  ", " AY stereo: Mono " };
          Config::ayConfig = (Config::ayConfig + 1) % 3;
          Config::save();
          menuToast(ayModes[Config::ayConfig]);
          return;
        }
        if (KeytoESP == fabgl::VK_F8 ||   // AY/YM chip select
            KeytoESP == fabgl::VK_F9 ||   // VGA/TV scan mode
            KeytoESP == fabgl::VK_F10) {  // 50/60 Hz scan rate
          menuToast(" Not supported ");
          return;
        }
        if (KeytoESP == fabgl::VK_F11) { // Turbo 3.5/7/14 MHz — 3-state cycle
          // (the configurable Turbo hotkey also offers 28 MHz as a 4th state)
          static const char* const mhz[3] =
              { " CPU: 3.5 MHz " , " CPU: 7 MHz   ", " CPU: 14 MHz  " };
          ESPectrum::multiplicator = (ESPectrum::multiplicator + 1) % 3;
          CPU::updateStatesInFrame();
          menuToast(mhz[ESPectrum::multiplicator]);
          return;
        }
        if (KeytoESP == fabgl::VK_F12) { // NMI — route through do_OSD with the
          // combo bound to the NMI hotkey, so its DS80/AY guards apply and
          // Pentagon/Profi get the visible NMI/NMI+DOS chooser (a bare
          // triggerNMI is invisible there: the ROM's 0x66 just RETNs).
          const Config::HotkeyBinding &hk = Config::hotkeys[Config::HK_NMI];
          if (hk.vk != (uint16_t)fabgl::VK_NONE) {
            int64_t osd_start = esp_timer_get_time();
            OSD::do_OSD((fabgl::VirtualKey)hk.vk, hk.alt, hk.ctrl);
            Kbd->emptyVirtualKeyQueue();
            VIDEO::brdnextframe = true;
            ESPectrum::ts_start += esp_timer_get_time() - osd_start;
          } else
            Z80::triggerNMI();
          return;
        }
        if (KeytoESP == fabgl::VK_TAB) { // swap drive letters A<->B
          rvmWD1793SwapDrives(&ESPectrum::fdd, 0, 1);
          bool wp = Config::driveWP[0];
          Config::driveWP[0] = Config::driveWP[1];
          Config::driveWP[1] = wp;
          Config::save(); // persists the per-unit filenames + WP flags
          menuToast(" Drives A <-> B ");
          return;
        }
        if (KeytoESP == fabgl::VK_j || KeytoESP == fabgl::VK_J) { // joystick type
          static const char* const joyNames[5] =
              { " Joy: Cursor     ", " Joy: Kempston   ", " Joy: Sinclair 1 ",
                " Joy: Sinclair 2 ", " Joy: Fuller     " };
          Config::joystick = (Config::joystick + 1) % 5; // Custom stays OSD-only
          // NOT setJoyMap() — it wipes joydef and pops a "load default map?"
          // dialog; the pad-button mapping is orthogonal to the port type and
          // stays editable in the OSD joystick menu.
          Config::save();
          menuToast(joyNames[Config::joystick]);
          return;
        }
        if (KeytoESP == fabgl::VK_ESCAPE) { // open the OSD main menu
          int64_t osd_start = esp_timer_get_time();
          OSD::do_OSD(fabgl::VK_F1, false, false);
          Kbd->emptyVirtualKeyQueue();
#ifdef DIRTY_LINES
          for (int i = 0; i < SPEC_H; i++)
            VIDEO::dirty_lines[i] |= 0x01;
#endif // DIRTY_LINES
          VIDEO::brdnextframe = true;
          ESPectrum::ts_start += esp_timer_get_time() - osd_start;
          return;
        }
      }
      if ((Kdown) &&
          ((KeytoESP >= fabgl::VK_F1 && KeytoESP <= fabgl::VK_F12) ||
            KeytoESP == fabgl::VK_PAUSE || KeytoESP == fabgl::VK_PRINTSCREEN ||
            KeytoESP == fabgl::VK_SCROLLLOCK || KeytoESP == fabgl::VK_NUMLOCK ||
            KeytoESP == fabgl::VK_INSERT ||
            KeytoESP == fabgl::VK_HOME || KeytoESP == fabgl::VK_END ||
            KeytoESP == fabgl::VK_PAGEUP || KeytoESP == fabgl::VK_PAGEDOWN ||
            KeytoESP == fabgl::VK_TILDE || KeytoESP == fabgl::VK_GRAVEACCENT ||
            KeytoESP == fabgl::VK_VOLUMEUP || KeytoESP == fabgl::VK_VOLUMEDOWN ||
            KeytoESP == fabgl::VK_VOLUMEMUTE ||
            KeytoESP == fabgl::VK_DELETE)) {
        // In Profi extended keyboard mode, F-keys and nav-keys pass to Z80 matrix
        // instead of opening OSD. PAUSE always opens OSD (escape hatch).
        // GRAVEACCENT/TILDE still go to OSD (for Alt+` toggle and max-speed hotkey).
#if !PICO_RP2040
        bool passToZ80 = Z80Ops::isProfi && Config::profi_ext_keys
            && KeytoESP != fabgl::VK_PAUSE
            && KeytoESP != fabgl::VK_TILDE
            && KeytoESP != fabgl::VK_GRAVEACCENT
            && KeytoESP != fabgl::VK_PRINTSCREEN
            && KeytoESP != fabgl::VK_SCROLLLOCK
            && KeytoESP != fabgl::VK_NUMLOCK
            && KeytoESP != fabgl::VK_VOLUMEUP
            && KeytoESP != fabgl::VK_VOLUMEDOWN
            && KeytoESP != fabgl::VK_VOLUMEMUTE;
        if (!passToZ80)
#endif
        {
          int64_t osd_start = esp_timer_get_time();
          OSD::do_OSD(
              KeytoESP,
              Kbd->isVKDown(fabgl::VK_LALT) || Kbd->isVKDown(fabgl::VK_RALT),
              Kbd->isVKDown(fabgl::VK_LCTRL) || Kbd->isVKDown(fabgl::VK_RCTRL));
          Kbd->emptyVirtualKeyQueue();
#ifdef DIRTY_LINES
          for (int i = 0; i < SPEC_H; i++)
            VIDEO::dirty_lines[i] |= 0x01;
#endif // DIRTY_LINES
          // Refresh border
          VIDEO::brdnextframe = true;
          // While paused the renderer never repaints (CPU::loop bails straight
          // to EndFrame), so whatever the OSD drew stays on screen forever.
          // Repaint the frozen frame and put the PAUSE box back on top.
          if (CPU::paused) {
            VIDEO::RedrawPausedFrame();
            OSD::osdCenteredMsg(OSD_PAUSE[Config::lang], LEVEL_INFO, 0);
          }
          ESPectrum::ts_start += esp_timer_get_time() - osd_start;
          return;
        }
        // else: key falls through to extended keyboard polling below
      }
      // Reset keys
      if (Kdown && NextKey.LALT) {
        if (NextKey.CTRL) {
          if (KeytoESP == fabgl::VK_DELETE) {
            // printf("Ctrl + Alt + Supr!\n");
            // ESP host reset
            Config::ram_file = NO_RAM_FILE;
            Config::save();
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
          } else
            ESPectrum::reset();
          return;
        }
      }

      if (Config::joystick == JOY_KEMPSTON)
        Ports::port[Config::kempstonPort] = 0;
      else if (Config::joystick == JOY_FULLER)
        Ports::port[0x7f] = 0xff;

      if (Config::joystick == JOY_KEMPSTON) {
        // Kempston 8-bit: 0=right,1=left,2=down,3=up,4=A/fire,5=B/altfire,6=X,7=Start.
        // The 4 direction codes always drive their own bit. Extra buttons (A,B,C,X,Y,Z,L2,R2)
        // are aliased via joydef slots 6..13: joydef[slot] gives the target action which
        // is resolved to a Kempston bit by the switch below.
        // directions: fixed bit = code - VK_JOY_RIGHT (0..3)
        for (int i = fabgl::VK_JOY_RIGHT; i <= fabgl::VK_JOY_UP; i++)
          if (Kbd->isVKDown((fabgl::VirtualKey)i))
            bitWrite(Ports::port[Config::kempstonPort], i - fabgl::VK_JOY_RIGHT, 1);
        // extra buttons A,B,C,X,Y,Z,L2,R2 -> joydef slots 6..13
        static const fabgl::VirtualKey extraRaw[8] = {
          fabgl::VK_JOY_A, fabgl::VK_JOY_B, fabgl::VK_JOY_C, fabgl::VK_JOY_X,
          fabgl::VK_JOY_Y, fabgl::VK_JOY_Z, fabgl::VK_JOY_L2, fabgl::VK_JOY_R2
        };
        for (int e = 0; e < 8; e++) {
          if (!Kbd->isVKDown(extraRaw[e])) continue;
          // Resolve the target action for this physical button from its joydef slot.
          // Only VK_JOY_* / VK_DPAD_FIRE|ALTFIRE targets drive a Kempston bit here;
          // VK_NONE or a keyboard key means this pad button does nothing on Kempston
          // (a keyboard target is handled by joyMap as a key press instead).
          uint16_t target = Config::joydef[6 + e];
          int bit = -1;
          switch (target) {
            case fabgl::VK_JOY_RIGHT:
            case fabgl::VK_DPAD_RIGHT:   bit = 0; break;
            case fabgl::VK_JOY_LEFT:
            case fabgl::VK_DPAD_LEFT:    bit = 1; break;
            case fabgl::VK_JOY_DOWN:
            case fabgl::VK_DPAD_DOWN:    bit = 2; break;
            case fabgl::VK_JOY_UP:
            case fabgl::VK_DPAD_UP:      bit = 3; break;
            case fabgl::VK_JOY_A:
            case fabgl::VK_DPAD_FIRE:    bit = 4; break;
            case fabgl::VK_JOY_B:
            case fabgl::VK_DPAD_ALTFIRE: bit = 5; break;
            case fabgl::VK_JOY_X:        bit = 6; break;
            case fabgl::VK_JOY_START:
            case fabgl::VK_DPAD_START:   bit = 7; break;
            default: break;
          }
          if (bit >= 0)
            bitWrite(Ports::port[Config::kempstonPort], bit, 1);
        }
        // Start is not in extraRaw (it flows via joydef[4] → VK_JOY_START separately)
        if (Kbd->isVKDown(fabgl::VK_JOY_START))
          bitWrite(Ports::port[Config::kempstonPort], 7, 1);
      } else if (Config::joystick == JOY_FULLER) { // Fuller
        if (Kbd->isVKDown(fabgl::VK_JOY_RIGHT)) {
          bitWrite(Ports::port[0x7f], 3, 0);
        }
        if (Kbd->isVKDown(fabgl::VK_JOY_LEFT)) {
          bitWrite(Ports::port[0x7f], 2, 0);
        }
        if (Kbd->isVKDown(fabgl::VK_JOY_DOWN)) {
          bitWrite(Ports::port[0x7f], 1, 0);
        }
        if (Kbd->isVKDown(fabgl::VK_JOY_UP)) {
          bitWrite(Ports::port[0x7f], 0, 0);
        }
        if (Kbd->isVKDown(fabgl::VK_JOY_A)) {
          bitWrite(Ports::port[0x7f], 7, 0);
        }
      }

      jShift =
          !(Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT));
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
      // Check keyboard status and map it to Spectrum Ports
      bitWrite(PS2cols[0], 0,
               (jShift) & (!Kbd->isVKDown(fabgl::VK_BACKSPACE)) &
                   (!Kbd->isVKDown(fabgl::VK_CAPSLOCK))         // Caps lock
                   & (!Kbd->isVKDown(VK_ESPECTRUM_GRAVEACCENT)) // Edit
                   & (!Kbd->isVKDown(VK_ESPECTRUM_TAB))         // Extended mode
                   & (!Kbd->isVKDown(fabgl::VK_ESCAPE))         // Break
      );                                                        // CAPS SHIFT
      bitWrite(PS2cols[0], 1,
               (!Kbd->isVKDown(fabgl::VK_Z)) & (!Kbd->isVKDown(fabgl::VK_z)));
      bitWrite(PS2cols[0], 2,
               (!Kbd->isVKDown(fabgl::VK_X)) & (!Kbd->isVKDown(fabgl::VK_x)));
      bitWrite(PS2cols[0], 3,
               (!Kbd->isVKDown(fabgl::VK_C)) & (!Kbd->isVKDown(fabgl::VK_c)));
      bitWrite(PS2cols[0], 4,
               (!Kbd->isVKDown(fabgl::VK_V)) & (!Kbd->isVKDown(fabgl::VK_v)));

      bitWrite(PS2cols[1], 0,
               (!Kbd->isVKDown(fabgl::VK_A)) & (!Kbd->isVKDown(fabgl::VK_a)));
      bitWrite(PS2cols[1], 1,
               (!Kbd->isVKDown(fabgl::VK_S)) & (!Kbd->isVKDown(fabgl::VK_s)));
      bitWrite(PS2cols[1], 2,
               (!Kbd->isVKDown(fabgl::VK_D)) & (!Kbd->isVKDown(fabgl::VK_d)));
      bitWrite(PS2cols[1], 3,
               (!Kbd->isVKDown(fabgl::VK_F)) & (!Kbd->isVKDown(fabgl::VK_f)));
      bitWrite(PS2cols[1], 4,
               (!Kbd->isVKDown(fabgl::VK_G)) & (!Kbd->isVKDown(fabgl::VK_g)));

      bitWrite(PS2cols[2], 0,
               (!Kbd->isVKDown(fabgl::VK_Q)) & (!Kbd->isVKDown(fabgl::VK_q)));
      bitWrite(PS2cols[2], 1,
               (!Kbd->isVKDown(fabgl::VK_W)) & (!Kbd->isVKDown(fabgl::VK_w)));
      bitWrite(PS2cols[2], 2,
               (!Kbd->isVKDown(fabgl::VK_E)) & (!Kbd->isVKDown(fabgl::VK_e)));
      bitWrite(PS2cols[2], 3,
               (!Kbd->isVKDown(fabgl::VK_R)) & (!Kbd->isVKDown(fabgl::VK_r)));
      bitWrite(PS2cols[2], 4,
               (!Kbd->isVKDown(fabgl::VK_T)) & (!Kbd->isVKDown(fabgl::VK_t)));

      bitWrite(PS2cols[3], 0,
               (!Kbd->isVKDown(fabgl::VK_1)) &
                   (!Kbd->isVKDown(fabgl::VK_EXCLAIM)) &
                   (!Kbd->isVKDown(VK_ESPECTRUM_GRAVEACCENT)) // Edit
                   & (j[1]));
      bitWrite(PS2cols[3], 1,
               (!Kbd->isVKDown(fabgl::VK_2)) & (!Kbd->isVKDown(fabgl::VK_AT)) &
                   (!Kbd->isVKDown(fabgl::VK_CAPSLOCK)) // Caps lock
                   & (j[2]));
      bitWrite(PS2cols[3], 2,
               (!Kbd->isVKDown(fabgl::VK_3)) &
                   (!Kbd->isVKDown(fabgl::VK_HASH)) & (j[3]));
      bitWrite(PS2cols[3], 3,
               (!Kbd->isVKDown(fabgl::VK_4)) &
                   (!Kbd->isVKDown(fabgl::VK_DOLLAR)) & (j[4]));
      bitWrite(PS2cols[3], 4,
               (!Kbd->isVKDown(fabgl::VK_5)) &
                   (!Kbd->isVKDown(fabgl::VK_PERCENT)) & (j[5]));

      bitWrite(PS2cols[4], 0,
               (!Kbd->isVKDown(fabgl::VK_0)) &
                   (!Kbd->isVKDown(fabgl::VK_RIGHTPAREN)) &
                   (!Kbd->isVKDown(fabgl::VK_BACKSPACE)) & (j[0]));
      bitWrite(PS2cols[4], 1,
               !Kbd->isVKDown(fabgl::VK_9) &
                   (!Kbd->isVKDown(fabgl::VK_LEFTPAREN)) & (j[9]));
      bitWrite(PS2cols[4], 2,
               (!Kbd->isVKDown(fabgl::VK_8)) &
                   (!Kbd->isVKDown(fabgl::VK_ASTERISK)) & (j[8]));
      bitWrite(PS2cols[4], 3,
               (!Kbd->isVKDown(fabgl::VK_7)) &
                   (!Kbd->isVKDown(fabgl::VK_AMPERSAND)) & (j[7]));
      bitWrite(PS2cols[4], 4,
               (!Kbd->isVKDown(fabgl::VK_6)) &
                   (!Kbd->isVKDown(fabgl::VK_CARET)) & (j[6]));

      bitWrite(PS2cols[5], 0,
               (!Kbd->isVKDown(fabgl::VK_P)) & (!Kbd->isVKDown(fabgl::VK_p)) &
                   (!Kbd->isVKDown(fabgl::VK_QUOTE)) // Double quote
      );
      bitWrite(PS2cols[5], 1,
               (!Kbd->isVKDown(fabgl::VK_O)) & (!Kbd->isVKDown(fabgl::VK_o)) &
                   (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
      );
      bitWrite(PS2cols[5], 2,
               (!Kbd->isVKDown(fabgl::VK_I)) & (!Kbd->isVKDown(fabgl::VK_i)));
      bitWrite(PS2cols[5], 3,
               (!Kbd->isVKDown(fabgl::VK_U)) & (!Kbd->isVKDown(fabgl::VK_u)));
      bitWrite(PS2cols[5], 4,
               (!Kbd->isVKDown(fabgl::VK_Y)) & (!Kbd->isVKDown(fabgl::VK_y)));

      bitWrite(PS2cols[6], 0, !Kbd->isVKDown(fabgl::VK_RETURN));
      bitWrite(PS2cols[6], 1,
               (!Kbd->isVKDown(fabgl::VK_L)) & (!Kbd->isVKDown(fabgl::VK_l)));
      bitWrite(PS2cols[6], 2,
               (!Kbd->isVKDown(fabgl::VK_K)) & (!Kbd->isVKDown(fabgl::VK_k)));
      bitWrite(PS2cols[6], 3,
               (!Kbd->isVKDown(fabgl::VK_J)) & (!Kbd->isVKDown(fabgl::VK_j)));
      bitWrite(PS2cols[6], 4,
               (!Kbd->isVKDown(fabgl::VK_H)) & (!Kbd->isVKDown(fabgl::VK_h)));

      bitWrite(PS2cols[7], 0,
               !Kbd->isVKDown(fabgl::VK_SPACE) &
                   (!Kbd->isVKDown(fabgl::VK_ESCAPE)) // Break
      );
      bitWrite(PS2cols[7], 1,
               (!Kbd->isVKDown(fabgl::VK_LCTRL)) // SYMBOL SHIFT
                   & (!Kbd->isVKDown(fabgl::VK_RCTRL)) &
                   (!Kbd->isVKDown(fabgl::VK_COMMA))       // Comma
                   & (!Kbd->isVKDown(fabgl::VK_PERIOD))    // Period
                   & (!Kbd->isVKDown(fabgl::VK_SEMICOLON)) // Semicolon
                   & (!Kbd->isVKDown(fabgl::VK_QUOTE))     // Double quote
                   & (!Kbd->isVKDown(VK_ESPECTRUM_TAB))    // Extended mode
      );                                                   // SYMBOL SHIFT
      bitWrite(PS2cols[7], 2,
               (!Kbd->isVKDown(fabgl::VK_M)) & (!Kbd->isVKDown(fabgl::VK_m)) &
                   (!Kbd->isVKDown(fabgl::VK_PERIOD)) // Period
      );
      bitWrite(PS2cols[7], 3,
               (!Kbd->isVKDown(fabgl::VK_N)) & (!Kbd->isVKDown(fabgl::VK_n)) &
                   (!Kbd->isVKDown(fabgl::VK_COMMA)) // Comma
      );
      bitWrite(PS2cols[7], 4,
               (!Kbd->isVKDown(fabgl::VK_B)) & (!Kbd->isVKDown(fabgl::VK_b)));

#if !PICO_RP2040
      // ── Profi extended keyboard ─────────────────────────────────────────
      // Active when arch=Profi and extended keyboard mode is on.
      //
      // Mechanism (verified from profi_v10.rom bank 0 disasm):
      //  - The ROM scanner (0x03A9) reads bit5 of each port 0xFE half-row into
      //    RAM 0x99DA. Bit5 of row 6 (0xBFFE) is the "ALT" modifier (BIT 6,A at
      //    0x1303). Held bit5 is asserted via extPort[6].
      //  - When ALT is set, the decoded ZX key is remapped via the table at
      //    ROM 0x365B: ALT + bare letter A..P → codes 0x75..0x84.
      //
      //  Matrix coords: rows are the 8 port-0xFE half-rows 0..7
      //  (0xFEFE,0xFDFE,0xFBFE,0xF7FE,0xEFFE,0xDFFE,0xBFFE,0x7FFE);
      //  CapsShift=(0,0), SymbolShift=(7,1).
      //
      //  TWO produce paths (verified via test cell-table 0x3427 + decode tables
      //  0x0450/0x0478/0x04A0 + ALT-remap 0x365B):
      //   (A) ALT held → bare letter remapped: A..J→F1-F10, K..P→nav (0x7F-0x84)
      //   (B) NO ALT → CapsShift/SymShift combo decoded directly:
      //         F11=Caps+Q(0x67) F12=Caps+W(0x68) ESC=Sym+1(0x1B) TAB=Sym+I(0x09)
      //  Path-B keys MUST NOT assert ALT, or 0x365B would remap them.
      if (Z80Ops::isProfi && Config::profi_ext_keys) {
        // Path A — bare letter
        bool f1  = Kbd->isVKDown(fabgl::VK_F1);
        bool f2  = Kbd->isVKDown(fabgl::VK_F2);
        bool f3  = Kbd->isVKDown(fabgl::VK_F3);
        bool f4  = Kbd->isVKDown(fabgl::VK_F4);
        bool f5  = Kbd->isVKDown(fabgl::VK_F5);
        bool f6  = Kbd->isVKDown(fabgl::VK_F6);
        bool f7  = Kbd->isVKDown(fabgl::VK_F7);
        bool f8  = Kbd->isVKDown(fabgl::VK_F8);
        bool f9  = Kbd->isVKDown(fabgl::VK_F9);
        bool f10 = Kbd->isVKDown(fabgl::VK_F10);
        bool kHome = Kbd->isVKDown(fabgl::VK_HOME);
        bool kEnd  = Kbd->isVKDown(fabgl::VK_END);
        bool kPgUp = Kbd->isVKDown(fabgl::VK_PAGEUP);
        bool kPgDn = Kbd->isVKDown(fabgl::VK_PAGEDOWN);
        bool kIns  = Kbd->isVKDown(fabgl::VK_INSERT);
        bool kDel  = Kbd->isVKDown(fabgl::VK_DELETE);
        // Arrows = CapsShift + 5/6/7/8 (ZX cursor keys; CAPS decode layer, no ALT)
        bool kUp    = Kbd->isVKDown(fabgl::VK_UP);
        bool kDown  = Kbd->isVKDown(fabgl::VK_DOWN);
        bool kLeft  = Kbd->isVKDown(fabgl::VK_LEFT);
        bool kRight = Kbd->isVKDown(fabgl::VK_RIGHT);
        // SymShift symbol keys not in the base ZX matrix (Profi sym layer 0x3B97)
        bool sMinus  = Kbd->isVKDown(fabgl::VK_MINUS);
        bool sEqual  = Kbd->isVKDown(fabgl::VK_EQUALS);
        bool sLBrk   = Kbd->isVKDown(fabgl::VK_LEFTBRACKET);
        bool sRBrk   = Kbd->isVKDown(fabgl::VK_RIGHTBRACKET);
        bool sBSlash = Kbd->isVKDown(fabgl::VK_BACKSLASH);
        bool sSlash  = Kbd->isVKDown(fabgl::VK_SLASH);
        bool sColon  = Kbd->isVKDown(fabgl::VK_COLON);
        bool sQuote  = Kbd->isVKDown(fabgl::VK_QUOTE);
        bool sSemi   = Kbd->isVKDown(fabgl::VK_SEMICOLON);
        bool sComma  = Kbd->isVKDown(fabgl::VK_COMMA);   // Sym+N → ','
        bool sPeriod = Kbd->isVKDown(fabgl::VK_PERIOD);  // Sym+M → '.'
        bool kBS     = Kbd->isVKDown(fabgl::VK_BACKSPACE); // Caps+0 → BS

        // Path B — Caps/Sym combo
        bool f11  = Kbd->isVKDown(fabgl::VK_F11);
        bool f12  = Kbd->isVKDown(fabgl::VK_F12);
        bool kEsc = Kbd->isVKDown(fabgl::VK_ESCAPE);
        bool kTab = Kbd->isVKDown(fabgl::VK_TAB);
        bool kCaps = Kbd->isVKDown(fabgl::VK_CAPSLOCK);
        bool lAlt = Kbd->isVKDown(fabgl::VK_LALT);   // left Alt  → Sym+Enter (0x69)
        bool rAlt = Kbd->isVKDown(fabgl::VK_RALT);   // right Alt → Sym+Space (0x71)
        bool altDown = lAlt || rAlt;

        // extPort bit5 = Profi ext column. Bit5-of-row6 (0xBFFE) is the "ALT"
        // modifier (ROM BIT 6,(0x99DA) at 0x114E). ALT makes the ROM remap a
        // bare letter via table 0x3571: A..J → F1-F10 (0x75-0x7E),
        // K..P → nav (0x7F-0x84). Without ALT the letters stay as A..P.
        for (int i = 0; i < 8; i++) Ports::extPort[i] |= 0x20;
        bool anyFkey = f1||f2||f3||f4||f5||f6||f7||f8||f9||f10;
        bool anyNav  = kHome || kEnd || kPgUp || kPgDn || kIns || kDel;
        // Direct Alt+letter combo: holding physical Alt + a normal letter that
        // the 0x3571 table remaps (A..P) should yield the same F-key/nav code
        // as the dedicated F-keys do — exactly like the real Profi keyboard.
        // The letter is already placed in the matrix by the normal scan; we
        // just add bit5 so the ROM remaps it. (A..J→F1-F10, K..P→nav.)
        bool altLetter = altDown &&
          (Kbd->isVKDown(fabgl::VK_A)||Kbd->isVKDown(fabgl::VK_a)||
           Kbd->isVKDown(fabgl::VK_B)||Kbd->isVKDown(fabgl::VK_b)||
           Kbd->isVKDown(fabgl::VK_C)||Kbd->isVKDown(fabgl::VK_c)||
           Kbd->isVKDown(fabgl::VK_D)||Kbd->isVKDown(fabgl::VK_d)||
           Kbd->isVKDown(fabgl::VK_E)||Kbd->isVKDown(fabgl::VK_e)||
           Kbd->isVKDown(fabgl::VK_F)||Kbd->isVKDown(fabgl::VK_f)||
           Kbd->isVKDown(fabgl::VK_G)||Kbd->isVKDown(fabgl::VK_g)||
           Kbd->isVKDown(fabgl::VK_H)||Kbd->isVKDown(fabgl::VK_h)||
           Kbd->isVKDown(fabgl::VK_I)||Kbd->isVKDown(fabgl::VK_i)||
           Kbd->isVKDown(fabgl::VK_J)||Kbd->isVKDown(fabgl::VK_j)||
           Kbd->isVKDown(fabgl::VK_K)||Kbd->isVKDown(fabgl::VK_k)||
           Kbd->isVKDown(fabgl::VK_L)||Kbd->isVKDown(fabgl::VK_l)||
           Kbd->isVKDown(fabgl::VK_M)||Kbd->isVKDown(fabgl::VK_m)||
           Kbd->isVKDown(fabgl::VK_N)||Kbd->isVKDown(fabgl::VK_n)||
           Kbd->isVKDown(fabgl::VK_O)||Kbd->isVKDown(fabgl::VK_o)||
           Kbd->isVKDown(fabgl::VK_P)||Kbd->isVKDown(fabgl::VK_p));
        // Bit5 (the 0x3571-remap modifier): for dedicated F1-F10/nav keys, and
        // for a direct physical Alt+letter combo. NOT for Alt-alone (that lights
        // the Alt cell via Sym+Enter/Space below; bit5 on row6 would corrupt it).
        if (anyFkey || anyNav || altLetter)
          Ports::extPort[6] &= ~0x20;

        // Path A: bare letter (ROM decodes A=F1 … J=F10; K..P + ALT = nav)
        if (f1)    PS2cols[1] &= ~0x01;   // A → (1,0)  F1
        if (f2)    PS2cols[7] &= ~0x10;   // B → (7,4)  F2
        if (f3)    PS2cols[0] &= ~0x08;   // C → (0,3)  F3
        if (f4)    PS2cols[1] &= ~0x04;   // D → (1,2)  F4
        if (f5)    PS2cols[2] &= ~0x04;   // E → (2,2)  F5
        if (f6)    PS2cols[1] &= ~0x08;   // F → (1,3)  F6
        if (f7)    PS2cols[1] &= ~0x10;   // G → (1,4)  F7
        if (f8)    PS2cols[6] &= ~0x10;   // H → (6,4)  F8
        if (f9)    PS2cols[5] &= ~0x04;   // I → (5,2)  F9
        if (f10)   PS2cols[6] &= ~0x08;   // J → (6,3)  F10
        if (kHome) PS2cols[6] &= ~0x04;   // K → (6,2)  HOME (0x7F)
        if (kEnd)  PS2cols[6] &= ~0x02;   // L → (6,1)  END  (0x80)
        if (kPgUp) PS2cols[7] &= ~0x04;   // M → (7,2)  PgUp (0x81)
        if (kPgDn) PS2cols[7] &= ~0x08;   // N → (7,3)  PgDn (0x82)
        if (kIns)  PS2cols[5] &= ~0x02;   // O → (5,1)  INS  (0x83)
        if (kDel)  PS2cols[5] &= ~0x01;   // P → (5,0)  DEL  (0x84)

        // Arrows = CapsShift + 5/6/7/8 (CAPS decode layer, no ALT)
        //   LEFT=Caps+5(3,4) DOWN=Caps+6(4,4) UP=Caps+7(4,3) RIGHT=Caps+8(4,2)
        if (kLeft)  { PS2cols[0] &= ~0x01; PS2cols[3] &= ~0x10; } // Caps+5 LEFT
        if (kDown)  { PS2cols[0] &= ~0x01; PS2cols[4] &= ~0x10; } // Caps+6 DOWN
        if (kUp)    { PS2cols[0] &= ~0x01; PS2cols[4] &= ~0x08; } // Caps+7 UP
        if (kRight) { PS2cols[0] &= ~0x01; PS2cols[4] &= ~0x04; } // Caps+8 RIGHT

        // Path B: shift-combo specials (hddboot decode tables 0x3B6F/97/BF).
        //   F11=Sym+Q(0x67) F12=Sym+W(0x68) ESC=Caps+1(0x1B) TAB=Caps+I(0x09)
        // The normal scanner maps VK_ESCAPE→Break (Caps+Space), so release the
        // stray Space bit first so ESC decodes cleanly to 0x1B.
        if ((kEsc || kTab) && !Kbd->isVKDown(fabgl::VK_SPACE))
          PS2cols[7] |= 0x01;            // release Space (7,0) left by ESC=Break
        // TAB: normal scan sets "Extended mode" = Caps+Sym; release stray SymShift.
        if (kTab) PS2cols[7] |= 0x02;
        // QUOTE: normal scan sets "Double quote" = Sym+P; release stray P (5,0).
        if (sQuote) PS2cols[5] |= 0x01;
        // CAPSLOCK: normal scan sets Caps+"2"; release stray "2" (3,1) so the
        // CpLoc cell (code 0x70 = Caps+Sym) decodes cleanly.
        if (kCaps) PS2cols[3] |= 0x02;
        if (f11) { PS2cols[7] &= ~0x02; PS2cols[2] &= ~0x01; } // Sym+Q → F11 (0x67)
        if (f12) { PS2cols[7] &= ~0x02; PS2cols[2] &= ~0x02; } // Sym+W → F12 (0x68)
        if (kEsc) { PS2cols[0] &= ~0x01; PS2cols[3] &= ~0x01; } // Caps+1 → ESC (0x1B)
        if (kTab) { PS2cols[0] &= ~0x01; PS2cols[5] &= ~0x04; } // Caps+I → TAB (0x09)
        if (kCaps){ PS2cols[0] &= ~0x01; PS2cols[7] &= ~0x02; } // Caps+Sym → CpLoc (0x70)
        // Alt cell (code 0x69) = SymShift + Enter. The bit5 modifier alone is
        // stripped by the scanner (AND 0x1F at 0x0618) and never decodes to a
        // code, so the test can't show it from extPort[6]; the real Profi Alt
        // key closes Sym+Enter. Assert that so the Alt cell lights.
        // Alt cell (Alt held alone, no remappable letter): L=Sym+Enter, R=Sym+Space.
        // Skipped when altLetter — then Alt acts as the F-key/nav remap modifier.
        if (lAlt && !altLetter) { PS2cols[7] &= ~0x02; PS2cols[6] &= ~0x01; } // → L-Alt (0x69)
        if (rAlt && !altLetter) { PS2cols[7] &= ~0x02; PS2cols[7] &= ~0x01; } // → R-Alt (0x71)

        // SymShift symbol keys (Profi sym layer): assert SymShift(7,1) + base key
        if (sMinus)  { PS2cols[7] &= ~0x02; PS2cols[6] &= ~0x08; } // Sym+J → '-'
        if (sEqual)  { PS2cols[7] &= ~0x02; PS2cols[6] &= ~0x02; } // Sym+L → '='
        if (sLBrk)   { PS2cols[7] &= ~0x02; PS2cols[5] &= ~0x10; } // Sym+Y → '['
        if (sRBrk)   { PS2cols[7] &= ~0x02; PS2cols[5] &= ~0x08; } // Sym+U → ']'
        if (sBSlash) { PS2cols[7] &= ~0x02; PS2cols[1] &= ~0x04; } // Sym+D → '\'
        if (sSlash)  { PS2cols[7] &= ~0x02; PS2cols[0] &= ~0x10; } // Sym+V → '/'
        if (sColon)  { PS2cols[7] &= ~0x02; PS2cols[0] &= ~0x02; } // Sym+Z → ':'
        if (sQuote)  { PS2cols[7] &= ~0x02; PS2cols[4] &= ~0x08; } // Sym+7 → '\''
        if (sSemi)   { PS2cols[7] &= ~0x02; PS2cols[5] &= ~0x02; } // Sym+O → ';'

        // Profi labels Ctrl/Shift opposite to ZX: cell "Ctrl"=CapsShift(0,0),
        // cell "Shift"=SymShift(7,1). Normal scanner maps phys-Shift→CapsShift
        // and phys-Ctrl→SymShift, so they appear swapped. In Ext mode drive
        // these two bits from the swapped physical keys so cells match labels.
        bool physCtrl  = Kbd->isVKDown(fabgl::VK_LCTRL)  || Kbd->isVKDown(fabgl::VK_RCTRL);
        bool physShift = Kbd->isVKDown(fabgl::VK_LSHIFT) || Kbd->isVKDown(fabgl::VK_RSHIFT);
        // Final authoritative value of the two shift bits, computed from ALL
        // sources at once (the normal scanner's reversed phys-Shift→CapsShift /
        // phys-Ctrl→SymShift is overridden here). A bit is asserted (cleared)
        // if ANY consumer needs it; otherwise released.
        //   CapsShift(0,0): phys Ctrl (swap), arrows, ESC, TAB, CapsLock
        //   SymShift(7,1):  phys Shift (swap), F11, F12, CapsLock, all sym-symbols
        bool anyArrow = kUp || kDown || kLeft || kRight;
        // Disasm: CapsShift-alone→0x74→cell "Ctrl"; SymShift-alone→0x73→cell
        // "Shift". So phys Ctrl → CapsShift, phys Shift → SymShift.
        bool needCaps = physCtrl || anyArrow || kEsc || kTab || kCaps || kBS;
        bool needSym  = physShift || f11 || f12 || kCaps || (altDown && !altLetter)
                      || sMinus || sEqual || sLBrk || sRBrk || sBSlash
                      || sSlash || sColon || sQuote || sSemi
                      || sComma || sPeriod;
        bitWrite(PS2cols[0], 0, !needCaps);
        bitWrite(PS2cols[7], 1, !needSym);

      } else {
        // Not in extended mode — release all extended bits
        for (int i = 0; i < 8; i++) Ports::extPort[i] = 0xFF;
      }
#endif
    }
  }
  if (r) {
    for (uint8_t rowidx = 0; rowidx < 8; rowidx++) {
      Ports::port[rowidx] = PS2cols[rowidx];
    }
  }
}

__not_in_flash("audio") void ESPectrum::BeeperGetSample() {
  uint32_t currentTstates = CPU::tstates;
  uint32_t delta = currentTstates - lastBeeperTstates;
  lastBeeperTstates = currentTstates;

  uint32_t effectiveFP = tstatesPerSampleFP;
  if (multiplicator) effectiveFP <<= multiplicator;

  // Accumulate beeper value weighted by time
  beeperSampleAccum += lastaudioBit * delta;
  beeperTstatesInSample += delta;
  accumulatorFP += (delta << 8);

  // Generate completed output samples
  while (accumulatorFP >= effectiveFP) {
    accumulatorFP -= effectiveFP;
    // Overflow tstates belong to next sample
    uint32_t overflowTstates = accumulatorFP >> 8;
    uint32_t completedAccum = beeperSampleAccum - lastaudioBit * overflowTstates;
    uint32_t completedTstates = beeperTstatesInSample - overflowTstates;
    // Write tstate-weighted average directly
    overSamplebuf[audbufcntover++] = (completedTstates > 0 && completedTstates < 256)
        ? (uint32_t)(completedAccum * beeper_recip[completedTstates]) >> 16 : 0;
    // Carry overflow to next sample
    beeperSampleAccum = lastaudioBit * overflowTstates;
    beeperTstatesInSample = overflowTstates;
  }
}

__not_in_flash("audio") void ESPectrum::CovoxGetSample() {
  if (!audioBufferCovoxL) return;
  uint32_t audbufpos = CPU::tstates / audioCOVOXDivider;
  if (multiplicator)
    audbufpos >>= multiplicator;
  if (audbufpos > audbufcntCovox) {
    uint8_t *sound_buf_l = audioBufferCovoxL + audbufcntCovox;
    uint8_t *sound_buf_r = audioBufferCovoxR + audbufcntCovox;
    int sound_bufsize = audbufpos - audbufcntCovox;
    while (sound_bufsize-- > 0) {
      *sound_buf_l++ = lastCovoxVal;
      *sound_buf_r++ = lastCovoxValR;
    }
    audbufcntCovox = audbufpos;
  }
}

__not_in_flash("audio") void ESPectrum::AYGetSample() {
  uint32_t audbufpos = CPU::tstates / audioAYDivider;
    if (multiplicator) audbufpos >>= multiplicator;
    if (audbufpos > audbufcntAY) {
        chip0.gen_sound(audbufpos - audbufcntAY, audbufcntAY);
        // chip1 only present when TurboSubsys::enabled
    if (Config::turbosound && chip1)
            chip1->gen_sound(audbufpos - audbufcntAY, audbufcntAY);
    audbufcntAY = audbufpos;
  }
}

#if !PICO_RP2040
__not_in_flash("audio") void ESPectrum::SAAGetSample() {
  uint32_t audbufpos = CPU::tstates / audioAYDivider; // SAA counter = 8MHz/256 = 31.25kHz, same rate as AY
  if (multiplicator) audbufpos >>= multiplicator;
  if (audbufpos > audbufcntSAA && saaChip) {
    saaChip->gen_sound(audbufpos - audbufcntSAA, audbufcntSAA);
    audbufcntSAA = audbufpos;
  }
}

__not_in_flash("audio") void ESPectrum::PITGetSample() {
  uint32_t audbufpos = CPU::tstates >> 7; // /128 instead of /112 — fast shift for PIT buffer position
  if (multiplicator)
    audbufpos >>= multiplicator;
  if (audbufpos > audbufcntPIT && audioBufferPIT) {
    Ports::pitGenSound(audioBufferPIT + audbufcntPIT, audbufpos - audbufcntPIT);
    audbufcntPIT = audbufpos;
  }
}
#endif

void ESPectrum::FDDGenSound() {
    // MB-02+ and Betadisk are mutually exclusive, so the active controller's
    // click and LED state feeds the shared fddSound generator.
    rvmWD1793 *ctrl = &fdd;
#if !PICO_RP2040
    if (MB02::enabled) ctrl = &mb02_fdd;
#endif
    uint8_t clicks = ctrl->fdd_clicks;
    ctrl->fdd_clicks = 0;
    if (clicks > 0) {
        if (clicks > 8) clicks = 8;
        fddSound.click_count = clicks;
        fddSound.motor_noise = false;
        int spacing = samplesPerFrame / (clicks + 1);
        for (int c = 0; c < clicks; c++) {
            fddSound.click_pos[c] = spacing * (c + 1);
        }
    } else if (ctrl->fdd_active_decay) {
        // Motor hum while the drive is genuinely spinning/transferring (head-load,
        // header search, real sector/track data movement — see wd1793.cpp). NOT
        // driven by LED::readActive/writeActive(FDD): those also fire on bare
        // WD1793 *command* writes (Ports.cpp counts reg 0 on write for LED colour
        // purposes), so bus-probing software that issues commands without ever
        // moving a real byte would otherwise keep the hum going with no disk
        // rotation/transfer actually happening. Decays once per frame in
        // LED::decay() — shared with the corner lamp and LED indicator glyph.
        fddSound.click_count = 0;
        fddSound.motor_noise = true;
    } else {
        fddSound.click_count = 0;
        fddSound.motor_noise = false;
    }
    fddSound.click_idx = 0;
    fddSound.decay_pos = 12;
}

// === Таймер ===
bool __not_in_flash_func(ESPectrum::AY_timer_callback)(repeating_timer_t *rt) {
  // uint32_t audbufpos = audbufcntAY++;
  // if (multiplicator) audbufpos >>= multiplicator;
  // if (audbufpos > audbufcntAY) {
  //     chip0.gen_sound(audbufpos - audbufcntAY, audbufcntAY);
  //     if (Config::turbosound)
  //         chip1.gen_sound(audbufpos - audbufcntAY, audbufcntAY);
  //     audbufcntAY = audbufpos;
  // }
  // uint8_t chip0Sample[2] = {0,0};
  // uint8_t chip1Sample[2] = {0,0};

  // if (AY_emu) {
  //     if (Config::turbosound != 0 || AySound::selected_chip == 0) {
  //         uint8_t *p0 = chip0.gen_sound();
  //         if (p0) { chip0Sample[0] = p0[0]; chip0Sample[1] = p0[1]; }
  //     }
  //     if (Config::turbosound != 0 || AySound::selected_chip == 1) {
  //         uint8_t *p1 = chip1.gen_sound();
  //         if (p1) { chip1Sample[0] = p1[0]; chip1Sample[1] = p1[1]; }
  //     }
  // }

  // // 4. Смешивание
  // int32_t mix_L = 0;
  // int32_t mix_R = mix_L;

  // if (AY_emu) {
  //     if (Config::turbosound != 0 || AySound::selected_chip == 0) {
  //         mix_L += chip0Sample[0];
  //         mix_R += chip0Sample[1];
  //     }
  //     if (Config::turbosound != 0 || AySound::selected_chip == 1) {
  //         mix_L += chip1Sample[0];
  //         mix_R += chip1Sample[1];
  //     }
  // }

  // // 5. Ограничение значений (для 8-бит)
  // uint8_t out_L = mix_L > 255 ? 255 : (mix_L < 0 ? 0 : mix_L);
  // uint8_t out_R = mix_R > 255 ? 255 : (mix_R < 0 ? 0 : mix_R);

  // pwm_audio_write(&out_L, &out_R, 1, nullptr, 0);

  return true;
}

uint8_t debug_number = 0;

#if !PICO_RP2040 && defined(VGA_HDMI)
extern "C" int hdmi_audio_dbg_stage(void);
extern "C" void hdmi_audio_dbg_stats(uint32_t *q_prod, uint32_t *q_cons, uint32_t *s_prod, uint32_t *s_cons);
#endif
//=======================================================================================
// MAIN LOOP
//=======================================================================================
void ESPectrum::loop() {

  // Check if we're booting into a pending (unconfirmed) video mode
  // Must be here (not in setup) because HDMI DMA starts on core1 after setup returns
  {
      uint8_t pend_hdmi = 0, pend_vga = 0;
      if (Config::loadPendingVideoMode(pend_hdmi, pend_vga)) {
          sleep_ms(500); // Let HDMI stabilize
          if (!OSD::videoModeConfirm(15)) {
              Config::hdmi_video_mode = pend_hdmi;
              Config::vga_video_mode = pend_vga;
              Config::save();
              Config::clearPendingVideoMode();
              OSD::esp_hard_reset();
          } else {
              Config::clearPendingVideoMode();
          }
      }
  }

  // Factory reset: hold R at boot -> confirm -> wipe storage.nvs (+ skip the
  // user's default.nvs this boot) -> reboot to compiled-in defaults.
  // My-Default reset: hold M at boot -> confirm -> wipe storage.nvs only
  // (default.nvs kept) -> reboot, which then falls back to it.
  // Pump the keyboard at FULL SPEED here, BEFORE the emulation for(;;)
  // starts: once it runs, a thrashing machine (Profi DS80 on SPI-PSRAM, ~4 FPS)
  // pumps tuh_task too rarely for USB to even enumerate, so a per-frame check inside
  // the loop never saw the key. R/M read as VK_R/VK_r or VK_M/VK_m depending on CAPSLOCK.
  //
  // Reliability (was ~50/50): the window is now guided and keyboard-aware.
  //  - We poll until the keyboard is actually READY (PS/2: instant; USB: mounted),
  //    then a short grace, instead of a blind fixed timeout that closed before a
  //    slow USB keyboard finished enumerating. Hard cap FR_MAX_US if none appears.
  //  - After FR_PROMPT_DELAY_US we draw a centered "Hold R / Hold M" hint
  //    so the user knows the window is open and holds long enough. A fast PS/2 hold
  //    is caught before the delay, so a normal reset never flashes the prompt.
  {
      extern void repeat_me_for_input();
      auto Kbd = PS2Controller.keyboard();

      // R/M state can't survive a reset (crt0 zeroes .bss, incl. the keyboard's
      // "currently down" bitmap, on every boot) and PS/2 has no "what's held
      // right now" query — only a fresh down-edge after this point sets it.
      // So the window below is the only chance to catch it; keep it generous.
      const uint32_t FR_PROMPT_DELAY_US = 400000;   // fast key-hold skips the prompt
      const uint32_t FR_GRACE_US        = 1000000;   // poll this long once kbd is ready
      const uint32_t FR_MAX_US          = 3500000;   // hard cap if no keyboard appears

      uint32_t fr_t0 = time_us_32();
      uint32_t fr_ready_at = 0;        // elapsed us when the keyboard became available
      bool rHeld = false;
      bool mHeld = false;
      bool promptShown = false;
      Debug::log("factory-reset: probing for held R/M (guided window)");
      for (;;) {
          uint32_t el = (uint32_t)(time_us_32() - fr_t0);
          repeat_me_for_input();       // pump USB (tuh_task) + PS/2 at full speed
          if (Kbd && (Kbd->isVKDown(fabgl::VK_R) || Kbd->isVKDown(fabgl::VK_r))) { rHeld = true; break; }
          if (Kbd && (Kbd->isVKDown(fabgl::VK_M) || Kbd->isVKDown(fabgl::VK_m))) { mHeld = true; break; }

          if (!fr_ready_at) {
#ifdef KBDUSB
              bool ready = usb_keyboard_mounted();
#else
              bool ready = true;       // PS/2 keyboard state is available immediately
#endif
              if (ready) fr_ready_at = el ? el : 1;
          }

          if (!promptShown && el >= FR_PROMPT_DELAY_US) {
              OSD::osdCenteredMsg(MSG_FACTORY_RESET_HOLD[Config::lang], LEVEL_INFO, 0);
              promptShown = true;      // persistent draw; emulation repaint erases it
          }

          if (el >= FR_MAX_US) break;                             // no keyboard ever seen
          if (fr_ready_at && (el - fr_ready_at) >= FR_GRACE_US) break;  // ready + grace, no R/M
          sleep_ms(2);
      }
      if (rHeld) {
          Debug::log("factory-reset: R held -> confirm");
          if (OSD::msgDialog(MSG_FACTORY_RESET_TITLE[Config::lang],
                             MSG_FACTORY_RESET_Q[Config::lang]) == DLG_YES) {
              bool ok = false;
              if (FileUtils::fsMount) {
                  FIL* flag = fopen2(SKIP_DEFAULT_FLAG, FA_WRITE | FA_CREATE_ALWAYS);
                  if (flag) fclose2(flag);
                  ok = (f_unlink(STORAGE_NVS) == FR_OK);
              }
              Debug::log("factory-reset: unlink %s -> reboot", ok ? "OK" : "FAIL");
              OSD::esp_hard_reset();   // never returns; Config::load() then uses compiled defaults
          }
          Debug::log("factory-reset: declined");
      } else if (mHeld) {
          Debug::log("my-default-reset: M held -> confirm");
          if (OSD::msgDialog(MSG_MYDEFAULT_RESET_TITLE[Config::lang],
                             MSG_MYDEFAULT_RESET_Q[Config::lang]) == DLG_YES) {
              bool ok = false;
              if (FileUtils::fsMount) ok = (f_unlink(STORAGE_NVS) == FR_OK);
              Debug::log("my-default-reset: unlink %s -> reboot", ok ? "OK" : "FAIL");
              OSD::esp_hard_reset();   // never returns; Config::load() then falls back to default.nvs
          }
          Debug::log("my-default-reset: declined");
      } else {
          Debug::log("factory-reset: no R/M (continue)");
      }
  }

#if !PICO_RP2040
  // Profi DS80 (hires) on SPI-PSRAM-only boards (no fast butter/QSPI-XIP PSRAM)
  // loads slowly through the slow SPI bus.  When DS80 mode turns on we want a
  // "loading" notice over the "PROFI PLUS" startup screen — but it has to wait
  // until the SYS ROM has actually painted that screen (see the handler after
  // CPU::loop()), otherwise the blocking box freezes a still-black pre-logo frame.
  const bool profi_spi_boot = (Config::arch == "Profi") &&
                              (butter_psram_size() == 0) &&
                              (psram_size() >= (16 << 10));
  bool prev_ds80_active = false;
  uint64_t profi_ds80_msg_at = 0; // 0 = not scheduled; else time_us_64() deadline
  // Delay from DS80-on to showing the box: long enough for the startup screen to
  // be drawn.  Tunable — raise if the box still lands before the logo appears.
  const uint64_t PROFI_DS80_MSG_DELAY_US = 2500000ull;
#endif

  for (;;) {
    if (debug_number != 0) {
      char msg[16];
      snprintf(msg, 16, "%02Xh", debug_number);
      OSD::osdCenteredMsg(msg, LEVEL_WARN, 5000);
      debug_number = 0;
    }
    ts_start = time_us_64();

    if (!CPU::paused) {
      uint64_t _aud_t0 = time_us_64();
      pwm_audio_write((uint8_t *)audioBuffer_L, (uint8_t *)audioBuffer_R,
                      maxSpeed ? 1 : samplesPerFrame, 0, 0);
      g_aud_write_us = (uint32_t)(time_us_64() - _aud_t0);
    } else
      g_aud_write_us = 0;

    // Send audioBuffer to pwmaudio
    audbufcnt = 0;
    audbufcntover = 0;
    audbufcntAY = 0;
    audbufcntCovox = 0;

#if !PICO_RP2040
    audbufcntSAA = 0;
    audbufcntPIT = 0;
#endif

    // Frame boundary: safe to apply pending subsystem (de)allocations.
    // Audio producers and the mixer are quiescent here.
    Subsystems::applyPending();

    lastBeeperTstates = 0;
    accumulatorFP = 0;
    beeperSampleAccum = 0;
    beeperTstatesInSample = 0;

    CPU::loop();

#if !PICO_RP2040
    if (profi_spi_boot) {
      // DS80 turning on (rising edge) schedules the notice; we show it only after
      // PROFI_DS80_MSG_DELAY_US so the startup screen is painted behind it (the box
      // is blocking and pauses rendering — see below — so the logo must be drawn
      // first, with the emulation running, before we freeze it).
      if (profi_ds80_active && !prev_ds80_active)
        profi_ds80_msg_at = time_us_64() + PROFI_DS80_MSG_DELAY_US;
      if (profi_ds80_msg_at && time_us_64() >= profi_ds80_msg_at) {
        // The notice MUST be the blocking variant (millispause>0): the framebuffer
        // is single-buffered and the centre is re-rendered every frame during
        // active scan, so a non-blocking centre draw races the renderer and never
        // shows.  The blocking path sleeps with rendering paused so it stays put.
        // When DS80 is still active, wrap it in the same palette guard the menu's
        // DS80Guard uses, so the standard OSD colour bytes render correctly over
        // the DS80 packed-pair framebuffer instead of as striped garbage.  (If the
        // startup screen turns out to be standard mode by now, skip the guard.)
        const bool ds80 = profi_ds80_active;
        if (ds80) {
          VIDEO::profi_ds80_osd_active = true;
          VIDEO::applyProfiOSDPalette();
        }
        //OSD::osdCenteredMsg(OSD_PROFI_LOADING[Config::lang], LEVEL_WARN, 2500);
        if (ds80) {
          VIDEO::profi_ds80_osd_active = false;
          if (profi_ds80_active) {
            VIDEO::restoreProfiLivePalette();
            VIDEO::clearDS80Padding();
          }
        }
        profi_ds80_msg_at = 0; // one-shot per DS80 activation
      }
      prev_ds80_active = profi_ds80_active;
    }

    if (ZiFi::enabled) ZiFi::tick();
    RTC::flushNVRAM(); // persist CMOS NVRAM to SD when dirty (debounced)
    Ports::serialMouseTick(); // arm the COM-mouse RST20H when movement queued

    // Auto-sync the RTC over SNTP at startup when both ZiFi and RTC are on and a
    // WiFi network is configured. Runs entirely in the background (non-blocking
    // state machine, no OSD) so it never freezes audio/video. Kicked off ~4s into
    // the run so the ESP has time to auto-reconnect; then stepped each loop tick.
    static bool     rtc_autosync_begun = false;
    static uint32_t rtc_autosync_at    = 0;
    // Reconnect WiFi at boot whenever WiFi is enabled and an SSID is saved. This is
    // driven ONLY by the WiFi switch — the NIC is no longer a trigger (it used to
    // pull WiFi up as a side effect via `ZiFi::enabled && rtc_enabled`, which is
    // exactly the leak that made FTP/SSH work only with the NIC on). The background
    // state machine also runs SNTP, harmless when RTC is off.
    if (!Config::wifi_ssid.empty() && Config::wifi_enabled) {
        if (!rtc_autosync_begun) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (rtc_autosync_at == 0) rtc_autosync_at = now + 4000;
            else if (now >= rtc_autosync_at) {
                rtc_autosync_begun = true;
                // Profi heap guard (decided once, replaces the old blanket
                // `arch != "Profi"` exclusion that left the ROMain/PQDOS clock
                // permanently at 00.00.00): autoSyncBegin brings the ESP link up
                // (ZiFi::init allocs the 8 KB RX ring + TX FIFO on the heap) —
                // fine on butter-PSRAM Profi (~68 KB free with CDC up), but the
                // SPI-PSRAM m1p2 Profi runs with ~10 KB free and OOMs (see
                // profi_zifi_oom_fix). Skip only when the headroom isn't there.
                if (Config::arch != "Profi" || getLargestAllocatable() >= 16384)
                    ZiFiAT::autoSyncBegin(Config::wifi_ssid, Config::wifi_pass, Config::wifi_tz);
            }
        } else {
            ZiFiAT::autoSyncPoll(); // no-op unless autoSyncBegin actually ran
        }
    }
#endif

    // SD automount: when the machine booted with no card (fsMount==false, and no
    // USB stick took over as root), probe periodically for one being inserted.
    // On the tick it comes online we mount it live — the OSD menus and file
    // dialogs gate on fsMount at render time, so they light up without a reboot.
    // We deliberately DON'T reload Config here: video-mode / arch settings from
    // the card can only be applied by a reboot, so live use keeps the RAM
    // defaults and only enables file access + the remembered disk mounts.
    if (!FileUtils::fsMount && !FileUtils::usbRoot) {
        static uint64_t sd_probe_at = 0;   // next allowed probe (throttle)
        uint64_t now = time_us_64();
        if (now >= sd_probe_at) {
            sd_probe_at = now + 2000000ull; // ~2 s between probes (each is a few ms)
            if (FileUtils::automountSD()) {
                Config::loadDiskMounts();   // restore remembered disk images
                Tape::LoadRemembered();     // and the remembered tape
                OSD::osdCenteredMsg(MSG_SD_AUTOMOUNT[Config::lang], LEVEL_INFO, 1500);
            }
        }
    }

    // GS-Z80 runs on core1 alongside pcm_call(); core0 only reads the ring.

    // Профилирование AY (только для отладки - закомментируйте после)
    // static uint64_t ay_total = 0, ay_count = 0;
    // uint64_t ay_start = time_us_64();

    // Process audio buffer
    uint64_t _mix_t0 = time_us_64();
    faudbufcnt = audbufcnt;
    faudioBit = lastaudioBit;
    faudbufcntAY = audbufcntAY;
    faudbufcntCovox = audbufcntCovox;

#if !PICO_RP2040
    faudbufcntPIT = audbufcntPIT;
    faudbufcntSAA = audbufcntSAA;
#endif

    if (!CPU::paused) {
#if LOAD_WAV_PIO
      if (Config::real_player) {
        if (Tape::tapeStatus != TAPE_LOADING) { // W/A
          Tape::tapeStatus = TAPE_LOADING;
          Tape::tapeFileType = TAPE_FTYPE_EMPTY;
          Tape::tapeFileName = "REAL AUDIO";
          TapeNameScroller = 0;
          Tape::tapeCurBlock = 0;
          Tape::tapeNumBlocks = 1;
          Tape::tapebufByteCount = 0;
          Tape::tapePlayOffset = 0;
          Tape::tapeFileSize = 100;
        }
        pwm_audio_in_frame_started();
      }
#endif
      int32_t t_us = Config::throtling * 1000l;
      if ((!t_us || idle > t_us) && !(maxSpeed && Tape::tapeStatus == TAPE_LOADING)) {
        // Finish fill of beeper audio buffer (tstate-weighted)
        if (beeperTstatesInSample > 0 && audbufcntover < (uint32_t)samplesPerFrame) {
          // Complete partial sample with constant beeper value
          uint32_t tstatesPerSampleInt = tstatesPerSampleFP >> 8;
          if (beeperTstatesInSample < tstatesPerSampleInt) {
            uint32_t remaining = tstatesPerSampleInt - beeperTstatesInSample;
            beeperSampleAccum += faudioBit * remaining;
            beeperTstatesInSample += remaining;
          }
          overSamplebuf[audbufcntover++] = beeperTstatesInSample < 256
              ? (uint32_t)(beeperSampleAccum * beeper_recip[beeperTstatesInSample]) >> 16
              : beeperSampleAccum / beeperTstatesInSample;
          beeperSampleAccum = 0;
          beeperTstatesInSample = 0;
        }
        // Fill remaining samples with constant beeper value
        while (audbufcntover < (uint32_t)samplesPerFrame) {
          overSamplebuf[audbufcntover++] = faudioBit;
        }
        if (Tape::tapeStatus != TAPE_LOADING) {
          // Smooth beeper buffer + detect constant DC in single pass
          static uint32_t dc_fade_q8 = 256u;
          uint8_t v0 = overSamplebuf[0];
          uint8_t prev = v0;
          bool is_const = true;
          for (int i = 1; i < samplesPerFrame; i++) {
            uint8_t curr = overSamplebuf[i];
            if (curr != v0) is_const = false;
            overSamplebuf[i] = ((uint32_t)prev + curr + 1) >> 1;
            prev = curr;
          }
          if (is_const && v0 > 0) {
            if (dc_fade_q8 >= 26u) dc_fade_q8 -= 26u; else dc_fade_q8 = 0u;
            uint8_t faded = (v0 * dc_fade_q8) >> 8;
            for (int i = 0; i < samplesPerFrame; i++)
              overSamplebuf[i] = faded;
          } else if (!is_const) {
            dc_fade_q8 = 256u;
          }
        }
        if (CovoxSubsys::enabled && (Config::covox || Config::soundriveEnabled()) && faudbufcntCovox < samplesPerFrame) {
          uint8_t *sound_buf_l = audioBufferCovoxL + faudbufcntCovox;
          uint8_t *sound_buf_r = audioBufferCovoxR + faudbufcntCovox;
          int sound_bufsize = samplesPerFrame - faudbufcntCovox;
          while (sound_bufsize-- > 0) {
            *sound_buf_l++ = lastCovoxVal;
            *sound_buf_r++ = lastCovoxValR;
          }
        }
#if !PICO_RP2040
        // KR580VI53 (8253 PIT) — complete buffer for remaining frame
        if (PitSubsys::enabled && Z80Ops::isByte && faudbufcntPIT < samplesPerFrame) {
          Ports::pitGenSound(audioBufferPIT + faudbufcntPIT,
                             samplesPerFrame - faudbufcntPIT);
        }
#endif
        {
            bool fddSndEnabled = (Config::trdosSoundLed & 2) != 0;
#if !PICO_RP2040
            if (MB02::enabled) fddSndEnabled = (Config::mb02SoundLed & 2) != 0;
#endif
            if (fddSndEnabled) FDDGenSound();
        }
        if (AY_emu && faudbufcntAY < samplesPerFrame) {
            if(Config::turbosound != 0 || AySound::selected_chip == 0) chip0.gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);
            if((Config::turbosound != 0 || AySound::selected_chip == 1) && chip1) chip1->gen_sound(samplesPerFrame - faudbufcntAY , faudbufcntAY);
        }
#if !PICO_RP2040
        if (SaaSubsys::enabled && saaChip && faudbufcntSAA < samplesPerFrame)
        {
          if (Tape::tapeStatus == TAPE_LOADING) {
            memset(saaChip->SamplebufSAA_L, 0, sizeof(saaChip->SamplebufSAA_L));
            memset(saaChip->SamplebufSAA_R, 0, sizeof(saaChip->SamplebufSAA_R));
          } else {
            saaChip->gen_sound(samplesPerFrame - faudbufcntSAA, faudbufcntSAA);
          }
        }
        if (MidiSubsys::enabled && audioBufferMIDI_L && audioBufferMIDI_R)
        {
          if (Midi::enabled == 3)
            SoftSynth::gen_sound(audioBufferMIDI_L, audioBufferMIDI_R, samplesPerFrame);
          else if (Midi::enabled == 4)
            MidiSynth::gen_sound(audioBufferMIDI_L, audioBufferMIDI_R, samplesPerFrame);
        }
#endif
        // Hoist frame-invariant source flags outside the mix loop
        bool mix_chip0 = AY_emu && (Config::turbosound != 0 || AySound::selected_chip == 0);
        bool mix_chip1 = AY_emu && (Config::turbosound != 0 || AySound::selected_chip == 1) && TurboSubsys::enabled && chip1;
        bool mix_covox = CovoxSubsys::enabled && audioBufferCovoxL;
#if !PICO_RP2040
        bool mix_saa = SaaSubsys::enabled && saaChip;
        bool mix_midi = MidiSubsys::enabled && (Midi::enabled == 3 || Midi::enabled == 4) && audioBufferMIDI_L && audioBufferMIDI_R;
        bool mix_pit = PitSubsys::enabled && audioBufferPIT;
#endif
        bool fddSndEnabledMix = (Config::trdosSoundLed & 2) != 0;
#if !PICO_RP2040
        if (MB02::enabled) fddSndEnabledMix = (Config::mb02SoundLed & 2) != 0;
#endif
        bool mix_fdd = fddSndEnabledMix && (fddSound.click_count > 0 || fddSound.motor_noise);
        for (int i = 0; i < samplesPerFrame; i++)
        {
          int beeper_L = overSamplebuf[i];
#if !PICO_RP2040
          if (mix_pit) beeper_L += audioBufferPIT[i];
#endif
          if (mix_fdd) beeper_L += getFDDSample(i);
          int beeper_R = beeper_L;
          if (mix_covox) {
            beeper_L += audioBufferCovoxL[i];
            beeper_R += audioBufferCovoxR[i];
          }
          if (mix_chip0) {
            beeper_L += chip0.SamplebufAY_L[i];
            beeper_R += chip0.SamplebufAY_R[i];
          }
          if (mix_chip1) {
            beeper_L += chip1->SamplebufAY_L[i];
            beeper_R += chip1->SamplebufAY_R[i];
          }
#if !PICO_RP2040
          if (mix_saa) {
            beeper_L += saaChip->SamplebufSAA_L[i];
            beeper_R += saaChip->SamplebufSAA_R[i];
          }
          if (mix_midi) {
            // Wavetable synth output is unipolar, centered at 128 (see MidiSynth::gen_sound);
            // summed like the beeper/AY, its DC is removed downstream by pwm_audio.
            beeper_L += audioBufferMIDI_L[i];
            beeper_R += audioBufferMIDI_R[i];
          }
#endif
          // GS is mixed live in the audio timer IRQ (pcm_call_inner),
          // not here — burst-sampling on core0 would time-compress it.
          audioBuffer_L[i] = beeper_L > 255 ? 255 : (beeper_L < 0 ? 0 : beeper_L);
          audioBuffer_R[i] = beeper_R > 255 ? 255 : (beeper_R < 0 ? 0 : beeper_R);
        }
      }
    }
    g_mix_us = (uint32_t)(time_us_64() - _mix_t0);
    {
      uint64_t _kbd_t0 = time_us_64();
      processKeyboard();
      g_kbd_us = (uint32_t)(time_us_64() - _kbd_t0);
    }
#ifdef USE_GS
    GS::pollPerf();
#endif
    // Update stats every 50 frames
    if (VIDEO::OSD && VIDEO::framecnt >= 10) {
      if (VIDEO::OSD & 0x04) {
        // printf("Vol. OSD out -> Framecnt: %d\n", VIDEO::framecnt);
        if (VIDEO::framecnt >= 100) {
          VIDEO::OSD &= 0xfb;
          if (ESPectrum::vol_changed) {
            ESPectrum::vol_changed = false;
            Config::save();
          }
          if (VIDEO::OSD == 0) {
            if (Config::aspect_16_9)
              VIDEO::Draw_OSD169 = VIDEO::MainScreen;
            else
              VIDEO::Draw_OSD43 = VIDEO::BottomBorder;
            VIDEO::brdnextframe = true;
          }
        }
      }
      if ((VIDEO::OSD & 0x04) == 0 && !CPU::paused) {
        if (VIDEO::OSD == 1 && Tape::tapeStatus == TAPE_LOADING) {
          snprintf(
              OSD::stats_lin1, sizeof(OSD::stats_lin1), " %-12s %04d/%04d ",
              Tape::tapeFileName.substr(0 + ESPectrum::TapeNameScroller, 12)
                  .c_str(),
              Tape::tapeCurBlock + 1, Tape::tapeNumBlocks);
          float percent =
              (float)((Tape::tapebufByteCount + Tape::tapePlayOffset) * 100) /
              (float)Tape::tapeFileSize;
          snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2),
                   " %05.2f%% %07d%s%07d ", percent,
                   Tape::tapebufByteCount + Tape::tapePlayOffset, "/",
                   Tape::tapeFileSize);
          if ((++ESPectrum::TapeNameScroller + 12) >
              Tape::tapeFileName.length())
            ESPectrum::TapeNameScroller = 0;
          OSD::drawStats();
        } else if (VIDEO::OSD == 2) {
          snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1),
                   "TST: %05d / IDL: %05d ", CPU::tstates_active,
                   (int)(ESPectrum::idle));
          snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2),
                   "FPS:%6.2f / FND:%6.2f ",
                   VIDEO::framecnt / (ESPectrum::totalseconds / 1000000),
                   VIDEO::framecnt /
                       (ESPectrum::totalsecondsnodelay / 1000000));
          OSD::drawStats();
        } else if (VIDEO::OSD == 3) {
          snprintf(OSD::stats_lin1, sizeof(OSD::stats_lin1),
                   "TST: %05d / IDL: %05d ", CPU::tstates_active,
                   (int)(ESPectrum::idle));

#if !PICO_RP2040
          if (MB02::enabled) {
            snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2),
                     "MB02 TR:#%02X/SEC:#%02X/S:%d ",
                     ESPectrum::mb02_fdd.track, ESPectrum::mb02_fdd.sector,
                     ESPectrum::mb02_fdd.side);
            OSD::drawStats();
          } else
#endif
          {
            snprintf(OSD::stats_lin2, sizeof(OSD::stats_lin2),
                    "ST:%-6sTR:#%02X/SEC:#%02X ",
                    rvmWD1793StepStateName(&ESPectrum::fdd).c_str(),
                    ESPectrum::fdd.track, ESPectrum::fdd.sector);
            OSD::drawStats();
          }
        }
        totalseconds = 0;
        totalsecondsnodelay = 0;
        VIDEO::framecnt = 0;
      }
    }
#if !PICO_RP2040
    // DS80: the stats overlay (F8) is only refreshed every 10 frames above, but the
    // DS80 scan-time renderer + per-frame border fill repaint the whole framebuffer
    // every frame — so the stats text would show for 1 frame then vanish for 9
    // (flicker).  Re-draw the cached stats lines every frame while DS80 is active so
    // they persist.  (Normal modes draw into the static border area, no flicker.)
    if (profi_ds80_active && (VIDEO::OSD & 0x03) && (VIDEO::OSD & 0x04) == 0 && !CPU::paused)
      OSD::drawStats();
    // Same flicker as above, but for the F9/F10 volume box (OSD bit 0x04):
    // it's only (re)drawn on key-press/timeout, so DS80's per-frame repaint
    // erases it after a single frame. Keep it pinned while it's showing.
    if (profi_ds80_active && (VIDEO::OSD & 0x04))
      OSD::drawVolumeBox();
#endif
    // Flashing flag change (disabled when ULA+ palette is active)
#if !PICO_RP2040
    if (!(VIDEO::flash_ctr++ & 0x0f) && !VIDEO::ulaplus_enabled)
#else
    if (!(VIDEO::flash_ctr++ & 0x0f))
#endif
      VIDEO::flashing ^= 0x80;

    // Draw fdd led indicator in top-right corner
    bool hasFdd = ((Z80Ops::isPentagon || Z80Ops::isProfi) || (Z80Ops::is128 && Z80Ops::isByte)) && Tape::tapeStatus != TAPE_LOADING
#if !PICO_RP2040
        && !DivMMC::enabled
#endif
        ;
    // Indicator sits at x=312 — inside the DS80 right border band.  The "off" state
    // erases it to the surrounding border colour so no square remains.
    //   Normal mode: border byte = zxColor(borderColor, 0).
    //   DS80 mode:   the band is filled with Palette[(~borderColor)&7] (inverse index,
    //     per ProfiRenderer).  The Graphics remap maps a ZX index i → Palette[i&0xF],
    //     so pass (~borderColor)&7 to land on the same byte and blend cleanly.
    uint8_t led_off_col = zxColor(VIDEO::borderColor, 0);
#if !PICO_RP2040
    if (profi_ds80_active) led_off_col = (uint8_t)(~VIDEO::borderColor) & 0x07;
#endif
    // Corner FDD lamp. ON/OFF follows rvmWD1793::fdd_active_decay — genuine
    // head-load/header-search/data-transfer activity, decremented once per frame by
    // LED::decay() (auto-clears; unlike the old rvmWD1793::led it can't stick on, and
    // unlike LED::readActive/writeActive(FDD) it isn't fooled by a bare command write
    // with no real disk access — see wd1793.h). COLOUR by the actual WD1793 command —
    // write-sector/track → red, else (read/seek) → blue.
    rvmWD1793 *fctrl = &fdd;
#if !PICO_RP2040
    if (MB02::enabled) fctrl = &mb02_fdd;
#endif
    bool fdd_active = fctrl->fdd_active_decay != 0;
    bool fdd_write  = ((fctrl->command & 0xE0) == 0xA0) ||   // Write Sector (0xA_/0xB_)
                      ((fctrl->command & 0xF0) == 0xF0);     // Write Track  (0xF_)
    // Foreground = lamp colour when active, else the border colour so the diskette
    // glyph vanishes into the border when idle. Reuse LEDIndicators' 8x8 diskette
    // sprite (instead of a plain square) so the corner lamp matches the border row.
    // drawGlyph paints the full 8x8 each frame (fg/bg), so it self-erases.
    uint8_t fdd_fg = !fdd_active ? led_off_col
                   : fdd_write   ? zxColor(2, 1)             // red  — write
                                 : zxColor(1, 1);            // blue — read / seek
#if !PICO_RP2040
    if (MB02::enabled && (Config::mb02SoundLed & 1)) {
        LED::drawGlyph(LED::FDD, 311, 2, fdd_fg, led_off_col);
    } else
#endif
    if (hasFdd && (Config::trdosSoundLed & 1)) {
        LED::drawGlyph(LED::FDD, 311, 2, fdd_fg, led_off_col);
    }

    elapsed = time_us_64() - ts_start;
    idle = target - elapsed;

#if defined(USE_GS) && GS_PERF_TRACE
    // Track min per-frame IDL across the current pollPerf interval — lets
    // us correlate worst-case host stalls with concurrent GS-side activity.
    extern volatile int32_t gs_perf_idle_min;
    extern volatile uint32_t gs_perf_idle_neg_frames;
    extern volatile uint32_t gs_perf_frames;
    int32_t i32 = (int32_t)idle;
    if (i32 < gs_perf_idle_min) gs_perf_idle_min = i32;
    if (i32 < 0) gs_perf_idle_neg_frames++;
    gs_perf_frames++;
#endif

#if SND_PORT_TRACE
    {
      static uint32_t snd_trace_frames = 0;
      if (++snd_trace_frames >= 250) {
        snd_trace_frames = 0;
        Ports::sndTraceDump();
      }
    }
#endif

#if !PICO_RP2040
    // Negative-IDL attribution (Profi): track the worst frame of every
    // 60-frame window and, when at least one frame overran the target, log a
    // breakdown of where its time went.  Counter deltas are robust to the
    // per-frame resets a PERF_TRACE build performs (cur < prev → external
    // reset → the current value IS the delta).
    if (Z80Ops::isProfi) {
      extern volatile uint32_t cpu_frame_us, fdd_step_us, endframe_us;
      extern volatile uint32_t g_frame_swap_us, g_frame_swap_idle_us, g_frame_accb;
      extern volatile uint32_t g_aud_write_us;
      static uint32_t p_cpu = 0, p_fdd = 0, p_ports = 0, p_pcalls = 0;
      uint32_t c;
      c = cpu_frame_us;        uint32_t d_cpu   = (c >= p_cpu)   ? c - p_cpu   : c; p_cpu = c;
      c = fdd_step_us;         uint32_t d_fdd   = (c >= p_fdd)   ? c - p_fdd   : c; p_fdd = c;
      c = Ports::fdd_ports_us; uint32_t d_ports = (c >= p_ports) ? c - p_ports : c; p_ports = c;
      c = Ports::fdd_ports_calls; uint32_t d_pcalls = (c >= p_pcalls) ? c - p_pcalls : c; p_pcalls = c;
      uint32_t pmax_win = Ports::fdd_ports_max; Ports::fdd_ports_max = 0;
      static uint32_t neg_cnt = 0, frame_cnt = 0;
      static int32_t  w_idle = INT32_MAX;
      static uint32_t w_el = 0, w_cpu = 0, w_fdd = 0, w_ports = 0, w_pcalls = 0;
      static uint32_t w_swap = 0, w_swapidle = 0, w_accb = 0;
      static uint32_t w_ef = 0, w_aud = 0, w_kbd = 0, w_pmax = 0, w_mix = 0;
      if ((int32_t)idle < w_idle) {
        w_idle = (int32_t)idle;   w_el = (uint32_t)elapsed;
        w_cpu = d_cpu;            w_fdd = d_fdd;   w_ports = d_ports;
        w_pcalls = d_pcalls;
        w_swap = g_frame_swap_us; w_swapidle = g_frame_swap_idle_us;
        w_accb = g_frame_accb;
        w_ef = endframe_us;       w_aud = g_aud_write_us;
        w_kbd = g_kbd_us;         w_mix = g_mix_us;
      }
      if (pmax_win > w_pmax) w_pmax = pmax_win;
      if (idle < 0) neg_cnt++;
      if (++frame_cnt >= 60) {
        // ef is inside cpu (EndFrame: DS80 border flush + the [SPI] print —
        // Debug::log over USB-CDC can cost ms, so a worst frame with big ef
        // and zero everything else is usually the diagnostics frame itself);
        // aud = pwm_audio_write DMA wait; kbd = processKeyboard;
        // post = el - cpu - aud - kbd (OSD stats/LED/ZiFi/RTC…);
        // pmax = longest single WD stepping call in the window.
        if (neg_cnt)
          Debug::log("[NEG2] 60f: neg=%u worst: idle=%d el=%u cpu=%u ef=%u aud=%u mix=%u kbd=%u post=%d fdd=%u ports=%u/%u pmax=%u swap=%u(idle %u) accb=%u",
                     neg_cnt, w_idle, w_el, w_cpu, w_ef, w_aud, w_mix, w_kbd,
                     (int)(w_el - w_cpu - w_aud - w_mix - w_kbd),
                     w_fdd, w_ports, w_pcalls, w_pmax,
                     w_swap, w_swapidle, w_accb);
        frame_cnt = 0; neg_cnt = 0; w_idle = INT32_MAX; w_pmax = 0;
      }
    }
#if FDD_PORT_TRACE
    // [FDC IDLE]: one-shot marker for "disk loading stopped here" — fires the
    // first time 3 consecutive 60-frame windows (~3-4s) pass with no new WD1793
    // command after having seen at least one, so a boot-load hang shows exactly
    // which track/sector/side/PC the last accepted command was, instead of
    // requiring a manual scan of every [FDC CMD]/[FDC RD-END] line by hand.
    // Re-arms if disk activity resumes and later stops again.
    if (Z80Ops::isProfi) {
      extern uint32_t g_fdcCmdCount;
      extern uint16_t g_fdcLastTrk, g_fdcLastPc;
      extern uint8_t  g_fdcLastSec, g_fdcLastSide, g_fdcLastCmd;
      static uint32_t p_fdcCmdCount = 0;
      static uint32_t idleWindows = 0;
      static bool reported = false;
      if (g_fdcCmdCount != p_fdcCmdCount) {
        p_fdcCmdCount = g_fdcCmdCount;
        idleWindows = 0;
        reported = false;
      } else if (g_fdcCmdCount > 0 && !reported) {
        if (++idleWindows >= 3) {
          reported = true;
          Debug::log("[FDC IDLE] no new WD1793 command for ~%u frames (total cmds=%u); "
                     "last: cmd=%02X trk=%d sec=%d side=%d pc=%04X",
                     idleWindows * 60, g_fdcCmdCount, g_fdcLastCmd, g_fdcLastTrk,
                     g_fdcLastSec, g_fdcLastSide, g_fdcLastPc);
        }
      }
    }
#endif
    // Deferred WD1793 SD I/O (track loads, PRO flush/f_sync) runs inside this
    // frame's idle window, so disk operations stop eating frame time (negative
    // IDL on Profi CP/M disk ops).  g_wdDeferLoads is refreshed EVERY frame,
    // including maxSpeed ones — a stale 'true' with no idle runner would leave
    // every track load waiting for wdTrackReady's in-frame fallback.
    g_wdDeferLoads = !maxSpeed && Z80Ops::isProfi;
    // Deferred pool promotions (butter accessor banks): allow 1 inline
    // promotion per frame; the rest queue for the idle window below.  On
    // maxSpeed there is no idle window — run everything inline as before.
    MemESP::promoFrameReset(maxSpeed ? 255 : 1);
    // Run the I/O hook also when a deferred track load is pending even with
    // no idle budget: in negative-IDL streaks the guest is FROZEN on that
    // load, and wdIdleIO's overdue escape must get a chance to run it (else
    // it waits for the 100 ms in-frame fallback — long stall AND the same
    // blocking cost).
    if (!maxSpeed && (idle > 3000 || fdd.trackLoadPending)) {
      uint64_t io_deadline = (uint64_t)(ts_start + target - 1200);
      wdIdleIO(&fdd, io_deadline);
      if (idle > 3000) MemESP::idleService(io_deadline);
      // The I/O consumed part of the wait budget — re-derive the remaining
      // idle for the pacing below (stats above keep the pre-I/O values).
      int64_t rem = target - (int64_t)(time_us_64() - ts_start);
      idle = rem > 0 ? rem : 0;
    }
#endif

    totalsecondsnodelay += elapsed;

    if (!maxSpeed) {
      // ZiFi over USB-CDC: the frame-pacing waits below are the longest no-pump
      // windows in the whole loop (up to ~13 ms busy-NOP) — long enough for a
      // mid-burst +IPD to overflow the CH340's ~256 B internals (hw: MRF's gopher
      // page truncated at the same offset every run). Keep servicing the host
      // stack while we wait; cdcPump() self-limits to ~1 kHz and is a no-op on
      // the GPIO UART transport.
      if (Config::v_sync_enabled) {
        for (;;) {
#if !PICO_RP2040
          if (ZiFi::cdcNicActive) ZiFi::cdcPump();
#endif
          if (v_sync) {
            v_sync = false;
            break;
          }
        }
#if !PICO_RP2040
        // Blanking just started (v_sync fires at scanout line v_active): apply
        // any pending Profi DS80 palette refresh now, while the scanout DMA is
        // off-screen — tear-free palette animation (see profiPaletteApplyPending).
        VIDEO::profiPaletteApplyPending();
#endif
      } else {
        if (idle > 0) {
#if !PICO_RP2040
          if (ZiFi::cdcNicActive) {
            int64_t e = (int64_t)time_us_64() + idle;
            while ((int64_t)time_us_64() < e) ZiFi::cdcPump();
          } else
#endif
          {
            delayMicroseconds(idle);
          }
        }
      }
    }
    totalseconds += time_us_64() - ts_start;
  }
}
