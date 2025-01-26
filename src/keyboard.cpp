/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.


* Please contact fdivitto2013@gmail.com if you need a commercial license.


* This library and related software is available under GPL v3.

  FabGL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FabGL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FabGL.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "keyboard.h"
#ifdef KBDUSB
    #include "ps2kbd_mrmltr.h"
#else
    #include "ps2.h"
#endif
#include <hardware/timer.h>
#include "Config.h"
#include "ESPectrum.h"

#pragma GCC optimize ("O2")

void repeat_me_for_input();

namespace fabgl {

Keyboard::Keyboard()
  : m_keyboardAvailable(false),
    m_virtualKeyQueue(),
    m_lastDeadKey(VK_NONE),
    m_codepage(nullptr)
{
}

Keyboard::~Keyboard()
{
  enableVirtualKeys(false, false);
}

void Keyboard::begin(bool generateVirtualKeys, bool createVKQueue, int PS2Port, bool doReset)
{
///  PS2Device::begin(PS2Port);
  m_CTRL       = false;
  m_LALT       = false;
  m_RALT       = false;
  m_SHIFT      = false;
  m_CAPSLOCK   = false;
  m_GUI        = false;
  m_NUMLOCK    = false;
  m_SCROLLLOCK = false;

///  m_uiApp = nullptr;
  reset(doReset);
  enableVirtualKeys(generateVirtualKeys, createVKQueue);
}

void Keyboard::enableVirtualKeys(bool generateVirtualKeys, bool createVKQueue)
{
  if (createVKQueue)
    generateVirtualKeys = true;
}

uint8_t m_VKMap[(int)(VK_LAST + 7) / 8];

// reset keyboard, set scancode 2 and US layout
bool Keyboard::reset(bool sendCmdReset)
{
  memset(m_VKMap, 0, sizeof(m_VKMap));

  // sets default layout
///  setLayout(&USLayout);

  if (sendCmdReset) {

    // 350ms keyboard poweron delay (look at NXP M68HC08 designer reference manual)
///    vTaskDelay(350 / portTICK_PERIOD_MS);

    // tries up to three times to reset keyboard
    for (int i = 0; i < 3; ++i) {
///      m_keyboardAvailable = send_cmdReset();
      if (m_keyboardAvailable)
        break;
///      vTaskDelay(350 / portTICK_PERIOD_MS);
    }
    // give the time to the device to be fully initialized
///    vTaskDelay(200 / portTICK_PERIOD_MS);

  } else 
    m_keyboardAvailable = true;

///  send_cmdSetScancodeSet(2);

  return m_keyboardAvailable;
}


bool Keyboard::setLEDs(bool numLock, bool capsLock, bool scrollLock) {
///  uint8_t v = 0;
///  if (numLock) v |= PS2_LED_NUM_LOCK;
///  if (capsLock) v |= PS2_LED_CAPS_LOCK;
///  if (scrollLock) v |= PS2_LED_SCROLL_LOCK;
///  keyboard_toggle_led(v);
  return true;
}

void Keyboard::getLEDs(bool * numLock, bool * capsLock, bool * scrollLock) {
///  uint8_t v = get_led_status();
///  *numLock    = v & PS2_LED_NUM_LOCK;
///  *capsLock   = v & PS2_LED_CAPS_LOCK;
///  *scrollLock = v & PS2_LED_SCROLL_LOCK;
}

int Keyboard::scancodeAvailable()
{
  return dataAvailable();
}

// -1 = virtual key cannot be translated to ASCII
int Keyboard::virtualKeyToASCII(VirtualKey virtualKey)
{
  VirtualKeyItem item;
  item.vk         = virtualKey;
  item.down       = true;
  item.CTRL       = m_CTRL;
  item.LALT       = m_LALT;
  item.RALT       = m_RALT;
  item.SHIFT      = m_SHIFT;
  item.GUI        = m_GUI;
  item.CAPSLOCK   = m_CAPSLOCK;
  item.NUMLOCK    = m_NUMLOCK;
  item.SCROLLLOCK = m_SCROLLLOCK;
  return fabgl::virtualKeyToASCII(item, m_codepage);
}

VirtualKey Keyboard::manageCAPSLOCK(VirtualKey vk)
{
  if (m_CAPSLOCK) {
    // inverts letters case
    if (vk >= VK_a && vk <= VK_z)
      vk = (VirtualKey)(vk - VK_a + VK_A);
    else if (vk >= VK_A && vk <= VK_Z)
      vk = (VirtualKey)(vk - VK_A + VK_a);
  }
  return vk;
}

void Keyboard::injectVirtualKey(VirtualKeyItem const & item, bool insert)
{
  // update m_VKMap
  if (item.down)
    m_VKMap[(int)item.vk >> 3] |= 1 << ((int)item.vk & 7);
  else
    m_VKMap[(int)item.vk >> 3] &= ~(1 << ((int)item.vk & 7));

  // has VK queue? Insert VK into it.
///  if (m_virtualKeyQueue) {
///    auto ticksToWait = (m_uiApp ? 0 : portMAX_DELAY);  // 0, and not portMAX_DELAY to avoid uiApp locks
    m_virtualKeyQueue.push(item);
///    if (insert)
///      xQueueSendToFront(m_virtualKeyQueue, &item, ticksToWait);
///    else
///      xQueueSendToBack(m_virtualKeyQueue, &item, ticksToWait);
///  }
}


void Keyboard::injectVirtualKey(VirtualKey virtualKey, bool keyDown, bool insert)
{
  VirtualKeyItem item;
  item.vk          = virtualKey;
  item.down        = keyDown;
  item.scancode[0] = 0;  // this is a manual insert, not scancode associated
  item.ASCII       = virtualKeyToASCII(virtualKey);
  item.CTRL        = m_CTRL;
  item.LALT        = m_LALT;
  item.RALT        = m_RALT;
  item.SHIFT       = m_SHIFT;
  item.GUI         = m_GUI;
  item.CAPSLOCK    = m_CAPSLOCK;
  item.NUMLOCK     = m_NUMLOCK;
  item.SCROLLLOCK  = m_SCROLLLOCK;
  injectVirtualKey(item, insert);
}


// inject a virtual key item into virtual key queue calling injectVirtualKey() and into m_uiApp
void Keyboard::postVirtualKeyItem(VirtualKeyItem const & item)
{
  // add into m_virtualKeyQueue and update m_VKMap
  injectVirtualKey(item, false);
/**
  // need to send events to uiApp?
  if (m_uiApp) {
    uiEvent evt = uiEvent(nullptr, item.down ? UIEVT_KEYDOWN : UIEVT_KEYUP);
    evt.params.key.VK    = item.vk;
    evt.params.key.ASCII = item.ASCII;
    evt.params.key.LALT  = item.LALT;
    evt.params.key.RALT  = item.RALT;
    evt.params.key.CTRL  = item.CTRL;
    evt.params.key.SHIFT = item.SHIFT;
    evt.params.key.GUI   = item.GUI;
    m_uiApp->postEvent(&evt);
  }
*/
}


// converts keypad virtual key to number (VK_KP_1 = 1, VK_KP_DOWN = 2, etc...)
// -1 = no convertible
int Keyboard::convKeypadVKToNum(VirtualKey vk)
{
  switch (vk) {
    case VK_KP_0:
    case VK_KP_INSERT:
      return 0;
    case VK_KP_1:
    case VK_KP_END:
      return 1;
    case VK_KP_2:
    case VK_KP_DOWN:
      return 2;
    case VK_KP_3:
    case VK_KP_PAGEDOWN:
      return 3;
    case VK_KP_4:
    case VK_KP_LEFT:
      return 4;
    case VK_KP_5:
    case VK_KP_CENTER:
      return 5;
    case VK_KP_6:
    case VK_KP_RIGHT:
      return 6;
    case VK_KP_7:
    case VK_KP_HOME:
      return 7;
    case VK_KP_8:
    case VK_KP_UP:
      return 8;
    case VK_KP_9:
    case VK_KP_PAGEUP:
      return 9;
    default:
      return -1;
  };
}


void Keyboard::SCodeToVKConverterTask(void * pvParameters)
{
  Keyboard * keyboard = (Keyboard*) pvParameters;

  // manage ALT + Keypad num
  uint8_t ALTNUMValue = 0;  // current value (0 = no value, 0 is not allowed)

  while (true) {
    VirtualKeyItem item;
    if (keyboard->blockingGetVirtualKey(&item)) {
      // onVirtualKey may set item.vk = VK_NONE!
      keyboard->onVirtualKey(&item.vk, item.down);
      if (item.vk != VK_NONE) {
        // manage left-ALT + NUM
        if (!isALT(item.vk) && keyboard->m_LALT) {
          // ALT was down, is this a keypad number?
          int num = convKeypadVKToNum(item.vk);
          if (num >= 0) {
            // yes this is a keypad num, if down update ALTNUMValue
            if (item.down)
              ALTNUMValue = (ALTNUMValue * 10 + num) & 0xff;
          } else {
            // no, back to normal case
            ALTNUMValue = 0;
            keyboard->postVirtualKeyItem(item);
          }
        } else if (ALTNUMValue > 0 && isALT(item.vk) && !item.down) {
          // ALT is up and ALTNUMValue contains a valid value, add it
          keyboard->postVirtualKeyItem(item); // post ALT up
          item.vk          = VK_ASCII;
          item.down        = true;
          item.scancode[0] = 0;
          item.ASCII       = ALTNUMValue;
          keyboard->postVirtualKeyItem(item); // ascii key down
          item.down        = false;
          keyboard->postVirtualKeyItem(item); // ascii key up
          ALTNUMValue = 0;
        } else {
          // normal case
          keyboard->postVirtualKeyItem(item);
        }
      }
    }
  }
}


bool Keyboard::isVKDown(VirtualKey virtualKey)
{
  bool r = m_VKMap[(int)virtualKey >> 3] & (1 << ((int)virtualKey & 7));

  // VK_PAUSE is never released (no scancode sent from keyboard on key up), so when queried it is like released
  if (virtualKey == VK_PAUSE)
    m_VKMap[(int)virtualKey >> 3] &= ~(1 << ((int)virtualKey & 7));

  return r;
}
/*
inline static void repalceKey(VirtualKeyItem& it, const VirtualKey k) {
  // update m_VKMap
  if (!it.down)
    m_VKMap[(int)it.vk >> 3] |= 1 << ((int)it.vk & 7);
  else
    m_VKMap[(int)it.vk >> 3] &= ~(1 << ((int)it.vk & 7));

  it.vk = k;

  if (it.down)
    m_VKMap[(int)it.vk >> 3] |= 1 << ((int)it.vk & 7);
  else
    m_VKMap[(int)it.vk >> 3] &= ~(1 << ((int)it.vk & 7));
}
*/
inline static void joyMap(const VirtualKeyItem& it) {
  VirtualKey virtualKey = it.vk;
  if (Config::joystick == JOY_KEMPSTON || Config::joystick == JOY_FULLER || Config::joystick == JOY_CUSTOM) {
    if (virtualKey == Config::joydef[0]) {
        joyPushData(fabgl::VK_JOY_LEFT, it.down);
    }
    else if (virtualKey == Config::joydef[1]) {
        joyPushData(fabgl::VK_JOY_RIGHT, it.down);
    }
    else if (virtualKey == Config::joydef[2]) {
        joyPushData(fabgl::VK_JOY_UP, it.down);
    }
    else if (virtualKey == Config::joydef[3]) {
        joyPushData(fabgl::VK_JOY_DOWN, it.down);
    }
    else if (virtualKey == Config::joydef[4]) {
        joyPushData(fabgl::VK_JOY_START, it.down);
    }
    else if (virtualKey == Config::joydef[5]) {
        joyPushData(fabgl::VK_JOY_MODE, it.down);
    }
    else if (virtualKey == Config::joydef[6]) {
        joyPushData(fabgl::VK_JOY_A, it.down);
    }
    else if (virtualKey == Config::joydef[7]) {
        joyPushData(fabgl::VK_JOY_B, it.down);
    }
    else if (virtualKey == Config::joydef[8]) {
        joyPushData(fabgl::VK_JOY_C, it.down);
    }
    else if (virtualKey == Config::joydef[9]) {
        joyPushData(fabgl::VK_JOY_X, it.down);
    }
    else if (virtualKey == Config::joydef[10]) {
        joyPushData(fabgl::VK_JOY_Y, it.down);
    }
    else if (virtualKey == Config::joydef[11]) {
        joyPushData(fabgl::VK_JOY_Z, it.down);
    }
  }
  else if (Config::joystick == JOY_SINCLAIR2) {
    if (virtualKey == Config::joydef[0]) {
        joyPushData(fabgl::VK_1, it.down);
    }
    else if (virtualKey == Config::joydef[1]) {
        joyPushData(fabgl::VK_2, it.down);
    }
    else if (virtualKey == Config::joydef[2]) {
        joyPushData(fabgl::VK_4, it.down);
    }
    else if (virtualKey == Config::joydef[3]) {
        joyPushData(fabgl::VK_3, it.down);
    }
    else if (virtualKey == Config::joydef[6]) {
        joyPushData(fabgl::VK_5, it.down);
    }
  }
  else if (Config::joystick == JOY_SINCLAIR1) {
    if (virtualKey == Config::joydef[0]) {
        joyPushData(fabgl::VK_6, it.down);
    }
    else if (virtualKey == Config::joydef[1]) {
        joyPushData(fabgl::VK_7, it.down);
    }
    else if (virtualKey == Config::joydef[2]) {
        joyPushData(fabgl::VK_9, it.down);
    }
    else if (virtualKey == Config::joydef[3]) {
        joyPushData(fabgl::VK_8, it.down);
    }
    else if (virtualKey == Config::joydef[6]) {
        joyPushData(fabgl::VK_0, it.down);
    }
  }
  else if (Config::joystick == JOY_CURSOR) {
    if (virtualKey == Config::joydef[0]) {
        joyPushData(fabgl::VK_5, it.down);
    }
    else if (virtualKey == Config::joydef[1]) {
        joyPushData(fabgl::VK_8, it.down);
    }
    else if (virtualKey == Config::joydef[2]) {
        joyPushData(fabgl::VK_7, it.down);
    }
    else if (virtualKey == Config::joydef[3]) {
        joyPushData(fabgl::VK_6, it.down);
    }
    else if (virtualKey == Config::joydef[6]) {
        joyPushData(fabgl::VK_0, it.down);
    }
  }
}

static bool xQueueReceive(std::queue<VirtualKeyItem>& q, VirtualKeyItem* item) {
    if ( !q.empty() ) {
        *item = q.front();
        q.pop();
        joyMap(*item);
        return true;
    }
    return false;
}

bool Keyboard::getNextVirtualKey(VirtualKeyItem* item, int timeOutMS)
{
  bool r = item && xQueueReceive(m_virtualKeyQueue, item);
  if (r && m_scancodeSet == 1) /// TODO: ???
    convertScancode2to1(item);
  return r;
}

VirtualKey Keyboard::getNextVirtualKey(bool * keyDown, int timeOutMS)
{
  VirtualKeyItem item;
  if (getNextVirtualKey(&item, timeOutMS)) {
    if (keyDown)
      *keyDown = item.down;
    return item.vk;
  }
  return VK_NONE;
}

#ifdef TFT
extern "C" void refresh_lcd();
#endif

int Keyboard::virtualKeyAvailable() {
    repeat_me_for_input();
    #ifdef TFT
    static uint64_t t1 = time_us_64();
    uint64_t t2 = time_us_64();
    if (t2 - t1 > 100000ull) { // 10 fps
        t1 = t2;
        refresh_lcd();
    }
    #endif
    return m_virtualKeyQueue.size();
}


void Keyboard::emptyVirtualKeyQueue()
{
  while ( !m_virtualKeyQueue.empty() ) {
    m_virtualKeyQueue.pop();
  }
}

void Keyboard::convertScancode2to1(VirtualKeyItem * item)
{
  uint8_t * rpos = item->scancode;
  uint8_t * wpos = rpos;
  uint8_t * epos = rpos + sizeof(VirtualKeyItem::scancode);
  while (*rpos && rpos < epos) {
    if (*rpos == 0xf0) {
      ++rpos;
      *wpos++ = 0x80 | convScancodeSet2To1(*rpos++);
    } else
      *wpos++ = convScancodeSet2To1(*rpos++);
  }
  if (wpos < epos)
    *wpos = 0;
}


uint8_t Keyboard::convScancodeSet2To1(uint8_t code)
{
  // 8042 scancodes set 2 to 1 translation table
  static const uint8_t S2TOS1[256] = {
    0xff, 0x43, 0x41, 0x3f, 0x3d, 0x3b, 0x3c, 0x58, 0x64, 0x44, 0x42, 0x40, 0x3e, 0x0f, 0x29, 0x59,
    0x65, 0x38, 0x2a, 0x70, 0x1d, 0x10, 0x02, 0x5a, 0x66, 0x71, 0x2c, 0x1f, 0x1e, 0x11, 0x03, 0x5b,
    0x67, 0x2e, 0x2d, 0x20, 0x12, 0x05, 0x04, 0x5c, 0x68, 0x39, 0x2f, 0x21, 0x14, 0x13, 0x06, 0x5d,
    0x69, 0x31, 0x30, 0x23, 0x22, 0x15, 0x07, 0x5e, 0x6a, 0x72, 0x32, 0x24, 0x16, 0x08, 0x09, 0x5f,
    0x6b, 0x33, 0x25, 0x17, 0x18, 0x0b, 0x0a, 0x60, 0x6c, 0x34, 0x35, 0x26, 0x27, 0x19, 0x0c, 0x61,
    0x6d, 0x73, 0x28, 0x74, 0x1a, 0x0d, 0x62, 0x6e, 0x3a, 0x36, 0x1c, 0x1b, 0x75, 0x2b, 0x63, 0x76,
    0x55, 0x56, 0x77, 0x78, 0x79, 0x7a, 0x0e, 0x7b, 0x7c, 0x4f, 0x7d, 0x4b, 0x47, 0x7e, 0x7f, 0x6f,
    0x52, 0x53, 0x50, 0x4c, 0x4d, 0x48, 0x01, 0x45, 0x57, 0x4e, 0x51, 0x4a, 0x37, 0x49, 0x46, 0x54,
    0x80, 0x81, 0x82, 0x41, 0x54, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
    0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
    0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf,
    0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
    0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
    0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
    0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
  };
  return S2TOS1[code];
}


} // end of namespace
