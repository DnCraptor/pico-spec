#pragma GCC optimize("Ofast")

#include <cstdio>
#include <cstring>
#include <cstdarg>

#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/vreg.h>
#include <hardware/sync.h>
#include <hardware/xip_cache.h>   // xip_cache_invalidate_all (clock/flash-timing fix-up)
#include <hardware/flash.h>
#include <hardware/clocks.h>
#include <hardware/uart.h>
#include <hardware/watchdog.h>    // watchdog_caused_reboot (boot breadcrumb)
#if !PICO_RP2040
#include <hardware/structs/powman.h>  // chip_reset reason decode (boot breadcrumb)
#endif

#include <hardware/pll.h>

#ifdef PICO_RP2350
#include <hardware/regs/qmi.h>
#include <hardware/structs/qmi.h>
#include <pico/bootrom.h>              // rom_func_lookup_inline (flash QE fix window)
#include <hardware/regs/pads_qspi.h>   // SD2/SD3 pull-ups (flash QE fix window)
#include <hardware/structs/pads_qspi.h>
#endif

#include "ESPectrum.h"
#if !PICO_RP2040
#include "MidiSynth.h"
#endif
#include "Config.h"
#include "BoardPins.h"
#include "FileUtils.h"
#ifdef USE_GS
#include "GS/GS.h"
#endif
#include "MemESP.h"
#include "ChipPackage.h"
#include "pwm_audio.h"
#include "messages.h"

#include "graphics.h"

#include "audio.h"
#include "ff.h"
#include "psram_spi.h"
#include "Debug.h"
#ifdef KBDUSB
    #include "ps2kbd_mrmltr.h"
#else
    #include "ps2.h"
#endif

#if USE_NESPAD
#include "nespad.h"
#endif

#define HOME_DIR (char*)"\\SPEC"

bool cursor_blink_state = false;
uint8_t CURSOR_X, CURSOR_Y = 0;
uint8_t rx[4] = { 0 };

struct semaphore vga_start_semaphore;
#if SOFTTV
struct semaphore graphics_init_done_semaphore;
#endif
#include "Video.h"

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

input_bits_t gamepad1_bits = { false, false, false, false, false, false, false, false };
input_bits_t gamepad2_bits = { false, false, false, false, false, false, false, false };

uint8_t nes_pad2_for_alf(void) {
    input_bits_t& bits = Config::secondJoy != 1 ? gamepad2_bits : gamepad1_bits;
    uint8_t data = 0xA0;
    data |= bits.b ? 0 : 1;
    data |= bits.down ? 0 : 0b10;
    data |= bits.right ? 0 : 0b100;
    data |= bits.up ? 0 : 0b1000;
    data |= bits.left ? 0 : 0b10000;
    data |= bits.a ? 0 : 0b1000000;
    return data;
}

/* Renderer loop on Pico's second core */
#define DISP_WIDTH 320
#define DISP_HEIGHT 240

#include "fabutils.h"
void repeat_handler(void);
void back2joy2(fabgl::VirtualKey virtualKey, bool down);
extern "C" int get_framebuffer_width();
extern "C" int get_framebuffer_height();

#define JPAD (Config::secondJoy == 3 ? back2joy2: joyPushData)

///#include "OSDMain.h"

extern "C" bool handleScancode(const uint32_t ps2scancode) {
    #if 0
    if (ps2scancode != 0x45 && ps2scancode != 0x1D && ps2scancode != 0xC5) {
        char tmp1[16];
        snprintf(tmp1, 16, "%08X", ps2scancode);
        OSD::osdCenteredMsg(tmp1, LEVEL_WARN, 500);
    }
    #endif
    static bool pause_detected = false;
    if (pause_detected) {
        pause_detected = false;
        if (ps2scancode == 0x1D) return true; // ignore next byte after 0x45, TODO: split with NumLock
    }
    if ( ((ps2scancode >> 8) & 0xFF) == 0xE0) { // E0 block
        uint8_t cd = ps2scancode & 0xFF;
        bool pressed = cd < 0x80;
        cd &= 0x7F;
        switch (cd) {
            case 0x5B: kbdPushData(fabgl::VirtualKey::VK_LGUI, pressed); return true; /// L WIN = Karabas "Menu"
            case 0x1D: {
                if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed);
                kbdPushData(fabgl::VirtualKey::VK_RCTRL, pressed);
                return true;
            }
            case 0x38: kbdPushData(fabgl::VirtualKey::VK_RALT, pressed); return true;
            case 0x5C: {  /// R WIN
                if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed);
                kbdPushData(fabgl::VirtualKey::VK_RCTRL, pressed);
                return true;
            }
            case 0x5D: kbdPushData(fabgl::VirtualKey::VK_LGUI, pressed); return true; /// MENU = Karabas "Menu"
            case 0x37: kbdPushData(fabgl::VirtualKey::VK_PRINTSCREEN, pressed); return true;
            case 0x46: kbdPushData(fabgl::VirtualKey::VK_BREAK, pressed); return true;
            case 0x52: kbdPushData(fabgl::VirtualKey::VK_INSERT, pressed); return true;
            case 0x47: {
                joyPushData(fabgl::VirtualKey::VK_MENU_HOME, pressed);
                kbdPushData(fabgl::VirtualKey::VK_HOME, pressed);
                return true;
            }
            case 0x4F: kbdPushData(fabgl::VirtualKey::VK_END, pressed); return true;
            case 0x49: kbdPushData(fabgl::VirtualKey::VK_PAGEUP, pressed); return true;
            case 0x51: kbdPushData(fabgl::VirtualKey::VK_PAGEDOWN, pressed); return true;
            case 0x53: kbdPushData(fabgl::VirtualKey::VK_DELETE, pressed); return true;
            case 0x48: {
                if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_UP, pressed);
                kbdPushData(fabgl::VirtualKey::VK_UP, pressed);
                return true;
            }
            case 0x50: {
                if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, pressed);
                kbdPushData(fabgl::VirtualKey::VK_DOWN, pressed);
                return true;
            }
            case 0x4B: {
                if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, pressed);
                kbdPushData(fabgl::VirtualKey::VK_LEFT, pressed);
                return true;
            }
            case 0x4D: {
                if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, pressed);
                kbdPushData(fabgl::VirtualKey::VK_RIGHT, pressed);
                return true;
            }
            case 0x35: kbdPushData(fabgl::VirtualKey::VK_SLASH, pressed); return true;
            case 0x1C: { // VK_KP_ENTER
                kbdPushData(Config::rightSpace ? fabgl::VirtualKey::VK_SPACE : fabgl::VirtualKey::VK_RETURN, pressed);
                return true;
            }
        }
        return true;
    }
    uint8_t cd = ps2scancode & 0xFF;
    bool pressed = cd < 0x80;
    cd &= 0x7F;
    switch (cd) {
        case 0x1E: kbdPushData(fabgl::VirtualKey::VK_A, pressed); return true;
        case 0x30: kbdPushData(fabgl::VirtualKey::VK_B, pressed); return true;
        case 0x2E: kbdPushData(fabgl::VirtualKey::VK_C, pressed); return true;
        case 0x20: kbdPushData(fabgl::VirtualKey::VK_D, pressed); return true;
        case 0x12: kbdPushData(fabgl::VirtualKey::VK_E, pressed); return true;
        case 0x21: kbdPushData(fabgl::VirtualKey::VK_F, pressed); return true;
        case 0x22: kbdPushData(fabgl::VirtualKey::VK_G, pressed); return true;
        case 0x23: kbdPushData(fabgl::VirtualKey::VK_H, pressed); return true;
        case 0x17: kbdPushData(fabgl::VirtualKey::VK_I, pressed); return true;
        case 0x24: kbdPushData(fabgl::VirtualKey::VK_J, pressed); return true;
        case 0x25: kbdPushData(fabgl::VirtualKey::VK_K, pressed); return true;
        case 0x26: kbdPushData(fabgl::VirtualKey::VK_L, pressed); return true;
        case 0x32: kbdPushData(fabgl::VirtualKey::VK_M, pressed); return true;
        case 0x31: kbdPushData(fabgl::VirtualKey::VK_N, pressed); return true;
        case 0x18: kbdPushData(fabgl::VirtualKey::VK_O, pressed); return true;
        case 0x19: kbdPushData(fabgl::VirtualKey::VK_P, pressed); return true;
        case 0x10: kbdPushData(fabgl::VirtualKey::VK_Q, pressed); return true;
        case 0x13: kbdPushData(fabgl::VirtualKey::VK_R, pressed); return true;
        case 0x1F: kbdPushData(fabgl::VirtualKey::VK_S, pressed); return true;
        case 0x14: kbdPushData(fabgl::VirtualKey::VK_T, pressed); return true;
        case 0x16: kbdPushData(fabgl::VirtualKey::VK_U, pressed); return true;
        case 0x2F: kbdPushData(fabgl::VirtualKey::VK_V, pressed); return true;
        case 0x11: kbdPushData(fabgl::VirtualKey::VK_W, pressed); return true;
        case 0x2D: kbdPushData(fabgl::VirtualKey::VK_X, pressed); return true;
        case 0x15: kbdPushData(fabgl::VirtualKey::VK_Y, pressed); return true;
        case 0x2C: kbdPushData(fabgl::VirtualKey::VK_Z, pressed); return true;

        case 0x0B: kbdPushData(fabgl::VirtualKey::VK_0, pressed); return true;
        case 0x02: kbdPushData(fabgl::VirtualKey::VK_1, pressed); return true;
        case 0x03: kbdPushData(fabgl::VirtualKey::VK_2, pressed); return true;
        case 0x04: kbdPushData(fabgl::VirtualKey::VK_3, pressed); return true;
        case 0x05: kbdPushData(fabgl::VirtualKey::VK_4, pressed); return true;
        case 0x06: kbdPushData(fabgl::VirtualKey::VK_5, pressed); return true;
        case 0x07: kbdPushData(fabgl::VirtualKey::VK_6, pressed); return true;
        case 0x08: kbdPushData(fabgl::VirtualKey::VK_7, pressed); return true;
        case 0x09: kbdPushData(fabgl::VirtualKey::VK_8, pressed); return true;
        case 0x0A: kbdPushData(fabgl::VirtualKey::VK_9, pressed); return true;

        case 0x29: kbdPushData(fabgl::VirtualKey::VK_TILDE, pressed); return true;
        case 0x0C: kbdPushData(fabgl::VirtualKey::VK_MINUS, pressed); return true;
        case 0x0D: kbdPushData(fabgl::VirtualKey::VK_EQUALS, pressed); return true;
        case 0x2B: kbdPushData(fabgl::VirtualKey::VK_BACKSLASH, pressed); return true;
        case 0x1A: kbdPushData(fabgl::VirtualKey::VK_LEFTBRACKET, pressed); return true;
        case 0x1B: kbdPushData(fabgl::VirtualKey::VK_RIGHTBRACKET, pressed); return true;
        case 0x27: kbdPushData(fabgl::VirtualKey::VK_SEMICOLON, pressed); return true;
        case 0x28: kbdPushData(fabgl::VirtualKey::VK_QUOTE, pressed); return true;
        case 0x33: kbdPushData(fabgl::VirtualKey::VK_COMMA, pressed); return true;
        case 0x34: kbdPushData(fabgl::VirtualKey::VK_PERIOD, pressed); return true;
        case 0x35: kbdPushData(fabgl::VirtualKey::VK_SLASH, pressed); return true;

        case 0x0E: {
            joyPushData(fabgl::VirtualKey::VK_MENU_BS, pressed);
            kbdPushData(fabgl::VirtualKey::VK_BACKSPACE, pressed);
            return true;
        }
        case 0x39: {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, pressed);
            kbdPushData(fabgl::VirtualKey::VK_SPACE, pressed);
            return true;
        }
        case 0x0F: {
            if (Config::TABasfire1) JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed);
            kbdPushData(fabgl::VirtualKey::VK_TAB, pressed);
            return true;
        }
        case 0x3A: kbdPushData(fabgl::VirtualKey::VK_CAPSLOCK, pressed); return true; /// TODO: CapsLock
        case 0x2A: kbdPushData(fabgl::VirtualKey::VK_LSHIFT, pressed); return true;
        case 0x1D: kbdPushData(fabgl::VirtualKey::VK_LCTRL, pressed); return true;
        case 0x38: {
            if (Config::CursorAsJoy) JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed);
            kbdPushData(fabgl::VirtualKey::VK_LALT, pressed);
            return true;
        }
        case 0x36: kbdPushData(fabgl::VirtualKey::VK_RSHIFT, pressed); return true;
        case 0x1C: {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, pressed);
            kbdPushData(fabgl::VirtualKey::VK_RETURN, pressed);
            return true;
        }
        case 0x01: kbdPushData(fabgl::VirtualKey::VK_ESCAPE, pressed); return true;
        case 0x3B: kbdPushData(fabgl::VirtualKey::VK_F1, pressed); return true;
        case 0x3C: kbdPushData(fabgl::VirtualKey::VK_F2, pressed); return true;
        case 0x3D: kbdPushData(fabgl::VirtualKey::VK_F3, pressed); return true;
        case 0x3E: kbdPushData(fabgl::VirtualKey::VK_F4, pressed); return true;
        case 0x3F: kbdPushData(fabgl::VirtualKey::VK_F5, pressed); return true;
        case 0x40: kbdPushData(fabgl::VirtualKey::VK_F6, pressed); return true;
        case 0x41: kbdPushData(fabgl::VirtualKey::VK_F7, pressed); return true;
        case 0x42: kbdPushData(fabgl::VirtualKey::VK_F8, pressed); return true;
        case 0x43: kbdPushData(fabgl::VirtualKey::VK_F9, pressed); return true;
        case 0x44: kbdPushData(fabgl::VirtualKey::VK_F10, pressed); return true;
        case 0x57: kbdPushData(fabgl::VirtualKey::VK_F11, pressed); return true;
        case 0x58: kbdPushData(fabgl::VirtualKey::VK_F12, pressed); return true;

        case 0x46: kbdPushData(fabgl::VirtualKey::VK_SCROLLLOCK, pressed); return true; /// TODO:
        case 0x45: {
            kbdPushData(fabgl::VirtualKey::VK_PAUSE, pressed);
            pause_detected = pressed;
            return true;
        }
        case 0x37: {
            JPAD(fabgl::VirtualKey::VK_DPAD_START, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_MULTIPLY, pressed);
            return true;
        }
        case 0x4A: {
            JPAD(fabgl::VirtualKey::VK_DPAD_SELECT, pressed);
            kbdPushData(fabgl::VirtualKey::VK_MINUS, pressed);
            return true;
        }
        case 0x4E: {
            JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed);
            kbdPushData(fabgl::VirtualKey::VK_PLUS, pressed);
            return true;
        }
        case 0x53: {
            JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_PERIOD, pressed);
            return true;
        }
        case 0x52: {
            JPAD(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_0, pressed);
            return true;
        }
        case 0x4F: {
            JPAD(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_1, pressed);
            return true;
        }
        case 0x50: {
            JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_2, pressed);
            return true;
        }
        case 0x51: {
            JPAD(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_3, pressed);
            return true;
        }
        case 0x4B: {
            JPAD(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_4, pressed);
            return true;
        }
        case 0x4C: {
            JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_5, pressed);
            return true;
        }
        case 0x4D: {
            JPAD(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_6, pressed);
            return true;
        }
        case 0x47: {
            JPAD(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_UP, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_7, pressed);
            return true;
        }
        case 0x48: {
            JPAD(fabgl::VirtualKey::VK_DPAD_UP, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_8, pressed);
            return true;
        }
        case 0x49: {
            JPAD(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_UP, pressed);
            kbdPushData(fabgl::VirtualKey::VK_KP_9, pressed);
            return true;
        }
    }
    return true;
}


#if USE_NESPAD

// NESPAD edge-detection uses its own prev-state instead of gamepad1_bits.
// Otherwise USB HID/XInput gamepad handlers (DS4, XInput) which also write
// gamepad1_bits would race the NESPAD tick — every NESPAD tick would see
// gamepad1_bits.X set by the USB pad, decide it's a release, and push a
// ghost release. The USB pad next callback would then see gamepad1_bits.X
// reset and push a duplicate press → button stuck.
static uint32_t nespad_prev_state = 0;
static bool nespad_active = false; // false until nespad_begin(); stays false if yielded to ZiFi

static void nespad_tick1(void) {
    if (!nespad_active) return;
    nespad_read();
    uint32_t cur = nespad_state;
    uint32_t prev = nespad_prev_state;
    uint32_t pressed  = cur & ~prev;   // 0->1 transitions
    uint32_t released = ~cur & prev;   // 1->0 transitions
    nespad_prev_state = cur;
    if (!pressed && !released) return;

    bool sel = (cur & DPAD_SELECT) != 0;

    if (pressed & DPAD_SELECT) {
        joyPushData(fabgl::VirtualKey::VK_MENU_BS, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, true);
    } else if (released & DPAD_SELECT) {
        joyPushData(fabgl::VirtualKey::VK_MENU_BS, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, false);
    }

    if (pressed & DPAD_A) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, true);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_0 : fabgl::VirtualKey::VK_RETURN, true);
    } else if (released & DPAD_A) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, false);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_0 : fabgl::VirtualKey::VK_RETURN, false);
    }

    if (pressed & DPAD_B) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, true);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_1 : fabgl::VirtualKey::VK_SPACE, true);
    } else if (released & DPAD_B) {
        joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, false);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_1 : fabgl::VirtualKey::VK_SPACE, false);
    }

    if (pressed & DPAD_START) {
        joyPushData(fabgl::VirtualKey::VK_MENU_HOME, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, true);
        if (sel || Config::joy2cursor)
            joyPushData(sel ? fabgl::VirtualKey::VK_F1 : fabgl::VirtualKey::VK_R, true);
    } else if (released & DPAD_START) {
        joyPushData(fabgl::VirtualKey::VK_MENU_HOME, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, false);
        if (sel || Config::joy2cursor)
            joyPushData(sel ? fabgl::VirtualKey::VK_F1 : fabgl::VirtualKey::VK_R, false);
    }

    if (pressed & DPAD_UP) {
        joyPushData(fabgl::VirtualKey::VK_MENU_UP, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, true);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_PAGEUP : fabgl::VirtualKey::VK_UP, true);
    } else if (released & DPAD_UP) {
        joyPushData(fabgl::VirtualKey::VK_MENU_UP, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, false);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_PAGEUP : fabgl::VirtualKey::VK_UP, false);
    }

    if (pressed & DPAD_DOWN) {
        joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, true);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_PAGEDOWN : fabgl::VirtualKey::VK_DOWN, true);
    } else if (released & DPAD_DOWN) {
        joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, false);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_PAGEDOWN : fabgl::VirtualKey::VK_DOWN, false);
    }

    if (pressed & DPAD_LEFT) {
        joyPushData(VIDEO::OSD & 0x04 ? fabgl::VirtualKey::VK_F9 : fabgl::VirtualKey::VK_MENU_LEFT, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, true);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_BACKSPACE : fabgl::VirtualKey::VK_LEFT, true);
    } else if (released & DPAD_LEFT) {
        joyPushData(VIDEO::OSD & 0x04 ? fabgl::VirtualKey::VK_F9 : fabgl::VirtualKey::VK_MENU_LEFT, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, false);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_BACKSPACE : fabgl::VirtualKey::VK_LEFT, false);
    }

    if (pressed & DPAD_RIGHT) {
        joyPushData(VIDEO::OSD & 0x04 ? fabgl::VirtualKey::VK_F10 : fabgl::VirtualKey::VK_MENU_RIGHT, true);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, true);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_K : fabgl::VirtualKey::VK_RIGHT, true);
    } else if (released & DPAD_RIGHT) {
        joyPushData(VIDEO::OSD & 0x04 ? fabgl::VirtualKey::VK_F10 : fabgl::VirtualKey::VK_MENU_RIGHT, false);
        if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, false);
        if (Config::joy2cursor) joyPushData(sel ? fabgl::VirtualKey::VK_K : fabgl::VirtualKey::VK_RIGHT, false);
    }
}

static void nespad_tick2(void) {
    if (!nespad_active) return;
    if (Config::secondJoy == 3) return;
    gamepad2_bits.a = (nespad_state2 & DPAD_A) != 0;
    gamepad2_bits.b = (nespad_state2 & DPAD_B) != 0;
    gamepad2_bits.select = (nespad_state2 & DPAD_SELECT) != 0;
    gamepad2_bits.start = (nespad_state2 & DPAD_START) != 0;
    gamepad2_bits.up = (nespad_state2 & DPAD_UP) != 0;
    gamepad2_bits.down = (nespad_state2 & DPAD_DOWN) != 0;
    gamepad2_bits.left = (nespad_state2 & DPAD_LEFT) != 0;
    gamepad2_bits.right = (nespad_state2 & DPAD_RIGHT) != 0;
}

static void nespad_tick1(void);
static void nespad_tick2(void);
#endif

void back2joy2(fabgl::VirtualKey virtualKey, bool down) {
    switch(virtualKey) {
            case fabgl::VirtualKey::VK_DPAD_FIRE    : gamepad2_bits.a = down; break;
            case fabgl::VirtualKey::VK_DPAD_ALTFIRE : gamepad2_bits.b = down; break;
            case fabgl::VirtualKey::VK_DPAD_UP      : gamepad2_bits.up = down; break;
            case fabgl::VirtualKey::VK_DPAD_DOWN    : gamepad2_bits.down = down; break;
            case fabgl::VirtualKey::VK_DPAD_LEFT    : gamepad2_bits.left = down; break;
            case fabgl::VirtualKey::VK_DPAD_RIGHT   : gamepad2_bits.right = down; break;
            case fabgl::VirtualKey::VK_DPAD_START   : gamepad2_bits.start = down; break;
            case fabgl::VirtualKey::VK_DPAD_SELECT  : gamepad2_bits.select = down; break;
    }
}

#ifdef KBDUSB
inline static bool isInReport(hid_keyboard_report_t const *report, const unsigned char keycode) {
    for (unsigned char i: report->keycode) {
        if (i == keycode) {
            return true;
        }
    }
    return false;
}

uint8_t pressed_key[256] = { 0 };
extern uint8_t debug_number;
fabgl::VirtualKey map_key(uint8_t kc) {
    switch(kc) {
        case HID_KEY_SPACE: return fabgl::VirtualKey::VK_SPACE;

        case HID_KEY_A: return fabgl::VirtualKey::VK_A;
        case HID_KEY_B: return fabgl::VirtualKey::VK_B;
        case HID_KEY_C: return fabgl::VirtualKey::VK_C;
        case HID_KEY_D: return fabgl::VirtualKey::VK_D;
        case HID_KEY_E: return fabgl::VirtualKey::VK_E;
        case HID_KEY_F: return fabgl::VirtualKey::VK_F;
        case HID_KEY_G: return fabgl::VirtualKey::VK_G;
        case HID_KEY_H: return fabgl::VirtualKey::VK_H;
        case HID_KEY_I: return fabgl::VirtualKey::VK_I;
        case HID_KEY_J: return fabgl::VirtualKey::VK_J;
        case HID_KEY_K: return fabgl::VirtualKey::VK_K;
        case HID_KEY_L: return fabgl::VirtualKey::VK_L;
        case HID_KEY_M: return fabgl::VirtualKey::VK_M;
        case HID_KEY_N: return fabgl::VirtualKey::VK_N;
        case HID_KEY_O: return fabgl::VirtualKey::VK_O;
        case HID_KEY_P: return fabgl::VirtualKey::VK_P;
        case HID_KEY_Q: return fabgl::VirtualKey::VK_Q;
        case HID_KEY_R: return fabgl::VirtualKey::VK_R;
        case HID_KEY_S: return fabgl::VirtualKey::VK_S;
        case HID_KEY_T: return fabgl::VirtualKey::VK_T;
        case HID_KEY_U: return fabgl::VirtualKey::VK_U;
        case HID_KEY_V: return fabgl::VirtualKey::VK_V;
        case HID_KEY_W: return fabgl::VirtualKey::VK_W;
        case HID_KEY_X: return fabgl::VirtualKey::VK_X;
        case HID_KEY_Y: return fabgl::VirtualKey::VK_Y;
        case HID_KEY_Z: return fabgl::VirtualKey::VK_Z;

        case HID_KEY_0: return fabgl::VirtualKey::VK_0;
        case HID_KEY_1: return fabgl::VirtualKey::VK_1;
        case HID_KEY_2: return fabgl::VirtualKey::VK_2;
        case HID_KEY_3: return fabgl::VirtualKey::VK_3;
        case HID_KEY_4: return fabgl::VirtualKey::VK_4;
        case HID_KEY_5: return fabgl::VirtualKey::VK_5;
        case HID_KEY_6: return fabgl::VirtualKey::VK_6;
        case HID_KEY_7: return fabgl::VirtualKey::VK_7;
        case HID_KEY_8: return fabgl::VirtualKey::VK_8;
        case HID_KEY_9: return fabgl::VirtualKey::VK_9;

        case HID_KEY_KEYPAD_0: return fabgl::VirtualKey::VK_KP_0;
        case HID_KEY_KEYPAD_1: return fabgl::VirtualKey::VK_KP_1;
        case HID_KEY_KEYPAD_2: return fabgl::VirtualKey::VK_KP_2;
        case HID_KEY_KEYPAD_3: return fabgl::VirtualKey::VK_KP_3;
        case HID_KEY_KEYPAD_4: return fabgl::VirtualKey::VK_KP_4;
        case HID_KEY_KEYPAD_5: return fabgl::VirtualKey::VK_KP_5;
        case HID_KEY_KEYPAD_6: return fabgl::VirtualKey::VK_KP_6;
        case HID_KEY_KEYPAD_7: return fabgl::VirtualKey::VK_KP_7;
        case HID_KEY_KEYPAD_8: return fabgl::VirtualKey::VK_KP_8;
        case HID_KEY_KEYPAD_9: return fabgl::VirtualKey::VK_KP_9;
        case HID_KEY_NUM_LOCK: return fabgl::VirtualKey::VK_NUMLOCK;
        case HID_KEY_KEYPAD_DIVIDE: return fabgl::VirtualKey::VK_KP_DIVIDE;
        case HID_KEY_KEYPAD_MULTIPLY: return fabgl::VirtualKey::VK_KP_MULTIPLY;
        case HID_KEY_KEYPAD_SUBTRACT: return fabgl::VirtualKey::VK_KP_MINUS;
        case HID_KEY_KEYPAD_ADD: return fabgl::VirtualKey::VK_KP_PLUS;
        case HID_KEY_KEYPAD_ENTER: return fabgl::VirtualKey::VK_KP_ENTER;
        case HID_KEY_KEYPAD_DECIMAL: return fabgl::VirtualKey::VK_KP_PERIOD;

        case HID_KEY_PRINT_SCREEN: return fabgl::VirtualKey::VK_PRINTSCREEN;
        case HID_KEY_SCROLL_LOCK: return fabgl::VirtualKey::VK_SCROLLLOCK;
        case HID_KEY_PAUSE: return fabgl::VirtualKey::VK_PAUSE;

        case HID_KEY_INSERT: return fabgl::VirtualKey::VK_INSERT;
        case HID_KEY_HOME: return fabgl::VirtualKey::VK_HOME;
        case HID_KEY_PAGE_UP: return fabgl::VirtualKey::VK_PAGEUP;
        case HID_KEY_PAGE_DOWN: return fabgl::VirtualKey::VK_PAGEDOWN;
        case HID_KEY_DELETE: return fabgl::VirtualKey::VK_DELETE;
        case HID_KEY_END: return fabgl::VirtualKey::VK_END;

        case HID_KEY_F1: return fabgl::VirtualKey::VK_F1;
        case HID_KEY_F2: return fabgl::VirtualKey::VK_F2;
        case HID_KEY_F3: return fabgl::VirtualKey::VK_F3;
        case HID_KEY_F4: return fabgl::VirtualKey::VK_F4;
        case HID_KEY_F5: return fabgl::VirtualKey::VK_F5;
        case HID_KEY_F6: return fabgl::VirtualKey::VK_F6;
        case HID_KEY_F7: return fabgl::VirtualKey::VK_F7;
        case HID_KEY_F8: return fabgl::VirtualKey::VK_F8;
        case HID_KEY_F9: return fabgl::VirtualKey::VK_F9;
        case HID_KEY_F10: return fabgl::VirtualKey::VK_F10;
        case HID_KEY_F11: return fabgl::VirtualKey::VK_F11;
        case HID_KEY_F12: return fabgl::VirtualKey::VK_F12;

        case HID_KEY_ALT_LEFT: return fabgl::VirtualKey::VK_LALT;
        case HID_KEY_ALT_RIGHT: return fabgl::VirtualKey::VK_RALT;
        case HID_KEY_CONTROL_LEFT: return fabgl::VirtualKey::VK_LCTRL;
        case HID_KEY_CONTROL_RIGHT: return fabgl::VirtualKey::VK_RCTRL;
        case HID_KEY_SHIFT_LEFT: return fabgl::VirtualKey::VK_LSHIFT;
        case HID_KEY_SHIFT_RIGHT: return fabgl::VirtualKey::VK_RSHIFT;
        case HID_KEY_CAPS_LOCK: return fabgl::VirtualKey::VK_CAPSLOCK;

        case HID_KEY_TAB: return fabgl::VirtualKey::VK_TAB;
        case HID_KEY_ENTER: return fabgl::VirtualKey::VK_RETURN;
        case HID_KEY_ESCAPE: return fabgl::VirtualKey::VK_ESCAPE;

        case HID_KEY_GRAVE: return fabgl::VirtualKey::VK_TILDE;
        case HID_KEY_MINUS: return fabgl::VirtualKey::VK_MINUS;
        case HID_KEY_EQUAL: return fabgl::VirtualKey::VK_EQUALS;
        case HID_KEY_BACKSLASH: return fabgl::VirtualKey::VK_BACKSLASH;
        case HID_KEY_EUROPE_1: return fabgl::VirtualKey::VK_BACKSLASH; // ???
        case HID_KEY_BRACKET_LEFT: return fabgl::VirtualKey::VK_LEFTBRACKET;
        case HID_KEY_BRACKET_RIGHT: return fabgl::VirtualKey::VK_RIGHTBRACKET;
        case HID_KEY_SEMICOLON: return fabgl::VirtualKey::VK_SEMICOLON;
        case HID_KEY_APOSTROPHE: return fabgl::VirtualKey::VK_QUOTE;
        case HID_KEY_COMMA: return fabgl::VirtualKey::VK_COMMA;
        case HID_KEY_PERIOD: return fabgl::VirtualKey::VK_PERIOD;
        case HID_KEY_SLASH: return fabgl::VirtualKey::VK_SLASH;
        case HID_KEY_BACKSPACE: return fabgl::VirtualKey::VK_BACKSPACE;

        case HID_KEY_ARROW_UP: return fabgl::VirtualKey::VK_UP;
        case HID_KEY_ARROW_DOWN: return fabgl::VirtualKey::VK_DOWN;
        case HID_KEY_ARROW_LEFT: return fabgl::VirtualKey::VK_LEFT;
        case HID_KEY_ARROW_RIGHT: return fabgl::VirtualKey::VK_RIGHT;

        case HID_KEY_VOLUME_UP: return fabgl::VirtualKey::VK_VOLUMEUP;
        case HID_KEY_VOLUME_DOWN: return fabgl::VirtualKey::VK_VOLUMEDOWN;
        case HID_KEY_MUTE: return fabgl::VirtualKey::VK_VOLUMEMUTE;
 // TODO:
//        case HID_KEY_GUI_LEFT: return fabgl::VirtualKey::VK_F1;
//        case HID_KEY_GUI_RIGHT: return fabgl::VirtualKey::VK_F1;
        default: break;
        //debug_number = kc;
    }
    return fabgl::VirtualKey::VK_NONE;
}

void kbdExtraMapping(fabgl::VirtualKey virtualKey, bool pressed) {
    switch(virtualKey) {
        case fabgl::VirtualKey::VK_RCTRL: if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed); break;
        case fabgl::VirtualKey::VK_MENU_RIGHT: if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed); break;
        case fabgl::VirtualKey::VK_HOME: joyPushData(fabgl::VirtualKey::VK_MENU_HOME, pressed); break;
        case fabgl::VirtualKey::VK_UP: {
            if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, pressed);
            joyPushData(fabgl::VirtualKey::VK_MENU_UP, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_DOWN: {
            if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_LEFT: {
            if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
            joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_RIGHT: {
            if (Config::CursorAsJoy) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
            joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_D: {
            if (Config::wasd) {
                joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, pressed);
            }
            break;
        }
        case fabgl::VirtualKey::VK_W: {
            if (Config::wasd) {
                joyPushData(fabgl::VirtualKey::VK_DPAD_UP, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_UP, pressed);
            }
            break;
        }
        case fabgl::VirtualKey::VK_A: {
            if (Config::wasd) {
                joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, pressed);
            }
            break;
        }
        case fabgl::VirtualKey::VK_S: {
            if (Config::wasd) {
                joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
                joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, pressed);
            }
            break;
        }
        case fabgl::VirtualKey::VK_KP_ENTER: { // VK_KP_ENTER
            kbdPushData(Config::rightSpace ? fabgl::VirtualKey::VK_SPACE : fabgl::VirtualKey::VK_RETURN, pressed);
            return;
        }
        case fabgl::VirtualKey::VK_L: {
            if (Config::wasd) kbdPushData(fabgl::VirtualKey::VK_DPAD_FIRE, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_K: {
            if (Config::wasd) kbdPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed);
            break;
        }

        case fabgl::VirtualKey::VK_BACKSPACE: joyPushData(fabgl::VirtualKey::VK_MENU_BS, pressed); break;
        case fabgl::VirtualKey::VK_SPACE:     joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, pressed); break;
        case fabgl::VirtualKey::VK_TAB:       if (Config::TABasfire1) JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed); break;
        case fabgl::VirtualKey::VK_LALT:
            if (Config::CursorAsJoy) JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed);
            kbdPushData(fabgl::VirtualKey::VK_LALT, pressed);
            break;
        case fabgl::VirtualKey::VK_RETURN:    joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, pressed); break;

        case fabgl::VirtualKey::VK_KP_MULTIPLY: JPAD(fabgl::VirtualKey::VK_DPAD_START, pressed); break;
        case fabgl::VirtualKey::VK_KP_MINUS:    JPAD(fabgl::VirtualKey::VK_DPAD_SELECT, pressed); break;
        case fabgl::VirtualKey::VK_KP_PLUS:     JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed); break;
        case fabgl::VirtualKey::VK_KP_PERIOD:   JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed); break;
        case fabgl::VirtualKey::VK_KP_0:        JPAD(fabgl::VirtualKey::VK_DPAD_ALTFIRE, pressed); break;

        case fabgl::VirtualKey::VK_KP_1: {
            JPAD(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_KP_2:        JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed); break;
        case fabgl::VirtualKey::VK_KP_3: {
            JPAD(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_KP_4:        JPAD(fabgl::VirtualKey::VK_DPAD_LEFT, pressed); break;
        case fabgl::VirtualKey::VK_KP_5:        JPAD(fabgl::VirtualKey::VK_DPAD_DOWN, pressed); break;
        case fabgl::VirtualKey::VK_KP_6:        JPAD(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed); break;
        case fabgl::VirtualKey::VK_KP_7: {
            JPAD(fabgl::VirtualKey::VK_DPAD_LEFT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_UP, pressed);
            break;
        }
        case fabgl::VirtualKey::VK_KP_8:        JPAD(fabgl::VirtualKey::VK_DPAD_UP, pressed); break;
        case fabgl::VirtualKey::VK_KP_9: {
            JPAD(fabgl::VirtualKey::VK_DPAD_RIGHT, pressed);
            JPAD(fabgl::VirtualKey::VK_DPAD_UP, pressed);
            break;
        }
    }
    kbdPushData(virtualKey, pressed);
}

typedef struct mod2key_s {
    hid_keyboard_modifier_bm_t mod;
    enum fabgl::VirtualKey     key;
} mod2key_t;

static mod2key_t mod2key[] = {
    { KEYBOARD_MODIFIER_LEFTALT,    fabgl::VirtualKey::VK_LALT },
    { KEYBOARD_MODIFIER_RIGHTALT,   fabgl::VirtualKey::VK_RALT },
    { KEYBOARD_MODIFIER_LEFTCTRL,   fabgl::VirtualKey::VK_LCTRL},
    { KEYBOARD_MODIFIER_RIGHTCTRL,  fabgl::VirtualKey::VK_RCTRL},
    { KEYBOARD_MODIFIER_RIGHTSHIFT, fabgl::VirtualKey::VK_RSHIFT},
    { KEYBOARD_MODIFIER_LEFTSHIFT,  fabgl::VirtualKey::VK_LSHIFT},
    // GUI/Win = the Karabas-Pro "Menu" key (ROMSET hotkeys Menu+F1..F4, see
    // ESPectrum::processKeyboard). Used to be bound to VK_F1/VK_F2 (OSD menu).
    { KEYBOARD_MODIFIER_RIGHTGUI,   fabgl::VirtualKey::VK_RGUI},
    { KEYBOARD_MODIFIER_LEFTGUI,    fabgl::VirtualKey::VK_LGUI},
};

void __not_in_flash_func(process_kbd_report)(
    hid_keyboard_report_t const *report,
    hid_keyboard_report_t const *prev_report
) {
    for (int i = 0; i < sizeof(mod2key) / sizeof(mod2key[0]); ++i) {
        if (report->modifier & mod2key[i].mod) { // LALT
            if (!pressed_key[mod2key[i].key]) {
                pressed_key[mod2key[i].key] = mod2key[i].key;
                kbdExtraMapping(mod2key[i].key, true);
            }
        } else {
            if (pressed_key[mod2key[i].key]) {
                kbdExtraMapping(mod2key[i].key, false);
                pressed_key[mod2key[i].key] = 0;
            }
        }
    }
    for (uint8_t pkc: prev_report->keycode) {
        if (!pkc) continue;
        bool key_still_pressed = false;
        for (uint8_t kc: report->keycode) {
            if (kc == pkc) {
                key_still_pressed = true;
                break;
            }
        }
        if (!key_still_pressed) {
            kbdExtraMapping((fabgl::VirtualKey)pressed_key[pkc], false);
            pressed_key[pkc] = 0;
        }
    }
    for (uint8_t kc: report->keycode) {
        if (!kc) continue;
        uint8_t* pk = pressed_key + kc;
        fabgl::VirtualKey vk = (fabgl::VirtualKey)*pk;
        if (vk == fabgl::VirtualKey::VK_NONE) { // it was not yet pressed
            vk = map_key(kc);
            if (vk != fabgl::VirtualKey::VK_NONE) {
                *pk = (uint8_t)vk;
                kbdExtraMapping(vk, true);
            }
        }
    }
}

Ps2Kbd_Mrmltr ps2kbd(
        pio1,
        KBD_CLOCK_PIN,
        process_kbd_report
);
#endif

// ── Uptime across soft reboots ───────────────────────────────────────────────
// HWInfo's Uptime used raw time_us_64() (µs since the last chip reset), so an
// F12 reboot / video-mode switch / crash watchdog restarted it from zero.
// Accumulate the pre-reboot seconds in a watchdog scratch register instead:
// scratch survives a watchdog reboot but is cleared by a power-on / RUN-pin
// reset — exactly the boundary a "since power-on" uptime wants.
// scratch[3] = 0x55 tag (top byte) | uptime seconds (24 bits ≈ 194 days).
// scratch[2] is MIDI_REFLASH_SCRATCH (MidiSynth.cpp); 4..7 belong to the
// SDK/bootrom watchdog_reboot vector.
#define UPTIME_SCRATCH  3
#define UPTIME_TAG      0x55000000u
#define UPTIME_TAG_MASK 0xFF000000u
static uint32_t s_uptime_offset_s = 0; // seconds accumulated before the last watchdog reboot

extern "C" uint32_t uptime_seconds(void) {
    return s_uptime_offset_s + (uint32_t)(time_us_64() / 1000000ULL);
}

// Called once at main() entry, before anything can arm the watchdog.
static void uptime_init(void) {
    uint32_t v = watchdog_hw->scratch[UPTIME_SCRATCH];
    if (watchdog_caused_reboot() && (v & UPTIME_TAG_MASK) == UPTIME_TAG)
        s_uptime_offset_s = v & ~UPTIME_TAG_MASK;
    watchdog_hw->scratch[UPTIME_SCRATCH] = UPTIME_TAG | (s_uptime_offset_s & ~UPTIME_TAG_MASK);
}

// Refreshed from the 150 ms input tick below so even a crash-path watchdog
// reboot (which never goes through esp_hard_reset) keeps the running total.
static inline void uptime_refresh(void) {
    watchdog_hw->scratch[UPTIME_SCRATCH] = UPTIME_TAG | (uptime_seconds() & ~UPTIME_TAG_MASK);
}

void repeat_me_for_input() {
    static uint32_t tickKbdRep1 = time_us_32();
    // 60 FPS loop
#define frame_tick (16666)
    static uint64_t tick = time_us_64();
    static bool tick1 = true;
    static uint64_t last_input_tick = tick;
        if (tick >= last_input_tick + frame_tick) {
#ifdef KBDUSB
            ps2kbd.tick();
#endif
#ifdef USE_NESPAD
            (tick1 ? nespad_tick1 : nespad_tick2)(); // split call for joy1 and 2
            tick1 = !tick1;
#endif
            last_input_tick = tick;
        }
        tick = time_us_64();
        uint32_t tickKbdRep2 = time_us_32();
        if (tickKbdRep2 - tickKbdRep1 > 150000) { // repeat each 150 ms
            repeat_handler();
            uptime_refresh();
            tickKbdRep1 = tickKbdRep2;
        }

#ifdef KBDUSB
        tuh_task();
#endif
}

#ifdef VGA_HDMI
extern "C" void hdmi_poll_reinit(void);
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
extern "C" void vga_reinit(void);
#endif
#endif
#ifdef TFT
extern "C" void refresh_lcd(void);
#endif

void __scratch_x("render") render_core() {
    multicore_lockout_victim_init();
    { extern size_t getFreeHeap(void); Debug::log("render: graphics_init begin, freeHeap=%u", (unsigned)getFreeHeap()); }
    graphics_init();
    { extern size_t getFreeHeap(void); Debug::log("render: graphics_init done, freeHeap=%u", (unsigned)getFreeHeap()); }
#if SOFTTV
    sem_release(&graphics_init_done_semaphore);
#endif
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    // graphics_init() hardcodes line_VS_begin/end for 640x480 (490/491).
    // For 720x modes video_mode is already set by VIDEO::Reset() on core0,
    // so update vsync line numbers from the mode table now.
    extern bool SELECT_VGA;
    if (SELECT_VGA) vga_reinit();
#endif
    graphics_set_buffer(NULL, get_framebuffer_width(), get_framebuffer_height());
    graphics_set_bgcolor(0x000000);
    graphics_set_flashmode(true, false);
    sem_acquire_blocking(&vga_start_semaphore);
    while (true) {
#ifdef VGA_HDMI
        hdmi_poll_reinit();
#endif
#ifdef TFT
        refresh_lcd();
#endif
        pcm_call();
#if defined(USE_GS) && !defined(SOFTTV)
        // Wall-clock-locked: runs GS-Z80 at exactly 12 MHz off core0.
        // Under SOFTTV, GS::pump() runs in pcm_call_inner (core0) instead,
        // because video_timer_callbackTV at 30 kHz would starve it here.
        GS::pump();
#endif
        tight_loop_contents();
    }
    __unreachable();
}

#if SOFTTV
typedef struct tv_out_mode_t {
    // double color_freq;
    float color_index;
    COLOR_FREQ_t c_freq;
    enum graphics_mode_t mode_bpp;
    g_out_TV_t tv_system;
    NUM_TV_LINES_t N_lines;
    bool cb_sync_PI_shift_lines;
    bool cb_sync_PI_shift_half_frame;
} tv_out_mode_t;
extern tv_out_mode_t tv_out_mode;

bool color_mode=true;
bool toggle_color() {
    color_mode=!color_mode;
    if(color_mode) {
        tv_out_mode.color_index= 1.0f;
    } else {
        tv_out_mode.color_index= 0.0f;
    }

    return true;
}
#endif

uint8_t psram_pin;
#if PICO_RP2350
#include <hardware/exception.h>
#include <hardware/structs/qmi.h>
#include <hardware/structs/xip.h>
#include <hardware/regs/sysinfo.h>

#ifdef BUTTER_PSRAM_GPIO
#define MB16 (16ul << 20)
#define MB8 (8ul << 20)
#define MB4 (4ul << 20)
#define MB1 (1ul << 20)
uint8_t* PSRAM_DATA = (uint8_t*)0x11000000;
static int BUTTER_PSRAM_SIZE = -1;

static void __not_in_flash_func(psram_retiming)() {
    const int max_psram_freq = Config::max_psram_freq * MHZ;
    const int clock_hz = clock_get_hz(clk_sys);
    int divisor = (clock_hz + max_psram_freq - 1) / max_psram_freq;
    if (divisor == 1 && clock_hz > 100000000) divisor = 2;
    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000) rxdelay += 1;
    const int clock_period_fs = 1000000000000000ll / clock_hz;
    const int max_select = (125 * 1000000) / clock_period_fs;
    const int min_deselect = (18 * 1000000 + (clock_period_fs - 1)) / clock_period_fs - (divisor + 1) / 2;
    qmi_hw->m[1].timing = 1 << QMI_M1_TIMING_COOLDOWN_LSB |
                          QMI_M1_TIMING_PAGEBREAK_VALUE_1024 << QMI_M1_TIMING_PAGEBREAK_LSB |
                          max_select << QMI_M1_TIMING_MAX_SELECT_LSB |
                          min_deselect << QMI_M1_TIMING_MIN_DESELECT_LSB |
                          rxdelay << QMI_M1_TIMING_RXDELAY_LSB |
                          divisor << QMI_M1_TIMING_CLKDIV_LSB;
}
uint32_t __not_in_flash_func(butter_psram_size)() {
#if BUTTER_PSRAM_GPIO == 255
    return 0;   // PSRAM disabled via CMake kill-switch (set(PSRAM OFF))
#else
    if (BUTTER_PSRAM_SIZE != -1) return BUTTER_PSRAM_SIZE;
    for(register int i = MB8; i < MB16; i += 4096)
        PSRAM_DATA[i] = 16;
    for(register int i = MB4; i < MB8; i += 4096)
        PSRAM_DATA[i] = 8;
    for(register int i = MB1; i < MB4; i += 4096)
        PSRAM_DATA[i] = 4;
    for(register int i = 0; i < MB1; i += 4096)
        PSRAM_DATA[i] = 1;
    register uint32_t res = PSRAM_DATA[MB16 - 4096];
    for (register int i = MB16 - MB1; i < MB16; i += 4096) {
        if (res != PSRAM_DATA[i]) {
            BUTTER_PSRAM_SIZE = 0;
            return 0;
        }
    }
    // A floating bus (no chip) can read back a consistent garbage value (e.g.
    // 0xFF); only the markers actually planted by the probe are trustworthy.
    if (res != 1 && res != 4 && res != 8 && res != 16)
        res = 0;
    BUTTER_PSRAM_SIZE = res << 20;
    return BUTTER_PSRAM_SIZE;
#endif
}

// Probe for a PSRAM chip on XIP CS1 via QMI direct mode: exit QPI (0xF5) in
// case the chip is still in QPI after a warm reboot, then Read ID (0x9F).
// APS6404-compatible chips answer MF ID 0x0D, KGD 0x5D; with no chip the SD
// lines float and read 0x00/0xFF. Must run entirely from RAM: while direct
// mode is enabled any flash (CS0) access would deadlock the QMI.
static bool __no_inline_not_in_flash_func(psram_detect)() {
    for (int attempt = 0; attempt < 3; ++attempt) {
        // Exit QPI mode (quad-width single byte, output enabled)
        qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
                            (QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB) |
                            0xF5;
        while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
            ;
        (void)qmi_hw->direct_rx;
        qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        for (volatile int d = 0; d < 64; ++d)   // respect min CS deselect time
            ;

        // Read ID: 0x9F + 3 address bytes, then MF ID and KGD
        qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        uint8_t mfid = 0, kgd = 0;
        for (int i = 0; i < 6; ++i) {
            qmi_hw->direct_tx = (i == 0) ? 0x9F : 0xFF;
            while (!(qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS))
                ;
            while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
                ;
            uint8_t b = (uint8_t)qmi_hw->direct_rx;
            if (i == 4) mfid = b;
            else if (i == 5) kgd = b;
        }
        qmi_hw->direct_csr &= ~QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        for (volatile int d = 0; d < 64; ++d)
            ;

        if (kgd == 0x5D || mfid == 0x5D)
            return true;
        // The first transaction after a chip reset is known to come back
        // garbage on some chips (see init_psram in psram_spi.c) — retry.
    }
    return false;
}
void __no_inline_not_in_flash_func(psram_init)(uint cs_pin) {
    gpio_set_function(cs_pin, GPIO_FUNC_XIP_CS1);

    // DIRECT-MODE WINDOW — strict discipline (root cause of the intermittent
    // post-reset boot hang, hw-traced 2026-07-07): while DIRECT_CSR.EN is set,
    // ANY flash (CS0) fetch stalls the core forever. That includes (a) calls
    // into flash-resident code — psram_retiming() used to run inside this
    // window and its clock_get_hz() is flash code, so the boot hung whenever
    // that line wasn't already XIP-cache-resident — and (b) any IRQ whose
    // handler lives in flash (USB is live here: tuh_init ran before us). So:
    // IRQs off for the whole window, nothing but RAM code inside, and the
    // window ends BEFORE psram_retiming/format setup (plain register writes
    // that don't need direct mode).
    const uint32_t ints = save_and_disable_interrupts();

    // Enable direct mode (manual CS for the ID probe), clkdiv of 30.
    qmi_hw->direct_csr = 30 << QMI_DIRECT_CSR_CLKDIV_LSB | QMI_DIRECT_CSR_EN_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // No chip on CS1 (board assembled without PSRAM): leave XIP CS1 unconfigured.
    // Without this check the size probe below reads a floating bus, which can
    // return a consistent garbage value and report a chip that is not there.
    if (!psram_detect()) {
        qmi_hw->direct_csr = 0;
        restore_interrupts(ints);
        BUTTER_PSRAM_SIZE = 0;
        return;
    }

    // Re-enter direct mode with auto CS, clkdiv of 10.
    qmi_hw->direct_csr = 10 << QMI_DIRECT_CSR_CLKDIV_LSB | \
                               QMI_DIRECT_CSR_EN_BITS | \
                               QMI_DIRECT_CSR_AUTO_CS1N_BITS;
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // Enable QPI mode on the PSRAM
    const uint CMD_QPI_EN = 0x35;
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS | CMD_QPI_EN;

    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS)
        ;

    // END of the direct-mode window — flash code is safe again from here on.
    qmi_hw->direct_csr = 0;
    restore_interrupts(ints);

    psram_retiming();

    // Set PSRAM commands and formats
    qmi_hw->m[1].rfmt =
        QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB |\
        QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_RFMT_ADDR_WIDTH_LSB |\
        QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |\
        QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_RFMT_DUMMY_WIDTH_LSB |\
        QMI_M0_RFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_RFMT_DATA_WIDTH_LSB |\
        QMI_M0_RFMT_PREFIX_LEN_VALUE_8   << QMI_M0_RFMT_PREFIX_LEN_LSB |\
        6                                << QMI_M0_RFMT_DUMMY_LEN_LSB;

    qmi_hw->m[1].rcmd = 0xEB;

    qmi_hw->m[1].wfmt =
        QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB |\
        QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q   << QMI_M0_WFMT_ADDR_WIDTH_LSB |\
        QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB |\
        QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q  << QMI_M0_WFMT_DUMMY_WIDTH_LSB |\
        QMI_M0_WFMT_DATA_WIDTH_VALUE_Q   << QMI_M0_WFMT_DATA_WIDTH_LSB |\
        QMI_M0_WFMT_PREFIX_LEN_VALUE_8   << QMI_M0_WFMT_PREFIX_LEN_LSB;

    qmi_hw->m[1].wcmd = 0x38;

    // Enable writes to PSRAM
    hw_set_bits(&xip_ctrl_hw->ctrl, XIP_CTRL_WRITABLE_M1_BITS);

    // init size
    butter_psram_size();
}
#else
uint8_t* PSRAM_DATA = (uint8_t*)0;
uint32_t __not_in_flash_func(butter_psram_size)() { return 0; }
#endif
#else
uint8_t* PSRAM_DATA = (uint8_t*)0;
uint32_t __not_in_flash_func(butter_psram_size)() { return 0; }
#endif

// Linker-defined per-core stack bounds: core0 = SCRATCH_Y (__Stack*),
// core1 = SCRATCH_X (__StackOne*). The fault can fire on EITHER core, so pick the
// right bounds by the SIO CPUID — otherwise a normal core1 SP looks like a core0
// "overflow" (it sits in SCRATCH_X, below __StackBottom) and mislabels the fault.
extern char __StackBottom;
extern char __StackTop;
extern char __StackOneBottom;
extern char __StackOneTop;

// C linkage so the naked-asm `b sigbus_handler` resolves without mangling.
extern "C" void sigbus_handler(uint32_t *frame) {
    static int count = 0;
    if (++count > 3) return;
    // Exception frame: [0]=r0 [1]=r1 [2]=r2 [3]=r3 [4]=r12 [5]=LR [6]=PC [7]=xPSR.
    // LR = the return address of the function that faulted (addr2line that).
    // `frame` ~= the SP at fault time; ovf = SP below this core's stack bottom.
    uint32_t pc   = frame[6];
    uint32_t lr   = frame[5];
    uint32_t sp   = (uint32_t)frame;
    uint32_t core = *(volatile uint32_t*)0xD0000000u;   // SIO CPUID (0 or 1)
    uint32_t bot  = core ? (uint32_t)(uintptr_t)&__StackOneBottom : (uint32_t)(uintptr_t)&__StackBottom;
    uint32_t top  = core ? (uint32_t)(uintptr_t)&__StackOneTop    : (uint32_t)(uintptr_t)&__StackTop;
    int ovf = (sp < bot);
#ifndef PICO_RP2040
    // CFSR/BFAR only exist on Cortex-M3+ (not M0+)
    uint32_t cfsr = *(volatile uint32_t*)0xE000ED28u;
    uint32_t bfar = *(volatile uint32_t*)0xE000ED38u;
    // PICO_USE_STACK_GUARDS: on an MSPLIM violation (UFSR.STKOF, CFSR bit 20)
    // v8-M clamps SP to the limit and suppresses the frame push — so sp==bot,
    // frame[] contents are garbage, and only the STKOF bit tells the story.
    if (cfsr & (1u << 20)) ovf = 1;
    printf("SIGBUS[%d] core%u: PC=%08x LR=%08x SP=%08x CFSR=%08x BFAR=%08x stackOvf=%d (bot=%08x top=%08x)\n",
           count, (unsigned)core, (unsigned)pc, (unsigned)lr, (unsigned)sp, (unsigned)cfsr, (unsigned)bfar,
           ovf, (unsigned)bot, (unsigned)top);
#else
    printf("SIGBUS[%d] core%u: PC=%08x LR=%08x SP=%08x stackOvf=%d\n",
           count, (unsigned)core, (unsigned)pc, (unsigned)lr, (unsigned)sp, ovf);
#endif
}
// Naked trampoline: pass the stacked exception frame to sigbus_handler.
// EXC_RETURN bit 2: 0 = MSP, 1 = PSP was active when the fault fired.
#ifdef PICO_RP2040
// Cortex-M0+: no IT blocks, TST only register form, high-reg immediates forbidden.
void __attribute__((naked)) sigbus(void) {
    __asm volatile (
        "movs r0, #4\n"
        "mov  r1, lr\n"
        "tst  r0, r1\n"       // Z=1 if bit2(LR)=0 → MSP
        "bne  1f\n"
        "mrs  r0, msp\n"
        "b    sigbus_handler\n"
        "1:\n"
        "mrs  r0, psp\n"
        "b    sigbus_handler\n"
    );
}
#else
void __attribute__((naked)) sigbus(void) {
    __asm volatile (
        "tst    lr, #4\n"
        "ite    eq\n"
        "mrseq  r0, msp\n"
        "mrsne  r0, psp\n"
        "b      sigbus_handler\n"
    );
}
#endif
void __attribute__((naked, noreturn)) __printflike(1, 0) dummy_panic(__unused const char *fmt, ...) {
    printf("*** PANIC ***\n");
    if (fmt)
        printf(fmt);
}

#ifndef PICO_RP2040
void __not_in_flash() flash_timings(int mhz) {
        const int max_flash_freq = Config::max_flash_freq * MHZ;
        const int clock_hz = mhz * MHZ;
        int divisor = (clock_hz + max_flash_freq - 1) / max_flash_freq;
        if (divisor == 1 && clock_hz > 100000000) {
            divisor = 2;
        }
        int rxdelay = divisor;
        if (clock_hz / divisor > 100000000) {
            rxdelay += 1;
        }
        qmi_hw->m[0].timing = 0x60007000 |
                            rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                            divisor << QMI_M0_TIMING_CLKDIV_LSB;
}
#endif

static void __not_in_flash_func(flash_info)() {
    if (rx[0] == 0) {
        uint8_t tx[4] = {0x9f};
        flash_do_cmd(tx, rx, 4);
    }
}

#if !PICO_RP2040
// Flash QE bit fix for Puya flash on RP2350 boards (see fhoedemakers/flash_config).
// Puya ships with SR2.QE=0 and its 01h command writes SR1 only, so boot2's
// Winbond-style 2-byte 01h status write silently fails — quad XIP keeps sampling
// WP#/HOLD# and locks up under overclock. Must run BEFORE flash_timings()/
// set_sys_clock — the overclock is what triggers the lockup.
//
// The whole command sequence runs inside ONE exit-XIP window. Per the PY25Q128HA
// datasheet, Volatile SR Write Enable (50h) must be IMMEDIATELY followed by the
// Write Status Register command — no other flash commands in between. Chaining
// flash_do_cmd() calls violates that: each call's epilogue re-runs boot2, which
// itself talks to the flash (SR2 check + its own Winbond-style SR write attempt),
// clearing the volatile-WE latch — hw-confirmed as "FIX FAILED" on ZERO2.
// Preferred path is the volatile SR2 write (instant, zero wear, re-applied each
// boot); if the chip ignores it, fall back to a one-time NON-volatile write
// (06h + 31h + WIP poll — the fhoedemakers-proven sequence).
// 0=n/a (not Puya)  1=already set  2=volatile  3=non-volatile 31h
// 4=FAILED  5=non-volatile 01h 2-byte  6=raw-window self-test failed
uint8_t flash_qe = 0;

const char* flash_qe_text() {
    switch (flash_qe) {
        case 1: return "set";
        case 2: return "set (volatile)";
        case 3: return "set (non-volatile)";
        case 5: return "set (01h fallback)";
        case 6: return "WINDOW FAULT";
        default: return "FIX FAILED";
    }
}

// One CS-framed command inside an open exit-XIP window (mirror of the QMI half
// of the SDK's flash_do_cmd). Caller guarantees: XIP exited, IRQs off, and no
// flash-resident code touched until the window is closed.
// NOTE: all three must be __no_inline_not_in_flash_func, NOT __not_in_flash_func:
// the latter permits inlining, and MinSizeRel inlined the whole window body into
// flash-resident main() — executing from (dead) XIP inside the window, hardfault
// at the first XIP-cache miss (hw-traced on ZERO2 with the debug probe).
static void __no_inline_not_in_flash_func(qe_cmd_raw)(const uint8_t *tx, uint8_t *rxb, size_t n) {
    hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS0N_BITS);   // CS low
    hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    size_t txr = n, rxr = n;
    while (txr || rxr) {
        uint32_t flags = qmi_hw->direct_csr;
        if (txr && !(flags & QMI_DIRECT_CSR_TXFULL_BITS)) {
            qmi_hw->direct_tx = *tx++;
            --txr;
        }
        if (rxr && !(flags & QMI_DIRECT_CSR_RXEMPTY_BITS)) {
            uint8_t b = (uint8_t)qmi_hw->direct_rx;
            if (rxb) *rxb++ = b;
            --rxr;
        }
    }
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_ASSERT_CS0N_BITS); // CS high
}

static uint8_t __no_inline_not_in_flash_func(qe_read_reg)(uint8_t cmd) {
    uint8_t tx[2] = { cmd, 0 }, rxb[2];
    qe_cmd_raw(tx, rxb, 2);
    return rxb[1];
}

// Diagnostics captured inside the window, logged afterwards:
// [0]=in-window JEDEC MF (self-test vs rx[1]) [1]=SR1 [2]=SR2 initial
// [3]=SR2 after volatile try [4]=SR1 after 06h (WEL check) [5]=SR2 final
uint8_t flash_qe_diag[6];

static void __no_inline_not_in_flash_func(flash_qe_fix)() {
    if (rx[1] != 0x85)                       // Puya only; Winbond is factory-set,
        return;                              // other vendors have different SR layouts
    // boot2 copy to re-enter fast XIP afterwards (on RP2350 crt0 parks boot2 in BOOTRAM)
    static uint32_t boot2_copy[64];
    const volatile uint32_t *b2 = (const volatile uint32_t *)BOOTRAM_BASE;
    for (int i = 0; i < 64; ++i)
        boot2_copy[i] = b2[i];
    __compiler_memory_barrier();

    rom_connect_internal_flash_fn connect_flash =
        (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
    rom_flash_exit_xip_fn exit_xip =
        (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
    rom_flash_flush_cache_fn flush_cache =
        (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);

    // ROM's exit_xip resets the QMI CS1 window to the clean 03h config — save and
    // restore it like the SDK does (harmless this early: psram_init runs later,
    // but keeps this function safe to call at any point).
    uint32_t m1_timing = qmi_hw->m[1].timing;
    uint32_t m1_rcmd   = qmi_hw->m[1].rcmd;
    uint32_t m1_rfmt   = qmi_hw->m[1].rfmt;
    uint32_t pads_save[count_of(pads_qspi_hw->io)];
    for (size_t i = 0; i < count_of(pads_qspi_hw->io); ++i)
        pads_save[i] = pads_qspi_hw->io[i];

    const uint32_t ints = save_and_disable_interrupts();
    connect_flash();
    exit_xip();
    // Pull SD2 (WP#) and SD3 (HOLD#) high during the window: with QE=0 the chip
    // interprets them as control pins, and in serial direct mode the QMI leaves
    // them undriven. A floating/low WP# + SRP0 hardware-protects the status
    // registers — every SR write is then silently ignored (pads are io[3]/io[4]:
    // SCLK, SD0, SD1, SD2, SD3, SS).
    hw_write_masked(&pads_qspi_hw->io[3], PADS_QSPI_GPIO_QSPI_SD2_PUE_BITS,
                    PADS_QSPI_GPIO_QSPI_SD2_PUE_BITS | PADS_QSPI_GPIO_QSPI_SD2_PDE_BITS);
    hw_write_masked(&pads_qspi_hw->io[4], PADS_QSPI_GPIO_QSPI_SD3_PUE_BITS,
                    PADS_QSPI_GPIO_QSPI_SD3_PUE_BITS | PADS_QSPI_GPIO_QSPI_SD3_PDE_BITS);
    // ---- window open: only qe_* helpers below (all RAM-resident) ----
    // Self-test: re-read JEDEC ID through our raw path; must match flash_info()'s.
    uint8_t jtx[4] = { 0x9f, 0, 0, 0 }, jrx[4];
    qe_cmd_raw(jtx, jrx, 4);
    flash_qe_diag[0] = jrx[1];
    flash_qe_diag[1] = qe_read_reg(0x05);
    uint8_t sr2 = qe_read_reg(0x35);
    flash_qe_diag[2] = sr2;
    if (jrx[1] != rx[1] || jrx[2] != rx[2] || jrx[3] != rx[3]) {
        flash_qe = 6;                                     // raw window broken — don't write anything
    } else if (sr2 & 0x02) {
        flash_qe = 1;                                     // QE already set
    } else {
        uint8_t wr31[2] = { 0x31, (uint8_t)(sr2 | 0x02) };
        const uint8_t c50 = 0x50, c06 = 0x06;
        qe_cmd_raw(&c50, NULL, 1);                        // Volatile SR Write Enable
        qe_cmd_raw(wr31, NULL, 2);                        // Write SR2 (volatile copy)
        for (volatile int i = 0; i < 2000; ++i);          // settle (volatile write is ~instant)
        flash_qe_diag[3] = qe_read_reg(0x35);
        if (flash_qe_diag[3] & 0x02) {
            flash_qe = 2;
        } else {
            // Volatile write ignored — one-time non-volatile write instead.
            qe_cmd_raw(&c06, NULL, 1);                    // Write Enable (WEL)
            flash_qe_diag[4] = qe_read_reg(0x05);
            if (flash_qe_diag[4] & 0x02) {                // WEL latched?
                qe_cmd_raw(wr31, NULL, 2);
                for (int i = 0; i < 20000; ++i)           // WIP poll, tW max ~12 ms
                    if (!(qe_read_reg(0x05) & 0x01))
                        break;
                flash_qe = (qe_read_reg(0x35) & 0x02) ? 3 : 4;
                if (flash_qe == 4) {
                    // Last resort: Winbond-style 2-byte 01h write (SR1+SR2) —
                    // some Puya parts route SR2 only through this form.
                    uint8_t wr01[3] = { 0x01, qe_read_reg(0x05), (uint8_t)(sr2 | 0x02) };
                    wr01[1] &= (uint8_t)~0x03;            // don't write back WIP/WEL
                    qe_cmd_raw(&c06, NULL, 1);
                    qe_cmd_raw(wr01, NULL, 3);
                    for (int i = 0; i < 20000; ++i)
                        if (!(qe_read_reg(0x05) & 0x01))
                            break;
                    flash_qe = (qe_read_reg(0x35) & 0x02) ? 5 : 4;
                }
            } else {
                flash_qe = 4;
            }
        }
    }
    flash_qe_diag[5] = qe_read_reg(0x35);
    // ---- close window: flush cache, re-enter fast XIP via boot2 ----
    flush_cache();
    ((void (*)(void))((intptr_t)boot2_copy + 1))();
    qmi_hw->m[1].timing = m1_timing;
    qmi_hw->m[1].rcmd   = m1_rcmd;
    qmi_hw->m[1].rfmt   = m1_rfmt;
    for (size_t i = 0; i < count_of(pads_qspi_hw->io); ++i)
        pads_qspi_hw->io[i] = pads_save[i];
    restore_interrupts(ints);
}
#endif

// Try to switch sys clock with PLL lock timeout.
// Returns true if PLL locked and clock switched; false if PLL did not lock
// (system remains on previous clock).
static bool __not_in_flash_func(try_set_sys_clock_khz)(uint32_t freq_khz) {
    uint vco, postdiv1, postdiv2;
    if (!check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2))
        return false;

    // Switch clk_sys to USB PLL (48 MHz) while we reconfigure sys PLL
    clock_configure_undivided(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        USB_CLK_HZ);

    // Reset and configure sys PLL
    uint32_t ref_freq = XOSC_HZ / PLL_SYS_REFDIV;
    uint32_t fbdiv = vco / ref_freq;
    reset_unreset_block_num_wait_blocking(RESET_PLL_SYS);
    pll_sys_hw->cs = PLL_SYS_REFDIV;
    pll_sys_hw->fbdiv_int = fbdiv;
    hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);

    // Wait for PLL lock with timeout (~50ms at 48MHz USB clock)
    for (int i = 0; i < 500000; i++) {
        if (pll_sys_hw->cs & PLL_CS_LOCK_BITS) {
            // PLL locked — enable post dividers
            pll_sys_hw->prim = (postdiv1 << PLL_PRIM_POSTDIV1_LSB) |
                               (postdiv2 << PLL_PRIM_POSTDIV2_LSB);
            hw_clear_bits(&pll_sys_hw->pwr, PLL_PWR_POSTDIVPD_BITS);

            uint32_t freq = vco / (postdiv1 * postdiv2);

            // Switch clk_sys back to sys PLL
            clock_configure_undivided(clk_sys,
                CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                freq);
            return true;
        }
        tight_loop_contents();
    }

    // PLL did not lock — restore sys PLL to power-down, switch back to USB PLL
    hw_set_bits(&pll_sys_hw->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS | PLL_PWR_POSTDIVPD_BITS);
    return false;
}

#if !PICO_RP2040
// Switch clk_sys to `mhz` and re-tune QMI flash (+ PSRAM) timing to match it — IRQs
// off and entirely from RAM — then drop the XIP cache. After a flash erase/program the
// bootrom leaves XIP at a conservative DEFAULT timing; reading flash at the overclock
// with that stale timing faults intermittently (same QMI hazard as the clock-switch
// block in main()). Buffer's flash-write window calls this to drop to 252 MHz for the
// write and to restore the running clock + correct timing afterwards.
void __not_in_flash_func(board_set_clock_and_timing)(uint32_t mhz) {
    const uint32_t ints = save_and_disable_interrupts();
    if (!try_set_sys_clock_khz(mhz * KHZ))
        set_sys_clock_khz(mhz * KHZ, true);
    flash_timings((int)mhz);
#if defined(BUTTER_PSRAM_GPIO) && BUTTER_PSRAM_GPIO != 255
    psram_retiming();
#endif
    xip_cache_invalidate_all();
    restore_interrupts(ints);
}
#endif

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
extern "C" uint8_t linkVGA01;
#endif
extern "C" int testPins(uint32_t pin0, uint32_t pin1);

// TinyUSB routes failed TU_ASSERTs here instead of a bkpt instruction (see
// CFG_TUSB_DEBUG_BREAKPOINT in tusb_config.h) — a bkpt freezes every debug
// session on recoverable asserts (dongle re-enumeration). Count and move on.
volatile uint32_t g_tusb_assert_count = 0;
extern "C" void picospec_tusb_assert_hook(void) { g_tusb_assert_count++; }

int main() {
    uptime_init();   // capture pre-reboot uptime from watchdog scratch (see uptime_seconds)
#if defined(DBG_UART_ENABLED) && defined(PICO_DEFAULT_UART)
    // Early console at the boot clock: proves bootrom/crt0 completed and logs the
    // reset reason before any flash/clock/PSRAM bring-up (the pre-stdio window
    // used to be silent, which made early-boot hangs undebuggable). The UART
    // divider goes stale after set_sys_clock; stdio_init_all below re-inits it,
    // so no prints in between.
    uart_init(uart_default, 115200);
    gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    Debug::log("main: entry, wd_reboot=%d", (int)watchdog_caused_reboot());
#if !PICO_RP2040
    // Decode WHY the chip reset (hw-traced 2026-07-21: "wd_reboot=0 mid-ZIP-extract"
    // reboots were undiagnosable — POR/BOR here means the supply sagged, RUN means
    // the reset button / debug probe, WDG means our own esp_hard_reset/crash path).
    {
        uint32_t cr = powman_hw->chip_reset;
        Debug::log("main: chip_reset=%08X%s%s%s%s%s%s", (unsigned)cr,
                   (cr & POWMAN_CHIP_RESET_HAD_POR_BITS)                  ? " POR"    : "",
                   (cr & POWMAN_CHIP_RESET_HAD_BOR_BITS)                  ? " BOR"    : "",
                   (cr & POWMAN_CHIP_RESET_HAD_RUN_LOW_BITS)              ? " RUN"    : "",
                   (cr & (POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_SWCORE_BITS |
                          POWMAN_CHIP_RESET_HAD_WATCHDOG_RESET_PSM_BITS)) ? " WDG"    : "",
                   (cr & POWMAN_CHIP_RESET_HAD_GLITCH_DETECT_BITS)        ? " GLITCH" : "",
                   (cr & POWMAN_CHIP_RESET_HAD_DP_RESET_REQ_BITS)         ? " DBG"    : "");
    }
#endif
    uart_tx_wait_blocking(uart_default);   // drain before the clock switch garbles it
#endif
    flash_info();
#if !PICO_RP2040
    flash_qe_fix();
    if (flash_qe)
        Debug::log("main: flash QE (Puya) %s | jed=%02X sr1=%02X sr2=%02X vol=%02X wel=%02X fin=%02X",
                   flash_qe_text(), flash_qe_diag[0], flash_qe_diag[1], flash_qe_diag[2],
                   flash_qe_diag[3], flash_qe_diag[4], flash_qe_diag[5]);
#endif
#ifdef PICO_RP2040
    vreg_set_voltage(VREG_VOLTAGE_MAX); // 1.30V — max for RP2040
    sleep_ms(10);
    if (!set_sys_clock_khz(CPU_MHZ * KHZ, false)) {
        set_sys_clock_khz(252 * KHZ, true); // fallback to 252MHz
    }

#else
    #if 0
        vreg_set_voltage(VREG_VOLTAGE_1_10); // Set voltage  //
        delay(100);
        set_sys_clock_khz(CPU_MHZ * KHZ, true);
    #else
        vreg_disable_voltage_limit();
        vreg_set_voltage(VREG_VOLTAGE_1_60);
        flash_timings(CPU_MHZ);
        sleep_ms(100);

        if (!set_sys_clock_khz(CPU_MHZ * KHZ, 0)) {
            #define CPU_MHZ 252
            set_sys_clock_khz(CPU_MHZ * KHZ, 1); // fallback to failsafe clocks
        }
    #endif
#endif

#if defined(DBG_UART_ENABLED)
    // Console UART explicitly enabled via <BOARD>_DBG_UART. We deliberately gate on
    // DBG_UART_ENABLED, NOT on PICO_DEFAULT_UART_TX_PIN: the board header always
    // defines a default UART (uart0/GP0-1 on pico2), which on PICO_DV is the ZiFi
    // line — calling stdio_init_all() there would grab GP0/1 and break the NIC/FTP.
    stdio_init_all();
#endif

#ifdef KBDUSB
    tuh_init(BOARD_TUH_RHPORT);
    ps2kbd.init_gpio();
#else
    keyboard_init();
#endif

    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 6; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
    #endif

#if PICO_RP2350
    #ifdef BUTTER_PSRAM_GPIO
    #if BUTTER_PSRAM_GPIO == 255
        psram_pin = 255;   // PSRAM disabled via CMake kill-switch (set(PSRAM OFF))
    #else
        psram_pin = chip_is_rp2350a() ? BUTTER_PSRAM_GPIO : 47;
        Debug::log("main: psram_init begin");
        psram_init(psram_pin);
        butter_psram_size();
        Debug::log("main: butter=%u KB", butter_psram_size() >> 10);
    #endif
    #endif
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, sigbus);
#endif


// NESPAD init moved below ESPectrum::setup() so Config (and thus the ZiFi pin
// selection) is loaded first — see the guarded nespad_begin() after setup().

#if !defined(BUTTER_PSRAM_GPIO) || BUTTER_PSRAM_GPIO != 255   // skip when PSRAM kill-switch on (set(PSRAM OFF))
#if PICO_RP2350
    if (butter_psram_size() == 0 || psram_pin != PSRAM_PIN_SCK) {
#endif
    #ifndef MURM2
        Debug::log("main: init_psram begin");
        init_psram();
    #endif
#if PICO_RP2350
    }
#endif
#endif
    Debug::log("main: psram init done");
    // send kbd reset only after initial process passed
#ifndef KBDUSB
    keyboard_send(0xFF);
#endif

    #if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    Debug::log("main: testPins begin");
    linkVGA01 = testPins(VGA_BASE_PIN, VGA_BASE_PIN + 1);
    Debug::log("main: testPins=%02X", linkVGA01);
    #endif

    Debug::log("main: before ESPectrum::setup()");
    ESPectrum::setup();
    Debug::log("main: after ESPectrum::setup()");
    Debug::log2SD("main: after ESPectrum::setup()");

    // NOTE: GM.DLS bank + pending ALF cartridge flash provisioning now runs inside
    // ESPectrum::setup(), right after Config::load() and BEFORE VIDEO::Init(). It must
    // precede VIDEO::Init() or the live HDMI engine stalls the QMI bus during the flash
    // erase (XIP-PSRAM goes away with XIP-flash) and hangs the board. See ESPectrum.cpp.

#if USE_NESPAD
    // Bring up the NES gamepad now that Config is loaded — unless ZiFi (RP2350)
    // has claimed any of its pins, in which case yield so the UART owns them.
    {
        bool nes_yield = false;
#if !PICO_RP2040
        nes_yield = BoardPins::zifiOwnsPin(NES_GPIO_CLK) ||
                    BoardPins::zifiOwnsPin(NES_GPIO_DATA) ||
                    BoardPins::zifiOwnsPin(NES_GPIO_LAT);
#endif
        if (!nes_yield) {
            nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);
            nespad_active = true;
        } else {
            Debug::log("NESPAD: yielded to ZiFi (pins claimed)");
        }
    }
#endif
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    for (int i = 0; i < 6; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
    #endif
    #if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    {
        FIL f;
        f_open(&f, CONFIG_DIR "/video_detect.code", FA_WRITE | FA_CREATE_ALWAYS);
        char buf[16] = {0};
        snprintf(buf, 16, "%02Xh\n", linkVGA01);
        UINT bw;
        f_write(&f, buf, strlen(buf), &bw);
        f_close(&f);
    }
    #endif

// #if defined(VGA_HDMI) && !defined(PICO_RP2040)
//     {
//         // 720x576 modes use real PAL pixel clock (27MHz).
//         // 405MHz is NOT achievable (PLL VCO would be 1620MHz, exceeds 1600MHz max).
//         // 270MHz sys_clk + pio_clk_div=1.0 → TMDS=270MHz → pixel=27MHz exactly.
//         // VCO=1080MHz (FBDIV=90, PD1=4): achievable on RP2350 and faster than ZERO2 (252MHz).
//         extern bool SELECT_VGA;
//         if (!SELECT_VGA && Config::hdmi_video_mode >= Config::VM_720x576_50) {
//             // Update QMI flash timing for 270MHz before changing sys_clk
//             const int new_mhz = 270;
//             const int max_flash = 66 * MHZ;
//             int divisor = (new_mhz * MHZ + max_flash - 1) / max_flash; // ceil(270/66) = 5
//             if (divisor == 1 && new_mhz * MHZ > 100000000) divisor = 2;
//             int rxdelay = divisor;
//             if (new_mhz * MHZ / divisor > 100000000) rxdelay += 1;
//             qmi_hw->m[0].timing = 0x60007000 |
//                                   rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
//                                   divisor << QMI_M0_TIMING_CLKDIV_LSB;
//             set_sys_clock_khz(new_mhz * KHZ, true);
//         }
//     }
// #endif

    // Apply saved CPU frequency and flash/PSRAM timing from Config
    {
#if !PICO_RP2040
        // Apply saved vreg voltage (vreg_disable_voltage_limit already called at boot)
        Debug::log2SD("main: vreg_set_voltage %d", (int)Config::vreq_voltage);
        vreg_set_voltage((enum vreg_voltage)Config::vreq_voltage);
        sleep_ms(10);
#endif
        uint16_t running_mhz = clock_get_hz(clk_sys) / 1000000;
        Debug::log2SD("main: cpu_mhz cfg=%d running=%d", (int)Config::cpu_mhz, (int)running_mhz);

        // CRITICAL ordering note: ESPectrum::setup() already ran, so GS is live
        // and the audio repeating-timer ISR fires on core0 at ~Audio_freq. That
        // ISR (pcm_call_inner) touches GS state and, with HDMI audio, flash/PSRAM.
        // try_set_sys_clock_khz() briefly drops clk_sys to the USB PLL and brings
        // it back at the new rate, while flash/PSRAM QMI timing still reflects the
        // boot clock until flash_timings()/psram_retiming() run. If that ISR (or
        // any flash/PSRAM access) lands inside this window at the higher clock with
        // stale QMI timing, the QMI hangs — observed as a freeze on the
        // try_set_sys_clock_khz() line at 252/504 MHz with GS + HDMI audio enabled
        // (378 MHz = boot clock, so the whole block is skipped). Run the switch and
        // the timing fix-up as one interrupt-disabled critical section, then bring
        // IRQs back only once flash/PSRAM timing matches the new clock. All four
        // helpers below are __not_in_flash so they execute safely with IRQs off.
        bool clk_changed = (Config::cpu_mhz != running_mhz);
        bool clk_locked  = true;
        {
            const uint32_t ints = save_and_disable_interrupts();
            if (clk_changed) {
                clk_locked = try_set_sys_clock_khz(Config::cpu_mhz * KHZ);
                if (!clk_locked) {
                    // PLL did not lock — restore original PLL
                    set_sys_clock_khz(running_mhz * KHZ, true);
                }
            }
            const uint16_t applied_mhz = clk_locked ? Config::cpu_mhz : running_mhz;
            // Re-apply flash/PSRAM timing for the now-active clock BEFORE re-enabling
            // interrupts, so the next flash/PSRAM access (incl. the audio ISR) is safe.
#ifndef PICO_RP2040
            flash_timings(applied_mhz);
#endif
#if PICO_RP2350 && defined(BUTTER_PSRAM_GPIO) && BUTTER_PSRAM_GPIO != 255
            psram_retiming();
#endif
            // Recalculate PIO SPI PSRAM clkdiv for the now-active sys_clk.
            // init_psram() ran at boot-time 378 MHz; if applied_mhz differs
            // (e.g. 504 MHz) the frozen clkdiv=1.5 drives SCK at 168 MHz — above
            // the 133 MHz spec.  Always re-apply to get a clean integer divider.
#ifdef PSRAM
            psram_update_clkdiv();
#endif
            restore_interrupts(ints);
        }

        if (clk_changed && clk_locked) {
            Debug::log2SD("main: sys_clk -> %d MHz OK", (int)Config::cpu_mhz);
#ifdef VGA_HDMI
            graphics_set_pio_clk_div((float)Config::cpu_mhz / 252.0f);
#endif
            // Reinit audio: I2S PIO divider was calculated for old sys_clk
            pcm_setup(ESPectrum::Audio_freq);
        } else if (clk_changed) {
            Debug::log2SD("main: PLL lock FAILED at %d MHz, restoring %d",
                          (int)Config::cpu_mhz, (int)running_mhz);
        }
    }

    sem_init(&vga_start_semaphore, 0, 1);
#if SOFTTV
    sem_init(&graphics_init_done_semaphore, 0, 1);
#endif
    { extern size_t getFreeHeap(void); Debug::log("main: launching core1, freeHeap=%u", (unsigned)getFreeHeap()); }
    Debug::log2SD("main: launching core1");
    multicore_launch_core1(render_core);
#if SOFTTV
    Debug::log2SD("main: waiting for graphics_init_done");
    sem_acquire_blocking(&graphics_init_done_semaphore);
    Debug::log2SD("main: graphics_init done, calling applyPalette");
    VIDEO::applyPalette();
    Debug::log2SD("main: applyPalette done");
#endif
    Debug::log2SD("main: releasing vga_start_semaphore");
    sem_release(&vga_start_semaphore);
    { extern size_t getFreeHeap(void); Debug::log("main: entering ESPectrum::loop(), freeHeap=%u", (unsigned)getFreeHeap()); }
    Debug::log2SD("main: entering ESPectrum::loop()");
    ESPectrum::loop();
    __unreachable();
}
