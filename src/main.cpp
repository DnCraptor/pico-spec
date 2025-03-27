#include <cstdio>
#include <cstring>
#include <cstdarg>

#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/vreg.h>
#include <hardware/sync.h>
#include <hardware/flash.h>

#include "ESPectrum.h"
#include "Config.h"
#include "MemESP.h"
#include "pwm_audio.h"
#include "messages.h"

#include "graphics.h"

#include "audio.h"
#include "ff.h"
#include "psram_spi.h"
#ifdef KBDUSB
    #include "ps2kbd_mrmltr.h"
#else
    #include "ps2.h"
#endif

#if USE_NESPAD
#include "nespad.h"
#endif

#pragma GCC optimize("Ofast")

#define HOME_DIR (char*)"\\SPEC"


bool cursor_blink_state = false;
uint8_t CURSOR_X, CURSOR_Y = 0;

struct semaphore vga_start_semaphore;
#include "Video.h"

static FATFS fs;

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
static input_bits_t gamepad2_bits = { false, false, false, false, false, false, false, false };

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
            case 0x5B: kbdPushData(fabgl::VirtualKey::VK_LCTRL, pressed); return true; /// L WIN
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
            case 0x5D: kbdPushData(fabgl::VirtualKey::VK_F1, pressed); return true; /// MENU
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

static void nespad_tick1(void) {
    nespad_read();
    
    { // secondJoy == 1 - first joy is in use by other (as second)

        if ((nespad_state & DPAD_SELECT) && !gamepad1_bits.select) {
            joyPushData(fabgl::VirtualKey::VK_MENU_BS, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, true);
        }
        else if (!(nespad_state & DPAD_SELECT) && gamepad1_bits.select) {
            joyPushData(fabgl::VirtualKey::VK_MENU_BS, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_SELECT, false);
        }
        gamepad1_bits.select = (nespad_state & DPAD_SELECT) != 0;

        if ((nespad_state & DPAD_A) && !gamepad1_bits.a) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_0 : fabgl::VirtualKey::VK_RETURN, true);
        }
        else if (!(nespad_state & DPAD_A) && gamepad1_bits.a) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_FIRE, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_0 : fabgl::VirtualKey::VK_RETURN, false);
        }

        if ((nespad_state & DPAD_B) && !gamepad1_bits.b) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_1 : fabgl::VirtualKey::VK_SPACE, true);
        }
        else if (!(nespad_state & DPAD_B) && gamepad1_bits.b) {
            joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_ALTFIRE, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_1 : fabgl::VirtualKey::VK_SPACE, false);
        }

        if ((nespad_state & DPAD_START) && !gamepad1_bits.start) {
            joyPushData(fabgl::VirtualKey::VK_MENU_HOME, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, true);
            if (gamepad1_bits.select || Config::joy2cursor)
                joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_F1 : fabgl::VirtualKey::VK_R, true);
        }
        else if (!(nespad_state & DPAD_START) && gamepad1_bits.start) {
            joyPushData(fabgl::VirtualKey::VK_MENU_HOME, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_START, false);
            if (gamepad1_bits.select || Config::joy2cursor)
                joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_F1 : fabgl::VirtualKey::VK_R, false);
        }

        if ((nespad_state & DPAD_UP) && !gamepad1_bits.up) {
            joyPushData(fabgl::VirtualKey::VK_MENU_UP, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEUP : fabgl::VirtualKey::VK_UP, true);
        }
        else if (!(nespad_state & DPAD_UP) && gamepad1_bits.up) {
            joyPushData(fabgl::VirtualKey::VK_MENU_UP, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_UP, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEUP : fabgl::VirtualKey::VK_UP, false);
        }

        if ((nespad_state & DPAD_DOWN) && !gamepad1_bits.down) {
            joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEDOWN : fabgl::VirtualKey::VK_DOWN, true);
        }
        else if (!(nespad_state & DPAD_DOWN) && gamepad1_bits.down) {
            joyPushData(fabgl::VirtualKey::VK_MENU_DOWN, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_DOWN, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_PAGEDOWN : fabgl::VirtualKey::VK_DOWN, false);
        }

        if ((nespad_state & DPAD_LEFT) && !gamepad1_bits.left) {
            joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_BACKSPACE : fabgl::VirtualKey::VK_LEFT, true);
        }
        else if (!(nespad_state & DPAD_LEFT) && gamepad1_bits.left) {
            joyPushData(fabgl::VirtualKey::VK_MENU_LEFT, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_LEFT, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_BACKSPACE : fabgl::VirtualKey::VK_LEFT, false);
        }

        if ((nespad_state & DPAD_RIGHT) && !gamepad1_bits.right) {
            joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, true);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, true);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_K : fabgl::VirtualKey::VK_RIGHT, true);
        }
        else if (!(nespad_state & DPAD_RIGHT) && gamepad1_bits.right) {
            joyPushData(fabgl::VirtualKey::VK_MENU_RIGHT, false);
            if (Config::secondJoy != 1) joyPushData(fabgl::VirtualKey::VK_DPAD_RIGHT, false);
            if (Config::joy2cursor) joyPushData(gamepad1_bits.select ? fabgl::VirtualKey::VK_K : fabgl::VirtualKey::VK_RIGHT, false);
        }
    }

    gamepad1_bits.a = (nespad_state & DPAD_A) != 0;
    gamepad1_bits.b = (nespad_state & DPAD_B) != 0;
    gamepad1_bits.start = (nespad_state & DPAD_START) != 0;
    gamepad1_bits.up = (nespad_state & DPAD_UP) != 0;
    gamepad1_bits.down = (nespad_state & DPAD_DOWN) != 0;
    gamepad1_bits.left = (nespad_state & DPAD_LEFT) != 0;
    gamepad1_bits.right = (nespad_state & DPAD_RIGHT) != 0;
}

static void nespad_tick2(void) {
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
 // TODO:
        case HID_KEY_GUI_LEFT: return fabgl::VirtualKey::VK_F1;
        case HID_KEY_GUI_RIGHT: return fabgl::VirtualKey::VK_F1;
        default: debug_number = kc;
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
        case fabgl::VirtualKey::VK_KP_ENTER: { // VK_KP_ENTER
            kbdPushData(Config::rightSpace ? fabgl::VirtualKey::VK_SPACE : fabgl::VirtualKey::VK_RETURN, pressed);
            return;
        }

        case fabgl::VirtualKey::VK_BACKSPACE: joyPushData(fabgl::VirtualKey::VK_MENU_BS, pressed); break;
        case fabgl::VirtualKey::VK_SPACE:     joyPushData(fabgl::VirtualKey::VK_MENU_ENTER, pressed); break;
        case fabgl::VirtualKey::VK_TAB:       if (Config::TABasfire1) JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed); break;
        case fabgl::VirtualKey::VK_LALT:      if (Config::CursorAsJoy) JPAD(fabgl::VirtualKey::VK_DPAD_FIRE, pressed); break;
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

void __not_in_flash_func(process_kbd_report)(
    hid_keyboard_report_t const *report,
    hid_keyboard_report_t const *prev_report
) {
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
            tickKbdRep1 = tickKbdRep2;
        }

#ifdef KBDUSB
        tuh_task();
#endif
}

void __scratch_x("render") render_core() {
    multicore_lockout_victim_init();
    graphics_init();

    graphics_set_buffer(NULL, DISP_WIDTH, DISP_HEIGHT); /// TODO:
    graphics_set_bgcolor(0x000000);
    graphics_set_flashmode(false, false);
    sem_acquire_blocking(&vga_start_semaphore);
    while (true) {
        pcm_call();
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

int main() {
#if !PICO_RP2040
    volatile uint32_t *qmi_m0_timing=(uint32_t *)0x400d000c;
    vreg_disable_voltage_limit();
    vreg_set_voltage(VREG_VOLTAGE_1_60);
    sleep_ms(33);
    *qmi_m0_timing = 0x60007204;
    set_sys_clock_khz(CPU_MHZ * KHZ, 0);
    *qmi_m0_timing = 0x60007204;
#else
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);
    set_sys_clock_khz(CPU_MHZ * KHZ, true);
#endif

#ifdef KBDUSB
    tuh_init(BOARD_TUH_RHPORT);
    ps2kbd.init_gpio();
#else
    keyboard_init();
#endif

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 6; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

#if USE_NESPAD
    nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);
#endif
    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(render_core);
    sem_release(&vga_start_semaphore);

    init_sound();
    pcm_setup(SOUND_FREQUENCY, SOUND_FREQUENCY);
#ifdef PSRAM
    init_psram();
#endif
    // send kbd reset only after initial process passed
#ifndef KBDUSB
    keyboard_send(0xFF);
#endif

    mem_desc_t::reset();
    ESPectrum::setup();
    ESPectrum::loop();
    __unreachable();
}
