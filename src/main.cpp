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
#include "ps2.h"

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

static input_bits_t gamepad1_bits = { false, false, false, false, false, false, false, false };
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

extern "C" bool handleScancode(const uint32_t ps2scancode) {
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
        case 0x45: kbdPushData(fabgl::VirtualKey::VK_PAUSE, pressed); return true;
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

static void nespad_tick2(void) {
    if (Config::secondJoy == 3) return;
//    if ((nespad_state2 & DPAD_A) && !gamepad2_bits.a) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_FIRE, true);
//    else if (!(nespad_state2 & DPAD_A) && gamepad2_bits.a) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_FIRE, false);
    gamepad2_bits.a = (nespad_state2 & DPAD_A) != 0;

//    if ((nespad_state2 & DPAD_B) && !gamepad2_bits.b) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE, true);
//    else if (!(nespad_state2 & DPAD_B) && gamepad2_bits.b) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE, false);
    gamepad2_bits.b = (nespad_state2 & DPAD_B) != 0;
    
//    if ((nespad_state2 & DPAD_SELECT) && !gamepad2_bits.select) joyPushData(fabgl::VirtualKey::VK_KP_PERIOD, true);
//    else if (!(nespad_state2 & DPAD_SELECT) && gamepad2_bits.select) joyPushData(fabgl::VirtualKey::VK_KP_PERIOD, false);
    gamepad2_bits.select = (nespad_state2 & DPAD_SELECT) != 0;

//    if ((nespad_state2 & DPAD_START) && !gamepad2_bits.start) joyPushData(fabgl::VirtualKey::VK_KP_ENTER, true);
//    else if (!(nespad_state2 & DPAD_START) && gamepad2_bits.start) joyPushData(fabgl::VirtualKey::VK_KP_ENTER, false);
    gamepad2_bits.start = (nespad_state2 & DPAD_START) != 0;

//    if ((nespad_state2 & DPAD_UP) && !gamepad2_bits.up) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_UP, true);
//    else if (!(nespad_state2 & DPAD_UP) && gamepad2_bits.up) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_UP, false);
    gamepad2_bits.up = (nespad_state2 & DPAD_UP) != 0;

//    if ((nespad_state2 & DPAD_DOWN) && !gamepad2_bits.down) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_DOWN, true);
//    else if (!(nespad_state2 & DPAD_DOWN) && gamepad2_bits.down) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_DOWN, false);
    gamepad2_bits.down = (nespad_state2 & DPAD_DOWN) != 0;

//    if ((nespad_state2 & DPAD_LEFT) && !gamepad2_bits.left) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_LEFT, true);
//    else if (!(nespad_state2 & DPAD_LEFT) && gamepad2_bits.left) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_LEFT, false);
    gamepad2_bits.left = (nespad_state2 & DPAD_LEFT) != 0;

//    if ((nespad_state2 & DPAD_RIGHT) && !gamepad2_bits.right) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_RIGHT, true);
//    else if (!(nespad_state2 & DPAD_RIGHT) && gamepad2_bits.right) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_RIGHT, false);
    gamepad2_bits.right = (nespad_state2 & DPAD_RIGHT) != 0;
}
#endif

static void nespad_tick1(void);
static void nespad_tick2(void);

void __scratch_x("render") render_core() {
    multicore_lockout_victim_init();
    graphics_init();

    graphics_set_buffer(NULL, DISP_WIDTH, DISP_HEIGHT); /// TODO:
    graphics_set_bgcolor(0x000000);
    graphics_set_flashmode(false, false);
    sem_acquire_blocking(&vga_start_semaphore);

    uint32_t tickKbdRep1 = time_us_32();
    // 60 FPS loop
#define frame_tick (16666)
    uint64_t tick = time_us_64();
    bool tick1 = true;
#ifdef TFT
    uint64_t last_renderer_tick = tick;
#endif
    uint64_t last_input_tick = tick;
    while (true) {
        pcm_call();
#ifdef TFT
        if (tick >= last_renderer_tick + frame_tick) {
            refresh_lcd();
            last_renderer_tick = tick;
        }
#endif
        if (tick >= last_input_tick + frame_tick) {
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

        //tuh_task();
        //hid_app_task();
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
    vreg_set_voltage(VREG_VOLTAGE_1_40);
#else
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
#endif
    sleep_ms(10);
    set_sys_clock_khz(CPU_MHZ * KHZ, true);

    keyboard_init();

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
    pcm_setup(SOUND_FREQUENCY, SOUND_FREQUENCY * 2 / 50); // 882 * 2  = 1764
#ifdef PSRAM
    init_psram();
#endif
    // send kbd reset only after initial process passed
    keyboard_send(0xFF);

    mem_desc_t::reset();
    ESPectrum::setup();
    ESPectrum::loop();
    __unreachable();
}
