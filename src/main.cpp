#include <cstdio>
#include <cstring>
#include <cstdarg>

#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <hardware/vreg.h>
#include <hardware/watchdog.h>
#include <hardware/sync.h>
#include <hardware/flash.h>

#include "ESPectrum.h"

#include "graphics.h"

#include "audio.h"
#include "ff.h"

#if USE_PS2_KBD

#include "ps2kbd_mrmltr.h"

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

static i2s_config_t i2s_config;

#define SND_FREQ 44100
#define SND_DIV 60
#define SND_CH 2
#define SND_CNT (SND_FREQ / SND_DIV)
#define buffermax (SND_CNT * SND_CH)

static void init_sound() {
    i2s_config = i2s_get_default_config();
    i2s_config.sample_freq = SND_FREQ;
    i2s_config.dma_trans_count = (uint16_t) SND_CNT;
    i2s_volume(&i2s_config, 0);
    i2s_init(&i2s_config);
}

typedef struct __attribute__((__packed__)) {
    uint8_t snd_vol;
} SETTINGS;

static SETTINGS settings = {
    .snd_vol = 8,
};

void sound_output(int samples, const BYTE *wave1, const BYTE *wave2, const BYTE *wave3, const BYTE *wave4,
                         const BYTE *wave5) {
    static int16_t samples_out[2][buffermax * 2];
    static int i_active_buf = 0;
    static int inx = 0;
    static int max = 0;
    static int min = 30000;
    for (int i = 0; i < samples; i++) {
        int r, l;
        int w1 = *wave1++;
        int w2 = *wave2++;
        int w3 = *wave3++;
        int w4 = *wave4++;
        int w5 = *wave5++;
        l = w1 * 6 + w2 * 3 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 32;
        r = w1 * 3 + w2 * 6 + w3 * 5 + w4 * 3 * 17 + w5 * 2 * 32;
        max = MAX(l, max);
        min = MIN(l, min);
        l -= 4000;
        r -= 4000;
        samples_out[i_active_buf][inx * 2] = (int16_t) l * settings.snd_vol;
        samples_out[i_active_buf][inx * 2 + 1] = (int16_t) r * settings.snd_vol;
        if (inx++ >= i2s_config.dma_trans_count) {
            inx = 0;
            i2s_dma_write(&i2s_config, reinterpret_cast<const int16_t *>(samples_out[i_active_buf]));
            i_active_buf ^= 1;
        }
    }
}


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

/* Renderer loop on Pico's second core */
#define DISP_WIDTH 320
#define DISP_HEIGHT 240

#if USE_PS2_KBD

#include "fabutils.h"
void kbdPushData(const fabgl::VirtualKey& virtualKey, bool down);

static fabgl::VirtualKey hid2vk(uint8_t c) {
    switch ( c )
    {
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
    case HID_KEY_1: return fabgl::VirtualKey::VK_1;
    case HID_KEY_2: return fabgl::VirtualKey::VK_2;
    case HID_KEY_3: return fabgl::VirtualKey::VK_3;
    case HID_KEY_4: return fabgl::VirtualKey::VK_4;
    case HID_KEY_5: return fabgl::VirtualKey::VK_5;
    case HID_KEY_6: return fabgl::VirtualKey::VK_6;
    case HID_KEY_7: return fabgl::VirtualKey::VK_7;
    case HID_KEY_8: return fabgl::VirtualKey::VK_8;
    case HID_KEY_9: return fabgl::VirtualKey::VK_9;
    case HID_KEY_0: return fabgl::VirtualKey::VK_0;
    case HID_KEY_ENTER: return fabgl::VirtualKey::VK_RETURN;
    case HID_KEY_ESCAPE: return fabgl::VirtualKey::VK_ESCAPE;
    case HID_KEY_BACKSPACE: return fabgl::VirtualKey::VK_BACKSPACE;
    case HID_KEY_TAB: return fabgl::VirtualKey::VK_TAB;
    case HID_KEY_SPACE: return fabgl::VirtualKey::VK_SPACE;
    case HID_KEY_MINUS: return fabgl::VirtualKey::VK_MINUS;
    case HID_KEY_EQUAL: return fabgl::VirtualKey::VK_EQUALS;
    case HID_KEY_BRACKET_LEFT: return fabgl::VirtualKey::VK_LEFTBRACKET;
    case HID_KEY_BRACKET_RIGHT: return fabgl::VirtualKey::VK_RIGHTBRACKET;
    case HID_KEY_BACKSLASH: return fabgl::VirtualKey::VK_BACKSLASH;
    case HID_KEY_EUROPE_1: return fabgl::VirtualKey::VK_EURO;
    case HID_KEY_SEMICOLON: return fabgl::VirtualKey::VK_SEMICOLON;
    case HID_KEY_APOSTROPHE: return fabgl::VirtualKey::VK_QUOTE;
    case HID_KEY_GRAVE: return fabgl::VirtualKey::VK_GRAVEACCENT; /// TODO:
    case HID_KEY_COMMA: return fabgl::VirtualKey::VK_COMMA;
    case HID_KEY_PERIOD: return fabgl::VirtualKey::VK_PERIOD;
    case HID_KEY_SLASH: return fabgl::VirtualKey::VK_SLASH;
    case HID_KEY_CAPS_LOCK: return fabgl::VirtualKey::VK_CAPSLOCK;
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
    case HID_KEY_PRINT_SCREEN: return fabgl::VirtualKey::VK_PRINTSCREEN;
    case HID_KEY_SCROLL_LOCK: return fabgl::VirtualKey::VK_SCROLLLOCK;
    case HID_KEY_PAUSE: return fabgl::VirtualKey::VK_PAUSE;
    case HID_KEY_INSERT: return fabgl::VirtualKey::VK_INSERT;
    case HID_KEY_HOME: return fabgl::VirtualKey::VK_HOME;
    case HID_KEY_PAGE_UP: return fabgl::VirtualKey::VK_PAGEUP;
    case HID_KEY_DELETE: return fabgl::VirtualKey::VK_DELETE;
    case HID_KEY_END: return fabgl::VirtualKey::VK_END;
    case HID_KEY_PAGE_DOWN: return fabgl::VirtualKey::VK_PAGEDOWN;
    case HID_KEY_ARROW_RIGHT: return fabgl::VirtualKey::VK_RIGHT;
    case HID_KEY_ARROW_LEFT: return fabgl::VirtualKey::VK_LEFT;
    case HID_KEY_ARROW_DOWN: return fabgl::VirtualKey::VK_DOWN;
    case HID_KEY_ARROW_UP: return fabgl::VirtualKey::VK_UP;
    case HID_KEY_NUM_LOCK: return fabgl::VirtualKey::VK_NUMLOCK;
    case HID_KEY_KEYPAD_DIVIDE: return fabgl::VirtualKey::VK_KP_DIVIDE;
    case HID_KEY_KEYPAD_MULTIPLY: return fabgl::VirtualKey::VK_KP_MULTIPLY;
    case HID_KEY_KEYPAD_SUBTRACT: return fabgl::VirtualKey::VK_KP_MINUS;
    case HID_KEY_KEYPAD_ADD: return fabgl::VirtualKey::VK_KP_PLUS;
    case HID_KEY_KEYPAD_ENTER: return fabgl::VirtualKey::VK_KP_ENTER;
    case HID_KEY_KEYPAD_1: return fabgl::VirtualKey::VK_KP_1;
    case HID_KEY_KEYPAD_2: return fabgl::VirtualKey::VK_KP_2;
    case HID_KEY_KEYPAD_3: return fabgl::VirtualKey::VK_KP_3;
    case HID_KEY_KEYPAD_4: return fabgl::VirtualKey::VK_KP_4;
    case HID_KEY_KEYPAD_5: return fabgl::VirtualKey::VK_KP_5;
    case HID_KEY_KEYPAD_6: return fabgl::VirtualKey::VK_KP_6;
    case HID_KEY_KEYPAD_7: return fabgl::VirtualKey::VK_KP_7;
    case HID_KEY_KEYPAD_8: return fabgl::VirtualKey::VK_KP_8;
    case HID_KEY_KEYPAD_9: return fabgl::VirtualKey::VK_KP_9;
    case HID_KEY_KEYPAD_0: return fabgl::VirtualKey::VK_KP_0;
    case HID_KEY_KEYPAD_DECIMAL: return fabgl::VirtualKey::VK_KP_PERIOD;
    case HID_KEY_EUROPE_2: return fabgl::VirtualKey::VK_EURO; /// TODO: ensure
    case HID_KEY_APPLICATION: return fabgl::VirtualKey::VK_APPLICATION;
///    case HID_KEY_POWER: return fabgl::VirtualKey::VK_;
    case HID_KEY_KEYPAD_EQUAL: return fabgl::VirtualKey::VK_KP_ENTER;
///    case HID_KEY_F13: return fabgl::VirtualKey::VK_;
    case HID_KEY_EXECUTE: return fabgl::VirtualKey::VK_RETURN;
//    case HID_KEY_HELP: return fabgl::VirtualKey::VK_;
//    case HID_KEY_MENU: return fabgl::VirtualKey::VK_;
    case HID_KEY_RETURN: return fabgl::VirtualKey::VK_RETURN;
    case HID_KEY_CONTROL_LEFT: return fabgl::VirtualKey::VK_LCTRL;
    case HID_KEY_SHIFT_LEFT: return fabgl::VirtualKey::VK_LSHIFT;
    case HID_KEY_ALT_LEFT: return fabgl::VirtualKey::VK_LALT;
    case HID_KEY_GUI_LEFT: return fabgl::VirtualKey::VK_LGUI;
    case HID_KEY_CONTROL_RIGHT: return fabgl::VirtualKey::VK_RCTRL;
    case HID_KEY_SHIFT_RIGHT: return fabgl::VirtualKey::VK_RSHIFT;
    case HID_KEY_ALT_RIGHT: return fabgl::VirtualKey::VK_RALT;
    case HID_KEY_GUI_RIGHT: return fabgl::VirtualKey::VK_RGUI;
    /// TODO: ensure it is enough
    default:
        return fabgl::VirtualKey::VK_NONE;
    }
}

void __not_in_flash_func(process_kbd_report)(hid_keyboard_report_t const* report,
                                             hid_keyboard_report_t const* prev_report) {

    for (unsigned char cc: report->keycode) {
        bool found = false;
        for (unsigned char pc: prev_report->keycode) {
            if ( pc == cc ) {
                found = true;
                break;
            }
        }
        if ( !found ) { // new one is selected
            kbdPushData(hid2vk(cc), true);
        }
    }
    for (unsigned char pc: prev_report->keycode) {
        bool found = false;
        for (unsigned char cc: report->keycode) {
            if ( pc == cc ) {
                found = true;
                break;
            }
        }
        if ( !found ) { // old not more is selected
            kbdPushData(hid2vk(pc), false);
        }
    }
}


Ps2Kbd_Mrmltr ps2kbd(
    pio1,
    0,
    process_kbd_report
);
#endif

#if USE_NESPAD

void nespad_tick() {
    nespad_read();

    gamepad1_bits.a = (nespad_state & DPAD_A) != 0;
    gamepad1_bits.b = (nespad_state & DPAD_B) != 0;
    gamepad1_bits.select = (nespad_state & DPAD_SELECT) != 0;
    gamepad1_bits.start = (nespad_state & DPAD_START) != 0;
    gamepad1_bits.up = (nespad_state & DPAD_UP) != 0;
    gamepad1_bits.down = (nespad_state & DPAD_DOWN) != 0;
    gamepad1_bits.left = (nespad_state & DPAD_LEFT) != 0;
    gamepad1_bits.right = (nespad_state & DPAD_RIGHT) != 0;

    // second
    gamepad2_bits.a = (nespad_state2 & DPAD_A) != 0;
    gamepad2_bits.b = (nespad_state2 & DPAD_B) != 0;
    gamepad2_bits.select = (nespad_state2 & DPAD_SELECT) != 0;
    gamepad2_bits.start = (nespad_state2 & DPAD_START) != 0;
    gamepad2_bits.up = (nespad_state2 & DPAD_UP) != 0;
    gamepad2_bits.down = (nespad_state2 & DPAD_DOWN) != 0;
    gamepad2_bits.left = (nespad_state2 & DPAD_LEFT) != 0;
    gamepad2_bits.right = (nespad_state2 & DPAD_RIGHT) != 0;
}
#endif

void __scratch_x("render") render_core() {
    multicore_lockout_victim_init();
    graphics_init();

    graphics_set_buffer(NULL, DISP_WIDTH, DISP_HEIGHT); /// TODO:
    graphics_set_bgcolor(0x000000);
///    graphics_set_offset(32, 0);
    graphics_set_flashmode(false, false);
    sem_acquire_blocking(&vga_start_semaphore);

    // 60 FPS loop
#define frame_tick (16666)
    uint64_t tick = time_us_64();
#ifdef TFT
    uint64_t last_renderer_tick = tick;
#endif
    uint64_t last_input_tick = tick;
    while (true) {
#ifdef TFT
        if (tick >= last_renderer_tick + frame_tick) {
            refresh_lcd();
            last_renderer_tick = tick;
        }
#endif
        if (tick >= last_input_tick + frame_tick * 1) {
            ps2kbd.tick();
            nespad_tick();
            last_input_tick = tick;
        }
        tick = time_us_64();


        tuh_task();
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
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);
    set_sys_clock_khz(378 * KHZ, true);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 6; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

#if USE_PS2_KBD
    ps2kbd.init_gpio();
#endif
#if USE_NESPAD
    nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);
#endif
    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(render_core);
    sem_release(&vga_start_semaphore);

    ESPectrum::setup();
    ESPectrum::loop();
    __unreachable();
}
