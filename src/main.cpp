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

/* Renderer loop on Pico's second core */
#define DISP_WIDTH 320
#define DISP_HEIGHT 240

#include "fabutils.h"
void kbdPushData(fabgl::VirtualKey virtualKey, bool down);
void repeat_handler(void);

extern "C" bool handleScancode(const uint32_t ps2scancode) {
    if ( ((ps2scancode >> 8) & 0xFF) == 0xE0) { // E0 block
        uint8_t cd = ps2scancode & 0xFF;
        bool pressed = cd < 0x80;
        cd &= 0x7F;
        switch (cd) {
            case 0x5B: kbdPushData(fabgl::VirtualKey::VK_LCTRL, pressed); return true; /// L WIN
            case 0x1D: kbdPushData(fabgl::VirtualKey::VK_RCTRL, pressed); return true;
            case 0x38: kbdPushData(fabgl::VirtualKey::VK_RALT, pressed); return true;
            case 0x5C: kbdPushData(fabgl::VirtualKey::VK_RCTRL, pressed); return true; /// R WIN
            case 0x5D: kbdPushData(fabgl::VirtualKey::VK_F1, pressed); return true; /// MENU
            case 0x37: kbdPushData(fabgl::VirtualKey::VK_PRINTSCREEN, pressed); return true;
            case 0x46: kbdPushData(fabgl::VirtualKey::VK_BREAK, pressed); return true;
            case 0x52: kbdPushData(fabgl::VirtualKey::VK_INSERT, pressed); return true;
            case 0x47: kbdPushData(fabgl::VirtualKey::VK_HOME, pressed); return true;
            case 0x4F: kbdPushData(fabgl::VirtualKey::VK_END, pressed); return true;
            case 0x49: kbdPushData(fabgl::VirtualKey::VK_PAGEUP, pressed); return true;
            case 0x51: kbdPushData(fabgl::VirtualKey::VK_PAGEDOWN, pressed); return true;
            case 0x53: kbdPushData(fabgl::VirtualKey::VK_DELETE, pressed); return true;
            case 0x48: kbdPushData(fabgl::VirtualKey::VK_UP, pressed); return true;
            case 0x4B: kbdPushData(fabgl::VirtualKey::VK_LEFT, pressed); return true;
            case 0x50: kbdPushData(fabgl::VirtualKey::VK_DOWN, pressed); return true;
            case 0x4D: kbdPushData(fabgl::VirtualKey::VK_RIGHT, pressed); return true;
            case 0x35: kbdPushData(fabgl::VirtualKey::VK_SLASH, pressed); return true;
            case 0x1C: kbdPushData(fabgl::VirtualKey::VK_KP_ENTER, pressed); return true;
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

        case 0x0E: kbdPushData(fabgl::VirtualKey::VK_BACKSPACE, pressed); return true;
        case 0x39: kbdPushData(fabgl::VirtualKey::VK_SPACE, pressed); return true;
        case 0x0F: kbdPushData(fabgl::VirtualKey::VK_TAB, pressed); return true;
        case 0x3A: kbdPushData(fabgl::VirtualKey::VK_CAPSLOCK, pressed); return true; /// TODO: CapsLock
        case 0x2A: kbdPushData(fabgl::VirtualKey::VK_LSHIFT, pressed); return true;
        case 0x1D: kbdPushData(fabgl::VirtualKey::VK_LCTRL, pressed); return true;
        case 0x38: kbdPushData(fabgl::VirtualKey::VK_LALT, pressed); return true;
        case 0x36: kbdPushData(fabgl::VirtualKey::VK_RSHIFT, pressed); return true;
        case 0x1C: kbdPushData(fabgl::VirtualKey::VK_RETURN, pressed); return true;

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
        case 0x45: kbdPushData(fabgl::VirtualKey::VK_NUMLOCK, pressed); return true; ///
        case 0x37: kbdPushData(fabgl::VirtualKey::VK_KP_MULTIPLY, pressed); return true;
        case 0x4A: kbdPushData(fabgl::VirtualKey::VK_MINUS, pressed); return true;
        case 0x4E: kbdPushData(fabgl::VirtualKey::VK_PLUS, pressed); return true;
        case 0x53: kbdPushData(fabgl::VirtualKey::VK_KP_PERIOD, pressed); return true;
        case 0x52: kbdPushData(fabgl::VirtualKey::VK_KP_0, pressed); return true;
        case 0x4F: kbdPushData(fabgl::VirtualKey::VK_KP_1, pressed); return true;
        case 0x50: kbdPushData(fabgl::VirtualKey::VK_KP_2, pressed); return true;
        case 0x51: kbdPushData(fabgl::VirtualKey::VK_KP_3, pressed); return true;
        case 0x4B: kbdPushData(fabgl::VirtualKey::VK_KP_4, pressed); return true;
        case 0x4C: kbdPushData(fabgl::VirtualKey::VK_KP_5, pressed); return true;
        case 0x4D: kbdPushData(fabgl::VirtualKey::VK_KP_6, pressed); return true;
        case 0x47: kbdPushData(fabgl::VirtualKey::VK_KP_7, pressed); return true;
        case 0x48: kbdPushData(fabgl::VirtualKey::VK_KP_8, pressed); return true;
        case 0x49: kbdPushData(fabgl::VirtualKey::VK_KP_9, pressed); return true;
    }
    return true;
}


#if USE_NESPAD

void joyPushData(fabgl::VirtualKey virtualKey, bool down);

void nespad_tick() {
    nespad_read();

    if ((nespad_state & DPAD_A) && !gamepad1_bits.a) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_FIRE, true);
    else if (!(nespad_state & DPAD_A) && gamepad1_bits.a) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_FIRE, false);
    gamepad1_bits.a = (nespad_state & DPAD_A) != 0;

    if ((nespad_state & DPAD_B) && !gamepad1_bits.b) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE, true);
    else if (!(nespad_state & DPAD_B) && gamepad1_bits.b) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE, false);
    gamepad1_bits.b = (nespad_state & DPAD_B) != 0;
    
    if ((nespad_state & DPAD_SELECT) && !gamepad1_bits.select) joyPushData(fabgl::VirtualKey::VK_SPACE, true);
    else if (!(nespad_state & DPAD_SELECT) && gamepad1_bits.select) joyPushData(fabgl::VirtualKey::VK_SPACE, false);
    gamepad1_bits.select = (nespad_state & DPAD_SELECT) != 0;

    if ((nespad_state & DPAD_START) && !gamepad1_bits.start) joyPushData(fabgl::VirtualKey::VK_RETURN, true);
    else if (!(nespad_state & DPAD_START) && gamepad1_bits.start) joyPushData(fabgl::VirtualKey::VK_RETURN, false);
    gamepad1_bits.start = (nespad_state & DPAD_START) != 0;

    if ((nespad_state & DPAD_UP) && !gamepad1_bits.up) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_UP, true);
    else if (!(nespad_state & DPAD_UP) && gamepad1_bits.up) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_UP, false);
    gamepad1_bits.up = (nespad_state & DPAD_UP) != 0;

    if ((nespad_state & DPAD_DOWN) && !gamepad1_bits.down) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_DOWN, true);
    else if (!(nespad_state & DPAD_DOWN) && gamepad1_bits.down) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_DOWN, false);
    gamepad1_bits.down = (nespad_state & DPAD_DOWN) != 0;

    if ((nespad_state & DPAD_LEFT) && !gamepad1_bits.left) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_LEFT, true);
    else if (!(nespad_state & DPAD_LEFT) && gamepad1_bits.left) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_LEFT, false);
    gamepad1_bits.left = (nespad_state & DPAD_LEFT) != 0;

    if ((nespad_state & DPAD_RIGHT) && !gamepad1_bits.right) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_RIGHT, true);
    else if (!(nespad_state & DPAD_RIGHT) && gamepad1_bits.right) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_RIGHT, false);
    gamepad1_bits.right = (nespad_state & DPAD_RIGHT) != 0;

    // second
    if ((nespad_state2 & DPAD_A) && !gamepad2_bits.a) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_FIRE, true);
    else if (!(nespad_state2 & DPAD_A) && gamepad2_bits.a) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_FIRE, false);
    gamepad2_bits.a = (nespad_state2 & DPAD_A) != 0;

    if ((nespad_state2 & DPAD_B) && !gamepad2_bits.b) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE, true);
    else if (!(nespad_state2 & DPAD_B) && gamepad2_bits.b) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_ALTFIRE, false);
    gamepad2_bits.b = (nespad_state2 & DPAD_B) != 0;
    
    if ((nespad_state2 & DPAD_SELECT) && !gamepad2_bits.select) joyPushData(fabgl::VirtualKey::VK_KP_PERIOD, true);
    else if (!(nespad_state2 & DPAD_SELECT) && gamepad2_bits.select) joyPushData(fabgl::VirtualKey::VK_KP_PERIOD, false);
    gamepad2_bits.select = (nespad_state2 & DPAD_SELECT) != 0;

    if ((nespad_state2 & DPAD_START) && !gamepad2_bits.start) joyPushData(fabgl::VirtualKey::VK_KP_ENTER, true);
    else if (!(nespad_state2 & DPAD_START) && gamepad2_bits.start) joyPushData(fabgl::VirtualKey::VK_KP_ENTER, false);
    gamepad2_bits.start = (nespad_state2 & DPAD_START) != 0;

    if ((nespad_state2 & DPAD_UP) && !gamepad2_bits.up) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_UP, true);
    else if (!(nespad_state2 & DPAD_UP) && gamepad2_bits.up) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_UP, false);
    gamepad2_bits.up = (nespad_state2 & DPAD_UP) != 0;

    if ((nespad_state2 & DPAD_DOWN) && !gamepad2_bits.down) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_DOWN, true);
    else if (!(nespad_state2 & DPAD_DOWN) && gamepad2_bits.down) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_DOWN, false);
    gamepad2_bits.down = (nespad_state2 & DPAD_DOWN) != 0;

    if ((nespad_state2 & DPAD_LEFT) && !gamepad2_bits.left) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_LEFT, true);
    else if (!(nespad_state2 & DPAD_LEFT) && gamepad2_bits.left) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_LEFT, false);
    gamepad2_bits.left = (nespad_state2 & DPAD_LEFT) != 0;

    if ((nespad_state2 & DPAD_RIGHT) && !gamepad2_bits.right) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_RIGHT, true);
    else if (!(nespad_state2 & DPAD_RIGHT) && gamepad2_bits.right) joyPushData(fabgl::VirtualKey::VK_KEMPSTON_RIGHT, false);
    gamepad2_bits.right = (nespad_state2 & DPAD_RIGHT) != 0;
}
#endif

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
        if (tick >= last_input_tick + frame_tick * 1) {
///            kbd_state_t* ks = get_kbd_state();
            nespad_tick();
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
    pcm_setup(ESPectrum::Audio_freq, ESP_AUDIO_SAMPLES_PENTAGON << 1);

    init_psram();

///    kbd_state_t* ks = process_input_on_boot();
    // send kbd reset only after initial process passed
    keyboard_send(0xFF);

    mem_desc_t::reset();
    ESPectrum::setup();
    ESPectrum::loop();
    __unreachable();
}
