#include <hardware/pwm.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "audio.h"
#include "pwm_audio.h"
#include "Config.h"
#include "LoadWavStream.h"
#include "PinSerialData_595.h"

// connection is possible 00->00 (external pull down)
static int test_0000_case(uint32_t pin0, uint32_t pin1, int res) {
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin0, 1);

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_down(pin1); /// external pulled down (so, just to ensure)
    sleep_ms(33);
    if ( gpio_get(pin1) ) { // 1 -> 1, looks really connected
        res |= (1 << 5) | 1;
    }
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    return res;
}

// connection is possible 01->01 (no external pull up/down)
static int test_0101_case(uint32_t pin0, uint32_t pin1, int res) {
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin0, 1);

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_down(pin1);
    sleep_ms(33);
    if ( gpio_get(pin1) ) { // 1 -> 1, looks really connected
        res |= (1 << 5) | 1;
    }
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    return res;
}

// connection is possible 11->11 (externally pulled up)
static int test_1111_case(uint32_t pin0, uint32_t pin1, int res) {
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_OUT);
    sleep_ms(33);
    gpio_put(pin0, 0);

    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1); /// external pulled up (so, just to ensure)
    sleep_ms(33);
    if ( !gpio_get(pin1) ) { // 0 -> 0, looks really connected
        res |= 1;
    }
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    return res;
}

extern "C" int testPins(uint32_t pin0, uint32_t pin1) {
    int res = 0b000000;
    /// do not try to test butter psram this way
#ifdef BUTTER_PSRAM_GPIO
    if (pin0 == BUTTER_PSRAM_GPIO || pin1 == BUTTER_PSRAM_GPIO) return res;
#endif
    #ifdef PICO_DEFAULT_LED_PIN
    if (pin0 == PICO_DEFAULT_LED_PIN || pin1 == PICO_DEFAULT_LED_PIN) return res; // LED
    #endif
    if (pin0 == 23 || pin1 == 23) return res; // SMPS Power
    if (pin0 == 24 || pin1 == 24) return res; // VBus sense
    // try pull down case (passive)
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_IN);
    gpio_pull_down(pin0);
    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_down(pin1);
    sleep_ms(33);
    int pin0vPD = gpio_get(pin0);
    int pin1vPD = gpio_get(pin1);
    gpio_deinit(pin0);
    gpio_deinit(pin1);
    /// try pull up case (passive)
    gpio_init(pin0);
    gpio_set_dir(pin0, GPIO_IN);
    gpio_pull_up(pin0);
    gpio_init(pin1);
    gpio_set_dir(pin1, GPIO_IN);
    gpio_pull_up(pin1);
    sleep_ms(33);
    int pin0vPU = gpio_get(pin0);
    int pin1vPU = gpio_get(pin1);
    gpio_deinit(pin0);
    gpio_deinit(pin1);

    res = (pin0vPD << 4) | (pin0vPU << 3) | (pin1vPD << 2) | (pin1vPU << 1);

    if (pin0vPD == 1) {
        if (pin0vPU == 1) { // pin0vPD == 1 && pin0vPU == 1
            if (pin1vPD == 1) { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 1
                    // connection is possible 11->11 (externally pulled up)
                    return test_1111_case(pin0, pin1, res);
                } else { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            } else { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 1 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            }
        } else {  // pin0vPD == 1 && pin0vPU == 0
            if (pin1vPD == 1) { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 0
                    // connection is possible 10->10 (pulled up on down, and pulled down on up?)
                    return res |= (1 << 5) | 1; /// NOT SURE IT IS POSSIBLE TO TEST SUCH CASE (TODO: think about real cases)
                }
            } else { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 1 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            }
        }
    } else { // pin0vPD == 0
        if (pin0vPU == 1) { // pin0vPD == 0 && pin0vPU == 1
            if (pin1vPD == 1) { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 1 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            } else { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 1
                    // connection is possible 01->01 (no external pull up/down)
                    return test_0101_case(pin0, pin1, res);
                } else { // pin0vPD == 0 && pin0vPU == 1 && pin1vPD == 0 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            }
        } else {  // pin0vPD == 0 && pin0vPU == 0
            if (pin1vPD == 1) { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 1
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 1 && pin1vPU == 0
                    // connection is impossible
                    return res;
                }
            } else { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 0
                if (pin1vPU == 1) { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 1
                    // connection is impossible
                    return res;
                } else { // pin0vPD == 0 && pin0vPU == 0 && pin1vPD == 0 && pin1vPU == 0
                    // connection is possible 00->00 (externally pulled down)
                    return test_0000_case(pin0, pin1, res);
                }
            }
        }
    }
    return res;
}

#define VOLUME_0DB          (16)

static volatile uint8_t vol = VOLUME_0DB;
uint8_t link_i2s_code = 0xFF;
bool is_i2s_enabled = false;

esp_err_t pwm_audio_set_volume(int8_t volume) {
    if (volume < -VOLUME_0DB) {
        vol = 0;
        return ESP_OK;
    }
    if (volume > VOLUME_0DB) {
        volume = VOLUME_0DB;
    }
    vol = volume + VOLUME_0DB;
    return ESP_OK;
}

// return next buffer
volatile int16_t* pcm_end_callback(volatile size_t* size) {
    *size = 0;
    return NULL;
}

static int16_t buff_L[640] = { 0 };
static int16_t buff_R[640] = { 0 };
static repeating_timer_t m_timer = { 0 };
static volatile size_t m_off = 0; // in 16-bit words
static volatile size_t m_size = 0; // 16-bit values prepared (available)
static volatile bool m_let_process_it = false;

esp_err_t pwm_audio_write(
    uint8_t *bufL,
    uint8_t *bufR,
    size_t len,
    size_t* bytes_written,
    uint32_t wait_ms
) {
    int16_t volume = vol;
    for (size_t i = 0; i < len; ++i) {
        buff_L[i] = (((int16_t)bufL[i]) << 7) * volume / VOLUME_0DB;
        buff_R[i] = (((int16_t)bufR[i]) << 7) * volume / VOLUME_0DB;
    }
    m_off = 0;
    m_size = len;
    if (bytes_written) *bytes_written = len;
    return ESP_OK;
}

//------------------------------------------------------------
static i2s_config_t i2s_config = {
		.sample_freq = 31250, 
		.channel_count = 2,
        .data_pin = I2S_DATA_PIO,
        .bck_pin = I2S_BCK_PIO,
        .lck_pin = I2S_LCK_PIO,
		.pio = pio1,
		.sm = 0,
        .dma_channel = 0,
        .dma_trans_count = 0,
        .dma_buf = NULL,
        .volume = 0,
        .program_offset = 0
	};

static void PWM_init_pin(uint8_t pinN, uint16_t max_lvl) {
    pwm_config config = pwm_get_default_config();
    gpio_set_function(pinN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&config, 1.0);
    pwm_config_set_wrap(&config, max_lvl); // MAX PWM value
    pwm_init(pwm_gpio_to_slice_num(pinN), &config, true);
}

#ifdef LOAD_WAV_PIO
inline static void inInit(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}
static bool hw_get_bit_LOAD() {
    uint8_t out = 0;
    out = gpio_get(LOAD_WAV_PIO);
    // valLoad=out*10;
    return out > 0;
};
#endif

void init_sound() {
    if (Config::audio_driver == 3) {
        Init_PWM_175(TSPIN_MODE_GP29);
    } else {
        if (link_i2s_code == 0xFF) {
            if (I2S_BCK_PIO != I2S_LCK_PIO && I2S_LCK_PIO != I2S_DATA_PIO && I2S_BCK_PIO != I2S_DATA_PIO) {
                link_i2s_code = testPins(I2S_DATA_PIO, I2S_BCK_PIO);
                is_i2s_enabled = link_i2s_code; // TODO: ensure
            }
        }
        if (Config::audio_driver != 0) {
            is_i2s_enabled = (Config::audio_driver == 2);
        }
        if (is_i2s_enabled) {
            i2s_volume(&i2s_config, 0);
        } else {
            PWM_init_pin(PWM_PIN0, (1 << 8) - 1);
            PWM_init_pin(PWM_PIN1, (1 << 8) - 1);
            /// PWM_init_pin(BEEPER_PIN, (1 << 8) - 1);
        }
    }
#ifdef LOAD_WAV_PIO
    //пин ввода звука
    inInit(LOAD_WAV_PIO);
#endif
}

#if LOAD_WAV_PIO
static LoadWavStream* lws = 0;

bool pcm_data_in(void) {
    return lws ? lws->get_in_sample() : false;
}
// reset input buffer to start on each frame started
void pwm_audio_in_frame_started(void) {
    if (!lws) lws = new LoadWavStream();
    lws->open_frame();
}
void pcm_audio_in_stop(void) {
    if (lws) delete lws;
    lws = 0;
}
#endif

static bool __not_in_flash_func(timer_callback)(repeating_timer_t *rt) { // core#1?
    m_let_process_it = true;
#if LOAD_WAV_PIO
    if (lws && Config::real_player) {
        lws->tick();
    }
#endif
    return true;
}

void pcm_call() {
    if (!m_let_process_it) {
        return;
    }
    m_let_process_it = false;
    if (Config::audio_driver == 3) {
/// TODO:
    }
    else if (is_i2s_enabled) {
        static int16_t v32[2];
        if (m_off < m_size) {
            v32[0] = *(buff_R + m_off);
            v32[1] = *(buff_L + m_off);
            ++m_off;
        }
        i2s_write(&i2s_config, v32, 1);
        // i2s_dma_write(&i2s_config, v32);
    } else {
        uint16_t outL = 0;
        uint16_t outR = 0;
        if (m_off < m_size) {
            // First-order error diffusion (noise shaping):
            // quantization error from previous sample is fed forward,
            // pushing PWM quantization noise to ultrasonic frequencies
            static int16_t err_L = 0, err_R = 0;
            int16_t* b_L = buff_L + m_off;
            int16_t* b_R = buff_R + m_off;
            ++m_off;
            int32_t xL = ((int32_t)*b_L) + 0x8000 + err_L;
            if (xL < 0) xL = 0; else if (xL > 0xFFFF) xL = 0xFFFF;
            outL = (uint16_t)xL >> 8;
            err_L = (int16_t)(xL - ((int32_t)outL << 8));
            int32_t xR = ((int32_t)*b_R) + 0x8000 + err_R;
            if (xR < 0) xR = 0; else if (xR > 0xFFFF) xR = 0xFFFF;
            outR = (uint16_t)xR >> 8;
            err_R = (int16_t)(xR - ((int32_t)outR << 8));
        } else {
            return;
        }
        pwm_set_gpio_level(PWM_PIN0, outR); // Право
        pwm_set_gpio_level(PWM_PIN1, outL); // Лево
    }
    return;
}

void pcm_cleanup(void) {
    m_let_process_it = false;
    cancel_repeating_timer(&m_timer);
    m_timer.delay_us = 0;
    if (Config::audio_driver == 3) {
        // TODO:
    } else if (is_i2s_enabled) {
        i2s_volume(&i2s_config, 16);
        i2s_deinit(&i2s_config);
    } else {
        uint16_t o = 0;
        pwm_set_gpio_level(PWM_PIN0, o); // Право
        pwm_set_gpio_level(PWM_PIN1, o); // Лево
    ///    pwm_set_gpio_level(BEEPER_PIN, o); // Beeper
    }
}

/// size - bytes
void pcm_setup(int hz) {
    if (Config::audio_driver == 3) {
        // TODO:
    } else if (is_i2s_enabled) {
        if (i2s_config.dma_buf) {
            pcm_cleanup();
        }
        i2s_config.sample_freq = hz;
        i2s_config.channel_count = 2;
        i2s_config.dma_trans_count = 1; // TODO: ensure
        i2s_init(&i2s_config);
    } else {
        if (m_timer.delay_us) {
            pcm_cleanup();
        }
    }
    m_let_process_it = false;
    //hz; // 44100;	//44000 //44100 //96000 //22050
    // negative timeout means exact delay (rather than delay between callbacks)
    add_repeating_timer_us(-1000000 / hz, timer_callback, NULL, &m_timer);
}
