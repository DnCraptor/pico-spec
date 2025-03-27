#include <hardware/pwm.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "audio.h"
#include "pwm_audio.h"
#include "Config.h"
#include "LoadWavStream.h"

#define VOLUME_0DB          (16)

static volatile uint8_t vol = VOLUME_0DB;

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
#ifdef I2S_SOUND
static i2s_config_t i2s_config = {
		.sample_freq = SOUND_FREQUENCY, 
		.channel_count = 2,
#ifdef MURM2
		.data_pin = BEEPER_PIN,
		.clock_pin_base = PWM_PIN0,
#else
        .data_pin = PWM_PIN0,
        .clock_pin_base = PWM_PIN1,
#endif
		.pio = pio1,
		.sm = 0,
        .dma_channel = 0,
        .dma_trans_count = 0,
        .dma_buf = NULL,
        .volume = 0,
        .program_offset = 0
	};
#else
static void PWM_init_pin(uint8_t pinN, uint16_t max_lvl) {
    pwm_config config = pwm_get_default_config();
    gpio_set_function(pinN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&config, 1.0);
    pwm_config_set_wrap(&config, max_lvl); // MAX PWM value
    pwm_init(pwm_gpio_to_slice_num(pinN), &config, true);
}
#endif

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
#ifdef I2S_SOUND
    i2s_volume(&i2s_config, 0);
#else
    PWM_init_pin(PWM_PIN0, (1 << 8) - 1);
    PWM_init_pin(PWM_PIN1, (1 << 8) - 1);
   /// PWM_init_pin(BEEPER_PIN, (1 << 8) - 1);
#endif
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
 #ifdef I2S_SOUND
    static int16_t v32[2];
    if (m_off < m_size) {
        v32[0] = *(buff_R + m_off);
        v32[1] = *(buff_L + m_off);
        ++m_off;
    }
    i2s_write(&i2s_config, v32, 1);
//    i2s_dma_write(&i2s_config, v32);
#else
    uint16_t outL = 0;
    uint16_t outR = 0;
    if (m_off < m_size) {
        int16_t* b_L = buff_L + m_off;
        int16_t* b_R = buff_R + m_off;
        uint32_t x = ((int32_t)*b_L) + 0x8000;
        outL = x >> 8; // 4
        ++m_off;
        x = ((int32_t)*b_R) + 0x8000;
        outR = x >> 8;///4;
    } else {
        return;
    }
    pwm_set_gpio_level(PWM_PIN0, outR); // Право
    pwm_set_gpio_level(PWM_PIN1, outL); // Лево
#endif
    return;
}

void pcm_cleanup(void) {
    m_let_process_it = false;
    cancel_repeating_timer(&m_timer);
    m_timer.delay_us = 0;
#ifdef I2S_SOUND
    i2s_volume(&i2s_config, 16);
    i2s_deinit(&i2s_config);
#else
    uint16_t o = 0;
    pwm_set_gpio_level(PWM_PIN0, o); // Право
    pwm_set_gpio_level(PWM_PIN1, o); // Лево
///    pwm_set_gpio_level(BEEPER_PIN, o); // Beeper
#endif
}

/// size - bytes
void pcm_setup(int hz, size_t size) {
#ifdef I2S_SOUND
    if (i2s_config.dma_buf) {
        pcm_cleanup();
    }
    i2s_config.sample_freq = hz;
    i2s_config.channel_count = 2;
    i2s_config.dma_trans_count = 1; // TODO: ensure
    i2s_init(&i2s_config);
#else
    if (m_timer.delay_us) {
        pcm_cleanup();
    }
#endif
    m_let_process_it = false;
    //hz; // 44100;	//44000 //44100 //96000 //22050
	// negative timeout means exact delay (rather than delay between callbacks)
	add_repeating_timer_us(-1000000 / hz, timer_callback, NULL, &m_timer);
}
