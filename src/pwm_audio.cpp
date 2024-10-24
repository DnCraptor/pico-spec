#include <hardware/pwm.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "audio.h"
#include "pwm_audio.h"
#include "Config.h"
#include "CPU.h"

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

static int16_t buff[640 << 1];

esp_err_t pwm_audio_write(const uint8_t* lbuf, const uint8_t* rbuf, size_t len) {
    int16_t volume = vol;
    size_t j = 0;
    for (size_t i = 0; i < len; ++i) {
        buff[j++] = (((int16_t)lbuf[i]) << 7) * volume / VOLUME_0DB;
        buff[j++] = (((int16_t)rbuf[i]) << 7) * volume / VOLUME_0DB;
    }
    pcm_set_buffer(buff, 2, len, NULL);
    return ESP_OK;
}

//------------------------------------------------------------
#ifdef I2S_SOUND
#define SOUND_FREQUENCY 44100
static i2s_config_t i2s_config = {
		.sample_freq = SOUND_FREQUENCY, 
		.channel_count = 2,
		.data_pin = PWM_PIN0,
		.clock_pin_base = PWM_PIN1,
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
#endif

void init_sound() {
#ifdef I2S_SOUND
    i2s_config.sample_freq = SOUND_FREQUENCY;
    i2s_config.dma_trans_count = SOUND_FREQUENCY / 100;
    i2s_volume(&i2s_config, 0);
#else
    PWM_init_pin(PWM_PIN0, (1 << 8) - 1);
    PWM_init_pin(PWM_PIN1, (1 << 8) - 1);
    PWM_init_pin(BEEPER_PIN, (1 << 8) - 1);
#endif
#ifdef LOAD_WAV_PIO
    //пин ввода звука
    inInit(LOAD_WAV_PIO);
#endif
}

static repeating_timer_t m_timer = { 0 };
static volatile pcm_end_callback_t m_cb = NULL;
#ifndef I2S_SOUND
static volatile int16_t* m_buff = NULL;
static volatile uint8_t m_channels = 0;
static volatile size_t m_off = 0; // in 16-bit words
static volatile size_t m_size = 0; // 16-bit values prepared (available)
static volatile bool m_let_process_it = false;
#endif
///static uint64_t ibuff_state_started = 0;
///static bool ibuff[640];
///static volatile uint16_t ibuff_off;
///static uint32_t statesInFrame = 100000;
///static uint32_t samplesPerFrame = 640;
///static uint32_t statesPerSample = 100000 / 640;

/**
bool pcm_data_in(void) {
    if (CPU::statesInFrame != statesInFrame || ESPectrum::samplesPerFrame != samplesPerFrame) {
        CPU::statesInFrame = statesInFrame;
        ESPectrum::samplesPerFrame = samplesPerFrame;
        statesPerSample = CPU::statesInFrame / ESPectrum::samplesPerFrame;
    }
    uint64_t statesPassedFromFrameStarted = CPU::global_tstates + CPU::tstates - ibuff_state_started;
    uint32_t obuff_off = statesPassedFromFrameStarted / statesPerSample;
    if (obuff_off >= sizeof(ibuff)) return false;
    return ibuff[obuff_off];
}
*/

static bool __not_in_flash_func(timer_callback)(repeating_timer_t *rt) { // core#1?
#ifndef I2S_SOUND
    m_let_process_it = true;
#endif
#if LOAD_WAV_PIO
///    if (Config::real_player && ibuff_off < sizeof(ibuff)) {
///        ibuff[ibuff_off++] = gpio_get(LOAD_WAV_PIO);
///    }
#endif
    return true;
}

void pcm_call() {
#ifndef I2S_SOUND
    if (!m_let_process_it) {
        return;
    }
    m_let_process_it = false;
    uint16_t outL = 0;
    uint16_t outR = 0;
    if (m_channels && m_buff && m_off < m_size) {
        volatile int16_t* b = m_buff + m_off;
        uint32_t x = ((int32_t)*b) + 0x8000;
        outL = x >> 8; // 4
        ++m_off;
        if (m_channels == 2) {
            ++b;
            x = ((int32_t)*b) + 0x8000;
            outR = x >> 8;///4;
            ++m_off;
        } else {
            outR = outL;
        }
    } else {
        return;
    }
    pwm_set_gpio_level(PWM_PIN0, outR); // Право
    pwm_set_gpio_level(PWM_PIN1, outL); // Лево
    if (m_channels == 1) {
        pwm_set_gpio_level(BEEPER_PIN, outL); // Beeper
    }
    if (m_cb && m_off >= m_size) {
        m_buff = m_cb(&m_size);
        m_off = 0;
    }
#endif
    return;
}

void pcm_cleanup(void) {
    cancel_repeating_timer(&m_timer);
    m_timer.delay_us = 0;
#ifdef I2S_SOUND
    i2s_volume(&i2s_config, 0);
    i2s_deinit(&i2s_config);
#else
    uint16_t o = 0;
    pwm_set_gpio_level(PWM_PIN0, o); // Право
    pwm_set_gpio_level(PWM_PIN1, o); // Лево
    pwm_set_gpio_level(BEEPER_PIN, o); // Beeper
    m_let_process_it = false;
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
    i2s_config.dma_trans_count = size >> 2;
    i2s_init(&i2s_config);
#else
    if (m_timer.delay_us) {
        pcm_cleanup();
    }
    m_let_process_it = false;
    //hz; // 44100;	//44000 //44100 //96000 //22050
	// negative timeout means exact delay (rather than delay between callbacks)
	add_repeating_timer_us(-1000000 / hz, timer_callback, NULL, &m_timer);
#endif
}

// reset input buffer to start on each frame started
void pwm_audio_in_frame_started(void) {
///    ibuff_state_started = CPU::global_tstates + CPU::tstates;
///    ibuff_off = 0;
}

// size - in 16-bit values count
void pcm_set_buffer(int16_t* buff, uint8_t channels, size_t size, pcm_end_callback_t cb) {
    m_cb = cb;
#ifdef I2S_SOUND
    i2s_config.channel_count = 2; // let ignore momo for this chip
    i2s_config.dma_trans_count = size; ///i2s_config.sample_freq / (size << 1); // Number of 32 bits words to transfer
    i2s_volume(&i2s_config, vol);
    i2s_dma_write(&i2s_config, buff);
#else
    m_buff = buff;
    m_channels = channels;
    m_size = size * channels;
    m_off = 0;
#endif
}
