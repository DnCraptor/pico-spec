#include <hardware/pwm.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include "audio.h"
#include "pwm_audio.h"
#include "Config.h"
#include "CPU.h"
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

# if WAV_FILE
bool writeWavHeader(FIL* fo, uint32_t sample_rate, uint16_t num_channels);
bool updateWavHeader(FIL* fo, uint32_t num_samples, uint16_t num_channels);
static FIL fo = { 0 };
static volatile uint32_t num_samples = 0;
#endif

esp_err_t pwm_audio_write(const uint8_t* lbuf, const uint8_t* rbuf, size_t len) {
    static int16_t buff1[640*2];
    static int16_t buff2[640*2];
    static bool is_buff1 = true;
    int16_t* buff = is_buff1 ? buff1 : buff2;
    is_buff1 = !is_buff1;
    int16_t volume = vol;
    if (len < 500) { // W/A
        for (size_t i = 0; i < 640; ++i) {
            size_t j = i << 1;
            size_t k = i * len / 640;
            buff[j  ] = (lbuf[k] << 7) * volume / VOLUME_0DB;
            buff[j+1] = (rbuf[k] << 7) * volume / VOLUME_0DB;
        }
        pcm_set_buffer(buff, 2, 640, NULL);
        return ESP_OK;
    }

# if WAV_FILE
    uint8_t* b8 = (uint8_t*)(is_buff1 ? buff1 : buff2);
    for (size_t i = 0; i < len; ++i) {
        size_t j = i << 1;
        b8[j  ] = lbuf[i];
        b8[j+1] = rbuf[i];
    }
    if (!fo.obj.fs) {
        f_open(&fo, "/1.wav", FA_WRITE | FA_CREATE_ALWAYS);
        writeWavHeader(&fo, len * 50, 2);
    }
    UINT written;
    f_write(&fo, buff, len * 2, &written);
    num_samples += len;
    return ESP_OK;
# endif

# if 1
    for (size_t i = 0; i < len; ++i) {
        size_t j = i << 1;
        buff[j  ] = (lbuf[i] << 7) * volume / VOLUME_0DB;
        buff[j+1] = (rbuf[i] << 7) * volume / VOLUME_0DB;
    }
# else
    static int lv, rv, n = 0;
    static int div = 4;
    for (size_t j = 0; j < len*2; j += 2) {
        buff[j  ] = lv * volume / VOLUME_0DB;
        buff[j+1] = rv * volume / VOLUME_0DB;
        if ((n++ % div) == 0) {
            if (lv == 0) {
                lv = (1 << 8) - 1;
                rv = lv;
            } else {
                rv = lv = 0;
            }
        }
    }
# endif
    pcm_set_buffer(buff, 2, len, NULL);
    return ESP_OK;
}

//------------------------------------------------------------
#ifdef I2S_SOUND
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
    pwm_config config = pwm_get_default_config();
    gpio_set_function(PWM_PIN0, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, (1 << 12) - 1); // MAX PWM value
    pwm_init(pwm_gpio_to_slice_num(PWM_PIN0), &config, true);
    pwm_init(pwm_gpio_to_slice_num(PWM_PIN1), &config, true);
    #if BEEPER_PIN
        gpio_set_function(BEEPER_PIN, GPIO_FUNC_PWM);
        pwm_config_set_clkdiv(&config, 127);
        pwm_init(pwm_gpio_to_slice_num(BEEPER_PIN), &config, true);
    #endif
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
static volatile size_t m_size = 0; // 16-bit values prepared (available)
static volatile bool m_let_process_it = false;
#endif

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

static volatile uint32_t current_buffer_start_us = 0;
static volatile uint32_t buffer_us = SOUND_FREQUENCY / 50; // длина буфера в микросекундах

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
#ifndef I2S_SOUND
    m_let_process_it = false;
    uint32_t ct = time_us_32();
    uint32_t dtf = ct - current_buffer_start_us;
    if (dtf > SOUND_FREQUENCY) return;
    size_t m_off = (!buffer_us ? 0 : dtf * m_size / buffer_us) & 0xFFFFFFFFFE; /// (us per start) * (samples per us) -> sample# since start => m_off
    if (m_buff && m_off < m_size) {
        volatile int16_t* b = m_buff + m_off;
        int16_t outL = *b++; // TODO: ensure L/R seq.
        int16_t outR = *b;
        pwm_set_gpio_level(PWM_PIN0, (uint16_t) ((int32_t) outR + 0x8000L) >> 4); // Право
        pwm_set_gpio_level(PWM_PIN1, (uint16_t) ((int32_t) outL + 0x8000L) >> 4); // Лево
        pwm_set_gpio_level(BEEPER_PIN, 0);
    }
#endif
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
    #if BEEPER_PIN
        pwm_set_gpio_level(BEEPER_PIN, o); // Beeper
    #endif
    m_let_process_it = false;
#endif
}

void close_all(void) {
# if WAV_FILE
    updateWavHeader(&fo, num_samples, 2);
    f_close(&fo);
# endif
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
	add_repeating_timer_us(-1000000ll / hz, timer_callback, NULL, &m_timer);
#endif
}

static uint32_t prev_buffer_start_us = 0;

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

    prev_buffer_start_us = current_buffer_start_us;
    current_buffer_start_us = time_us_32();
    buffer_us = current_buffer_start_us - prev_buffer_start_us;
#endif
}
