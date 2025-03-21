#pragma once

void init_sound();
void pcm_setup(int hz, size_t size);
void pcm_cleanup(void);
typedef volatile int16_t* (*pcm_end_callback_t)(volatile size_t* size);
void pcm_set_buffer(uint8_t* buff, uint8_t channels, size_t size, pcm_end_callback_t cb);
// internal call on core#1
void pcm_call();
bool pcm_data_in(void);
void pwm_audio_in_frame_started(void);

#define esp_err_t int
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 1
#define ESP_FAIL -1

/**
 * @brief Set volume for pwm audio
 *
 * @attention when the volume is too small, it will produce serious distortion
 *
 * @param volume Volume to set (-16 ~ 16).
 *        Set to 0 for original output;
 *        Set to less then 0 for attenuation, and -16 is mute;
 *        Set to more than 0 for enlarge, and 16 is double output
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t pwm_audio_set_volume(int8_t volume);

#if LOAD_WAV_PIO
void pcm_audio_in_stop(void);
#endif

void pwm_audio_write(const uint8_t* lbuf, const uint8_t* rbuf, size_t len);
void pwm_audio_sync(const uint8_t* lbuf, const uint8_t* rbuf, size_t len);
void pwm_audio_lock(bool b);
