#pragma once
#include <stdint.h>

void hdmi_audio_init(void);

void hdmi_audio_write_sample(int16_t left, int16_t right);
