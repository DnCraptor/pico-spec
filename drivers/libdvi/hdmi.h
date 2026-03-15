#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

void hdmi_audio_init(void);

void hdmi_audio_write_sample(int16_t left, int16_t right);

void dvi_line(uint32_t line);

void graphics_init0(void);

#ifdef __cplusplus
}
#endif
