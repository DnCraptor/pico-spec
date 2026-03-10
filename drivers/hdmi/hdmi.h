#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "inttypes.h"
#include "stdbool.h"

#include "hardware/pio.h"

#define PIO_VIDEO pio0
#define PIO_VIDEO_ADDR pio0
#define VIDEO_DMA_IRQ (DMA_IRQ_0)

#ifndef HDMI_BASE_PIN
#define HDMI_BASE_PIN (6)
#endif

#if defined(ZERO) || defined(ZERO2)
	#define HDMI_PIN_RGB_notBGR (0)
	#define HDMI_PIN_invert_diffpairs (0)
	#define beginHDMI_PIN_data (HDMI_BASE_PIN)
	#define beginHDMI_PIN_clk (HDMI_BASE_PIN + 6)
#else
	#define HDMI_PIN_RGB_notBGR (1)
	#define HDMI_PIN_invert_diffpairs (1)
	#define beginHDMI_PIN_clk (HDMI_BASE_PIN)
	#define beginHDMI_PIN_data (HDMI_BASE_PIN+2)
#endif

#define TEXTMODE_COLS 53
#define TEXTMODE_ROWS 30

// HDMI audio support (RP2350 only)
#if !PICO_RP2040
void hdmi_audio_init(void);
void hdmi_audio_write_sample(int16_t left, int16_t right);
#endif

// Hot video mode reinit (reuses existing PIO/DMA resources)
void hdmi_reinit(void);
// Call from core1 loop to process pending reinit
void hdmi_poll_reinit(void);

// TODO: Сделать настраиваемо
static const uint8_t textmode_palette[16] = {
    200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215
};

#ifdef __cplusplus
}
#endif
