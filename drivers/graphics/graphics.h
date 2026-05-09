#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"

#ifdef TFT
#include "st7789.h"
#endif
#ifdef VGA_HDMI
#include "vga.h"
#include "hdmi.h"
#endif
#ifdef TV
#include "tv.h"
#endif
#ifdef SOFTTV
#include "tv-software.h"
#endif
#include "font6x8.h"
#include "font8x8.h"
#include "font8x16.h"

typedef struct video_mode_t{
  int v_total;
  int v_active;
  int freq;
  int pixel_clk;
  int vsync_start;
  int vsync_end;
  int screen_width;
  int h_sync_bytes;
  int h_bp_bytes;
  int h_fp_bytes;
  int line_bytes;
  int v_offset;
  float pio_clk_div; // PIO divider = sys_clk / TMDS_clk, must be integer or half-integer (n/2)
  // VGA-only overrides for fields above. If 0/zero, VGA uses the main fields.
  // HDMI never reads these — its timing is unaffected.
  int vga_v_total;
  int vga_v_active;
  int vga_pixel_clk;
  int vga_vsync_start;
  int vga_vsync_end;
  int vga_h_sync_bytes;
  int vga_h_bp_bytes;
  int vga_h_fp_bytes;
  int vga_screen_width;
};

enum graphics_mode_t {
    TEXTMODE_DEFAULT,
    GRAPHICSMODE_DEFAULT,
};

extern uint16_t graphics_max_tft_freq_mhz; // max SPI freq for TFT, MHz

void graphics_init();

void graphics_set_mode(enum graphics_mode_t mode);

void graphics_set_buffer(uint8_t* buffer, uint16_t width, uint16_t height);

void graphics_set_offset(int x, int y);

void graphics_set_palette(uint8_t i, uint32_t color);

void graphics_set_textbuffer(uint8_t* buffer);

void graphics_set_bgcolor(uint32_t color888);

void graphics_set_flashmode(bool flash_line, bool flash_frame);

void graphics_set_scanlines(bool enabled);
void graphics_set_dither(bool enabled);

void draw_text(const char string[TEXTMODE_COLS + 1], uint32_t x, uint32_t y, uint8_t color, uint8_t bgcolor);
void draw_window(const char title[TEXTMODE_COLS + 1], uint32_t x, uint32_t y, uint32_t width, uint32_t height);

void clrScr(uint8_t color);

struct video_mode_t graphics_get_video_mode(int mode);
void graphics_set_pio_clk_div(float div);

#ifdef __cplusplus
}
#endif
