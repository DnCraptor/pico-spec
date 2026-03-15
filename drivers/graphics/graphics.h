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
};

enum graphics_mode_t {
    TEXTMODE_DEFAULT,
    GRAPHICSMODE_DEFAULT,
};

void graphics_init();

void graphics_set_mode(enum graphics_mode_t mode);

void graphics_set_buffer(uint8_t* buffer, uint16_t width, uint16_t height);

void graphics_set_offset(int x, int y);

void graphics_set_palette(uint8_t i, uint32_t color);

void graphics_set_textbuffer(uint8_t* buffer);

void graphics_set_bgcolor(uint32_t color888);

void graphics_set_flashmode(bool flash_line, bool flash_frame);

void clrScr(uint8_t color);

struct video_mode_t graphics_get_video_mode(int mode);

#ifdef __cplusplus
}
#endif
