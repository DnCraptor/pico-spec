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
  int h_total;
  int h_width;
  int freq;
  int vgaPxClk;
};

enum graphics_mode_t {
    TEXTMODE_DEFAULT,
    GRAPHICSMODE_DEFAULT,
};

void graphics_init();

void graphics_set_mode(enum graphics_mode_t mode);

void graphics_set_buffer(uint8_t* buffer, uint16_t width, uint16_t height);

void graphics_set_offset(int x, int y);

static void graphics_set_palette(uint8_t i, uint32_t color);

void graphics_set_textbuffer(uint8_t* buffer);

void graphics_set_bgcolor(uint32_t color888);

void graphics_set_flashmode(bool flash_line, bool flash_frame);

void draw_text(const char string[TEXTMODE_COLS + 1], uint32_t x, uint32_t y, uint8_t color, uint8_t bgcolor);
void draw_window(const char title[TEXTMODE_COLS + 1], uint32_t x, uint32_t y, uint32_t width, uint32_t height);

void clrScr(uint8_t color);

struct video_mode_t graphics_get_video_mode(int mode);

#ifdef __cplusplus
}
#endif
