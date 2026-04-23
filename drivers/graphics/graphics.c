#include "graphics.h"
#include <string.h>
#include "pico.h"

uint16_t graphics_max_tft_freq_mhz = 126;

// PIO clock divider must be integer or half-integer (n/2) for clean TMDS pixel clock.
// TMDS bit clock = pixel_clock * 10 = 252MHz for 25.2MHz pixel clock.
// sys_clk=378MHz → div=1.5; sys_clk=252MHz → div=1.0
#if CPU_MHZ <= 252
#define PIO_DIV  1.0f
#else
#define PIO_DIV  1.5f
#endif

static struct video_mode_t video_mode[] = {
    { // [0] 640x480 60Hz
        .v_total = 524,
        .v_active = 480,
        .freq = 60,
        .pixel_clk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [1] 640x480 50Hz Pentagon 48.82Hz
        .v_total = 644,
        .v_active = 480,
        .freq = 50,
        .pixel_clk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [2] 640x480 50Hz 48K 50.08Hz
        .v_total = 628,
        .v_active = 480,
        .freq = 50,
        .pixel_clk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [3] 640x480 50Hz 128K 50.02Hz
        .v_total = 629,
        .v_active = 480,
        .freq = 50,
        .pixel_clk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [4] 720x576 50Hz Pentagon full border — 25.2MHz pixel (sys_clk=378MHz, div=1.5)
        .v_total = 644,   // 25.2MHz/800/644 = 48.91Hz (Pentagon 48.83Hz)
        .v_active = 576,
        .freq = 50,
        .pixel_clk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [5] 720x576 50Hz 48K full border — 25.2MHz pixel
        .v_total = 628,   // 25.2MHz/800/628 = 50.09Hz (48K 50.08Hz)
        .v_active = 576,
        .freq = 50,
        .pixel_clk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [6] 720x576 50Hz 128K full border — 25.2MHz pixel
        .v_total = 629,   // 25.2MHz/800/629 = 50.00Hz (128K 50.02Hz)
        .v_active = 576,
        .freq = 50,
        .pixel_clk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [7] 720x480 60Hz half border
        .v_total = 524,
        .v_active = 480,
        .freq = 60,
        .pixel_clk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    },
    { // [8] 720x576 60Hz full border — 25.2MHz pixel (non-standard: v_active>v_total)
        .v_total = 524,   // 25.2MHz/800/524 ≈ 60.1Hz; v_active=576>524 so all lines are active
        .v_active = 576,
        .freq = 60,
        .pixel_clk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400,
        .v_offset = 0,
        .pio_clk_div = PIO_DIV
    }
};

/**
void draw_text(const char string[TEXTMODE_COLS + 1], uint32_t x, uint32_t y, uint8_t color, uint8_t bgcolor) {
if (!text_buffer) return;
    uint8_t* t_buf = text_buffer + TEXTMODE_COLS * 2 * y + 2 * x;
    for (int xi = TEXTMODE_COLS * 2; xi--;) {
        if (!*string) break;
        *t_buf++ = *string++;
        *t_buf++ = bgcolor << 4 | color & 0xF;
    }
}
*/
void draw_window(const char title[TEXTMODE_COLS + 1], uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    char line[width + 1];
    memset(line, 0, sizeof line);
    width--;
    height--;
    // Рисуем рамки

    memset(line, 0xCD, width); // ═══


    line[0] = 0xC9; // ╔
    line[width] = 0xBB; // ╗
    draw_text(line, x, y, 11, 1);

    line[0] = 0xC8; // ╚
    line[width] = 0xBC; //  ╝
    draw_text(line, x, height + y, 11, 1);

    memset(line, ' ', width);
    line[0] = line[width] = 0xBA;

    for (int i = 1; i < height; i++) {
        draw_text(line, x, y + i, 11, 1);
    }

    snprintf(line, width - 1, " %s ", title);
    draw_text(line, x + (width - strlen(line)) / 2, y, 14, 3);
}

struct video_mode_t __not_in_flash_func(graphics_get_video_mode)(int mode)
{
    return video_mode[mode];
}

void graphics_set_pio_clk_div(float div)
{
    for (int i = 0; i < sizeof(video_mode)/sizeof(video_mode[0]); i++)
        video_mode[i].pio_clk_div = div;
}

#ifdef VGA_HDMI
extern void hdmi_set_scanlines(bool enabled);
extern void vga_set_scanlines(bool enabled);
void graphics_set_scanlines(bool enabled) {
    hdmi_set_scanlines(enabled);
    vga_set_scanlines(enabled);
}
#else
void graphics_set_scanlines(bool enabled) {
    (void)enabled;
}
#endif