#include "graphics.h"
#include <string.h>

static struct video_mode_t video_mode[] = {
    { // 640x480 60Hz
        .h_total = 524,
        .h_width = 480,
        .freq = 60,
        .vgaPxClk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400
    },
    { // 640x480 50Hz Pentagon 48.82Hz
        // HDMI pixel clock stays 25.175MHz (same as 60Hz), frame rate set by h_total
        .h_total = 644,
        .h_width = 480,
        .freq = 50,
        .vgaPxClk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400
    },
    { // 640x480 50Hz 48K 50.08Hz
        .h_total = 628,
        .h_width = 480,
        .freq = 50,
        .vgaPxClk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400
    },
    { // 640x480 50Hz 128K 50.02Hz
        .h_total = 629,
        .h_width = 480,
        .freq = 50,
        .vgaPxClk = 25175000,
        .vsync_start = 490,
        .vsync_end = 492,
        .screen_width = 320,
        .h_sync_bytes = 48,
        .h_bp_bytes = 24,
        .h_fp_bytes = 8,
        .line_bytes = 400
    },
    { // 720x576 50Hz Pentagon full border (25.175MHz pixel clock, 800px/line)
        // Uses same TMDS rate as 640x480 (252MHz, PIO divider 1.5) to avoid jitter
        // 720 active + 32 sync + 32 BP + 16 FP = 800 pixels/line = 400 bytes
        .h_total = 644,
        .h_width = 576,
        .freq = 50,
        .vgaPxClk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400
    },
    { // 720x576 50Hz 48K full border
        .h_total = 628,
        .h_width = 576,
        .freq = 50,
        .vgaPxClk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400
    },
    { // 720x576 50Hz 128K full border
        .h_total = 629,
        .h_width = 576,
        .freq = 50,
        .vgaPxClk = 25175000,
        .vsync_start = 581,
        .vsync_end = 586,
        .screen_width = 360,
        .h_sync_bytes = 16,
        .h_bp_bytes = 16,
        .h_fp_bytes = 8,
        .line_bytes = 400
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

struct video_mode_t graphics_get_video_mode(int mode)
{
    return video_mode[mode];
}