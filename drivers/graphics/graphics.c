#include "graphics.h"
#include <string.h>

static struct video_mode_t video_mode[] = {
    { // 640x480 60Hz
        .h_total = 524,
        .h_width = 480,
        .freq = 60,
        .vgaPxClk = 25175000
    },
    { // 640x480 50Hz Pentagon 48.82Hz
        .h_total = 644,
        .h_width = 480,
        .freq = 50,
        .vgaPxClk = 20979000
    },
    { // 640x480 50Hz 48K 50.08Hz
        .h_total = 628,
        .h_width = 480,
        .freq = 50,
        .vgaPxClk = 20979000
    },
    { // 640x480 50Hz 128K 50.02Hz
        .h_total = 629,
        .h_width = 480,
        .freq = 50,
        .vgaPxClk = 20979000
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