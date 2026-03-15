#include "graphics.h"
#include <string.h>

// PIO clock divider must be integer or half-integer (n/2) for clean TMDS pixel clock.
// TMDS bit clock = pixel_clock * 10 = 252MHz for 25.2MHz pixel clock.
// sys_clk=378MHz → div=1.5; sys_clk=252MHz → div=1.0
#if CPU_MHZ <= 252 || PICO_RP2040
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

struct video_mode_t graphics_get_video_mode(int mode)
{
    return video_mode[mode];
}