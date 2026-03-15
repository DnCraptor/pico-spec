#include <pico.h>
#include <hardware/structs/bus_ctrl.h>

#include "hdmi.h"
#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#define DVI_TIMING dvi_timing_640x480p_60hz
struct dvi_inst dvi0;
#define AUDIO_BUFFER_SIZE   256
audio_sample_t      audio_buffer[AUDIO_BUFFER_SIZE];
struct repeating_timer audio_timer;
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 240

static int16_t R = 0, L = 0;

void pcm_call(void);

bool __not_in_flash_func(audio_timer_callback)(struct repeating_timer *t) {
	while(true) {
		int size = get_write_size(&dvi0.audio_ring, false);
		if (size == 0) return true;
		audio_sample_t *audio_ptr = get_write_pointer(&dvi0.audio_ring);
		audio_sample_t sample;
		static uint sample_count = 0;
		for (int cnt = 0; cnt < size; cnt++) {
			sample.channels[0] = R;
			sample.channels[1] = L;
			*audio_ptr++ = sample;
			sample_count = sample_count + 1;
		}
		increase_write_pointer(&dvi0.audio_ring, size);
	}
}

void hdmi_audio_init(void) {
    //
}

void __not_in_flash_func(hdmi_audio_write_sample)(int16_t left, int16_t right) {
    L = left;
    R = right;
}

static const uint32_t palB[16] = {
    // --------------------------
    // plane 0: BLUE
    // --------------------------
    0x7f103, // 0  black
    0xb3e30, // 1  blue
    0x7f103, // 2  red
    0xb3e30, // 3  magenta
    0x7f103, // 4  green
    0xb3e30, // 5  cyan
    0x7f103, // 6  yellow
    0xb3e30, // 7  white

    0x7f103, // 8  bright black
    0xbf203, // 9  bright blue
    0x7f103, // 10 bright red
    0xbf203, // 11 bright magenta
    0x7f103, // 12 bright green
    0xbf203, // 13 bright cyan
    0x7f103, // 14 bright yellow
    0xbf203 // 15 bright white
};
static const uint32_t palG[16] = {
    // --------------------------
    // plane 1: GREEN
    // --------------------------
    0x7f103, // 0  black
    0x7f103, // 1  blue
    0x7f103, // 2  red
    0x7f103, // 3  magenta
    0xb3e30, // 4  green
    0xb3e30, // 5  cyan
    0xb3e30, // 6  yellow
    0xb3e30, // 7  white

    0x7f103, // 8  bright black
    0x7f103, // 9  bright blue
    0x7f103, // 10 bright red
    0x7f103, // 11 bright magenta
    0xbf203, // 12 bright green
    0xbf203, // 13 bright cyan
    0xbf203, // 14 bright yellow
    0xbf203 // 15 bright white
};
static const uint32_t palR[16] = {
    // --------------------------
    // plane 2: RED
    // --------------------------
    0x7f103, // 0  black
    0x7f103, // 1  blue
    0xb3e30, // 2  red
    0xb3e30, // 3  magenta
    0x7f103, // 4  green
    0x7f103, // 5  cyan
    0xb3e30, // 6  yellow
    0xb3e30, // 7  white

    0x7f103, // 8  bright black
    0x7f103, // 9  bright blue
    0xbf203, // 10 bright red
    0xbf203, // 11 bright magenta
    0x7f103, // 12 bright green
    0x7f103, // 13 bright cyan
    0xbf203, // 14 bright yellow
    0xbf203  // 15 bright white
};

static uint32_t pal[3*256];

void graphics_set_palette(const uint8_t i, const uint32_t color888) {
 //   pal[i] = (i & 1) ? 0x7f103 : 0xbf203; // W/A just to show "something"
}

void graphics_init0(void) {
    for (int i = 0; i < 16; ++i) {
     //   graphics_set_palette(i, rgb); // TODO:
        pal[i] = palB[i];
        pal[i + 256] = palG[i];
        pal[i + 512] = palR[i];
    }
    for (int i = 16; i < 256; ++i) {
        pal[i] = (i & 1) ? 0x7f103 : 0xbf203; // W/A just to show "something"
        pal[i + 256] = (i & 1) ? 0x7f103 : 0xbf203; // W/A just to show "something"
        pal[i + 512] = (i & 1) ? 0x7f103 : 0xbf203; // W/A just to show "something"
    }
	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());
	hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
	// HDMI Audio related
	dvi_get_blank_settings(&dvi0)->top    = 4 * 0;
	dvi_get_blank_settings(&dvi0)->bottom = 4 * 0;
	dvi_audio_sample_buffer_set(&dvi0, audio_buffer, AUDIO_BUFFER_SIZE);
	dvi_set_audio_freq(&dvi0, 44100, 28000, 6272);
	add_repeating_timer_ms(-2, audio_timer_callback, NULL, &audio_timer);
}

void graphics_init(void) {
// before a cycle on core1
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
}

void graphics_set_buffer(uint8_t* buffer, const uint16_t width, const uint16_t height) {

}

void graphics_set_bgcolor(const uint32_t color888) {

}

// Update a VGA palette entry WITHOUT dithering (both palettes get identical solid color)
// Use for the 16 standard Spectrum colors to avoid visible dithering artifacts
void vga_set_palette_entry_solid(uint8_t i, uint32_t color888) {

}

void graphics_set_flashmode(const bool flash_line, const bool flash_frame) {

}

uint8_t* getLineBuffer(int line);

void __not_in_flash_func(dvi_line)(uint32_t line) {
    uint8_t* src = getLineBuffer(line);
    uint32_t *tmdsbuf;
    queue_remove_blocking(&dvi0.q_tmds_free, &tmdsbuf);
    uint32_t* pali = (uint32_t*)pal;
    for (int plane = 0; plane < 3; ++plane, pali += 256) {
        uint32_t* target = tmdsbuf + plane * (FRAME_WIDTH / DVI_SYMBOLS_PER_WORD);
        for (uint x = 0; x < 320; ++x) {
            *target++ = pali[src[x ^ 2]];
if (x & 2) pcm_call(); // once per 4 pixels... TODO: ensure enough freq. (may be use separate timer?)
        }
    }
    queue_add_blocking(&dvi0.q_tmds_valid, &tmdsbuf);
}
