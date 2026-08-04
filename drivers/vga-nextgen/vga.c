
#include "graphics.h"
#include "hardware/clocks.h"
#include "stdbool.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/systick.h"

#include "hardware/dma.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdio.h>
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "stdlib.h"

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)

/// TODO: .h
bool SELECT_VGA = false;
uint8_t* getLineBuffer(int line);
void ESPectrum_vsync();
int get_video_mode();

uint16_t pio_program_VGA_instructions[] = {
    //     .wrap_target
    0x6008, //  0: out    pins, 8
    //     .wrap
};

const struct pio_program pio_program_VGA = {
    .instructions = pio_program_VGA_instructions,
    .length = 1,
    .origin = -1,
};


static uint32_t* lines_pattern[4];
static uint32_t* lines_pattern_data = NULL;
static int _SM_VGA = -1;


//static int N_lines_total = 525;
//static int N_lines_visible = 480;
static int line_VS_begin = 490;
static int line_VS_end = 491;
static int shift_picture = 0;

static int visible_line_size = 320;
static int line_size = 800;
static int HS_SIZE = 96;
static int HS_SHIFT = 656;

// Maximum DMA buffer size for any supported VGA mode (e.g. 720x576 needs ~880).
// lines_pattern_data is allocated to this size, smaller modes use only part of it.
#define VGA_MAX_LINE_SIZE 1024

// Sync byte templates set during init. Stored to allow re-rendering line patterns
// when the mode (and thus HS_SIZE / line_size) changes at runtime.
static uint8_t TMPL_LINE8_g = 0b11000000;
static uint8_t TMPL_HS8_g   = 0b10000000;
static uint8_t TMPL_VS8_g   = 0b01000000;
static uint8_t TMPL_VHS8_g  = 0b00000000;


static int dma_chan_ctrl;
static int dma_chan;

#endif

/// TODO:
int graphics_buffer_width = 0;
int graphics_buffer_height = 0;
int graphics_buffer_shift_x = 0;
int graphics_buffer_shift_y = 0;

static bool is_flash_line = false;
static bool is_flash_frame = false;

// Unified DS80-active flag: set by both vga_set_profi_ds80_mode and hdmi_set_profi_ds80_mode.
// Kept on all platforms (1 byte) — stays false on RP2040.
volatile bool profi_ds80_active = false;

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)

bool vga_scanlines = false;
// Scanline brightness level: 0=off, 1=darkest .. 4=lightest. Level 2 is the
// legacy ~50% look and the default. Drives dim_rgb888() when (re)building the
// dimmed palette. vga_scanlines stays a fast on/off flag for the render path.
static uint8_t vga_scanline_level = 2;

static uint32_t bg_color[2];
static uint16_t palette16_mask = 0;

// VGA8 dithered palette LUT: /42 checkerboard dithering (7 levels per channel, 343 colors)
// [0][i] = even scanline pixel pair,  [1][i] = odd scanline pixel pair
// pico-spec: these VGA-only tables (~4 KB total) are lazily heap-allocated and only
// exist when VGA is the active output (SELECT_VGA). On HDMI boots they stay NULL —
// the setters below no-op — so the SRAM isn't reserved. See vga_alloc_buffers().
static uint16_t (*palette_vga16)[256] = (uint16_t (*)[256]) 0;

// Scanline dimmed palette: dithered at reduced brightness for scanline effect.
// Rebuilt from vga_color888[] whenever the scanline level changes.
static uint16_t *palette_vga16_scanline = (uint16_t *) 0;
// Original RGB888 per palette index + whether it was set via the solid path.
// Cached so the dimmed palette can be rebuilt on a brightness-level change
// without re-walking the whole palette from the emulator side.
static uint32_t *vga_color888 = (uint32_t *) 0;
static bool     *vga_color_solid = (bool *) 0;

// Profi DS80 packed-pair palette: slot byte → uint16_t with two distinct VGA pixels.
// Built by vga_set_profi_ds80_mode(); slot comes from profi_pair_lookup[p0][p1].
// low byte = left pixel (p0), high byte = right pixel (p1) — PIO right-shifts out LSB first.
// [0] = even scan lines, [1] = odd scan lines (Bayer 2×2 checkerboard dithering).
// Profi DS80 is RP2350-only; the 1 KB palette is excluded on RP2040 to save RAM
// (heap is ~5 KB after the framebuffer there).
#if !PICO_RP2040
static uint16_t (*palette_vga_ds80)[256] = (uint16_t (*)[256]) 0;  // pico-spec: lazy (see above)
#endif

// Allocate the VGA-only palette tables on first use (VGA active only). Idempotent.
// Keeps ~4 KB out of .bss on HDMI boots, where these are never touched.
// pico_malloc/calloc PANIC on OOM instead of returning NULL, so pre-check the
// largest satisfiable block (same guard as Buffer::palloc / graphics_init_hdmi)
// — this runs on core1 right after setup, when free heap can be only a few KB.
static void vga_alloc_buffers(void) {
    if (vga_color888) return;                                   // already allocated
    extern size_t getLargestAllocatable(void);
    size_t need = 2 * 256 * sizeof(uint16_t) + 256 * sizeof(uint16_t)
                + 256 * sizeof(uint32_t) + 256 * sizeof(bool);
#if !PICO_RP2040
    need += 2 * 256 * sizeof(uint16_t);
#endif
    if (getLargestAllocatable() < need) {
        printf("vga_alloc_buffers: OOM allocating VGA palette tables (%u B needed)\n", (unsigned)need);
        return;
    }
    palette_vga16          = (uint16_t (*)[256]) calloc(2 * 256, sizeof(uint16_t));
    palette_vga16_scanline = (uint16_t *)        calloc(256,     sizeof(uint16_t));
    vga_color888           = (uint32_t *)        calloc(256,     sizeof(uint32_t));
    vga_color_solid        = (bool *)            calloc(256,     sizeof(bool));
#if !PICO_RP2040
    palette_vga_ds80       = (uint16_t (*)[256]) calloc(2 * 256, sizeof(uint16_t));
#endif
}

// True once the VGA tables exist. On HDMI (SELECT_VGA false) returns false so the
// palette setters become no-ops; on VGA it allocates lazily on first call.
static inline bool vga_buffers_ready(void) {
    if (vga_color888) return true;
    if (!SELECT_VGA) return false;
    vga_alloc_buffers();
    return vga_color888 != (uint32_t *) 0;
}

static uint text_buffer_width = 0;
static uint text_buffer_height = 0;

static uint16_t txt_palette[16];

//буфер 2К текстовой палитры для быстрой работы
static uint16_t* txt_palette_fast = NULL;
//static uint16_t txt_palette_fast[256*4];

enum graphics_mode_t graphics_mode = GRAPHICSMODE_DEFAULT;

void __time_critical_func() dma_handler_VGA() {
    dma_hw->ints0 = 1u << dma_chan_ctrl;
    static uint32_t frame_number = 0;
    static uint32_t screen_line = 0;
    static uint8_t* input_buffer = NULL;
    screen_line++;

    struct video_mode_t mode = graphics_get_video_mode(get_video_mode());
    int v_total = mode.vga_v_total ? mode.vga_v_total : mode.v_total;
    int v_active = mode.vga_v_active ? mode.vga_v_active : mode.v_active;

    if (screen_line == v_total) {
        screen_line = 0;
        frame_number++;
        input_buffer = getLineBuffer(screen_line);
    }

    // Signal vsync at start of blanking (after last visible line), so emulator
    // renders the next frame during blanking while VGA isn't reading frameBuffer —
    // prevents tearing at the top of the screen.
    if (screen_line == v_active) {
        ESPectrum_vsync();
    }

    if (screen_line >= v_active) {
        //заполнение цветом фона
        if (screen_line == v_active | screen_line == v_active + 3) {
            uint32_t* output_buffer_32bit = lines_pattern[2 + (screen_line & 1)];
            output_buffer_32bit += shift_picture / 4;
            uint32_t p_i = (screen_line & is_flash_line) + (frame_number & is_flash_frame) & 1;
            uint32_t color32 = bg_color[p_i];
            for (int i = visible_line_size / 2; i--;) {
                *output_buffer_32bit++ = color32;
            }
        }

        //синхросигналы
        if (screen_line >= line_VS_begin && screen_line <= line_VS_end)
            dma_channel_set_read_addr(dma_chan_ctrl, &lines_pattern[1], false); //VS SYNC
        else
            dma_channel_set_read_addr(dma_chan_ctrl, &lines_pattern[0], false);
        return;
    }

    if (!input_buffer) {
        dma_channel_set_read_addr(dma_chan_ctrl, &lines_pattern[0], false);
        return;
    } //если нет видеобуфера - рисуем пустую строку

    int y, line_number;

    uint32_t* * output_buffer = &lines_pattern[2 + (screen_line & 1)];
    switch (graphics_mode) {
        case GRAPHICSMODE_DEFAULT:
            line_number = screen_line / 2;
            y = screen_line / 2 - graphics_buffer_shift_y;
            break;
/**
        case TEXTMODE_160x100:
        case TEXTMODE_53x30:
        case TEXTMODE_DEFAULT: {
            uint16_t* output_buffer_16bit = (uint16_t *)*output_buffer;
            output_buffer_16bit += shift_picture / 2;
            const uint font_height = 16;

            // "слой" символа
            uint32_t glyph_line = screen_line % font_height;
if (!text_buffer) return;
            //указатель откуда начать считывать символы
            uint8_t* text_buffer_line = &text_buffer[screen_line / font_height * text_buffer_width * 2];

            for (int x = 0; x < text_buffer_width; x++) {
                //из таблицы символов получаем "срез" текущего символа
                uint8_t glyph_pixels = font_8x16[*text_buffer_line++ * font_height + glyph_line];
                //считываем из быстрой палитры начало таблицы быстрого преобразования 2-битных комбинаций цветов пикселей
                uint16_t* color = &txt_palette_fast[*text_buffer_line++ * 4];
#if 0
                if (cursor_blink_state && !manager_started &&
                    (screen_line / 16 == CURSOR_Y && x == CURSOR_X && glyph_line >= 11 && glyph_line <= 13)) {
                    *output_buffer_16bit++ = color[3];
                    *output_buffer_16bit++ = color[3];
                    *output_buffer_16bit++ = color[3];
                    *output_buffer_16bit++ = color[3];
                    if (text_buffer_width == 40) {
                        *output_buffer_16bit++ = color[3];
                        *output_buffer_16bit++ = color[3];
                        *output_buffer_16bit++ = color[3];
                        *output_buffer_16bit++ = color[3];
                    }
                }
                else
#endif
                {
                    *output_buffer_16bit++ = color[glyph_pixels & 3];
                    if (text_buffer_width == 40) *output_buffer_16bit++ = color[glyph_pixels & 3];
                    glyph_pixels >>= 2;
                    *output_buffer_16bit++ = color[glyph_pixels & 3];
                    if (text_buffer_width == 40) *output_buffer_16bit++ = color[glyph_pixels & 3];
                    glyph_pixels >>= 2;
                    *output_buffer_16bit++ = color[glyph_pixels & 3];
                    if (text_buffer_width == 40) *output_buffer_16bit++ = color[glyph_pixels & 3];
                    glyph_pixels >>= 2;
                    *output_buffer_16bit++ = color[glyph_pixels & 3];
                    if (text_buffer_width == 40) *output_buffer_16bit++ = color[glyph_pixels & 3];
                }
            }
            dma_channel_set_read_addr(dma_chan_ctrl, output_buffer, false);
            return;
        }
*/
        default: {
            dma_channel_set_read_addr(dma_chan_ctrl, &lines_pattern[0], false); // TODO: ensue it is required
            return;
        }
    }

    if (y < 0) {
        dma_channel_set_read_addr(dma_chan_ctrl, &lines_pattern[0], false); // TODO: ensue it is required
        return;
    }
    if (y >= graphics_buffer_height) {
        // заполнение линии цветом фона
        if (y == graphics_buffer_height | y == graphics_buffer_height + 1 |
            y == graphics_buffer_height + 2) {
            uint32_t* output_buffer_32bit = *output_buffer;
            uint32_t p_i = ((screen_line & is_flash_line) + (frame_number & is_flash_frame)) & 1;
            uint32_t color32 = bg_color[p_i];

            output_buffer_32bit += shift_picture / 4;
            for (int i = visible_line_size / 2; i--;) {
                *output_buffer_32bit++ = color32;
            }
        }
        dma_channel_set_read_addr(dma_chan_ctrl, output_buffer, false);
        return;
    };

    //зона прорисовки изображения
    //начальные точки буферов
    uint8_t* input_buffer_8bit ///= input_buffer + y / 2 * 80 + (y & 1) * 8192;
             = getLineBuffer(y);

    uint16_t* output_buffer_16bit = (uint16_t *)(*output_buffer);
    output_buffer_16bit += shift_picture / 2; //смещение началы вывода на размер синхросигнала

    //    g_buf_shx&=0xfffffffe;//4bit buf
    //    graphics_buffer_shift_x &= 0xfffffff1; //1bit buf
        graphics_buffer_shift_x &= 0xfffffff2; //2bit buf

    //для div_factor 2
    uint max_width = graphics_buffer_width;
    if (graphics_buffer_shift_x < 0) {
        //vbuf8-=g_buf_shx; //8bit buf
///            input_buffer_8bit -= graphics_buffer_shift_x / 8; //1bit buf
            input_buffer_8bit -= graphics_buffer_shift_x / 4; //2bit buf
        max_width += graphics_buffer_shift_x;
    }
    else {
#define div_factor (2)
        output_buffer_16bit += graphics_buffer_shift_x * 2 / div_factor;
    }


    int width = MIN((visible_line_size - ((graphics_buffer_shift_x > 0) ? (graphics_buffer_shift_x) : 0)), max_width);
    if (width < 0) return; // TODO: detect a case

    // Индекс палитры в зависимости от настроек чередования строк и кадров
///    uint16_t* current_palette = palette[(y & is_flash_line) + (frame_number & is_flash_frame) & 1];

    uint8_t* output_buffer_8bit;
    switch (graphics_mode) {
        case GRAPHICSMODE_DEFAULT: {
#if !PICO_RP2040
            if (profi_ds80_active) {
                uint16_t* pal = palette_vga_ds80[screen_line & 1];
                for (int x = 0; x < width; ++x) {
                    register uint8_t idx = input_buffer_8bit[x ^ 2];
                    *output_buffer_16bit++ = pal[idx];
                }
            } else
#endif
            {
                uint16_t* pal = (vga_scanlines && (screen_line & 1))
                    ? palette_vga16_scanline
                    : palette_vga16[screen_line & 1];
                for (int x = 0; x < width; ++x) {
                    register uint8_t idx = input_buffer_8bit[x ^ 2];
                    *output_buffer_16bit++ = pal[idx];
                }
            }
            break;
        }
        default:
            break;
    }
    dma_channel_set_read_addr(dma_chan_ctrl, output_buffer, false);
}

void graphics_set_mode(enum graphics_mode_t mode) {
    if (!SELECT_VGA) {
        graphics_mode = mode;
        return;
    }
    vga_alloc_buffers();   // VGA active — the palette-mask loop below needs the tables
    text_buffer_width = 80;
    text_buffer_height = 30;
///    memset(graphics_buffer, 0, graphics_buffer_height * graphics_buffer_width);
    if (_SM_VGA < 0) return; // если  VGA не инициализирована -

    graphics_mode = mode;

    // Если мы уже проиницилизированы - выходим
    if (txt_palette_fast && lines_pattern_data) {
        return;
    };
    uint8_t TMPL_VHS8 = 0;
    uint8_t TMPL_VS8 = 0;
    uint8_t TMPL_HS8 = 0;
    uint8_t TMPL_LINE8 = 0;

    double fdiv = 100;
    // line_size, HS_SIZE, HS_SHIFT are now globals — assigned from video_mode below

    switch (graphics_mode) {
        case TEXTMODE_DEFAULT:
            //текстовая палитра
            for (int i = 0; i < 16; i++) {
                txt_palette[i] = txt_palette[i] & 0x3f | palette16_mask >> 8;
            }

            if (!txt_palette_fast) {
                txt_palette_fast = (uint16_t *)calloc(256 * 4, sizeof(uint16_t));
                for (int i = 0; i < 256; i++) {
                    const uint8_t c1 = txt_palette[i & 0xf];
                    const uint8_t c0 = txt_palette[i >> 4];

                    txt_palette_fast[i * 4 + 0] = c0 | c0 << 8;
                    txt_palette_fast[i * 4 + 1] = c1 | c0 << 8;
                    txt_palette_fast[i * 4 + 2] = c0 | c1 << 8;
                    txt_palette_fast[i * 4 + 3] = c1 | c1 << 8;
                }
            }
        case GRAPHICSMODE_DEFAULT: {
            TMPL_LINE8 = 0b11000000;
            palette16_mask = 0xc0c0;
            line_VS_begin = 490;
            line_VS_end = 491;
            struct video_mode_t vMode = graphics_get_video_mode(get_video_mode());
            int vga_px = vMode.vga_pixel_clk ? vMode.vga_pixel_clk : vMode.pixel_clk;
            fdiv = clock_get_hz(clk_sys) / vga_px; //частота пиксельклока
            // Compute line layout from video_mode fields (×2 because VGA byte=pixel
            // and HDMI table values are in HDMI-bytes which encode 2 pixels each).
            int hs_b = vMode.vga_h_sync_bytes ? vMode.vga_h_sync_bytes : vMode.h_sync_bytes;
            int bp_b = vMode.vga_h_bp_bytes   ? vMode.vga_h_bp_bytes   : vMode.h_bp_bytes;
            int fp_b = vMode.vga_h_fp_bytes   ? vMode.vga_h_fp_bytes   : vMode.h_fp_bytes;
            int sw_b = vMode.vga_screen_width ? vMode.vga_screen_width : vMode.screen_width;
            HS_SIZE          = hs_b * 2;
            int bp           = bp_b * 2;
            visible_line_size = sw_b;
            int active_bytes = visible_line_size * 2;
            int fp           = fp_b * 2;
            line_size        = HS_SIZE + bp + active_bytes + fp;
            shift_picture    = HS_SIZE + bp;              // offset where active picture starts in line buffer
            HS_SHIFT         = line_size - shift_picture; // legacy unused
            break;
        }
        default:
            return;
    }

    //корректировка  палитры по маске бит синхры
    bg_color[0] = bg_color[0] & 0x3f3f3f3f | palette16_mask | palette16_mask << 16;
    bg_color[1] = bg_color[1] & 0x3f3f3f3f | palette16_mask | palette16_mask << 16;
    // Re-apply sync bits to all VGA palette entries (palette may have been
    // initialized before palette16_mask was set)
    for (int i = 0; i < 256; i++) {
        palette_vga16[0][i] = (palette_vga16[0][i] & 0x3f3f) | palette16_mask;
        palette_vga16[1][i] = (palette_vga16[1][i] & 0x3f3f) | palette16_mask;
        palette_vga16_scanline[i] = (palette_vga16_scanline[i] & 0x3f3f) | palette16_mask;
    }

    // Save sync byte templates as globals so vga_reinit() can re-render line patterns
    TMPL_LINE8_g = TMPL_LINE8;
    TMPL_HS8_g   = TMPL_LINE8 ^ 0b01000000;
    TMPL_VS8_g   = TMPL_LINE8 ^ 0b10000000;
    TMPL_VHS8_g  = TMPL_LINE8 ^ 0b11000000;
    TMPL_VHS8 = TMPL_VHS8_g;
    TMPL_VS8  = TMPL_VS8_g;
    TMPL_HS8  = TMPL_HS8_g;

    //инициализация шаблонов строк и синхросигнала
    if (!lines_pattern_data) //выделение памяти, если не выделено
    {
        const uint32_t div32 = (uint32_t)(fdiv * (1 << 16) + 0.0);
        PIO_VGA->sm[_SM_VGA].clkdiv = div32 & 0xfffff000; //делитель для конкретной sm
        dma_channel_set_trans_count(dma_chan, line_size / 4, false);

        // Allocate to max possible line_size so re-renders during mode switches don't overrun.
        // pico_calloc PANICs on OOM instead of returning NULL — pre-check the
        // largest satisfiable block (this runs on core1 right after setup, when
        // free heap can be only a few KB).
        {
            extern size_t getLargestAllocatable(void);
            size_t need = (VGA_MAX_LINE_SIZE * 4 / 4) * sizeof(uint32_t);
            if (getLargestAllocatable() < need) {
                printf("graphics_set_mode: OOM allocating lines_pattern_data (%u B needed)\n", (unsigned)need);
                return;
            }
        }
        lines_pattern_data = (uint32_t *)calloc(VGA_MAX_LINE_SIZE * 4 / 4, sizeof(uint32_t));

        for (int i = 0; i < 4; i++) {
            lines_pattern[i] = &lines_pattern_data[i * (VGA_MAX_LINE_SIZE / 4)];
        }
    }

    // (Re-)render line templates for current line_size / HS_SIZE
    uint8_t* base_ptr = (uint8_t *)lines_pattern[0];
    memset(base_ptr, TMPL_LINE8, line_size);     // empty line: idle level
    memset(base_ptr, TMPL_HS8, HS_SIZE);         // hsync pulse at start

    base_ptr = (uint8_t *)lines_pattern[1];
    memset(base_ptr, TMPL_VS8, line_size);       // vsync line: vsync level
    memset(base_ptr, TMPL_VHS8, HS_SIZE);        // with hsync pulse at start

    // image line templates start as a copy of the empty line
    memcpy((uint8_t *)lines_pattern[2], lines_pattern[0], line_size);
    memcpy((uint8_t *)lines_pattern[3], lines_pattern[0], line_size);
}

void vga_reinit() {
    vga_alloc_buffers();   // VGA is the active output here — ensure the palette tables exist
    // Update VGA sync parameters, horizontal layout, and PIO pixel clock from current video_mode.
    // 50Hz modes use vga_pixel_clk override (lower clock + smaller v_total) so
    // monitors detect the signal as 640x480@50, not PAL 720x576@50.
    // HDMI ignores vga_* fields and keeps standard 25.175MHz / v_total=628..644.
    struct video_mode_t mode = graphics_get_video_mode(get_video_mode());
    line_VS_begin = mode.vga_vsync_start ? mode.vga_vsync_start : mode.vsync_start;
    line_VS_end = mode.vga_vsync_end ? mode.vga_vsync_end : mode.vsync_end;
    int pixel_clk = mode.vga_pixel_clk ? mode.vga_pixel_clk : mode.pixel_clk;
    double fdiv = (double)clock_get_hz(clk_sys) / (double)pixel_clk;
    const uint32_t div32 = (uint32_t)(fdiv * (1 << 16) + 0.0);
    PIO_VGA->sm[_SM_VGA].clkdiv = div32 & 0xfffff000;  // integer div only — fractional causes jitter

    // Recompute horizontal layout from video_mode and re-render line templates.
    // VGA byte=pixel; HDMI table values encode 2 pixels per byte, so multiply by 2.
    int hs_b = mode.vga_h_sync_bytes ? mode.vga_h_sync_bytes : mode.h_sync_bytes;
    int bp_b = mode.vga_h_bp_bytes   ? mode.vga_h_bp_bytes   : mode.h_bp_bytes;
    int fp_b = mode.vga_h_fp_bytes   ? mode.vga_h_fp_bytes   : mode.h_fp_bytes;
    int sw_b = mode.vga_screen_width ? mode.vga_screen_width : mode.screen_width;
    int new_HS_SIZE   = hs_b * 2;
    int new_bp        = bp_b * 2;
    int new_visible   = sw_b;
    int new_active    = new_visible * 2;
    int new_fp        = fp_b * 2;
    int new_line_size = new_HS_SIZE + new_bp + new_active + new_fp;

    if (new_line_size > VGA_MAX_LINE_SIZE) return;  // safety: don't overflow buffer

    bool layout_changed = (new_line_size != line_size) || (new_HS_SIZE != HS_SIZE)
                          || (new_visible != visible_line_size);

    HS_SIZE           = new_HS_SIZE;
    visible_line_size = new_visible;
    line_size         = new_line_size;
    shift_picture     = new_HS_SIZE + new_bp;
    HS_SHIFT          = line_size - shift_picture;

    if (layout_changed && lines_pattern_data) {
        // Re-render line templates with new HS_SIZE / line_size
        uint8_t* base_ptr = (uint8_t *)lines_pattern[0];
        memset(base_ptr, TMPL_LINE8_g, line_size);
        memset(base_ptr, TMPL_HS8_g, HS_SIZE);

        base_ptr = (uint8_t *)lines_pattern[1];
        memset(base_ptr, TMPL_VS8_g, line_size);
        memset(base_ptr, TMPL_VHS8_g, HS_SIZE);

        memcpy((uint8_t *)lines_pattern[2], lines_pattern[0], line_size);
        memcpy((uint8_t *)lines_pattern[3], lines_pattern[0], line_size);

        dma_channel_set_trans_count(dma_chan, line_size / 4, false);
    }
}
#endif // #if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)

void graphics_set_buffer(uint8_t* buffer, const uint16_t width, const uint16_t height) {
    graphics_buffer_width = width;
    graphics_buffer_height = height;
}


void graphics_set_offset(const int x, const int y) {
    graphics_buffer_shift_x = x;
    graphics_buffer_shift_y = y;
}

void graphics_set_flashmode(const bool flash_line, const bool flash_frame) {
    is_flash_frame = flash_frame;
    is_flash_line = flash_line;
}

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
// Bayer 2×2 ordered dithering: /21 → 13 levels/channel → 2197 perceived colors
// Threshold matrix: [0 2]  Fill order per sub-level: (0,0), (1,1), (0,1), (1,0)
//                   [3 1]
static void vga_rgb888_dither(uint32_t color888, uint16_t *even_pair, uint16_t *odd_pair) {
    uint8_t r_level = ((color888 >> 16) & 0xff) / 21;
    uint8_t g_level = ((color888 >> 8) & 0xff) / 21;
    uint8_t b_level = (color888 & 0xff) / 21;

    uint8_t r_lo = r_level >> 2, r_sub = r_level & 3, r_hi = r_lo + (r_lo < 3 && r_sub);
    uint8_t g_lo = g_level >> 2, g_sub = g_level & 3, g_hi = g_lo + (g_lo < 3 && g_sub);
    uint8_t b_lo = b_level >> 2, b_sub = b_level & 3, b_hi = b_lo + (b_lo < 3 && b_sub);

    // 4 pixel positions in 2×2 block, ordered by Bayer threshold
    uint8_t p00 = ((r_sub >= 1 ? r_hi : r_lo) << 4) | ((g_sub >= 1 ? g_hi : g_lo) << 2) | (b_sub >= 1 ? b_hi : b_lo);
    uint8_t p01 = ((r_sub >= 3 ? r_hi : r_lo) << 4) | ((g_sub >= 3 ? g_hi : g_lo) << 2) | (b_sub >= 3 ? b_hi : b_lo);
    uint8_t p10 = (r_lo << 4) | (g_lo << 2) | b_lo;
    uint8_t p11 = ((r_sub >= 2 ? r_hi : r_lo) << 4) | ((g_sub >= 2 ? g_hi : g_lo) << 2) | (b_sub >= 2 ? b_hi : b_lo);

    // Pack pixel pairs: LSB = first pixel (PIO right-shift), MSB = second pixel
    *even_pair = ((p01 << 8) | p00) & 0x3f3f | palette16_mask;
    *odd_pair  = ((p11 << 8) | p10) & 0x3f3f | palette16_mask;
}

// Per-level scanline brightness, as a 0..256 multiplier applied to each RGB
// channel. Index by level (1..4); level 2 == 128/256 == the legacy ~50% look.
// dark -> light. Index 0 is unused (scanlines off).
static const uint16_t scanline_dim_num[5] = { 128, 64, 128, 184, 224 };

// Dim RGB888 color for the current scanline brightness level.
static uint32_t dim_rgb888(uint32_t color888) {
    uint16_t num = scanline_dim_num[(vga_scanline_level <= 4) ? vga_scanline_level : 2];
    uint8_t r = (((color888 >> 16) & 0xff) * num) >> 8;
    uint8_t g = (((color888 >> 8) & 0xff) * num) >> 8;
    uint8_t b = ((color888 & 0xff) * num) >> 8;
    return (r << 16) | (g << 8) | b;
}

// Recompute the dimmed scanline pixel for a single index from its cached color.
static void vga_build_scanline_entry(uint8_t i) {
    if (!vga_color888) return;                 // VGA tables not allocated (HDMI active)
    uint32_t dim = dim_rgb888(vga_color888[i]);
    if (vga_color_solid[i]) {
        uint8_t r2 = ((dim >> 16) & 0xff) / 85;
        uint8_t g2 = ((dim >> 8) & 0xff) / 85;
        uint8_t b2 = (dim & 0xff) / 85;
        uint8_t vga6 = (r2 << 4) | (g2 << 2) | b2;
        palette_vga16_scanline[i] = ((vga6 << 8) | vga6) & 0x3f3f | palette16_mask;
    } else {
        uint16_t dummy;
        vga_rgb888_dither(dim, &dummy, &palette_vga16_scanline[i]);
    }
}

// Update a single VGA palette LUT entry with Bayer dithering
void vga_set_palette_entry(uint8_t i, uint32_t color888) {
    if (!vga_buffers_ready()) return;          // HDMI active → VGA tables unused
    vga_rgb888_dither(color888, &palette_vga16[0][i], &palette_vga16[1][i]);
    vga_color888[i] = color888;
    vga_color_solid[i] = false;
    // Scanline: dithered at the current brightness level (odd pair for single-line rendering)
    vga_build_scanline_entry(i);
}
#else
void vga_set_palette_entry_solid(uint8_t i, uint32_t color888) {}
#endif

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
// Update a VGA palette entry WITHOUT dithering (both palettes get identical solid color)
// Use for the 16 standard Spectrum colors to avoid visible dithering artifacts
void vga_set_palette_entry_solid(uint8_t i, uint32_t color888) {
    if (!vga_buffers_ready()) return;          // HDMI active → VGA tables unused
    uint8_t r2 = ((color888 >> 16) & 0xff) / 85;
    uint8_t g2 = ((color888 >> 8) & 0xff) / 85;
    uint8_t b2 = (color888 & 0xff) / 85;
    uint8_t vga6 = (r2 << 4) | (g2 << 2) | b2;
    uint16_t solid = ((vga6 << 8) | vga6) & 0x3f3f | palette16_mask;
    palette_vga16[0][i] = solid;
    palette_vga16[1][i] = solid;
    vga_color888[i] = color888;
    vga_color_solid[i] = true;
    // Scanline: dimmed solid at the current brightness level
    vga_build_scanline_entry(i);
}

// Build VGA DS80 packed-pair palette from the 16-color Profi palette and pair_lut.
// pair_lut is profi_pair_lookup[0][0] (flat 256-byte): pair_lut[p0*16+p1] = slot.
// Each slot byte maps to a uint16_t: low byte = VGA pixel for p0 (left),
// high byte = VGA pixel for p1 (right). PIO right-shifts LSB first → correct order.
// Two tables (even/odd scan lines) implement Bayer 2×2 checkerboard dithering:
//   p0 is always at even screen x, p1 at odd screen x.
//   vga_rgb888_dither() gives: even_pair=low(even_x,even_y)/high(odd_x,even_y),
//                               odd_pair =low(even_x,odd_y) /high(odd_x,odd_y).
void vga_set_profi_ds80_mode(bool active,
                              const uint32_t *palette16_rgb888,
                              const uint8_t  *pair_lut) {
#if PICO_RP2040
    // Profi DS80 is RP2350-only; no palette table on RP2040. Keep the symbol so
    // any (guarded-out) caller still links, but it never activates.
    (void)active; (void)palette16_rgb888; (void)pair_lut;
    profi_ds80_active = false;
#else
    if (active && palette16_rgb888 && pair_lut && vga_buffers_ready()) {
        // Dithered VGA pixel values for each of 16 Profi colors.
        uint8_t vga_even_left[16];   // even scan-line, left  pixel (even screen x)
        uint8_t vga_even_right[16];  // even scan-line, right pixel (odd screen x)
        uint8_t vga_odd_left[16];    // odd scan-line,  left  pixel
        uint8_t vga_odd_right[16];   // odd scan-line,  right pixel
        for (int i = 0; i < 16; i++) {
            uint16_t ep, op;
            vga_rgb888_dither(palette16_rgb888[i], &ep, &op);
            vga_even_left[i]  = ep & 0x3F;
            vga_even_right[i] = (ep >> 8) & 0x3F;
            vga_odd_left[i]   = op & 0x3F;
            vga_odd_right[i]  = (op >> 8) & 0x3F;
        }
        // Initialise all slots to (black, black) so unused/border slots are safe.
        uint16_t black_pair = palette16_mask;
        for (int s = 0; s < 256; s++) {
            palette_vga_ds80[0][s] = black_pair;
            palette_vga_ds80[1][s] = black_pair;
        }
        // Fill every (p0, p1) combination that has a valid slot.
        // written[] guard: for merged slots (paper=8 → paper=0 for ink≤5), the first
        // pair wins so paper=0's colour is used — matches hdmi_set_profi_ds80_mode().
        bool written[256] = { false };
        for (int p0 = 0; p0 < 16; p0++) {
            for (int p1 = 0; p1 < 16; p1++) {
                uint8_t slot = pair_lut[p0 * 16 + p1];
                if (written[slot]) continue;
                written[slot] = true;
                palette_vga_ds80[0][slot] = (((uint16_t)vga_even_right[p1] << 8) | vga_even_left[p0]) | palette16_mask;
                palette_vga_ds80[1][slot] = (((uint16_t)vga_odd_right[p1]  << 8) | vga_odd_left[p0])  | palette16_mask;
            }
        }
        profi_ds80_active = true;
    } else {
        profi_ds80_active = false;
    }
#endif // !PICO_RP2040
}

#else
void graphics_set_bgcolor_hdmi(uint32_t color888);
void graphics_set_bgcolor(const uint32_t color888) {
    graphics_set_bgcolor_hdmi(color888);
}
#endif // #if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
void graphics_set_bgcolor_hdmi(uint32_t color888);
void graphics_set_bgcolor(const uint32_t color888) {
    if (!SELECT_VGA) {
        graphics_set_bgcolor_hdmi(color888);
        return;
    }
    uint16_t p0, p1;
    vga_rgb888_dither(color888, &p0, &p1);
    bg_color[0] = (p0 << 16) | p0;
    bg_color[1] = (p1 << 16) | p1;
}

#ifndef VGA_HDMI
// Standalone VGA build: provide graphics_set_palette
void graphics_set_palette(const uint8_t i, const uint32_t color888) {
    vga_set_palette_entry(i, color888);
}
#endif

void vga_set_scanlines(uint8_t level) {
    if (!vga_buffers_ready()) return;          // HDMI active → VGA tables unused
    if (level > 4) level = 4;
    vga_scanlines = (level != 0);
    // Off keeps the previous brightness so toggling back is cheap; only a real
    // level change forces a dimmed-palette rebuild.
    if (level != 0 && level != vga_scanline_level) {
        vga_scanline_level = level;
        for (int i = 0; i < 256; ++i) vga_build_scanline_entry(i);
    }
}

#ifndef VGA_HDMI
void graphics_set_scanlines(uint8_t level) {
    vga_set_scanlines(level);
}
#endif

#else 
void graphics_init_hdmi();
void graphics_init() {
    graphics_init_hdmi();
}
#endif // defined(VGA_HDMI) && !defined(SUPPRESS_VGA)

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
uint8_t linkVGA01;
void graphics_init_hdmi();
void graphics_init() {
    if (video_driver == 0) {
        #if defined(ZERO) || defined(ZERO2) || defined(PICO_DV)
            SELECT_VGA = linkVGA01 == 0x1F;
        #else
            SELECT_VGA = (linkVGA01 == 0) || (linkVGA01 == 0x1F);
        #endif
    } else {
        SELECT_VGA = video_driver == 1;
    }
    if (!SELECT_VGA) {
        graphics_init_hdmi();
        return;
    }
    //инициализация палитры по умолчанию
    //текстовая палитра
    for (int i = 0; i < 16; i++) {
        const uint8_t b = i & 1 ? (i >> 3 ? 3 : 2) : 0;
        const uint8_t r = i & 4 ? (i >> 3 ? 3 : 2) : 0;
        const uint8_t g = i & 2 ? (i >> 3 ? 3 : 2) : 0;

        const uint8_t c = r << 4 | g << 2 | b;

        txt_palette[i] = c & 0x3f | 0xc0;
    }
    //инициализация PIO
    //загрузка программы в один из PIO
    const uint offset = pio_add_program(PIO_VGA, &pio_program_VGA);
    _SM_VGA = pio_claim_unused_sm(PIO_VGA, true);
    const uint sm = _SM_VGA;

    for (int i = 0; i < 8; i++) {
        gpio_init(VGA_BASE_PIN + i);
        gpio_set_dir(VGA_BASE_PIN + i, GPIO_OUT);
        pio_gpio_init(PIO_VGA, VGA_BASE_PIN + i);
    }; //резервируем под выход PIO

    pio_sm_set_consecutive_pindirs(PIO_VGA, sm, VGA_BASE_PIN, 8, true); //конфигурация пинов на выход

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + (pio_program_VGA.length - 1));

    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX); //увеличение буфера TX за счёт RX до 8-ми
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_out_pins(&c, VGA_BASE_PIN, 8);
    pio_sm_init(PIO_VGA, sm, offset, &c);

    pio_sm_set_enabled(PIO_VGA, sm, true);

    //инициализация DMA
    dma_chan_ctrl = dma_claim_unused_channel(true);
    dma_chan = dma_claim_unused_channel(true);
    //основной ДМА канал для данных
    dma_channel_config c0 = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32);

    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);

    uint dreq = DREQ_PIO1_TX0 + sm;
    if (PIO_VGA == pio0) dreq = DREQ_PIO0_TX0 + sm;

    channel_config_set_dreq(&c0, dreq);
    channel_config_set_chain_to(&c0, dma_chan_ctrl); // chain to other channel

    dma_channel_configure(
        dma_chan,
        &c0,
        &PIO_VGA->txf[sm], // Write address
        lines_pattern[0], // read address
        600 / 4, //
        false // Don't start yet
    );
    //канал DMA для контроля основного канала
    dma_channel_config c1 = dma_channel_get_default_config(dma_chan_ctrl);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);

    channel_config_set_read_increment(&c1, false);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_chain_to(&c1, dma_chan); // chain to other channel

    dma_channel_configure(
        dma_chan_ctrl,
        &c1,
        &dma_hw->ch[dma_chan].read_addr, // Write address
        &lines_pattern[0], // read address
        1, //
        false // Don't start yet
    );

    graphics_set_mode(GRAPHICSMODE_DEFAULT);
    irq_set_exclusive_handler(VGA_DMA_IRQ, dma_handler_VGA);
    dma_channel_set_irq0_enabled(dma_chan_ctrl, true);
    irq_set_enabled(VGA_DMA_IRQ, true);
    dma_start_channel_mask(1u << dma_chan);
}

#endif // defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
