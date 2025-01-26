/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "graphics.h"

#include <string.h>
#include <pico/multicore.h>

#include "st7789.pio.h"
#include "hardware/dma.h"

#ifndef SCREEN_WIDTH
#define SCREEN_WIDTH 320
#endif

#ifndef SCREEN_HEIGHT
#define SCREEN_HEIGHT 240
#endif

// 126MHz SPI
#define SERIAL_CLK_DIV 3.0f
#define MADCTL_BGR_PIXEL_ORDER (1<<3)
#define MADCTL_ROW_COLUMN_EXCHANGE (1<<5)
#define MADCTL_COLUMN_ADDRESS_ORDER_SWAP (1<<6)


#define CHECK_BIT(var, pos) (((var)>>(pos)) & 1)

static uint sm = 0;
static PIO pio = pio0;
static uint st7789_chan;

static uint16_t __scratch_x("tft_palette") palette[64];

static uint graphics_buffer_width = 0;
static uint graphics_buffer_height = 0;
static int graphics_buffer_shift_x = 0;
static int graphics_buffer_shift_y = 0;

enum graphics_mode_t graphics_mode = GRAPHICSMODE_DEFAULT;

static const uint8_t init_seq[] = {
    1, 20, 0x01, // Software reset
    1, 10, 0x11, // Exit sleep mode
    2, 2, 0x3a, 0x55, // Set colour mode to 16 bit
#ifdef ILI9341
    // ILI9341
    2, 0, 0x36, MADCTL_ROW_COLUMN_EXCHANGE | MADCTL_BGR_PIXEL_ORDER, // Set MADCTL
#else
    // ST7789
    2, 0, 0x36, MADCTL_COLUMN_ADDRESS_ORDER_SWAP | MADCTL_ROW_COLUMN_EXCHANGE, // Set MADCTL
#endif
    5, 0, 0x2a, 0x00, 0x00, SCREEN_WIDTH >> 8, SCREEN_WIDTH & 0xff, // CASET: column addresses
    5, 0, 0x2b, 0x00, 0x00, SCREEN_HEIGHT >> 8, SCREEN_HEIGHT & 0xff, // RASET: row addresses
    1, 2, 0x20, // Inversion OFF
    1, 2, 0x13, // Normal display on, then 10 ms delay
    1, 2, 0x29, // Main screen turn on, then wait 500 ms
    0 // Terminate list
};
// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little

static inline void lcd_set_dc_cs(const bool dc, const bool cs) {
    sleep_us(5);
    gpio_put_masked((1u << TFT_DC_PIN) | (1u << TFT_CS_PIN), !!dc << TFT_DC_PIN | !!cs << TFT_CS_PIN);
    sleep_us(5);
}

static inline void lcd_write_cmd(const uint8_t* cmd, size_t count) {
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    st7789_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        st7789_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st7789_lcd_put(pio, sm, *cmd++);
    }
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_set_window(const uint16_t x,
                                  const uint16_t y,
                                  const uint16_t width,
                                  const uint16_t height) {
    static uint8_t screen_width_cmd[] = { 0x2a, 0x00, 0x00, SCREEN_WIDTH >> 8, SCREEN_WIDTH & 0xff };
    static uint8_t screen_height_command[] = { 0x2b, 0x00, 0x00, SCREEN_HEIGHT >> 8, SCREEN_HEIGHT & 0xff };
    screen_width_cmd[2] = x;
    screen_width_cmd[4] = x + width - 1;

    screen_height_command[2] = y;
    screen_height_command[4] = y + height - 1;
    lcd_write_cmd(screen_width_cmd, 5);
    lcd_write_cmd(screen_height_command, 5);
}

static inline void lcd_init(const uint8_t* init_seq) {
    const uint8_t* cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void start_pixels() {
    const uint8_t cmd = 0x2c; // RAMWR
    st7789_lcd_wait_idle(pio, sm);
    st7789_set_pixel_mode(pio, sm, false);
    lcd_write_cmd(&cmd, 1);
    st7789_set_pixel_mode(pio, sm, true);
    lcd_set_dc_cs(1, 0);
}

void stop_pixels() {
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
    st7789_set_pixel_mode(pio, sm, false);
}

void create_dma_channel() {
    st7789_chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(st7789_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    dma_channel_configure(
        st7789_chan, // Channel to be configured
        &c, // The configuration we just created
        &pio->txf[sm], // The write address
        NULL, // The initial read address - set later
        0, // Number of transfers - set later
        false // Don't start yet
    );
}

///#define R(c) (c & 0b11)
///#define G(c) ((c & 0b1100) >> 2)
////#define B(c) ((c & 0b110000) >> 4)
//RRRRR GGGGGG BBBBB
///#define RGB888(c) ((R(c) << 14) | (G(c) << 9) | B(c) << 3)

//RRRR RGGG GGGB BBBB
#ifdef ILI9341
#define RGB888(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#else
#if TFT_INV
#define RGB888(r, g, b) ((((~r & 0xFF) >> 3) << 11) | (((~g & 0xFF) >> 2) << 5) | ((~b & 0xFF) >> 3))
#else
#define RGB888(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#endif
#endif

static const uint16_t textmode_palette_tft[17] = {
    //R, G, B
    RGB888(0x00, 0x00, 0x00), //black
    RGB888(0x00, 0x00, 0xC4), //blue
    RGB888(0x00, 0xC4, 0x00), //green
    RGB888(0x00, 0xC4, 0xC4), //cyan
    RGB888(0xC4, 0x00, 0x00), //red
    RGB888(0xC4, 0x00, 0xC4), //magenta
    RGB888(0xC4, 0x7E, 0x00), //brown
    RGB888(0xC4, 0xC4, 0xC4), //light gray
    RGB888(0xC4, 0xC4, 0x00), //yellow
    RGB888(0x4E, 0x4E, 0xDC), //light blue
    RGB888(0x4E, 0xDC, 0x4E), //light green
    RGB888(0x4E, 0xF3, 0xF3), //light cyan
    RGB888(0xDC, 0x4E, 0x4E), //light red
    RGB888(0xF3, 0x4E, 0xF3), //light magenta
    RGB888(0xF3, 0xF3, 0x4E), //light yellow
    RGB888(0xFF, 0xFF, 0xFF), //white
    RGB888(0xFF, 0x7E, 0x00) //orange
};

void graphics_init() {
    const uint offset = pio_add_program(pio, &st7789_lcd_program);
    sm = pio_claim_unused_sm(pio, true);
    st7789_lcd_program_init(pio, sm, offset, TFT_DATA_PIN, TFT_CLK_PIN, SERIAL_CLK_DIV);

    gpio_init(TFT_CS_PIN);
    gpio_init(TFT_DC_PIN);
    gpio_init(TFT_RST_PIN);
    gpio_init(TFT_LED_PIN);
    gpio_set_dir(TFT_CS_PIN, GPIO_OUT);
    gpio_set_dir(TFT_DC_PIN, GPIO_OUT);
    gpio_set_dir(TFT_RST_PIN, GPIO_OUT);
    gpio_set_dir(TFT_LED_PIN, GPIO_OUT);

    gpio_put(TFT_CS_PIN, 1);
    gpio_put(TFT_RST_PIN, 1);
    lcd_init(init_seq);
    gpio_put(TFT_LED_PIN, 1);

    for (uint8_t c = 0; c <= 0b00111111; ++c) {
        size_t idx = 0;
        switch (c)
        {
        case 0b000000: idx = 0; break; // black

        case 0b000001: idx = 4; break; // red
        case 0b000010: idx = 4; break;

        case 0b000011: idx = 12; break; // light red
        case 0b010011: idx = 12; break;

        case 0b000100: idx = 2; break; // green
        case 0b001000: idx = 2; break;
        case 0b001001: idx = 2; break;

        case 0b001100: idx = 10; break; // light green

        case 0b010000: idx = 1; break; // blue
        case 0b100000: idx = 1; break;

        case 0b110000: idx = 9; break; // light blue

        case 0b000101: idx = 8; break; // yellow
        case 0b000110: idx = 8; break;
        case 0b001010: idx = 8; break;
        case 0b001011: idx = 8; break;
        case 0b001110: idx = 8; break;

        case 0b001111: idx = 14; break; // light tellow

        case 0b010001: idx = 5; break; // magenta
        case 0b010010: idx = 5; break;
        case 0b100001: idx = 5; break;
        case 0b100010: idx = 5; break;
        case 0b110010: idx = 5; break;
        case 0b100011: idx = 5; break;

        case 0b110011: idx = 13; break; // light magenta

        case 0b010100: idx = 3; break; // cyan
        case 0b100100: idx = 3; break;
        case 0b011000: idx = 3; break;
        case 0b101000: idx = 3; break;
        case 0b111000: idx = 3; break;
        case 0b101100: idx = 3; break;

        case 0b111100: idx = 11; break; // light cyan

        case 0b010101: idx = 7; break; // gray
        case 0b010110: idx = 7; break;
        case 0b100101: idx = 7; break;
        case 0b100110: idx = 7; break;
        case 0b010111: idx = 7; break;
        case 0b011001: idx = 7; break;
        case 0b011111: idx = 7; break;
        case 0b111001: idx = 7; break;
        case 0b111010: idx = 7; break;
        case 0b101001: idx = 7; break;
        case 0b101010: idx = 7; break;

        case 0b111111: idx = 15; break; // white

        case 0b000111: idx = 16; break; // orange

        default: idx = 15; break;
        }
        palette[c] = textmode_palette_tft[idx];
    }
    clrScr(0);

    create_dma_channel();
}

void inline graphics_set_mode(const enum graphics_mode_t mode) {
/**
    graphics_mode = -1;
    sleep_ms(16);
    clrScr(0);
    graphics_mode = mode;
*/
}

void graphics_set_buffer(uint8_t* buffer, const uint16_t width, const uint16_t height) {
    graphics_buffer_width = width;
    graphics_buffer_height = height;
}

void graphics_set_offset(const int x, const int y) {
    graphics_buffer_shift_x = x;
    graphics_buffer_shift_y = y;
}

void clrScr(const uint8_t color) {
    lcd_set_window(0, 0,SCREEN_WIDTH,SCREEN_HEIGHT);
    uint32_t i = SCREEN_WIDTH * SCREEN_HEIGHT;
    start_pixels();
    while (--i) {
        st7789_lcd_put_pixel(pio, sm, 0x0000);
    }
    stop_pixels();
}

void st7789_dma_pixels(const uint16_t* pixels, const uint num_pixels) {
    // Ensure any previous transfer is finished.
    dma_channel_wait_for_finish_blocking(st7789_chan);

    dma_channel_hw_addr(st7789_chan)->read_addr = (uintptr_t)pixels;
    dma_channel_hw_addr(st7789_chan)->transfer_count = num_pixels;
    const uint ctrl = dma_channel_hw_addr(st7789_chan)->ctrl_trig;
    dma_channel_hw_addr(st7789_chan)->ctrl_trig = ctrl | DMA_CH0_CTRL_TRIG_INCR_READ_BITS;
}

uint8_t* getLineBuffer(int line);
void ESPectrum_vsync();

void __inline __scratch_x("refresh_lcd") refresh_lcd() {
    ESPectrum_vsync();
    switch (graphics_mode) {
        case GRAPHICSMODE_DEFAULT: {
            lcd_set_window(graphics_buffer_shift_x, graphics_buffer_shift_y, graphics_buffer_width,
                           graphics_buffer_height);
            start_pixels();
            for (register size_t y = 0; y < graphics_buffer_height; ++y) {
                register uint8_t* bitmap = getLineBuffer(y);
                if (!bitmap) continue;
                for (register size_t x = 0; x < graphics_buffer_width; ++x) {
                    register uint8_t c = bitmap[x ^ 2];
                    st7789_lcd_put_pixel(pio, sm, palette[c & 0b111111]);
                }
            }
            stop_pixels();
        }
    }
}
