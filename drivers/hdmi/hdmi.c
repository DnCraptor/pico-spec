#include "graphics.h"
#include <stdio.h>
#include <string.h>
#include "malloc.h"
#include <stdalign.h>
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"

//PIO параметры
static uint offs_prg0 = 0;
static uint offs_prg1 = 0;

//SM
static int SM_video = -1;
static int SM_conv = -1;

//активный видеорежим
extern enum graphics_mode_t graphics_mode;

// Scanlines mode: when enabled, every other physical line is dark
static bool hdmi_scanlines = false;

//буфер  палитры 256 цветов в формате R8G8B8
static uint32_t palette[256];

// SCREEN_WIDTH is now dynamic: mode.screen_width (320 for 640x480, 360 for 720x576)

// #define HDMI_WIDTH 480 //480 Default
// #define HDMI_HEIGHT 644 //524 Default
// #define HDMI_HZ 52 //60 Default

extern int graphics_buffer_width, graphics_buffer_height, graphics_buffer_shift_x, graphics_buffer_shift_y;

//DMA каналы
//каналы работы с первичным графическим буфером
static int dma_chan_ctrl;
static int dma_chan;
//каналы работы с конвертацией палитры
static int dma_chan_pal_conv_ctrl;
static int dma_chan_pal_conv;

//DMA буферы
//основные строчные данные
static uint32_t* __scratch_x("hdmi_ptr_3") dma_lines[2] = { NULL,NULL };
static uint32_t* __scratch_x("hdmi_ptr_4") DMA_BUF_ADDR[3];

// Pre-filled scanline buffer (dark line with valid HDMI sync), used as a
// third "virtual" ping-pong slot when scanlines mode is on. DMA reads it
// directly — no per-IRQ rendering. Why: prevents h-sync jitter on strict
// receivers that rejected the v1.2.13/14 dynamic approaches.
static uint8_t hdmi_scanline_buf[400];

//ДМА палитра для конвертации
//в хвосте этой памяти выделяется dma_data
static alignas(4096) uint32_t conv_color[1240];
// map64colors removed — frame buffer now stores direct 8-bit palette indices

//индекс, проверяющий зависание
static uint32_t irq_inx = 0;

//функции и константы HDMI

#define BASE_HDMI_CTRL_INX (240)
#define IDX_SCANLINE        (244)   // dark gray for scanline effect

// Data Island palette indices (for HDMI audio, below control words to avoid overlap)
#define IDX_DI_PREAMBLE     (220)
#define IDX_DI_GUARD        (221)
#define IDX_DI_DATA_BASE    (222)   // 222..237 (16 entries for 32 pixel clocks)

#if !PICO_RP2040
// HDMI audio state
#define HDMI_AUDIO_RING_SIZE 1024
#define HDMI_AUDIO_RING_MASK (HDMI_AUDIO_RING_SIZE - 1)
static volatile int16_t hdmi_audio_ring_L[HDMI_AUDIO_RING_SIZE];
static volatile int16_t hdmi_audio_ring_R[HDMI_AUDIO_RING_SIZE];
static volatile uint32_t hdmi_audio_wr = 0;
static volatile uint32_t hdmi_audio_rd = 0;
static bool hdmi_audio_enabled = false;

// Pre-computed conv_color data for DI packets (32 uint64_t = 16 entry pairs each)
// These are ready to memcpy into conv_color[IDX_DI_DATA_BASE] in ISR
static uint64_t acr_cc[32];         // ACR packet conv_color entries (static, computed once)
static uint64_t infoframe_cc[32];   // InfoFrame conv_color entries (static, computed once)
static uint64_t audio_pkt_cc[32];   // Audio sample packet (double-buffered, produced by pcm_call)
static volatile bool audio_pkt_ready = false;  // set by producer, cleared by ISR consumer

static uint64_t preamble_entry[2];
static uint64_t guard_entry[2];

// Forward declarations
static void __attribute__((noinline)) hdmi_audio_hw_init(void);
static void hdmi_fill_di_indices_bp(uint8_t *line_buf, int h_sync_bytes, int h_bp_bytes);
static void hdmi_fill_di_indices_vblank(uint8_t *line_buf, int h_sync_bytes, int line_bytes);
#endif

//программа конвертации адреса

uint16_t pio_program_instructions_conv_HDMI[] = {
    //         //     .wrap_target
    0x80a0, //  0: pull   block
    0x40e8, //  1: in     osr, 8
    0x4034, //  2: in     x, 20
    0x8020, //  3: push   block
    //     .wrap
};


const struct pio_program pio_program_conv_addr_HDMI = {
    .instructions = pio_program_instructions_conv_HDMI,
    .length = 4,
    .origin = -1,
};

//программа видеовывода
static const uint16_t instructions_PIO_HDMI[] = {
    0x7006, //  0: out    pins, 6         side 2
    0x7006, //  1: out    pins, 6         side 2
    0x7006, //  2: out    pins, 6         side 2
    0x7006, //  3: out    pins, 6         side 2
    0x7006, //  4: out    pins, 6         side 2
    0x6806, //  5: out    pins, 6         side 1
    0x6806, //  6: out    pins, 6         side 1
    0x6806, //  7: out    pins, 6         side 1
    0x6806, //  8: out    pins, 6         side 1
    0x6806, //  9: out    pins, 6         side 1
};

static const struct pio_program program_PIO_HDMI = {
    .instructions = instructions_PIO_HDMI,
    .length = 10,
    .origin = -1,
};

static uint64_t get_ser_diff_data(const uint16_t dataR, const uint16_t dataG, const uint16_t dataB) {
    uint64_t out64 = 0;
    for (int i = 0; i < 10; i++) {
        out64 <<= 6;
        if (i == 5) out64 <<= 2;
#ifdef PICO_PC
        uint8_t bG = (dataR >> (9 - i)) & 1;
        uint8_t bR = (dataG >> (9 - i)) & 1;
#else
        uint8_t bR = (dataR >> (9 - i)) & 1;
        uint8_t bG = (dataG >> (9 - i)) & 1;
#endif
        uint8_t bB = (dataB >> (9 - i)) & 1;

        bR |= (bR ^ 1) << 1;
        bG |= (bG ^ 1) << 1;
        bB |= (bB ^ 1) << 1;

        if (HDMI_PIN_invert_diffpairs) {
            bR ^= 0b11;
            bG ^= 0b11;
            bB ^= 0b11;
        }
        uint8_t d6;
        if (HDMI_PIN_RGB_notBGR) {
            d6 = (bR << 4) | (bG << 2) | (bB << 0);
        }
        else {
            d6 = (bB << 4) | (bG << 2) | (bR << 0);
        }


        out64 |= d6;
    }
    return out64;
}

//конвертор TMDS
static uint tmds_encoder(const uint8_t d8) {
    int s1 = 0;
    for (int i = 0; i < 8; i++) s1 += (d8 & (1 << i)) ? 1 : 0;
    bool is_xnor = false;
    if ((s1 > 4) || ((s1 == 4) && ((d8 & 1) == 0))) is_xnor = true;
    uint16_t d_out = d8 & 1;
    uint16_t qi = d_out;
    for (int i = 1; i < 8; i++) {
        d_out |= ((qi << 1) ^ (d8 & (1 << i))) ^ (is_xnor << i);
        qi = d_out & (1 << i);
    }

    if (is_xnor) d_out |= 1 << 9;
    else d_out |= 1 << 8;

    return d_out;
}

static void pio_set_x(PIO pio, const int sm, uint32_t v) {
    uint instr_shift = pio_encode_in(pio_x, 4);
    uint instr_mov = pio_encode_mov(pio_x, pio_isr);
    for (int i = 0; i < 8; i++) {
        const uint32_t nibble = (v >> (i * 4)) & 0xf;
        pio_sm_exec(pio, sm, pio_encode_set(pio_x, nibble));
        pio_sm_exec(pio, sm, instr_shift);
    }
    pio_sm_exec(pio, sm, instr_mov);
}

uint8_t* getLineBuffer(int line);
void ESPectrum_vsync();
int get_video_mode();

static inline void* __not_in_flash_func(nf_memset)(void* ptr, int value, size_t len)
{
    uint8_t* p = (uint8_t*)ptr;
    uint8_t v8 = (uint8_t)value;

    // --- выравниваем до 4 байт ---
    while (len && ((uintptr_t)p & 3)) {
        *p++ = v8;
        len--;
    }

    // --- основной 32-битный цикл ---
    if (len >= 4) {
        uint32_t v32 = v8;
        v32 |= v32 << 8;
        v32 |= v32 << 16;

        uint32_t* p32 = (uint32_t*)p;
        size_t n32 = len >> 2;

        while (n32--) {
            *p32++ = v32;
        }

        p = (uint8_t*)p32;
        len &= 3;
    }

    // --- хвост ---
    while (len--) {
        *p++ = v8;
    }

    return ptr;
}

static void __scratch_x("hdmi_driver") dma_handler_HDMI() {
    static uint32_t inx_buf_dma;
    static uint line = 0;
    struct video_mode_t mode = graphics_get_video_mode(get_video_mode());
    irq_inx++;

    dma_hw->ints0 = 1u << dma_chan_ctrl;

    // Решаем источник для следующей строки (line+1) до её начала, чтобы
    // ctrl-канал был перезаряжен ровно один раз. Подмена адреса задним
    // числом, как в v1.2.13, давала срыв синхры на чувствительных приёмниках.
    uint next_line = (line >= mode.v_total) ? 1 : (line + 1);
    bool next_is_scanline = hdmi_scanlines
                         && (next_line <= mode.v_active)
                         && !(next_line & 1);
    if (next_is_scanline) {
        // scanline-строка играет из статичного буфера, ping-pong не трогаем
        dma_channel_set_read_addr(dma_chan_ctrl, &DMA_BUF_ADDR[2], false);
    } else {
        dma_channel_set_read_addr(dma_chan_ctrl, &DMA_BUF_ADDR[inx_buf_dma & 1], false);
    }

    if (line >= mode.v_total ) {
        line = 0;
    } else {
        ++line;
    }

    // Сигнализируем vsync в начале blanking-периода (после последней видимой строки),
    // чтобы эмулятор рендерил следующий кадр во время blanking,
    // пока HDMI не читает frameBuffer — предотвращает тиринг в верхней части экрана
    if (line == mode.v_active) {
        ESPectrum_vsync();
    }

    // Pixel-doubling: рендерим один раз на пару строк.
    // Без scanlines — рендер на нечётных (исторически).
    // Со scanlines — нечётная активная строка играет статический серый
    // буфер (адрес уже перезаряжен на прошлом IRQ), чётная — рендерится из FB.
    // Граничная line == v_active (чётная) — это первая строка blanking;
    // её надо пропустить, иначе получаем лишнюю активную строку (482).
    if (line < mode.v_active) {
        if (hdmi_scanlines) {
            if (line & 1) return;            // нечётные = серая, ничего не пишем
        } else {
            if (!(line & 1)) return;         // чётные пропускаются (доигрывают пред. буфер)
        }
    } else {
        if (!(line & 1)) return;             // в blanking пропускаем чётные (включая v_active)
    }
    inx_buf_dma++;

    uint8_t* activ_buf = (uint8_t *)dma_lines[inx_buf_dma & 1];

    const int h_sync = mode.h_sync_bytes;
    const int h_bp = mode.h_bp_bytes;
    const int h_fp = mode.h_fp_bytes;
    const int scr_w = mode.screen_width;
    const int line_sz = mode.line_bytes;

    if (line < mode.v_active ) {
        uint8_t* output_buffer = activ_buf + h_sync + h_bp;
        int y = (line >> 1) + mode.v_offset;
        //область изображения
        uint8_t* input_buffer = getLineBuffer(y);
        if (!input_buffer) return;
        // заполняем пространство сверху и снизу графического буфера
        if (false || (graphics_buffer_shift_y > y) || (y >= (graphics_buffer_shift_y + graphics_buffer_height))
            || (graphics_buffer_shift_x >= scr_w) || (
                (graphics_buffer_shift_x + graphics_buffer_width) < 0)) {
            nf_memset(output_buffer, 255, scr_w);
            goto ex;
        }

        uint8_t* activ_buf_end = output_buffer + scr_w;
        // рисуем пространство слева от буфера
        for (int i = graphics_buffer_shift_x; i-- > 0;) {
            *output_buffer++ = 255;
        }

        // рисуем сам видеобуфер+пространство справа
        const uint8_t* input_buffer_end = input_buffer + graphics_buffer_width;
        if (graphics_buffer_shift_x < 0) input_buffer -= graphics_buffer_shift_x;
        register size_t x = 0;
        while (activ_buf_end > output_buffer) {
            if (input_buffer < input_buffer_end) {
                // Direct 8-bit palette index — no mask or lookup needed
                *output_buffer++ = input_buffer[(x++) ^ 2];
            }
            else
                *output_buffer++ = 255;
        }
ex:

        //ССИ — горизонтальная синхронизация
        nf_memset(activ_buf + h_sync, BASE_HDMI_CTRL_INX, h_bp);
        nf_memset(activ_buf, BASE_HDMI_CTRL_INX + 1, h_sync);
        nf_memset(activ_buf + line_sz - h_fp, BASE_HDMI_CTRL_INX, h_fp);

        // Audio sample packets sent in vblank only (h_bp too tight for active lines)
    }
    else {
        int blanking_rest = line_sz - h_sync;
        if ((line >= mode.vsync_start) && (line < mode.vsync_end)) {
            //кадровый синхроимпульс
            nf_memset(activ_buf + h_sync, BASE_HDMI_CTRL_INX + 2, blanking_rest);
            nf_memset(activ_buf, BASE_HDMI_CTRL_INX + 3, h_sync);
        }
        else {
            //ССИ без изображения
            nf_memset(activ_buf + h_sync, BASE_HDMI_CTRL_INX, blanking_rest);
            nf_memset(activ_buf, BASE_HDMI_CTRL_INX + 1, h_sync);

#if !PICO_RP2040
            if (hdmi_audio_enabled) {
                // DI packets on specific vblank lines (ISR fires on odd lines only)
                uint32_t vbl_line = line - mode.v_active;  // offset from start of vblank
                uint64_t *src = NULL;
                if (vbl_line == 3) {
                    src = acr_cc;
                } else if (vbl_line == 5) {
                    src = infoframe_cc;
                } else if (vbl_line == 7 && audio_pkt_ready) {
                    src = audio_pkt_cc;
                    audio_pkt_ready = false;
                }
                if (src) {
                    uint64_t *cc64 = (uint64_t *)conv_color;
                    for (int i = 0; i < 16; i++) {
                        cc64[(IDX_DI_DATA_BASE + i) * 2]     = src[i * 2];
                        cc64[(IDX_DI_DATA_BASE + i) * 2 + 1] = src[i * 2 + 1];
                    }
                    hdmi_fill_di_indices_vblank(activ_buf, h_sync, line_sz);
                }
            }
#endif
        };
    }
}


static inline void irq_remove_handler_DMA_core1() {
    irq_set_enabled(VIDEO_DMA_IRQ, false);
    irq_remove_handler(VIDEO_DMA_IRQ, irq_get_exclusive_handler(VIDEO_DMA_IRQ));
}

static inline void irq_set_exclusive_handler_DMA_core1() {
    irq_set_exclusive_handler(VIDEO_DMA_IRQ, dma_handler_HDMI);
    irq_set_priority(VIDEO_DMA_IRQ, 0);
    irq_set_enabled(VIDEO_DMA_IRQ, true);
}

//деинициализация - инициализация ресурсов
static inline bool hdmi_init() {
    //выключение прерывания DMA
    if (VIDEO_DMA_IRQ == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(dma_chan_ctrl, false);
    }
    else {
        dma_channel_set_irq1_enabled(dma_chan_ctrl, false);
    }

    irq_remove_handler_DMA_core1();


    //остановка всех каналов DMA
    dma_hw->abort = (1 << dma_chan_ctrl) | (1 << dma_chan) | (1 << dma_chan_pal_conv) | (
                        1 << dma_chan_pal_conv_ctrl);
    while (dma_hw->abort) tight_loop_contents();

    //выключение SM основной и конвертора

#if ZERO2
    pio_set_gpio_base(PIO_VIDEO, 16);
    pio_set_gpio_base(PIO_VIDEO_ADDR, 16);
#endif

    // pio_sm_restart(PIO_VIDEO, SM_video);
    pio_sm_set_enabled(PIO_VIDEO, SM_video, false);

    //pio_sm_restart(PIO_VIDEO_ADDR, SM_conv);
    pio_sm_set_enabled(PIO_VIDEO_ADDR, SM_conv, false);


    //удаление программ из соответствующих PIO
    pio_remove_program(PIO_VIDEO_ADDR, &pio_program_conv_addr_HDMI, offs_prg1);
    pio_remove_program(PIO_VIDEO, &program_PIO_HDMI, offs_prg0);


    offs_prg1 = pio_add_program(PIO_VIDEO_ADDR, &pio_program_conv_addr_HDMI);
    offs_prg0 = pio_add_program(PIO_VIDEO, &program_PIO_HDMI);
    pio_set_x(PIO_VIDEO_ADDR, SM_conv, ((uint32_t)conv_color >> 12));

    //заполнение палитры
    for (int ci = 0; ci < 240; ci++) graphics_set_palette(ci, palette[ci]); //

    //255 - цвет фона
    graphics_set_palette(255, palette[255]);

    // Scanline color: dark gray RGB888
    graphics_set_palette(IDX_SCANLINE, 0x202020);

    //240-243 служебные данные(синхра) напрямую вносим в массив -конвертер
    uint64_t* conv_color64 = (uint64_t *)conv_color;
    const uint16_t b0 = 0b1101010100;
    const uint16_t b1 = 0b0010101011;
    const uint16_t b2 = 0b0101010100;
    const uint16_t b3 = 0b1010101011;
    const int base_inx = BASE_HDMI_CTRL_INX;

    conv_color64[2 * base_inx + 0] = get_ser_diff_data(b0, b0, b3);
    conv_color64[2 * base_inx + 1] = get_ser_diff_data(b0, b0, b3);

    conv_color64[2 * (base_inx + 1) + 0] = get_ser_diff_data(b0, b0, b2);
    conv_color64[2 * (base_inx + 1) + 1] = get_ser_diff_data(b0, b0, b2);

    conv_color64[2 * (base_inx + 2) + 0] = get_ser_diff_data(b0, b0, b1);
    conv_color64[2 * (base_inx + 2) + 1] = get_ser_diff_data(b0, b0, b1);

    conv_color64[2 * (base_inx + 3) + 0] = get_ser_diff_data(b0, b0, b0);
    conv_color64[2 * (base_inx + 3) + 1] = get_ser_diff_data(b0, b0, b0);

    //настройка PIO SM для конвертации

    pio_sm_config c_c = pio_get_default_sm_config();
    sm_config_set_wrap(&c_c, offs_prg1, offs_prg1 + (pio_program_conv_addr_HDMI.length - 1));
    sm_config_set_in_shift(&c_c, true, false, 32);

    pio_sm_init(PIO_VIDEO_ADDR, SM_conv, offs_prg1, &c_c);
    pio_sm_set_enabled(PIO_VIDEO_ADDR, SM_conv, true);

    //настройка PIO SM для вывода данных
    c_c = pio_get_default_sm_config();
    sm_config_set_wrap(&c_c, offs_prg0, offs_prg0 + (program_PIO_HDMI.length - 1));

    //настройка side set
    sm_config_set_sideset_pins(&c_c,beginHDMI_PIN_clk);
    sm_config_set_sideset(&c_c, 2,false,false);
    for (int i = 0; i < 2; i++) {
        pio_gpio_init(PIO_VIDEO, beginHDMI_PIN_clk + i);
        gpio_set_drive_strength(beginHDMI_PIN_clk + i, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(beginHDMI_PIN_clk + i, GPIO_SLEW_RATE_FAST);
    }

#if ZERO2
    // Настройка направлений пинов для state machines
    pio_sm_set_consecutive_pindirs(PIO_VIDEO, SM_video, HDMI_BASE_PIN, 8, true);
    pio_sm_set_consecutive_pindirs(PIO_VIDEO_ADDR, SM_conv, HDMI_BASE_PIN, 8, true);

    uint64_t mask64 = (uint64_t)(3u << beginHDMI_PIN_clk);
    pio_sm_set_pins_with_mask64(PIO_VIDEO, SM_video, mask64, mask64);
    pio_sm_set_pindirs_with_mask64(PIO_VIDEO, SM_video, mask64, mask64);
    // пины
#else
    pio_sm_set_pins_with_mask(PIO_VIDEO, SM_video, 3u << beginHDMI_PIN_clk, 3u << beginHDMI_PIN_clk);
    pio_sm_set_pindirs_with_mask(PIO_VIDEO, SM_video, 3u << beginHDMI_PIN_clk, 3u << beginHDMI_PIN_clk);
    // пины
#endif

    for (int i = 0; i < 6; i++) {
        gpio_set_slew_rate(beginHDMI_PIN_data + i, GPIO_SLEW_RATE_FAST);
        pio_gpio_init(PIO_VIDEO, beginHDMI_PIN_data + i);
        gpio_set_drive_strength(beginHDMI_PIN_data + i, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(beginHDMI_PIN_data + i, GPIO_SLEW_RATE_FAST);
    }
    pio_sm_set_consecutive_pindirs(PIO_VIDEO, SM_video, beginHDMI_PIN_data, 6, true);
    //конфигурация пинов на выход
    sm_config_set_out_pins(&c_c, beginHDMI_PIN_data, 6);

    //
    sm_config_set_out_shift(&c_c, true, true, 30);
    sm_config_set_fifo_join(&c_c, PIO_FIFO_JOIN_TX);

    struct video_mode_t hdmi_mode = graphics_get_video_mode(get_video_mode());
    // Use pre-computed clean divider (integer or half-integer) to avoid PIO clock jitter
    sm_config_set_clkdiv(&c_c, hdmi_mode.pio_clk_div);
    pio_sm_init(PIO_VIDEO, SM_video, offs_prg0, &c_c);
    pio_sm_set_enabled(PIO_VIDEO, SM_video, true);

    //настройки DMA
    int line_u32 = hdmi_mode.line_bytes / 4; // uint32_t per line buffer
    dma_lines[0] = &conv_color[1024];
    dma_lines[1] = &conv_color[1024 + line_u32];

    //основной рабочий канал
    dma_channel_config cfg_dma = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_8);
    channel_config_set_chain_to(&cfg_dma, dma_chan_ctrl); // chain to other channel

    channel_config_set_read_increment(&cfg_dma, true);
    channel_config_set_write_increment(&cfg_dma, false);


    uint dreq = DREQ_PIO1_TX0 + SM_conv;
    if (PIO_VIDEO_ADDR == pio0) dreq = DREQ_PIO0_TX0 + SM_conv;

    channel_config_set_dreq(&cfg_dma, dreq);

    dma_channel_configure(
        dma_chan,
        &cfg_dma,
        &PIO_VIDEO_ADDR->txf[SM_conv], // Write address
        &dma_lines[0][0], // read address
        hdmi_mode.line_bytes, //
        false // Don't start yet
    );

    //контрольный канал для основного
    cfg_dma = dma_channel_get_default_config(dma_chan_ctrl);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_32);
    channel_config_set_chain_to(&cfg_dma, dma_chan); // chain to other channel

    channel_config_set_read_increment(&cfg_dma, false);
    channel_config_set_write_increment(&cfg_dma, false);

    DMA_BUF_ADDR[0] = &dma_lines[0][0];
    DMA_BUF_ADDR[1] = &dma_lines[1][0];
    DMA_BUF_ADDR[2] = (uint32_t*)hdmi_scanline_buf;

    // Pre-fill scanline buffer: dark gray content + valid HDMI sync.
    // DMA reads this directly on every scanline-affected line; no per-line
    // rendering in the IRQ.
    {
        const int ls = hdmi_mode.line_bytes;
        const int hs = hdmi_mode.h_sync_bytes;
        const int bp = hdmi_mode.h_bp_bytes;
        const int fp = hdmi_mode.h_fp_bytes;
        const int sw = hdmi_mode.screen_width;
        nf_memset(hdmi_scanline_buf, BASE_HDMI_CTRL_INX + 1, hs);          // hsync
        nf_memset(hdmi_scanline_buf + hs, BASE_HDMI_CTRL_INX, bp);         // back porch
        nf_memset(hdmi_scanline_buf + hs + bp, IDX_SCANLINE, sw);          // dark gray content
        nf_memset(hdmi_scanline_buf + ls - fp, BASE_HDMI_CTRL_INX, fp);    // front porch
    }

    dma_channel_configure(
        dma_chan_ctrl,
        &cfg_dma,
        &dma_hw->ch[dma_chan].read_addr, // Write address
        &DMA_BUF_ADDR[0], // read address
        1, //
        false // Don't start yet
    );

    //канал - конвертер палитры

    cfg_dma = dma_channel_get_default_config(dma_chan_pal_conv);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_32);
    channel_config_set_chain_to(&cfg_dma, dma_chan_pal_conv_ctrl); // chain to other channel

    channel_config_set_read_increment(&cfg_dma, true);
    channel_config_set_write_increment(&cfg_dma, false);

    dreq = DREQ_PIO1_TX0 + SM_video;
    if (PIO_VIDEO == pio0) dreq = DREQ_PIO0_TX0 + SM_video;

    channel_config_set_dreq(&cfg_dma, dreq);

    dma_channel_configure(
        dma_chan_pal_conv,
        &cfg_dma,
        &PIO_VIDEO->txf[SM_video], // Write address
        &conv_color[0], // read address
        4, //
        false // Don't start yet
    );

    //канал управления конвертером палитры

    cfg_dma = dma_channel_get_default_config(dma_chan_pal_conv_ctrl);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_32);
    channel_config_set_chain_to(&cfg_dma, dma_chan_pal_conv); // chain to other channel

    channel_config_set_read_increment(&cfg_dma, false);
    channel_config_set_write_increment(&cfg_dma, false);

    dreq = DREQ_PIO1_RX0 + SM_conv;
    if (PIO_VIDEO_ADDR == pio0) dreq = DREQ_PIO0_RX0 + SM_conv;

    channel_config_set_dreq(&cfg_dma, dreq);

    dma_channel_configure(
        dma_chan_pal_conv_ctrl,
        &cfg_dma,
        &dma_hw->ch[dma_chan_pal_conv].read_addr, // Write address
        &PIO_VIDEO_ADDR->rxf[SM_conv], // read address
        1, //
        true // start yet
    );

    //стартуем прерывание и канал
    if (VIDEO_DMA_IRQ == DMA_IRQ_0) {
        dma_channel_acknowledge_irq0(dma_chan_ctrl);
        dma_channel_set_irq0_enabled(dma_chan_ctrl, true);
    }
    else {
        dma_channel_acknowledge_irq1(dma_chan_ctrl);
        dma_channel_set_irq1_enabled(dma_chan_ctrl, true);
    }

    irq_set_exclusive_handler_DMA_core1();

    dma_start_channel_mask((1u << dma_chan_ctrl));

    return true;
};

// In VGA_HDMI mode, also update the VGA palette LUT
#ifdef VGA_HDMI
extern void vga_set_palette_entry(uint8_t i, uint32_t color888);
#endif

// Cross-core reinit: core0 sets flag, core1 executes hdmi_init()
static volatile bool hdmi_reinit_pending = false;
static volatile bool hdmi_reinit_done = false;

void hdmi_reinit() {
    // Stop DMA channels from core0 so the DMA IRQ stops firing.
    // This frees core1 from back-to-back ISR calls, allowing its
    // main loop to reach hdmi_poll_reinit().
    dma_hw->abort = (1u << dma_chan_ctrl) | (1u << dma_chan)
                  | (1u << dma_chan_pal_conv) | (1u << dma_chan_pal_conv_ctrl);
    while (dma_hw->abort) tight_loop_contents();

    // Signal core1 to do the reinit (IRQ handler must be registered on core1)
    hdmi_reinit_done = false;
    __dmb();
    hdmi_reinit_pending = true;
    __sev();
    // Wait for core1 to complete
    while (!hdmi_reinit_done) {
        tight_loop_contents();
    }
}

void hdmi_poll_reinit() {
    if (hdmi_reinit_pending) {
        hdmi_reinit_pending = false;
        hdmi_init();
        __dmb();
        hdmi_reinit_done = true;
    }
}

void graphics_set_palette(uint8_t i, uint32_t color888) {
    palette[i] = color888 & 0x00ffffff;

#ifdef VGA_HDMI
    // Update VGA palette LUT in parallel
    vga_set_palette_entry(i, color888);
#endif

    if ((i >= BASE_HDMI_CTRL_INX) && (i != 255) && (i != IDX_SCANLINE)) return; //не записываем "служебные" цвета
#if !PICO_RP2040
    if (hdmi_audio_enabled && i >= IDX_DI_PREAMBLE && i <= (IDX_DI_DATA_BASE + 15)) return;
#endif

    uint64_t* conv_color64 = (uint64_t *)conv_color;
    const uint8_t R = (color888 >> 16) & 0xff;
    const uint8_t G = (color888 >> 8) & 0xff;
    const uint8_t B = (color888 >> 0) & 0xff;
    conv_color64[i * 2] = get_ser_diff_data(tmds_encoder(R), tmds_encoder(G), tmds_encoder(B));
    conv_color64[i * 2 + 1] = conv_color64[i * 2] ^ 0x0003ffffffffffffl;
};

#define RGB888(r, g, b) ((r<<16) | (g << 8 ) | b )

void graphics_init_hdmi() {
    // PIO и DMA
    SM_video = pio_claim_unused_sm(PIO_VIDEO, true);
    SM_conv = pio_claim_unused_sm(PIO_VIDEO_ADDR, true);
    dma_chan_ctrl = dma_claim_unused_channel(true);
    dma_chan = dma_claim_unused_channel(true);
    dma_chan_pal_conv_ctrl = dma_claim_unused_channel(true);
    dma_chan_pal_conv = dma_claim_unused_channel(true);

    // Palette is initialized centrally by Video.cpp Init()
    hdmi_init();

#if !PICO_RP2040
    if (hdmi_audio_enabled) {
        hdmi_audio_hw_init();
    }
#endif
}

void graphics_set_bgcolor_hdmi(uint32_t color888) //определяем зарезервированный цвет в палитре
{
    graphics_set_palette(255, color888);
};

void hdmi_set_scanlines(bool enabled) {
    hdmi_scanlines = enabled;
}

// ============================================================
// HDMI Audio — Data Island encoding (RP2350 only)
// ============================================================
#if !PICO_RP2040

// TERC4 encoding table: 4-bit value → 10-bit TMDS-like codeword (HDMI 1.3 spec Table 5-3)
static const uint16_t terc4_table[16] = {
    0x29C, 0x263, 0x2E4, 0x2E2, 0x171, 0x11E, 0x18E, 0x13C,
    0x2CC, 0x139, 0x19B, 0x26B, 0x164, 0x1D6, 0x0D4, 0x133
};

// BCH ECC for HDMI packets (polynomial x^8+x^4+x^3+x^2+1 = 0x11D)
static uint8_t hdmi_bch_ecc(const uint8_t *data, int len) {
    uint8_t ecc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; bit++) {
            uint8_t fb = ((ecc >> 7) ^ (byte & 1));
            ecc <<= 1;
            if (fb) ecc ^= 0x1D;
            byte >>= 1;
        }
    }
    return ecc;
}

// ACR parameters: N=4096 for 32kHz, CTS = TMDS_clk * N / (128 * Fs)
#define HDMI_ACR_N   4096
#define HDMI_ACR_CTS 25805

// Encode one Data Island pixel clock
static inline uint64_t hdmi_di_pixel(uint8_t ch0_4bit, uint8_t ch1_4bit, uint8_t ch2_4bit) {
    return get_ser_diff_data(terc4_table[ch2_4bit], terc4_table[ch1_4bit], terc4_table[ch0_4bit]);
}

// Build a complete Data Island packet into 32 uint64_t (16 conv_color entry pairs)
static void hdmi_build_packet(uint64_t *out, const uint8_t header[4],
                              const uint8_t subpkt[4][8], uint8_t hsync) {
    uint32_t hdr_bits = header[0] | (header[1] << 8) | (header[2] << 16) | (header[3] << 24);

    uint64_t sp_bits[4];
    for (int s = 0; s < 4; s++) {
        sp_bits[s] = 0;
        for (int b = 0; b < 8; b++) {
            sp_bits[s] |= ((uint64_t)subpkt[s][b]) << (b * 8);
        }
    }

    for (int pc = 0; pc < 32; pc++) {
        uint8_t hdr_bit = (hdr_bits >> pc) & 1;
        uint8_t ch0 = (hsync & 1) | (0 << 1) | (hdr_bit << 2) | (0 << 3);

        uint8_t sp0_bit0 = (sp_bits[0] >> (pc * 2)) & 1;
        uint8_t sp0_bit1 = (sp_bits[0] >> (pc * 2 + 1)) & 1;
        uint8_t sp2_bit0 = (sp_bits[2] >> (pc * 2)) & 1;
        uint8_t sp2_bit1 = (sp_bits[2] >> (pc * 2 + 1)) & 1;
        uint8_t ch1 = sp0_bit0 | (sp0_bit1 << 1) | (sp2_bit0 << 2) | (sp2_bit1 << 3);

        uint8_t sp1_bit0 = (sp_bits[1] >> (pc * 2)) & 1;
        uint8_t sp1_bit1 = (sp_bits[1] >> (pc * 2 + 1)) & 1;
        uint8_t sp3_bit0 = (sp_bits[3] >> (pc * 2)) & 1;
        uint8_t sp3_bit1 = (sp_bits[3] >> (pc * 2 + 1)) & 1;
        uint8_t ch2 = sp1_bit0 | (sp1_bit1 << 1) | (sp3_bit0 << 2) | (sp3_bit1 << 3);

        int entry_idx = pc / 2;
        int sub_idx = pc & 1;
        out[entry_idx * 2 + sub_idx] = hdmi_di_pixel(ch0, ch1, ch2);
    }
}

static void hdmi_build_acr_packet(uint8_t hsync) {
    uint8_t header[4];
    header[0] = 0x01;  // ACR packet type
    header[1] = 0x00;
    header[2] = 0x00;
    header[3] = hdmi_bch_ecc(header, 3);

    uint8_t subpkt[4][8];
    nf_memset(subpkt, 0, sizeof(subpkt));
    // Subpacket 0: CTS[19:0] and N[19:0]
    uint32_t cts = HDMI_ACR_CTS;
    uint32_t n = HDMI_ACR_N;
    // Per HDMI 1.4b table 5-10:
    subpkt[0][0] = 0;                    // SB0: reserved
    subpkt[0][1] = (cts >> 16) & 0x0F;  // SB1: CTS[19:16]
    subpkt[0][2] = (cts >> 8) & 0xFF;   // SB2: CTS[15:8]
    subpkt[0][3] = (cts >> 0) & 0xFF;   // SB3: CTS[7:0]
    subpkt[0][4] = (n >> 16) & 0x0F;    // SB4: N[19:16]
    subpkt[0][5] = (n >> 8) & 0xFF;     // SB5: N[15:8]
    subpkt[0][6] = (n >> 0) & 0xFF;     // SB6: N[7:0]
    subpkt[0][7] = hdmi_bch_ecc(subpkt[0], 7);
    // All 4 subpackets identical per HDMI spec
    for (int s = 1; s < 4; s++) {
        memcpy(subpkt[s], subpkt[0], 8);
    }

    hdmi_build_packet(acr_cc, header, subpkt, hsync);
}

static void hdmi_build_infoframe_packet(uint8_t hsync) {
    uint8_t header[4];
    header[0] = 0x84;  // Audio InfoFrame type
    header[1] = 0x01;  // version
    header[2] = 0x0A;  // length = 10
    header[3] = hdmi_bch_ecc(header, 3);

    uint8_t subpkt[4][8];
    nf_memset(subpkt, 0, sizeof(subpkt));
    // Subpacket 0: checksum + audio info
    // PB0 = checksum: -(sum of HB0..HB2 + PB1..PB10) & 0xFF
    uint8_t pb1 = 0x01;  // CT=1(LPCM), CC=1(2 channels)
    uint8_t pb2 = 0x00;  // SS=00, SF=00 (refer to stream header)
    uint8_t pb4 = 0x00;  // CA=0x00 (FL/FR)
    uint8_t pb5 = 0x00;  // DM_INH=0, LSV=0
    int sum = header[0] + header[1] + header[2] + pb1 + pb2 + pb4 + pb5;
    uint8_t pb0 = (uint8_t)(-(sum) & 0xFF);  // checksum
    subpkt[0][0] = pb0;
    subpkt[0][1] = pb1;
    subpkt[0][2] = pb2;
    subpkt[0][3] = 0;    // PB3: reserved
    subpkt[0][4] = pb4;
    subpkt[0][5] = pb5;
    subpkt[0][6] = 0;    // PB6
    subpkt[0][7] = hdmi_bch_ecc(subpkt[0], 7);
    // Subpacket 1: PB7..PB13 (zeroes)
    subpkt[1][7] = hdmi_bch_ecc(subpkt[1], 7);
    // Subpackets 2-3: zeroes
    for (int s = 2; s < 4; s++) subpkt[s][7] = hdmi_bch_ecc(subpkt[s], 7);

    hdmi_build_packet(infoframe_cc, header, subpkt, hsync);
}

// Build Audio Sample Packet from ring buffer into audio_pkt_cc
// Called from pcm_call (non-ISR context on core#1)
static void hdmi_encode_audio_sample_packet(uint8_t hsync) {
    uint8_t header[4];
    header[0] = 0x02;  // Audio Sample packet type
    header[1] = 0x0F;  // B[3:0]=sample_present (all 4), layout=0
    header[2] = 0x00;
    header[3] = hdmi_bch_ecc(header, 3);

    uint8_t subpkt[4][8];
    uint32_t rd = hdmi_audio_rd;
    for (int s = 0; s < 4; s++) {
        int16_t left  = hdmi_audio_ring_L[(rd + s) & HDMI_AUDIO_RING_MASK];
        int16_t right = hdmi_audio_ring_R[(rd + s) & HDMI_AUDIO_RING_MASK];
        uint32_t sample_L = ((uint32_t)(uint16_t)left) << 8;
        uint32_t sample_R = ((uint32_t)(uint16_t)right) << 8;
        subpkt[s][0] = sample_L & 0xFF;
        subpkt[s][1] = (sample_L >> 8) & 0xFF;
        subpkt[s][2] = (sample_L >> 16) & 0xFF;
        subpkt[s][3] = sample_R & 0xFF;
        subpkt[s][4] = (sample_R >> 8) & 0xFF;
        subpkt[s][5] = (sample_R >> 16) & 0xFF;
        subpkt[s][6] = 0x00;
        subpkt[s][7] = hdmi_bch_ecc(subpkt[s], 7);
    }

    hdmi_build_packet(audio_pkt_cc, header, subpkt, hsync);
    hdmi_audio_rd = rd + 4;
    audio_pkt_ready = true;
}

static void hdmi_build_preamble_guard(uint8_t hsync) {
    // Preamble: CTL0=1, CTL1=0, CTL2=1, CTL3=0 (Data Island preamble)
    // Ch0: HSYNC + VSYNC=0 → use control character
    // Ch1: CTL0=1, CTL1=0  Ch2: CTL2=1, CTL3=0
    const uint16_t b0 = 0b1101010100; // VSYNC=0, HSYNC=0
    const uint16_t b2 = 0b0101010100; // VSYNC=1, HSYNC=0
    uint16_t ch0_ctrl = hsync ? b2 : b0;  // ch0 carries sync
    // Data island preamble: CTL0=1 CTL1=0 → use specific control chars
    uint16_t ch1_ctrl = 0b0010101011;  // CTL0=1, CTL1=0
    uint16_t ch2_ctrl = 0b0010101011;  // CTL2=1, CTL3=0
    preamble_entry[0] = get_ser_diff_data(ch2_ctrl, ch1_ctrl, ch0_ctrl);
    preamble_entry[1] = preamble_entry[0];

    // Guard band: TERC4 encoded per HDMI spec 5.2.3.3
    // Ch0: TERC4({HSYNC, VSYNC=0, 1, 1})  Ch1,Ch2: TERC4(4'b0100)
    uint8_t ch0_gb = (hsync & 1) | (0 << 1) | (1 << 2) | (1 << 3);
    guard_entry[0] = hdmi_di_pixel(ch0_gb, 0x04, 0x04);
    guard_entry[1] = guard_entry[0];
}

// Fill Data Island indices into h_bp region (for 640x480 modes)
static void __attribute__((noinline)) hdmi_fill_di_indices_bp(uint8_t *line_buf, int h_sync_bytes, int h_bp_bytes) {
    if (h_bp_bytes < 24) return;
    uint8_t *bp = line_buf + h_sync_bytes;
    // [0..1] blanking control (already set)
    bp[0] = BASE_HDMI_CTRL_INX;
    bp[1] = BASE_HDMI_CTRL_INX;
    // [2..5] preamble (4 bytes = 8 TMDS pixels)
    bp[2] = IDX_DI_PREAMBLE;
    bp[3] = IDX_DI_PREAMBLE;
    bp[4] = IDX_DI_PREAMBLE;
    bp[5] = IDX_DI_PREAMBLE;
    // [6] leading guard band
    bp[6] = IDX_DI_GUARD;
    // [7..22] packet body (16 bytes = 32 TMDS pixels)
    for (int i = 0; i < 16; i++) {
        bp[7 + i] = IDX_DI_DATA_BASE + i;
    }
    // [23] trailing guard
    bp[23] = IDX_DI_GUARD;
}

// Fill Data Island in vblank line
static void __attribute__((noinline)) hdmi_fill_di_indices_vblank(uint8_t *line_buf, int h_sync_bytes, int line_bytes) {
    int rest = line_bytes - h_sync_bytes;
    if (rest < 24) return;
    uint8_t *bp = line_buf + h_sync_bytes;
    bp[0] = BASE_HDMI_CTRL_INX;
    bp[1] = BASE_HDMI_CTRL_INX;
    bp[2] = IDX_DI_PREAMBLE;
    bp[3] = IDX_DI_PREAMBLE;
    bp[4] = IDX_DI_PREAMBLE;
    bp[5] = IDX_DI_PREAMBLE;
    bp[6] = IDX_DI_GUARD;
    for (int i = 0; i < 16; i++) {
        bp[7 + i] = IDX_DI_DATA_BASE + i;
    }
    bp[23] = IDX_DI_GUARD;
}

static void __attribute__((noinline)) hdmi_audio_hw_init(void) {
    hdmi_build_preamble_guard(0);
    hdmi_build_acr_packet(0);
    hdmi_build_infoframe_packet(0);

    uint64_t *conv_color64 = (uint64_t *)conv_color;
    conv_color64[IDX_DI_PREAMBLE * 2]     = preamble_entry[0];
    conv_color64[IDX_DI_PREAMBLE * 2 + 1] = preamble_entry[1];
    conv_color64[IDX_DI_GUARD * 2]         = guard_entry[0];
    conv_color64[IDX_DI_GUARD * 2 + 1]     = guard_entry[1];

    hdmi_audio_enabled = true;
}

void hdmi_audio_init(void) {
    hdmi_audio_wr = 0;
    hdmi_audio_rd = 0;
    nf_memset((void *)hdmi_audio_ring_L, 0, sizeof(hdmi_audio_ring_L));
    nf_memset((void *)hdmi_audio_ring_R, 0, sizeof(hdmi_audio_ring_R));
    hdmi_audio_enabled = true;  // flag for graphics_init_hdmi to call hdmi_audio_hw_init
}

void hdmi_audio_write_sample(int16_t left, int16_t right) {
    uint32_t wr = hdmi_audio_wr;
    hdmi_audio_ring_L[wr & HDMI_AUDIO_RING_MASK] = left;
    hdmi_audio_ring_R[wr & HDMI_AUDIO_RING_MASK] = right;
    hdmi_audio_wr = wr + 1;

    // Every 4 samples, pre-encode a packet for the DMA ISR to pick up
    uint32_t avail = (wr + 1) - hdmi_audio_rd;
    if (avail >= 4 && !audio_pkt_ready) {
        hdmi_encode_audio_sample_packet(0);
    }
}

#endif // !PICO_RP2040
