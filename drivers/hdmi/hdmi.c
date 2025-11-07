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

//буфер  палитры 256 цветов в формате R8G8B8
static uint32_t palette[256];


#define SCREEN_WIDTH (320)
#define SCREEN_HEIGHT (240)

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
static uint32_t* __scratch_x("hdmi_ptr_4") DMA_BUF_ADDR[2];

//ДМА палитра для конвертации
//в хвосте этой памяти выделяется dma_data
static alignas(4096) uint32_t conv_color[1224];
static uint8_t __scratch_y("hdmi_ptr_5") map64colors[64] = { 0 };

//индекс, проверяющий зависание
static uint32_t irq_inx = 0;

//функции и константы HDMI

#define BASE_HDMI_CTRL_INX (240)
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

static void __scratch_x("hdmi_driver") dma_handler_HDMI() {
    static uint32_t inx_buf_dma;
    static uint line = 0;
    struct video_mode_t mode = graphics_get_video_mode(get_video_mode());
    irq_inx++;

    dma_hw->ints0 = 1u << dma_chan_ctrl;
    dma_channel_set_read_addr(dma_chan_ctrl, &DMA_BUF_ADDR[inx_buf_dma & 1], false);

    if (line >= mode.h_total ) {
        line = 0;
        ESPectrum_vsync();
    } else {
        ++line;
    }

    if ((line & 1) == 0) return;
    inx_buf_dma++;

    uint8_t* activ_buf = (uint8_t *)dma_lines[inx_buf_dma & 1];

    if (line < mode.h_width ) {
        uint8_t* output_buffer = activ_buf + 72; //для выравнивания синхры;
        int y = line >> 1;
        //область изображения
        uint8_t* input_buffer = getLineBuffer(y);
        if (!input_buffer) return;
        switch (graphics_mode) {
            case GRAPHICSMODE_DEFAULT:
                //заполняем пространство сверху и снизу графического буфера
                if (false || (graphics_buffer_shift_y > y) || (y >= (graphics_buffer_shift_y + graphics_buffer_height))
                    || (graphics_buffer_shift_x >= SCREEN_WIDTH) || (
                        (graphics_buffer_shift_x + graphics_buffer_width) < 0)) {
                    memset(output_buffer, 255, SCREEN_WIDTH);
                    break;
                }

                uint8_t* activ_buf_end = output_buffer + SCREEN_WIDTH;
            //рисуем пространство слева от буфера
                for (int i = graphics_buffer_shift_x; i-- > 0;) {
                    *output_buffer++ = 255;
                }

            //рисуем сам видеобуфер+пространство справа
///                input_buffer = &graphics_buffer[(y - graphics_buffer_shift_y) * graphics_buffer_width];
                const uint8_t* input_buffer_end = input_buffer + graphics_buffer_width;
                if (graphics_buffer_shift_x < 0) input_buffer -= graphics_buffer_shift_x;
                register size_t x = 0;
                while (activ_buf_end > output_buffer) {
                    if (input_buffer < input_buffer_end) {
                        register uint8_t c = input_buffer[(x++) ^ 2];
                        *output_buffer++ = map64colors[c & 0b00111111];
                    }
                    else
                        *output_buffer++ = 255;
                }
                break;
            default:
                for (int i = SCREEN_WIDTH; i--;) {
                    uint8_t i_color = *input_buffer++;
                    i_color = (i_color & 0xf0) == 0xf0 ? 255 : i_color;
                    *output_buffer++ = i_color;
                }
                break;
        }


        // memset(activ_buf,2,320);//test

        //ССИ
        //для выравнивания синхры

        // --|_|---|_|---|_|----
        //---|___________|-----
        memset(activ_buf + 48,BASE_HDMI_CTRL_INX, 24);
        memset(activ_buf,BASE_HDMI_CTRL_INX + 1, 48);
        memset(activ_buf + 392,BASE_HDMI_CTRL_INX, 8);

        //без выравнивания
        // --|_|---|_|---|_|----
        //------|___________|----
        //   memset(activ_buf+320,BASE_HDMI_CTRL_INX,8);
        //   memset(activ_buf+328,BASE_HDMI_CTRL_INX+1,48);
        //   memset(activ_buf+376,BASE_HDMI_CTRL_INX,24);
    }
    else {
        if ((line >= 490) && (line < 492)) {
            //кадровый синхроимпульс
            //для выравнивания синхры
            // --|_|---|_|---|_|----
            //---|___________|-----
            memset(activ_buf + 48,BASE_HDMI_CTRL_INX + 2, 352);
            memset(activ_buf,BASE_HDMI_CTRL_INX + 3, 48);
            //без выравнивания
            // --|_|---|_|---|_|----
            //-------|___________|----

            // memset(activ_buf,BASE_HDMI_CTRL_INX+2,328);
            // memset(activ_buf+328,BASE_HDMI_CTRL_INX+3,48);
            // memset(activ_buf+376,BASE_HDMI_CTRL_INX+2,24);
        }
        else {
            //ССИ без изображения
            //для выравнивания синхры

            memset(activ_buf + 48,BASE_HDMI_CTRL_INX, 352);
            memset(activ_buf,BASE_HDMI_CTRL_INX + 1, 48);

            // memset(activ_buf,BASE_HDMI_CTRL_INX,328);
            // memset(activ_buf+328,BASE_HDMI_CTRL_INX+1,48);
            // memset(activ_buf+376,BASE_HDMI_CTRL_INX,24);
        };
    }


    // y=(y==524)?0:(y+1);
    // inx_buf_dma++;
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

    int hdmi_hz = graphics_get_video_mode(get_video_mode()).freq;
    sm_config_set_clkdiv(&c_c, (clock_get_hz(clk_sys) / 252000000.0f) * (60 / hdmi_hz));
    pio_sm_init(PIO_VIDEO, SM_video, offs_prg0, &c_c);
    pio_sm_set_enabled(PIO_VIDEO, SM_video, true);

    //настройки DMA
    dma_lines[0] = &conv_color[1024];
    dma_lines[1] = &conv_color[1124];

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
        400, //
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

void graphics_set_palette(uint8_t i, uint32_t color888) {
    palette[i] = color888 & 0x00ffffff;


    if ((i >= BASE_HDMI_CTRL_INX) && (i != 255)) return; //не записываем "служебные" цвета

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

    // массив индексов палитры для всех 64 комбинаций BGR
    for (uint8_t c = 0; c < 64; ++c) {
        uint8_t b = (c >> 4) & 0b11;
        uint8_t g = (c >> 2) & 0b11;
        uint8_t r = c & 0b11;

        // базовые 16 цветов
        if (r == 0 && g == 0 && b == 0) map64colors[c] = 0;   // black
        else if (r == 3 && g == 0 && b == 0) map64colors[c] = 1; // red
        else if (r == 0 && g == 3 && b == 0) map64colors[c] = 2; // green
        else if (r == 0 && g == 0 && b == 3) map64colors[c] = 3; // blue
        else if (r == 3 && g == 3 && b == 0) map64colors[c] = 4; // yellow
        else if (r == 3 && g == 0 && b == 3) map64colors[c] = 5; // magenta
        else if (r == 0 && g == 3 && b == 3) map64colors[c] = 6; // cyan
        else if (r == 3 && g == 3 && b == 3) map64colors[c] = 7; // white
        else {
            // промежуточные оттенки для остальных комбинаций
            // индекс палитры: 8..63
            map64colors[c] = 8 + c;
        }
    }

    // Таблица уровней Pulsar RGB
    const uint8_t pulsar_levels[4] = {0x00, 0x55, 0xAA, 0xFF};

    // Настройка 64 цветов палитры
    for (uint8_t b = 0; b < 4; ++b) {
        for (uint8_t g = 0; g < 4; ++g) {
            for (uint8_t r = 0; r < 4; ++r) {
                uint8_t idx = (b << 4) | (g << 2) | r;
                uint8_t palette_idx = map64colors[idx];

                uint8_t R = pulsar_levels[r];
                uint8_t G = pulsar_levels[g];
                uint8_t B = pulsar_levels[b];

                graphics_set_palette(palette_idx, RGB888(R, G, B));
            }
        }
    }

    // for (uint8_t c = 0; c <= 0b111111; ++c) {
    //     map64colors[c] = c;
    //     switch (c)
    //     {
    //         case 0b000000: graphics_set_palette(c, RGB888(0x00, 0x00, 0x00)); break; // Black
    //         case 0b000001: graphics_set_palette(c, RGB888(0xCD, 0x00, 0x00)); break; // Red
    //         case 0b000010: graphics_set_palette(c, RGB888(0xAA, 0x00, 0x00)); break; // 
    //         case 0b000011: graphics_set_palette(c, RGB888(0xFF, 0x00, 0x00)); break; // Bright Red
    //         case 0b000100: graphics_set_palette(c, RGB888(0x00, 0xCD, 0x00)); break; // Green
    //         case 0b000101: graphics_set_palette(c, RGB888(0xCD, 0xCD, 0x00)); break; // Yellow
    //         case 0b000110: graphics_set_palette(c, RGB888(0xAA, 0xCD, 0x00)); break; // 
    //         case 0b000111: graphics_set_palette(c, RGB888(0xFF, 0xCD, 0x00)); break; // 
    //         case 0b001000: graphics_set_palette(c, RGB888(0x00, 0xAA, 0x00)); break; // 
    //         case 0b001001: graphics_set_palette(c, RGB888(0xCD, 0xAA, 0x00)); break; //
    //         case 0b001010: graphics_set_palette(c, RGB888(0xAA, 0xAA, 0x00)); break; //
    //         case 0b001011: graphics_set_palette(c, RGB888(0xFF, 0xAA, 0x00)); break; //
    //         case 0b001100: graphics_set_palette(c, RGB888(0x00, 0xFF, 0x00)); break; // Bright Green
    //         case 0b001101: graphics_set_palette(c, RGB888(0xCD, 0xFF, 0x00)); break; //
    //         case 0b001110: graphics_set_palette(c, RGB888(0xAA, 0xFF, 0x00)); break; //
    //         case 0b001111: graphics_set_palette(c, RGB888(0xFF, 0xFF, 0x00)); break; // Bright Yellow
    //         case 0b010000: graphics_set_palette(c, RGB888(0x00, 0x00, 0xCD)); break; // Blue 
    //         case 0b010001: graphics_set_palette(c, RGB888(0xCD, 0x00, 0xCD)); break; // Magenta
    //         case 0b010010: graphics_set_palette(c, RGB888(0xAA, 0x00, 0xCD)); break; //
    //         case 0b010011: graphics_set_palette(c, RGB888(0xFF, 0x00, 0xCD)); break; //
    //         case 0b010100: graphics_set_palette(c, RGB888(0x00, 0xCD, 0xCD)); break; // Cyan
    //         case 0b010101: graphics_set_palette(c, RGB888(0xCD, 0xCD, 0xCD)); break; // White (light gray)
    //         case 0b010110: graphics_set_palette(c, RGB888(0xAA, 0xCD, 0xCD)); break; //
    //         case 0b010111: graphics_set_palette(c, RGB888(0xFF, 0xCD, 0xCD)); break; //
    //         case 0b011000: graphics_set_palette(c, RGB888(0x00, 0xAA, 0xCD)); break; //
    //         case 0b011001: graphics_set_palette(c, RGB888(0xCD, 0xAA, 0xCD)); break; //
    //         case 0b011010: graphics_set_palette(c, RGB888(0xAA, 0xAA, 0xCD)); break; //
    //         case 0b011011: graphics_set_palette(c, RGB888(0xFF, 0xAA, 0xCD)); break; //
    //         case 0b011100: graphics_set_palette(c, RGB888(0x00, 0xFF, 0xCD)); break; //
    //         case 0b011101: graphics_set_palette(c, RGB888(0xCD, 0xFF, 0xCD)); break; //
    //         case 0b011110: graphics_set_palette(c, RGB888(0xAA, 0xFF, 0xCD)); break; //
    //         case 0b011111: graphics_set_palette(c, RGB888(0xFF, 0xFF, 0xCD)); break; //
    //         case 0b100000: graphics_set_palette(c, RGB888(0x00, 0x00, 0xAA)); break; //
    //         case 0b100001: graphics_set_palette(c, RGB888(0xCD, 0x00, 0xAA)); break; //
    //         case 0b100010: graphics_set_palette(c, RGB888(0xAA, 0x00, 0xAA)); break; //
    //         case 0b100011: graphics_set_palette(c, RGB888(0xFF, 0x00, 0xAA)); break; //
    //         case 0b100100: graphics_set_palette(c, RGB888(0x00, 0xCD, 0xAA)); break; //
    //         case 0b100101: graphics_set_palette(c, RGB888(0xCD, 0xCD, 0xAA)); break; //
    //         case 0b100110: graphics_set_palette(c, RGB888(0xAA, 0xCD, 0xAA)); break; //
    //         case 0b100111: graphics_set_palette(c, RGB888(0xFF, 0xCD, 0xAA)); break; //
    //         case 0b101000: graphics_set_palette(c, RGB888(0x00, 0xAA, 0xAA)); break; //
    //         case 0b101001: graphics_set_palette(c, RGB888(0xCD, 0xAA, 0xAA)); break; //
    //         case 0b101010: graphics_set_palette(c, RGB888(0xAA, 0xAA, 0xAA)); break; //
    //         case 0b101011: graphics_set_palette(c, RGB888(0xFF, 0xAA, 0xAA)); break; //
    //         case 0b101100: graphics_set_palette(c, RGB888(0x00, 0xFF, 0xAA)); break; //
    //         case 0b101101: graphics_set_palette(c, RGB888(0xCD, 0xFF, 0xAA)); break; //
    //         case 0b101110: graphics_set_palette(c, RGB888(0xAA, 0xFF, 0xAA)); break; //
    //         case 0b101111: graphics_set_palette(c, RGB888(0xFF, 0xFF, 0xAA)); break; //
    //         case 0b110000: graphics_set_palette(c, RGB888(0x00, 0x00, 0xFF)); break; // Bright Blue
    //         case 0b110001: graphics_set_palette(c, RGB888(0xCD, 0x00, 0xFF)); break; //
    //         case 0b110010: graphics_set_palette(c, RGB888(0xAA, 0x00, 0xFF)); break; //
    //         case 0b110011: graphics_set_palette(c, RGB888(0xFF, 0x00, 0xFF)); break; // Bright Magenta
    //         case 0b110100: graphics_set_palette(c, RGB888(0x00, 0xCD, 0xFF)); break; //
    //         case 0b110101: graphics_set_palette(c, RGB888(0xCD, 0xCD, 0xFF)); break; //
    //         case 0b110110: graphics_set_palette(c, RGB888(0xAA, 0xCD, 0xFF)); break; //
    //         case 0b110111: graphics_set_palette(c, RGB888(0xFF, 0xCD, 0xFF)); break; //
    //         case 0b111000: graphics_set_palette(c, RGB888(0x00, 0xAA, 0xFF)); break; //
    //         case 0b111001: graphics_set_palette(c, RGB888(0xCD, 0xAA, 0xFF)); break; //
    //         case 0b111010: graphics_set_palette(c, RGB888(0xAA, 0xAA, 0xFF)); break; //
    //         case 0b111011: graphics_set_palette(c, RGB888(0xFF, 0xAA, 0xFF)); break; //
    //         case 0b111100: graphics_set_palette(c, RGB888(0x00, 0xFF, 0xFF)); break; // Bright Cyan
    //         case 0b111101: graphics_set_palette(c, RGB888(0xCD, 0xFF, 0xFF)); break; //
    //         case 0b111110: graphics_set_palette(c, RGB888(0xAA, 0xFF, 0xFF)); break; //
    //         case 0b111111: graphics_set_palette(c, RGB888(0xFF, 0xFF, 0xFF)); break; // Bright White
    //     }
    // }

    hdmi_init();
}

void graphics_set_bgcolor_hdmi(uint32_t color888) //определяем зарезервированный цвет в палитре
{
    graphics_set_palette(255, color888);
};
