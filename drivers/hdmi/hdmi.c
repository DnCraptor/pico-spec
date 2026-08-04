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
#include "hardware/structs/bus_ctrl.h"

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
// Scanline brightness level: 0=off, 1=darkest .. 4=lightest. Level 2 is the
// legacy 0x202020 look and the default. Drives the gray of IDX_SCANLINE.
static uint8_t hdmi_scanline_level = 2;

// Bayer 2x2 dither: when enabled, alternates each pixel's palette index between
// idx and idx|0x40 according to (y^x) parity. Only safe when active palette
// indices are within [0..63] (ULA+ range) and palette[64..127] hold dither
// neighbours populated by Video.cpp.
static bool hdmi_dither = false;

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
#if PICO_RP2040
// HDMI-only: on RP2040 the VGA-HDMI build runs only ONE output per boot (VGA or
// HDMI, picked at runtime). Keeping this 4960 B (+4 KB alignment hole) in .bss
// costs SRAM on VGA boots — which is where RP2040 is tightest (the framebuffer
// barely fits). So allocate it lazily, 4 KB-aligned, in graphics_init_hdmi()
// only when HDMI is the active output; stays NULL on VGA boots. The PIO addr
// generator needs the 4 KB alignment (its low 12 bits index into the table).
static uint32_t *conv_color = NULL;
#else
static alignas(4096) uint32_t conv_color[1240];
#endif
// Snapshot of the standard palette taken at DS80-enable time. Used to restore
// conv_color back to the doubled-pixel mode when DS80 turns off.
// pico-spec: lazily heap-allocated (~5 KB) — kept out of .bss while DS80 is off
// (the common case: boot/Service ROM/std screen). Allocated on DS80 enable, freed
// on DS80 disable. Cold path (DS80 toggle only), no alignment requirement.
static uint32_t *conv_color_std_snapshot = (uint32_t *) 0;
static bool conv_color_std_snapshot_valid = false;
// map64colors removed — frame buffer now stores direct 8-bit palette indices

//индекс, проверяющий зависание
static uint32_t irq_inx = 0;

//функции и константы HDMI

#define BASE_HDMI_CTRL_INX (240)
#define IDX_SCANLINE        (244)   // dark gray for scanline effect

// Data Island palette indices (for HDMI audio, below control words to avoid overlap).
// Emulator video uses palette indices 0..136 (ZX 0..16, gigascreen blends 17..136,
// ULA+/dither 0..127); textmode (unused at runtime) maps 200..215; 240..244 are sync.
// Free safe ranges used here: 184..199 and 217..237.
#define IDX_DI_PREAMBLE     (220)   // in front porch, ch0 = {H=1,V=1}
#define IDX_DI_GUARD        (221)   // leading guard band (V=1), at the hsync edge
#define IDX_DI_DATA_BASE    (222)   // set 0: 222..237 (16 entries = 32 pixel clocks)
#define IDX_DI_DATA2_BASE   (184)   // set 1: 184..199 (second in-flight island)
#define IDX_DI_GUARD_TRAIL  (219)   // trailing guard band (V=1, hsync level may differ)
#define IDX_VIDEO_PREAMBLE  (218)   // CTL period announcing video data
#define IDX_VIDEO_GUARD     (217)   // video leading guard band
#define IDX_DI_PREAMBLE_VS  (216)   // preamble on vsync lines, ch0 = {H=1,V=0}
#define IDX_DI_GUARD_VS     (238)   // leading guard band (V=0)
#define IDX_DI_GUARD_TRAIL_VS (239) // trailing guard band (V=0)

// Per-level scanline gray (RGB888). Index by level 1..4, dark -> light.
// Level 2 == 0x202020 == the legacy look and the default. [0] unused.
static const uint32_t hdmi_scanline_gray_lut[5] = {
    0x202020, 0x101010, 0x202020, 0x404040, 0x606060
};
static inline uint32_t hdmi_scanline_gray(void) {
    return hdmi_scanline_gray_lut[(hdmi_scanline_level <= 4) ? hdmi_scanline_level : 2];
}

#if !PICO_RP2040
// ============================================================
// HDMI audio state (Data Island injection, RP2350 only)
//
// Architecture: every scanline (except those overlapping the vsync pulse)
// carries one HDMI Data Island placed at the start of the line buffer,
// inside the hsync pulse (the island's TERC4 ch0 transports the sync levels,
// exactly like PicoDVI does). Because each rendered ping-pong line buffer is
// transmitted TWICE (vertical pixel doubling), the island's 16 data bytes of
// buffer A reference conv_color set 0 and buffer B set 1; the set contents
// are rewritten between the two plays, so every transmitted line carries a
// unique packet (no duplicated audio samples).
// ============================================================
// Ring depth: this is NOT the stall cushion (that is the packet queue below,
// drained by the core1 ISR). Producer and consumer of THIS ring are both the
// core0 pcm tick — hdmi_audio_write_sample() upsamples into it and encodes
// packets out of it in the same call — and that same function hard-caps the
// backlog at 256 samples ("if (avail > 256) drop"). So occupancy is ~2 samples
// in steady state and 256 in the worst case it allows; 512 keeps 2x headroom
// over that cap. Was 1024 (4x the reachable depth): the 2 KB it gave back is
// what lets Gigascreen + HDMI audio coexist on a butter-less board (the SRAM
// budget gate missed it by ~0.5 KB). Must stay a power of two (RING_MASK).
#define HDMI_AUDIO_RING_SIZE 512
#define HDMI_AUDIO_RING_MASK (HDMI_AUDIO_RING_SIZE - 1)
// Heap-allocated together with aq_blob (~8.6 KB total) by hdmi_audio_init();
// freed by hdmi_audio_deinit() via the HdmiAudio subsystem so the SRAM is
// spent only when audio_driver == HDMI.
static volatile int16_t *hdmi_audio_ring_L = NULL;
static volatile int16_t *hdmi_audio_ring_R = NULL;
static volatile uint32_t hdmi_audio_wr = 0;
static volatile uint32_t hdmi_audio_rd = 0;
static volatile bool hdmi_audio_enabled = false;

// HDMI sinks only accept the standard rate set (32/44.1/48/88.2/96/176.4/192
// kHz) — a non-standard rate in ACR/channel-status makes some TVs reject the
// whole input (SpeccyP hit the same wall). The pcm timer produces 31250 Hz
// (ESP_AUDIO_FREQ); we Bresenham-upsample it to the HDMI output rate.
// 48 kHz (ratio 1.536, alternating 1/2 samples per tick) is the most
// universally supported PCM rate — strict sinks (Sony/Philips) often refuse
// 32 kHz (not listed in their EDID SAD) and mute, while accepting 48 kHz.
// Trade-off vs the old 32 kHz (ratio 1.024, near-constant 1/tick): 48 kHz
// produces bursts that can stress shallow TV audio FIFOs, and the queue's
// stall-tolerance shrinks ~33% (the same packet count covers fewer ms) — the
// bang-bang pacing below absorbs the burst; the queue (~10.7 ms at 128 pkt)
// stays well above the 4 ms that once caused underrun, so depth is unchanged.
#define HDMI_AUDIO_FS    48000
#define HDMI_AUDIO_FS_IN 31250

// Audio InfoFrame SF field (CEA-861 / IEC 60958 sample-frequency code), derived
// from HDMI_AUDIO_FS so the declared rate can never drift from the actual one.
#define HDMI_AUDIO_SF_CODE ( \
    (HDMI_AUDIO_FS) == 32000  ? 1 : \
    (HDMI_AUDIO_FS) == 44100  ? 2 : \
    (HDMI_AUDIO_FS) == 48000  ? 3 : \
    (HDMI_AUDIO_FS) == 88200  ? 4 : \
    (HDMI_AUDIO_FS) == 96000  ? 5 : \
    (HDMI_AUDIO_FS) == 176400 ? 6 : \
    (HDMI_AUDIO_FS) == 192000 ? 7 : 0)

// Encoded-packet blobs: 32 uint64 = 16 conv_color entry pairs = one island payload
typedef uint64_t hdmi_di_blob_t[32];
static hdmi_di_blob_t blob_null, blob_acr, blob_audio_if, blob_avi_if, blob_vendor_if;
static hdmi_di_blob_t blob_null_vs;  // Null packet with VSYNC=0 baked (vsync lines)

// Raw (un-encoded) audio-sample packet: 4-byte header + 4x8-byte subpackets
// (ECC already filled by the producer). 36 bytes vs 256 for the encoded blob —
// the TERC4 expansion (hdmi_pack_blob) is deferred to the ISR consumer, which
// encodes straight into conv_color. Trades ~27 KB of queue SRAM for a small
// per-audio-line encode in the core1 video ISR (runs in blanking).
typedef struct { uint8_t hdr[4]; uint8_t sp[4][8]; } hdmi_audio_pkt_t;

// SPSC queue of raw audio-sample packets: core0 audio IRQ -> core1 video ISR.
// The consumer (hdmi_di_load) emits packets metered by the VIDEO LINE CLOCK
// (see hdmi_au_spl24 below), not greedily, so the queue must buffer the gap
// between the core0 producer timer (which can stall on SD/USB/flash/GS) and
// the steady line-locked drain. 128 packets x 4 samples / 48 kHz ≈ 10.7 ms of
// stall tolerance (was 32 = 4 ms, too shallow → underrun → TV audio mute).
#define HDMI_AQ_LEN 128
static hdmi_audio_pkt_t *aq_blob = NULL;  // heap, see hdmi_audio_ring_L note
static volatile uint32_t aq_wr = 0, aq_rd = 0;

// Emission pacing: depth-targeted bang-bang around HDMI_AU_TARGET queued
// packets (64 pkt x 4 samples / 48 kHz ≈ 5.3 ms latency cushion). The ISR
// accrues per-line-pair credit (.24 fixed-point samples) at +1% over nominal
// while the queue is above target (drain backlog gently) and -1% at/below it
// (let the producer rebuild the cushion); hdmi_di_load pops one packet per
// 4<<24 credit and spends credit only on an actual pop. Long-run emission ==
// producer rate exactly (a zero-mean ±1% wobble the sink's FIFO absorbs).
// The cushion rides out core0 producer stalls (SD/USB/flash IRQ pressure —
// the dropouts seen at 378 MHz where a near-empty queue punched audible
// holes), and the +1% ceiling keeps the post-stall backlog from dumping at
// 1 pkt/line = 4x the ACR-declared 48 kHz (TV audio FIFO overflow -> ~0.5 s
// mute, the original rare-dropout cause).
// NB: the two rates must STRADDLE the producer rate with margin: the ISR
// line counter wraps 0..v_total INCLUSIVE (a 640x480 frame is 525 lines,
// not 524), so per-frame accrual misses ~0.19%; an all-below-nominal rate
// starves the sink into a permanent mute. ±1% absorbs that and any
// per-mode counting quirks.
#define HDMI_AU_TARGET 64
static uint32_t hdmi_au_spl24_hi = 0, hdmi_au_spl24_lo = 0;
static uint32_t hdmi_au_pos = 0;       // consumer-only (core1)
// Credit ceiling. Must survive the longest run of consecutive lines that emit
// NO audio packet (InfoFrame burst + vsync), or banked credit is clamped away
// and the consumer systematically under-delivers -> sink mutes permanently.
// 576p50 puts vsync_start right after the 4 InfoFrame lines (9-line contiguous
// run) — at 48 kHz that needs ~14 samples of credit; the old fixed 8<<24 (2
// packets) clamped it (worked at 32 kHz, ~9 samples, by a hair). Computed per
// mode from spl24 in hdmi_audio_hw_init. The post-run catch-up burst is bounded
// by the actual queue backlog, not this ceiling, so a generous cap is safe.
static uint32_t hdmi_au_cap = (8u << 24);

// Per-character ch0 TERC4 base (sync levels + D3 first-char flag), mode-baked.
// _vs variant has VSYNC=0 (Null packets transmitted on vsync lines).
static uint8_t di_ch0_data[32];
static uint8_t di_ch0_data_vs[32];
// First logical line of the InfoFrame/ACR burst (v_active, or 0 if no vblank)
static uint di_if_base = 0;
// Cached vsync window for the ISR-side packet loader
static uint di_vs_start = 0, di_vs_end = 0;

// Per-argument TERC4 serialization LUTs: full pixel = lut_a1[ch2]|lut_a2[ch1]|lut_a3[ch0]
static uint64_t terc_lut_a1[16], terc_lut_a2[16], terc_lut_a3[16];

// Forward declarations
static void __attribute__((noinline)) hdmi_audio_hw_init(void);
static void hdmi_di_load(uint set, uint logical_line);

// Diagnostic mode: cycle injection stages ~21 s each. 0 = no injection (pure
// DVI), 1 = islands on vblank lines only, 2 = + video preamble/guard on active
// lines, 3 = full per-line islands. 0 = staging off (always full scheme).
#define HDMI_AUDIO_DEBUG_STAGES 0
#if HDMI_AUDIO_DEBUG_STAGES
static volatile uint32_t hdmi_dbg_frame_ct = 0;
#endif
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

// Current HDMI scanline counter (exposed for Profi palette refresh sync).
volatile uint hdmi_current_line = 0;

// IRQ-latency diagnostic: largest gap between consecutive HDMI line IRQs (µs).
// If the IRQ is serviced late (line not ready in time), this spikes well above
// the nominal per-IRQ interval. Read+reset from core0 (Video PERF log).
volatile uint32_t hdmi_irq_max_gap_us = 0;

static void __scratch_x("hdmi_driver") dma_handler_HDMI() {
    static uint32_t inx_buf_dma;
    static uint line = 0;
    uint32_t isr_t0 = time_us_32();
    uint32_t isr_gap;
    {
        static uint32_t last_irq_us = 0;
        isr_gap = isr_t0 - last_irq_us;
        last_irq_us = isr_t0;
        if (isr_gap > hdmi_irq_max_gap_us) hdmi_irq_max_gap_us = isr_gap;
    }
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
    hdmi_current_line = line; // expose to main context for palette refresh sync

#if !PICO_RP2040 && HDMI_AUDIO_DEBUG_STAGES
    // Stage clock must tick before the per-line early returns below
    if (line == 0) hdmi_dbg_frame_ct++;
#endif

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

#if !PICO_RP2040
    // HDMI-audio packet loads run BEFORE the render: the set for the next
    // scanline must be written before that line starts, and doing it after
    // the render leaves too little margin when the ISR fires late (SD/USB
    // IRQ pressure) — torn packets have bad BCH and make the sink mute.
    bool au_ok_now = false, au_ok_prev = false, au_video_guards = false, au_pair_vs = false;
    // DS80 mode is island-compatible: when HDMI audio is on, init_profi_pair_lookup
    // (Video.cpp) keeps the DI palette indices (184..199, 216..239) out of pair_lut.
    if (hdmi_audio_enabled) {
        const uint b = inx_buf_dma & 1;
        au_pair_vs = (line >= mode.vsync_start) && (line < mode.vsync_end);
        au_ok_now = true; au_ok_prev = true; au_video_guards = true;
        // Line-clock credit, bang-bang by queue depth vs target (see the
        // hdmi_au_spl24_hi declaration). This ISR covers exactly 2
        // transmitted lines (pixel doubling); with scanlines enabled the gray
        // 2nd play carries no island and its di_load call is skipped, so
        // accrue per pair HERE — metering per call would halve the delivered
        // rate. Cap (hdmi_au_cap) sized per mode to the longest no-pop run so
        // credit banked through the ACR/InfoFrame/vsync span is never clamped
        // away (that under-delivers and mutes the sink — 576p50 @48kHz).
        hdmi_au_pos += ((aq_wr - aq_rd) > HDMI_AU_TARGET ? hdmi_au_spl24_hi
                                                         : hdmi_au_spl24_lo) * 2;
        if (hdmi_au_pos > hdmi_au_cap) hdmi_au_pos = hdmi_au_cap;
#if HDMI_AUDIO_DEBUG_STAGES
        {
            const uint stage = (hdmi_dbg_frame_ct >> 10) & 3;
            if (stage == 0) { au_ok_now = false; au_ok_prev = false; au_video_guards = false; }
            else if (stage <= 2) {
                au_ok_now  = (line >= (uint)mode.v_active);
                au_ok_prev = ((line - 2) >= (uint)mode.v_active);
                au_video_guards = (stage == 2);
            }
        }
#endif
        // Packet for the OTHER buffer's 2nd play (transmits on the next
        // scanline). The line that just started consumes its island bytes
        // within ~1.8 µs of ISR entry — wait that out first. With scanlines
        // enabled the 2nd play of an active pair is the static gray buffer
        // (no island) — skip so audio packets aren't lost.
        // An extremely late ISR (gap well past the ~32 µs line period) can't
        // finish this write before the next line starts reading the set — a
        // torn packet has bad BCH and the sink mutes. Skip instead: the
        // previous packet repeats (valid, minor artifact, no mute).
        if (au_ok_prev && isr_gap < 45 &&
            !(hdmi_scanlines && (line + 1) <= mode.v_active)) {
            while (time_us_32() - isr_t0 < 5) tight_loop_contents();
            hdmi_di_load(b ^ 1, line - 1);
        }
        // Packet for the just-rendered buffer's 1st play; its set was last
        // read at the start of the previous scanline — safe immediately.
        if (au_ok_now) hdmi_di_load(b, line);
    }
#endif

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

        // DS80 fast path: replace the byte-loop x^2 read with a 32-bit pair-swap.
        // The ^2 swap (x XOR 2) within each 4-byte group = rotate the 32-bit word
        // by 16 bits (swap high and low 16-bit halves).  Sequential 32-bit loads
        // are cache-friendly; this is ~6x faster than the byte loop, increasing
        // the ISR timing margin from ~1.3 µs to ~2.1 µs at 504 MHz.
        if (profi_ds80_active && !hdmi_dither) {
            const uint32_t* __restrict in32  = (const uint32_t*)input_buffer;
            uint32_t* __restrict       out32 = (uint32_t*)output_buffer;
            const int words = scr_w >> 2;
            for (int i = 0; i < words; i++) {
                uint32_t v = in32[i];
                out32[i] = (v >> 16) | (v << 16);
            }
            goto ex;
        }

        // рисуем пространство слева от буфера
        for (int i = graphics_buffer_shift_x; i-- > 0;) {
            *output_buffer++ = 255;
        }

        // рисуем сам видеобуфер+пространство справа
        const uint8_t* input_buffer_end = input_buffer + graphics_buffer_width;
        if (graphics_buffer_shift_x < 0) input_buffer -= graphics_buffer_shift_x;
        register size_t x = 0;
        if (hdmi_dither) {
            // Bayer 2x2: pixel at (x,y) takes idx | (((y^x)&1) << 6)
            // palette[idx | 0x40] holds the dither neighbour (Video.cpp)
            const uint8_t row_xor = (y & 1) ? 0x40 : 0x00;
            while (activ_buf_end > output_buffer) {
                if (input_buffer < input_buffer_end) {
                    uint8_t idx = input_buffer[(x) ^ 2];
                    *output_buffer++ = idx | (row_xor ^ ((x & 1) ? 0x40 : 0x00));
                    x++;
                } else {
                    *output_buffer++ = 255;
                }
            }
        } else {
            while (activ_buf_end > output_buffer) {
                if (input_buffer < input_buffer_end) {
                    // Direct 8-bit palette index — no mask or lookup needed
                    *output_buffer++ = input_buffer[(x++) ^ 2];
                }
                else
                    *output_buffer++ = 255;
            }
        }
ex:

        //ССИ — горизонтальная синхронизация
        nf_memset(activ_buf + h_sync, BASE_HDMI_CTRL_INX, h_bp);
        nf_memset(activ_buf, BASE_HDMI_CTRL_INX + 1, h_sync);
        nf_memset(activ_buf + line_sz - h_fp, BASE_HDMI_CTRL_INX, h_fp);
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
        };
    }

#if !PICO_RP2040
    if (hdmi_audio_enabled) {
        // Byte-level decoration (after the sync memsets above). Vsync lines
        // carry VSYNC=0 variants (Null packets), matching the memset branch.
        if (au_ok_now) {
            // Self-contained island inside the hsync pulse:
            // [0..3] DI preamble, [4] leading guard, [5..20] packet, [21] trailing guard
            uint8_t *p = activ_buf;
            const uint8_t pre = au_pair_vs ? IDX_DI_PREAMBLE_VS : IDX_DI_PREAMBLE;
            p[0] = pre; p[1] = pre; p[2] = pre; p[3] = pre;
            p[4] = au_pair_vs ? IDX_DI_GUARD_VS : IDX_DI_GUARD;
            const uint8_t dbase = (inx_buf_dma & 1) ? IDX_DI_DATA2_BASE : IDX_DI_DATA_BASE;
            for (int i = 0; i < 16; i++) p[5 + i] = dbase + i;
            p[21] = au_pair_vs ? IDX_DI_GUARD_TRAIL_VS : IDX_DI_GUARD_TRAIL;
        }
        if (au_video_guards && line < mode.v_active) {
            // HDMI mode requires a video preamble + guard band before active video
            uint8_t *bp_end = activ_buf + h_sync + h_bp;
            bp_end[-5] = IDX_VIDEO_PREAMBLE; bp_end[-4] = IDX_VIDEO_PREAMBLE;
            bp_end[-3] = IDX_VIDEO_PREAMBLE; bp_end[-2] = IDX_VIDEO_PREAMBLE;
            bp_end[-1] = IDX_VIDEO_GUARD;
        }
    }
#endif
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

    // Scanline color: gray RGB888 for the current brightness level
    graphics_set_palette(IDX_SCANLINE, hdmi_scanline_gray());

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

    // clk pins are 38/39 on ZERO2 — shift in 64-bit (3u << 38 would be UB on a 32-bit int)
    uint64_t mask64 = ((uint64_t)3u << beginHDMI_PIN_clk);
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
    // Win DMA-arbiter against other channels (audio I2S, SD, PSRAM): this feeds
    // the TMDS PIO FIFO; starving it underruns the line → HDMI sync loss.
    channel_config_set_high_priority(&cfg_dma, true);

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
#if !PICO_RP2040
        if (hdmi_audio_enabled) {
            // HDMI (audio) mode: active lines need video preamble + guard band
            for (int i = 0; i < 4; i++) hdmi_scanline_buf[hs + bp - 5 + i] = IDX_VIDEO_PREAMBLE;
            hdmi_scanline_buf[hs + bp - 1] = IDX_VIDEO_GUARD;
        }
#endif
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
    channel_config_set_high_priority(&cfg_dma, true); // index→TMDS feeder, same rationale

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
    channel_config_set_high_priority(&cfg_dma, true); // must keep pace with dma_pal_conv/dma_chan

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

    // Give the DMA bus master priority over the two cores at the SRAM/AHB arbiter.
    // Without this, a busy core0 (emulator + per-frame video work) can win SRAM
    // arbitration and stall the video feeder DMAs mid-line → PIO FIFO underrun →
    // HDMI sync loss. Worse at higher core0 clock (more bus pressure per line
    // time), which matches the "drops out at 504 MHz" symptom. Video DMA bursts
    // are short, so cores lose only a few cycles — a safe, standard setting.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS;

    dma_start_channel_mask((1u << dma_chan_ctrl));

    return true;
};

// In VGA_HDMI mode, also update the VGA palette LUT
#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
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

#if defined(VGA_HDMI) && !defined(SUPPRESS_VGA)
    // Update VGA palette LUT in parallel
    vga_set_palette_entry(i, color888);
#endif

    if ((i >= BASE_HDMI_CTRL_INX) && (i != 255) && (i != IDX_SCANLINE)) return; //не записываем "служебные" цвета
#if !PICO_RP2040
    // Protect Data Island / guard-band conv_color entries from palette writes
    if (hdmi_audio_enabled &&
        ((i >= IDX_DI_DATA2_BASE && i < IDX_DI_DATA2_BASE + 16) ||
         (i >= IDX_DI_PREAMBLE_VS && i <= IDX_DI_GUARD_TRAIL_VS))) return;
#endif

    // On RP2040 conv_color is NULL on VGA boots (allocated only for HDMI, see
    // graphics_init_hdmi) — the parallel vga_set_palette_entry() above already
    // updated the live VGA LUT, so there is nothing more to do here.
    if (!conv_color) return;
    uint64_t* conv_color64 = (uint64_t *)conv_color;
    const uint8_t R = (color888 >> 16) & 0xff;
    const uint8_t G = (color888 >> 8) & 0xff;
    const uint8_t B = (color888 >> 0) & 0xff;
    conv_color64[i * 2] = get_ser_diff_data(tmds_encoder(R), tmds_encoder(G), tmds_encoder(B));
    // Second pixel of the pair: the opposite-disparity variant of the SAME
    // byte — flip TMDS data bits D0-7 AND the inversion flag D9 (keep D8).
    // The old mask (0x0003ffffffffffff) flipped only D0-7, producing
    // characters outside the legal TMDS code set: DVI sinks decode them
    // anyway (±1 LSB color error), but strict HDMI-mode receivers classify
    // every character and reject the video period — black screen with
    // working Data Island audio. Serialized layout: symbol bit b occupies
    // bits [6b+2(b>=5) .. +5], so D0-7 = bits 0..49, D9 = bits 56..61.
    conv_color64[i * 2 + 1] = conv_color64[i * 2] ^ 0x3F03FFFFFFFFFFFFull;
};

#define RGB888(r, g, b) ((r<<16) | (g << 8 ) | b )

// Profi DS80 "packed nibble" mode:
//   Normally conv_color[i*2] and conv_color[i*2+1] hold TMDS for one palette
//   index, so 1 fb byte → 2 identical HDMI pixels (hardware doubling).
//
//   In DS80 mode we switch PIO X register to a *separate* conv_color_ds80
//   buffer where entries 0..255 are encoded as packed pairs: each fb byte 'b'
//   yields 2 different HDMI pixels (high nibble = left, low = right).
//   Entries 220..244 (HDMI sync/control) are mirrored from conv_color so the
//   HDMI stream stays well-formed.
//
//   The switch happens at vsync time (HDMI ISR honors a pending flag) so we
//   never tear scanout mid-frame.
extern volatile bool profi_ds80_active; // defined in vga.c, shared with VGA path

// Profi DS80 packed-pair palette setup.
// active=true:
//   - Snapshot the current conv_color (for full restore on disable, including audio/sync).
//   - For every (ink,paper) pair encoded in pair_lut[ink*16+paper], write
//     conv_color[slot] = (TMDS(palette[ink]), TMDS(palette[paper])).
//     pair_lut never references sync slots 240-244 or border 255; with HDMI
//     audio on it also avoids the Data Island indices 184-199/216-239
//     (init_profi_pair_lookup, Video.cpp) — islands keep running through DS80.
//   - Explicitly set slot 255 = (TMDS(black), TMDS(black)) so HDMI border fill is black.
// active=false: restore from snapshot (skipping live DI slots, see below).
// Idempotent in both directions.
void hdmi_set_profi_ds80_mode(bool active,
                               const uint32_t *palette16_rgb888,
                               const uint8_t  *pair_lut) {
    // active=true with palette args: activate OR refresh palette (no early-return)
    // active=true without args: skip
    // active=false: deactivate (skip if already inactive)
    if (active && (!palette16_rgb888 || !pair_lut)) return;
    if (!active && !profi_ds80_active) return;

    uint64_t *cc64 = (uint64_t *)conv_color;
    if (active) {
        // Snapshot only on first activation, not on refresh
        if (!profi_ds80_active) {
            // Lazily allocate the ~5 KB snapshot now (freed again on DS80 disable).
            // If it fails, refuse to enter DS80 (stay in std mode) rather than crash
            // or leave no way to restore — degraded but safe.
            if (!conv_color_std_snapshot)
                conv_color_std_snapshot = (uint32_t *) malloc(1240 * sizeof(uint32_t));
            if (!conv_color_std_snapshot) return;
            for (int i = 0; i < 1240; i++) conv_color_std_snapshot[i] = conv_color[i];
            conv_color_std_snapshot_valid = true;
        }

        uint64_t tmds16[16];
        // Per-channel TMDS values (10-bit) and per-channel disparity (ones×2 - 10).
        // Separate R/G/B disparity is critical: for colors like bright-red (+8,-8,-8)
        // and bright-green (-8,+8,-8), the combined disparity is same-sign (-8×-8=+64)
        // but individual channels need OPPOSITE complement decisions — using a single
        // combined use_compl would leave R and G at ±16, causing sync loss.
        uint R_raw[16], G_raw[16], B_raw[16];
        int disp_R[16], disp_G[16], disp_B[16];
        for (int p = 0; p < 16; p++) {
            uint32_t c = palette16_rgb888[p] & 0x00ffffff;
            uint R = tmds_encoder((c >> 16) & 0xff);
            uint G = tmds_encoder((c >>  8) & 0xff);
            uint B = tmds_encoder( c        & 0xff);
            R_raw[p] = R; G_raw[p] = G; B_raw[p] = B;
            tmds16[p] = get_ser_diff_data(R, G, B);
            disp_R[p] = (int)__builtin_popcount(R & 0x3FF) * 2 - 10;
            disp_G[p] = (int)__builtin_popcount(G & 0x3FF) * 2 - 10;
            disp_B[p] = (int)__builtin_popcount(B & 0x3FF) * 2 - 10;
        }
        // Write all unique slots referenced by pair_lut.
        // pixel-0 = ink  (first  HDMI pixel clock of the pair)
        // pixel-1 = paper (second HDMI pixel clock of the pair)
        //
        // DC balance: for each channel independently, complement paper's data bits 0-7
        // (via ^ 0xFF on the 10-bit TMDS value) when ink and paper have same-sign
        // disparity on that channel.  Channels with opposite-sign disparity are already
        // balanced without complementing.  Per-channel decisions are passed to
        // get_ser_diff_data, which assembles the 64-bit differential pair normally.
        //
        // pair_lut normalises paper=8 → paper=0 (bright-black bg = black bg), so
        // (ink, paper=8) shares the slot of (ink, paper=0).  The written[] guard
        // ensures the canonical paper=0 TMDS value is NOT overwritten by paper=8.
        // ink=8 has independent slots (different from ink=0) so palette[8] renders
        // independently; changing palette[8] cannot corrupt black-ink pixels.
        bool written[256] = {};
        for (int ink = 0; ink < 16; ink++) {
            for (int paper = 0; paper < 16; paper++) {
                uint8_t slot = pair_lut[ink * 16 + paper];
                // pair_lut guarantees slot is never in sync/border range
                if (!written[slot]) {
                    written[slot] = true;
                    cc64[slot * 2 + 0] = tmds16[ink];
                    // Per-channel complement: flip data bits 0-7 when same-sign disparity.
                    uint R_p = R_raw[paper] ^ ((disp_R[ink] * disp_R[paper] >= 0) ? 0xFF : 0);
                    uint G_p = G_raw[paper] ^ ((disp_G[ink] * disp_G[paper] >= 0) ? 0xFF : 0);
                    uint B_p = B_raw[paper] ^ ((disp_B[ink] * disp_B[paper] >= 0) ? 0xFF : 0);
                    cc64[slot * 2 + 1] = get_ser_diff_data(R_p, G_p, B_p);
                }
            }
        }
        // Slot 255: border fill byte — must be (black, black).
        // Black: all channels disp=-8 (same-sign self) → complement all channels.
        cc64[255 * 2 + 0] = tmds16[0];
        cc64[255 * 2 + 1] = get_ser_diff_data(R_raw[0]^0xFF, G_raw[0]^0xFF, B_raw[0]^0xFF);

        __dmb(); // ensure all conv_color writes are visible to core1 before flag is set
        profi_ds80_active = true;
    } else {
        if (conv_color_std_snapshot_valid && conv_color_std_snapshot) {
            for (int i = 0; i < 1240; i++) {
#if !PICO_RP2040
                // With audio live, the core1 ISR owns the DI slots: data sets are
                // rewritten every line (a stale snapshot word restored mid-scan =
                // torn packet, bad BCH, sink mute) and the control entries are
                // mode constants identical to the snapshot — skip the whole range.
                const int slot = i >> 2;  // 4 uint32 words per palette slot
                if (hdmi_audio_enabled &&
                    ((slot >= 184 && slot <= 199) || (slot >= 216 && slot <= 239)))
                    continue;
#endif
                conv_color[i] = conv_color_std_snapshot[i];
            }
        }
        profi_ds80_active = false;
        // Release the ~5 KB snapshot while DS80 is off (re-taken on next enable).
        free(conv_color_std_snapshot);
        conv_color_std_snapshot = (uint32_t *) 0;
        conv_color_std_snapshot_valid = false;
    }
}

void graphics_init_hdmi() {
#if PICO_RP2040
    // conv_color is not in .bss on RP2040 (saves ~5 KB on VGA boots). Allocate it
    // now that HDMI is confirmed to be the active output. The PIO addr generator
    // reconstructs table addresses from its X register = (base >> 12), so the
    // table MUST be 4 KB-aligned — over-allocate and round up (never freed; lives
    // for the whole HDMI session).
    // pico_malloc PANICs on OOM instead of returning NULL, so pre-check the
    // largest satisfiable block (same guard as Buffer::palloc) — this runs on
    // core1 right after setup, when free heap can be only a few KB.
    if (!conv_color) {
        extern size_t getLargestAllocatable(void);
        const size_t need = 1240 * sizeof(uint32_t) + 4096;
        if (getLargestAllocatable() >= need) {
            uintptr_t raw = (uintptr_t)malloc(need);
            conv_color = (uint32_t *)((raw + 4095) & ~(uintptr_t)4095);
        } else {
            printf("graphics_init_hdmi: OOM allocating conv_color (%u B needed)\n", (unsigned)need);
        }
    }
#endif
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

void hdmi_set_scanlines(uint8_t level) {
    if (level > 4) level = 4;
    hdmi_scanlines = (level != 0);
    // Off keeps the previous brightness so toggling back is cheap; a real level
    // change re-tints the scanline palette index live (no mode switch needed).
    if (level != 0 && level != hdmi_scanline_level) {
        hdmi_scanline_level = level;
        graphics_set_palette(IDX_SCANLINE, hdmi_scanline_gray());
    }
}

void hdmi_set_dither(bool enabled) {
    hdmi_dither = enabled;
}

// ============================================================
// HDMI Audio — Data Island encoding (RP2350 only)
//
// Reference implementation: PICO-BK drivers/libdvi (ikjordan/DnCraptor
// PicoDVI audio fork, hw-confirmed working) + HDMI 1.4b spec.
// ============================================================
#if !PICO_RP2040

// TERC4 encoding table: 4-bit value → 10-bit codeword (HDMI 1.4b Table 5-4)
static const uint16_t terc4_table[16] = {
    0b1010011100, 0b1001100011, 0b1011100100, 0b1011100010,
    0b0101110001, 0b0100011110, 0b0110001110, 0b0100111100,
    0b1011001100, 0b0100111001, 0b0110011100, 0b1011000110,
    0b1010001110, 0b1001110001, 0b0101100011, 0b1011000011
};

// BCH(64,56)/(32,24) ECC table, polynomial x^8+x^4+x^3+x^2+1 (in RAM: the
// encoder runs in IRQ context and XIP flash stalls under PSRAM traffic)
static uint8_t hdmi_bch_table[256] = {
    0x00, 0xd9, 0xb5, 0x6c, 0x6d, 0xb4, 0xd8, 0x01, 0xda, 0x03, 0x6f, 0xb6, 0xb7, 0x6e, 0x02, 0xdb,
    0xb3, 0x6a, 0x06, 0xdf, 0xde, 0x07, 0x6b, 0xb2, 0x69, 0xb0, 0xdc, 0x05, 0x04, 0xdd, 0xb1, 0x68,
    0x61, 0xb8, 0xd4, 0x0d, 0x0c, 0xd5, 0xb9, 0x60, 0xbb, 0x62, 0x0e, 0xd7, 0xd6, 0x0f, 0x63, 0xba,
    0xd2, 0x0b, 0x67, 0xbe, 0xbf, 0x66, 0x0a, 0xd3, 0x08, 0xd1, 0xbd, 0x64, 0x65, 0xbc, 0xd0, 0x09,
    0xc2, 0x1b, 0x77, 0xae, 0xaf, 0x76, 0x1a, 0xc3, 0x18, 0xc1, 0xad, 0x74, 0x75, 0xac, 0xc0, 0x19,
    0x71, 0xa8, 0xc4, 0x1d, 0x1c, 0xc5, 0xa9, 0x70, 0xab, 0x72, 0x1e, 0xc7, 0xc6, 0x1f, 0x73, 0xaa,
    0xa3, 0x7a, 0x16, 0xcf, 0xce, 0x17, 0x7b, 0xa2, 0x79, 0xa0, 0xcc, 0x15, 0x14, 0xcd, 0xa1, 0x78,
    0x10, 0xc9, 0xa5, 0x7c, 0x7d, 0xa4, 0xc8, 0x11, 0xca, 0x13, 0x7f, 0xa6, 0xa7, 0x7e, 0x12, 0xcb,
    0x83, 0x5a, 0x36, 0xef, 0xee, 0x37, 0x5b, 0x82, 0x59, 0x80, 0xec, 0x35, 0x34, 0xed, 0x81, 0x58,
    0x30, 0xe9, 0x85, 0x5c, 0x5d, 0x84, 0xe8, 0x31, 0xea, 0x33, 0x5f, 0x86, 0x87, 0x5e, 0x32, 0xeb,
    0xe2, 0x3b, 0x57, 0x8e, 0x8f, 0x56, 0x3a, 0xe3, 0x38, 0xe1, 0x8d, 0x54, 0x55, 0x8c, 0xe0, 0x39,
    0x51, 0x88, 0xe4, 0x3d, 0x3c, 0xe5, 0x89, 0x50, 0x8b, 0x52, 0x3e, 0xe7, 0xe6, 0x3f, 0x53, 0x8a,
    0x41, 0x98, 0xf4, 0x2d, 0x2c, 0xf5, 0x99, 0x40, 0x9b, 0x42, 0x2e, 0xf7, 0xf6, 0x2f, 0x43, 0x9a,
    0xf2, 0x2b, 0x47, 0x9e, 0x9f, 0x46, 0x2a, 0xf3, 0x28, 0xf1, 0x9d, 0x44, 0x45, 0x9c, 0xf0, 0x29,
    0x20, 0xf9, 0x95, 0x4c, 0x4d, 0x94, 0xf8, 0x21, 0xfa, 0x23, 0x4f, 0x96, 0x97, 0x4e, 0x22, 0xfb,
    0x93, 0x4a, 0x26, 0xff, 0xfe, 0x27, 0x4b, 0x92, 0x49, 0x90, 0xfc, 0x25, 0x24, 0xfd, 0x91, 0x48,
};

static inline uint8_t hdmi_bch3(const uint8_t *p) {
    uint8_t v = hdmi_bch_table[p[0]];
    v = hdmi_bch_table[p[1] ^ v];
    v = hdmi_bch_table[p[2] ^ v];
    return v;
}

static inline uint8_t hdmi_bch7(const uint8_t *p) {
    uint8_t v = hdmi_bch_table[p[0]];
    for (int i = 1; i < 7; i++) v = hdmi_bch_table[p[i] ^ v];
    return v;
}

// Byte parity (even parity bit), packed 1 bit per value (in RAM, IRQ path)
static uint8_t hdmi_parity_table[32] = {
    0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69, 0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96,
    0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96, 0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69
};
static inline uint8_t hdmi_parity8(uint8_t x) {
    return (hdmi_parity_table[x >> 3] >> (x & 7)) & 1;
}

// Serialize ONE argument slot of get_ser_diff_data (other channels contribute
// zero bits). full_pixel == ser_arg(a,1)|ser_arg(b,2)|ser_arg(c,3) for
// get_ser_diff_data(a,b,c) — channel bit positions in each 6-bit group are
// disjoint, so per-channel LUTs can be OR-combined.
static uint64_t hdmi_ser_one_arg(uint16_t data, int arg) {
    uint64_t out64 = 0;
    for (int i = 0; i < 10; i++) {
        out64 <<= 6;
        if (i == 5) out64 <<= 2;
        uint8_t bit = (data >> (9 - i)) & 1;
        uint8_t b2 = bit | ((bit ^ 1) << 1);
        if (HDMI_PIN_invert_diffpairs) b2 ^= 0b11;
        // wire: 0=R position, 1=G position, 2=B position of the d6 group
#ifdef PICO_PC
        int wire = (arg == 1) ? 1 : (arg == 2) ? 0 : 2;  // R/G swapped (see get_ser_diff_data)
#else
        int wire = (arg == 1) ? 0 : (arg == 2) ? 1 : 2;
#endif
        int shift;
        if (HDMI_PIN_RGB_notBGR) shift = (wire == 0) ? 4 : (wire == 1) ? 2 : 0;
        else                     shift = (wire == 2) ? 4 : (wire == 1) ? 2 : 0;
        out64 |= (uint64_t)(b2 << shift);
    }
    return out64;
}

static void hdmi_build_terc_luts(void) {
    for (int v = 0; v < 16; v++) {
        terc_lut_a1[v] = hdmi_ser_one_arg(terc4_table[v], 1);  // ch2 (arg "R")
        terc_lut_a2[v] = hdmi_ser_one_arg(terc4_table[v], 2);  // ch1 (arg "G")
        terc_lut_a3[v] = hdmi_ser_one_arg(terc4_table[v], 3);  // ch0 (arg "B")
    }
}

// Pack a 31-byte HDMI packet (4-byte header + 4×8-byte subpackets, ECC already
// filled) into 32 island characters (= 16 conv_color entry pairs).
// Channel mapping per HDMI 1.4b §5.2.3.4:
//   ch0: D0=HSYNC D1=VSYNC D2=header bit D3=1 (0 on first island character)
//   ch1: D0..D3 = bit 2t of subpackets 0..3
//   ch2: D0..D3 = bit 2t+1 of subpackets 0..3
// Sync levels and D3 come pre-baked in di_ch0_data (mode geometry dependent).
static void __not_in_flash_func(hdmi_pack_blob_ch0)(uint64_t out[32], const uint8_t hdr[4],
                                                    const uint8_t sp[4][8], const uint8_t *ch0base) {
    uint32_t hdr_bits = hdr[0] | (hdr[1] << 8) | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
    for (int i = 0; i < 8; i++) {
        // 4x8 bit transpose (PICO-BK encode_subpacket)
        uint32_t v = sp[0][i] | (sp[1][i] << 8) | ((uint32_t)sp[2][i] << 16) | ((uint32_t)sp[3][i] << 24);
        uint32_t t = (v ^ (v >> 7)) & 0x00aa00aa; v = v ^ t ^ (t << 7);
        t = (v ^ (v >> 14)) & 0x0000cccc; v = v ^ t ^ (t << 14);
        const int pc = i * 4;
        const uint8_t c1[4] = { (uint8_t)(v & 15), (uint8_t)((v >> 16) & 15),
                                (uint8_t)((v >> 4) & 15), (uint8_t)((v >> 20) & 15) };
        const uint8_t c2[4] = { (uint8_t)((v >> 8) & 15), (uint8_t)((v >> 24) & 15),
                                (uint8_t)((v >> 12) & 15), (uint8_t)((v >> 28) & 15) };
        for (int k = 0; k < 4; k++) {
            uint8_t ch0 = ch0base[pc + k] | (((hdr_bits >> (pc + k)) & 1) << 2);
            out[pc + k] = terc_lut_a1[c2[k]] | terc_lut_a2[c1[k]] | terc_lut_a3[ch0];
        }
    }
}

static void __not_in_flash_func(hdmi_pack_blob)(uint64_t out[32], const uint8_t hdr[4],
                                                const uint8_t sp[4][8]) {
    hdmi_pack_blob_ch0(out, hdr, sp, di_ch0_data);
}

// ---------- static packets ----------

static void hdmi_build_null_blob(void) {
    uint8_t hdr[4] = { 0, 0, 0, 0 };
    uint8_t sp[4][8];
    nf_memset(sp, 0, sizeof(sp));
    hdmi_pack_blob(blob_null, hdr, sp);
    hdmi_pack_blob_ch0(blob_null_vs, hdr, sp, di_ch0_data_vs);
}

static void hdmi_build_acr_blob(uint32_t cts, uint32_t n) {
    uint8_t hdr[4] = { 0x01, 0x00, 0x00, 0 };
    hdr[3] = hdmi_bch3(hdr);
    uint8_t sp[4][8];
    nf_memset(sp, 0, sizeof(sp));
    sp[0][0] = 0;
    sp[0][1] = (cts >> 16) & 0xFF;
    sp[0][2] = (cts >> 8) & 0xFF;
    sp[0][3] = cts & 0xFF;
    sp[0][4] = (n >> 16) & 0xFF;
    sp[0][5] = (n >> 8) & 0xFF;
    sp[0][6] = n & 0xFF;
    sp[0][7] = hdmi_bch7(sp[0]);
    for (int s = 1; s < 4; s++) memcpy(sp[s], sp[0], 8);
    hdmi_pack_blob(blob_acr, hdr, sp);
}

// InfoFrame checksum over header + payload, stored in PB0
static void hdmi_if_checksum(uint8_t hdr[4], uint8_t sp[4][8]) {
    int s = hdr[0] + hdr[1] + hdr[2];
    int n = hdr[2] + 1;
    for (int j = 0; j < 4; j++)
        for (int i = 0; i < 7 && n; i++, n--)
            s += sp[j][i];
    sp[0][0] = (uint8_t)(-s);
    hdr[3] = hdmi_bch3(hdr);
    for (int j = 0; j < 4; j++) sp[j][7] = hdmi_bch7(sp[j]);
}

static void hdmi_build_audio_if_blob(void) {
    uint8_t hdr[4] = { 0x84, 0x01, 0x0A, 0 };
    uint8_t sp[4][8];
    nf_memset(sp, 0, sizeof(sp));
    sp[0][1] = 0x11;  // CT=1 (PCM), CC=1 (2 channels)
    sp[0][2] = (HDMI_AUDIO_SF_CODE << 2) | 0x01;  // SF=HDMI_AUDIO_FS, SS=1 (16-bit). Matches ACR + actual output rate.
    sp[0][4] = 0x00;  // CA = FL/FR
    sp[0][5] = 0x00;  // LSV=0, DM_INH=0
    hdmi_if_checksum(hdr, sp);
    hdmi_pack_blob(blob_audio_if, hdr, sp);
}

static void hdmi_build_avi_if_blob(int active_width, int active_height) {
    uint8_t hdr[4] = { 0x82, 0x02, 0x0D, 0 };
    uint8_t sp[4][8];
    nf_memset(sp, 0, sizeof(sp));
    // VIC: declares a CEA timing the sink validates against. Our pixel clock is
    // a non-standard 25.2 MHz for ALL modes (the 720-wide modes are NOT real
    // 720x480p60/720x576p50 — those need 27 MHz). Declaring VIC=2/17 makes
    // strict sinks (Philips/Sony) cross-check the timing, fail it, and drop
    // audio (or fall back to DVI). Send VIC=0 ("no specific format") for the
    // 720-wide modes so the sink takes the timing as-is — what frank-nes does
    // for its non-standard timings (works on Philips). 640x480 stays VIC=1: it
    // genuinely is ~640x480p60 and that timing validates.
    uint8_t vic;
    if (active_width == 720) vic = 0;
    else vic = 1;
    (void)active_height;
    sp[0][1] = 0x00;  // Y=0 RGB, no scan/active-format info
    sp[0][2] = 0x08;  // R=8 same-as-picture
    sp[0][3] = 0x00;  // Q=0 default range
    sp[0][4] = vic;
    sp[0][5] = 0x00;  // no pixel repetition
    hdmi_if_checksum(hdr, sp);
    hdmi_pack_blob(blob_avi_if, hdr, sp);
}

static void hdmi_build_vendor_if_blob(void) {
    uint8_t hdr[4] = { 0x81, 0x01, 0x05, 0 };
    uint8_t sp[4][8];
    nf_memset(sp, 0, sizeof(sp));
    sp[0][1] = 0x03;  // IEEE OUI 0x000C03 (HDMI Licensing), LSB first
    sp[0][2] = 0x0C;
    sp[0][3] = 0x00;
    hdmi_if_checksum(hdr, sp);
    hdmi_pack_blob(blob_vendor_if, hdr, sp);
}

// ---------- audio sample packets (IEC 60958 framing) ----------

// IEC 60958 consumer channel-status block, 192 frames = 24 bytes:
// byte0 0x04 = consumer/PCM/copyright-not-asserted, byte3 = 0x02 (48 kHz),
// byte4 = 0xD2 (16-bit word length, original fs 48 kHz)
static inline uint8_t hdmi_cs_bit(int frame_index) {
    uint8_t byte;
    switch (frame_index >> 3) {
        case 0:  byte = 0x04; break;
        case 3:  byte = 0x02; break;
        case 4:  byte = 0xD2; break;
        default: byte = 0x00; break;
    }
    return (byte >> (frame_index & 7)) & 1;
}

static int hdmi_iec_frame_ct = 0;  // PICO-BK convention: frame_index = (192 - ct) & 191

// Encode one Audio Sample packet (4 samples L+R, 16-bit) from the ring buffer.
// Runs on core0 in the 31250 Hz pcm timer IRQ (every 4th tick, ~6 µs with LUTs).
static void __not_in_flash_func(hdmi_encode_audio_blob)(hdmi_audio_pkt_t *out) {
    int ct = hdmi_iec_frame_ct;
    uint8_t *hdr = out->hdr;
    hdr[0] = 0x02;
    hdr[1] = 0x0F;                              // layout=0, samples 0-3 present
    hdr[2] = (ct < 4 ? (1 << ct) : 0) << 4;     // B.x: IEC block start flags
    hdr[3] = hdmi_bch3(hdr);

    uint32_t rd = hdmi_audio_rd;
    for (int s = 0; s < 4; s++) {
        int16_t l = hdmi_audio_ring_L[(rd + s) & HDMI_AUDIO_RING_MASK];
        int16_t r = hdmi_audio_ring_R[(rd + s) & HDMI_AUDIO_RING_MASK];
        // No channel-status bits (C=0, V=0, U=0) — exactly what SpeccyP
        // transmits; the sink takes the rate from ACR + Audio InfoFrame
        uint8_t *d = out->sp[s];
        d[0] = 0;                               // 24-bit sample, low byte 0
        d[1] = l & 0xFF;
        d[2] = (l >> 8) & 0xFF;
        d[3] = 0;
        d[4] = r & 0xFF;
        d[5] = (r >> 8) & 0xFF;
        uint8_t pl = hdmi_parity8(d[1]) ^ hdmi_parity8(d[2]);
        uint8_t pr = hdmi_parity8(d[4]) ^ hdmi_parity8(d[5]);
        d[6] = (pl << 3) | (pr << 7);
        d[7] = hdmi_bch7(d);
        if (--ct < 0) ct = 191;
    }
    hdmi_iec_frame_ct = ct;
    hdmi_audio_rd = rd + 4;
    // TERC4 encoding deferred to the ISR (hdmi_di_load) — the raw 36-byte
    // packet sits in the queue until popped.
}

// ---------- ISR-side packet loader ----------

// Copy the packet for logical line `l` into conv_color data set `set`.
// AVI/Vendor/Audio InfoFrames go out once per frame on vblank lines 2..4 (or
// lines 2..4 when the mode has no vblank); ACR goes out on vblank line 1 and
// every 4th vblank line after (strict sinks need a dense ACR stream); audio
// packets whenever the producer queue has one; Null packet otherwise.
static void __not_in_flash_func(hdmi_di_load)(uint set, uint logical_line) {
    const uint64_t *src = NULL;
    const hdmi_audio_pkt_t *qpkt = NULL;
    bool from_q = false;

    // The line-clock audio meter credit (hdmi_au_pos) is accrued by the ISR
    // once per line pair — this loader only SPENDS it, because with scanlines
    // enabled it runs once (not twice) per pair.

    // Vsync pairs transmit the VSYNC=0 Null packet (no queue pop). Pair-level
    // check: the pair's render line is the odd one.
    const uint pr = (logical_line & 1) ? logical_line : logical_line - 1;
    const uint vbl = (di_if_base != 0 && logical_line > di_if_base)
                         ? logical_line - di_if_base
                         : (di_if_base == 0 ? logical_line : 0);
    if ((pr >= di_vs_start) && (pr < di_vs_end)) src = blob_null_vs;
    else if (vbl == 1) src = blob_acr;                          // ACR — first of many per frame (see below)
    else if (vbl == 2) src = blob_avi_if;                       // AVI once per frame
    else if (vbl == 3) src = blob_vendor_if;                    // Vendor Specific (HDMI mode signal — keeps sink in HDMI, not DVI)
    else if (vbl == 4) src = blob_audio_if;                     // Audio InfoFrame once per frame
    else if (vbl != 0 && (vbl & 3) == 0) src = blob_acr;        // extra ACR every 4th vblank line (vbl 8,12,16,...). Strict sinks (Philips/Sony) recover the audio clock from the ACR stream and need it FAR more often than once/frame to lock the PLL — they stay silent at 1/frame even though lenient sinks (Xiaomi) accept it. frank-nes/SpeccyP send ACR this often. Displaced audio-packet lines are recovered by the credit metering (long-run delivery unchanged).
    else if (hdmi_au_pos >= (4u << 24)) {
        // Credit for one packet (4 samples) available. Pop only when the
        // queue has one (and the depth is sane; a desynced/wrapped pair —
        // defensive, see hdmi_audio_init — is treated as empty). Credit is
        // spent ONLY on an actual pop: an empty queue emits Null and keeps
        // the credit (capped by the ISR), so emission resumes the moment the
        // producer catches up.
        const uint32_t qd = aq_wr - aq_rd;
        if (qd != 0 && qd <= HDMI_AQ_LEN) {
            hdmi_au_pos -= (4u << 24);
            qpkt = &aq_blob[aq_rd & (HDMI_AQ_LEN - 1)];
            from_q = true;
        }
        else src = blob_null;
    }
    else src = blob_null;

    uint64_t *dst = ((uint64_t *)conv_color) + (set ? IDX_DI_DATA2_BASE : IDX_DI_DATA_BASE) * 2;
    if (from_q) {
        // Encode the raw audio packet straight into conv_color (replaces the
        // 32-uint64 copy — see hdmi_audio_pkt_t). hdmi_pack_blob writes all 32.
        hdmi_pack_blob(dst, qpkt->hdr, qpkt->sp);
        __dmb();
        aq_rd = aq_rd + 1;
    } else {
        for (int i = 0; i < 32; i++) dst[i] = src[i];
    }
}

// ---------- init ----------

// Pick ACR N/CTS for the real TMDS pixel clock (clk_sys / (pio_div * 10)).
// CEA-861-F Table 7-1 recommended N for 48 kHz is 6144; at 25.2 MHz it gives
// CTS=25200. Fall back to other N if the clock doesn't divide cleanly.
static void hdmi_pick_acr(uint32_t pix_hz, uint32_t *n_out, uint32_t *cts_out) {
    const uint64_t denom = 128ull * HDMI_AUDIO_FS;  // 4,096,000
    static const uint16_t n_cand[] = { 4096, 6144, 6000, 6720, 5120, 12288 };
    for (unsigned i = 0; i < sizeof(n_cand) / sizeof(n_cand[0]); i++) {
        uint64_t num = (uint64_t)pix_hz * n_cand[i];
        if (num % denom == 0) {
            *n_out = n_cand[i];
            *cts_out = (uint32_t)(num / denom);
            return;
        }
    }
    *n_out = 6144;
    *cts_out = (uint32_t)(((uint64_t)pix_hz * 6144 + denom / 2) / denom);
}

static void __attribute__((noinline)) hdmi_audio_hw_init(void) {
    // Buffers not allocated (subsystem off / OOM) — audio stays disabled
    if (!aq_blob) return;
    struct video_mode_t mode = graphics_get_video_mode(get_video_mode());
    // Island block (22 bytes) + ≥2 bytes control + video preamble (4) + guard (1)
    // must fit into hsync + back porch
    if (mode.h_sync_bytes + mode.h_bp_bytes < 31) return;

    // All conv_color entries and static blobs are constants of the video mode
    // (which can't change without a reboot). On subsequent resets, skip the
    // write entirely: overwriting conv_color preamble/guard entries while the
    // core1 HDMI ISR is reading them produces a torn 64-bit symbol → invalid
    // TERC4 → TV loses signal lock (picture goes black).
    if (hdmi_audio_enabled) return;

    hdmi_build_terc_luts();

    // ch0 of the 32 packet characters: D0=HSYNC level, D1=VSYNC level, D3=1
    // except the first character. Line layout (SpeccyP placement — the one
    // this TV's audio decoder accepted): bytes [0..3] DI preamble (inside the
    // hsync pulse), [4] leading guard, [5..20] packet (px 10..41), [21]
    // trailing guard. On 32px-sync modes the tail crosses into the back
    // porch (level 1) — the per-character bake handles both.
    const int hs_px = mode.h_sync_bytes * 2;
    for (int pc = 0; pc < 32; pc++) {
        const int px = 10 + pc;  // 8px preamble + 2px leading guard before the packet
        const uint8_t h = (px < hs_px) ? 0 : 1;
        di_ch0_data[pc]    = h | 0b0010 | (pc == 0 ? 0 : 0b1000);
        di_ch0_data_vs[pc] = h |          (pc == 0 ? 0 : 0b1000);
    }
    di_if_base = (mode.v_active < mode.v_total) ? (uint)mode.v_active : 0;
    di_vs_start = (uint)mode.vsync_start;
    di_vs_end = (uint)mode.vsync_end;

    // Static conv_color entries. Argument order of get_ser_diff_data is
    // (ch2, ch1, ch0) — ch0 carries syncs.
    uint64_t *cc = (uint64_t *)conv_color;
    const uint16_t CTL00 = 0b1101010100;        // CTLx = {0,0}
    const uint16_t CTL01 = 0b0010101011;        // CTLx = {0,1}
    const uint16_t SYNC_H1V1 = 0b1010101011;    // ch0 control: blanking idle
    const uint16_t SYNC_H0V1 = 0b0101010100;    // ch0 control: hsync pulse
    const uint16_t SYNC_H0V0 = 0b1101010100;    // ch0 control: hsync pulse, vsync line
    const uint16_t GB_DI = 0b0100110011;        // data island guard, ch1/ch2
    const uint16_t GB_VID = 0b1011001100;       // video guard, ch0/ch2

    // Data island preamble: CTL0=1 CTL1=0 CTL2=1 CTL3=0; px 0..7, inside the
    // hsync pulse (ch0 carries the asserted sync levels)
    uint64_t pre = get_ser_diff_data(CTL01, CTL01, SYNC_H0V1);
    cc[IDX_DI_PREAMBLE * 2] = pre;
    cc[IDX_DI_PREAMBLE * 2 + 1] = pre;
    uint64_t pre_vs = get_ser_diff_data(CTL01, CTL01, SYNC_H0V0);
    cc[IDX_DI_PREAMBLE_VS * 2] = pre_vs;
    cc[IDX_DI_PREAMBLE_VS * 2 + 1] = pre_vs;
    // Island guard bands: ch0 = TERC4({1,1,VSYNC,HSYNC}), ch1=ch2=0b0100110011.
    // Leading guard px 8,9 (inside hsync); trailing guard px 42,43.
    for (int p = 0; p < 2; p++) {
        const uint8_t h_trail = ((42 + p) < hs_px) ? 0 : 1;
        cc[IDX_DI_GUARD * 2 + p]          = get_ser_diff_data(GB_DI, GB_DI, terc4_table[0b1110]);
        cc[IDX_DI_GUARD_TRAIL * 2 + p]    = get_ser_diff_data(GB_DI, GB_DI, terc4_table[0b1110 | h_trail]);
        cc[IDX_DI_GUARD_VS * 2 + p]       = get_ser_diff_data(GB_DI, GB_DI, terc4_table[0b1100]);
        cc[IDX_DI_GUARD_TRAIL_VS * 2 + p] = get_ser_diff_data(GB_DI, GB_DI, terc4_table[0b1100 | h_trail]);
    }
    // Video preamble: CTL0=1 CTL1=0 CTL2=0 CTL3=0; sits at the end of back porch
    uint64_t vp = get_ser_diff_data(CTL00, CTL01, SYNC_H1V1);
    cc[IDX_VIDEO_PREAMBLE * 2] = vp;
    cc[IDX_VIDEO_PREAMBLE * 2 + 1] = vp;
    // Video guard band: fixed 10-bit patterns per HDMI 1.4b §5.2.2.1
    uint64_t vg = get_ser_diff_data(GB_VID, GB_DI, GB_VID);
    cc[IDX_VIDEO_GUARD * 2] = vg;
    cc[IDX_VIDEO_GUARD * 2 + 1] = vg;

    // The static scanline (gray) buffer plays as active video — it needs the
    // video preamble + guard band too (no island: its line head stays control)
    {
        const int bp_end = mode.h_sync_bytes + mode.h_bp_bytes;
        for (int i = 0; i < 4; i++) hdmi_scanline_buf[bp_end - 5 + i] = IDX_VIDEO_PREAMBLE;
        hdmi_scanline_buf[bp_end - 1] = IDX_VIDEO_GUARD;
    }

    // ACR for the real pixel clock; 31250 Hz is declared via N/CTS only
    uint32_t pix_hz = (uint32_t)((float)clock_get_hz(clk_sys) / (mode.pio_clk_div * 10.0f));
    uint32_t acr_n, acr_cts;
    hdmi_pick_acr(pix_hz, &acr_n, &acr_cts);

    // Bang-bang credit rates per transmitted line, .24 fixed point. pix_hz is
    // the TMDS pixel clock; each line buffer byte (conv_color symbol) covers 2
    // pixels, so lines/sec = pix_hz / (2 * line_bytes). Nominal credit is
    // Fs * 2^24 / lines_per_sec; hi/lo = ±1% (see hdmi_au_spl24_hi declaration).
    const uint32_t lines_per_sec = pix_hz / (2u * (uint32_t)mode.line_bytes);
    const uint32_t spl24 = (uint32_t)(((uint64_t)HDMI_AUDIO_FS * (1u << 24)
                                       + lines_per_sec / 2) / lines_per_sec);
    hdmi_au_spl24_hi = spl24 + spl24 / 100;
    hdmi_au_spl24_lo = spl24 - spl24 / 100;

    // Credit ceiling = enough to bank the longest no-pop run (InfoFrame burst +
    // vsync) without clamping; clamping there under-delivers and the sink mutes
    // (576p50 @48kHz: 9-line contiguous run, vsync_start right after the 4
    // InfoFrame lines). InfoFrames span 4 lines from di_if_base+1 (lines 1..4
    // when no vblank); vsync is [vsync_start, vsync_end). The runs merge into
    // one when adjacent/overlapping, else the longest is max(4, vsync width).
    const uint if0 = (di_if_base ? di_if_base : 0) + 1;
    const uint if1 = if0 + 4;
    const uint vs0 = (uint)mode.vsync_start, vs1 = (uint)mode.vsync_end;
    uint nopop_run;
    if (if0 <= vs1 && vs0 <= if1) {                       // merged run
        const uint lo = if0 < vs0 ? if0 : vs0;
        const uint hi = if1 > vs1 ? if1 : vs1;
        nopop_run = hi - lo;
    } else {                                              // disjoint
        const uint vw = vs1 - vs0;
        nopop_run = vw > 4 ? vw : 4;
    }
    // run*spl24_hi (per-line credit at the +1% accrual) + 2 packets headroom
    // for start-of-run leftover and pair quantization. The post-run catch-up is
    // bounded by the queue backlog, not this cap, so the headroom is harmless.
    uint64_t cap = (uint64_t)nopop_run * hdmi_au_spl24_hi + (8u << 24);
    hdmi_au_cap = cap < (8u << 24) ? (8u << 24) : (uint32_t)cap;

    printf("hdmi_audio_hw_init: mode=%d hs=%d bp=%d v_act=%d v_tot=%d pix=%lu N=%lu CTS=%lu lps=%lu spl24=%lu run=%u cap=%lu\n",
           get_video_mode(), mode.h_sync_bytes, mode.h_bp_bytes, mode.v_active, mode.v_total,
           (unsigned long)pix_hz, (unsigned long)acr_n, (unsigned long)acr_cts,
           (unsigned long)lines_per_sec, (unsigned long)spl24,
           nopop_run, (unsigned long)hdmi_au_cap);

    hdmi_build_null_blob();
    hdmi_build_acr_blob(acr_cts, acr_n);
    hdmi_build_audio_if_blob();
    hdmi_build_avi_if_blob(mode.screen_width * 2, mode.v_active);
    hdmi_build_vendor_if_blob();

    if (!hdmi_audio_enabled) {
        // Null-fill only on first init: once the ISR is running it manages both
        // sets itself, and a concurrent memcpy would produce a torn BCH packet
        // that makes the TV mute for ~0.5 s.
        memcpy(&cc[IDX_DI_DATA_BASE * 2],  blob_null, sizeof(blob_null));
        memcpy(&cc[IDX_DI_DATA2_BASE * 2], blob_null, sizeof(blob_null));
        hdmi_au_pos = 0;
        // Pre-fill the latency cushion with silent audio packets (the sample
        // ring is still zeroed — audio isn't enabled yet, so no concurrency):
        // the full target depth exists from the very first emitted island and
        // the bang-bang never has to build it by underpacing real audio.
        for (int i = 0; i < HDMI_AU_TARGET; i++)
            hdmi_encode_audio_blob(&aq_blob[(aq_wr + i) & (HDMI_AQ_LEN - 1)]);
        aq_wr = aq_wr + HDMI_AU_TARGET;
        hdmi_audio_rd = hdmi_audio_wr;  // encoding consumed zeroed ring; rewind
        __dmb();
        hdmi_audio_enabled = true;
    }
}

int hdmi_audio_dbg_stage(void) {
#if HDMI_AUDIO_DEBUG_STAGES
    if (!hdmi_audio_enabled) return -1;
    return (hdmi_dbg_frame_ct >> 10) & 3;
#else
    return -1;
#endif
}

void hdmi_audio_dbg_stats(uint32_t *q_prod, uint32_t *q_cons, uint32_t *s_prod, uint32_t *s_cons) {
    *q_prod = aq_wr;
    *q_cons = aq_rd;
    *s_prod = hdmi_audio_wr;
    *s_cons = hdmi_audio_rd;
}

bool hdmi_audio_init(void) {
    // init_sound() re-runs on every machine reset/arch switch. Once audio is
    // live, the SPSC indices are owned by the core0 timer IRQ (producer) and
    // the core1 video ISR (consumer) — zeroing them here races their
    // increments (lost update leaves aq_rd "ahead" of aq_wr, wedging the
    // queue forever and feeding garbage blobs to the sink). Reset only on
    // the first bring-up.
    if (!hdmi_audio_enabled) {
        if (!aq_blob) {
            // Raw packet queue + both sample rings in one block (~8.6 KB):
            // 128x36 B queue (raw, not TERC4-encoded) + 2x1024x2 B rings.
            // Paid only while HDMI audio is enabled.
            uint8_t *blk = (uint8_t *)calloc(HDMI_AQ_LEN * sizeof(hdmi_audio_pkt_t)
                                             + 2 * HDMI_AUDIO_RING_SIZE * sizeof(int16_t), 1);
            if (!blk) return false;
            aq_blob = (hdmi_audio_pkt_t *)blk;
            hdmi_audio_ring_L = (volatile int16_t *)(blk + HDMI_AQ_LEN * sizeof(hdmi_audio_pkt_t));
            hdmi_audio_ring_R = hdmi_audio_ring_L + HDMI_AUDIO_RING_SIZE;
        }
        hdmi_audio_wr = 0;
        hdmi_audio_rd = 0;
        aq_wr = 0;
        aq_rd = 0;
        nf_memset((void *)hdmi_audio_ring_L, 0, HDMI_AUDIO_RING_SIZE * sizeof(int16_t));
        nf_memset((void *)hdmi_audio_ring_R, 0, HDMI_AUDIO_RING_SIZE * sizeof(int16_t));
    }
    // Init order safe both ways: graphics_init_hdmi() (core1 startup) calls
    // hdmi_audio_hw_init() when the flag is already set; if init_sound() runs
    // later, this direct call brings the hardware tables up instead.
    hdmi_audio_hw_init();
    return hdmi_audio_enabled;
}

void hdmi_audio_deinit(void) {
    if (hdmi_audio_enabled) {
        hdmi_audio_enabled = false;
        __dmb();
        // An in-flight core1 scanline ISR that sampled the flag before the
        // clear may still read aq_blob; it finishes within one line period
        // (~32 µs) — wait that out with margin before freeing.
        busy_wait_us(200);
    }
    free((void *)aq_blob);  // single block, see hdmi_audio_init
    aq_blob = NULL;
    hdmi_audio_ring_L = NULL;
    hdmi_audio_ring_R = NULL;
}

void __not_in_flash_func(hdmi_audio_write_sample)(int16_t left, int16_t right) {
    if (!hdmi_audio_enabled) return;
    // Bresenham upsample 31250 -> 48000 with linear interpolation between
    // consecutive input samples (plain sample-and-hold leaves audible HF
    // imaging on AY tones). Long-run output rate is exactly 48000/31250.
    static uint32_t accum = 0;
    static int16_t prevL = 0, prevR = 0;
    uint32_t wr = hdmi_audio_wr;
    accum += HDMI_AUDIO_FS;
    while (accum >= HDMI_AUDIO_FS_IN) {
        accum -= HDMI_AUDIO_FS_IN;
        // Output position inside the (prev, current) input interval:
        // f = (FS - accum) / FS in [0..1] (16-bit fixed point)
        const uint32_t f16 = ((HDMI_AUDIO_FS - accum) << 16) / HDMI_AUDIO_FS;
        const int16_t l = (int16_t)(prevL + (((int32_t)(left - prevL) * (int32_t)f16) >> 16));
        const int16_t r = (int16_t)(prevR + (((int32_t)(right - prevR) * (int32_t)f16) >> 16));
        hdmi_audio_ring_L[wr & HDMI_AUDIO_RING_MASK] = l;
        hdmi_audio_ring_R[wr & HDMI_AUDIO_RING_MASK] = r;
        wr++;
    }
    prevL = left;
    prevR = right;
    hdmi_audio_wr = wr;
    uint32_t avail = wr - hdmi_audio_rd;
    // If the consumer stalled (paranoia — the core1 ISR drains every line,
    // including DS80 mode), drop the backlog instead of replaying
    // half-overwritten ring data afterwards
    if (avail > 256) {
        hdmi_audio_rd = wr - 4;
        avail = 4;
    }
    // Encode up to 2 packets per tick (steady state needs ~0.4/tick; the cap
    // bounds IRQ time while still allowing catch-up after a backlog)
    for (int k = 0; k < 2 && avail >= 4 && (aq_wr - aq_rd) < HDMI_AQ_LEN; k++) {
        hdmi_encode_audio_blob(&aq_blob[aq_wr & (HDMI_AQ_LEN - 1)]);
        __dmb();
        aq_wr = aq_wr + 1;
        avail = hdmi_audio_wr - hdmi_audio_rd;
    }
}

#endif // !PICO_RP2040

