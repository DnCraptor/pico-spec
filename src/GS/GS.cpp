#ifdef USE_GS

#include "GS.h"
#include "GS_ROM.h"
#include "Debug.h"

extern "C" {
#include "Z80_redcode.h"
}

#include "pico.h"
#include "pico/time.h"
#include "hardware/sync.h"
#include <string.h>

// Atomic byte OR/AND via GCC __atomic builtins — compile to LDREXB/STREXB on
// ARM Cortex-M33, which is safe across both RP2350 cores sharing the AHB bus.
// reg_status is shared between core0 (host) and core1 (GS-Z80 emulation);
// plain |= / &= are non-atomic RMW and can silently lose bits under concurrent access.
static inline void gs_status_or(volatile uint8_t* p, uint8_t bits) {
    __atomic_fetch_or(p, bits, __ATOMIC_SEQ_CST);
}
static inline void gs_status_and(volatile uint8_t* p, uint8_t mask) {
    __atomic_fetch_and(p, mask, __ATOMIC_SEQ_CST);
}

extern uint8_t* PSRAM_DATA;
extern uint32_t butter_psram_size();

#define GS_DIAG(fmt, ...) do {} while(0)
#define GS_DIAG_HOST(fmt, ...) do {} while(0)
extern int butter_pages;

#include "MemESP.h"
#include "DivMMC.h"

bool     GS::enabled       = false;
uint32_t GS::gs_ram_size   = 0;
volatile uint8_t  GS::reg_command   = 0;
volatile uint8_t  GS::reg_data_zx   = 0;
volatile uint8_t  GS::reg_data_gs   = 0;
volatile uint8_t  GS::reg_status    = 0;
uint8_t  GS::reg_page      = 0;
uint8_t  GS::reg_vol[4]    = {0,0,0,0};
uint8_t  GS::reg_ch[4]     = {0x80,0x80,0x80,0x80};
uint32_t GS::int_count     = 0;

static Z80      s_cpu;
static uint8_t* s_gs_ram      = nullptr;
static uint32_t s_gs_ram_mask = 0;
static uint32_t s_int_timer_ts = 0;
static bool     s_int_pending  = false;



// 16 KB work-RAM (CPU 0x4000-0x7FFF: stack, variables, DAC mirrors) kept in
// SRAM to avoid PSRAM latency on every stack push/pop and tight-loop variable
// access.  Banked sample pages (CPU 0x8000-0xFFFF, page>0) stay in PSRAM.
static uint8_t s_gs_work_ram[0x4000];

// DAC snapshot ring buffer. Producer: step() pushes at each INT (37500 Hz
// avg, jittery during core1 stalls). Consumer: pcm_call_inner timer IRQ at
// steady 31250 Hz with 6:5 fractional decimation. 1024 entries ≈ 27 ms of
// GS-time — absorbs typical 10-25 ms core1 slowdowns (heavy core0 PSRAM use,
// sustained OSD activity) without draining. Single-writer (core1 main) /
// single-reader (core1 IRQ preempts main) — volatile wpos/rpos atomic on ARM.
#define GS_RING_SIZE 1024
#define GS_RING_MASK (GS_RING_SIZE - 1)
static int16_t s_ring_L[GS_RING_SIZE];
static int16_t s_ring_R[GS_RING_SIZE];
static volatile uint32_t s_ring_wpos = 0;
static volatile uint32_t s_ring_rpos = 0;
static uint32_t s_drain_frac = 0;

// Real GS hardware is 12 MHz, INT every 320 T = 37500 Hz. Unreal runs at
// 24/640 for extra compute headroom, but our redcode emulator on PSRAM-backed
// RAM caps at ~18 T/µs effective, so 24 MHz target under-delivers (measured
// ~77% → 29 kHz IRQ, pitch shifted ~25% low). 12 MHz + 320 T is well within
// capacity and matches native GS firmware timing exactly.
static constexpr uint32_t GS_CLOCK_HZ    = 12000000;
static constexpr uint32_t GS_INT_HZ      = 37500;
static constexpr uint32_t GS_INT_PERIOD  = GS_CLOCK_HZ / GS_INT_HZ;  // 320

static inline uint32_t __not_in_flash_func(gs_map_addr)(uint16_t address) {
    return (uint32_t)(GS::reg_page - 1) * 0x8000u + (address - 0x8000u);
}

// Private SRAM cache for PSRAM (banked sample pages). XIP cache on RP2350 is
// shared between cores — core0 video rendering evicts lines we need. A small
// private cache in SRAM is immune to that contention.
//
// 4-way set-associative, FIFO replacement: 16 sets × 4 ways × 64 bytes = 4 KB.
// 4-way associativity is critical for 4-channel playback: all 4 channels tend
// to read at the same relative offset within their respective 32 KB pages, so
// they all map to the same direct-mapped set and thrash each other. With 4 ways,
// all 4 channels' active lines coexist without eviction.
#define GS_PC_LINE_SZ    64
#define GS_PC_LINE_BITS  6                         // log2(64)
#define GS_PC_LINE_MASK  (GS_PC_LINE_SZ - 1)
#define GS_PC_SETS       16
#define GS_PC_WAYS       4
#define GS_PC_SETS_MASK  (GS_PC_SETS - 1)
static uint8_t  s_pc_data[GS_PC_SETS][GS_PC_WAYS][GS_PC_LINE_SZ];  // 4 KB
static uint32_t s_pc_tag[GS_PC_SETS][GS_PC_WAYS];   // ~0u = invalid
static uint8_t  s_pc_next[GS_PC_SETS];               // FIFO victim pointer

static inline void __not_in_flash_func(gs_pc_invalidate_line)(uint32_t psram_off) {
    uint32_t line = psram_off >> GS_PC_LINE_BITS;
    uint32_t set  = line & GS_PC_SETS_MASK;
    for (int w = 0; w < GS_PC_WAYS; w++) {
        if (s_pc_tag[set][w] == line) { s_pc_tag[set][w] = ~0u; return; }
    }
}

static inline zuint8 __not_in_flash_func(gs_pc_read)(uint32_t psram_off) {
    uint32_t line = psram_off >> GS_PC_LINE_BITS;
    uint32_t set  = line & GS_PC_SETS_MASK;
    uint32_t col  = psram_off & GS_PC_LINE_MASK;
    for (int w = 0; w < GS_PC_WAYS; w++) {
        if (s_pc_tag[set][w] == line) return s_pc_data[set][w][col];   // hit
    }
    // Miss — FIFO eviction, bulk-copy 64 bytes from PSRAM (XIP burst friendly).
    uint8_t v = s_pc_next[set];
    s_pc_next[set] = (v + 1) & (GS_PC_WAYS - 1);
    memcpy(s_pc_data[set][v], &s_gs_ram[line << GS_PC_LINE_BITS], GS_PC_LINE_SZ);
    s_pc_tag[set][v] = line;
    return s_pc_data[set][v][col];
}

static inline zuint8 __not_in_flash_func(gs_mem_raw_read)(zuint16 address) {
    if (address < 0x4000) return ROM_GS_M[address];
    if (address < 0x8000) return s_gs_work_ram[address - 0x4000];
    if (GS::reg_page == 0) return ROM_GS_M[address - 0x8000];
    // Banked sample pages — use private SRAM cache to survive core0 XIP thrash.
    uint32_t off = (gs_map_addr(address) + 0x4000) & s_gs_ram_mask;
    return gs_pc_read(off);
}

static zuint8 __not_in_flash_func(gs_cb_read)(void* ctx, zuint16 address) {
    (void)ctx;
    zuint8 v = gs_mem_raw_read(address);
    // DAC latch: any DATA read from 0x6000-0x7FFF feeds the fetched byte to
    // channel (addr>>8)&3. Standard firmware streams samples via LD A,(HL).
    // Opcode/operand fetches are NOT counted (different callback below).
    if ((address & 0xE000) == 0x6000) {
        GS::reg_ch[(address >> 8) & 3] = v;
    }
    return v;
}

static void __not_in_flash_func(gs_cb_write)(void* ctx, zuint16 address, zuint8 value) {
    (void)ctx;
    if (address < 0x4000) return;
    if (address < 0x8000) {
        s_gs_work_ram[address - 0x4000] = value;  // SRAM — no PSRAM latency
        return;
    }
    if (GS::reg_page == 0) return;
    uint32_t off = (gs_map_addr(address) + 0x4000) & s_gs_ram_mask;
    s_gs_ram[off] = value;  // PSRAM — banked sample pages
    gs_pc_invalidate_line(off);  // keep SRAM cache coherent
}

static zuint8 __not_in_flash_func(gs_cb_in)(void* ctx, zuint16 port) {
    (void)ctx;
    uint8_t p = port & 0x0F;
    uint8_t v;
    switch (p) {
        case 0x01:
            v = GS::reg_command;
            // D0 stays set until GS explicitly acks via IN/OUT 0x05
            break;
        case 0x02:
            v = GS::reg_data_zx;
            // DMB: ensure we read reg_data_zx before clearing the "data ready" flag
            // so the host (core0) cannot overwrite it before we've consumed it.
            __dmb();
            gs_status_and(&GS::reg_status, ~0x80u);
            break;
        case 0x03:
            // Per UnrealSpeccy gsz80.cpp: also stores 0xFF into reg_data_gs.
            // Otherwise a stale prior response would be returned to ZX on
            // next hostReadB3, breaking GS→ZX protocol state.
            GS::reg_data_gs = 0xFF;
            gs_status_or(&GS::reg_status, 0x80u);
            v = 0xFF;
            break;
        case 0x04:
            v = GS::reg_status; break;
        case 0x05:
            gs_status_and(&GS::reg_status, ~0x01u);
            v = 0xFF;
            break;
        case 0x0A:
            v = GS::reg_status;
            gs_status_and(&GS::reg_status, 0x7Fu);
            if (GS::reg_page & 0x01) gs_status_or(&GS::reg_status, 0x80u);
            break;
        case 0x0B:
            v = GS::reg_status;
            gs_status_and(&GS::reg_status, 0xFEu);
            if ((GS::reg_vol[0] >> 5) & 0x01) gs_status_or(&GS::reg_status, 0x01u);
            break;
        default:
            v = 0xFF; break;
    }
    if (p <= 0x0B) GS_DIAG("gsZ80 IN  %02X = %02X", p, v);
    return v;
}

static void __not_in_flash_func(gs_cb_out)(void* ctx, zuint16 port, zuint8 value) {
    (void)ctx;
    uint8_t p = port & 0x0F;
    // Skip page register (0x00) to avoid drowning the log during RAM test
    if (p > 0x00 && p <= 0x0B) GS_DIAG("gsZ80 OUT %02X <- %02X", p, value);
    switch (p) {
        case 0x00:
            GS::reg_page = value & 0x3F;
            return;
        case 0x02:
            gs_status_and(&GS::reg_status, ~0x80u);
            return;
        case 0x03:
            GS::reg_data_gs = value;
            __dmb();  // data must be visible to core0 before setting D7
            gs_status_or(&GS::reg_status, 0x80u);
            return;
        case 0x05:
            gs_status_and(&GS::reg_status, ~0x01u);  // command-ack: clear D0
            return;
        case 0x06: GS::reg_vol[0] = value & 0x3F; return;
        case 0x07: GS::reg_vol[1] = value & 0x3F; return;
        case 0x08: GS::reg_vol[2] = value & 0x3F; return;
        case 0x09: GS::reg_vol[3] = value & 0x3F; return;
        default:
            return;
    }
}

// Called by redcode for all three INT modes (IM0/IM1/IM2) at the INTA
// M-cycle — the moment the Z80 acknowledges the interrupt. We deassert
// INT_LINE here instead of after a blind 32T window; this implements the
// level-triggered model: INT stays asserted until acknowledged, so
// firmware running DI for >32T no longer silently drops the interrupt.
static zuint8 __not_in_flash_func(gs_cb_inta)(void* ctx, zuint16 pc) {
    (void)ctx; (void)pc;
    s_int_pending = false;
    z80_int(&s_cpu, Z_FALSE);
    return 0xFF;  // IM0: RST 38h  |  IM2: vector at (I<<8)|0xFF = 0x17FF → ISR
}

static zuint8 __not_in_flash_func(gs_cb_fetch_opcode)(void* ctx, zuint16 address) {
    (void)ctx;
    return gs_mem_raw_read(address);
}
static zuint8 __not_in_flash_func(gs_cb_fetch)(void* ctx, zuint16 address) {
    (void)ctx;
    return gs_mem_raw_read(address);
}
static zuint8 __not_in_flash_func(gs_cb_nop)(void* ctx, zuint16 address) {
    (void)ctx; (void)address;
    return 0;
}

bool GS::init(uint32_t ram_size_bytes) {
    if (enabled) return true;

    uint32_t rounded = 0x20000;
    while (rounded < ram_size_bytes && rounded < (2u << 20)) rounded <<= 1;
    ram_size_bytes = rounded;

    uint32_t psram = butter_psram_size();
    if (psram == 0) {
        Debug::log("GS::init: no butter PSRAM");
        return false;
    }

    size_t butter_used  = (size_t)butter_pages * MEM_PG_SZ;
    size_t divmmc_total = DivMMC::use_psram
        ? (size_t)DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE
        : 0;
    size_t reserved_below = butter_used + divmmc_total;

    if ((size_t)psram < reserved_below + ram_size_bytes) {
        Debug::log("GS::init: not enough PSRAM (need %u, have %u free)",
                   (unsigned)ram_size_bytes, (unsigned)(psram - reserved_below));
        return false;
    }

    s_gs_ram      = PSRAM_DATA + (psram - ram_size_bytes);
    s_gs_ram_mask = ram_size_bytes - 1;
    gs_ram_size   = ram_size_bytes;

    memset(s_gs_work_ram, 0, sizeof(s_gs_work_ram));
    for (int i = 0; i < GS_PC_SETS; i++) {
        for (int w = 0; w < GS_PC_WAYS; w++) s_pc_tag[i][w] = ~0u;
        s_pc_next[i] = 0;
    }

    memset(&s_cpu, 0, sizeof(s_cpu));
    s_cpu.context      = nullptr;
    s_cpu.fetch_opcode = gs_cb_fetch_opcode;
    s_cpu.fetch        = gs_cb_fetch;
    s_cpu.read         = gs_cb_read;
    s_cpu.write        = gs_cb_write;
    s_cpu.in           = gs_cb_in;
    s_cpu.out          = gs_cb_out;
    s_cpu.halt         = nullptr;
    s_cpu.nop          = gs_cb_nop;
    s_cpu.nmia         = nullptr;
    s_cpu.inta         = gs_cb_inta;
    s_cpu.int_fetch    = nullptr;
    s_cpu.ld_i_a       = nullptr;
    s_cpu.ld_r_a       = nullptr;
    s_cpu.reti         = nullptr;
    s_cpu.retn         = nullptr;
    s_cpu.hook         = nullptr;
    s_cpu.illegal      = nullptr;

    z80_power(&s_cpu, Z_TRUE);
    z80_instant_reset(&s_cpu);

    reset();
    enabled = true;
    Debug::log("GS::init: OK, ram=%u KB at +%u MB",
               (unsigned)(ram_size_bytes >> 10),
               (unsigned)((psram - ram_size_bytes) >> 20));
    return true;
}

void GS::deinit() {
    enabled = false;
    s_gs_ram = nullptr;
    s_gs_ram_mask = 0;
    gs_ram_size = 0;
}

void GS::reset() {
    reg_command = 0;
    reg_data_zx = 0;
    reg_data_gs = 0;
    reg_status  = 0;
    reg_page    = 0;
    for (int i = 0; i < 4; i++) { reg_vol[i] = 0; reg_ch[i] = 0x80; }
    int_count = 0;
    s_int_timer_ts = 0;
    s_int_pending = false;
    s_ring_wpos = 0;
    s_ring_rpos = 0;
    s_drain_frac = 0;
    for (int i = 0; i < GS_RING_SIZE; i++) { s_ring_L[i] = 0; s_ring_R[i] = 0; }
    for (int i = 0; i < GS_PC_SETS; i++) {
        for (int w = 0; w < GS_PC_WAYS; w++) s_pc_tag[i][w] = ~0u;
        s_pc_next[i] = 0;
    }
    if (enabled) {
        z80_instant_reset(&s_cpu);
    }
}

void __not_in_flash_func(GS::topUpBudget)(int tstates) {
    (void)tstates;
    // Budget mechanism removed: step() runs freely. GS-Z80 time is bounded
    // by how often the emulator gets called; we don't need an explicit cap.
}

void __not_in_flash_func(GS::pump)() {
    if (!enabled) return;
    // Ring-fill feedback pacing. Consumer (audio IRQ + fractional decimation)
    // drains at rock-steady 37500 entries/sec regardless of core1 jitter, so
    // producer auto-paces to match it via this check. Pitch = consumer rate
    // exactly. Wall-clock variance is absorbed by the ring (13 ms depth).
    uint32_t used = s_ring_wpos - s_ring_rpos;
    if (used >= (GS_RING_SIZE * 7 / 8)) return;   // ring near full: pause
    // Larger batch reduces per-call overhead (time_us, ring check, step setup).
    // 16 INTs ≈ 5120 T ≈ 280 µs — fine for core1 since tight loop has nothing
    // else time-critical (HDMI DMA/audio IRQ are hardware-driven, not polled).
    step((int)GS_INT_PERIOD * 16);
}

int __not_in_flash_func(GS::step)(int tstates) {
    if (!enabled) return 0;
    int remaining = tstates;
    int total_ran = 0;
    while (remaining > 0) {
        uint32_t until_int = GS_INT_PERIOD - s_int_timer_ts;
        uint32_t chunk = (remaining < (int)until_int) ? (uint32_t)remaining : until_int;
        zusize ran = z80_run(&s_cpu, chunk);
        if (ran == 0) ran = chunk;
        s_int_timer_ts += ran;
        remaining -= (int)ran;
        total_ran += (int)ran;
        if (s_int_timer_ts >= GS_INT_PERIOD) {
            s_int_timer_ts -= GS_INT_PERIOD;
            int_count++;
            // Level-triggered INT: assert only once per period; gs_cb_inta
            // deasserts when the Z80 acknowledges it. If firmware is in DI
            // when the period fires, INT_LINE stays high until EI executes
            // (redcode's EI: if (INT_LINE) REQUEST |= Z80_REQUEST_INT).
            // This matches real GS hardware and Unreal Speccy's int_pend model.
            if (!s_int_pending) {
                s_int_pending = true;
                z80_int(&s_cpu, Z_TRUE);
            }
            zusize ran_int = z80_run(&s_cpu, 32);
            if (ran_int == 0) ran_int = 32;
            s_int_timer_ts += ran_int;
            remaining -= (int)ran_int;
            total_ran += (int)ran_int;

            // Capture DAC snapshot into ring. Per Unreal, apply Unreal stereo
            // mix (L=0+1, R=2+3 with 50% cross-mix, /2 final scale).
            int32_t c0 = (int32_t)reg_vol[0] * ((int32_t)reg_ch[0] - 128);
            int32_t c1 = (int32_t)reg_vol[1] * ((int32_t)reg_ch[1] - 128);
            int32_t c2 = (int32_t)reg_vol[2] * ((int32_t)reg_ch[2] - 128);
            int32_t c3 = (int32_t)reg_vol[3] * ((int32_t)reg_ch[3] - 128);
            int32_t l = c0 + c1;
            int32_t r = c2 + c3;
            uint32_t w = s_ring_wpos;
            s_ring_L[w & GS_RING_MASK] = (int16_t)((l + (r >> 1)) >> 1);
            s_ring_R[w & GS_RING_MASK] = (int16_t)((r + (l >> 1)) >> 1);
            // DMB: ring data must be written before wpos is visible to the
            // consumer on core0 (pcm_call_inner timer IRQ).
            __dmb();
            s_ring_wpos = w + 1;

            // (Periodic IRQ-rate log disabled: Debug::log on core1 itself
            //  caused ~4ms blocking that confused jitter measurement.)
        }
    }
    return total_ran;
}

uint8_t GS::hostReadB3() {
    uint8_t v = reg_data_gs;
    __dmb();  // consume data before clearing the flag
    gs_status_and(&reg_status, ~0x80u);
    return v;
}

uint8_t GS::hostReadBB() {
    return reg_status | 0x7E;  // bits 1-6 always set (hardware signature)
}

// Backpressure: spin while D7 is still set (GS-Z80 hasn't read the previous
// byte yet). Without this, ZPlayer's bulk module load (16384 bytes/sec,
// ~60 µs interval) loses 1-6 bytes per second because core0 (host Z80)
// retires IO writes faster than core1 (GS-Z80 @ 12 MHz) can drain them. A
// lost byte shifts every following byte one slot down in PSRAM, leaving the
// loaded sample bank corrupt and causing channels to play garbage at random
// points in the module.
void GS::hostWriteB3(uint8_t data) {
    if (reg_status & 0x80u) {
        uint32_t spin_t0 = time_us_32();
        while ((reg_status & 0x80u) && (time_us_32() - spin_t0) < 1000) {
            __dmb();
        }
    }
    reg_data_zx = data;
    __dmb();  // data must be visible to core1 before D7 is set
    gs_status_or(&reg_status, 0x80u);
}

void GS::hostWriteBB(uint8_t data) {
    reg_command = data;
    __dmb();  // command must be visible to core1 before D0 is set
    gs_status_or(&reg_status, 0x01u);
}

int16_t __not_in_flash_func(GS::getSampleLeft)() {
    int l = (int)reg_vol[0] * ((int)reg_ch[0] - 128)
          + (int)reg_vol[3] * ((int)reg_ch[3] - 128);
    return (int16_t)l;
}

int16_t __not_in_flash_func(GS::getSampleRight)() {
    int r = (int)reg_vol[1] * ((int)reg_ch[1] - 128)
          + (int)reg_vol[2] * ((int)reg_ch[2] - 128);
    return (int16_t)r;
}

// Convert signed int16 DAC mix (roughly ±16000) to unsigned 0..255
// with silence at 128. >>7 fits ±125 into the byte; +128 biases.
static inline uint8_t gs_to_u8(int32_t v) {
    int32_t u = 128 + (v >> 7);
    if (u < 0)   u = 0;
    if (u > 255) u = 255;
    return (uint8_t)u;
}

void __not_in_flash_func(GS::getLiveLR)(uint8_t& L, uint8_t& R) {
    // Drain from ring with 6:5 fractional decimation (37500→31250).
    // Consumer rate: 31250 Hz (audio IRQ). Producer: 37500 Hz avg (INT).
    // Each call consumes 37500/31250 = 1.2 ring entries on average.
    uint32_t w = s_ring_wpos;
    // DMB: ensures we see the ring data that was written before wpos was
    // incremented by the producer on core1 (matched by dmb in step()).
    __dmb();
    uint32_t r = s_ring_rpos;
    uint32_t avail = w - r;

    if (avail == 0) {
        // Ring empty — hold last value (silence at startup).
        if (w == 0) { L = 128; R = 128; return; }
        uint32_t last = (w - 1) & GS_RING_MASK;
        L = gs_to_u8(s_ring_L[last]);
        R = gs_to_u8(s_ring_R[last]);
        return;
    }

    s_drain_frac += GS_INT_HZ;              // += 37500
    uint32_t n = s_drain_frac / 31250u;
    s_drain_frac %= 31250u;
    if (n == 0) n = 1;
    if (n > avail) n = avail;

    int32_t sumL = 0, sumR = 0;
    for (uint32_t i = 0; i < n; i++) {
        sumL += s_ring_L[(r + i) & GS_RING_MASK];
        sumR += s_ring_R[(r + i) & GS_RING_MASK];
    }
    L = gs_to_u8(sumL / (int32_t)n);
    R = gs_to_u8(sumR / (int32_t)n);
    s_ring_rpos = r + n;
}

#endif // USE_GS
