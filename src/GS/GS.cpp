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
// Wall-clock anchor for pump(). 0 means "uninitialized — sample on next
// pump call"; reset() clears it so a paused/restarted GS doesn't try to
// catch up time spent paused.
static uint32_t s_pump_last_us = 0;

// Cache stats: hit/miss counts per second to gauge how often gs_pc_read
// has to fetch a fresh 64-byte line from PSRAM (vs hitting in SRAM cache).
// High miss count during slow seconds = GS-Z80 thrashing PSRAM and
// fighting core0 for the bus.
static volatile uint32_t s_perf_pc_hit  = 0;
static volatile uint32_t s_perf_pc_miss = 0;

// Performance counters polled once per second by pollPerf() from core0.
// Collected on core1 in pump()/step()/gs_cb_in to give a snapshot of how
// busy GS-Z80 is and where its time goes; cross-correlated with core0's
// per-frame IDL minimum to spot stalls.
static volatile uint32_t s_perf_pump_calls = 0;     // total pump() entries
static volatile uint32_t s_perf_pump_skip  = 0;     // pump() returned early (ring full)
static volatile uint32_t s_perf_tstates    = 0;     // GS-Z80 T-states executed
static volatile uint32_t s_perf_p04_total  = 0;     // total IN port 04 reads
static volatile uint32_t s_perf_p04_spin   = 0;     // IN port 04 reads where PC == prev PC (spinwait)
static volatile uint16_t s_perf_p04_pc     = 0;     // last PC of port-04 read (for spinwait detection)

// Core0-side counters updated from ESPectrum.cpp main frame loop. extern
// so the cross-module reference stays simple.
volatile int32_t  gs_perf_idle_min        = 0x7FFFFFFF;
volatile uint32_t gs_perf_idle_neg_frames = 0;
volatile uint32_t gs_perf_frames          = 0;
// Core0-side host IO counters
static volatile uint32_t s_perf_h_b3w   = 0;
static volatile uint32_t s_perf_h_b3r   = 0;
static volatile uint32_t s_perf_h_bbw   = 0;
static volatile uint32_t s_perf_h_bbr   = 0;
static volatile uint32_t s_perf_h_spin_us = 0;     // total spinwait µs/sec in hostWriteB3




// 16 KB work-RAM (CPU 0x4000-0x7FFF: stack, variables, DAC mirrors) kept in
// SRAM to avoid PSRAM latency on every stack push/pop and tight-loop variable
// access.  Banked sample pages (CPU 0x8000-0xFFFF, page>0) stay in PSRAM.
static uint8_t s_gs_work_ram[0x4000];

// Host→GS FIFO. Some loaders (e.g. FH1_GS_TZ.scl) stream samples into port
// 0xB3 in a tight LD A,(HL) / OUT (B3),A loop with no IN (BB) handshake,
// running ~50000 bytes/sec — faster than the GS firmware's IN A,(2) per-INT
// drain rate (~37500/sec). Without buffering, every host OUT would race
// reg_data_zx and either stall core0 in hostWriteB3's 1 ms spinwait
// (FPS→1) or silently drop a byte (sample bank shifts → no audio after
// load). The FIFO absorbs the burst at host speed and GS-Z80 drains at its
// own pace.
//
// Single-producer (core0 host) / single-consumer (core1 GS-Z80). Power-of-2
// size with mask wrapping; volatile uint32_t pos atomic on ARM.
#define GS_HOST_FIFO_SIZE 512
#define GS_HOST_FIFO_MASK (GS_HOST_FIFO_SIZE - 1)
static uint8_t s_host_fifo[GS_HOST_FIFO_SIZE];
static volatile uint32_t s_host_fifo_w = 0;
static volatile uint32_t s_host_fifo_r = 0;

// Same FIFO for command port BB. FH1_GS_TZ.scl streams interleaved
// CMD/DATA pairs (OUT BB,A; OUT B3,A; ...) at thousands per second; the
// scalar reg_command was getting overwritten before firmware could read
// it via IN A,(1), so only the LAST of each burst survived. With a FIFO
// the firmware sees every command in order.
#define GS_CMD_FIFO_SIZE 256
#define GS_CMD_FIFO_MASK (GS_CMD_FIFO_SIZE - 1)
static uint8_t s_cmd_fifo[GS_CMD_FIFO_SIZE];
static volatile uint32_t s_cmd_fifo_w = 0;
static volatile uint32_t s_cmd_fifo_r = 0;

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
static constexpr uint32_t GS_CLOCK_HZ    = 13125000;
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

// Last-hit memo: when the firmware streams a sample, 64 consecutive byte
// reads land on the same cache line. A one-entry memo skips the 4-way
// associative scan on the dominant case. Set by every line that gets
// touched (hit or fill), invalidated when its line is evicted.
static uint32_t       s_pc_last_line = ~0u;
static const uint8_t* s_pc_last_buf  = nullptr;

static inline void __not_in_flash_func(gs_pc_invalidate_line)(uint32_t psram_off) {
    uint32_t line = psram_off >> GS_PC_LINE_BITS;
    uint32_t set  = line & GS_PC_SETS_MASK;
    for (int w = 0; w < GS_PC_WAYS; w++) {
        if (s_pc_tag[set][w] == line) { s_pc_tag[set][w] = ~0u; break; }
    }
    if (s_pc_last_line == line) { s_pc_last_line = ~0u; s_pc_last_buf = nullptr; }
}

static inline zuint8 __not_in_flash_func(gs_pc_read)(uint32_t psram_off) {
    uint32_t line = psram_off >> GS_PC_LINE_BITS;
    uint32_t col  = psram_off & GS_PC_LINE_MASK;
    if (line == s_pc_last_line) { s_perf_pc_hit++; return s_pc_last_buf[col]; }
    uint32_t set  = line & GS_PC_SETS_MASK;
    for (int w = 0; w < GS_PC_WAYS; w++) {
        if (s_pc_tag[set][w] == line) {
            s_pc_last_line = line;
            s_pc_last_buf  = s_pc_data[set][w];
            s_perf_pc_hit++;
            return s_pc_data[set][w][col];
        }
    }
    // Miss — FIFO eviction, bulk-copy 64 bytes from PSRAM (XIP burst friendly).
    s_perf_pc_miss++;
    uint8_t v = s_pc_next[set];
    s_pc_next[set] = (v + 1) & (GS_PC_WAYS - 1);
    memcpy(s_pc_data[set][v], &s_gs_ram[line << GS_PC_LINE_BITS], GS_PC_LINE_SZ);
    s_pc_tag[set][v] = line;
    s_pc_last_line = line;
    s_pc_last_buf  = s_pc_data[set][v];
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

// Page table for fast fetch/read of non-banked address space (0x0000-0x7FFF).
// Populated by gs_init_fetch_pages() from GS::init.
static const uint8_t* s_fetch_page[8];

// Hot data-read path. Layout of the GS-Z80 address space for reads:
//   0x0000-0x3FFF: ROM (one of two firmware images)
//   0x4000-0x5FFF: work_ram low half — vars/stack
//   0x6000-0x7FFF: work_ram high half AND DAC latch sink (writes into
//                  reg_ch[(addr>>8)&3] on every read here, by which the
//                  firmware streams samples to the DAC).
//   0x8000-0xFFFF: banked PSRAM page (sample data) — goes through cache
//
// Use the same fetch_page table for non-banked reads (single load, no
// branches). For 0x6000-0x7FFF the cheap DAC update is folded into the
// fast path. Banked region falls through to gs_pc_read.
static zuint8 __not_in_flash_func(gs_cb_read)(void* ctx, zuint16 address) {
    (void)ctx;
    if (address < 0x8000) {
        const uint8_t* base = s_fetch_page[address >> 13];
        zuint8 v = base[address & 0x1FFF];
        // DAC latch — only for the 0x6000-0x7FFF page. Cheap test: page-id
        // bits 110 == 3, so check (address >> 13) == 3.
        if ((address >> 13) == 3) {
            GS::reg_ch[(address >> 8) & 3] = v;
        }
        return v;
    }
    // Banked PSRAM (sample data) — cache-backed.
    if (GS::reg_page == 0) return ROM_GS_M[address - 0x8000];
    uint32_t off = (gs_map_addr(address) + 0x4000) & s_gs_ram_mask;
    return gs_pc_read(off);
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
        case 0x01: {
            // Drain one command from cmd FIFO. Falls back to scalar
            // reg_command if FIFO empty.
            uint32_t r = s_cmd_fifo_r;
            uint32_t w = s_cmd_fifo_w;
            __dmb();
            if (r != w) {
                v = s_cmd_fifo[r & GS_CMD_FIFO_MASK];
                s_cmd_fifo_r = r + 1;
                if ((r + 1) == w) {
                    gs_status_and(&GS::reg_status, ~0x01u);
                }
            } else {
                v = GS::reg_command;
                // D0 stays set until GS explicitly acks via IN/OUT 0x05
            }
            break;
        }
        case 0x02: {
            // Drain one byte from the host FIFO. D7 reflects "FIFO non-empty":
            // it stays set as long as there's data, and clears as soon as the
            // last byte is consumed. We do NOT fall back to a legacy single-
            // byte slot — mixing two channels (FIFO + reg_data_zx) breaks
            // ordering when host bursts after a quiet period.
            uint32_t r = s_host_fifo_r;
            uint32_t w = s_host_fifo_w;
            __dmb();
            if (r != w) {
                v = s_host_fifo[r & GS_HOST_FIFO_MASK];
                s_host_fifo_r = r + 1;
                if ((r + 1) == w) {
                    gs_status_and(&GS::reg_status, ~0x80u);
                }
            } else {
                // FIFO empty (D7 should already be 0; double-clear is safe).
                v = 0xFF;
                gs_status_and(&GS::reg_status, ~0x80u);
            }
            break;
        }
        case 0x03:
            // Per UnrealSpeccy gsz80.cpp: also stores 0xFF into reg_data_gs.
            // Otherwise a stale prior response would be returned to ZX on
            // next hostReadB3, breaking GS→ZX protocol state.
            GS::reg_data_gs = 0xFF;
            gs_status_or(&GS::reg_status, 0x80u);
            v = 0xFF;
            break;
        case 0x04: {
            v = GS::reg_status;
            uint16_t pc = Z80_PC(s_cpu);
            s_perf_p04_total++;
            if (pc == s_perf_p04_pc) s_perf_p04_spin++;
            s_perf_p04_pc = pc;
            break;
        }
        case 0x05:
            // Command ack — clear D0 only if cmd FIFO is empty. If there
            // are more pending commands, leave D0 set so firmware loops
            // back to read the next one.
            if (s_cmd_fifo_r == s_cmd_fifo_w) {
                gs_status_and(&GS::reg_status, ~0x01u);
            }
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
            // Data-port ack — only clear D7 if data FIFO empty.
            if (s_host_fifo_r == s_host_fifo_w) {
                gs_status_and(&GS::reg_status, ~0x80u);
            }
            return;
        case 0x03:
            GS::reg_data_gs = value;
            __dmb();  // data must be visible to core0 before setting D7
            gs_status_or(&GS::reg_status, 0x80u);
            return;
        case 0x05:
            // Command ack — only clear D0 if cmd FIFO empty.
            if (s_cmd_fifo_r == s_cmd_fifo_w) {
                gs_status_and(&GS::reg_status, ~0x01u);
            }
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


// Hot path: fetch_opcode/fetch run on every Z80 instruction. GS firmware
// code lives entirely in 0x0000-0x7FFF (ROM + work_ram); banked PSRAM
// pages at 0x8000-0xFFFF hold sample data only and are never executed.
//
// We keep an 8-entry page table (one slot per 8 KB of address space, indexed
// by address >> 13). Each slot points to the base of a contiguous SRAM-/
// flash-resident buffer, so a fetch turns into a single load + offset add
// — no branches. Slot 0 (ROM page 0) is set at init; slot for the banked
// region (>=0x8000) is left null so the fallback handles it.
// (Forward-declaration of s_fetch_page is above gs_cb_read.)

static inline void gs_init_fetch_pages(void) {
    s_fetch_page[0] = ROM_GS_M + 0x0000;        // 0x0000-0x1FFF
    s_fetch_page[1] = ROM_GS_M + 0x2000;        // 0x2000-0x3FFF
    s_fetch_page[2] = s_gs_work_ram + 0x0000;   // 0x4000-0x5FFF
    s_fetch_page[3] = s_gs_work_ram + 0x2000;   // 0x6000-0x7FFF
    // Slots 4-7 stay null; firmware never executes from there.
    for (int i = 4; i < 8; i++) s_fetch_page[i] = nullptr;
}

static zuint8 __not_in_flash_func(gs_cb_fetch_opcode)(void* ctx, zuint16 address) {
    (void)ctx;
    const uint8_t* base = s_fetch_page[address >> 13];
    if (base) return base[address & 0x1FFF];
    // Defensive fallback if firmware ever jumps into banked PSRAM.
    return gs_mem_raw_read(address);
}
static zuint8 __not_in_flash_func(gs_cb_fetch)(void* ctx, zuint16 address) {
    (void)ctx;
    const uint8_t* base = s_fetch_page[address >> 13];
    if (base) return base[address & 0x1FFF];
    return gs_mem_raw_read(address);
}
static zuint8 __not_in_flash_func(gs_cb_nop)(void* ctx, zuint16 address) {
    (void)ctx; (void)address;
    return 0;
}

// Direct-call entry points used by Z80_redcode.c when GS_Z80_DIRECT_CALLBACKS
// is defined. Each is a thin wrapper around the gs_cb_* function the static
// callback table would dispatch to anyway, but reached via a normal direct
// call so the M33 branch predictor doesn't get clobbered on every Z80
// instruction. Marked __not_in_flash_func so they live with the redcode in
// SRAM (.time_critical) and don't introduce a flash hop on the hot path.
extern "C" {
zuint8 __not_in_flash_func(gs_direct_fetch_opcode)(zuint16 address) { return gs_cb_fetch_opcode(nullptr, address); }
zuint8 __not_in_flash_func(gs_direct_fetch       )(zuint16 address) { return gs_cb_fetch       (nullptr, address); }
zuint8 __not_in_flash_func(gs_direct_read        )(zuint16 address) { return gs_cb_read        (nullptr, address); }
void   __not_in_flash_func(gs_direct_write       )(zuint16 address, zuint8 value) { gs_cb_write(nullptr, address, value); }
zuint8 __not_in_flash_func(gs_direct_in          )(zuint16 port)    { return gs_cb_in          (nullptr, port); }
void   __not_in_flash_func(gs_direct_out         )(zuint16 port, zuint8 value) { gs_cb_out     (nullptr, port, value); }
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
    gs_init_fetch_pages();   // hot-path fetch table: see gs_cb_fetch_opcode
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
    s_pump_last_us = 0;
    s_ring_wpos = 0;
    s_ring_rpos = 0;
    s_drain_frac = 0;
    for (int i = 0; i < GS_RING_SIZE; i++) { s_ring_L[i] = 0; s_ring_R[i] = 0; }
    for (int i = 0; i < GS_PC_SETS; i++) {
        for (int w = 0; w < GS_PC_WAYS; w++) s_pc_tag[i][w] = ~0u;
        s_pc_next[i] = 0;
    }
    s_pc_last_line = ~0u;
    s_pc_last_buf  = nullptr;
    s_host_fifo_w = 0;
    s_host_fifo_r = 0;
    s_cmd_fifo_w = 0;
    s_cmd_fifo_r = 0;
    if (enabled) {
        z80_instant_reset(&s_cpu);
    }
}

void __not_in_flash_func(GS::topUpBudget)(int tstates) {
    (void)tstates;
    // Budget mechanism removed: step() runs freely. GS-Z80 time is bounded
    // by how often the emulator gets called; we don't need an explicit cap.
}

void GS::pollPerf() {
    static uint32_t s_last_us = 0;
    uint32_t now = time_us_32();
    if ((now - s_last_us) < 1000000) return;
    uint32_t dt = now - s_last_us;
    s_last_us = now;
    if (!enabled) {
        gs_perf_idle_min        = 0x7FFFFFFF;
        gs_perf_idle_neg_frames = 0;
        gs_perf_frames          = 0;
        return;
    }

    // Snapshot + reset core1 counters
    uint32_t pc_calls = s_perf_pump_calls;
    uint32_t pc_skip  = s_perf_pump_skip;
    uint32_t tst      = s_perf_tstates;
    uint32_t p04t     = s_perf_p04_total;
    uint32_t p04s     = s_perf_p04_spin;
    s_perf_pump_calls = 0;
    s_perf_pump_skip  = 0;
    s_perf_tstates    = 0;
    s_perf_p04_total  = 0;
    s_perf_p04_spin   = 0;

    // Host counters
    uint32_t b3w  = s_perf_h_b3w;
    uint32_t b3r  = s_perf_h_b3r;
    uint32_t bbw  = s_perf_h_bbw;
    uint32_t bbr  = s_perf_h_bbr;
    uint32_t hsw  = s_perf_h_spin_us;
    s_perf_h_b3w = 0;
    s_perf_h_b3r = 0;
    s_perf_h_bbw = 0;
    s_perf_h_bbr = 0;
    s_perf_h_spin_us = 0;

    // Core0 IDL stats
    int32_t  idle_min = gs_perf_idle_min;
    uint32_t neg      = gs_perf_idle_neg_frames;
    uint32_t fr       = gs_perf_frames;
    gs_perf_idle_min        = 0x7FFFFFFF;
    gs_perf_idle_neg_frames = 0;
    gs_perf_frames          = 0;

    uint32_t pc_h = s_perf_pc_hit;
    uint32_t pc_m = s_perf_pc_miss;
    s_perf_pc_hit = 0;
    s_perf_pc_miss = 0;

    // GS-Z80 effective MHz (12 MHz target)
    uint32_t gs_khz = (uint32_t)((uint64_t)tst * 1000u / dt);  // = T-states/ms

    // Skip first probe-only burst — only log if there's work or interesting stalls
    if (tst == 0 && b3w == 0 && b3r == 0 && fr == 0) return;

    // PSRAM cache miss rate as a percentage (0-100). Useful to spot bus
    // contention vs internal emulator work.
    uint32_t pc_miss_pct = (pc_h + pc_m) ? (pc_m * 100u / (pc_h + pc_m)) : 0;

    // Debug::log on core0 over UART is a blocking operation (~1.5-2 ms for
    // 200+ char line at 115200 baud). When run from pollPerf each second
    // this self-induces an exactly-once-per-second IDL stall of ~-2000 µs
    // that masquerades as a real perf problem. Suppress here unless you
    // really need the trace; flip to 1 when actively diagnosing.
#if 0
    uint32_t fifo_used = s_host_fifo_w - s_host_fifo_r;
    Debug::log("PERF: fr=%u IDL_min=%d neg=%u | GS:%uMhz pump=%u/%u p04=%u(spin=%u) pc_miss=%u/%u(%u%%) fifo=%u | host: B3=%uw/%ur BB=%uw/%ur spin=%uus",
               (unsigned)fr,
               (int)idle_min,
               (unsigned)neg,
               (unsigned)(gs_khz / 1000),
               (unsigned)(pc_calls - pc_skip),
               (unsigned)pc_calls,
               (unsigned)p04t,
               (unsigned)p04s,
               (unsigned)pc_m,
               (unsigned)(pc_h + pc_m),
               (unsigned)pc_miss_pct,
               (unsigned)fifo_used,
               (unsigned)b3w,
               (unsigned)b3r,
               (unsigned)bbw,
               (unsigned)bbr,
               (unsigned)hsw);
#else
    (void)fr; (void)idle_min; (void)neg; (void)gs_khz;
    (void)pc_calls; (void)pc_skip; (void)p04t; (void)p04s;
    (void)pc_m; (void)pc_h; (void)pc_miss_pct;
    (void)b3w; (void)b3r; (void)bbw; (void)bbr; (void)hsw;
#endif
}

void __not_in_flash_func(GS::pump)() {
    if (!enabled) return;
    s_perf_pump_calls++;
    // Wall-clock-locked pacing. Independent of how fast the emulator can
    // crunch instructions — we always advance GS-Z80 time at exactly
    // GS_CLOCK_HZ T-states per real second. With INSN handlers placed in
    // SRAM, the emulator runs faster than 12 MHz wall-clock; without this
    // gate the producer overshoots ring writes and audio glitches.
    //
    // s_pump_last_us holds the last wall-time we actually advanced from.
    // Each call computes elapsed_us, converts to T-states (12 T/µs), and
    // hands that to step(). Cap the chunk so a single huge pause (e.g.
    // OSD overlay, debug log) doesn't make us run thousands of INTs in
    // one batch — clamp to 1 ms = 12000 T-states.
    uint32_t now = time_us_32();
    if (s_pump_last_us == 0) s_pump_last_us = now;
    uint32_t dt_us = now - s_pump_last_us;
    if (dt_us == 0) return;
    if (dt_us > 1000) dt_us = 1000;          // clamp: max 1 ms per pump
    int budget_t = (int)(dt_us * 12);         // 12 T-states per µs at 12 MHz
    s_pump_last_us = now;

    // Ring-fill safety: if consumer fell badly behind (shouldn't happen
    // when wall-clock paced), don't push more or we'll overrun.
    uint32_t used = s_ring_wpos - s_ring_rpos;
    if (used >= (GS_RING_SIZE * 7 / 8)) {
        s_perf_pump_skip++;
        return;
    }

    int ran = step(budget_t);
    s_perf_tstates += (uint32_t)ran;
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
    s_perf_h_b3r++;
    uint8_t v = reg_data_gs;
    __dmb();  // consume data before clearing the flag
    uint32_t fifo_used = s_host_fifo_w - s_host_fifo_r;
    // D7 is multiplexed: "GS→host response ready" (set by firmware OUT 03)
    // AND "host→GS FIFO non-empty" (set by hostWriteB3). Only clear if the
    // FIFO is empty — otherwise firmware loses the FIFO indicator and never
    // drains. Z-Player's tight OUT B3/IN B3 pattern hits this race.
    if (fifo_used == 0) {
        gs_status_and(&reg_status, ~0x80u);
    }
    return v;
}

uint8_t GS::hostReadBB() {
    s_perf_h_bbr++;
    return reg_status | 0x7E;  // bits 1-6 always set (hardware signature)
}

// Push host byte into the FIFO. Returns immediately while there's room
// (typical case: SCL/MOD bulk loads). Spins only when FIFO is full;
// after a short bound, drops the oldest byte to keep moving — preferable
// to a long stall on core0 or to overwriting reg_data_zx mid-stream
// (which would cascade-corrupt the sample bank).
//
// FIFO size is 512 bytes; at 37500 bytes/sec drain rate that's ~14 ms
// buffer — enough to absorb a full SCL sector (256 B) plus a margin.
void GS::hostWriteB3(uint8_t data) {
    s_perf_h_b3w++;
    uint32_t w = s_host_fifo_w;
    uint32_t used = w - s_host_fifo_r;
    if (used >= GS_HOST_FIFO_SIZE) {
        uint32_t spin_t0 = time_us_32();
        while ((s_host_fifo_w - s_host_fifo_r) >= GS_HOST_FIFO_SIZE
               && (time_us_32() - spin_t0) < 500) {
            __dmb();
        }
        s_perf_h_spin_us += time_us_32() - spin_t0;
        if ((s_host_fifo_w - s_host_fifo_r) >= GS_HOST_FIFO_SIZE) {
            s_host_fifo_r = s_host_fifo_w - GS_HOST_FIFO_SIZE + 1;
        }
        w = s_host_fifo_w;
    }
    s_host_fifo[w & GS_HOST_FIFO_MASK] = data;
    __dmb();
    s_host_fifo_w = w + 1;
    __dmb();
    gs_status_or(&reg_status, 0x80u);  // D7=1: FIFO non-empty
}

void GS::hostWriteBB(uint8_t data) {
    s_perf_h_bbw++;
    // Push into command FIFO (drop-oldest if full — same policy as B3).
    uint32_t w = s_cmd_fifo_w;
    uint32_t used = w - s_cmd_fifo_r;
    if (used >= GS_CMD_FIFO_SIZE) {
        s_cmd_fifo_r = s_cmd_fifo_w - GS_CMD_FIFO_SIZE + 1;
    }
    s_cmd_fifo[w & GS_CMD_FIFO_MASK] = data;
    __dmb();
    s_cmd_fifo_w = w + 1;
    reg_command = data;  // mirror to scalar for any direct-readers
    __dmb();
    gs_status_or(&reg_status, 0x01u);  // D0=1: command available
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
