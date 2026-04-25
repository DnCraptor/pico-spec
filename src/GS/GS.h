#ifndef GS_H
#define GS_H

#include <stdint.h>
#include <stddef.h>

#ifdef USE_GS

class GS {
public:
    static bool enabled;

    static bool init(uint32_t ram_size_bytes);
    static void deinit();
    static void reset();

    // Returns actual T-states executed (may exceed requested due to z80_run
    // completing the current instruction; pump() uses this to maintain exact
    // 12 MHz wall-clock rate via debt tracking).
    static int step(int tstates);

    // Wall-clock-locked pump. Call in a tight loop on core1; runs the GS-Z80
    // at exactly 12 MHz regardless of how fast the emulator could go.
    static void pump();


    // Top up the GS-Z80 tstates budget. Call once per Spectrum frame with the
    // frame duration × GS clock ratio (e.g. 240000 for 20 ms @ 12 MHz). step()
    // calls consume the budget; when budget runs out, step() is a no-op, which
    // keeps GS-Z80 from running faster than real time.
    static void topUpBudget(int tstates);

    static uint8_t hostReadB3();
    static uint8_t hostReadBB();
    static void    hostWriteB3(uint8_t data);
    static void    hostWriteBB(uint8_t data);

    static int16_t getSampleLeft();
    static int16_t getSampleRight();

    // Read the current DAC mix as unsigned 0..255 (silence = 128).
    // Called from the audio timer IRQ at 31.25 kHz — GS-Z80 runs on core1
    // at 12 MHz wall-clock, so reg_ch/reg_vol reflect live real-time state.
    static void getLiveLR(uint8_t& L, uint8_t& R);

    static uint32_t gs_ram_size;

    // Shared between core0 (host) and core1 (GS-Z80): must be volatile.
    // All |= / &= on reg_status use LDREX/STREX via gs_status_or/and().
    static volatile uint8_t  reg_command;
    static volatile uint8_t  reg_data_zx;
    static volatile uint8_t  reg_data_gs;
    static volatile uint8_t  reg_status;

    static uint8_t  reg_page;
    static uint8_t  reg_vol[4];
    static uint8_t  reg_ch[4];

    static uint32_t int_count;
};

#endif // USE_GS

#endif // GS_H
