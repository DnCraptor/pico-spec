/*
    zxnDMA / Z80 DMA emulation for ZX Spectrum Next compatibility
    Supports both Port #0B (Zilog Z80 DMA) and Port #6B (zxnDMA) modes.

    Reference: https://wiki.specnext.dev/DMA
*/

#ifndef Z80DMA_h
#define Z80DMA_h

#if !PICO_RP2040

#include <inttypes.h>

class Z80DMA {
public:
    static void reset();
    static void writePort(uint8_t data);
    static uint8_t readPort();
    static void handleDMA();


private:
    // Configured state (from WR0-WR5)
    static uint8_t transfer_dir;    // 0=A->B, 1=B->A
    static uint16_t port_a_addr;
    static uint16_t block_length;
    static bool port_a_is_io;
    static int8_t port_a_inc;       // +1, -1, or 0
    static uint8_t port_a_cycles;   // 2, 3, or 4 T-states

    static uint16_t port_b_addr;
    static bool port_b_is_io;
    static int8_t port_b_inc;
    static uint8_t port_b_cycles;

    static uint8_t prescalar;       // 0=continuous, >0=burst
    static bool auto_restart;

    // Working state (active transfer)
    static uint16_t cur_port_a;
    static uint16_t cur_port_b;
    static uint32_t byte_counter;
    static bool transfer_active;
    static bool transfer_started;
    static bool block_end;

    // Write state machine
    enum WriteState : uint8_t { WS_COMMAND, WS_PARAM_BYTE };
    static WriteState write_state;
    static uint8_t current_wr;      // which WR we're collecting params for (0,2,4)
    static uint8_t param_mask;      // bitmask of remaining params
    static uint8_t param_index;     // which param bit we're on

    static bool read_mask_pending;  // next write is read mask byte
    static bool wr2_prescalar_pending; // next write is prescalar byte

    // Read state machine
    static uint8_t read_mask;       // 7-bit: which registers to return
    static uint8_t read_index;
    static bool read_sequence_active;

    // Reentrancy guard
    static bool dma_in_progress;

    // Internal methods
    static void processCommand(uint8_t data);
    static void processWR0(uint8_t data);
    static void processWR1(uint8_t data);
    static void processWR2(uint8_t data);
    static void processWR4(uint8_t data);
    static void processWR5(uint8_t data);
    static void processWR6(uint8_t cmd);
    static void collectParam(uint8_t data);

    static void doLoad();
    static void doContinue();
    static void doEnable();
    static void doDisable();
    static void executeTransfer();
    static void transferOneByte();
    static uint8_t getStatusByte();
};

#endif // !PICO_RP2040
#endif // Z80DMA_h
