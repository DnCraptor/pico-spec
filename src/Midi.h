#pragma once

#if !PICO_RP2040

#include <inttypes.h>

class Midi {
public:
    static void init();
    static void deinit();
    static void send(uint8_t b);
    static void flush();

    static bool enabled;

private:
    static constexpr int BUF_SIZE = 256;
    static uint8_t buf[BUF_SIZE];
    static volatile uint8_t head;
    static volatile uint8_t tail;
    static bool hw_initialized;
};

#endif // !PICO_RP2040
