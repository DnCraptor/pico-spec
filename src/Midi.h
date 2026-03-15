#pragma once

#if !PICO_RP2040

#include <inttypes.h>

class Midi {
public:
    static void init();
    static void deinit();
    static void send(uint8_t b);
    static bool busy();

    static uint8_t enabled;  // 0=Off, 1=AY bitbang, 2=ShamaZX, 3=Soft Synth

private:
    static bool hw_initialized;
};

#endif // !PICO_RP2040
