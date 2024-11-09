#ifndef LOAD_WAV_STREAM
#define LOAD_WAV_STREAM

#include <queue>
#include <hardware/gpio.h>
#include "CPU.h"

#define MAX_IN_SAMPLES 512

class LoadWavStream {
        uint32_t statesInFrame = 100000;
        uint32_t samplesPerFrame = 640;
        uint32_t statesPerSample = 100000 / 640;
        volatile size_t buf_out_off = 0;
        uint64_t buf_started_time = 0;
        uint8_t buf[MAX_IN_SAMPLES];
        volatile size_t bit_out_n = 0;
    public:
        // out
        inline void tick(void) {
            if ( buf_out_off >= MAX_IN_SAMPLES ) {
                return; /// ??
            }
            uint8_t v = gpio_get(LOAD_WAV_PIO) ? 1 : 0;
            uint8_t* pv = &buf[buf_out_off];
            uint8_t c = bit_out_n == 0 ? 0 : *pv;
            *pv = c | (v << bit_out_n);
            ++bit_out_n;
            if (bit_out_n == 8) {
                bit_out_n = 0;
                ++buf_out_off;
            }
        }
        inline void open_frame(void) {
            bit_out_n = 0;
            buf_out_off = 0;
            buf_started_time = CPU::global_tstates + CPU::tstates;
            memset(buf, 0, MAX_IN_SAMPLES);
        }
        // in
        inline bool get_in_sample(void) {
            if (CPU::statesInFrame != statesInFrame || ESPectrum::samplesPerFrame != samplesPerFrame) {
                statesInFrame = CPU::statesInFrame;
                samplesPerFrame = ESPectrum::samplesPerFrame;
                statesPerSample = statesInFrame / samplesPerFrame;
            }
            uint64_t statesPassedFromFrameStarted = CPU::global_tstates + CPU::tstates - buf_started_time;
            size_t obuff_off = statesPassedFromFrameStarted / statesPerSample;
            size_t idx = obuff_off >> 3;
            if ( idx >= MAX_IN_SAMPLES ) {
                return 0;
            }
            uint8_t bit = obuff_off & 7;
            return (buf[idx] >> bit) & 1;
        }
};

#endif
