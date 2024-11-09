#ifndef LOAD_WAV_STREAM
#define LOAD_WAV_STREAM

#include <queue>
#include <hardware/gpio.h>
#include "CPU.h"

class LoadWavStream {
        uint32_t statesInFrame = 100000;
        uint32_t samplesPerFrame = 640;
        uint32_t statesPerSample = 100000 / 640;
        size_t buf_out_off = 0;
        size_t buf_in_off = 0;
        uint64_t buf_started_time = 0;
        uint8_t bufs[640];
    public:
        // out
        inline void tick(void) {
            if ( buf_out_off >= 640 ) {
                buf_out_off = 0;
            }
            if (buf_out_off == 0) {
                buf_started_time = CPU::global_tstates + CPU::tstates;
            }
            bufs[buf_out_off++] = gpio_get(LOAD_WAV_PIO);
        }
        // in
        inline void open_frame(void) {
            buf_in_off = 0;
        }
        inline bool get_in_sample(void) {
            if (CPU::statesInFrame != statesInFrame || ESPectrum::samplesPerFrame != samplesPerFrame) {
                statesInFrame = CPU::statesInFrame;
                samplesPerFrame = ESPectrum::samplesPerFrame;
                statesPerSample = statesInFrame / samplesPerFrame;
            }
            uint64_t statesPassedFromFrameStarted = CPU::global_tstates + CPU::tstates - buf_started_time;
            size_t obuff_off = statesPassedFromFrameStarted / statesPerSample;
            return obuff_off < 640 ? bufs[obuff_off++] : false;
        }
};

#endif
