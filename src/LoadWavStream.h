#ifndef LOAD_WAV_STREAM
#define LOAD_WAV_STREAM

#include <queue>
#include <hardware/gpio.h>
#include "CPU.h"

#define MAX_IN_SAMPLES 128

class LoadWavStream {
        uint32_t statesInFrame = 0;
        uint32_t samplesPerFrame = 0;
        volatile size_t buf_out_off = 0;
        volatile size_t bit_out_n = 0;
        uint64_t buf_started_state_n = 0;
        volatile uint8_t* buf1 = 0;
        volatile uint8_t* buf2 = 0;
        volatile bool lock_in = false;
    public:
        // out
        inline void tick(void) {
            if ( !buf1 || buf_out_off >= MAX_IN_SAMPLES ) {
                return; /// ??
            }
            uint8_t v = gpio_get(LOAD_WAV_PIO) ? 1 : 0;
            uint8_t max_wait = 20;
            while(lock_in && max_wait) --max_wait;
            volatile uint8_t* pv = buf1 + buf_out_off;
            uint8_t c = bit_out_n == 0 ? 0 : *pv;
            *pv = c | (v << bit_out_n);
            ++bit_out_n;
            if (bit_out_n == 8) {
                bit_out_n = 0;
                ++buf_out_off;
            }
        }
        inline void open_frame(void) {
            uint64_t st = CPU::global_tstates + CPU::tstates;
            if (buf2) {
                samplesPerFrame = (buf_out_off << 3) + bit_out_n;
                statesInFrame = (st - buf_started_state_n) & 0xFFFFFFFFF;
                delete[] buf2;
            }
            uint8_t* bufn = new uint8_t[MAX_IN_SAMPLES]();

            lock_in = true;
            buf2 = buf1;
            bit_out_n = 0;
            buf_out_off = 0;
            buf1 = bufn;
            lock_in = false;

            buf_started_state_n = st;
        }
        // in
        inline bool get_in_sample(void) {
            if ( !buf2 || statesInFrame <= 0 ) {
                return 0;
            }
            ///uint64_t time_from_start = time_us_64() - buf_started_time; // microseconds
            ///size_t samples_from_start = (buf_out_off << 3) + bit_out_n; // samples already recorded
            ///uint64_t microsecondsPerSample = time_from_start / samples_from_start; // microseconds/sample /0!!
            uint32_t statesPassedFromFrameStarted = (CPU::global_tstates + CPU::tstates - buf_started_state_n) & 0xFFFFFFFFF; // states
///            uint32_t statesPerSample = statesInFrame / samplesPerFrame;
///            size_t obuff_off = statesPassedFromFrameStarted / statesPerSample;
            uint32_t obuff_off = statesPassedFromFrameStarted * samplesPerFrame / statesInFrame;
            size_t idx = (size_t)(obuff_off >> 3);
            if ( idx >= MAX_IN_SAMPLES ) {
                return 0;
            }
            uint8_t bit = (uint8_t)(obuff_off & 7);
            return (buf2[idx] >> bit) & 1;
        }
};

#endif
