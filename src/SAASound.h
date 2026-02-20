/*
  SAA1099 Sound Chip Emulation for pico-spec ZX Spectrum emulator

  Based on Philips SAA1099 datasheet and SAASound library by Dave Hooper
  https://github.com/stripwax/SAASound

  Optimized for RP2350: integer-only arithmetic, flash-resident tables,
  minimal RAM footprint.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#ifndef SAASound_h
#define SAASound_h

#include "hardconfig.h"
#include "ESPectrum.h"
#include <inttypes.h>
#include <stddef.h>
#include <string.h>

class SAASound {
public:
    SAASound();

    void init();
    void reset();
    void selectRegister(uint8_t reg);
    void setRegisterData(uint8_t data);
    uint8_t getRegisterData();
    void set_sound_format(int freq, int chans, int bits);
    void gen_sound(int bufsize, int bufpos);

    uint8_t SamplebufSAA_L[ESP_AUDIO_SAMPLES_PENTAGON];
    uint8_t SamplebufSAA_R[ESP_AUDIO_SAMPLES_PENTAGON];

private:
    uint8_t regs[32];
    uint8_t selectedRegister;

    struct Channel {
        uint8_t amp_left;
        uint8_t amp_right;
        uint8_t freq_offset;
        uint8_t octave;
        bool tone_on;
        bool noise_on;
        uint32_t counter;
        uint32_t period;
        uint8_t bit;          // current tone output (0 or 1)
    };
    Channel channels[6];

    struct NoiseGen {
        uint8_t source;       // 0-3
        uint32_t counter;
        uint32_t period;
        uint32_t lfsr;        // 18-bit LFSR state
        uint8_t bit;          // current noise output (0 or 1)
    };
    NoiseGen noise[2];

    // Envelope shapes:
    // 0=zero, 1=max, 2=single decay, 3=repeat decay,
    // 4=single triangle, 5=repeat triangle, 6=single attack, 7=repeat attack
    struct EnvelopeGen {
        bool enabled;
        bool invert_right;
        uint8_t shape;        // 3 bits (0-7)
        bool resolution;      // 0=4-bit (16 steps), 1=3-bit (8 steps)
        bool ext_clock;       // 0=internal (from freq gen), 1=external
        uint8_t position;     // phase position (0-15, steps by 1 or 2)
        uint8_t phase;        // current phase index (0 or 1)
        bool ended;           // true when non-looping envelope has finished
        // Pending data: deferred until next envelope clock tick
        bool new_data;
        bool pending_enabled;
        bool pending_invert_right;
        uint8_t pending_shape;
        bool pending_resolution;
        bool pending_ext_clock;
    };
    EnvelopeGen envs[2];

    bool sound_enabled;
    int sample_rate;

    void updateChannelPeriod(int ch);
    void updateNoisePeriod(int ng);
    void advanceEnvelope(int env_idx);
    uint8_t getEnvelopeLevel(int env_idx);

    static const uint8_t vol_table[16];
};

extern SAASound saaChip;

#endif // SAASound_h
