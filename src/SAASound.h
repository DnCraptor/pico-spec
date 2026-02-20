/*
  SAA1099 Sound Chip Emulation for pico-spec ZX Spectrum emulator

  Based on UnrealSpeccy SAA1099 driver by Juergen Buchmueller and Manuel Abadia
  https://github.com/mkoloberdin/unrealspeccy/blob/master/sndrender/saa1099.cpp

  Adapted for RP2350: integer-only arithmetic, 8-bit buffer output,
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
        uint8_t frequency;      // frequency register (0x00-0xFF)
        uint8_t octave;         // octave (0-7)
        bool freq_enable;       // tone enable
        bool noise_enable;      // noise enable
        uint8_t amp[2];         // raw amplitude (0-15), [0]=left [1]=right
        uint8_t envelope[2];    // envelope level (0-15 or 16=off), [0]=left [1]=right
        uint32_t counter;       // tone counter (integer accumulator)
        uint32_t period;        // tone period = max(511 - frequency, 1)
        uint8_t level;          // current square wave output (0 or 1)
    };
    Channel channels[6];

    struct NoiseGen {
        uint32_t counter;       // noise rate counter
        uint32_t level;         // noise LFSR state
    };
    NoiseGen noise[2];

    // Envelope state (matches UnrealSpeccy saa1099_state layout)
    uint8_t noise_params[2];        // noise source (0-3)
    uint8_t env_enable[2];          // 0 or 0x80
    uint8_t env_reverse_right[2];   // active invert-right flag
    uint8_t env_reverse_right_buf[2]; // buffered invert-right
    uint8_t env_mode[2];            // active envelope mode (0-7)
    uint8_t env_mode_buf[2];        // buffered envelope mode
    uint8_t env_bits[2];            // 3-bit resolution flag (0 or 0x10)
    uint8_t env_clock[2];           // active external clock flag (0 or 0x20)
    uint8_t env_clock_buf[2];       // buffered external clock
    int env_step[2];                // envelope position (0-63)
    bool env_upd[2];                // buffered data pending

    bool all_ch_enable;             // global sound enable

    void updateEnvelope(int ch);

    static const uint8_t envelope[8][64];
};

extern SAASound saaChip;

#endif // SAASound_h
