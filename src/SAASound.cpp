/*
  SAA1099 Sound Chip Emulation for pico-spec ZX Spectrum emulator

  Based on UnrealSpeccy SAA1099 driver by Juergen Buchmueller and Manuel Abadia
  https://github.com/mkoloberdin/unrealspeccy/blob/master/sndrender/saa1099.cpp

  Key behavioral elements ported from UnrealSpeccy:
    - Envelope: 64-step counter (0-31 once, then 32-63 loop), with deferred
      parameter updates applied only at natural shape boundaries.
    - Mixing: subtractive noise model (noise subtracts half amplitude).
    - Buzz: ch2/ch5 output DC at envelope level when freq_enable=0.
    - Noise LFSR: bits 14 and 6 tapped (XNOR feedback, shift left).

  Adapted for RP2350: integer-only arithmetic (no doubles), 8-bit buffer output.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "SAASound.h"
#include "Config.h"
#include <string.h>

#ifdef PICO_RP2040
#include <pico/platform.h>
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("audio")
#endif

// Envelope table from UnrealSpeccy — 8 shapes x 64 steps.
// Steps 0-31 play once, then steps 32-63 loop forever.
const uint8_t SAASound::envelope[8][64] = {
    /* 0: zero amplitude */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* 1: maximum amplitude */
    {15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
     15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
     15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
     15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15 },
    /* 2: single decay */
    {15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* 3: repetitive decay */
    {15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 },
    /* 4: single triangular */
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* 5: repetitive triangular */
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 },
    /* 6: single attack */
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    /* 7: repetitive attack */
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15 }
};

SAASound saaChip;

SAASound::SAASound() {
    init();
}

void SAASound::init() {
    selectedRegister = 0;
    all_ch_enable = false;
    memset(regs, 0, sizeof(regs));
    memset(channels, 0, sizeof(channels));
    memset(noise, 0, sizeof(noise));

    for (int i = 0; i < 6; i++) {
        channels[i].period = 511;
        channels[i].envelope[0] = 16;  // 16 = envelope off (pass-through)
        channels[i].envelope[1] = 16;
    }

    for (int i = 0; i < 2; i++) {
        noise_params[i] = 0;
        env_enable[i] = 0;
        env_reverse_right[i] = 0;
        env_reverse_right_buf[i] = 0;
        env_mode[i] = 0;
        env_mode_buf[i] = 0;
        env_bits[i] = 0;
        env_clock[i] = 0;
        env_clock_buf[i] = 0;
        env_step[i] = 0;
        env_upd[i] = false;
    }

    memset(SamplebufSAA_L, 0, sizeof(SamplebufSAA_L));
    memset(SamplebufSAA_R, 0, sizeof(SamplebufSAA_R));
}

void SAASound::reset() {
    init();
}

void SAASound::set_sound_format(int freq, int chans, int bits) {
    (void)freq; (void)chans; (void)bits;
}

// Register address select — also triggers external envelope clock
// (matches UnrealSpeccy WrCtl)
void SAASound::selectRegister(uint8_t reg) {
    selectedRegister = reg & 0x1F;
    if (selectedRegister == 0x18 || selectedRegister == 0x19) {
        if (env_clock[0]) updateEnvelope(0);
        if (env_clock[1]) updateEnvelope(1);
    }
}

uint8_t SAASound::getRegisterData() {
    if (selectedRegister > 0x1F) return 0xFF;
    return regs[selectedRegister];
}

// Register data write (matches UnrealSpeccy WrData)
void SAASound::setRegisterData(uint8_t data) {
    if (selectedRegister > 0x1F) return;
    regs[selectedRegister] = data;

    switch (selectedRegister) {
    // Amplitude registers 0x00-0x05
    case 0x00: case 0x01: case 0x02:
    case 0x03: case 0x04: case 0x05: {
        int ch = selectedRegister;
        channels[ch].amp[0] = data & 0x0F;
        channels[ch].amp[1] = (data >> 4) & 0x0F;
        break;
    }

    // Frequency registers 0x08-0x0D
    case 0x08: case 0x09: case 0x0A:
    case 0x0B: case 0x0C: case 0x0D: {
        int ch = selectedRegister - 0x08;
        channels[ch].frequency = data;
        int p = 511 - (int)data;
        channels[ch].period = (p < 1) ? 1 : (uint32_t)p;
        break;
    }

    // Octave registers
    case 0x10:
        channels[0].octave = data & 0x07;
        channels[1].octave = (data >> 4) & 0x07;
        break;
    case 0x11:
        channels[2].octave = data & 0x07;
        channels[3].octave = (data >> 4) & 0x07;
        break;
    case 0x12:
        channels[4].octave = data & 0x07;
        channels[5].octave = (data >> 4) & 0x07;
        break;

    // Frequency enable
    case 0x14:
        channels[0].freq_enable = data & 0x01;
        channels[1].freq_enable = data & 0x02;
        channels[2].freq_enable = data & 0x04;
        channels[3].freq_enable = data & 0x08;
        channels[4].freq_enable = data & 0x10;
        channels[5].freq_enable = data & 0x20;
        break;

    // Noise enable
    case 0x15:
        channels[0].noise_enable = data & 0x01;
        channels[1].noise_enable = data & 0x02;
        channels[2].noise_enable = data & 0x04;
        channels[3].noise_enable = data & 0x08;
        channels[4].noise_enable = data & 0x10;
        channels[5].noise_enable = data & 0x20;
        break;

    // Noise generator parameters
    case 0x16:
        noise_params[0] = data & 0x03;
        noise_params[1] = (data >> 4) & 0x03;
        break;

    // Envelope generators
    case 0x18: case 0x19: {
        int ch = selectedRegister - 0x18;
        // Immediate: resolution and enable
        env_bits[ch] = data & 0x10;
        env_enable[ch] = data & 0x80;
        if (!(data & 0x80))
            env_step[ch] = 0;  // reset on disable
        // Buffered: shape, invert, clock — applied at natural boundary
        env_reverse_right_buf[ch] = data & 0x01;
        env_mode_buf[ch] = (data >> 1) & 0x07;
        env_clock_buf[ch] = data & 0x20;
        env_upd[ch] = true;
        break;
    }

    // Sound enable & sync
    case 0x1C:
        all_ch_enable = data & 0x01;
        if (data & 0x02) {
            for (int i = 0; i < 6; i++) {
                channels[i].level = 0;
                channels[i].counter = 0;
            }
        }
        break;

    default:
        break;
    }
}

// Envelope advance — ported from UnrealSpeccy saa1099_envelope().
//
// env_step counts 0..63 then loops 32..63:
//   step = ((step + 1) & 0x3F) | (step & 0x20)
//
// Buffered parameters (mode, invert_right, clock) are applied only when
// the current shape reaches a natural boundary — preventing mid-shape cutoff.
void SAASound::updateEnvelope(int ch) {
    if (env_enable[ch]) {
        int step, mode, mask;

        // Advance step: 0→63 once, then loop 32→63
        step = env_step[ch] =
            ((env_step[ch] + 1) & 0x3F) | (env_step[ch] & 0x20);

        mode = env_mode[ch];

        // Check if buffered parameters should be applied at natural boundary
        if (env_upd[ch]) {
            if (
                ((mode == 1 || mode == 3 || mode == 7) && step && ((step & 0x0F) == 0))
                ||
                ((mode == 5) && step && ((step & 0x1F) == 0))
                ||
                ((mode == 0 || mode == 2 || mode == 6) && step > 0x0F)
                ||
                ((mode == 4) && step > 0x1F)
            ) {
                mode = env_mode[ch] = env_mode_buf[ch];
                env_reverse_right[ch] = env_reverse_right_buf[ch];
                env_clock[ch] = env_clock_buf[ch];
                env_step[ch] = 1;
                step = 1;
                env_upd[ch] = false;
            }
        }

        mask = 15;
        if (env_bits[ch])
            mask &= ~1;  // 3-bit resolution: mask LSB

        channels[ch * 3 + 2].envelope[0] = envelope[mode][step] & mask;
        if (env_reverse_right[ch] & 0x01)
            channels[ch * 3 + 2].envelope[1] = (15 - envelope[mode][step]) & mask;
        else
            channels[ch * 3 + 2].envelope[1] = envelope[mode][step] & mask;
    } else {
        // Envelope disabled: all channels in group get 16 (unity pass-through)
        for (int i = 0; i < 3; i++) {
            channels[ch * 3 + i].envelope[0] = 16;
            channels[ch * 3 + i].envelope[1] = 16;
        }
    }
}

// Main audio generation — runs in RAM for speed on RP2350
IRAM_ATTR void SAASound::gen_sound(int bufsize, int bufpos) {
    uint8_t *buf_L = SamplebufSAA_L + bufpos;
    uint8_t *buf_R = SamplebufSAA_R + bufpos;

    if (!all_ch_enable) {
        memset(buf_L, 0, bufsize);
        memset(buf_R, 0, bufsize);
        return;
    }

    while (bufsize-- > 0) {
        int output_l = 0;
        int output_r = 0;

        // Process each channel: advance tone, clock envelope, then mix
        for (int ch = 0; ch < 6; ch++) {
            // Advance tone counter (integer accumulator, same rate as UnrealSpeccy)
            channels[ch].counter += (1 << channels[ch].octave);
            while (channels[ch].counter >= channels[ch].period) {
                channels[ch].counter -= channels[ch].period;
                channels[ch].level ^= 1;

                // Internal envelope clock: ch1 clocks env0, ch4 clocks env1
                if (ch == 1 && env_clock[0] == 0)
                    updateEnvelope(0);
                if (ch == 4 && env_clock[1] == 0)
                    updateEnvelope(1);
            }

            // Get amplitude with bit-0 masking for envelope channels
            uint8_t al = channels[ch].amp[0];
            uint8_t ar = channels[ch].amp[1];
            if ((ch == 2 && env_enable[0]) || (ch == 5 && env_enable[1])) {
                al &= 0x0E;
                ar &= 0x0E;
            }

            uint8_t el = channels[ch].envelope[0];
            uint8_t er = channels[ch].envelope[1];

            // UnrealSpeccy mixing model:
            //   noise: subtract half amplitude (when noise enabled & noise level high)
            //   tone:  add full amplitude (when freq enabled & tone level high)
            //   buzz:  add full amplitude (ch2/ch5 when freq disabled & envelope on)
            if (channels[ch].noise_enable && (noise[ch / 3].level & 1)) {
                output_l -= al * el;
                output_r -= ar * er;
            }

            if (channels[ch].freq_enable) {
                if (channels[ch].level & 1) {
                    output_l += al * el * 2;
                    output_r += ar * er * 2;
                }
            } else if ((ch == 2 || ch == 5) && env_enable[ch / 3]) {
                output_l += al * el * 2;
                output_r += ar * er * 2;
            }
        }

        // Advance noise generators AFTER mixing (UnrealSpeccy order)
        for (int ng = 0; ng < 2; ng++) {
            uint32_t step, period;
            switch (noise_params[ng]) {
            case 0: step = 2; period = 1; break;   // 31.25 kHz x 2
            case 1: step = 1; period = 1; break;   // 15.625 kHz x 2
            case 2: step = 1; period = 2; break;   // 7.8125 kHz x 2
            default: // source 3: use tone channel frequency
                step = 1 << channels[ng * 3].octave;
                period = channels[ng * 3].period;
                break;
            }
            noise[ng].counter += step;
            while (noise[ng].counter >= period) {
                noise[ng].counter -= period;
                // LFSR: bits 14 and 6 tapped (UnrealSpeccy polynomial)
                if (((noise[ng].level & 0x4000) == 0) == ((noise[ng].level & 0x0040) == 0))
                    noise[ng].level = (noise[ng].level << 1) | 1;
                else
                    noise[ng].level <<= 1;
            }
        }

        // Scale to 8-bit output.
        // Per-channel max (tone, env off): amp(15) * env(16) * 2 = 480
        // 6 channels max: 2880. >>4 = 180. Good for SAM Coupe (SAA only).
        int out_l = (output_l + 8) >> 4;
        int out_r = (output_r + 8) >> 4;
        if (out_l < 0) out_l = 0;
        if (out_r < 0) out_r = 0;
        *buf_L++ = (uint8_t)(out_l > 255 ? 255 : out_l);
        *buf_R++ = (uint8_t)(out_r > 255 ? 255 : out_r);
    }
}
