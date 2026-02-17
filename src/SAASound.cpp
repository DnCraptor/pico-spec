/*
  SAA1099 Sound Chip Emulation for pico-spec ZX Spectrum emulator

  Based on Philips SAA1099 datasheet and SAASound library by Dave Hooper
  https://github.com/stripwax/SAASound

  SAA1099 specs:
    - 6 tone channels, 8 octaves, 256 tones per octave
    - 2 noise generators (18-bit Galois LFSR)
    - 2 envelope generators (8 shapes)
    - Stereo: independent 4-bit L/R amplitude per channel
    - Clock: 8 MHz, internal /256 divider for counters

  Optimized for RP2350: integer-only arithmetic, flash-resident tables.

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

// Volume table: maps 4-bit SAA volume (0-15) to output level
// Linear mapping, max 18 per channel so 6 channels sum to ~108
// Headroom for AY+SAA coexistence: 108 + 140 = 248 < 255
const uint8_t SAASound::vol_table[16] = {
    0, 1, 2, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 16, 17, 18
};

// Tone period: full-resolution (511 - offset), no octave shift.
// Octave is applied as counter step (1 << octave) in gen_sound,
// preserving fractional precision through accumulation.
// Counter runs at 8MHz/256 = 31.25kHz base rate.

// Envelope lookup table from MAME (8 shapes × 64 steps)
// Step counter uses ((step+1)&0x3f)|(step&0x20): loops in 32-63 after first pass
static const uint8_t envelope_table[8][64] = {
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
    sound_enabled = false;
    sample_rate = 31250;
    memset(regs, 0, sizeof(regs));
    memset(channels, 0, sizeof(channels));
    memset(noise, 0, sizeof(noise));
    memset(envs, 0, sizeof(envs));
    // Initialize all channel periods to max (offset=0, octave=0)
    for (int i = 0; i < 6; i++) {
        channels[i].period = 511;
    }
    // Initialize noise LFSRs to non-zero (18-bit all ones)
    noise[0].lfsr = 0x3FFFF;
    noise[1].lfsr = 0x3FFFF;
    noise[0].period = 1;
    noise[1].period = 1;
    memset(SamplebufSAA_L, 0, sizeof(SamplebufSAA_L));
    memset(SamplebufSAA_R, 0, sizeof(SamplebufSAA_R));
}

void SAASound::reset() {
    init();
}

void SAASound::set_sound_format(int freq, int chans, int bits) {
    sample_rate = freq;
}

void SAASound::selectRegister(uint8_t reg) {
    selectedRegister = reg & 0x1F;
    // External envelope clock: only triggered when selecting the corresponding
    // envelope register (0x18 or 0x19), not on every register address write.
    // Verified by stripwax/SAASound: _WriteAddress only calls ExternalClock
    // when m_nCurrentSaaReg == 24 or 25.
    if (selectedRegister == 0x18) {
        if ((envs[0].enabled && envs[0].ext_clock) || envs[0].new_data) advanceEnvelope(0);
    } else if (selectedRegister == 0x19) {
        if ((envs[1].enabled && envs[1].ext_clock) || envs[1].new_data) advanceEnvelope(1);
    }
}

uint8_t SAASound::getRegisterData() {
    if (selectedRegister > 0x1F) return 0xFF;
    return regs[selectedRegister];
}

void SAASound::setRegisterData(uint8_t data) {
    if (selectedRegister > 0x1F) return;
    regs[selectedRegister] = data;

    switch (selectedRegister) {
    // Amplitude registers 0x00-0x05: low nibble = left, high nibble = right
    case 0x00: case 0x01: case 0x02:
    case 0x03: case 0x04: case 0x05:
        channels[selectedRegister].amp_left = data & 0x0F;
        channels[selectedRegister].amp_right = (data >> 4) & 0x0F;
        break;

    // Frequency offset registers 0x08-0x0D
    case 0x08: case 0x09: case 0x0A:
    case 0x0B: case 0x0C: case 0x0D:
        channels[selectedRegister - 0x08].freq_offset = data;
        updateChannelPeriod(selectedRegister - 0x08);
        break;

    // Octave registers
    case 0x10: // Octave ch0 (low nibble), ch1 (high nibble)
        channels[0].octave = data & 0x07;
        channels[1].octave = (data >> 4) & 0x07;
        updateChannelPeriod(0);
        updateChannelPeriod(1);
        break;
    case 0x11: // Octave ch2, ch3
        channels[2].octave = data & 0x07;
        channels[3].octave = (data >> 4) & 0x07;
        updateChannelPeriod(2);
        updateChannelPeriod(3);
        break;
    case 0x12: // Octave ch4, ch5
        channels[4].octave = data & 0x07;
        channels[5].octave = (data >> 4) & 0x07;
        updateChannelPeriod(4);
        updateChannelPeriod(5);
        break;

    // Frequency enable (bit N = channel N tone on)
    case 0x14:
        for (int i = 0; i < 6; i++)
            channels[i].tone_on = (data >> i) & 1;
        break;

    // Noise enable (bit N = channel N noise on)
    case 0x15:
        for (int i = 0; i < 6; i++)
            channels[i].noise_on = (data >> i) & 1;
        break;

    // Noise generator control
    // bits 0-1: noise gen 0 source, bits 4-5: noise gen 1 source
    // 0=31.3kHz, 1=15.6kHz, 2=7.8kHz, 3=use freq gen (ch0/ch3)
    case 0x16:
        noise[0].source = data & 0x03;
        noise[1].source = (data >> 4) & 0x03;
        updateNoisePeriod(0);
        updateNoisePeriod(1);
        break;

    // Envelope generator 0 (affects channels 0-2, clocked by ch1)
    // Parameters are deferred until next envelope clock tick (new_data mechanism)
    case 0x18: {
        envs[0].pending_invert_right = data & 0x01;
        envs[0].pending_shape = (data >> 1) & 0x07;
        envs[0].pending_resolution = (data >> 4) & 0x01;
        envs[0].pending_ext_clock = (data >> 5) & 0x01;
        envs[0].pending_enabled = (data >> 7) & 0x01;
        envs[0].new_data = true;
        break;
    }

    // Envelope generator 1 (affects channels 3-5, clocked by ch4)
    case 0x19: {
        envs[1].pending_invert_right = data & 0x01;
        envs[1].pending_shape = (data >> 1) & 0x07;
        envs[1].pending_resolution = (data >> 4) & 0x01;
        envs[1].pending_ext_clock = (data >> 5) & 0x01;
        envs[1].pending_enabled = (data >> 7) & 0x01;
        envs[1].new_data = true;
        break;
    }

    // Sound enable / reset
    case 0x1C:
        sound_enabled = data & 0x01;
        if (data & 0x02) {
            // Sync: reset all generators
            for (int i = 0; i < 6; i++) {
                channels[i].counter = 0;
                channels[i].bit = 0;
            }
            for (int i = 0; i < 2; i++) {
                noise[i].counter = 0;
                noise[i].lfsr = 0x3FFFF;
                noise[i].bit = 0;
            }
            envs[0].position = envs[1].position = 0;
            envs[0].new_data = envs[1].new_data = false;
        }
        break;

    default:
        break;
    }
}

void SAASound::updateChannelPeriod(int ch) {
    // Full-resolution period: no octave shift here.
    // Octave is applied as counter step (1 << octave) in gen_sound.
    // SAA1099 counter counts offset → 510 (511 - offset values), then toggles.
    channels[ch].period = 511 - channels[ch].freq_offset;
    // If this channel is a noise source, update noise period too
    if (ch == 0 && noise[0].source == 3) updateNoisePeriod(0);
    if (ch == 3 && noise[1].source == 3) updateNoisePeriod(1);
}

void SAASound::updateNoisePeriod(int ng) {
    switch (noise[ng].source) {
    case 0: noise[ng].period = 1; break;   // 31.3 kHz
    case 1: noise[ng].period = 2; break;   // 15.6 kHz
    case 2: noise[ng].period = 4; break;   // 7.8 kHz
    case 3: // Use frequency generator (ch0 for ng0, ch3 for ng1)
        noise[ng].period = channels[ng * 3].period;
        if (noise[ng].period < 1) noise[ng].period = 1;
        break;
    }
}

// Advance envelope by one step (MAME-compatible 64-step counter)
// Step goes 0..63, then loops in 32..63 via sticky bit 5
void SAASound::advanceEnvelope(int env_idx) {
    EnvelopeGen &env = envs[env_idx];

    // Process pending parameter write (deferred from register write)
    if (env.new_data) {
        env.enabled = env.pending_enabled;
        env.invert_right = env.pending_invert_right;
        env.shape = env.pending_shape;
        env.resolution = env.pending_resolution;
        env.ext_clock = env.pending_ext_clock;
        env.position = 0;
        env.new_data = false;
        return; // Don't advance on the same tick as loading
    }

    if (!env.enabled) return;

    // 3-bit resolution steps by 2 (envelope runs 2x faster, 8 distinct levels)
    // 4-bit resolution steps by 1 (16 distinct levels)
    int step = env.resolution ? 2 : 1;
    env.position = ((env.position + step) & 0x3F) | (env.position & 0x20);
}

// Get current envelope amplitude level (0-15)
// Uses MAME lookup table indexed by shape and 64-step position
uint8_t SAASound::getEnvelopeLevel(int env_idx) {
    const EnvelopeGen &env = envs[env_idx];
    if (!env.enabled) return 16; // Disabled = pass-through (16/16 = unity)

    return envelope_table[env.shape][env.position];
}

// Main audio generation — runs in RAM for speed on RP2350
IRAM_ATTR void SAASound::gen_sound(int bufsize, int bufpos) {
    uint8_t *buf_L = SamplebufSAA_L + bufpos;
    uint8_t *buf_R = SamplebufSAA_R + bufpos;

    if (!sound_enabled) {
        memset(buf_L, 0, bufsize);
        memset(buf_R, 0, bufsize);
        return;
    }

    while (bufsize-- > 0) {
        int mix_l = 0;
        int mix_r = 0;

        // Advance tone generators
        // Counter step = 1 << octave (higher octaves count faster).
        // Period stays at full resolution (511 - offset), so fractional
        // precision is preserved through accumulation — no integer truncation.
        for (int ch = 0; ch < 6; ch++) {
            channels[ch].counter += (1 << channels[ch].octave);
            while (channels[ch].counter >= channels[ch].period) {
                channels[ch].counter -= channels[ch].period;
                channels[ch].bit ^= 1;
                // Channel 1 clocks envelope 0, channel 4 clocks envelope 1
                // new_data check ensures pending params get processed even if currently disabled
                if (ch == 1 && ((envs[0].enabled && !envs[0].ext_clock) || envs[0].new_data)) {
                    advanceEnvelope(0);
                }
                if (ch == 4 && ((envs[1].enabled && !envs[1].ext_clock) || envs[1].new_data)) {
                    advanceEnvelope(1);
                }
                // Channels 0 and 3 can trigger noise generators in source mode 3
                if (ch == 0 && noise[0].source == 3) {
                    if (noise[0].lfsr & 1)
                        noise[0].lfsr = (noise[0].lfsr >> 1) ^ 0x20400;
                    else
                        noise[0].lfsr >>= 1;
                    noise[0].bit = noise[0].lfsr & 1;
                }
                if (ch == 3 && noise[1].source == 3) {
                    if (noise[1].lfsr & 1)
                        noise[1].lfsr = (noise[1].lfsr >> 1) ^ 0x20400;
                    else
                        noise[1].lfsr >>= 1;
                    noise[1].bit = noise[1].lfsr & 1;
                }
            }
        }

        // Advance noise generators (for fixed-rate sources 0-2)
        for (int ng = 0; ng < 2; ng++) {
            if (noise[ng].source < 3) {
                if (++noise[ng].counter >= noise[ng].period) {
                    noise[ng].counter = 0;
                    // 18-bit Galois LFSR with mask 0x20400 (verified against real SAA1099P)
                    if (noise[ng].lfsr & 1)
                        noise[ng].lfsr = (noise[ng].lfsr >> 1) ^ 0x20400;
                    else
                        noise[ng].lfsr >>= 1;
                    noise[ng].bit = noise[ng].lfsr & 1;
                }
            }
        }

        // Mix all 6 channels
        // Envelope only applies to channels 2 and 5 (verified by stripwax/SAASound
        // reference implementation — channels 0,1,3,4 have NULL envelope pointer).
        // Per channel max: vol_table[15] * 16 * 2 = 576
        // 6 channels max: 3456. After >>5: 108.
        for (int ch = 0; ch < 6; ch++) {
            int noise_ng = ch / 3;

            int mix_mode = (channels[ch].tone_on ? 1 : 0) | (channels[ch].noise_on ? 2 : 0);
            if (mix_mode == 0) continue;

            // Envelope only on channels 2 and 5 (last channel of each group)
            int env_group = ch / 3;
            bool use_env = (ch == 2 || ch == 5) && envs[env_group].enabled;

            // Phase inversion only when envelope is active on THIS channel
            uint8_t tone_bit = channels[ch].bit;
            if (use_env) tone_bit ^= 1;

            int out_level;
            switch (mix_mode) {
            case 1: out_level = tone_bit ? 2 : 0; break;
            case 2: out_level = noise[noise_ng].bit ? 2 : 0; break;
            case 3: out_level = tone_bit ? (2 - noise[noise_ng].bit) : 0; break;
            default: out_level = 0; break;
            }

            if (out_level > 0) {
                uint8_t al = channels[ch].amp_left;
                uint8_t ar = channels[ch].amp_right;
                uint8_t env_l, env_r;

                if (use_env) {
                    // Envelope path: mask amp bit 0, apply envelope level
                    al &= 0x0E;
                    ar &= 0x0E;
                    uint8_t env_level = getEnvelopeLevel(env_group);
                    env_l = env_level;
                    env_r = env_level;
                    if (envs[env_group].invert_right) {
                        env_r = 15 - env_level;
                    }
                } else {
                    // Non-envelope path: full amplitude, unity multiplier
                    env_l = 16;
                    env_r = 16;
                }

                mix_l += vol_table[al] * env_l * out_level;
                mix_r += vol_table[ar] * env_r * out_level;
            }
        }

        // Single division at the end: >>5 scales 0-3456 to 0-108
        // Rounding offset (+16 = half step) rescues near-threshold signals
        int out_l = (mix_l + 16) >> 5;
        int out_r = (mix_r + 16) >> 5;
        *buf_L++ = (uint8_t)(out_l > 255 ? 255 : out_l);
        *buf_R++ = (uint8_t)(out_r > 255 ? 255 : out_r);
    }
}
