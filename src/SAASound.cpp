/*
  SAA1099 Sound Chip Emulation for pico-spec ZX Spectrum emulator

  Based on stripwax/SAASound by Dave Hooper — verified against real SAA1099P.
  https://github.com/stripwax/SAASound

  Flattened from CSAAFreq, CSAANoise, CSAAEnv, CSAAAmp, CSAADevice into a
  single class. All logic ported faithfully:

  - Tone: integer step/period counter (mathematically equivalent to stripwax's
    fixed-point table). Octave/offset buffering with Philips documented quirk.
  - Noise: 18-bit Galois LFSR (x^18+x^11+x^1, mask 0x20400, seed 0xFFFFFFFF).
    Sources 0-2 counter-based, source 3 triggered by tone ch0/ch3.
  - Envelope: phase-based with resolution switching, buffered parameter updates
    applied only at natural phase boundaries (stripwax CSAAEnv::Tick logic).
  - Mixer: intermediate 0/1/2 model with PDM effective amplitude for envelope
    channels (stripwax SAAAmp).
  - Envelope only applies to ch2 (env0) and ch5 (env1).

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

// Envelope shape data — from stripwax CSAAEnv::cs_EnvData.
// [resolution 0=4bit, 1=3bit][phase 0-1][position 0-15]
const SAASound::EnvShape SAASound::env_shapes[8] = {
    /* 0: zero */
    {1, false, {{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}},
                 {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}},
    /* 1: maximum */
    {1, true,  {{{15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15},
                  {15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15}},
                 {{14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14},
                  {14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14}}}},
    /* 2: single decay */
    {1, false, {{{15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}},
                 {{14,14,12,12,10,10,8,8,6,6,4,4,2,2,0,0},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}},
    /* 3: repetitive decay */
    {1, true,  {{{15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}},
                 {{14,14,12,12,10,10,8,8,6,6,4,4,2,2,0,0},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}},
    /* 4: single triangular */
    {2, false, {{{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
                  {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0}},
                 {{0,0,2,2,4,4,6,6,8,8,10,10,12,12,14,14},
                  {14,14,12,12,10,10,8,8,6,6,4,4,2,2,0,0}}}},
    /* 5: repetitive triangular */
    {2, true,  {{{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
                  {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0}},
                 {{0,0,2,2,4,4,6,6,8,8,10,10,12,12,14,14},
                  {14,14,12,12,10,10,8,8,6,6,4,4,2,2,0,0}}}},
    /* 6: single attack */
    {1, false, {{{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}},
                 {{0,0,2,2,4,4,6,6,8,8,10,10,12,12,14,14},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}},
    /* 7: repetitive attack */
    {1, true,  {{{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}},
                 {{0,0,2,2,4,4,6,6,8,8,10,10,12,12,14,14},
                  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}}}
};

// PDM effective amplitude table — from stripwax CSAAAmp::EffectiveAmplitude.
// Models analog PDM interaction of amplitude and envelope signals on real chip.
// Pre-multiplied by 4: pdm_x4[amp/2][env_level]
const uint16_t SAASound::pdm_x4[8][16] = {
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {  0,  4,  4,  8,  8, 12, 12, 16, 16, 20, 20, 24, 24, 28, 28, 32},
    {  0,  4,  8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60},
    {  0,  8, 12, 20, 24, 32, 36, 44, 48, 56, 60, 68, 72, 80, 84, 92},
    {  0,  8, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96,104,112,120},
    {  0, 12, 20, 32, 40, 52, 60, 72, 80, 92,100,112,120,132,140,152},
    {  0, 12, 24, 36, 48, 60, 72, 84, 96,108,120,132,144,156,168,180},
    {  0, 16, 28, 44, 56, 72, 84,100,112,128,140,156,168,184,196,212}
};

SAASound saaChip;

SAASound::SAASound() {
    init();
}

void SAASound::init() {
    selectedRegister = 0;
    outputEnabled = false;
    syncState = false;
    memset(regs, 0, sizeof(regs));
    memset(channels, 0, sizeof(channels));
    memset(noise, 0, sizeof(noise));
    memset(envs, 0, sizeof(envs));

    for (int i = 0; i < 6; i++) {
        channels[i].period = 511;
        channels[i].level = 1; // INITIAL_LEVEL from stripwax
    }

    // LFSR seed 0xFFFFFFFF (stripwax CSAANoise constructor)
    noise[0].rand = 0xFFFFFFFF;
    noise[1].rand = 0xFFFFFFFF;

    for (int i = 0; i < 2; i++) {
        envs[i].envelope_ended = true;
        envs[i].resolution = 1;
        envs[i].left_level = 0;
        envs[i].right_level = 0;
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

//////////////////////////////////////////////////////////////////////
// Register address select — also triggers external envelope clock
// (from stripwax CSAADevice::_WriteAddress)
//////////////////////////////////////////////////////////////////////
void SAASound::selectRegister(uint8_t reg) {
    selectedRegister = reg & 0x1F;
    // External envelope clock: only when selecting the corresponding
    // envelope register. Env0 on reg 24, Env1 on reg 25.
    if (selectedRegister == 24) {
        if (envs[0].clock_externally && envs[0].enabled) envTick(0);
    } else if (selectedRegister == 25) {
        if (envs[1].clock_externally && envs[1].enabled) envTick(1);
    }
}

uint8_t SAASound::getRegisterData() {
    if (selectedRegister > 0x1F) return 0xFF;
    return regs[selectedRegister];
}

//////////////////////////////////////////////////////////////////////
// Register data write (from stripwax CSAADevice::_WriteData)
//////////////////////////////////////////////////////////////////////
void SAASound::setRegisterData(uint8_t data) {
    if (selectedRegister > 0x1F) return;
    regs[selectedRegister] = data;

    switch (selectedRegister) {
    // Amplitude (=> CSAAAmp::SetAmpLevel)
    case 0: case 1: case 2: case 3: case 4: case 5:
        channels[selectedRegister].amp_left = data & 0x0F;
        channels[selectedRegister].amp_right = (data >> 4) & 0x0F;
        break;

    // Frequency offset (=> CSAAFreq::SetFreqOffset)
    case 8: case 9: case 10: case 11: case 12: case 13: {
        int ch = selectedRegister - 8;
        if (!syncState) {
            channels[ch].next_offset = data;
            channels[ch].new_data = true;
            // Philips quirk: if offset written without octave change,
            // defer offset to the half-cycle after next
            if (channels[ch].next_octave == channels[ch].octave)
                channels[ch].ignore_offset = true;
        } else {
            // During sync: apply immediately
            channels[ch].new_data = false;
            channels[ch].ignore_offset = false;
            channels[ch].freq_offset = data;
            channels[ch].next_offset = data;
            channels[ch].octave = channels[ch].next_octave;
            int p = 511 - (int)data;
            channels[ch].period = (p < 1) ? 1 : (uint32_t)p;
        }
        break;
    }

    // Octave (=> CSAAFreq::SetFreqOctave)
    case 16: case 17: case 18: {
        int base = (selectedRegister - 16) * 2;
        uint8_t oct_lo = data & 0x07;
        uint8_t oct_hi = (data >> 4) & 0x07;
        for (int i = 0; i < 2; i++) {
            int ch = base + i;
            uint8_t oct = (i == 0) ? oct_lo : oct_hi;
            if (!syncState) {
                channels[ch].next_octave = oct;
                channels[ch].new_data = true;
                channels[ch].ignore_offset = false;
            } else {
                channels[ch].new_data = false;
                channels[ch].ignore_offset = false;
                channels[ch].octave = oct;
                channels[ch].next_octave = oct;
                channels[ch].freq_offset = channels[ch].next_offset;
                int p = 511 - (int)channels[ch].freq_offset;
                channels[ch].period = (p < 1) ? 1 : (uint32_t)p;
            }
        }
        break;
    }

    // Tone mixer (=> CSAAAmp::SetToneMixer, bit 0 of mix_mode)
    case 20:
        for (int i = 0; i < 6; i++) {
            if (data & (1 << i))
                channels[i].mix_mode |= 1;
            else
                channels[i].mix_mode &= ~1;
        }
        break;

    // Noise mixer (=> CSAAAmp::SetNoiseMixer, bit 1 of mix_mode)
    case 21:
        for (int i = 0; i < 6; i++) {
            if (data & (1 << i))
                channels[i].mix_mode |= 2;
            else
                channels[i].mix_mode &= ~2;
        }
        break;

    // Noise source (=> CSAANoise::SetSource)
    case 22:
        noise[0].source = data & 0x03;
        noise[1].source = (data >> 4) & 0x03;
        break;

    // Envelope 0 (=> CSAAEnv::SetEnvControl)
    case 24:
        envSetControl(0, data);
        break;

    // Envelope 1 (=> CSAAEnv::SetEnvControl)
    case 25:
        envSetControl(1, data);
        break;

    // Global enable and sync (=> CSAADevice register 28)
    case 28: {
        bool newSync = (data & 0x02) != 0;
        if (newSync != syncState) {
            if (newSync) {
                // Sync ON: reset tone and noise counters
                for (int i = 0; i < 6; i++) {
                    channels[i].counter = 0;
                    channels[i].level = 1; // INITIAL_LEVEL
                    // Apply pending freq data immediately
                    channels[i].octave = channels[i].next_octave;
                    channels[i].freq_offset = channels[i].next_offset;
                    int p = 511 - (int)channels[i].freq_offset;
                    channels[i].period = (p < 1) ? 1 : (uint32_t)p;
                    channels[i].new_data = false;
                    channels[i].ignore_offset = false;
                }
                for (int i = 0; i < 2; i++) {
                    noise[i].counter = 0;
                }
                // Note: envelopes are NOT synced (per stripwax)
            }
            syncState = newSync;
        }

        bool newEnabled = (data & 0x01) != 0;
        outputEnabled = newEnabled;
        break;
    }

    default:
        break;
    }
}

//////////////////////////////////////////////////////////////////////
// Tone generator: update buffered octave/offset on half-cycle
// (from stripwax CSAAFreq::UpdateOctaveOffsetData)
//////////////////////////////////////////////////////////////////////
void SAASound::toneUpdateData(int ch) {
    Channel &c = channels[ch];
    if (!c.new_data) return;

    // Always apply octave
    c.octave = c.next_octave;
    // Only apply offset if not ignored (Philips quirk)
    if (!c.ignore_offset) {
        c.freq_offset = c.next_offset;
        c.new_data = false;
    }
    c.ignore_offset = false;

    // Recompute period
    int p = 511 - (int)c.freq_offset;
    c.period = (p < 1) ? 1 : (uint32_t)p;
}

//////////////////////////////////////////////////////////////////////
// Envelope: SetEnvControl (from stripwax CSAAEnv::SetEnvControl)
//////////////////////////////////////////////////////////////////////
void SAASound::envSetControl(int env, uint8_t data) {
    EnvelopeGen &e = envs[env];

    bool bEnabled = (data & 0x80) != 0;

    // If was disabled and still disabled, nothing to do
    if (!bEnabled && !e.enabled) return;

    e.enabled = bEnabled;
    if (!e.enabled) {
        // Disabling: mark as ended
        e.envelope_ended = true;
        return;
    }

    // Resolution is immediate (with phase position adjustment)
    uint8_t new_res = (data & 0x10) ? 2 : 1;
    if (e.resolution == 1 && new_res == 2) {
        e.phase_position &= 0x0E; // 4-bit→3-bit: clear LSB
    } else if (e.resolution == 2 && new_res == 1) {
        e.phase_position |= 0x01; // 3-bit→4-bit: set LSB
    }
    e.resolution = new_res;

    // Buffered parameters: apply immediately if envelope ended,
    // otherwise buffer until phase completion
    if (e.envelope_ended) {
        envSetNewData(env, data);
        e.new_data = false;
    } else {
        // Update levels for possible resolution change
        envSetLevels(env);
        // Buffer new data
        e.new_data = true;
        e.next_data = data;
    }
}

//////////////////////////////////////////////////////////////////////
// Envelope: Tick (from stripwax CSAAEnv::Tick)
//////////////////////////////////////////////////////////////////////
void SAASound::envTick(int env) {
    EnvelopeGen &e = envs[env];

    if (!e.enabled) {
        e.envelope_ended = true;
        e.phase = 0;
        e.phase_position = 0;
        return;
    }

    if (e.envelope_ended) return;

    // Advance position
    e.phase_position += e.resolution;

    bool bProcessNewData = false;
    if (e.phase_position >= 16) {
        e.phase++;
        if (e.phase == e.num_phases) {
            if (!e.looping) {
                // Non-looping: envelope ended at sustain level (0)
                e.envelope_ended = true;
                bProcessNewData = true;
            } else {
                // Looping: restart from phase 0
                e.envelope_ended = false;
                e.phase = 0;
                e.phase_position -= 16;
                bProcessNewData = true;
            }
        } else {
            // Middle of multi-phase envelope (triangular shapes)
            e.envelope_ended = false;
            e.phase_position -= 16;
        }
    } else {
        // Still within same phase
        e.envelope_ended = false;
    }

    // Apply buffered data at natural boundary
    if (e.new_data && bProcessNewData) {
        e.new_data = false;
        envSetNewData(env, e.next_data);
    } else {
        envSetLevels(env);
    }
}

//////////////////////////////////////////////////////////////////////
// Envelope: SetLevels (from stripwax CSAAEnv::SetLevels)
//////////////////////////////////////////////////////////////////////
void SAASound::envSetLevels(int env) {
    EnvelopeGen &e = envs[env];
    const EnvShape &shape = env_shapes[e.shape];

    if (e.resolution == 1) {
        // 4-bit resolution
        if (e.envelope_ended && !e.looping)
            e.left_level = 0;
        else
            e.left_level = shape.levels[0][e.phase][e.phase_position];
        if (e.invert_right)
            e.right_level = 15 - e.left_level;
        else
            e.right_level = e.left_level;
    } else {
        // 3-bit resolution
        if (e.envelope_ended && !e.looping)
            e.left_level = 0;
        else
            e.left_level = shape.levels[1][e.phase][e.phase_position];
        if (e.invert_right)
            e.right_level = 14 - e.left_level;
        else
            e.right_level = e.left_level;
    }
}

//////////////////////////////////////////////////////////////////////
// Envelope: SetNewEnvData (from stripwax CSAAEnv::SetNewEnvData)
//////////////////////////////////////////////////////////////////////
void SAASound::envSetNewData(int env, uint8_t data) {
    EnvelopeGen &e = envs[env];

    e.phase = 0;
    e.phase_position = 0;
    e.shape = (data >> 1) & 0x07;
    e.invert_right = (data & 0x01) != 0;
    e.clock_externally = (data & 0x20) != 0;
    e.num_phases = env_shapes[e.shape].num_phases;
    e.looping = env_shapes[e.shape].looping;
    e.resolution = (data & 0x10) ? 2 : 1;
    e.enabled = (data & 0x80) != 0;

    if (e.enabled) {
        e.envelope_ended = false;
    } else {
        e.envelope_ended = true;
        e.phase = 0;
        e.phase_position = 0;
    }

    envSetLevels(env);
}

//////////////////////////////////////////////////////////////////////
// Main audio generation — runs in RAM for speed on RP2350
// (from stripwax CSAADevice::_TickAndOutputStereo)
//
// Tick order matches stripwax:
//   1. Noise generators (sources 0-2)
//   2. For each channel 0-5: tone tick → mix → accumulate
//      Tone tick may trigger noise (source 3) or envelope (internal clock)
//////////////////////////////////////////////////////////////////////
IRAM_ATTR void SAASound::gen_sound(int bufsize, int bufpos) {
    uint8_t *buf_L = SamplebufSAA_L + bufpos;
    uint8_t *buf_R = SamplebufSAA_R + bufpos;

    while (bufsize-- > 0) {
        // During sync: output silence, don't advance generators
        if (syncState) {
            *buf_L++ = 0;
            *buf_R++ = 0;
            continue;
        }

        // 1. Tick noise generators (sources 0-2 only)
        for (int ng = 0; ng < 2; ng++) {
            if (noise[ng].source < 3) {
                uint32_t period;
                switch (noise[ng].source) {
                case 0: period = 1; break;  // 31250 Hz
                case 1: period = 2; break;  // 15625 Hz
                default: period = 4; break; // 7812.5 Hz
                }
                noise[ng].counter++;
                while (noise[ng].counter >= period) {
                    noise[ng].counter -= period;
                    // 18-bit Galois LFSR (x^18+x^11+x^1, verified against SAA1099P)
                    if (noise[ng].rand & 1)
                        noise[ng].rand = (noise[ng].rand >> 1) ^ 0x20400;
                    else
                        noise[ng].rand >>= 1;
                }
            }
        }

        // 2. Process channels 0-5
        int output_l = 0, output_r = 0;

        for (int ch = 0; ch < 6; ch++) {
            Channel &c = channels[ch];
            int ng = ch / 3;

            // --- Tone tick (CSAAFreq::Tick) ---
            c.counter += (1 << c.octave);
            while (c.counter >= c.period) {
                c.counter -= c.period;
                c.level ^= 1;

                // Trigger connected devices (from CSAAFreq constructor wiring):
                // ch0 → noise[0] source 3, ch3 → noise[1] source 3
                if (ch == 0 && noise[0].source == 3) {
                    if (noise[0].rand & 1)
                        noise[0].rand = (noise[0].rand >> 1) ^ 0x20400;
                    else
                        noise[0].rand >>= 1;
                }
                if (ch == 3 && noise[1].source == 3) {
                    if (noise[1].rand & 1)
                        noise[1].rand = (noise[1].rand >> 1) ^ 0x20400;
                    else
                        noise[1].rand >>= 1;
                }
                // ch1 → env[0] internal clock, ch4 → env[1] internal clock
                if (ch == 1 && envs[0].enabled && !envs[0].clock_externally)
                    envTick(0);
                if (ch == 4 && envs[1].enabled && !envs[1].clock_externally)
                    envTick(1);

                // Update buffered octave/offset on half-cycle completion
                toneUpdateData(ch);
            }

            // --- Mixer (CSAAAmp::Tick + TickAndOutputStereo) ---
            int tone_level = c.level;
            int noise_level = noise[ng].rand & 1;
            int intermediate;

            switch (c.mix_mode) {
            case 0: intermediate = 0; break;
            case 1: intermediate = tone_level * 2; break;
            case 2: intermediate = noise_level * 2; break;
            case 3: intermediate = tone_level * (2 - noise_level); break;
            default: intermediate = 0; break;
            }

            // Output amplitude
            if (!outputEnabled) {
                // Global mute — no output
            } else if ((ch == 2 && envs[0].enabled) || (ch == 5 && envs[1].enabled)) {
                // Envelope channel with active envelope: use PDM table
                int env_idx = ch / 3;
                int el = envs[env_idx].left_level;
                int er = envs[env_idx].right_level;
                int al_div2 = c.amp_left >> 1;
                int ar_div2 = c.amp_right >> 1;
                output_l += pdm_x4[al_div2][el] * (2 - intermediate);
                output_r += pdm_x4[ar_div2][er] * (2 - intermediate);
            } else {
                // Non-envelope channel: simple amplitude * intermediate
                output_l += c.amp_left * intermediate * 16;
                output_r += c.amp_right * intermediate * 16;
            }
        }

        // 3. Scale to uint8_t
        // Max per channel (non-env): 15 * 2 * 16 = 480
        // Max per channel (env): pdm_x4[7][15] * 2 = 212 * 2 = 424
        // 6 channels max: ~2880. >>4 = 180.
        int out_l = (output_l + 8) >> 4;
        int out_r = (output_r + 8) >> 4;
        *buf_L++ = (uint8_t)(out_l > 255 ? 255 : out_l);
        *buf_R++ = (uint8_t)(out_r > 255 ? 255 : out_r);
    }
}
