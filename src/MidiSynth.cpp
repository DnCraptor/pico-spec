#include "MidiSynth.h"

#if !PICO_RP2040

#include <string.h>
#include "pico.h"
#include "Config.h"

// MIDI note frequency table (Q24 phase increment for 31250 Hz sample rate)
// phase_inc = (freq * 16777216) / 31250
static uint32_t phase_inc_table[128];

// Build phase increment table at init
static void buildPhaseTable() {
    for (int n = 0; n < 128; n++) {
        double freq = 440.0 * __builtin_exp2((n - 69) / 12.0);
        double inc = freq * 16777216.0 / MIDISYNTH_SAMPLE_RATE;
        phase_inc_table[n] = (uint32_t)(inc + 0.5);
    }
}

// Calculate phase increment with pitch bend
// bend: -8192..+8191, range ±2 semitones (GM default)
static uint32_t calcPhaseInc(uint8_t note, int16_t bend) {
    if (bend == 0) return phase_inc_table[note];

    int32_t semi_q12 = (int32_t)bend * 2 * 4096 / 8192;
    int32_t note_q12 = ((int32_t)note << 12) + semi_q12;
    if (note_q12 < 0) note_q12 = 0;
    if (note_q12 > (127 << 12)) note_q12 = 127 << 12;

    int base = note_q12 >> 12;
    uint32_t frac = note_q12 & 0xFFF;

    uint32_t inc0 = phase_inc_table[base];
    uint32_t inc1 = (base < 127) ? phase_inc_table[base + 1] : inc0;

    return inc0 + (((int32_t)(inc1 - inc0) * (int32_t)frac) >> 12);
}

// Simple sine approximation for vibrato LFO (-256..+256)
static int16_t sinApprox(uint16_t phase10) {
    uint16_t p = phase10 & 0x3FF;
    int16_t half;
    if (p < 256)       half = (int16_t)p;
    else if (p < 512)  half = 511 - (int16_t)p;
    else if (p < 768)  half = -((int16_t)p - 512);
    else               half = (int16_t)p - 1024;
    return half;
}

// Statics
uint8_t MidiSynth::midi_status = 0;
uint8_t MidiSynth::midi_data[2] = {0, 0};
uint8_t MidiSynth::midi_data_pos = 0;
uint8_t MidiSynth::midi_expected = 0;
uint8_t MidiSynth::preset = 0;

MidiSynth::Voice MidiSynth::voices[MIDISYNTH_MAX_VOICES];
uint8_t MidiSynth::channel_program[16];
uint8_t MidiSynth::channel_volume[16];
uint8_t MidiSynth::channel_expression[16];
uint8_t MidiSynth::channel_modulation[16];
uint8_t MidiSynth::channel_pan[16];
int16_t MidiSynth::channel_pitchbend[16];

void MidiSynth::init() {
    buildPhaseTable();
    preset = Config::midi_synth_preset;
    reset();
}

void MidiSynth::reset() {
    memset(voices, 0, sizeof(voices));
    memset(channel_program, 0, sizeof(channel_program));
    memset(channel_pitchbend, 0, sizeof(channel_pitchbend));
    memset(channel_modulation, 0, sizeof(channel_modulation));
    for (int i = 0; i < 16; i++) {
        channel_volume[i] = 100;
        channel_expression[i] = 127;
        channel_pan[i] = 64;
    }
    midi_status = 0;
    midi_data_pos = 0;
    midi_expected = 0;
}

static uint8_t dataLenForStatus(uint8_t status) {
    uint8_t hi = status & 0xF0;
    switch (hi) {
        case 0x80: return 2;
        case 0x90: return 2;
        case 0xA0: return 2;
        case 0xB0: return 2;
        case 0xC0: return 1;
        case 0xD0: return 1;
        case 0xE0: return 2;
        default:   return 0;
    }
}

void MidiSynth::feedByte(uint8_t b) {
    if (b & 0x80) {
        if (b >= 0xF8) return;
        if (b >= 0xF0) {
            midi_status = 0;
            midi_data_pos = 0;
            midi_expected = 0;
            return;
        }
        midi_status = b;
        midi_data_pos = 0;
        midi_expected = dataLenForStatus(b);
    } else {
        if (midi_expected == 0) return;
        midi_data[midi_data_pos++] = b;
        if (midi_data_pos >= midi_expected) {
            processMessage(midi_status, midi_data[0],
                           midi_expected > 1 ? midi_data[1] : 0);
            midi_data_pos = 0;
        }
    }
}

void MidiSynth::processMessage(uint8_t status, uint8_t d0, uint8_t d1) {
    uint8_t ch = status & 0x0F;
    uint8_t hi = status & 0xF0;

    switch (hi) {
        case 0x90:
            if (d1 > 0)
                noteOn(ch, d0 & 0x7F, d1 & 0x7F);
            else
                noteOff(ch, d0 & 0x7F);
            break;
        case 0x80:
            noteOff(ch, d0 & 0x7F);
            break;
        case 0xB0:
            controlChange(ch, d0 & 0x7F, d1 & 0x7F);
            break;
        case 0xC0:
            programChange(ch, d0 & 0x7F);
            break;
        case 0xE0:
            pitchBend(ch, ((int16_t)((d1 << 7) | d0)) - 8192);
            break;
    }
}

int MidiSynth::findVoice(uint8_t ch, uint8_t note) {
    for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
        if (voices[i].velocity && voices[i].channel == ch && voices[i].note == note)
            return i;
    }
    return -1;
}

int MidiSynth::allocVoice(uint8_t ch, uint8_t note) {
    for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
        if (voices[i].velocity == 0)
            return i;
    }
    int best = 0;
    uint8_t best_env = 255;
    for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
        if (voices[i].env_stage == 3) return i;
        if (voices[i].env < best_env) {
            best_env = voices[i].env;
            best = i;
        }
    }
    return best;
}

// Preset-aware patch parameters
// {wave, duty, attack, decay, sustain, release, filter_k}
MidiSynth::PatchParams MidiSynth::getPatch(uint8_t ch, uint8_t program) {
    // Channel 10 (9 zero-based) = percussion — always noise
    if (ch == 9) {
        return { WAVE_NOISE, 128, 255, 40, 0, 30, 0 };
    }

    uint8_t group = program >> 3;

    // Chiptune duty varies by group for variety
    static const uint8_t chip_duty[16] = {
        128, 96, 192, 160, 64, 128, 128, 192,
        160, 224, 128, 128, 96, 160, 64, 128
    };

    switch (preset) {
        default:
        case 0: // GM — mixed waveforms per instrument family
            switch (group) {
                case 0:  return { WAVE_TRIANGLE, 128, 64, 12, 160, 16, 180 };  // Piano
                case 1:  return { WAVE_TRIANGLE, 128, 128, 20, 80, 24, 140 };  // Chrom Perc
                case 2:  return { WAVE_SQUARE, 192, 32, 4, 230, 12, 60 };      // Organ
                case 3:  return { WAVE_SAW, 128, 64, 16, 140, 20, 160 };       // Guitar
                case 4:  return { WAVE_SAW, 128, 48, 10, 180, 16, 200 };       // Bass
                case 5:  return { WAVE_SAW, 128, 8, 4, 220, 8, 180 };          // Strings
                case 6:  return { WAVE_SAW, 128, 6, 4, 210, 6, 160 };          // Ensemble
                case 7:  return { WAVE_SQUARE, 192, 16, 6, 200, 12, 120 };     // Brass
                case 8:  return { WAVE_SQUARE, 160, 24, 8, 200, 14, 140 };     // Reed
                case 9:  return { WAVE_TRIANGLE, 128, 20, 6, 220, 10, 100 };   // Pipe
                case 10: return { WAVE_SAW, 128, 32, 4, 230, 12, 80 };         // Synth Lead
                case 11: return { WAVE_SAW, 128, 4, 4, 220, 4, 200 };          // Synth Pad
                case 12: return { WAVE_SQUARE, 96, 16, 8, 180, 16, 100 };      // Synth FX
                case 13: return { WAVE_SAW, 128, 48, 12, 160, 16, 120 };       // Ethnic
                case 14: return { WAVE_NOISE, 128, 128, 32, 60, 24, 60 };      // Percussive
                case 15: return { WAVE_NOISE, 128, 24, 8, 140, 12, 80 };       // SFX
                default: return { WAVE_SQUARE, 128, 32, 8, 200, 12, 120 };
            }

        case 1: // Piano — all triangle, natural decay
            return { WAVE_TRIANGLE, 128, 64, 12, 120, 16, 160 };

        case 2: // Chiptune — all square, varied duty, no filter, fast
            return { WAVE_SQUARE, chip_duty[group], 128, 8, 220, 16, 0 };

        case 3: // Strings — all saw, slow attack, long sustain, warm
            return { WAVE_SAW, 128, 4, 4, 230, 4, 200 };

        case 4: // Rock — bright, punchy
            if (group == 2 || group == 7 || group == 8)
                return { WAVE_SQUARE, 160, 64, 8, 200, 16, 40 };
            else
                return { WAVE_SAW, 128, 64, 8, 200, 16, 40 };

        case 5: // Organ — all square, sustained
            return { WAVE_SQUARE, 192, 32, 2, 240, 8, 40 };

        case 6: // Music Box — triangle, fast decay, low sustain
            return { WAVE_TRIANGLE, 128, 128, 24, 40, 16, 180 };

        case 7: // Synth — all saw, medium filter
            return { WAVE_SAW, 128, 32, 4, 220, 8, 120 };
    }
}

void MidiSynth::noteOn(uint8_t ch, uint8_t note, uint8_t vel) {
    int v = findVoice(ch, note);
    if (v < 0) v = allocVoice(ch, note);

    Voice &voice = voices[v];
    PatchParams p = getPatch(ch, channel_program[ch]);

    voice.channel = ch;
    voice.note = note;
    voice.velocity = vel;
    voice.phase = 0;
    voice.phase_inc = calcPhaseInc(note, channel_pitchbend[ch]);
    voice.duty = p.duty;
    voice.wave = p.wave;
    voice.env = 0;
    voice.env_stage = 0;
    voice.env_target = p.sustain_level;
    voice.attack_rate = p.attack_rate;
    voice.decay_rate = p.decay_rate;
    voice.release_rate = p.release_rate;
    voice.filter_k = p.filter_k;
    voice.filter_z = 128;

    if (voice.wave == WAVE_NOISE) {
        voice.noise_lfsr = 0x7FFFF;
        voice.phase_inc = phase_inc_table[note < 127 ? note : 127];
    }
}

void MidiSynth::noteOff(uint8_t ch, uint8_t note) {
    int v = findVoice(ch, note);
    if (v >= 0) {
        voices[v].env_stage = 3;
    }
}

void MidiSynth::controlChange(uint8_t ch, uint8_t cc, uint8_t val) {
    switch (cc) {
        case 1:  channel_modulation[ch] = val; break;
        case 7:  channel_volume[ch] = val; break;
        case 10: channel_pan[ch] = val; break;
        case 11: channel_expression[ch] = val; break;
        case 121:
            channel_modulation[ch] = 0;
            channel_pitchbend[ch] = 0;
            channel_volume[ch] = 100;
            channel_expression[ch] = 127;
            channel_pan[ch] = 64;
            break;
        case 123:
        case 120:
            for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
                if (voices[i].channel == ch)
                    voices[i].velocity = 0;
            }
            break;
    }
}

void MidiSynth::programChange(uint8_t ch, uint8_t prog) {
    channel_program[ch] = prog;
}

void MidiSynth::pitchBend(uint8_t ch, int16_t bend) {
    channel_pitchbend[ch] = bend;
    for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
        if (voices[i].velocity && voices[i].channel == ch) {
            voices[i].phase_inc = calcPhaseInc(voices[i].note, bend);
        }
    }
}

// Vibrato LFO — global phase counter, ~6 Hz at 31250 Hz sample rate
static uint32_t vibrato_phase = 0;
#define VIBRATO_INC 197

void __not_in_flash("midi") MidiSynth::gen_sound(uint8_t *buf_L, uint8_t *buf_R, int count) {
    for (int i = 0; i < count; i++) {
        int32_t mix_L = 0;
        int32_t mix_R = 0;

        // Advance vibrato LFO
        vibrato_phase = (vibrato_phase + VIBRATO_INC) & 0x7FFF;
        int16_t vib_sin = sinApprox(vibrato_phase >> 5);

        for (int v = 0; v < MIDISYNTH_MAX_VOICES; v++) {
            Voice &vc = voices[v];
            if (vc.velocity == 0) continue;

            // ADSR envelope
            switch (vc.env_stage) {
                case 0: // Attack
                    if ((int)vc.env + vc.attack_rate < 255) {
                        vc.env += vc.attack_rate;
                    } else {
                        vc.env = 255;
                        vc.env_stage = 1;
                    }
                    break;
                case 1: // Decay
                    if (vc.env > vc.env_target + vc.decay_rate) {
                        vc.env -= vc.decay_rate;
                    } else {
                        vc.env = vc.env_target;
                        vc.env_stage = 2;
                    }
                    break;
                case 2: // Sustain
                    break;
                case 3: // Release
                    if (vc.env > vc.release_rate) {
                        vc.env -= vc.release_rate;
                    } else {
                        vc.env = 0;
                        vc.velocity = 0;
                        continue;
                    }
                    break;
            }

            // Vibrato from modulation wheel
            uint32_t phase_inc = vc.phase_inc;
            uint8_t mod = channel_modulation[vc.channel];
            if (mod > 0) {
                int32_t vib = ((int32_t)(phase_inc >> 10) * mod * vib_sin) >> 17;
                phase_inc = (uint32_t)((int32_t)phase_inc + vib);
            }

            // Waveform generation — signed -128..+127
            uint8_t phase_hi = (vc.phase >> 16) & 0xFF;
            int32_t sample;

            switch (vc.wave) {
                case WAVE_SQUARE:
                    sample = (phase_hi < vc.duty) ? 127 : -128;
                    break;
                case WAVE_SAW:
                    sample = (int32_t)phase_hi - 128;
                    break;
                case WAVE_TRIANGLE:
                    if (phase_hi < 128)
                        sample = (int32_t)phase_hi * 2 - 128;
                    else
                        sample = 382 - (int32_t)phase_hi * 2;
                    break;
                case WAVE_NOISE: {
                    uint32_t old_phase = vc.phase;
                    if (((old_phase >> 16) & 0xFF) > phase_hi) {
                        uint32_t lfsr = vc.noise_lfsr;
                        lfsr = (lfsr >> 1) ^ ((lfsr & 1) ? 0x20400 : 0);
                        vc.noise_lfsr = lfsr;
                    }
                    sample = (vc.noise_lfsr & 0xFF) - 128;
                    break;
                }
            }

            // Shift to unsigned 0..255 for unipolar mixing (like AY/SAA)
            int32_t usample = sample + 128;

            // 1-pole low-pass filter (unsigned domain 0..255)
            if (vc.filter_k > 0) {
                int16_t diff = (int16_t)usample - vc.filter_z;
                uint8_t alpha = 255 - vc.filter_k;
                if (alpha < 8) alpha = 8;
                vc.filter_z += (diff * alpha) >> 8;
                usample = vc.filter_z;
            }

            // Amplitude: velocity * envelope * channel_volume
            // usample 0..255, amp 0..~125 → voiced 0..~996
            int32_t amp = ((uint32_t)vc.velocity * vc.env * channel_volume[vc.channel]) >> 17;
            int32_t voiced = (usample * amp) >> 5;

            // Stereo pan
            uint8_t pan = channel_pan[vc.channel];
            uint16_t pan_R = pan * 2;
            uint16_t pan_L = 254 - pan_R;
            mix_L += (voiced * pan_L) >> 8;
            mix_R += (voiced * pan_R) >> 8;

            vc.phase = (vc.phase + phase_inc) & 0xFFFFFF;
        }

        // Soft clip to unsigned 0..255 (silence = 0)
        if (mix_L > 240) mix_L = 240 + ((mix_L - 240) >> 2);
        if (mix_R > 240) mix_R = 240 + ((mix_R - 240) >> 2);
        if (mix_L > 255) mix_L = 255;
        if (mix_R > 255) mix_R = 255;

        buf_L[i] = (uint8_t)mix_L;
        buf_R[i] = (uint8_t)mix_R;
    }
}

#endif // !PICO_RP2040
