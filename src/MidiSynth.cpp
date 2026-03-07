#include "MidiSynth.h"

#if !PICO_RP2040

#include <string.h>
#include "pico.h"

// MIDI note frequency table (Q16 phase increment for 31250 Hz sample rate)
// phase_inc = (freq * 65536) / 31250
// Pre-computed for notes 0..127
static uint32_t phase_inc_table[128];

// Build phase increment table at init
static void buildPhaseTable() {
    // A4 = note 69 = 440 Hz
    // freq = 440 * 2^((note - 69) / 12)
    // phase_inc = freq * 65536 / 31250
    for (int n = 0; n < 128; n++) {
        // Use fixed-point math to avoid float at runtime
        // But we compute the table once, so float is fine here
        double freq = 440.0 * __builtin_exp2((n - 69) / 12.0);
        double inc = freq * 65536.0 / MIDISYNTH_SAMPLE_RATE;
        phase_inc_table[n] = (uint32_t)(inc + 0.5);
    }
}

// Statics
uint8_t MidiSynth::midi_status = 0;
uint8_t MidiSynth::midi_data[2] = {0, 0};
uint8_t MidiSynth::midi_data_pos = 0;
uint8_t MidiSynth::midi_expected = 0;

MidiSynth::Voice MidiSynth::voices[MIDISYNTH_MAX_VOICES];
uint8_t MidiSynth::channel_program[16];
uint8_t MidiSynth::channel_volume[16];

void MidiSynth::init() {
    buildPhaseTable();
    reset();
}

void MidiSynth::reset() {
    memset(voices, 0, sizeof(voices));
    memset(channel_program, 0, sizeof(channel_program));
    for (int i = 0; i < 16; i++)
        channel_volume[i] = 100; // default GM volume
    midi_status = 0;
    midi_data_pos = 0;
    midi_expected = 0;
}

// Determine expected data byte count for a given status byte
static uint8_t dataLenForStatus(uint8_t status) {
    uint8_t hi = status & 0xF0;
    switch (hi) {
        case 0x80: return 2; // Note Off
        case 0x90: return 2; // Note On
        case 0xA0: return 2; // Poly Aftertouch
        case 0xB0: return 2; // Control Change
        case 0xC0: return 1; // Program Change
        case 0xD0: return 1; // Channel Pressure
        case 0xE0: return 2; // Pitch Bend
        default:   return 0; // System messages — ignore
    }
}

void MidiSynth::feedByte(uint8_t b) {
    if (b & 0x80) {
        // Status byte
        if (b >= 0xF8) return; // Real-time messages — ignore
        if (b >= 0xF0) {
            // System common — reset running status
            midi_status = 0;
            midi_data_pos = 0;
            midi_expected = 0;
            return;
        }
        midi_status = b;
        midi_data_pos = 0;
        midi_expected = dataLenForStatus(b);
    } else {
        // Data byte
        if (midi_expected == 0) return; // no valid status
        midi_data[midi_data_pos++] = b;
        if (midi_data_pos >= midi_expected) {
            processMessage(midi_status, midi_data[0],
                           midi_expected > 1 ? midi_data[1] : 0);
            midi_data_pos = 0; // ready for running status
        }
    }
}

void MidiSynth::processMessage(uint8_t status, uint8_t d0, uint8_t d1) {
    uint8_t ch = status & 0x0F;
    uint8_t hi = status & 0xF0;

    switch (hi) {
        case 0x90: // Note On (vel=0 means Note Off)
            if (d1 > 0)
                noteOn(ch, d0 & 0x7F, d1 & 0x7F);
            else
                noteOff(ch, d0 & 0x7F);
            break;
        case 0x80: // Note Off
            noteOff(ch, d0 & 0x7F);
            break;
        case 0xB0: // Control Change
            controlChange(ch, d0 & 0x7F, d1 & 0x7F);
            break;
        case 0xC0: // Program Change
            programChange(ch, d0 & 0x7F);
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
    // First: find free voice
    for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
        if (voices[i].velocity == 0)
            return i;
    }
    // Steal: find voice in release stage, or oldest (lowest env)
    int best = 0;
    uint8_t best_env = 255;
    for (int i = 0; i < MIDISYNTH_MAX_VOICES; i++) {
        if (voices[i].env_stage == 2) return i; // releasing — steal
        if (voices[i].env < best_env) {
            best_env = voices[i].env;
            best = i;
        }
    }
    return best;
}

uint32_t MidiSynth::noteToPhaseInc(uint8_t note) {
    if (note > 127) note = 127;
    return phase_inc_table[note];
}

// Duty cycle per program group (GM instrument families)
// 0-7: Piano(128), 8-15: Chromatic Perc(96), 16-23: Organ(192),
// 24-31: Guitar(160), 32-39: Bass(64), 40-47: Strings(128),
// 48-55: Ensemble(128), 56-63: Brass(192), 64-71: Reed(160),
// 72-79: Pipe(224), 80-87: Synth Lead(128), 88-95: Synth Pad(128),
// 96-103: Synth FX(96), 104-111: Ethnic(160), 112-119: Percussive(64),
// 120-127: SFX(128)
static const uint8_t program_duty[16] = {
    128, 96, 192, 160, 64, 128, 128, 192,
    160, 224, 128, 128, 96, 160, 64, 128
};

static uint8_t getDutyForProgram(uint8_t prog) {
    return program_duty[prog >> 3];
}

void MidiSynth::noteOn(uint8_t ch, uint8_t note, uint8_t vel) {
    // Ch10 (drums) — simple noise-like: use note as-is
    int v = findVoice(ch, note);
    if (v < 0) v = allocVoice(ch, note);

    Voice &voice = voices[v];
    voice.channel = ch;
    voice.note = note;
    voice.velocity = vel;
    voice.phase = 0;
    voice.phase_inc = noteToPhaseInc(note);
    voice.duty = getDutyForProgram(channel_program[ch]);
    voice.env = 255;       // full attack
    voice.env_stage = 0;   // attack
}

void MidiSynth::noteOff(uint8_t ch, uint8_t note) {
    int v = findVoice(ch, note);
    if (v >= 0) {
        voices[v].env_stage = 2; // release
    }
}

void MidiSynth::controlChange(uint8_t ch, uint8_t cc, uint8_t val) {
    switch (cc) {
        case 7:  // Volume
            channel_volume[ch] = val;
            break;
        case 123: // All Notes Off
        case 120: // All Sound Off
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

void __not_in_flash("midi") MidiSynth::gen_sound(uint8_t *buf_L, uint8_t *buf_R, int count) {
    for (int i = 0; i < count; i++) {
        int32_t mix = 0;

        for (int v = 0; v < MIDISYNTH_MAX_VOICES; v++) {
            Voice &vc = voices[v];
            if (vc.velocity == 0) continue;

            // Envelope processing
            switch (vc.env_stage) {
                case 0: // Attack — instant (already at 255)
                    vc.env_stage = 1;
                    break;
                case 1: // Sustain — hold
                    break;
                case 2: // Release — fast decay
                    if (vc.env > 8)
                        vc.env -= 8;
                    else {
                        vc.env = 0;
                        vc.velocity = 0; // free voice
                        continue;
                    }
                    break;
            }

            // Square wave with variable duty cycle
            uint8_t phase_hi = (vc.phase >> 8) & 0xFF;
            int32_t sample = (phase_hi < vc.duty) ? 1 : -1;

            // Amplitude: velocity * envelope * channel_volume / (127 * 255 * 127)
            // Simplified: (vel * env * ch_vol) >> 21 gives ~0..1
            // We want output range per voice ~0..16 to stay in 8-bit mix headroom
            int32_t amp = ((uint32_t)vc.velocity * vc.env * channel_volume[vc.channel]) >> 17;
            mix += sample * amp;

            vc.phase += vc.phase_inc;
        }

        // Scale and center to unsigned 8-bit
        // mix range: roughly -128..+128 with 16 voices at moderate volume
        // Shift to 0..255 range
        mix = (mix >> 1) + 128;
        if (mix < 0) mix = 0;
        if (mix > 255) mix = 255;

        buf_L[i] = (uint8_t)mix;
        buf_R[i] = (uint8_t)mix;
    }
}

#endif // !PICO_RP2040
