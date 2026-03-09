#pragma once

#if !PICO_RP2040

#include <inttypes.h>

// Polyphonic MIDI synthesizer with multiple waveforms, filters, and percussion
// 16 voices, General MIDI program → waveform/duty mapping
// Output: 8-bit unsigned mono mixed into L/R buffers

#define MIDISYNTH_MAX_VOICES 16
#define MIDISYNTH_SAMPLE_RATE 31250

class MidiSynth {
public:
    static void init();
    static void reset();

    // Feed raw MIDI byte (called from Midi::send path)
    static void feedByte(uint8_t b);

    // Generate audio samples into buffer (called once per frame)
    static void gen_sound(uint8_t *buf_L, uint8_t *buf_R, int count);

    static uint8_t preset; // synth preset (mirrors Config::midi_synth_preset)

private:
    // MIDI parser state
    static uint8_t midi_status;     // running status byte
    static uint8_t midi_data[2];    // data bytes buffer
    static uint8_t midi_data_pos;   // how many data bytes collected
    static uint8_t midi_expected;   // how many data bytes expected

    static void processMessage(uint8_t status, uint8_t d0, uint8_t d1);
    static void noteOn(uint8_t ch, uint8_t note, uint8_t vel);
    static void noteOff(uint8_t ch, uint8_t note);
    static void controlChange(uint8_t ch, uint8_t cc, uint8_t val);
    static void programChange(uint8_t ch, uint8_t prog);
    static void pitchBend(uint8_t ch, int16_t bend);

    // Waveform types
    enum WaveType : uint8_t {
        WAVE_SQUARE = 0,
        WAVE_SAW    = 1,
        WAVE_TRIANGLE = 2,
        WAVE_NOISE  = 3,    // for percussion
    };

    // Voice allocation
    struct Voice {
        uint8_t  channel;    // MIDI channel
        uint8_t  note;       // MIDI note number
        uint8_t  velocity;   // 0 = free
        uint32_t phase;      // phase accumulator (Q24 fixed point)
        uint32_t phase_inc;  // phase increment per sample (Q24)
        uint8_t  duty;       // duty cycle 0-255 (128 = 50%) for square wave
        uint8_t  env;        // envelope level 0-255
        uint8_t  env_stage;  // 0=attack, 1=decay, 2=sustain, 3=release
        uint8_t  env_target; // sustain level
        uint8_t  attack_rate;  // envelope rate
        uint8_t  decay_rate;
        uint8_t  release_rate;
        WaveType wave;       // waveform type
        int16_t  filter_z;   // 1-pole low-pass filter state (Q8)
        uint8_t  filter_k;   // filter coefficient 0-255 (0=no filter, 255=max)
        uint32_t noise_lfsr; // LFSR state for noise
    };

    static Voice voices[MIDISYNTH_MAX_VOICES];
    static uint8_t channel_program[16];   // current program per channel
    static uint8_t channel_volume[16];     // CC7 volume per channel
    static uint8_t channel_expression[16]; // CC11 expression per channel
    static uint8_t channel_modulation[16]; // CC1 modulation per channel
    static uint8_t channel_pan[16];       // CC10 pan: 0=left, 64=center, 127=right
    static int16_t channel_pitchbend[16]; // -8192..+8191, 0 = center

    static int allocVoice(uint8_t ch, uint8_t note);
    static int findVoice(uint8_t ch, uint8_t note);

    // Get waveform/envelope/filter parameters for a GM program
    struct PatchParams {
        WaveType wave;
        uint8_t  duty;         // for square wave
        uint8_t  attack_rate;
        uint8_t  decay_rate;
        uint8_t  sustain_level;
        uint8_t  release_rate;
        uint8_t  filter_k;    // low-pass filter strength
    };
    static PatchParams getPatch(uint8_t ch, uint8_t program);
};

#endif // !PICO_RP2040
