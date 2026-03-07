#pragma once

#if !PICO_RP2040

#include <inttypes.h>

// Simple polyphonic square-wave MIDI synthesizer
// 16 voices, General MIDI program changes select waveform/duty
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

    // Voice allocation
    struct Voice {
        uint8_t  channel;    // MIDI channel
        uint8_t  note;       // MIDI note number
        uint8_t  velocity;   // 0 = free
        uint32_t phase;      // phase accumulator (Q16 fixed point)
        uint32_t phase_inc;  // phase increment per sample (Q16)
        uint8_t  duty;       // duty cycle 0-255 (128 = 50%)
        uint8_t  env;        // simple envelope level 0-255
        uint8_t  env_stage;  // 0=attack, 1=sustain, 2=release
    };

    static Voice voices[MIDISYNTH_MAX_VOICES];
    static uint8_t channel_program[16];  // current program per channel
    static uint8_t channel_volume[16];   // CC7 volume per channel

    static int allocVoice(uint8_t ch, uint8_t note);
    static int findVoice(uint8_t ch, uint8_t note);
    static uint32_t noteToPhaseInc(uint8_t note);
};

#endif // !PICO_RP2040
