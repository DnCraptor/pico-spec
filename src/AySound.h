/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

AY SOUND EMULATION, based on libayemu by:
Sashnov Alexander <sashnov@ngs.ru> and Roman Scherbakov <v_soft@nm.ru> 

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or 
visit https://zxespectrum.speccy.org/contacto

*/

#ifndef AySound_h
#define AySound_h

#include "hardconfig.h"
#include "ESPectrum.h"
#include <inttypes.h>
#include <stddef.h>

// typedef unsigned char ayemu_ay_reg_frame_t[14];

/* Types of stereo.
    The codes of stereo types used for generage sound. */
typedef enum
{
    AYEMU_MONO = 0,
    AYEMU_ABC,
    AYEMU_ACB,
    AYEMU_BAC,
    AYEMU_BCA,
    AYEMU_CAB,
    AYEMU_CBA,
    AYEMU_STEREO_CUSTOM = 255
} ayemu_stereo_t;

/* Sound chip type.
    Constant for identify used chip for emulation */
typedef enum {
    AYEMU_AY,            /*< default AY chip (lion17 for now) */
    AYEMU_YM,            /*< default YM chip (lion17 for now) */
    AYEMU_AY_LION17,     /*< emulate AY with Lion17 table */
    AYEMU_YM_LION17,     /*< emulate YM with Lion17 table */
    AYEMU_AY_KAY,        /*< emulate AY with HACKER KAY table */
    AYEMU_YM_KAY,        /*< emulate YM with HACKER KAY table */
    AYEMU_AY_LOG,        /*< emulate AY with logariphmic table */
    AYEMU_YM_LOG,        /*< emulate YM with logariphmic table */
    AYEMU_AY_CUSTOM,     /*< use AY with custom table. */
    AYEMU_YM_CUSTOM      /*< use YM with custom table. */
} ayemu_chip_t;

// Registers
// regs[0] = finePitchChannelA;
// regs[1] = coarsePitchChannelA;
// regs[2] = finePitchChannelB;
// regs[3] = coarsePitchChannelB;
// regs[4] = finePitchChannelC;
// regs[5] = coarsePitchChannelC;
// regs[6] = noisePitch;
// regs[7] = mixer;
// regs[8] = volumeChannelA;
// regs[9] = volumeChannelB;
// regs[10] = volumeChannelC;
// regs[11] = envelopeFineDuration;
// regs[12] = envelopeCoarseDuration;
// regs[13] = envelopeShape;
// regs[14] = ioPortA;
// regs[15] = ioPortB;

/* parsed by #ayemu_set_regs() AY registers data \internal */
typedef struct
{
    int tone_a;       /*< R0, R1 */
    int tone_b;       /*< R2, R3 */    
    int tone_c;       /*< R4, R5 */
    int noise;        /*< R6 */
    int R7_tone_a;    /*< R7 bit 0 */
    int R7_tone_b;    /*< R7 bit 1 */
    int R7_tone_c;    /*< R7 bit 2 */
    int R7_noise_a;   /*< R7 bit 3 */
    int R7_noise_b;   /*< R7 bit 4 */
    int R7_noise_c;   /*< R7 bit 5 */
    int vol_a;        /*< R8 bits 3-0 */
    int vol_b;        /*< R9 bits 3-0 */
    int vol_c;        /*< R10 bits 3-0 */
    int env_a;        /*< R8 bit 4 */
    int env_b;        /*< R9 bit 4 */
    int env_c;        /*< R10 bit 4 */
    int env_freq;     /*< R11, R12 */
    int env_style;    /*< R13 */
    int IOPortA;      /*< R14 */
    int IOPortB;      /*< R15 */
}
ayemu_regdata_t;

/* Output sound format \internal */
typedef struct
{
  int freq;           /*< sound freq */
  int channels;       /*< channels (1-mono, 2-stereo) */
  int bpc;            /*< bits (8 or 16) */
}
ayemu_sndfmt_t;

class AySound
{
public:
    static int selected_chip;
    void updToneA();
    void updToneB();
    void updToneC();
    void updNoisePitch();
    void updMixer();
    void updVolA();
    void updVolB();
    void updVolC();
    void updEnvFreq();
    void updEnvType();
    void updIOPortA();
    void updIOPortB();
    
    void reset();
    uint8_t getRegisterData();
    void selectRegister(uint8_t data);
    void setRegisterData(uint8_t data);

    void init();
    int set_chip_type(ayemu_chip_t chip, int *custom_table);
    void set_chip_freq(int chipfreq);
    int set_stereo(ayemu_stereo_t stereo, int *custom_eq);
    int set_sound_format(int freq, int chans, int bits);
    void prepare_generation();
    void gen_sound(int bufsize, int bufpos);

    uint8_t SamplebufAY_L[ESP_AUDIO_SAMPLES_PENTAGON];
    uint8_t SamplebufAY_R[ESP_AUDIO_SAMPLES_PENTAGON];

private:

    /* emulator settings */
    int table[32];                   /*< table of volumes for chip */
    ayemu_chip_t type;               /*< general chip type (\b AYEMU_AY or \b AYEMU_YM) */
    int ChipFreq;                    /*< chip emulator frequency */
    // static int eq[6];                       /*< volumes for channels.
                                            // Array contains 6 elements: 
                                            // A left, A right, B left, B right, C left and C right;
                                            // range -100...100 */
    ayemu_regdata_t ayregs;          /*< parsed registers data */
    ayemu_sndfmt_t sndfmt;           /*< output sound format */

    // flags
    int default_chip_flag;           /*< =1 after init, resets in #ayemu_set_chip_type() */
    int default_stereo_flag;         /*< =1 after init, resets in #ayemu_set_stereo() */
    int default_sound_format_flag;   /*< =1 after init, resets in #ayemu_set_sound_format() */
    int dirty;                       /*< dirty flag. Sets if any emulator properties changed */

    int bit_a;                       /*< state of channel A generator */
    int bit_b;                       /*< state of channel B generator */
    int bit_c;                       /*< state of channel C generator */
    int bit_n;                       /*< current generator state */
    int period_n;                    // Noise period 
    int cnt_a;                       /*< back counter of A */
    int cnt_b;                       /*< back counter of B */
    int cnt_c;                       /*< back counter of C */
    int cnt_n;                       /*< back counter of noise generator */
    int cnt_e;                       /*< back counter of envelop generator */
    int ChipTacts_per_outcount;      /*< chip's counts per one sound signal count */
    int Amp_Global;                  /*< scale factor for amplitude */
    int EnvNum;                      /*< number of current envilopment (0...15) */
    int env_pos;                     /*< current position in envelop (0...127) */
    int Cur_Seed;                    /*< random numbers counter */

    uint8_t regs[16];
    uint8_t selectedRegister;
};

extern AySound chip0;
extern AySound chip1;
extern AySound* chips[2];

#endif // AySound_h
