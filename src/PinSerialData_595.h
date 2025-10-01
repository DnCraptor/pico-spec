#pragma once

#include "hardware/pio.h"


#define LATCH_595_PIN (26)
#define CLK_595_PIN (27)
#define DATA_595_PIN (28)
#define CLK_AY_PIN1 (21)
#define CLK_AY_PIN2 (29)

#define TSPIN_MODE_OFF  (0)
#define TSPIN_MODE_GP21 (1)
#define TSPIN_MODE_GP29 (2)
#define TSPIN_MODE_BOTH (3)

void Init_PWM_175(uint8_t tspin_mode);
void Deinit_PWM_175();

void send_to_595(uint16_t data);

/// HW TurboSound support
extern uint16_t control_bits;
#define CS_SAA1099	(1<<15)
#define AY_Enable (1<<14)
#define SAVE (1<<13)
#define Beeper (1<<12)
#define CS_AY1 (1<<11)
#define CS_AY0	(1<<10)
#define BDIR (1<<9)
#define BC1 (1<<8)
#define LOW(x) (control_bits &= ~(x))
#define HIGH(x) (control_bits |= (x))
void AY_to595Beep(bool Beep);
