#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <hardware/gpio.h>

// Probe PCM5122 on I2C bus without full initialization.
// Returns true if chip responds at address 0x4C.
// If not found, deinits I2C and releases pins.
bool pcm5122_detect(uint sda_pin, uint scl_pin);

// Full PCM5122 initialization via I2C:
// slave mode (BCK as clock source), 16-bit I2S, unmute.
// Returns true if chip responds and init succeeds.
bool pcm5122_init(uint sda_pin, uint scl_pin);

#ifdef __cplusplus
}
#endif
