#pragma once
#include <stdint.h>

// ZX Spectrum 48K keyboard bitmap — 254x156 px, 8-bit palette index
// (0..0xF = ZX 16-colour index, everything else = transparent).
// Definition lives in kbd_img.cpp (placed in flash via __in_flash()).
// Source: https://github.com/billgilbert7000/SpeccyP (GPL).
extern const uint8_t kbd_img[39624];
