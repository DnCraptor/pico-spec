#ifndef ROM_128K_CUSTOM_H
#define ROM_128K_CUSTOM_H

#include <hardware/flash.h>

extern "C" unsigned char __in_flash() __aligned(4096) gb_rom_0_128k_custom[];

#endif
