#ifndef ROM_48K_CUSTOM_H
#define ROM_48K_CUSTOM_H

#include <hardware/flash.h>

extern "C" unsigned char __in_flash() __aligned(4096) gb_rom_0_48k_custom[];

#endif
