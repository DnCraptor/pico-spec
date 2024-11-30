#ifndef ROM_48K_ES_H
#define ROM_48K_ES_H

#if !NO_SPAIN_ROM_48k
//ROM 0 48K SPANISH
#include <hardware/flash.h>

extern "C" unsigned char __in_flash() __aligned(4096) gb_rom_0_48k_es[];
#endif

#endif
