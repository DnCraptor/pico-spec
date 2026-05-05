/* GS_ROM.h — declaration of the General Sound firmware ROM (32 KB).

   The definition in src/GS/GS_ROM.c is gs105b.rom (RomanRom2, version 1.05b,
   https://zxgit.org/RomanRom2/GeneralSound). It is declared weak so a
   project-local translation unit can override it at link time. */

#ifndef GS_ROM_H
#define GS_ROM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t ROM_GS_M[32768];      /* v1.05b — RomanRom2, 2015 (default) */
extern const uint8_t ROM_GS_V104B[32768]; /* v1.04 Beta — psb & Evgeny Muchkin, 2007 */

#ifdef __cplusplus
}
#endif

#endif /* GS_ROM_H */
