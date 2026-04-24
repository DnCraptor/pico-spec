/* GS_ROM.h — declaration of the General Sound firmware ROM (32 KB).

   The definition lives in src/GS/GS_ROM.c (a weak 32 KB stub of HALT 0x76).
   To enable a working GS emulation, drop a real 32 KB GS firmware as a
   non-weak ROM_GS_M definition in a new translation unit (e.g. gs105b.c);
   its strong symbol will override the weak stub at link time.

   Reference firmware: billgilbert7000/GeneralSoundPico_SpeccyP/src/rom/
   (files gs104.h, gs105b.h). Those are not bundled here due to licensing. */

#ifndef GS_ROM_H
#define GS_ROM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t ROM_GS_M[32768];

#ifdef __cplusplus
}
#endif

#endif /* GS_ROM_H */
