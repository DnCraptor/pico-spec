/* Z80_compat.h — standalone replacement for Zeta (Z) library headers.
   Provides all Z_* macros and z* types needed by Z80_redcode.h / Z80_redcode.c
   so the Zeta library does not need to be installed.
   Derived from billgilbert7000/GeneralSoundPico_SpeccyP/src/Z80_compat.h. */

#ifndef Z80_COMPAT_H
#define Z80_COMPAT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

typedef uint8_t  zuint8;
typedef int8_t   zsint8;
typedef uint16_t zuint16;
typedef uint32_t zuint32;
typedef size_t       zusize;
typedef int          zsint;
typedef unsigned int zuint;
typedef char         zchar;
typedef bool         zbool;

typedef union {
    uint16_t uint16_value;
    struct { uint8_t at_0, at_1; } uint8_values;
} ZInt16;

typedef union {
    uint32_t uint32_value;
    uint8_t  uint8_array[4];
    uint16_t uint16_array[2];
} ZInt32;

#ifndef Z_NULL
#define Z_NULL NULL
#endif

#ifndef Z_TRUE
#define Z_TRUE  1
#endif

#ifndef Z_FALSE
#define Z_FALSE 0
#endif

#define Z_USIZE(value)        ((zusize)(value))
#define Z_USIZE_MAXIMUM       SIZE_MAX
#define Z_UINT16(value)       ((zuint16)(value))
#define Z_UINT32(value)       ((zuint32)(value))

#define Z_CAST(type) (type)

#if defined(__GNUC__) || defined(__clang__)
#   define Z_ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#   define Z_ALWAYS_INLINE inline
#endif

#ifdef __cplusplus
#   define Z_EXTERN_C_BEGIN extern "C" {
#   define Z_EXTERN_C_END   }
#else
#   define Z_EXTERN_C_BEGIN
#   define Z_EXTERN_C_END
#endif

#if defined(_WIN32)
#   define Z_API_EXPORT __declspec(dllexport)
#   define Z_API_IMPORT __declspec(dllimport)
#else
#   define Z_API_EXPORT __attribute__((visibility("default")))
#   define Z_API_IMPORT __attribute__((visibility("default")))
#endif

#define Z_UINT8_ROTATE_LEFT(value, rotation) \
    ((zuint8)(((zuint8)(value) << (rotation)) | ((zuint8)(value) >> (8 - (rotation)))))

#define Z_UINT8_ROTATE_RIGHT(value, rotation) \
    ((zuint8)(((zuint8)(value) >> (rotation)) | ((zuint8)(value) << (8 - (rotation)))))

#define Z_UINT16_BIG_ENDIAN(value) \
    ((zuint16)(((zuint16)(value) >> 8) | ((zuint16)(value) << 8)))

#define Z_UINT32_BIG_ENDIAN(value) \
    ((zuint32)( \
        (((zuint32)(value) & 0xFF000000u) >> 24) | \
        (((zuint32)(value) & 0x00FF0000u) >>  8) | \
        (((zuint32)(value) & 0x0000FF00u) <<  8) | \
        (((zuint32)(value) & 0x000000FFu) << 24)))

// Put z80_run and other exported Z80 functions in SRAM on RP2350.
// Both cores share the XIP cache: core0 runs Z80_JLS, core1 runs redcode pump.
// z80_run in flash = XIP-cache contention with core0 Z80_JLS → effective rate
// drop under load. z80_run ≈ 916 B, z80_int ≈ 40 B — fits trivially in SRAM.
// Z80_redcode.h checks #ifndef Z80_API, so defining it here takes precedence
// and applies section(".time_critical.gs_z80") to all API functions.
#if defined(PICO_RP2350)
#   define Z80_API __attribute__((noinline, section(".time_critical.gs_z80")))
#endif

#define Z_MEMBER_OFFSET(type, member) ((zusize)offsetof(type, member))

#define Z_EMPTY
#define Z_UNUSED(variable) (void)(variable);

#ifdef _WIN32
#   define Z_MICROSOFT_STD_CALL __stdcall
#else
#   define Z_MICROSOFT_STD_CALL
#endif

/* Direct-callback override: bypass Z80::fetch_opcode/fetch/read/write/in/out
 * function-pointer indirect calls in the redcode hot loop. We have a single
 * Z80 instance (GS-Z80 on core1) and stable callback identities, so calling
 * them directly lets the compiler inline (where possible) and avoids the
 * indirect-branch penalty that flushes the M33 branch predictor on every
 * Z80 instruction. Defined when Z80_redcode.c is built as part of the GS
 * core1 emulator. The actual implementations live in src/GS/GS.cpp. */
#if defined(GS_Z80_DIRECT_CALLBACKS)
#  ifdef __cplusplus
extern "C" {
#  endif
zuint8 gs_direct_fetch_opcode(zuint16 address);
zuint8 gs_direct_fetch       (zuint16 address);
zuint8 gs_direct_read        (zuint16 address);
void   gs_direct_write       (zuint16 address, zuint8 value);
zuint8 gs_direct_in          (zuint16 port);
void   gs_direct_out         (zuint16 port, zuint8 value);
#  ifdef __cplusplus
}
#  endif
#endif

#endif /* Z80_COMPAT_H */
