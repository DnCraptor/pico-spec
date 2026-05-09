#ifndef Debug_h
#define Debug_h

#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <cstring>

using namespace std;

#define DEFAULT_BLINK_COUNT 5

// Mirror of DEBUG_LOG_PATH from FileUtils.h. Kept as a literal here so Debug.h
// stays a leaf header (FileUtils.h pulls MemESP.h which depends on Config).
#define STORAGE_LOG "/.config/pico-spec/debug.log"

class Debug
{
public:

    static void led_blink();
    static void led_on();
    static void led_off();

    static void log(const char* fmt, ...);
    static void log2SD(const string& data);
    static void log2SD(const char* fmt, ...);
};

#ifdef __cplusplus
extern "C" {
#endif
void debug_log2sd(const char* fmt, ...);
#ifdef __cplusplus
}
#endif

#endif