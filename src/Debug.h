#ifndef Debug_h
#define Debug_h

#include <stdio.h>
#include <inttypes.h>
#include <string>
#include <cstring>

using namespace std;

#define DEFAULT_BLINK_COUNT 5

#define STORAGE_LOG "/debug.log"

class Debug
{
public:

    static void led_blink();
    static void led_on();
    static void led_off();

    static void log(string data);
    static void log(const char* fmt, ...);
};

#endif