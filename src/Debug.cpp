#include "Debug.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "FileUtils.h"

static uint32_t log_counter = 0;

void Debug::led_blink()
{
#if DEBUG
    for (int i = 0; i < DEFAULT_BLINK_COUNT; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }
#endif
}

void Debug::led_on()
{
#if DEBUG
    gpio_put(PICO_DEFAULT_LED_PIN, true);
#endif
}

void Debug::led_off()
{
#if DEBUG
    gpio_put(PICO_DEFAULT_LED_PIN, false);
#endif
}

void Debug::log(string data)
{
#if DEBUG
    if (!FileUtils::fsMount) return;
    std::string nvs = std::string(MOUNT_POINT_SD) + STORAGE_LOG;

    // Заголовок сессии при первой записи
    bool first = (log_counter == 0);

    char prefix[32];
    snprintf(prefix, sizeof(prefix), "%08u %7u ", (unsigned)log_counter++,
             (unsigned)(to_ms_since_boot(get_absolute_time())));

    std::string logEntry;
    if (first) {
        const char* reason = watchdog_caused_reboot() ? "WATCHDOG" : "POWER-ON";
        char hdr[64];
        snprintf(hdr, sizeof(hdr), "--- BOOT (%s) ---\n", reason);
        logEntry = std::string(hdr);
    }
    logEntry += std::string(prefix) + data + "\n";

    // Лимит 10KB — перезаписываем с начала при переполнении
    FIL* handle = fopen2(nvs.c_str(), FA_WRITE | FA_OPEN_APPEND);
    if (handle) {
        if (f_size(handle) >= 10240) {
            fclose2(handle);
            handle = fopen2(nvs.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
            if (!handle) return;
        }
        UINT btw;
        f_write(handle, logEntry.c_str(), logEntry.size(), &btw);
        fclose2(handle);
    }
#endif
}

void Debug::log(const char* fmt, ...)
{
    if (!FileUtils::fsMount) return;

    char buf[256]; // можно увеличить при необходимости
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    log(std::string(buf)); // вызов основной версии
}