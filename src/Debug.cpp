#include <ctime>

#include "Debug.h"
#include "pico/stdlib.h"
#include "FileUtils.h"

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
    // Формируем путь к файлу
    std::string nvs = std::string(MOUNT_POINT_SD) + STORAGE_LOG;

    // Получаем текущее время
    std::time_t t = std::time(nullptr);
    char timeStr[64];
    std::strftime(timeStr, sizeof(timeStr), "[%Y-%m-%d %H:%M:%S] ", std::localtime(&t));

    // Объединяем timestamp и данные
    std::string logEntry = std::string(timeStr) + data + "\n";

    // Открываем файл
    FIL* handle = fopen2(nvs.c_str(), FA_WRITE | FA_OPEN_APPEND);
    if (handle) {
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