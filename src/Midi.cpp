#include "Midi.h"

#if !PICO_RP2040

#include "Config.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>

#ifndef MIDI_TX_PIN
#define MIDI_TX_PIN 4
#endif

#define MIDI_UART uart1
#define MIDI_BAUD 31250

bool Midi::enabled = false;
bool Midi::hw_initialized = false;
uint8_t Midi::buf[BUF_SIZE] = {0};
volatile uint8_t Midi::head = 0;
volatile uint8_t Midi::tail = 0;

void Midi::init() {
    if (hw_initialized) return;

    uart_init(MIDI_UART, MIDI_BAUD);
    gpio_set_function(MIDI_TX_PIN, UART_FUNCSEL_NUM(MIDI_UART, MIDI_TX_PIN));
    uart_set_format(MIDI_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(MIDI_UART, true);

    head = tail = 0;
    hw_initialized = true;
}

void Midi::deinit() {
    if (!hw_initialized) return;
    uart_deinit(MIDI_UART);
    gpio_deinit(MIDI_TX_PIN);
    hw_initialized = false;
    head = tail = 0;
}

void __not_in_flash("midi") Midi::send(uint8_t b) {
    uint8_t next = head + 1;
    if (next != tail) {
        buf[head] = b;
        head = next;
    }
}

void Midi::flush() {
    while (tail != head) {
        if (!uart_is_writable(MIDI_UART))
            break;
        uart_putc_raw(MIDI_UART, buf[tail]);
        tail++;
    }
}

#endif // !PICO_RP2040
