#include "Midi.h"

#if !PICO_RP2040

#include "Config.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>

#ifndef MIDI_TX_PIN
#define MIDI_TX_PIN 4
#endif

// Auto-select UART instance based on TX pin (funcsel 2)
// UART0 TX: GPIO 0, 8, 16, 24, 32, 40
// UART1 TX: GPIO 4, 12, 20, 28, 36, 44
// Pattern: (pin / 4) % 2 → 0=UART0, 1=UART1
// Note: only even GPIO can be TX; odd GPIO = RX
#if ((MIDI_TX_PIN / 4) % 2 == 0)
#define MIDI_UART uart0
#else
#define MIDI_UART uart1
#endif

#define MIDI_BAUD 31250

uint8_t Midi::enabled = 0;
bool Midi::hw_initialized = false;

void Midi::init() {
    if (hw_initialized) return;

    uart_init(MIDI_UART, MIDI_BAUD);
    gpio_set_function(MIDI_TX_PIN, UART_FUNCSEL_NUM(MIDI_UART, MIDI_TX_PIN));
    uart_set_format(MIDI_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(MIDI_UART, true);

    hw_initialized = true;
}

void Midi::deinit() {
    if (!hw_initialized) return;
    uart_deinit(MIDI_UART);
    gpio_deinit(MIDI_TX_PIN);
    hw_initialized = false;
}

// Non-blocking send: write to UART TX FIFO if space available, else drop.
// The ShamaZX driver polls busy() before calling, so drops shouldn't happen.
void __not_in_flash("midi") Midi::send(uint8_t b) {
    if (uart_is_writable(MIDI_UART))
        uart_get_hw(MIDI_UART)->dr = b;
}

// Check if UART TX FIFO is full (maps to SAM2695 "receiver full" bit 6)
bool __not_in_flash("midi") Midi::busy() {
    return !uart_is_writable(MIDI_UART);
}

#endif // !PICO_RP2040
