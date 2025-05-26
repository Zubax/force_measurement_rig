// Copyright (C) 2023 Zubax Robotics

#include "platform.h"
#include <avr/io.h>

struct pin_spec
{
    volatile uint8_t* const reg;  // The PORT or PIN register for the pin.
    const uint8_t           bit;  // The index of the pin in the register.
};

static inline void pin_write(const struct pin_spec pin, const bool value)
{
    const uint8_t sreg = SREG;
    __asm__("cli");
    if (value)
    {
        *pin.reg |= (1U << pin.bit);
    }
    else
    {
        *pin.reg &= ~(1U << pin.bit);
    }
    SREG = sreg;
}

static inline bool pin_read(const struct pin_spec pin)
{
    return (*pin.reg & (1U << pin.bit)) != 0;
}

void platform_init(void)
{
    __asm__("cli");
    __asm__("wdr");
    WDTCSR |= (1U << WDE) | (1U << WDCE);
    WDTCSR = (1U << WDE) | (1U << WDP3) | (0U << WDP2) | (0U << WDP1) | (1U << WDP0);  // Watchdog, 8 sec

    CLKPR = 0x80;  // Disable prescaler
    CLKPR = 0x00;

    // GPIO
    DDRB  = 1U << 5U;                 // LED on PB5
    PORTB = 0xFFU;                    // All pull-ups, LED on
    DDRD  = (1U << 1U) | (1U << 2U);  // TXD, Load cell SCK
    PORTD = 0xFFU;                    // All pull-ups, SCK high (idle state).

    // Serial port at 38400 baud with 0.2% error. This is the fastest available standard baud rate.
    // For calculation, see http://wormfood.net/avrbaudcalc.php.
    UCSR0A = 0;
    UCSR0B = (1U << 7U) | (1U << 6U) | (1U << 4U) | (1U << 3U);
    UCSR0C = (1U << 2U) | (1U << 1U);
    UBRR0  = 25;  // NOLINT(readability-magic-numbers)

    __asm__("sei");
}

void platform_kick_watchdog(void)
{
    __asm__("wdr");
}

void platform_led(const bool on)
{
    pin_write((struct pin_spec){&PORTB, 5}, on);
}