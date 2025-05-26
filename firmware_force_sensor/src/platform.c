// Copyright (C) 2023 Zubax Robotics

#include "platform.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>

#if F_CPU != 16000000
#    error "Core clock must be 16MHz, adjust the timings if this is not the case"
#endif

// NOLINTBEGIN(hicpp-no-assembler,cppcoreguidelines-avoid-non-const-global-variables,readability-magic-numbers)

static uint8_t g_buf_tx[200];
static uint8_t g_buf_rx[500];

struct fifo
{
    uint8_t* const pbuf;
    const size_t   bufsize;
    size_t         in;
    size_t         out;
    size_t         len;
};

static struct fifo g_fifo_tx = {g_buf_tx, sizeof(g_buf_tx), 0, 0, 0};
static struct fifo g_fifo_rx = {g_buf_rx, sizeof(g_buf_rx), 0, 0, 0};

static void fifo_push(struct fifo* const pfifo, const uint8_t data)
{
    const uint8_t sreg = SREG;
    __asm__("cli");
    pfifo->pbuf[pfifo->in++] = data;
    if (pfifo->in >= pfifo->bufsize)
    {
        pfifo->in = 0;
    }
    if (pfifo->len >= pfifo->bufsize)
    {
        pfifo->out++;
        if (pfifo->out >= pfifo->bufsize)
        {
            pfifo->out = 0;
        }
    }
    else
    {
        pfifo->len++;
    }
    SREG = sreg;
}

static int16_t fifo_pop(struct fifo* const pfifo)
{
    int16_t       retval = -1;
    const uint8_t sreg   = SREG;
    __asm__("cli");
    if (pfifo->len <= 0)
    {
        goto _exit;
    }
    pfifo->len--;
    retval = pfifo->pbuf[pfifo->out++];
    if (pfifo->out >= pfifo->bufsize)
    {
        pfifo->out = 0;
    }
_exit:
    SREG = sreg;
    return retval;
}

static size_t fifo_len(const struct fifo* const pfifo)
{
    const uint8_t sreg = SREG;
    __asm__("cli");
    const size_t retval = pfifo->len;
    SREG                = sreg;
    return retval;
}

static bool is_tx_idle(void)
{
    return (fifo_len(&g_fifo_tx) == 0) && (UCSR0A & (1U << 5U));
}

ISR(USART_TX_vect)
{
    const int16_t val = fifo_pop(&g_fifo_tx);
    if (val >= 0)
    {
        UDR0 = val;
    }
}

ISR(USART_RX_vect)
{
    const uint8_t val = UDR0;
    if ((UCSR0A & ((1U << 4U /*frame error*/) | (1U << 2U /*parity error*/))) == 0)
    {
        fifo_push(&g_fifo_rx, val);
    }
}

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

/// Read an arbitrary number of HX711 sensors in parallel using a shared SCK line and dedicated data lines.
/// The shared clock allows perfectly simultaneous reading of all sensors, although whether the sampling itself is
/// simultaneous depends on the sensors' internal design.
/// If at least one sensor is not ready, the function will block until all sensors are ready.
/// The results are left-shifted to 32 bits.
static inline void read_hx711_gain128(const struct pin_spec        pin_sck,
                                      const size_t                 data_pin_count,
                                      const struct pin_spec* const pins_data,
                                      int32_t* const               results)
{
    static const uint8_t num_bits       = 24;
    static const double  sck_low_min_us = 0.2;  // See datasheet.
    pin_write(pin_sck, false);                  // Set SCK low to leave the low-power mode if it was active.
    // Wait for all sensors to become ready.
    {
        bool busy = false;
        do
        {
            busy = false;
            for (size_t i = 0; i < data_pin_count; i++)
            {
                busy = busy || pin_read(pins_data[i]);
            }
        } while (busy);
    }
    // Clear the results.  NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    memset(results, 0, sizeof(int32_t) * data_pin_count);
    // Communicate with HX711
    for (uint8_t i = 0; i < num_bits; i++)
    {
        pin_write(pin_sck, true);
        _delay_us(sck_low_min_us);  // The loop adds quite a bit of overhead.
        for (size_t j = 0; j < data_pin_count; j++)
        {
            results[j] *= 2;
            results[j] += pin_read(pins_data[j]);
        }
        pin_write(pin_sck, false);
        _delay_us(sck_low_min_us);
    }
    // 25th pulse for gain 128
    pin_write(pin_sck, true);
    _delay_us(1);
    pin_write(pin_sck, false);
    _delay_us(1);
    // Sign-extend the values by upscaling to 32 bits.
    for (size_t i = 0; i < data_pin_count; i++)
    {
        results[i] <<= 8U;  // NOLINT(hicpp-signed-bitwise)
    }
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

void platform_led(const bool on)
{
    pin_write((struct pin_spec){&PORTB, 5}, on);
}

void platform_kick_watchdog(void)
{
    __asm__("wdr");
}

void platform_serial_write(const size_t size, const void* const data)
{
    const uint8_t* bytes     = data;
    size_t         remaining = size;
    const uint8_t  sreg      = SREG;
    __asm__("cli");
    if (is_tx_idle())
    {
        UDR0 = *bytes++;
        remaining--;
    }
    SREG = sreg;  // End of the critical section here
    while (remaining-- > 0)
    {
        while (fifo_len(&g_fifo_tx) >= g_fifo_tx.bufsize)
        {
            __asm__ volatile("nop");
        }
        fifo_push(&g_fifo_tx, *bytes++);
    }
}

int16_t platform_serial_read(void)
{
    return fifo_pop(&g_fifo_rx);  // Critical section is not needed here.
}

void platform_load_cell_read(int32_t out[PLATFORM_LOAD_CELL_COUNT])
{
    static const struct pin_spec data_pins[PLATFORM_LOAD_CELL_COUNT] = {
        {&PIND, 3},
        {&PIND, 4},
    };
    read_hx711_gain128((struct pin_spec){&PORTD, 2}, PLATFORM_LOAD_CELL_COUNT, data_pins, out);
}

void platform_calibration_read(const size_t size, uint8_t* const out)
{
    eeprom_read_block(out, (const void*) 0, size);
}
void platform_calibration_write(const size_t size, const uint8_t* const out)
{
    eeprom_write_block(out, (void*) 0, size);
}

// NOLINTEND(hicpp-no-assembler,cppcoreguidelines-avoid-non-const-global-variables,readability-magic-numbers)
