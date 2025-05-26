// Copyright (C) 2023 Zubax Robotics

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void platform_init(void);

void platform_led(const bool on);

void platform_kick_watchdog(void);

/// The call is non-blocking unless the buffer is full. Transmission is interrupt-driven.
void platform_serial_write(const size_t size, const void* const data);
/// The call is non-blocking. Returns -1 if the buffer is empty, otherwise the byte value in the range [0, 255].
int16_t platform_serial_read(void);

#define PLATFORM_LOAD_CELL_COUNT 2

/// Returns the raw signed ADC counts per load cell. The gain is unspecified (subject to calibration).
/// The receiver is responsible for mapping the value to newtons.
void platform_load_cell_read(int32_t out[PLATFORM_LOAD_CELL_COUNT]);

/// Opaque calibration data stored in the non-volatile memory. Its format is application-defined.
void platform_calibration_read(const size_t size, uint8_t* const out);
void platform_calibration_write(const size_t size, const uint8_t* const out);
