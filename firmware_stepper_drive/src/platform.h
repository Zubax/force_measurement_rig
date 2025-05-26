// Copyright (C) 2023 Zubax Robotics

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

void platform_init(void);

void platform_led(const bool on);

void platform_kick_watchdog(void);

