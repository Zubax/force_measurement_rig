// Copyright (C) 2023 Zubax Robotics

#include "platform.h"

#include <util/delay.h>

int main(void)
{
    platform_init();

    while (true)
    {
        platform_kick_watchdog();
        platform_led(false);
        _delay_ms(1000);
        platform_led(true);
        _delay_ms(1000);
    }
    return 0;
}
