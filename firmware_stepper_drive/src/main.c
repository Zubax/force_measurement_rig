// Copyright (C) 2023 Zubax Robotics

#include "platform.h"
#include "packet.h"

#include <string.h>

void execute_step(const int32_t step)
{
    switch (step) {
    case -1:
        platform_driver_step(false);
        break;
    case 1:
        platform_driver_step(true);
        break;
    case 0:
    default:
        platform_driver_stop();
    }
}

int main(void)
{
    struct packet_parser parser  = {0};
    int32_t received_step = 0;

    platform_init();
    platform_driver_setup();
    execute_step(received_step);

    while (true)
    {
        platform_kick_watchdog();

        // Step in the current direction
        execute_step(received_step);
        // Send the current direction
        packet_send(sizeof(received_step), &received_step, platform_serial_write);

        // Process the pending incoming data. There may be many bytes accumulated in the buffer.
        while (true)
        {
            const int16_t rx = platform_serial_read();
            if (rx < 0)
            {
                break;
            }
            if (packet_parse(&parser, (uint8_t) rx))
            {
                if (parser.payload_size == sizeof(int32_t))
                {
                    memcpy(&received_step, parser.payload, sizeof(int32_t));
                }
            }
        }
    }
    return 0;
}
