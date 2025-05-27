// Copyright (C) 2023 Zubax Robotics

#include "platform.h"
#include "packet.h"

enum Direction {
    BACK,
    STOP,
    FORWARD
  };

void direction_step(enum Direction direction)
{
    switch (direction) {
    case BACK:
        platform_driver_step(false);
        break;
    case FORWARD:
        platform_driver_step(true);
        break;
    case STOP:
    default:
        platform_driver_stop();
    }
}

int main(void)
{
    struct packet_parser parser  = {0};
    enum Direction direction = BACK;

    platform_init();
    platform_driver_setup();
    direction_step(direction);

    while (true)
    {
        platform_kick_watchdog();

        // Step in the current direction
        direction_step(direction);
        // Send the current direction
        packet_send(sizeof(direction), &direction, platform_serial_write);

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
                // current_direction = platform_process_command(parser.payload_size, parser.payload);
            }
        }
    }
    return 0;
}
