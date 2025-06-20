// Copyright (C) 2023 Zubax Robotics

#include "platform.h"
#include "packet.h"

#define CALIBRATION_DATA_SIZE 40

struct reading
{
    uint64_t seq_num;
    uint64_t reserved_a;
    uint64_t reserved_b;
    int32_t  load_cell_raw[4];
    uint8_t  calibration_data[CALIBRATION_DATA_SIZE];
};
_Static_assert(sizeof(struct reading) == 80, "Invalid layout");  // NOLINT(readability-magic-numbers)

int main(void)
{
    platform_init();
    struct packet_parser parser  = {0};
    struct reading       reading = {0};
    platform_calibration_read(CALIBRATION_DATA_SIZE, reading.calibration_data);
    while (true)
    {
        // Read the next sample. The LED is off while waiting for the data.
        platform_led(false);
        platform_load_cell_read(reading.load_cell_raw);
        platform_led(true);
        // Send the reading.
        packet_send(sizeof(reading), &reading, platform_serial_write);
        // Prepare for the next iteration.
        platform_kick_watchdog();
        reading.seq_num++;

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
                platform_calibration_write(parser.payload_size, parser.payload);
                platform_calibration_read(CALIBRATION_DATA_SIZE, reading.calibration_data);
            }
        }
    }
    return 0;
}
