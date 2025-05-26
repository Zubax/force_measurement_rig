// Copyright (c) 2023  Zubax Robotics  <info@zubax.com>

#include "packet.h"
#include <string.h>
#include <assert.h>

static size_t  g_offset;
static uint8_t g_buffer[1024];

static void cb_write(const size_t size, const void* const data)
{
    memcpy(g_buffer + g_offset, data, size);
    g_offset += size;
}

static void test_crc(void)
{
    assert(crc16_ccitt_false_add(CRC16_CCITT_FALSE_INITIAL_VALUE, 0, NULL) == CRC16_CCITT_FALSE_INITIAL_VALUE);
    assert(crc16_ccitt_false_add(CRC16_CCITT_FALSE_INITIAL_VALUE, 9, "123456789") == 0x29B1);
    assert(crc16_ccitt_false_add(CRC16_CCITT_FALSE_INITIAL_VALUE, 11, "123456789\x29\xB1") ==
           CRC16_CCITT_FALSE_RESIDUE);
}

static void test_packet(void)
{
    struct packet_parser parser = {0};

    // Send an empty packet.
    packet_send(0, NULL, cb_write);
    assert(g_offset == 10);
    assert(0 == memcmp(g_buffer, "\xB4\x4C\xEC\xF2\x00\x00\x00\x00\xff\xff", g_offset));

    // Parse the packet.
    for (size_t i = 0; i < g_offset; i++)
    {
        assert(packet_parse(&parser, g_buffer[i]) == (i == g_offset - 1));
    }
    assert(parser.payload_size == 0);
    // Check internal states as well.
    assert(parser.crc == CRC16_CCITT_FALSE_RESIDUE);
    assert(parser.payload_offset == 0);
    assert(parser.stage == 0);

    // Send a non-empty packet.
    g_offset = 0;
    packet_send(9, "123456789", cb_write);
    assert(g_offset == 19);
    assert(0 ==
           memcmp(g_buffer, "\xB4\x4C\xEC\xF2\x09\x00\x00\x00\x31\x32\x33\x34\x35\x36\x37\x38\x39\x29\xb1", g_offset));

    // Parse the packet.
    for (size_t i = 0; i < g_offset; i++)
    {
        assert(packet_parse(&parser, g_buffer[i]) == (i == g_offset - 1));
    }
    assert(parser.payload_size == 9);
    assert(0 == memcmp(parser.payload, "123456789", parser.payload_size));
    // Check internal states as well.
    assert(parser.crc == CRC16_CCITT_FALSE_RESIDUE);
    assert(parser.payload_offset == 9);
    assert(parser.stage == 0);
}

int main()
{
    test_crc();
    test_packet();
    return 0;
}
