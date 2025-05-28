// Copyright (c) 2023  Zubax Robotics  <info@zubax.com>

#pragma once

#include "crc.h"
#include <stdint.h>
#include <stdbool.h>

/// The packet magic is a truly random number that does not mean anything.
#define PACKET_MAGIC 0xF2EC4CB4ULL

struct packet_header
{
    uint32_t magic;
    uint8_t  payload_size;
    uint8_t  reserved[3];
};
_Static_assert(sizeof(struct packet_header) == 8, "Invalid layout");

struct packet_parser
{
    uint8_t  stage;
    size_t   payload_size;
    size_t   payload_offset;
    uint8_t  payload[255];  // NOLINT(readability-magic-numbers)
    uint16_t crc;
};

static inline void packet_send(const uint8_t     size,
                               const void* const data,
                               void (*const writer)(const size_t, const void* const))
{
    const struct packet_header header = {.magic = PACKET_MAGIC, .payload_size = size};
    writer(sizeof(header), &header);
    writer(size, data);
    const uint16_t crc          = crc16_ccitt_false_add(CRC16_CCITT_FALSE_INITIAL_VALUE, size, data);
    const uint8_t  crc_bytes[2] = {(uint8_t) (crc >> 8U), (uint8_t) crc};
    writer(sizeof(crc_bytes), crc_bytes);
}

/// Updates the packet parser state machine with the newly received byte.
/// Each packet contains the packet_header in the beginning, followed by the payload, followed by the CRC.
/// The return value is true if the packet is successfully parsed, false otherwise.
/// In case of a successful parse, the payload is stored in the eponymous field, same for its size.
static inline bool packet_parse(struct packet_parser* const state, const uint8_t byte)
{
    static const uint8_t byte_mask = 0xFFU;
    bool                 result    = false;
    switch (state->stage)
    {
    case 0:
    case 1:
    case 2:
    case 3:
    {
        if (byte == ((PACKET_MAGIC >> (8U * state->stage)) & byte_mask))
        {
            state->stage++;
        }
        else
        {
            state->stage = 0;
        }
        break;
    }
    case 4:
    {
        state->payload_size   = byte;
        state->payload_offset = 0;
        state->crc            = CRC16_CCITT_FALSE_INITIAL_VALUE;
        state->stage++;
        if (state->payload_size > sizeof(state->payload))
        {
            state->stage = 0;
        }
        break;
    }
    case 5:
    case 6:
    case 7:
    {
        state->stage++;
        break;
    }
    case 8:
    {
        if (state->payload_offset < state->payload_size)
        {
            state->payload[state->payload_offset] = byte;
            state->crc                            = crc16_ccitt_false_add_byte(state->crc, byte);
            state->payload_offset++;
        }
        else
        {
            state->crc = crc16_ccitt_false_add_byte(state->crc, byte);
            state->stage++;
        }
        break;
    }
    default:
    {
        state->crc   = crc16_ccitt_false_add_byte(state->crc, byte);
        result       = CRC16_CCITT_FALSE_RESIDUE == state->crc;
        state->stage = 0;
        break;
    }
    }
    return result;
}
