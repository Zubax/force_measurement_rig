from __future__ import annotations

import asyncio
import serial
import struct
import logging
import dataclasses
import concurrent.futures

_logger = logging.getLogger(__name__)


@dataclasses.dataclass(frozen=True)
class Packet:
    """
    A packet received from or sent to the digitizer.
    """

    payload: memoryview

    MAX_PAYLOAD_SIZE = 255

    _MAGIC_INT = 0xF2EC4CB4
    _MAGIC_BYTES = _MAGIC_INT.to_bytes(4, "little")
    _HEADER_FORMAT = struct.Struct(r"< L B 3x")
    _CRC_SIZE = 2

    @staticmethod
    def parse(data: memoryview | bytes | bytearray) -> tuple[memoryview, Packet | None]:
        r"""
        Parses the packet received via the serial port from the strain gauge digitizer in real time.
        Returns the remaining data and the parsed packet, or None if the data is incomplete.
        If the return value contains a valid packet, this function should be invoked again with the remaining data
        because there may be more packets in it.

        >>> rem, pkt = Packet.parse(b"")
        >>> assert rem == b''
        >>> assert pkt is None
        >>>
        >>> valid_packet = b"\xB4\x4C\xEC\xF2\x00\x00\x00\x00\xff\xff"
        >>> rem, pkt = Packet.parse(valid_packet)
        >>> assert rem == b''
        >>> assert pkt is not None
        >>> assert pkt.payload == b''
        >>>
        >>> rem, pkt = Packet.parse(valid_packet + b"123")
        >>> assert rem == b'123'
        >>> assert pkt is not None
        >>> assert pkt.payload == b''
        >>>
        >>> rem, pkt = Packet.parse(valid_packet[:3] + valid_packet + b"123")
        >>> assert rem == b'123'
        >>> assert pkt is not None
        >>> assert pkt.payload == b''
        >>>
        >>> valid_packet = b"\xB4\x4C\xEC\xF2\x09\x00\x00\x00\x31\x32\x33\x34\x35\x36\x37\x38\x39\x29\xb1"
        >>> rem, pkt = Packet.parse(valid_packet)
        >>> assert rem == b''
        >>> assert pkt is not None
        >>> assert pkt.payload == b'123456789'
        >>>
        >>> rem, pkt = Packet.parse(valid_packet[:10] + valid_packet + b"123")
        >>> assert rem == b'123'
        >>> assert pkt is not None
        >>> assert pkt.payload == b'123456789'
        """
        data = memoryview(data)
        magic_size = len(Packet._MAGIC_BYTES)
        while len(data) > Packet._HEADER_FORMAT.size:
            while len(data) >= magic_size and data[:magic_size] != Packet._MAGIC_BYTES:
                data = data[1:]
            if len(data) < Packet._HEADER_FORMAT.size:  # Not enough data for the header, no point in continuing.
                return data, None
            magic, payload_size = Packet._HEADER_FORMAT.unpack_from(data)
            assert magic == Packet._MAGIC_INT
            if len(data) < Packet._HEADER_FORMAT.size + payload_size + Packet._CRC_SIZE:
                return data, None  # Need more data, will continue later.
            data = data[Packet._HEADER_FORMAT.size :]  # Skip the header. We won't need it anymore.
            if not CRC16CCITTFalse.new(data[: payload_size + Packet._CRC_SIZE]).check_residue():
                continue
            payload, data = data[:payload_size], data[payload_size + Packet._CRC_SIZE :]
            pkt = Packet(payload)
            _logger.debug("Parsed %s, remainder %d bytes", pkt, len(data))
            return data, pkt
        return data, None

    def compile(self) -> bytes:
        r"""
        Compiles the packet into a byte sequence ready to be sent to the digitizer.

        >>> Packet(memoryview(b"")).compile().hex()
        'b44cecf200000000ffff'
        >>> Packet(memoryview(b"123456789")).compile().hex()
        'b44cecf20900000031323334353637383929b1'
        """
        if len(self.payload) > self.MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(self.payload)} > {self.MAX_PAYLOAD_SIZE} bytes")
        return b"".join(
            (
                Packet._HEADER_FORMAT.pack(Packet._MAGIC_INT, len(self.payload)),
                self.payload,
                CRC16CCITTFalse.new(self.payload).value_as_bytes,
            )
        )


class CRC16CCITTFalse:
    """
    - Name:           CRC-16/CCITT-FALSE
    - Initial value:  0xFFFF
    - Polynomial:     0x1021
    - Reverse:        No
    - Output XOR:     0
    - Residue:        0
    - Check:          0x29B1

    >>> assert CRC16CCITTFalse().value == 0xFFFF
    >>> c = CRC16CCITTFalse()
    >>> c.add(b'123456')
    >>> c.add(b'789')
    >>> c.value
    10673
    >>> c.add(b'')
    >>> c.value
    10673
    >>> c.add(c.value_as_bytes)
    >>> c.value
    0
    >>> c.check_residue()
    True
    """

    def __init__(self) -> None:
        assert len(self._TABLE) == 256
        self._value = 0xFFFF

    @classmethod
    def new(cls, data: bytes | bytearray | memoryview) -> CRC16CCITTFalse:
        c = cls()
        c.add(data)
        return c

    def add(self, data: bytes | bytearray | memoryview) -> None:
        val = self._value
        for x in data:
            val = ((val << 8) & 0xFFFF) ^ self._TABLE[(val >> 8) ^ x]
        self._value = val

    def check_residue(self) -> bool:
        return self._value == 0

    @property
    def value(self) -> int:
        return self._value

    @property
    def value_as_bytes(self) -> bytes:
        return self.value.to_bytes(2, "big")

    # fmt: off
    _TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD,
        0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A,
        0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
        0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
        0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
        0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87,
        0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
        0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290,
        0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E,
        0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F,
        0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
        0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83,
        0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
        0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
    ]
    # fmt: on


class IOManager:
    BAUD = 38400

    def __init__(self, serial_port: serial.Serial) -> None:
        self._port = serial_port
        if not self._port.is_open:
            self._port.open()
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._backlog: bytes | memoryview = b""

    def close(self) -> None:
        self._port.close()

    async def flush(self) -> None:
        await self._once()
        self._backlog = b""

    async def _once(self) -> Packet | None:
        self._port.timeout = 0
        self._backlog = b"".join(
            (self._backlog, await asyncio.get_event_loop().run_in_executor(self._executor, self._port.readall))
        )
        self._backlog, pkt = Packet.parse(self._backlog)
        if _logger.isEnabledFor(logging.DEBUG):
            _logger.debug("%s: Parsed %s, remainder:\n%s", self, pkt, self._backlog.hex())
        return pkt

    def __repr__(self) -> str:
        return f"{type(self).__name__}(serial_port={self._port})"
