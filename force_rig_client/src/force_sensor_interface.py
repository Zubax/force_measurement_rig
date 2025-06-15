from __future__ import annotations

import asyncio
import dataclasses
import struct
import serial
import logging
import numpy as np

from serial_interface import IOManager, Packet
from numpy.typing import NDArray
from typing import Optional, TypeVar, Generic

_logger = logging.getLogger(__name__)

T = TypeVar("T")


class MovingAverage(Generic[T]):
    """
    >>> ma = MovingAverage(3, 0)
    >>> ma(1)  # doctest: +ELLIPSIS
    0.333...
    >>> ma(2)  # doctest: +ELLIPSIS
    1.0
    >>> ma(3)  # doctest: +ELLIPSIS
    2.0

    >>> maa = MovingAverage(3, np.array([1, 2, 3]))
    >>> maa(np.array([2, 2, 2]))  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    array([1.333..., 2..., 2.666...])
    >>> maa(np.array([2, 2, 2]))  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    array([1.666..., 2..., 2.333...])
    >>> maa(np.array([2, 2, 2]))  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    array([2..., 2..., 2...])
    """

    def __init__(self, depth: int, initial: T) -> None:
        self._values = [initial] * depth
        self._sum = sum(self._values)  # type: ignore
        self._index = 0

    def __call__(self, value: T) -> T:
        self._sum -= self._values[self._index]  # type: ignore
        self._values[self._index] = value
        self._sum += value  # type: ignore
        self._index = (self._index + 1) % len(self._values)
        return self._sum * (1 / len(self._values))  # type: ignore


@dataclasses.dataclass(frozen=True)
class ForceSensorReading:
    """
    A single reading reported by the digitizer.
    """

    seq_num: int
    adc_readings: NDArray[np.int32]
    calibration: NDArray[np.float64]

    CHANNEL_COUNT = 4


class ForceSensorInterface(IOManager):
    """
    Reads the data from the serial port and parses it into readings.
    Also allows sending commands to the digitizer.

    >>> import serial
    >>> import time
    >>> port = serial.serial_for_url("loop://")
    >>> valid_packet = bytes.fromhex(
    ...     "B44CECF250000000"
    ...     "020000000000000000000000000000000000000000000000"
    ...     "00998F0F00BC64040000000000000000"
    ...     "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
    ...     "473D")
    >>> _ = port.write(valid_packet)
    >>> async def test():
    ...     reader = ForceSensorInterface(port)
    ...     reading = await reader.fetch(timeout=1)
    ...     assert reading is not None
    ...     assert reading.seq_num == 2
    ...     assert (reading.adc_readings == [261069056, 73710592, 0, 0]).all()
    ...     assert reading.calibration.shape == (2, 4)
    ...     reading = await reader.fetch(timeout=1)
    ...     assert reading is None
    ...     reader.close()
    >>> asyncio.run(test())
    """

    _STRUCT_READING = struct.Struct(r"< Q 8x 8x 16s 40s")

    def __init__(self, port: serial.Serial, fir_order: int = 2) -> None:
        super().__init__(port)
        self._port: serial.Serial = port
        self._fir_order: int = fir_order
        self._zero_bias: Optional[NDArray[np.float64]] = None
        self._lpf: Optional[MovingAverage[np.float64]] = None
        self._f_peak: np.float64 = np.float64(0)

    async def read(self, deadline: float) -> ForceSensorReading | None:
        """
        Waits for up to the specified deadline for a new reading to arrive.
        Returns the new reading, or None if the deadline has expired.
        """
        while True:
            if pkt := await self._once():
                seq_num, adc_readings, calibration = self._STRUCT_READING.unpack_from(pkt.payload)
                rd = ForceSensorReading(
                    seq_num=seq_num,
                    adc_readings=np.frombuffer(
                        adc_readings,
                        dtype=np.int32,
                        count=ForceSensorReading.CHANNEL_COUNT,
                    ),
                    calibration=np.frombuffer(
                        calibration,
                        dtype=np.float32,
                        count=ForceSensorReading.CHANNEL_COUNT * 2,
                    )
                    .reshape((2, ForceSensorReading.CHANNEL_COUNT))
                    .astype(np.float64),
                )
                _logger.debug("%s: Received reading %s", self, rd)
                return rd
            if deadline < asyncio.get_event_loop().time():
                return None
            await asyncio.sleep(1e-3)  # This is silly but works for the MVP.

    async def write_calibration(self, cal: NDArray[np.float64]) -> bool:
        """
        Writes the calibration data to the digitizer and waits for confirmation.
        Returns True if the calibration was accepted, False otherwise (in which case retrying may help).
        """
        payload = cal.astype(np.float32).tobytes()
        buf = Packet(memoryview(payload)).compile()
        _logger.debug("%s: Sending calibration packet: %s", self, buf.hex())
        await asyncio.get_event_loop().run_in_executor(self._executor, self._port.write, buf)
        await asyncio.sleep(1.0)  # Wait for the new data to be processed.
        await self.flush()
        rd = await self.read(asyncio.get_event_loop().time() + 10)
        if rd is None:
            _logger.info("%s: Calibration confirmation timed out", self)
            return False
        _logger.info(
            "%s: Received calibration confirmation. Sent calibration:\n%s\nReceived calibration:\n%s",
            self,
            cal,
            rd.calibration,
        )
        return np.allclose(rd.calibration, cal, atol=1e-3, rtol=1e-3, equal_nan=True)


def compute_forces(rd: ForceSensorReading) -> NDArray[np.float64]:
    return np.array(
        [
            np.polynomial.Polynomial(np.flip(coe.flatten()))(float(x))
            for x, coe in zip(rd.adc_readings, rd.calibration.T)
        ]
    )


async def fetch(iom: ForceSensorInterface, loop: asyncio.AbstractEventLoop) -> ForceSensorReading:
    rd = await iom.read(deadline=loop.time() + 10.0)
    if rd is None:
        raise RuntimeError("Timed out while waiting for data")
    return rd


async def do_bias_calibration(
    iom: ForceSensorInterface, loop: asyncio.AbstractEventLoop, uncalibrated_forces: NDArray[np.float64], n_samples: int
) -> NDArray[np.float64]:

    agg = np.zeros_like(uncalibrated_forces)
    log_interval = max(n_samples // 10, 1)  # Log progress at 10% intervals

    for i in range(n_samples):
        agg += compute_forces(await fetch(iom, loop))
        if (i + 1) % log_interval == 0 or i == n_samples - 1:
            _logger.info("Calibrating bias... %d/%d samples collected", i + 1, n_samples)

    return agg / n_samples
