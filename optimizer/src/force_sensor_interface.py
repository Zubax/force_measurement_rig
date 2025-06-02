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

    async def do_bias_calibration(self, n_samples: int = 50) -> None:
        # assert self._zero_bias is None, "Calibration is already done"
        rd = await self.fetch(timeout=5)
        uncalibrated_forces = self.compute_forces(rd)
        self._lpf = MovingAverage(self._fir_order, uncalibrated_forces)
        agg = np.zeros_like(uncalibrated_forces)
        _logger.info("Zero bias calibration!")
        for i in range(n_samples):
            agg += self.compute_forces(await self.fetch(timeout=1))
            progress = (i + 1) / n_samples * 100
            # _logger.info(f"Progress: {progress:6.2f}% ({i + 1}/{n_samples})")
        self._zero_bias = agg / n_samples

    async def fetch(self, timeout: float) -> ForceSensorReading | None:
        deadline = asyncio.get_event_loop().time() + timeout
        while True:
            if pkt := await self._once():
                seq_num, adc_readings, calibration = self._STRUCT_READING.unpack_from(pkt.payload)
                return ForceSensorReading(
                    seq_num=seq_num,
                    adc_readings=np.frombuffer(adc_readings, dtype=np.int32, count=ForceSensorReading.CHANNEL_COUNT),
                    calibration=np.frombuffer(calibration, dtype=np.float32, count=ForceSensorReading.CHANNEL_COUNT * 2)
                    .reshape((2, ForceSensorReading.CHANNEL_COUNT))
                    .astype(np.float64),
                )
            if deadline < asyncio.get_event_loop().time():
                return None
            await asyncio.sleep(1e-3)

    @staticmethod
    def compute_forces(rd: ForceSensorReading) -> NDArray[np.float64]:
        return np.array(
            [
                np.polynomial.Polynomial(np.flip(coe.flatten()))(float(x))
                for x, coe in zip(rd.adc_readings, rd.calibration.T)
            ]
        )

    async def read_instant_force(self, timeout: int) -> np.float64:
        assert self._zero_bias is not None, "Calibration hasn't been done"

        rd = await asyncio.wait_for(self.fetch(timeout=timeout), timeout=timeout)

        forces = self._lpf(np.float64(self.compute_forces(rd) - self._zero_bias))
        f_instant = np.sum(forces)
        self._f_peak = f_instant if abs(f_instant) > abs(self._f_peak) else self._f_peak
        return f_instant

    def reset_peak_force(self):
        self._f_peak = np.float64(0)

    def read_peak_force(self) -> np.float64:
        return self._f_peak

    async def write_calibration(self, cal: NDArray[np.float64]) -> bool:
        payload = cal.astype(np.float32).tobytes()
        buf = Packet(memoryview(payload)).compile()
        await asyncio.get_event_loop().run_in_executor(self._executor, self._port.write, buf)
        await asyncio.sleep(1.0)
        await self.flush()
        rd = await self.fetch(timeout=1)
        return np.allclose(rd.calibration, cal, atol=1e-3, rtol=1e-3, equal_nan=True)
