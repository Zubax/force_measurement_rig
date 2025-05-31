from __future__ import annotations

import asyncio
import dataclasses
import logging
import struct
import numpy as np

from serial_interface import IOManager, Packet

_logger = logging.getLogger(__name__)


@dataclasses.dataclass(frozen=True)
class StepDriveCommand:
    """
    Step command that is sent to motor step driver
    """

    step: np.int32  # 0 = stop, 1 = forward, -1 = backward


class StepDriveControl(IOManager):
    """
    Reads the data from the serial port and parses it into commands.
    Also allows sending commands to the motor step driver.

    >>> import serial
    >>> import time
    >>> port = serial.serial_for_url("loop://")
    >>> valid_packet = bytes.fromhex(
    ...     "B44CECF204000000"
    ...     "FFFFFFFF"
    ...     "1D0F")
    >>> _ = port.write(valid_packet)
    >>> async def test():
    ...     reader = StepDriveControl(port)
    ...     command = await reader.read(asyncio.get_event_loop().time() + 1)
    ...     assert command is not None
    ...     assert command.step == np.int32(-1)
    ...     command = await reader.read(asyncio.get_event_loop().time() + 1)
    ...     assert command is None
    ...     reader.close()
    >>> asyncio.run(test())
    """

    _STRUCT_COMMAND = struct.Struct(r"< i")

    _DIRECTION_TO_STEP = {"UP": np.int32(-1), "STOP": np.int32(0), "DOWN": np.int32(1)}

    async def read(self, deadline: float) -> StepDriveCommand | None:
        while True:
            pkt = await self._once()
            if pkt is not None:
                (step,) = self._STRUCT_COMMAND.unpack_from(pkt.payload)
                return StepDriveCommand(step=np.int32(step))
            if deadline < asyncio.get_event_loop().time():
                return None
            await asyncio.sleep(1e-3)

    async def _send_command(self, command: np.int32) -> bool:
        payload = command.astype(np.int32).tobytes()
        buf = Packet(memoryview(payload)).compile()
        res = await asyncio.to_thread(self._port.write, buf)
        assert res is not None
        await asyncio.sleep(1.0)
        await self.flush()
        rd = await self.read(asyncio.get_event_loop().time() + 1)
        return rd is not None and (rd.step == command)

    async def up(self):
        while not await self._send_command(self._DIRECTION_TO_STEP["UP"]):
            _logger.info("Sending command to pull arm up")

    async def stop(self):
        while not await self._send_command(self._DIRECTION_TO_STEP["STOP"]):
            _logger.info("Sending command to stop arm")

    async def down(self):
        while not await self._send_command(self._DIRECTION_TO_STEP["DOWN"]):
            _logger.info("Sending command to pull arm down")

    @staticmethod
    def step_to_direction(step: np.int32) -> str:
        if step == -1:
            return "BACKWARD"
        elif step == 0:
            return "STOP"
        elif step == 1:
            return "FORWARD"
        else:
            raise ValueError(f"Invalid step value: {step}")
