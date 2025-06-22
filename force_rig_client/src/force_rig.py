import asyncio
import logging
import time
import serial
import numpy as np

from fluxgrip_config import FluxGripConfig
from force_sensor_interface import ForceSensorInterface
from step_drive_control import StepDriveControl

from typing import Optional
from numpy.typing import NDArray

# DSDL imports
from uavcan.primitive.array import Integer32_1

_logger = logging.getLogger(__name__)


class ForceRig:
    def __init__(self, step_drive_port: serial.Serial, force_sensor_port: serial.Serial):
        self._step_drive_control = StepDriveControl(step_drive_port)
        self._force_sensor_interface = ForceSensorInterface(force_sensor_port)

    async def setup(self):
        await self._step_drive_control.stop()

        _ = await self._force_sensor_interface.get_instant_forces(calibrate=True)

    async def close(self):
        await self._step_drive_control.stop()
        self._step_drive_control.close()
        self._force_sensor_interface.close()

    # _CONTACT_FORCE_THRESHOLD = np.float64(1)
    # _ARM_DOWN_TIMEOUT = 3
    # _ARM_UP_TIMEOUT = 1  # Should probably be less than _ARM_DOWN_TIMEOUT

    async def move_arm_down_for(self, timeout: float) -> None:

        await self._step_drive_control.down()

        end_time = time.time() + timeout
        while True:
            if time.time() > end_time:
                break
        # and stop arm!
        await self._step_drive_control.stop()

    async def move_arm_down(self) -> None:
        await self._step_drive_control.down()

    async def move_arm_up_for(self, timeout: float) -> None:
        await self._step_drive_control.up()

        end_time = time.time() + timeout
        while True:
            if time.time() > end_time:
                break
        # and stop arm!
        await self._step_drive_control.stop()

    async def move_arm_up(self) -> None:
        await self._step_drive_control.up()

    async def stop_arm(self) -> None:
        await self._step_drive_control.stop()

    async def get_instant_force(self) -> float:
        forces = await self._force_sensor_interface.get_instant_forces()
        return sum(forces)


