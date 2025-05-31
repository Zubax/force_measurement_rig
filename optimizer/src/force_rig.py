import asyncio
import logging
import time
import numpy as np

from fluxgrip_config import FluxGripConfig
from force_sensor_interface import ForceSensorInterface
from step_drive_control import StepDriveControl
from optimizer import DEMAG_REGISTER_LENGTH

from typing import Optional
from numpy.typing import NDArray

# DSDL imports
from uavcan.primitive.array import Integer32_1

_logger = logging.getLogger(__name__)


class ForceRig:
    def __init__(self, step_drive_port, force_sensor_port):
        self._step_drive_control = StepDriveControl(step_drive_port)
        self._force_sensor_interface = ForceSensorInterface(force_sensor_port)
        self._fluxgrip_config: Optional[FluxGripConfig] = None

    async def setup(self):
        await self._fluxgrip_config.start()
        await self._force_sensor_interface.do_bias_calibration()

    async def configure_demag_register(self, data: NDArray[np.int32]):
        _logger.info("Configuring demag registers!")
        assert data.size == DEMAG_REGISTER_LENGTH
        dsdl_data = Integer32_1(data)
        await self._fluxgrip_config.configure_demag_cycle(dsdl_data)

    _CONTACT_FORCE_THRESHOLD = np.float64(10)
    _ARM_DOWN_TIMEOUT = 2
    _ARM_UP_TIMEOUT = 1  # Should probably be less than _ARM_DOWN_TIMEOUT

    async def pull_arm_down(self) -> None:
        _logger.info("Pulling arm down!")
        await self._step_drive_control.down()

        # Keep lowering the arm until either force sensor reports change, or timeout!
        start_time = time.time()
        while True:
            if time.time() - start_time > self._ARM_DOWN_TIMEOUT:
                break
            try:
                force_reading = await self._force_sensor_interface.read_instant_force(timeout=1)
                if force_reading > self._CONTACT_FORCE_THRESHOLD:
                    break
            except asyncio.TimeoutError:
                await self._step_drive_control.stop()
        # and stop arm!
        await self._step_drive_control.stop()

    async def magnetize_and_demagnetize(self) -> None:
        _logger.info("Magnetizing and demagnetizing!")
        await self._fluxgrip_config.magnetize()
        await asyncio.sleep(1)
        await self._fluxgrip_config.demagnetize()

    async def pull_arm_up(self) -> None:
        _logger.info("Pulling up arm!")
        self._force_sensor_interface.reset_peak_force()
        await self._step_drive_control.up()

        # Keep pulling arm up until timeout reached
        start_time = time.time()
        while True:
            # We need to keep polling the force sensor to make sure f_peak gets updated
            _ = await self._force_sensor_interface.read_instant_force(timeout=1)
            if time.time() - start_time > self._ARM_UP_TIMEOUT:
                break
        # and stop arm!
        await self._step_drive_control.stop()

    def read_peak_force(self) -> np.float64:
        peak_force = self._force_sensor_interface.read_peak_force()
        _logger.info(f"Remaining magnetic force: {peak_force}N")

        return peak_force
