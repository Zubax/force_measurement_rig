import os
import sys
import asyncio
import logging
import numpy as np

import serial

# from src.force_rig import ForceRig

# Add the src directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from force_rig import ForceRig

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)


async def main() -> None:
    step_drive_port = serial.serial_for_url("/dev/ttyUSB0", baudrate=38400)
    force_sensor_port = serial.serial_for_url("/dev/ttyUSB1", baudrate=38400)
    force_rig = ForceRig(step_drive_port, force_sensor_port)
    await force_rig.setup()

    # fmt: off
    new_demag_values = np.array(
        [
            -100, +90, -80, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1,
            +11, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1,
            +1, -1, +1, -1,
        ],
        dtype=np.int32,
    )
    # fmt: off
    try:
        while True:
            _logger.info("Configuring")
            await force_rig.configure_demag_register(new_demag_values)
            await force_rig.pull_arm_down()
            _logger.info("Magnetizing and demagnetizing")
            await force_rig.magnetize_and_demagnetize()
            await force_rig.pull_arm_up()
            peak_force = force_rig.read_peak_force()
            _logger.info(f"Remaining magnetic force: {peak_force:.1f} N")
            break
    except KeyboardInterrupt:
        pass
    finally:
        await force_rig.close()


if __name__ == "__main__":
    asyncio.run(main())
