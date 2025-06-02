import os
import sys
import asyncio
import serial
import numpy as np
import logging

# Add the src directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from step_drive_control import StepDriveControl, StepDriveCommand

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)


async def main() -> None:
    step_drive_port = serial.serial_for_url("/dev/ttyUSB0", baudrate=38400)
    step_drive_control = StepDriveControl(step_drive_port)
    current_command = await step_drive_control.fetch(timeout=5)
    assert current_command is not None
    _logger.info(f"Current command: {current_command.step}")
    await step_drive_control.down()
    await asyncio.sleep(2)
    await step_drive_control.stop()
    step_drive_control.close()


if __name__ == "__main__":
    asyncio.run(main())
