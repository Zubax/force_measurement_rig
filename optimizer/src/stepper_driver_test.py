import asyncio
import serial
import numpy as np
import logging

from serial_interface import StepDriveCommand, StepDriveIOManager

logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)


async def command_drive(port: serial.Serial) -> None:
    """
    Sends commands to the motor step driver.
    """

    async def fetch() -> StepDriveCommand:
        feedback_command = await iom.read(deadline=loop.time() + 10.0)
        if feedback_command is None:
            raise RuntimeError("Timed out while waiting for feedback command")
        return feedback_command

    def step_to_direction(step: np.int32) -> str:
        if step == -1:
            return "BACKWARD"
        elif step == 0:
            return "STOP"
        elif step == 1:
            return "FORWARD"
        else:
            raise ValueError(f"Invalid step value: {step}")

    iom = StepDriveIOManager(port)
    _logger.info("Starting %s", iom)
    loop = asyncio.get_running_loop()
    try:
        while True:
            command = np.int32(1)
            print(f"Sending command: {step_to_direction(command)}")
            while not await iom.send_command(command):
                print("Sending command...")
            print(f"Command sent successfully")
            feedback_command = await fetch()
            print(f"Feedback command: %s", step_to_direction(feedback_command.step))
    except KeyboardInterrupt:
        pass
    finally:
        iom.close()


def main() -> None:
    step_drive_port = serial.serial_for_url("/dev/ttyUSB0", baudrate=38400)
    # step_drive_port = serial.serial_for_url("loop://") # For this to work, comment out "await self._once()" in IOManager::flush()
    asyncio.run(command_drive(step_drive_port))


if __name__ == "__main__":
    main()
