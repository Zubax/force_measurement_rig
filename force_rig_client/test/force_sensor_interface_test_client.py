import sys
import os
import asyncio
import serial
import logging

# Add the src directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from force_sensor_interface import ForceSensorInterface

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)


async def main() -> None:
    force_sensor_port = serial.serial_for_url("/dev/ttyUSB0", baudrate=38400)
    force_sensor_interface = ForceSensorInterface(force_sensor_port)
    await force_sensor_interface.do_bias_calibration()
    force_sensor_interface.reset_peak_force()

    try:
        while True:
            f_instant = await force_sensor_interface.read_instant_force(timeout=5)
            f_peak = force_sensor_interface.read_peak_force()
            _logger.info(f"f_instant: {abs(f_instant):.1f} N")
            _logger.info(f"f_peak: {abs(f_peak):.1f} N")

    except KeyboardInterrupt:
        pass
    finally:
        force_sensor_interface.close()


if __name__ == "__main__":
    asyncio.run(main())
