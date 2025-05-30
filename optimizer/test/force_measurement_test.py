import asyncio
import serial
import logging
import sys
import numpy as np
from numpy.typing import NDArray

from serial_interface import ForceSensorReading, ForceSensorIOManager
from force_measurement_utils import compute_forces, MovingAverage

logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)


async def display(port: serial.Serial, fir_order: int, calibrate_zero_bias: bool) -> None:
    """
    Display the force readings from the FMR rig in a human-readable format. This is the main command.
    """

    async def fetch() -> ForceSensorReading:
        rd = await iom.read(deadline=loop.time() + 10.0)
        if rd is None:
            raise RuntimeError("Timed out while waiting for data")
        return rd

    async def do_bias_calibration(uncalibrated_forces: NDArray[np.float64], n_samples: int) -> NDArray[np.float64]:
        agg = np.zeros_like(uncalibrated_forces)
        print("Zero bias calibration:")
        for i in range(n_samples):
            agg += compute_forces(await fetch())
            progress = (i + 1) / n_samples * 100
            sys.stdout.write(f"\rProgress: {progress:6.2f}% ({i + 1}/{n_samples})")
            sys.stdout.flush()
        print()  # Move to next line after completion
        return agg / n_samples

    iom = ForceSensorIOManager(port)
    _logger.info("Starting %s", iom)
    loop = asyncio.get_running_loop()
    f_peak = 0.0
    try:
        rd = await fetch()
        forces = compute_forces(rd)
        lpf = MovingAverage(fir_order, forces)
        zero_bias = np.zeros_like(forces)
        if calibrate_zero_bias:
            zero_bias = await do_bias_calibration(forces, 50)
            # inform(f"Zero bias: {zero_bias} N")
            print(f"Zero bias: {zero_bias}", flush=True)
        while True:
            rd = await fetch()
            forces = lpf(compute_forces(rd) - zero_bias)
            fmt = f"#{rd.seq_num:06d}: "
            breakdown = "".join(f"{x:+08.1f}" for x in forces)
            f_instant = sum(forces)
            f_peak = f_instant if abs(f_instant) > abs(f_peak) else f_peak
            fmt += f"F = {f_instant:+08.1f} N"
            fmt += f" = {breakdown}"
            fmt += f" F_peak = {f_peak:+08.1f} N"
            print(f"\r{fmt}  ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        iom.close()


def main() -> None:
    force_sensor_port = serial.serial_for_url("/dev/ttyUSB0", baudrate=38400)
    asyncio.run(display(force_sensor_port, fir_order=2, calibrate_zero_bias=True))


if __name__ == "__main__":
    main()
