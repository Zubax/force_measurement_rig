from datetime import datetime

from skopt import gp_minimize
from skopt.space import Integer
import numpy as np
import serial
import asyncio
import logging

from fluxgrip_config import FluxGripConfig
from step_drive_control import StepDriveControl
from force_sensor_interface import (
    ForceSensorInterface,
    MovingAverage,
    ForceSensorReading,
    compute_forces,
    fetch,
    do_bias_calibration,
)

from uavcan.primitive.array import Integer32_1

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)

# Create search space: 51 integers from -100 to 100
DEMAG_REGISTER_LENGTH = 26
search_space = [Integer(-50, 50) for _ in range(DEMAG_REGISTER_LENGTH)]

# We'll store the best force here
best_force = float("inf")
best_values = None
test_number = 1

default_first_demag_val = [-100,-90,-81,+73,+66,-59,-53,+48,+43,-39,-35,+31,+28,-25,-23,+21,+19,-17,-15,+14,-12,+11,-10,+9,-8]

# Good initial guess
x0 = [
    +7,-6,+6,-5,+5,-4,+4,-3,+3,-3,+3,-2,+2,-2,+2,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1
]
y0 = 6



async def async_objective(demag_values: list[int]) -> float:
    """
    Objective function to minimize: the measured remaining force
    """
    global best_force, best_values

    loop = asyncio.get_event_loop()
    # You can replace the ports and duration here with your actual setup
    force_port = serial.Serial('/dev/ttyUSB0', ForceSensorInterface.BAUD)
    drive_port = serial.Serial('/dev/ttyUSB1', StepDriveControl.BAUD)

    async def run_one():
        nonlocal demag_values
        nonlocal force_port, drive_port
        result_holder = {"f_pos_peak": 9999.0}

        async def patched_execute():
            global test_number
            step_drive_control = StepDriveControl(drive_port)
            force_sensor_interface = ForceSensorInterface(force_port)
            fluxgrip_config = FluxGripConfig()
            try:
                _logger.info("Configuring FluxGrip")
                await fluxgrip_config.start()
                new_demag_val = Integer32_1(np.array(default_first_demag_val + demag_values, dtype=np.int32))
                _logger.info(f"Testing: {new_demag_val}")
                with open("log.txt", "a") as file:
                    file.write(f"Test #{test_number}\n")
                    test_number +=1
                    file.write(f"Demag values: {new_demag_val}\n")
                await fluxgrip_config.configure_demag_cycle(new_demag_val)

                _logger.info("Moving arm downwards for 1 second")
                await step_drive_control.down()
                await asyncio.sleep(1)
                await step_drive_control.stop()
                # inform("Press Enter to continue")
                # await asyncio.to_thread(input)

                _logger.info("Magnetize and Demagnetize")
                await fluxgrip_config.magnetize()
                await asyncio.sleep(4)
                await fluxgrip_config.demagnetize()
                await asyncio.sleep(5)

                f_abs_peak = 0.0
                f_pos_peak = 0.0
                loop = asyncio.get_running_loop()
                forces_read = await fetch(force_sensor_interface, loop, flush=True)
                forces = compute_forces(forces_read)
                _logger.info(f"forces: {forces}")
                lpf = MovingAverage(6, forces)
                _logger.info(f"lpf: {lpf}")
                _logger.info("Calibrating")
                zero_bias = await do_bias_calibration(force_sensor_interface, loop, forces, 50)

                await step_drive_control.up()
                timeout = loop.time() + 60 # Moving up is slower, therefore longer duration is required
                max_force = 15 # max force to prevent damage to setup
                has_started_pulling = False
                plate_attached = True
                timed_out = False
                force_too_large = False
                stopped_increasing = False
                last_forces = []
                force_stability_threshold = 0.2
                stability_sample_count = 100
                while not timed_out and plate_attached and not force_too_large:
                    rd = await fetch(force_sensor_interface, loop)
                    forces = lpf(compute_forces(rd) - zero_bias)
                    f_instant = sum(forces)
                    f_abs_peak = max(f_abs_peak, abs(f_instant))
                    f_pos_peak = max(f_pos_peak, f_instant)
                    _logger.info(f"f_instant: {f_instant} f_abs_peak: {f_abs_peak} f_pos_peak: {f_pos_peak}")
                    if f_instant > 0.3:
                        if has_started_pulling is False:
                            _logger.info("Pulling has started")
                            has_started_pulling = True
                    if has_started_pulling and f_instant < 0.80*f_pos_peak: # Force drop means the plate has detached
                        _logger.info("Plate detached")
                        plate_attached = False
                    # Keep track of last 100 force samples
                    last_forces.append(f_instant)
                    if len(last_forces) > stability_sample_count:
                        last_forces.pop(0)
                    # Check if the force has increased by more than force_stability_threshold N in the last 100 samples
                    if len(last_forces) == stability_sample_count:
                        if max(last_forces) - min(last_forces) < force_stability_threshold and has_started_pulling:
                            stopped_increasing = True

                    if has_started_pulling and stopped_increasing:
                        _logger.info("Force has stopped increasing")
                        plate_attached = False
                    if f_instant > max_force:
                        _logger.info("Force too large")
                        force_too_large = True
                    if loop.time() > timeout:
                        if plate_attached and not force_too_large:
                            _logger.info("Wire might have stretched, adding 5 seconds")
                            timeout = loop.time() + 5
                        else:
                            _logger.info("Timed out")
                            timed_out = True
                await step_drive_control.stop()

                if f_pos_peak > max_force:
                    result_holder["f_pos_peak"] = 9999.0 # Return high penalty
                else:
                    result_holder["f_pos_peak"] = f_pos_peak
                with open("log.txt", "a") as file:
                    file.write(f"Result: {result_holder["f_pos_peak"]}\n")
                    file.write("===\n")
            except asyncio.TimeoutError:
                _logger.error("Some error occurred, probably timeout of demagnetization")

            finally:
                await step_drive_control.stop()
                step_drive_control.close()
                force_sensor_interface.close()
                fluxgrip_config.close()

        await patched_execute()
        _logger.info("Returning result")
        return result_holder["f_pos_peak"]

    try:
        result = await run_one()
    except Exception as e:
        print("Error during run:", e)
        return 9999.0  # Return high penalty

    if result < best_force:
        best_force = result
        best_values = demag_values.copy()

    print(f"Force: {result:.2f} N")
    return result

def objective(demag_values: list[int]) -> float:
    return asyncio.run(async_objective(demag_values))

def main() -> None:
    with open("log.txt", "a") as file:
        file.write(f"Starting execution: {datetime.now()}\n")
    # Run optimization
    res = gp_minimize(objective, search_space, n_calls=30, random_state=42, x0=x0, y0=y0)
    print("\n✅ Best force:", res.fun)
    print("🔧 Best demag values:")
    print(res.x)

if __name__ == "__main__":
    main()

