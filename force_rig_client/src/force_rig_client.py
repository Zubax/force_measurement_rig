import asyncio
import time
from collections import deque
from matplotlib import pyplot

import click
import logging

import emoji
import serial
import sys
import numpy as np

from typing import Any, Callable, Coroutine
from shutil import get_terminal_size

from matplotlib.pyplot import savefig

from client_utils import inform, coroutine

from force_rig import ForceRig
from force_sensor_interface import ForceSensorInterface
from fluxgrip_config import FluxGripConfig
from step_drive_control import StepDriveControl

from uavcan.primitive.array import Integer32_1

logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)


@click.command(
    cls=click.Group,
    context_settings={
        "max_content_width": get_terminal_size()[0],
    },
)
@click.option("--verbose", "-v", count=True, help="Emit verbose log messages. Specify twice for extra verbosity.")
def cli(verbose: int) -> None:
    log_level = {
        0: logging.WARNING,
        1: logging.INFO,
        2: logging.DEBUG,
    }.get(verbose or 0, logging.DEBUG)
    logging.root.setLevel(log_level)


force_sensor_port_option = click.option(
    "--force-port",
    default="/dev/ttyUSB0",
    show_default=True,
    metavar="PORT_NAME",
    help="Force Sensor Serial port to use, or its URI",
    callback=lambda ctx, param, value: serial.serial_for_url(
        value,
        baudrate=ForceSensorInterface.BAUD,
        dsrdtr=None,  # On Arduino, DTR is used to reset the board, which we don't want.
        rtscts=None,
    ),
)


step_drive_port_option = click.option(
    "--drive-port",
    default="/dev/ttyUSB1",
    show_default=True,
    metavar="PORT_NAME",
    help="Step Drive Serial port to use, or its URI",
    callback=lambda ctx, param, value: serial.serial_for_url(
        value,
        baudrate=StepDriveControl.BAUD,
        dsrdtr=None,  # On Arduino, DTR is used to reset the board, which we don't want.
        rtscts=None,
    ),
)


@cli.command()
@force_sensor_port_option
@step_drive_port_option
@coroutine
async def execute(force_port: serial.Serial, drive_port: serial.Serial) -> None:
    """
    Execute a full force measurement cycle.
    Assumes that start is with arm at top position
    """
    # Measuring some demagnetization parameter set goes as follows:
    # Starting position of arm is assumed to be at t=0
    # 1. Setup:
    #    - calibration of force sensor
    #    - configuration of magnet
    # 2. Moving arm downwards, until either:
    #    - force sensor reports a negative force above 1N
    #    - t_max is reached (something went wrong)
    # 3. Magnet is magnetized and demagnetized
    # 4. Arm is moved upwards, force sensor readings are recorded
    #    - raw_force_sensor_reading
    #    - first_derivative_force
    # 5. Once first_derivative_force is larger than FIRST_DERIVATIVE_PULLING_THRESHOLD -> Pulling has started
    # 6. Once first_derivative_force is smaller than FIRST_DERIVATIVE_DETACHED_THRESHOLD -> Plate is detached
    # 7. Print max force
    test_values = [[-100, -90, -81,  73,  66, -59, -53,  48,  43, -39, -35,  31,  28, -25, -23,  21,  19, -17, -15,  14, -12,  11, -10,   9,  -8, -10,  43,  23, -17,   7,   2,  46,  34,  25,   4,   9,  47,  11, -22, -20, -33, -48,  -8, -11, -21, -49, -30,  21,  29,  11,  43],
                   [-100, -90, -81,  73,  66, -59, -53,  48,  43, -39, -35,  31,  28, -25, -23,  21,  19, -17, -15,  14, -12,  11, -10,   9,  -8, -10,  43,  23, -17,   7,   2,  46,  34,  25,   4,   9,  47,  11, -22, -20, -33, -48,  -8, -11, -21, -49, -30,  21,  29,  11,  43]]
    force_rig = ForceRig(drive_port, force_port)
    inform("Force rig setup")
    await force_rig.setup()

    fluxgrip_config = FluxGripConfig()
    inform("FluxGrip setup")
    await fluxgrip_config.start()

    for test_value_index in range(0, len(test_values)):
        try:
            new_demag_values = Integer32_1(np.array(
                test_values[test_value_index],
                dtype=np.int32,
            ))
            inform(f"Testing demag values: {test_values[test_value_index]}")
            await fluxgrip_config.configure_demag_cycle(new_demag_values)

            inform("Moving arm downwards until TOUCH_FORCE is detected")

            t_current = 0 # We assume we're starting from top position
            TOUCH_FORCE = -1.0 # Once pressure sensor detect 1N, we can assume the plate has touched the magnet
            start_time_down = time.time()
            await force_rig.move_arm_down()
            counter = 0
            while True:
                f_instant = await force_rig.get_instant_force()
                fmt = click.style(f"#{counter:06d}: ", dim=True)
                fmt += click.style(f"F_instant = {f_instant:+08.1f} N", fg="cyan", bold=True)
                inform(f"\r{fmt}", nl=False)
                if f_instant < TOUCH_FORCE:
                    break
                counter +=1

            await force_rig.move_arm_down_for(10.0) # To be sure the plate is completely flat on the magnet
            t_current = time.time() - start_time_down

            # Magnetize and demagnetize
            inform("\nMagnetizing and demagnetizing")
            await fluxgrip_config.magnetize()
            await asyncio.sleep(1)
            await fluxgrip_config.demagnetize()
            await asyncio.sleep(1)

            # Move arm up and print the force sensor data
            # 2 stop conditions:
            # 1. if t_current risks becoming negative (we've reached the top)
            # 2. if plate has detached (Force drops by DELTA_THRESHOLD)
            DELTA_THRESHOLD = 0.5
            start_time_up = time.time()
            await force_rig.move_arm_up()
            counter = 0
            f_peak = 0.0
            f_instant_storage = []
            plate_detached = False
            data_timeout = time.time()
            while True:
                f_instant = await force_rig.get_instant_force()
                f_instant_storage.append(f_instant)
                fmt = click.style(f"#{counter:06d}: ", dim=True)
                f_peak = f_instant if f_instant > f_peak else f_peak
                fmt += click.style(f"f_instant = {f_instant:+08.1f} N", fg="green", bold=True)
                fmt += click.style(f" f_peak = {f_peak:+08.1f} N", fg="cyan", bold=True)
                inform(f"\r{fmt}  ", nl=False)
                counter +=1
                if time.time() - start_time_up > t_current:
                    inform("\nTop reached "+emoji.emojize(":melting_face:"))
                    break
                if len(f_instant_storage) > 2:
                    if f_instant_storage[-2] - f_instant_storage[-1] > DELTA_THRESHOLD and not plate_detached:
                        inform("Plate detached!")
                        plate_detached = True
                        data_timeout = time.time() + 10 # we collect a bit more data and make sure the plate is completely removed from magnet
                if plate_detached:
                    if time.time() > data_timeout:
                        break

            await force_rig.stop_arm()
            total_time_up = time.time() - start_time_up
            t_current -= total_time_up

            # Plot out the result
            fig, axs = pyplot.subplots(2, 1, figsize=(10, 8))

            axs[0].plot(f_instant_storage, marker='o', color='blue')
            axs[0].set_title("F_instant")
            axs[0].set_xlabel("Time")
            axs[0].set_ylabel("Force [N]")
            axs[0].axhline(y=f_peak, color='red', linestyle='--', label='f_peak')
            axs[0].grid(True)

            # First derivative
            f_diff = np.diff(f_instant_storage)
            t_diff = range(1, len(f_instant_storage)) # is 1 point shorter

            axs[1].plot(t_diff, f_diff, marker='x', color='orange')
            axs[1].set_title("F_instant: first derivative")
            axs[1].set_xlabel("Time")
            axs[1].set_ylabel("ΔF / Δt")
            axs[1].axhline(y=-DELTA_THRESHOLD, color='red', linestyle='--', label='delta_threshold')
            axs[1].grid(True)

            # Demag values used and resulting remaining magnetic force
            test_value_counter = 1
            demag_text =  "Demag values: " + ', '.join(map(str, test_values[test_value_index]))
            result_text = f"\nF_peak: {f_peak:.2f} N"
            fig.text(0.5, 0.01, demag_text+result_text, ha='center', va='bottom', fontsize=8, wrap=True)

            pyplot.tight_layout(rect=[0, 0.06, 1, 1])  # leave space for the text
            pyplot.savefig(f"result_{test_value_index}", format="png")

            inform("\nReturning to start position")
            if t_current > 0:
                await force_rig.move_arm_up_for(t_current)

        except KeyboardInterrupt:
            await force_rig.stop_arm()
            await force_rig.close()
            fluxgrip_config.close()
            pass

    await force_rig.stop_arm()
    await force_rig.close()
    fluxgrip_config.close()

def main() -> None:  # https://click.palletsprojects.com/en/8.1.x/exceptions/
    status: Any = 1
    # noinspection PyBroadException
    try:
        status = cli.main(standalone_mode=False)
    except SystemExit as ex:
        status = ex.code
    except (KeyboardInterrupt, click.Abort) as ex:
        status = 127
        _logger.info("Interrupted")
        _logger.debug("%s: %s", type(ex).__name__, ex, exc_info=True)
    except click.ClickException as ex:
        status = ex.exit_code
        try:
            inform("", fg="red", reset=False, nl=False)
            ex.show()
        finally:
            inform("", nl=False)
    except Exception as ex:  # pylint: disable=broad-except
        inform(f"Error (run with -v for more info): {type(ex).__name__}: {ex}", fg="red")
        _logger.info("Error: %s: %s", type(ex).__name__, ex, exc_info=True)
    except BaseException as ex:  # pylint: disable=broad-except
        inform(f"Internal error, please report: {ex}", fg="red")
        _logger.exception("%s: %s", type(ex).__name__, ex)
    _logger.debug("EXIT %r", status)
    sys.exit(status)


if __name__ == "__main__":
    main()
