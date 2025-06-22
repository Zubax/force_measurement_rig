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
from skopt import gp_minimize
from skopt.space import Integer

from client_utils import inform, coroutine

from force_rig import ForceRig
from force_sensor_interface import ForceSensorInterface
from fluxgrip_config import FluxGripConfig
# from src.bayesian_optimizer import search_space
from step_drive_control import StepDriveControl
from force_measurement_session import ForceMeasurementSession

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
    test_values = [[-100, -90, -81,  73,  66, -59, -53,  48,  43, -39, -35,  31,  28, -25, -23,  21,  19, -17, -15,  14, -12,  11, -10,   9,  -8, -10,  43,  23, -17,   7,   2,  46,  34,  25,   4,   9,  47,  11, -22, -20, -33, -48,  -8, -11, -21, -49, -30,  21,  29,  11,  43]]
    force_measurement_session = ForceMeasurementSession(force_port, drive_port)
    await force_measurement_session.setup()

    for value in test_values:
        average = await force_measurement_session.run_cycle(value)
        inform(f"\naverage f_peak: {average:.1f}")

    await force_measurement_session.cleanup()

@cli.command()
@force_sensor_port_option
@step_drive_port_option
def optimize(force_port: serial.Serial, drive_port: serial.Serial) -> None:
    """
    Optimize
    """
    # Good initial guess
    x0 = [
        +7,-6,+6,-5,+5,-4,+4,-3,+3,-3,+3,-2,+2,-2,+2,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1
    ]
    y0 = 8
    force_measurement_session = ForceMeasurementSession(force_port, drive_port)

    loop = asyncio.new_event_loop()
    loop.run_until_complete(force_measurement_session.setup())

    search_space = [Integer(-30, 30) for _ in range(len(x0))]

    def optimize_target(params) -> float:
        FIXED_PRE_DEMAG_VALUES = [-100,-90,-81,+73,+66,-59,-53,+48,+43,-39,-35,+31,+28,-25,-23,+21,+19,-17,-15,+14,-12,+11,-10,+9,-8]
        result = loop.run_until_complete(force_measurement_session.run_cycle(params, FIXED_PRE_DEMAG_VALUES))
        return result


    res = gp_minimize(optimize_target, search_space, n_calls=750, random_state=42, x0=x0, y0=y0)
    inform(f"\n✅ Best force: {res.fun}")
    inform(f"\n🧲 Best demag values: {res.x}")

    loop.run_until_complete(force_measurement_session.cleanup())

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
