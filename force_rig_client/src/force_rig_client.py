import asyncio
import click
import logging
import serial
import sys
import numpy as np

from typing import Any, Callable, Coroutine
from shutil import get_terminal_size
from client_utils import inform, coroutine

from force_rig import ForceRig

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
    Execute a full force measurement cycle
    """
    inform("Setting up ForceRig")
    force_rig = ForceRig(drive_port, force_port)
    await force_rig.setup()
    step_drive_control = StepDriveControl(drive_port)
    force_sensor_interface = ForceSensorInterface(force_port)
    fluxgrip_config = FluxGripConfig()
    try:
        inform("Configuring FluxGrip")
        await fluxgrip_config.start()

        new_demag_values = np.array(
            [-100, -90, -81,  73,  66, -59, -53,  48,  43, -39, -35,  31,  28, -25, -23,  21,  19, -17, -15,  14, -12,  11, -10,   9,  -8, -10,  43,  23, -17,   7,   2,  46,  34,  25,   4,   9,  47,  11, -22, -20, -33, -48,  -8, -11, -21, -49, -30,  21,  29,  11,  43],
            dtype=np.int32,
        )
        # [-100,-90,-81,+73,+66,-59,-53,+48,+43,-39,-35,+31,+28,-25,-23,+21,+19,-17,-15,+14,-12,+11,-10,+9,-8,+7,-6,+6,-5,+5,-4,+4,-3,+3,-3,+3,-2,+2,-2,+2,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1],
        new_demag_val = Integer32_1(new_demag_values)
        await fluxgrip_config.configure_demag_cycle(new_demag_val)

        # Move arm down
        # if not duration > 0:
        #     raise click.BadParameter("must be positive", param_hint="duration")
        inform(f"Moving arm downwards for 1 second")
        await step_drive_control.down()
        await asyncio.sleep(1) # Downwards movement is much faster than upwards
        await step_drive_control.stop()

        inform("Make sure the plate is attached correctly! (and press Enter to continue)")
        await asyncio.to_thread(input)

        # Magnetize and demagnetize
        await fluxgrip_config.magnetize()
        await asyncio.sleep(3)
        await fluxgrip_config.demagnetize()
        await asyncio.sleep(3)

        # Setup force sensor
        f_abs_peak = 0.0
        f_pos_peak = 0.0
        loop = asyncio.get_running_loop()
        forces_read = await fetch(force_sensor_interface, loop)
        forces = compute_forces(forces_read)
        lpf = MovingAverage(2, forces)  # fir_order = 2 (default)
        inform(f"Calibrating")
        zero_bias = await do_bias_calibration(force_sensor_interface, loop, forces, 50)

        # Move arm up and print the force sensor data
        inform(f"Moving arm upwards for {duration} seconds")
        await step_drive_control.up()
        timeout = loop.time() + duration
        f_pos_peak_buffer = [0] * 10
        has_started_pulling = False
        plate_attached = True
        while loop.time() < timeout and plate_attached:
            rd = await fetch(force_sensor_interface, loop)
            forces = lpf(compute_forces(rd) - zero_bias)
            fmt = click.style(f"#{rd.seq_num:06d}: ", dim=True)
            breakdown = "".join(f"{x:+08.1f}" for x in forces)
            f_instant = sum(forces)
            f_abs_peak = f_instant if abs(f_instant) > abs(f_abs_peak) else f_abs_peak
            f_pos_peak = f_instant if f_instant > f_pos_peak else f_pos_peak
            f_pos_peak_buffer = f_pos_peak_buffer[1:] + [f_pos_peak]
            fmt += click.style(f"F = {f_instant:+08.1f} N", fg="green", bold=True)
            fmt += click.style(f" F_breakdown = {breakdown}", dim=True)
            fmt += click.style(f" F_abs_peak = {f_abs_peak:+08.1f} N", fg="cyan", bold=True)
            fmt += click.style(f" F_pos_peak = {f_pos_peak:+08.1f} N", fg="magenta", bold=True)
            inform(f"\r{fmt}  ", nl=False)
            is_increasing = all(f_pos_peak_buffer[i] < f_pos_peak_buffer[i + 1] for i in range(len(f_pos_peak_buffer) - 1))
            if is_increasing:
                inform("Has started pulling")
                has_started_pulling = True
            if has_started_pulling:
                delta = f_pos_peak - f_instant
                if delta > 1:
                    inform("Plate has detached")
                    plate_attached = False
                    # await asyncio.sleep(2) # Allow arm to move a little higher before stopping
        await step_drive_control.stop()
    except KeyboardInterrupt:
        pass
    finally:
        await step_drive_control.stop()
        step_drive_control.close()
        force_sensor_interface.close()
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
