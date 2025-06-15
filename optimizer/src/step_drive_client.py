import sys
import asyncio
import click
import logging
import serial

from typing import Any, Callable, Coroutine
from serial_interface import IOManager
from shutil import get_terminal_size
from step_drive_control import StepDriveControl
from client_utils import inform, coroutine

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


port_option = click.option(
    "--port",
    "-P",
    default="/dev/ttyUSB1",
    show_default=True,
    metavar="PORT_NAME",
    help="Serial port to use, or its URI",
    callback=lambda ctx, param, value: serial.serial_for_url(
        value,
        baudrate=IOManager.BAUD,
        dsrdtr=None,  # On Arduino, DTR is used to reset the board, which we don't want.
        rtscts=None,
    ),
)


@cli.command()
@port_option
@click.option("--duration", "-d", default=1, show_default=True, help="Timeout until the motor stops running")
@coroutine
async def up(port: serial.Serial, duration: int) -> None:
    """
    This command is used to move the arm in the upwards direction.
    """
    if not duration > 0:
        raise click.BadParameter("must be positive", param_hint="duration")
    step_drive_control = StepDriveControl(port)
    _logger.info(f"Moving upwards for {duration} seconds")
    await step_drive_control.up()
    await asyncio.sleep(duration)
    await step_drive_control.stop()
    step_drive_control.close()


@cli.command()
@port_option
@click.option("--duration", "-d", default=1, show_default=True, help="Timeout until the motor stops running")
@coroutine
async def down(port: serial.Serial, duration: int) -> None:
    """
    This command is used to move the arm in the downwards direction.
    """
    if not duration > 0:
        raise click.BadParameter("must be positive", param_hint="duration")
    step_drive_control = StepDriveControl(port)
    _logger.info(f"Moving downwards for {duration} seconds")
    await step_drive_control.down()
    await asyncio.sleep(duration)
    await step_drive_control.stop()
    step_drive_control.close()


def main() -> None:
    status: Any = 1
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
