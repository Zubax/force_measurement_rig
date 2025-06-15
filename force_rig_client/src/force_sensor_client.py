#!/usr/bin/env python3
# Copyright (C) 2023 Zubax Robotics
from __future__ import annotations

import sys
import asyncio
import logging
import click
import serial
import numpy as np

from shutil import get_terminal_size
from typing import Any
from numpy.typing import NDArray

from client_utils import inform, coroutine
from force_sensor_interface import (
    ForceSensorReading,
    MovingAverage,
    ForceSensorInterface,
    compute_forces,
    fetch,
    do_bias_calibration,
)


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
    default="/dev/ttyUSB0",
    show_default=True,
    metavar="PORT_NAME",
    help="Serial port to use, or its URI",
    callback=lambda ctx, param, value: serial.serial_for_url(
        value,
        baudrate=ForceSensorInterface.BAUD,
        dsrdtr=None,  # On Arduino, DTR is used to reset the board, which we don't want.
        rtscts=None,
    ),
)


@cli.command()
@port_option
@click.option("--fir-order", "-f", default=2, type=int, show_default=True, help="Order of the FIR filter to apply")
@click.option("--calibrate-zero-bias", "-z", is_flag=True, help="Perform zero bias calibration at startup (tare)")
@coroutine
async def display(port: serial.Serial, fir_order: int, calibrate_zero_bias: bool) -> None:
    """
    Display the force readings from the FMR rig in a human-readable format. This is the main command.
    """

    iom = ForceSensorInterface(port)
    _logger.info("Starting %s", iom)
    loop = asyncio.get_running_loop()
    f_peak = 0.0
    try:
        rd = await fetch(iom, loop)
        forces = compute_forces(rd)
        lpf = MovingAverage(fir_order, forces)
        zero_bias = np.zeros_like(forces)
        if calibrate_zero_bias:
            zero_bias = await do_bias_calibration(iom, loop, forces, 50)
            inform(f"Zero bias: {zero_bias} N")
        while True:
            rd = await fetch(iom, loop)
            forces = lpf(compute_forces(rd) - zero_bias)
            fmt = click.style(f"#{rd.seq_num:06d}: ", dim=True)
            breakdown = "".join(f"{x:+08.1f}" for x in forces)
            f_instant = sum(forces)
            f_peak = f_instant if abs(f_instant) > abs(f_peak) else f_peak
            fmt += click.style(f"F = {f_instant:+08.1f} N", fg="green", bold=True)
            fmt += click.style(f" = {breakdown}", dim=True)
            fmt += click.style(f" F_peak = {f_peak:+08.1f} N", fg="cyan", bold=True)
            inform(f"\r{fmt}  ", nl=False)
    except KeyboardInterrupt:
        pass
    finally:
        iom.close()


@cli.command()
@port_option
@click.option("--nsamples", "-n", default=100, show_default=True, help="Number of samples to average per channel")
@coroutine
async def calibrate(port: serial.Serial, nsamples: int) -> None:
    """
    This command needs to be executed once after the FMR rig is assembled, to calibrate the strain gauges,
    or after a strain gauge is replaced.
    To complete the calibration process, each strain gauge will need to be loaded with known forces,
    so an external force gauge or a known weight is needed.
    """
    if not nsamples > 0:
        raise click.BadParameter("must be positive", param_hint="nsamples")
    loop = asyncio.get_running_loop()

    async def fetch(flush: bool) -> ForceSensorReading:
        if flush:
            await iom.flush()
        rd = await iom.read(deadline=loop.time() + 10.0)
        if rd is None:
            raise RuntimeError("Timed out while waiting for data")
        return rd

    async def calibrate_one(idx: int) -> NDArray[np.float64] | None:
        """Calibrate one channel and return its calibration coefficients."""
        min_samples = 2
        inform(
            f"Calibrating channel #{idx}. "
            f"When prompted, load the corresponding strain gauge with a known force, "
            f"enter the value of the force in newtons (which may be positive or negative), "
            f"and wait for the sample collection process to finish. "
            f"At least {min_samples} such samples are required for calibration; "
            f"one of the samples should be done with no load. "
            f"More samples allow for a more accurate calibration.",
        )
        adc_to_force_samples: list[tuple[float, float]] = []
        while True:
            inform(
                f"Collecting calibration datapoint #{len(adc_to_force_samples)} for channel #{idx}. "
                f"Enter the force acting on this channel in newtons; "
                f"enter nothing to complete calibration of this channel.",
            )
            inp = click.prompt(
                f"Force [newton] or nothing to finish channel #{idx}",
                err=True,
                type=str,
                default="",
                show_default=False,
            ).strip()
            if not inp:
                break
            force = float(inp)
            sigma = 0
            for j in range(nsamples):
                sigma += (await fetch(flush=j == 0)).adc_readings[idx]
                inform(
                    f"\rSample {j + 1} of {nsamples}: ADC {sigma / (j+1):010.0f} -> {force:06.1f} N ",
                    nl=False,
                )
            click.echo()
            adc_to_force_samples.append((sigma / nsamples, force))
        _logger.info(
            "Collected datapoints adc -> force:\n%s",
            "\n".join(f"{adc:09.1f} -> {force:09.1f} N" for adc, force in adc_to_force_samples),
        )
        if len(adc_to_force_samples) < min_samples:
            inform(f"Insufficient datapoints collected for channel #{idx}; calibration not performed.", fg="red")
            return None
        try:
            return np.polyfit(*zip(*adc_to_force_samples), deg=1)  # type: ignore
        except Exception as ex:
            inform(f"Regression failure: {type(ex).__name__}: {ex}", fg="red")
            return None

    iom = ForceSensorInterface(port)
    _logger.info("Starting %s", iom)
    try:
        rd = await fetch(flush=True)
        chan_count = len(rd.adc_readings)
        cal = rd.calibration.copy()  # Make a copy because the source may be non-modifiable.
        inform(f"Original calibration coeffs:\n{cal}", fg="yellow")
        for idx in range(chan_count):
            new = await calibrate_one(idx)
            if new is not None:
                cal[:, idx] = new
            if np.isnan(cal[:, idx]).any():
                cal[:, idx] = 0
                inform(
                    f"Calibration data for channel #{idx} is invalid; replacing the invalid coefficients with zeros",
                    fg="yellow",
                )
            inform(
                f"Calibration of channel #{idx} complete. Channel coeffs: {cal[:, idx]}",
                fg="green" if new is not None else "yellow",
            )

        inform("New calibration coeffs:", fg="green")
        print(cal)
        inform("Writing calibration data, this may take a few seconds...")
        while not await iom.write_calibration(cal):
            inform("Writing calibration data failed, retrying...", fg="red")
        inform("Calibration data written successfully.", fg="green")
    finally:
        iom.close()


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
