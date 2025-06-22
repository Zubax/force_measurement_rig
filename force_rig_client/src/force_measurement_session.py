import time
import numpy as np
import click
import emoji
import asyncio

from force_rig import ForceRig
from fluxgrip_config import FluxGripConfig
from serial import Serial
from client_utils import inform
from uavcan.primitive.array import Integer32_1
from matplotlib import pyplot

class ForceMeasurementSession:
    def __init__(self, force_port: Serial, drive_port: Serial):
        self._force_rig = ForceRig(drive_port, force_port)
        self._fluxgrip_config = FluxGripConfig()
        self._t_current: float = 0
        self._test_index: int = 0

    async def setup(self):
        await self._force_rig.setup()
        await self._fluxgrip_config.start()

    async def cleanup(self):
        if self._t_current > 0:
            await self._force_rig.move_arm_up_for(self._t_current)
        await self._force_rig.stop_arm()
        await self._force_rig.close()
        self._fluxgrip_config.close()

    async def run_cycle(self, demag_values):
        new_demag_values = Integer32_1(np.array(
            demag_values,
            dtype=np.int32,
        ))
        inform(f"Testing demag values: {demag_values}")
        await self._fluxgrip_config.configure_demag_cycle(new_demag_values)

        inform("Moving arm downwards until TOUCH_FORCE is detected")

        self._t_current = 0 # We assume we're starting from top position
        TOUCH_FORCE = -1.0 # Once pressure sensor detect 1N, we can assume the plate has touched the magnet
        start_time_down = time.time()
        await self._force_rig.move_arm_down()
        counter = 0
        while True:
            f_instant = await self._force_rig.get_instant_force()
            fmt = click.style(f"#{counter:06d}: ", dim=True)
            fmt += click.style(f"F_instant = {f_instant:+08.1f} N", fg="cyan", bold=True)
            inform(f"\r{fmt}", nl=False)
            if f_instant < TOUCH_FORCE:
                break
            counter +=1

        await self._force_rig.move_arm_down_for(10.0) # To be sure the plate is completely flat on the magnet
        self._t_current = time.time() - start_time_down

        # Magnetize and demagnetize
        inform("\nMagnetizing and demagnetizing")
        await self._fluxgrip_config.magnetize()
        await asyncio.sleep(1)
        await self._fluxgrip_config.demagnetize()
        await asyncio.sleep(1)

        # Move arm up and print the force sensor data
        # 2 stop conditions:
        # 1. if t_current risks becoming negative (we've reached the top)
        # 2. if plate has detached (Force drops by DELTA_THRESHOLD)
        DELTA_THRESHOLD = 0.5
        start_time_up = time.time()
        await self._force_rig.move_arm_up()
        counter = 0
        f_peak = 0.0
        f_instant_storage = []
        plate_detached = False
        data_timeout = time.time()
        while True:
            f_instant = await self._force_rig.get_instant_force()
            f_instant_storage.append(f_instant)
            fmt = click.style(f"#{counter:06d}: ", dim=True)
            f_peak = f_instant if f_instant > f_peak else f_peak
            fmt += click.style(f"f_instant = {f_instant:+08.1f} N", fg="green", bold=True)
            fmt += click.style(f" f_peak = {f_peak:+08.1f} N", fg="cyan", bold=True)
            inform(f"\r{fmt}  ", nl=False)
            counter +=1
            if time.time() - start_time_up > self._t_current:
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

        await self._force_rig.stop_arm()
        total_time_up = time.time() - start_time_up
        self._t_current -= total_time_up

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
        demag_text =  "Demag values: " + ', '.join(map(str, demag_values))
        result_text = f"\nF_peak: {f_peak:.2f} N"
        fig.text(0.5, 0.01, demag_text+result_text, ha='center', va='bottom', fontsize=8, wrap=True)

        pyplot.tight_layout(rect=[0, 0.06, 1, 1])  # leave space for the text
        pyplot.savefig(f"result_{self._test_index}", format="png")
