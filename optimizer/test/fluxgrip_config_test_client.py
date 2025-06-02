import asyncio
import logging
import sys
import os

# Add the src directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from fluxgrip_config import FluxGripConfig
import numpy as np

from uavcan.primitive.array import Integer32_1

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")

# TODO BEFORE RUNNING TEST:
# export CYPHAL_PATH="$HOME/force_measurement_rig/optimizer/lib/public_regulated_data_types:$HOME/force_measurement_rig/optimizer/lib/zubax_dsdl"
# export PYCYPHAL_PATH="$HOME/force_measurement_rig/optimizer/dsdl_types"


async def main() -> None:
    fluxgrip_config = FluxGripConfig()
    await fluxgrip_config.start()
    # fmt: off
    new_demag_values = np.array(
        [
            -100, -90, -81, +73, +66, -59, -53, +48, -43, +39, -35, +31, -28, +25, -23, +21, -19, +17, -15, +14, -12,
            +11, -10, +9, -8, +7, -6, +6, -5, +5, -4, +4, -3, +3, -3, +3, -2, +2, -2, +2, -1, +1, -1, +1, -1, +1, -1,
            +1, -1, -1, -1,
        ],
        dtype=np.int32,
    )
    # fmt: off
    new_demag_val = Integer32_1(new_demag_values)
    await fluxgrip_config.configure_demag_cycle(new_demag_val)
    await asyncio.sleep(5)
    await fluxgrip_config.magnetize()
    await asyncio.sleep(5)
    await fluxgrip_config.demagnetize()
    try:
        await asyncio.Event().wait()  # Keeps the event loop running forever
    except KeyboardInterrupt:
        pass
    finally:
        fluxgrip_config.close()


if __name__ == "__main__":
    asyncio.run(main())
