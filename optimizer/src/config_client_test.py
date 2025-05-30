import asyncio
from config_client import CyphalContext
import numpy as np

from uavcan.primitive.array import Integer32_1


async def main() -> None:
    cyphal_context = CyphalContext()
    await cyphal_context.start()
    new_demag_values = np.array(
        [
            -100,
            -90,
            -81,
            +73,
            +66,
            -59,
            -53,
            +48,
            -43,
            +39,
            -35,
            +31,
            -28,
            +25,
            -23,
            +21,
            -19,
            +17,
            -15,
            +14,
            -12,
            +11,
            -10,
            +9,
            -8,
            +7,
            -6,
            +6,
            -5,
            +5,
            -4,
            +4,
            -3,
            +3,
            -3,
            +3,
            -2,
            +2,
            -2,
            +2,
            -1,
            +1,
            -1,
            +1,
            -1,
            +1,
            -1,
            +1,
            -1,
            -1,
            -1,
        ],
        dtype=np.int32,
    )
    new_demag_val = Integer32_1(new_demag_values)
    await cyphal_context.configure_demag_cycle(new_demag_val)
    try:
        await asyncio.Event().wait()  # Keeps the event loop running forever
    except KeyboardInterrupt:
        pass
    finally:
        cyphal_context.close()


if __name__ == "__main__":
    asyncio.run(main())
