import pycyphal
import logging
import asyncio

from typing import Optional
from pathlib import Path
import numpy as np
from numpy.typing import NDArray

from register_proxy import RegisterProxy

from pycyphal.application import Node, make_transport
from pycyphal.application.node_tracker import NodeTracker
from pycyphal.application.register import ValueProxy, Natural16, Natural32
from pycyphal.transport import Transport

# DSDL imports
from uavcan.node import ID_1
from uavcan.node import Mode_1
from uavcan.node import GetInfo_1
from uavcan.node import Heartbeat_1
from uavcan.node import ExecuteCommand_1
from uavcan.pnp import NodeIDAllocationData_1
from uavcan.primitive.array import Integer32_1

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
_logger = logging.getLogger(__name__)

DEFAULT_CONTROLLER_NODE_ID = 1
DEFAULT_TARGET_NODE_ID = 125


class CyphalContext:
    def __init__(self) -> None:
        # ControllerNode-related
        self._transport: Optional[Transport] = None
        self._controller_node: Optional[Node] = None
        self._node_tracker: Optional[NodeTracker] = None

        # TargetNode-related
        # self._target_node: Optional[Node] = None
        self._register_proxy: Optional[RegisterProxy] = None

    async def wait_for_node_online(self) -> None:
        fluxgrip_found = False
        while not fluxgrip_found:
            for node_id, node in self._node_tracker.registry.items():
                node_info = node.info
                if node_info and node_info.name.tobytes().decode() == "com.zubax.fluxgrip":
                    assert node_id == DEFAULT_TARGET_NODE_ID, "Expects FluxGrip to have DEFAULT_TARGET_NODE_ID"
                    _logger.info("FluxGrip found!")
                    fluxgrip_found = True
                else:
                    await asyncio.sleep(1)
                    _logger.info("Waiting for FluxGrip to come online...")
            await asyncio.sleep(1)
            _logger.info(f"Online nodes: {len(self._node_tracker.registry.items())}")

    async def start(self) -> None:
        available_canfaces = list(Path("/dev/serial/by-id").glob("usb-*Zubax*Babel*"))
        sorted_canfaces = sorted(available_canfaces, key=lambda p: str(p))
        bitrate = 1_000_000
        use_secondary_interface = 0
        reg = {
            # transport-related
            "uavcan.can.iface": ValueProxy("slcan:" + str(sorted_canfaces[int(use_secondary_interface)])),
            "uavcan.can.bitrate": ValueProxy(Natural32([bitrate, bitrate])),
            "uavcan.can.mtu": ValueProxy(Natural16([8])),
            # node-related
            "uavcan.node.id": ValueProxy(Natural16([DEFAULT_CONTROLLER_NODE_ID])),
        }
        self._transport = make_transport(reg)
        _logger.info(f"Transport configured: {self._transport}")

        self._controller_node = pycyphal.application.make_node(
            info=GetInfo_1.Response(name="org.opencyphal.controller.node"),
            transport=self._transport,
            reconfigurable_transport=True,
        )
        self._node_tracker = pycyphal.application.node_tracker.NodeTracker(self._controller_node)
        self._node_tracker.get_info_timeout = 1.0
        self._controller_node.start()
        _logger.info(f"Controller node started: {self._controller_node}")

        await self.wait_for_node_online()

        self._register_proxy = RegisterProxy(self._controller_node, DEFAULT_TARGET_NODE_ID)
        await self._register_proxy.reload()

    def close(self) -> None:
        self._controller_node.close()
        _logger.info(f"Controller node closed!")

    async def configure_demag_cycle(self, demag_val: Integer32_1):
        assert len(demag_val.value) == 51
        _logger.info(f"Setting new demag cycle values: {demag_val}")
        res = int(await self._register_proxy.write_through("magnet.demag", demag_val))
        _logger.info(f"Read back demag values: {res}")
        _logger.info("Rebooting FluxGrip")
        cln_exe = self._controller_node.make_client(ExecuteCommand_1, DEFAULT_TARGET_NODE_ID)
        resp, _ = await cln_exe.call(ExecuteCommand_1.Request(command=ExecuteCommand_1.Request.COMMAND_RESTART))
        assert isinstance(resp, ExecuteCommand_1.Response)
        assert resp.status == ExecuteCommand_1.Response.STATUS_SUCCESS
        await asyncio.sleep(5)  # Give some time for reboot to start
        await self.wait_for_node_online()
