from __future__ import annotations

import re
import pycyphal
import logging
import asyncio
import collections.abc

from typing import Optional
from pathlib import Path
from typing import Iterator, Awaitable

from pycyphal.application import Node, make_transport
from pycyphal.application.node_tracker import NodeTracker
from pycyphal.application.register import ValueProxy, ValueProxyWithFlags, Natural16, Natural32
from pycyphal.transport import Transport
from pycyphal.presentation import Publisher, Subscriber

# DSDL imports
from uavcan.node import GetInfo_1
from uavcan.node import ExecuteCommand_1
from uavcan.primitive.array import Integer32_1
from uavcan.primitive.scalar import Integer8_1
from uavcan.register import Access_1, List_1, Name_1
from zubax.fluxgrip import Feedback_0

_logger = logging.getLogger(__name__)


class RegisterProxy(collections.abc.Mapping[str, pycyphal.application.register.ValueProxyWithFlags]):
    _VALID_PAT = re.compile(r"^[a-z_]+(\.\w+)+[<=>]?$")

    def __init__(self, local_node: pycyphal.application.Node, remote_node_id: int):
        self._cache: dict[str, ValueProxyWithFlags] = {}
        self._acc = local_node.make_client(Access_1, remote_node_id)
        self._list = local_node.make_client(List_1, remote_node_id)

    @property
    def remote_node_id(self) -> int:
        return int(self._acc.output_transport_session.destination_node_id)

    @staticmethod
    async def new(local_node: pycyphal.application.Node, remote_node_id: int) -> RegisterProxy:
        rp = RegisterProxy(local_node, remote_node_id)
        await rp.reload()
        return rp

    async def reload(self) -> None:
        """
        Invalidate the local cache and fetch all registers from the remote node anew.
        This is basically equivalent to creating a new instance.
        """
        _logger.debug("{!r}: Reloading", self)
        names: set[str] = set()
        for idx in range(2**16):
            req = List_1.Request(idx)
            resp = await self._list(req)
            if not resp:
                raise pycyphal.application.NetworkTimeoutError(f"Register list has timed out: {req}")
            assert isinstance(resp, List_1.Response)
            if name := resp.name.name.tobytes().decode():
                assert self._VALID_PAT.match(name), f"Invalid register name: {name!r}"
                names.add(name)
            else:
                break
        self._cache.clear()
        for n in names:
            await self.read_through(n)
        _logger.debug("Fetched registers:")
        for key, value in self._cache.items():
            _logger.debug(f"{key} -> {repr(value)}")
        assert names == set(self._cache.keys())

    async def read_through(self, name: str) -> ValueProxyWithFlags:
        """
        Fetch the register from the remote node and update the local cache.
        Returns empty if no such register, in which case the cache is not altered.
        Raises pycyphal.application.NetworkTimeoutError on timeout.
        """
        return await self.write_through(name, pycyphal.application.register.Value())

    async def write_through(self, name: str, value: pycyphal.application.register.RelaxedValue) -> ValueProxyWithFlags:
        """
        Write the specified register on the remote node, update the cache with the response.
        Returns empty if no such register, in which case the cache is not altered.
        Raises pycyphal.application.NetworkTimeoutError on timeout.
        If the value is empty, this is equivalent to a read operation.
        """
        if name in self._cache:
            _logger.debug("{!r}: {!r} was found in cache!", self, name)
            v_with_flags = self._cache[name]  # copy is necessary to use correct type stored in value!
            v_with_flags.assign(value)  # update the value
            v = v_with_flags.value
        else:
            _logger.debug("{!r}: {!r} was not found in cache!", self, name)
            v = pycyphal.application.register.ValueProxy(value).value
            _logger.debug("{!r}: type of value: {!r}", self, type(v))
        req = Access_1.Request(name=Name_1(name), value=v)
        resp = await self._acc(req)
        if not resp:
            await asyncio.sleep(10)
            resp = await self._acc(req)
            if not resp:
                raise pycyphal.application.NetworkTimeoutError(f"Register access has timed out: {req}")
        assert isinstance(resp, Access_1.Response)
        res = ValueProxyWithFlags(resp.value, resp.mutable, resp.persistent)
        if not res.value.empty:
            self._cache[name] = res
            _logger.debug("{!r}: {!r} <- {!r}", self, name, res)
        return res

    def __getitem__(self, k: str) -> ValueProxyWithFlags:
        """Obtain item from the cache (does not access the network). The value may be obsolete."""
        return self._cache[k]

    def __len__(self) -> int:
        return len(self._cache)

    def __iter__(self) -> Iterator[str]:
        """Iterate the cached values, which may be obsolete (does not access the network)."""
        return iter(self._cache)

    def __repr__(self) -> str:
        return str(
            pycyphal.util.repr_attributes(
                self, remote_node_id=self._acc.output_transport_session.destination_node_id, size=len(self._cache)
            )
        )

    def __str__(self) -> str:
        if len(self) == 0:
            return ""
        max_name_len = max(map(len, self.keys()))
        return "\n".join(
            "\t".join(
                [
                    k.ljust(max_name_len),
                    ("immutab", "mutable")[v.mutable],
                    ("volatil", "persist")[v.persistent],
                    str(pycyphal.dsdl.to_builtin(v.value)),
                ]
            )
            for k, v in sorted(self.items())
        )


DEFAULT_CONTROLLER_NODE_ID = 1
DEFAULT_TARGET_NODE_ID = 125
EXPECTED_COMMAND_TOPIC_ID = 1000
EXPECTED_FEEDBACK_TOPIC_ID = 1001


class FluxGripConfig:
    def __init__(self) -> None:
        # ControllerNode-related
        self._transport: Optional[Transport] = None
        self._controller_node: Optional[Node] = None
        self._node_tracker: Optional[NodeTracker] = None
        self._pub_command: Optional[Publisher] = None
        self._sub_feedback: Optional[Subscriber] = None

        # TargetNode-related
        self._register_proxy: Optional[RegisterProxy] = None

    async def wait_for_node_online(self) -> None:
        fluxgrip_found = False
        while not fluxgrip_found:
            for node_id, node in self._node_tracker.registry.items():
                node_info = node.info
                if node_info and node_info.name.tobytes().decode() == "com.zubax.fluxgrip":
                    assert node_id == DEFAULT_TARGET_NODE_ID, "Expects FluxGrip to have DEFAULT_TARGET_NODE_ID"
                    _logger.debug("FluxGrip found!")
                    fluxgrip_found = True
                else:
                    await asyncio.sleep(1)
                    _logger.debug("Waiting for FluxGrip to come online...")
            await asyncio.sleep(1)
            _logger.debug(f"Online nodes: {len(self._node_tracker.registry.items())}")

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
        _logger.debug(f"Transport configured: {self._transport}")

        self._controller_node = pycyphal.application.make_node(
            info=GetInfo_1.Response(name="org.opencyphal.controller.node"),
            transport=self._transport,
            reconfigurable_transport=True,
        )
        self._node_tracker = pycyphal.application.node_tracker.NodeTracker(self._controller_node)
        self._node_tracker.get_info_timeout = 1.0
        self._controller_node.start()
        _logger.debug(f"Controller node started: {self._controller_node}")

        await self.wait_for_node_online()

        self._register_proxy = RegisterProxy(self._controller_node, DEFAULT_TARGET_NODE_ID)
        await self._register_proxy.reload()

        _logger.debug("Setting up Command publisher")
        self._pub_command = self._controller_node.make_publisher(Integer8_1, EXPECTED_COMMAND_TOPIC_ID)

        _logger.debug("Setting up Feedback subscriber")
        self._sub_feedback = self._controller_node.make_subscriber(Feedback_0, EXPECTED_FEEDBACK_TOPIC_ID)

    def close(self) -> None:
        self._controller_node.close()
        _logger.debug(f"Controller node closed!")

    async def configure_demag_cycle(self, demag_val: Integer32_1):
        assert len(demag_val.value) == 51
        _logger.debug(f"Setting new demag cycle values: {demag_val}")
        res = int(await self._register_proxy.write_through("magnet.demag", demag_val))
        _logger.debug(f"Read back demag values: {res}")
        _logger.debug("Rebooting FluxGrip")
        cln_exe = self._controller_node.make_client(ExecuteCommand_1, DEFAULT_TARGET_NODE_ID)
        resp, _ = await cln_exe.call(ExecuteCommand_1.Request(command=ExecuteCommand_1.Request.COMMAND_RESTART))
        assert isinstance(resp, ExecuteCommand_1.Response)
        assert resp.status == ExecuteCommand_1.Response.STATUS_SUCCESS
        await asyncio.sleep(5)  # Give some time for reboot to start
        await self.wait_for_node_online()

    async def magnetize(self) -> None:
        while await self._sub_feedback.get(0):
            pass
        feedback_msg = await self._sub_feedback.get(20)
        assert feedback_msg is not None
        assert feedback_msg.magnetized == False

        assert await self._pub_command.publish(Integer8_1(value=1))

        async def wait_for_magnet_to_magnetize() -> None:
            feedback_msg = await self._sub_feedback.get(5)
            while not feedback_msg.magnetized or not feedback_msg.remagnetization_state == 0:
                _logger.debug("Waiting for magnet to magnetize")
                feedback_msg = await self._sub_feedback.get(5)
                while await self._sub_feedback.get(0):
                    pass

        try:
            await asyncio.wait_for(wait_for_magnet_to_magnetize(), timeout=10)
            _logger.debug("Magnetized successfully")
        except asyncio.TimeoutError:
            raise TimeoutError("Timeout while waiting for magnet to magnetize")

    async def demagnetize(self) -> None:
        while await self._sub_feedback.get(0):
            pass
        feedback_msg = await self._sub_feedback.get(20)
        assert feedback_msg is not None
        assert feedback_msg.magnetized == True

        assert await self._pub_command.publish(Integer8_1(value=0))

        async def wait_for_magnet_to_demagnetize() -> None:
            feedback_msg = await self._sub_feedback.get(5)
            while feedback_msg.magnetized or not feedback_msg.remagnetization_state == 0:
                _logger.debug("Waiting for magnet to demagnetize")
                feedback_msg = await self._sub_feedback.get(5)
                while await self._sub_feedback.get(0):
                    pass

        try:
            await asyncio.wait_for(wait_for_magnet_to_demagnetize(), timeout=60)
            _logger.debug("Demagnetized successfully")
        except asyncio.TimeoutError:
            raise TimeoutError("Timeout while waiting for magnet to demagnetize")
