from __future__ import annotations

import re
import pycyphal
import collections.abc
import asyncio
import logging

from typing import Iterator
from pycyphal.application.register import ValueProxyWithFlags

# DSDL imports
from uavcan.register import Access_1, List_1, Name_1

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(process)07d %(levelname)-3.3s %(name)s: %(message)s")
logger = logging.getLogger(__name__)


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
        logger.debug("{!r}: Reloading", self)
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
        # logger.info(f"{self}: Fetched names: {names}")
        self._cache.clear()
        for n in names:
            await self.read_through(n)
        logger.info("Fetched registers:")
        for key, value in self._cache.items():
            logger.info(f"{key} -> {repr(value)}")
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
            logger.debug("{!r}: {!r} was found in cache!", self, name)
            v_with_flags = self._cache[name]  # copy is necessary to use correct type stored in value!
            v_with_flags.assign(value)  # update the value
            v = v_with_flags.value
        else:
            logger.debug("{!r}: {!r} was not found in cache!", self, name)
            v = pycyphal.application.register.ValueProxy(value).value
            logger.debug("{!r}: type of value: {!r}", self, type(v))
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
            logger.debug("{!r}: {!r} <- {!r}", self, name, res)
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
