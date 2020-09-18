from typing import Callable
from typing import Optional
from typing import Type
from pathlib import Path
import aioros

from .client import ActionClient


class NodeHandle(aioros.NodeHandle):

    async def create_action_client(
        self,
        ns: str,
        action
    ) -> ActionClient:
        action_client = ActionClient(ns, action)
        await action_client.init(self)
        return action_client


def run_forever(
    func: Callable[[NodeHandle], int],
    node_name: str,
    *,
    loop=None,
    xmlrpc_port: int = 0,
    tcpros_port: int = 0,
    unixros_path: Optional[Path] = None,
    node_handle_cls: Type[aioros.NodeHandle] = NodeHandle,
) -> None:
    return aioros.run_forever(
        func,
        node_name,
        loop=loop,
        xmlrpc_port=xmlrpc_port,
        tcpros_port=tcpros_port,
        unixros_path=unixros_path,
        node_handle_cls=node_handle_cls)


def run_until_complete(
    func: Callable[[NodeHandle], int],
    node_name: str,
    *,
    loop=None,
    xmlrpc_port: int = 0,
    tcpros_port: int = 0,
    unixros_path: Optional[Path] = None,
    node_handle_cls: Type[aioros.NodeHandle] = NodeHandle,
) -> None:
    return aioros.run_until_complete(
        func,
        node_name,
        loop=loop,
        xmlrpc_port=xmlrpc_port,
        tcpros_port=tcpros_port,
        unixros_path=unixros_path,
        node_handle_cls=node_handle_cls)
