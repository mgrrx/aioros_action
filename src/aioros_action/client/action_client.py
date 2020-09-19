from asyncio import gather
from typing import Callable
from typing import Dict
from typing import NamedTuple
from typing import Optional
from uuid import uuid4

from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Header

from aioros import NodeHandle
from aioros import Publisher
from aioros import Subscription

from .goal_handle import GoalHandle
from .state_machine import CommStateMachine


class GoalInfo(NamedTuple):
    handle: GoalHandle
    state_machine: CommStateMachine


class ActionClient:

    def __init__(self, ns: str, action):
        self.ns = ns
        ac = action()
        self._action_goal = type(ac.action_goal)
        self._action_result = type(ac.action_result)
        self._action_feedback = type(ac.action_feedback)
        self._node_handle: Optional[NodeHandle] = None
        self._pub_goal: Optional[Publisher] = None
        self._pub_cancel: Optional[Publisher] = None
        self._sub_status: Optional[Subscription] = None
        self._sub_result: Optional[Subscription] = None
        self._sub_feedback: Optional[Subscription] = None
        self._goals: Dict[str, GoalInfo] = {}

    async def init(self, node_handle: NodeHandle) -> None:
        self._node_handle = node_handle
        self._pub_goal = await node_handle.create_publisher(
            f'{self.ns}/goal',
            self._action_goal)
        self._pub_cancel = await node_handle.create_publisher(
            f'{self.ns}/cancel',
            GoalID)
        self._sub_status = await node_handle.create_subscription(
            f'{self.ns}/status',
            GoalStatusArray,
            self._on_status)
        self._sub_result = await node_handle.create_subscription(
            f'{self.ns}/result',
            self._action_result,
            self._on_result)
        self._sub_feedback = await node_handle.create_subscription(
            f'{self.ns}/feedback',
            self._action_feedback,
            self._on_feedback)

    async def close(self):
        for goal_info in self._goals.values():
            await goal_info.handle.cancel()
        self._goals.clear()

        if self._pub_goal:
            await self._pub_goal.close()
            self._pub_goal = None

        if self._pub_cancel:
            await self._pub_cancel.close()
            self._pub_cancel = None

        if self._sub_status:
            await self._sub_status.close()
            self._sub_status = None

        if self._sub_result:
            await self._sub_result.close()
            self._sub_result = None

        if self._sub_feedback:
            await self._sub_feedback.close()
            self._sub_feedback = None

        self._node_handle = None

    async def wait_for_server(self):
        await gather(self._pub_goal.wait_for_subscribers(),
                     self._pub_cancel.wait_for_subscribers(),
                     self._sub_feedback.wait_for_publishers(),
                     self._sub_result.wait_for_publishers(),
                     self._sub_status.wait_for_publishers())

    def send_goal(
        self,
        goal,
        *,
        on_done_callback: Optional[Callable] = None,
        on_feedback_callback: Optional[Callable] = None
    ) -> GoalHandle:
        action_goal = self._action_goal(
            header=Header(stamp=self._node_handle.get_time()),
            goal_id=GoalID(id=str(uuid4())),
            goal=goal)
        self._pub_goal.publish(action_goal)

        state_machine = CommStateMachine(
            action_goal.goal_id,
            on_done_callback=on_done_callback,
            on_feedback_callback=on_feedback_callback)
        goal_handle = GoalHandle(
            state_machine,
            self.cancel_goal)
        self._goals[action_goal.goal_id.id] = GoalInfo(
            goal_handle,
            state_machine)
        return goal_handle

    async def cancel_goal(self, goal_handle) -> bool:
        if goal_handle.goal_id in self._goals:
            await self._pub_cancel.publish(goal_handle.goal_id)
            return True
        return False

    async def cancel_all_goals(self):
        await self._pub_cancel.publish(GoalID(id=""))

    async def _on_status(self, msg: GoalStatusArray):
        for goal_info in self._goals.values():
            goal_info.state_machine.update_status(msg)

    async def _on_feedback(self, msg):
        goal_info = self._goals.get(msg.status.goal_id.id)
        if goal_info:
            goal_info.state_machine.update_feedback(msg)

    async def _on_result(self, msg):
        goal_info = self._goals.get(msg.status.goal_id.id)
        if goal_info:
            goal_info.state_machine.update_result(msg)


async def create_client(
    node_handle: NodeHandle,
    ns: str,
    action
) -> ActionClient:
    action_client = ActionClient(ns, action)
    await action_client.init(node_handle)
    return action_client
