from typing import Callable

from actionlib_msgs.msg import GoalID

from .state_machine import CommState
from .state_machine import CommStateMachine
from .state_machine import GoalStatus


class GoalHandle:

    def __init__(
        self,
        state_machine: CommStateMachine,
        on_cancel_callback: Callable[['GoalHandle'], bool]
    ) -> None:
        self._sm = state_machine
        self._on_cancel_callback = on_cancel_callback

    @property
    def goal_id(self) -> GoalID:
        return self._sm.goal_id

    @property
    def goal_status(self) -> GoalStatus:
        return self._sm.goal_status

    @property
    def comm_state(self) -> CommState:
        return self._sm.state

    async def cancel(self) -> bool:
        return await self._on_cancel_callback(self)

    async def wait_for_result(self):
        return await self._sm.wait_for_result()
