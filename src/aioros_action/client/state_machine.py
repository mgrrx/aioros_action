from asyncio import Event
from enum import IntEnum
from enum import unique
from typing import Callable
from typing import Optional

from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus as GoalStatusMsg
from actionlib_msgs.msg import GoalStatusArray


@unique
class CommState(IntEnum):
    WAITING_FOR_GOAL_ACK = 0
    PENDING = 1
    ACTIVE = 2
    WAITING_FOR_RESULT = 3
    WAITING_FOR_CANCEL_ACK = 4
    RECALLING = 5
    PREEMPTING = 6
    DONE = 7
    LOST = 8

    @property
    def terminal(self):
        return frozenset((
            self.DONE,
            self.LOST))

    @property
    def is_terminal(self):
        return self in self.terminal


@unique
class GoalStatus(IntEnum):
    PENDING = GoalStatusMsg.PENDING
    ACTIVE = GoalStatusMsg.ACTIVE
    PREEMPTED = GoalStatusMsg.PREEMPTED
    SUCCEEDED = GoalStatusMsg.SUCCEEDED
    ABORTED = GoalStatusMsg.ABORTED
    REJECTED = GoalStatusMsg.REJECTED
    PREEMPTING = GoalStatusMsg.PREEMPTING
    RECALLING = GoalStatusMsg.RECALLING
    RECALLED = GoalStatusMsg.RECALLED
    LOST = GoalStatusMsg.LOST

    @property
    def terminal(self):
        return frozenset((
            self.RECALLED,
            self.REJECTED,
            self.PREEMPTED,
            self.ABORTED,
            self.SUCCEEDED,
            self.LOST))

    @property
    def is_terminal(self):
        return self in self.terminal


class CommStateMachine:

    def __init__(
        self,
        goal_id: GoalID,
        *,
        on_done_callback: Optional[Callable] = None,
        on_feedback_callback: Optional[Callable] = None
    ) -> None:
        self._result = None
        self._result_event = Event()
        self._goal_id: GoalID = goal_id
        self._goal_status: GoalStatus = GoalStatus.PENDING
        self._state: CommState = CommState.WAITING_FOR_GOAL_ACK
        self._on_done_callback = on_done_callback
        self._on_feedback_callback = on_feedback_callback

    @property
    def state(self) -> CommState:
        return self._state

    @state.setter
    def state(self, new_status: CommState) -> None:
        self._state = new_status

    @property
    def goal_id(self) -> GoalID:
        return self._goal_id

    @property
    def goal_status(self) -> GoalStatus:
        return self._goal_status

    @goal_status.setter
    def goal_status(self, new_status: GoalStatus) -> None:
        self._goal_status = new_status

    async def wait_for_result(self):
        await self._result_event.wait()
        return self._result

    def update_status(self, status_array: GoalStatusArray) -> None:
        try:
            self.goal_status = GoalStatus(next(
                status.status
                for status in status_array.status_list
                if status.goal_id.id == self._goal_id.id))
        except ValueError:
            return
        except StopIteration:
            if self.state not in (CommState.WAITING_FOR_GOAL_ACK,
                                  CommState.WAITING_FOR_RESULT,
                                  CommState.DONE):
                self.goal_status = GoalStatus.LOST
                self.state = CommState.DONE
            return

        for state in TRANSITIONS[self.state][self.goal_status]:
            try:
                self.comm_state = state
            except InvalidTransitionError:
                pass

    def update_result(self, action_result) -> None:
        self.goal_status = GoalStatus(action_result.status.status)
        if not self.state.is_terminal:
            self.update_status(
                GoalStatusArray(status_list=[action_result.status]))
            self._result = action_result.result
            self._result_event.set()
            self.state = CommState.DONE
            if self._on_done_callback:
                self._on_done_callback(action_result.result)

    def update_feedback(self, action_feedback) -> None:
        if self._on_feedback_callback and self.state != CommState.DONE:
            self._on_feedback_callback(action_feedback.feedback)


class InvalidTransitionError(Exception):
    pass


class InvalidTransition(tuple):

    def __iter__(self):
        raise InvalidTransitionError()


TRANSITIONS = {
    CommState.WAITING_FOR_GOAL_ACK: {
        GoalStatus.PENDING:    (CommState.PENDING, ),
        GoalStatus.ACTIVE:     (CommState.ACTIVE, ),
        GoalStatus.REJECTED:   (CommState.PENDING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.RECALLING:  (CommState.PENDING,
                                CommState.RECALLING),
        GoalStatus.RECALLED:   (CommState.PENDING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTED:  (CommState.ACTIVE,
                                CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.ACTIVE,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.ACTIVE,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: (CommState.ACTIVE,
                                CommState.PREEMPTING)},
    CommState.PENDING: {
        GoalStatus.PENDING:    (),
        GoalStatus.ACTIVE:     (CommState.ACTIVE, ),
        GoalStatus.REJECTED:   (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.RECALLING:  (CommState.RECALLING, ),
        GoalStatus.RECALLED:   (CommState.RECALLING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTED:  (CommState.ACTIVE,
                                CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.ACTIVE,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.ACTIVE,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: (CommState.ACTIVE,
                                CommState.PREEMPTING)},
    CommState.ACTIVE: {
        GoalStatus.PENDING:    InvalidTransition(),
        GoalStatus.ACTIVE:     (),
        GoalStatus.REJECTED:   InvalidTransition(),
        GoalStatus.RECALLING:  InvalidTransition(),
        GoalStatus.RECALLED:   InvalidTransition(),
        GoalStatus.PREEMPTED:  (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.ABORTED:    (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.PREEMPTING: (CommState.PREEMPTING, )},
    CommState.WAITING_FOR_RESULT: {
        GoalStatus.PENDING:    InvalidTransition(),
        GoalStatus.ACTIVE:     (),
        GoalStatus.REJECTED:   (),
        GoalStatus.RECALLING:  InvalidTransition(),
        GoalStatus.RECALLED:   (),
        GoalStatus.PREEMPTED:  (),
        GoalStatus.SUCCEEDED:  (),
        GoalStatus.ABORTED:    (),
        GoalStatus.PREEMPTING: InvalidTransition()},
    CommState.WAITING_FOR_CANCEL_ACK: {
        GoalStatus.PENDING:    (),
        GoalStatus.ACTIVE:     (),
        GoalStatus.REJECTED:   (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.RECALLING:  (CommState.RECALLING, ),
        GoalStatus.RECALLED:   (CommState.RECALLING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTED:  (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: (CommState.PREEMPTING, )},
    CommState.RECALLING: {
        GoalStatus.PENDING:    InvalidTransition(),
        GoalStatus.ACTIVE:     InvalidTransition(),
        GoalStatus.REJECTED:   (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.RECALLING:  (),
        GoalStatus.RECALLED:   (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.PREEMPTED:  (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.SUCCEEDED:  (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.ABORTED:    (CommState.PREEMPTING,
                                CommState.WAITING_FOR_RESULT),
        GoalStatus.PREEMPTING: (CommState.PREEMPTING, )},
    CommState.PREEMPTING: {
        GoalStatus.PENDING:    InvalidTransition(),
        GoalStatus.ACTIVE:     InvalidTransition(),
        GoalStatus.REJECTED:   InvalidTransition(),
        GoalStatus.RECALLING:  InvalidTransition(),
        GoalStatus.RECALLED:   InvalidTransition(),
        GoalStatus.PREEMPTED:  (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.SUCCEEDED:  (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.ABORTED:    (CommState.WAITING_FOR_RESULT, ),
        GoalStatus.PREEMPTING: ()},
    CommState.DONE: {
        GoalStatus.PENDING:    InvalidTransition(),
        GoalStatus.ACTIVE:     InvalidTransition(),
        GoalStatus.REJECTED:   (),
        GoalStatus.RECALLING:  InvalidTransition(),
        GoalStatus.RECALLED:   (),
        GoalStatus.PREEMPTED:  (),
        GoalStatus.SUCCEEDED:  (),
        GoalStatus.ABORTED:    (),
        GoalStatus.PREEMPTING: InvalidTransition()}}
