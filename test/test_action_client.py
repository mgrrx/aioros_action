#!/usr/bin/env python3

import asyncio
import aioros_action

from actionlib_tutorials.msg import FibonacciAction
from actionlib_tutorials.msg import FibonacciGoal


async def main(nh: aioros_action.NodeHandle):
    ac = await nh.create_action_client(
        'fibonacci',
        FibonacciAction)
    await asyncio.wait_for(ac.wait_for_server(), timeout=3.0)
    gh = ac.send_goal(
        FibonacciGoal(order=5),
        on_done_callback=lambda msg: print("done cb:", msg),
        on_feedback_callback=lambda msg: print("feedback cb:", msg))
    print(await gh.wait_for_result())


if __name__ == "__main__":
    aioros_action.run_until_complete(main, 'test_action_client')
