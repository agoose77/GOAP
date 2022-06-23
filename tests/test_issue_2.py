from goap.action import Action
from goap.planner import RegressivePlanner
from util import plan_as_contents


class BecomeTwo(Action):
    effects = {"age": 2}
    preconditions = {"age": 1}


def test():
    world_state = {"age": 1}
    goal_state = {"age": 2}

    actions = [BecomeTwo()]
    planner = RegressivePlanner(world_state, actions)
    print("Initial State:", world_state)

    plan = planner.find_plan(goal_state)
    assert plan_as_contents(plan) == [(BecomeTwo, {})], plan
