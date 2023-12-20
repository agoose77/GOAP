from goap.action import Action
from goap.planner import RegressivePlanner
from util import plan_as_contents


class Action1(Action):
    effects = {"FIRST": True}
    preconditions = {}


class Action2(Action):
    effects = {"SECOND": True}
    preconditions = {"FIRST": True}


class Action3(Action):
    effects = {"THIRD": True}
    preconditions = {"FIRST": True, "SECOND": True}


def test():
    world_state = {"FIRST": False, "SECOND": False, "THIRD": False, "FOURTH": False}
    goal_state = {"THIRD": True}

    actions = [Action1(), Action2(), Action3()]
    planner = RegressivePlanner(world_state, actions)

    plan = planner.find_plan(goal_state)
    assert plan_as_contents(plan) == [(Action1, {}), (Action2, {}), (Action3, {})], plan
