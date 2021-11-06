"""
Sometimes we want to define an action that provides a _service_ rather than
satisfies one particular state value. We can do this by providing ellipsis as the
effect.

When the procedural precondition is called, the action will be given the resolved
services dictionary, where it can succeed or fail on the values.

In this example, the `ChantIncantation` action will chant _anything_ it is asked to.
Here, the `Haunt` action is requesting `chant_incantation` service.
"""
from goap.action import Action
from goap.planner import RegressivePlanner


class Haunt(Action):
    effects = {"is_spooky": True}
    preconditions = {"is_undead": True, "chant_incantation": "WOOO I'm a ghost"}


class BecomeUndead(Action):
    effects = {"is_undead": True}
    preconditions = {"is_undead": False}


class ChantIncantation(Action):
    effects = {"chant_incantation": ...}
    preconditions = {}


if __name__ == "__main__":
    world_state = {"is_spooky": False}
    goal_state = {"is_spooky": True}
    print("Initial State:", world_state)
    print("Goal State:   ", goal_state)

    actions = [BecomeUndead(), Haunt(), ChantIncantation()]
    planner = RegressivePlanner(world_state, actions)

    plan = planner.find_plan(goal_state)
    print(plan)
