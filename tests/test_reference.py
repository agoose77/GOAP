from goap.action import Action, reference
from goap.planner import RegressivePlanner

from util import plan_as_contents


class Haunt(Action):
    effects = {"is_spooky": True}
    preconditions = {"performs_magic": "abracadabra"}


class PerformMagic(Action):
    effects = {"performs_magic": ...}
    preconditions = {
        "chant_incantation": reference("performs_magic"),
    }


class ChantIncantation(Action):
    effects = {"chant_incantation": ...}
    preconditions = {}


def test():
    world_state = {"is_spooky": False, "chant_incantation": None}
    goal_state = {"is_spooky": True}

    actions = [Haunt(), ChantIncantation(), PerformMagic()]
    planner = RegressivePlanner(world_state, actions)

    plan = planner.find_plan(goal_state)

    assert plan_as_contents(plan) == [
        (ChantIncantation, {"chant_incantation": "abracadabra"}),
        (PerformMagic, {"performs_magic": "abracadabra"}),
        (Haunt, {}),
    ]
