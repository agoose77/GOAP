import pytest

from goap.action import Action
from goap.planner import RegressivePlanner
from util import plan_as_contents

SPOOKY_INCANTATION = "WOOO I'm a ghost"


class Haunt(Action):
    effects = {"is_spooky": True}
    preconditions = {"chant_incantation": SPOOKY_INCANTATION}


class ChantIncantation(Action):
    effects = {"chant_incantation": ...}
    preconditions = {}
    # Higher cost to favour specific incantation
    cost = 2


class ChantSpecificIncantation(Action):
    INCANTATION = "I am ghostly"
    effects = {"chant_incantation": INCANTATION}
    preconditions = {}


@pytest.mark.parametrize(
    ["action_types", "goal_state", "plan_contents"],
    [
        # General incantation
        ([ChantIncantation, ChantSpecificIncantation],
         {"chant_incantation": SPOOKY_INCANTATION},
         [(ChantIncantation, {"chant_incantation": SPOOKY_INCANTATION})]),

        # Specific incantation
        ([ChantIncantation, ChantSpecificIncantation],
         {"chant_incantation": ChantSpecificIncantation.INCANTATION},
         [(ChantSpecificIncantation, {})]),

        # Goal includes service
        ([ChantIncantation, ChantSpecificIncantation, Haunt],
         {"is_spooky": True},
         [(ChantIncantation, {"chant_incantation": SPOOKY_INCANTATION}),
          (Haunt, {})]),

    ]

)
def test(action_types, goal_state, plan_contents):
    world_state = {"chant_incantation": None, "is_undead": False, "is_spooky": False}

    actions = [cls() for cls in action_types]
    planner = RegressivePlanner(world_state, actions)

    plan = planner.find_plan(goal_state)
    assert plan_as_contents(plan) == plan_contents
