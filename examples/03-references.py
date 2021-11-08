"""
Sometimes we also want to pass the service value to another dependent action.
In this example, the `PerformMagic` action requires two service (actions),
`chant_incantation` and `cast_spell`, and passes the `performs_magic` effect value
to both of them using `reference`.
"""

from goap.action import Action, reference
from goap.planner import RegressivePlanner


class Haunt(Action):
    effects = {"is_spooky": True}
    preconditions = {"is_undead": True, "performs_magic": "abracadabra"}


class BecomeUndead(Action):
    effects = {"is_undead": True}
    preconditions = {"is_undead": False}


class PerformMagic(Action):
    effects = {"performs_magic": ...}
    preconditions = {
        "chant_incantation": reference("performs_magic"),
        "cast_spell": reference("performs_magic"),
    }


class ChantIncantation(Action):
    effects = {"chant_incantation": ...}
    preconditions = {}


class CastSpell(Action):
    effects = {"cast_spell": ...}
    preconditions = {}


if __name__ == "__main__":
    world_state = {"is_spooky": False, "is_undead": False}
    goal_state = {"is_spooky": True}
    print("Initial State:", world_state)
    print("Goal State:   ", goal_state)

    actions = [BecomeUndead(), Haunt(), CastSpell(), ChantIncantation(), PerformMagic()]
    planner = RegressivePlanner(world_state, actions)

    plan = planner.find_plan(goal_state)
    print(plan)
