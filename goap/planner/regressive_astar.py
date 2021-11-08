from collections import defaultdict
from dataclasses import dataclass
from operator import attrgetter
from typing import Any, Optional, List, Dict

from .plan import PlanStep
from ..action import Action
from ..algo import AStarAlgorithm

# Regressive algo faster
# http://alumni.media.mit.edu/~jorkin/GOAP_draft_AIWisdom2_2003.pdf


State = Dict[str, Any]


class _NotDefined:
    def __repr__(self):
        return "NOT_DEFINED"


NOT_DEFINED = _NotDefined()


@dataclass(frozen=True)
class RegressiveGOAPAStarNode:
    current_state: State
    goal_state: State
    action: Optional[Action] = None

    def __hash__(self):
        return id(self)

    @property
    def services(self):
        return {k: self.current_state[k] for k in self.action.service_names}

    def compute_cost(self, other: "RegressiveGOAPAStarNode") -> float:
        # The action determines the cost, and is stored on the "destination" node
        # i.e. x -- action --> y is encoded as (x, None), (y, action)
        return other.action.get_cost(other.services)

    def apply_action(
            self, world_state: State, action: Action
    ) -> "RegressiveGOAPAStarNode":
        # New node is a consequence of transitioning from
        # node to neighbour via action (which is the edge)
        current_state = self.current_state.copy()
        for key, value in action.effects.items():
            # After applying this action, as far as the regressive planner is concerned
            # we want to our new goal to be to satisfy the actions preconditions.
            # (see the next for block)
            if value is Ellipsis:
                # This action is "fulfilling" some of our goal state, hence
                # when using services (...), we take from the goal state!
                value = self.goal_state[key]

            # Normally we work backwards from the goal by building up a current state
            # that ultimately matches our goal state.
            # However, when an action shares a precondition and effect, we effectively
            # "update" our goal locally, so just as when we encounter a new unseen goal,
            # we need to take the current state from the world state
            elif key in action.preconditions:
                value = world_state.get(key, NOT_DEFINED)

            current_state[key] = value

        # Now update our goals!
        goal_state = self.goal_state.copy()
        for key, value in action.preconditions.items():
            # Allow references to effects (for pass-through services)
            if hasattr(value, "goap_effect_reference"):
                value = current_state[value.goap_effect_reference]

            goal_state[key] = value

            # Add new fields to current state
            if key not in current_state:
                current_state[key] = world_state.get(key, NOT_DEFINED)

        return self.__class__(current_state, goal_state, action)

    @property
    def is_satisfied(self):
        return not self.unsatisfied_keys

    @property
    def unsatisfied_keys(self) -> List[str]:
        return [
            k
            for k, v in self.goal_state.items()
            # Allow keys not to be defined (e.g. for internal fields)
            if self.current_state[k] != v
        ]


_key_action_precedence = attrgetter("action.precedence")


class RegressiveGOAPAStarSearch(AStarAlgorithm):
    def __init__(self, world_state: State, actions: List[Action]):
        self._actions = actions
        self._effect_to_action = self._create_effect_table(actions)
        self._world_state = world_state

    def _create_effect_table(self, actions: List[Action]):
        effect_to_actions = defaultdict(list)

        for action in actions:
            for effect in action.effects:
                effect_to_actions[effect].append(action)

        effect_to_actions.default_factory = None
        return effect_to_actions

    def reconstruct_path(self, node, goal, parents):
        path = super().reconstruct_path(node, goal, parents)
        return reversed(path)

    def is_finished(self, node: RegressiveGOAPAStarNode, goal, parents):
        return node.is_satisfied

    def get_neighbours(self, node: RegressiveGOAPAStarNode):
        neighbours = []

        for key in node.unsatisfied_keys:
            goal_value = node.goal_state[key]

            for action in self._effect_to_action[key]:
                effect_value = action.effects[key]
                # If this effect does not satisfies the goal
                if effect_value != goal_value and effect_value is not Ellipsis:
                    continue

                # Compute neighbour after following action edge.
                neighbour = node.apply_action(self._world_state, action)

                # Ensure we can visit this node
                if not action.check_procedural_precondition(
                        neighbour.services, is_planning=True
                ):
                    continue

                neighbours.append(neighbour)
        neighbours.sort(key=_key_action_precedence)
        return neighbours

    def get_h_score(self, node: RegressiveGOAPAStarNode):
        return len(node.unsatisfied_keys)

    def get_g_score(
            self, node: RegressiveGOAPAStarNode, neighbour: RegressiveGOAPAStarNode
    ):
        return node.compute_cost(neighbour)


class RegressivePlanner:
    def __init__(self, world_state, actions):
        self._actions = actions
        self._world_state = world_state
        self._search = RegressiveGOAPAStarSearch(world_state, actions)

    @property
    def actions(self):
        return self._actions

    @property
    def world_state(self):
        return self._world_state

    def _create_plan_steps(self, path: List[RegressiveGOAPAStarNode]) -> List[PlanStep]:
        plan = []
        for node in path:
            if node.action is None:
                break
            plan.append(PlanStep(node.action, node.services))
        return plan

    def find_plan(self, goal_state: State) -> List[PlanStep]:
        # Initially populate the current state with keys from the goal, using
        # current values
        initial_state = {k: self._world_state[k] for k in goal_state}
        start = RegressiveGOAPAStarNode(initial_state, goal_state, None)

        path = self._search.find_path(start, goal_state)
        return self._create_plan_steps(path)
