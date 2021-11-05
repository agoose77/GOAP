from collections import defaultdict
from dataclasses import dataclass
from operator import attrgetter
from typing import Any, Optional

from .plan import PlanStep
from ..action import Action
from ..algo import AStarAlgorithm

# Regressive algo faster
# http://alumni.media.mit.edu/~jorkin/GOAP_draft_AIWisdom2_2003.pdf


State = dict[str, Any]

_NOT_DEFINED = object()


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

    def apply_action(self, action: Action) -> "RegressiveGOAPAStarNode":
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
            # "update" our goal locally, so we need to apply the *precondition*
            # (our "new" goal) rather than the effect (our "old" goal)

            elif key in action.preconditions:
                value = action.preconditions[key]

            current_state[key] = value

        # Now update our goals!
        goal_state = self.goal_state.copy()
        for key, value in action.preconditions.items():
            # Allow references to effects (for pass-through services)
            if hasattr(value, "goap_effect_reference"):
                value = current_state[value.goap_effect_reference]

            goal_state[key] = value

        return self.__class__(current_state, goal_state, action)

    @property
    def is_satisfied(self):
        return not self.unsatisfied_keys

    @property
    def unsatisfied_keys(self) -> list[str]:
        return [k for k, v in self.goal_state.items() if
                self.current_state.get(k, _NOT_DEFINED) != v]


_key_action_precedence = attrgetter("action.precedence")


class RegressiveGOAPAStarSearch(AStarAlgorithm):

    def __init__(self, actions: list[Action]):
        self._actions = actions
        self._effect_to_action = self._create_effect_table(actions)

    def _create_effect_table(self, actions: list[Action]):
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
                neighbour = node.apply_action(action)

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

    def get_g_score(self, node: RegressiveGOAPAStarNode,
                    neighbour: RegressiveGOAPAStarNode):
        return node.compute_cost(neighbour)


class RegressivePlanner:
    def __init__(self, actions):
        self._actions = actions
        self._search = RegressiveGOAPAStarSearch(actions)

    def find_plan(self, current_state: State, goal_state: State):
        start = RegressiveGOAPAStarNode(current_state, goal_state, None)
        path = self._search.find_path(start, goal_state)
        plan = []
        for node in path:
            if node.action is None:
                break
            step = PlanStep(node.action, node.services)
            plan.append(step)
        return plan
