from collections import defaultdict
from enum import Enum, auto
from logging import getLogger
from operator import attrgetter
from typing import Any, Dict, NamedTuple, List, Sequence, AbstractSet, Tuple, Iterable, Generator, Optional, DefaultDict

from .action import Action, ActionStatus, StateType
from .astar import AStarAlgorithm
from .utils import ListView

__all__ = "Goal", "Planner", "ActionNode", "GoalNode"

logger = getLogger(__name__)


class PlanStatus(Enum):
    failure = auto()
    success = auto()
    running = auto()
    cancelled = auto()


class UnsatisfiableGoalEncountered(BaseException):
    pass


class Goal:
    state: StateType = {}
    priority: float = 1.0

    def get_relevance(self, world_state: StateType) -> float:
        """Return the relevance of the goal given the current world state.
        Larger is more relevant.

        :param world_state: world state symbol table
        :return:
        """
        return self.priority

    def is_satisfied(self, world_state: StateType) -> bool:
        """Return True if this goal is satisfied under the given world state

        :param world_state: world state symbol table
        :return:
        """
        for key, value in self.state.items():
            if world_state[key] != value:
                return False

        return True


class Node:
    """Base class for GOAP A* Node."""

    def __init__(self, current_state: StateType = None, goal_state: StateType = None):
        if current_state is None:
            current_state = {}

        if goal_state is None:
            goal_state = {}

        self.current_state = current_state
        self.goal_state = goal_state

    @property
    def unsatisfied_state(self) -> AbstractSet[Tuple[str, Any]]:
        """Return the keys of the unsatisfied state symbols between the goal and current state."""
        return self.goal_state.items() - self.current_state.items()

    def satisfies_goal_state(self, state: StateType) -> bool:
        """Determine if provided state satisfies required goal state.

        :param state: state to test
        """
        for key, goal_value in self.goal_state.items():
            try:
                current_value = state[key]

            except KeyError:
                return False

            if current_value != goal_value:
                return False

        return True


class GoalNode(Node):
    """A* Node with associated GOAP goal."""

    def __repr__(self) -> str:
        return "{}(goal_state={!r})".format(type(self).__name__, self.goal_state)


class ActionNode(Node):
    """A* Node with associated GOAP action."""

    def __init__(self, action: Action, current_state: StateType = None, goal_state: StateType = None):
        super().__init__(current_state, goal_state)

        self.action = action

    def __repr__(self) -> str:
        return "{}(action={!r})".format(type(self).__name__, self.action)

    @classmethod
    def create_neighbour(cls, action: Action, parent: "ActionNode", world_state: StateType) -> "ActionNode":
        """Create an ActionNode instance, with the same initial starting state as a given action.

        :param action: action for the new node
        :param parent: node from which the neighbour is created
        :param world_state: current world state, used to populate missing state fields
        :return:
        """
        current_state = parent.current_state.copy()
        goal_state = parent.goal_state.copy()

        cls._update_states_from_action(action, current_state, goal_state, world_state)

        return cls(action, current_state, goal_state)

    @staticmethod
    def _update_states_from_action(
        action: Action, current_state: StateType, goal_state: StateType, world_state: StateType
    ):
        """Update internal current and goal states, according to action effects and preconditions

        This step is used to backwards track from goal state, adding requirements of this action to goal, and
        applying effects to current state.
        A* search terminates once current state satisfies goal state

        Required for heuristic estimate.

        :param action: Action object
        :param current_state: mapping of current world state of pathfinder
        :param goal_state: goal state of pathfinder
        :param world_state: external world state (to populate default values for state keys introduced by preconditions)
        :return:
        """
        # Get state of preconditions
        action_preconditions = action.preconditions

        # 1 Update current state from effects, resolve variables
        action.apply_effects(current_state, goal_state)

        # 2 Update goal state from action preconditions, resolve variables
        for key, value in action_preconditions.items():
            # If we are overwriting an existing goal, it must already be satisfied (i.e. goal[k] == current[k]),
            # or else this path is unsolveable!
            if key in goal_state:
                if current_state[key] != goal_state[key]:
                    raise UnsatisfiableGoalEncountered()

            # Preconditions can expose effect plugins to a dependency
            if hasattr(value, "forwarded_effect_name"):
                value = current_state[value.forwarded_effect_name]

            goal_state[key] = value

            if key not in current_state:
                current_state[key] = world_state[key]


class ActionPlanStep(NamedTuple):
    """Container object for bound action and its goal state."""

    action: Action
    goal_state: StateType


class ActionPlan:
    """Manager of a series of Actions which fulfil a goal state."""

    def __init__(self, plan_steps: List[ActionPlanStep], world_state: StateType):
        self._steps = plan_steps
        self._world_state = world_state

        self._current_step: Optional[ActionPlanStep] = None
        self._is_cancelled = False
        self._generator = self._execution_loop(plan_steps, world_state)
        next(self._generator)

    def __repr__(self) -> str:
        return "{}(plan_steps={!r})".format(type(self).__name__, self._steps)

    def __str__(self) -> str:
        return "{}: {}".format(
            type(self).__name__,
            " -> ".join([("{!r}*" if s is self._current_step else "{!r}").format(s) for s in self._steps]),
        )

    def _execution_loop(
        self, plan_steps: Iterable[ActionPlanStep], world_state: StateType
    ) -> Generator[PlanStatus, Optional[bool], PlanStatus]:
        """Generator which yields `PlanStatus` values at discrete yield points (which indicate the end of one evaluation step).

         :param plan_steps: iterable of ActionPlanStep objects
         :param world_state: world state symbol table
         :return:
         """
        # Initial yield to allow us to initialise the generator with next()
        yield PlanStatus.running

        for step in plan_steps:
            self._current_step = step

            action = step.action
            goal_state = step.goal_state

            # 1. Initialise plan step
            if not action.check_procedural_precondition(world_state, goal_state, is_planning=False):
                # Stop execution as action cannot be executed
                return PlanStatus.failure

            action.on_enter(world_state, goal_state)

            # 2. Update plan step
            while True:
                status: ActionStatus = action.get_status(world_state, goal_state)
                if status == ActionStatus.running:
                    # Suspend execution as action is "busy"
                    is_cancelled = yield PlanStatus.running

                    if is_cancelled:
                        # We were cancelled, inform action of failure
                        action.on_failure(world_state, goal_state)
                        return PlanStatus.cancelled
                else:
                    break

            # 3.b Finish plan step (failure)
            if status == ActionStatus.failure:
                action.on_failure(world_state, goal_state)
                # Stop execution as action failed
                return PlanStatus.failure

            # 3.a Finish plan step (success)
            if status == ActionStatus.success:
                action.on_exit(world_state, goal_state)
                if action.apply_effects_on_exit:
                    action.apply_effects(world_state, goal_state)

        return PlanStatus.success

    @property
    def current_step(self) -> Optional[ActionPlanStep]:
        """Return current plan step."""
        return self._current_step

    @property
    def steps(self):
        """Return view over plan steps."""
        return ListView(self._steps)

    def update(self):
        """Advance plan by one execution step."""
        try:
            return self._generator.send(self._is_cancelled)
        except StopIteration as exc:
            return exc.value

    def cancel(self):
        """Cancel execution of plan."""
        self._is_cancelled = True


_key_action_precedence = attrgetter("action.precedence")


class Planner(AStarAlgorithm):
    def __init__(self, actions: Sequence[Action], world_state: StateType):
        self.actions = actions
        self.world_state = world_state

        self.effects_to_actions = self.create_effect_table(actions)

    def find_plan_for_goal(self, goal_state: StateType):
        """Find shortest plan to produce goal state.

        :param goal_state: state of goal
        """
        world_state = self.world_state

        goal_node = GoalNode({k: world_state.get(k) for k in goal_state}, goal_state)
        node_path = self.find_path(goal_node)

        node_path_next = iter(node_path)
        next(node_path_next)

        plan_steps = [
            ActionPlanStep(node.action, next_node.goal_state) for next_node, node in zip(node_path, node_path_next)
        ]
        plan_steps.reverse()

        return ActionPlan(plan_steps, self.world_state)

    def get_h_score(self, node: ActionNode, goal: GoalNode) -> int:
        """Rough estimate of cost of node, based upon satisfaction of goal state.

        :param node: node to evaluate heuristic
        :param goal: goal node
        """
        return len(node.unsatisfied_state)

    def get_g_score(self, current: ActionNode, node: ActionNode) -> float:
        """Determine procedural (or static) cost of this action.

        :param current: node to move from (unused)
        :param node: node to move to (unused)
        """
        # Update world states
        return node.action.get_cost(self.world_state, node.goal_state)

    def get_neighbours(self, node: ActionNode) -> List[ActionNode]:
        """Return new nodes for given node which satisfy missing state.

        :param node: node performing request
        """
        effects_to_actions = self.effects_to_actions
        world_state = self.world_state
        goal_state = node.goal_state
        node_action = getattr(node, "action", None)

        neighbours = []
        for effect, _ in node.unsatisfied_state:
            try:
                actions = effects_to_actions[effect]
            except KeyError:
                continue

            # Create new node instances for every node
            for action in actions:
                if action is node_action:
                    continue

                if not action.check_procedural_precondition(world_state, goal_state, is_planning=True):
                    continue

                # Try and create a neighbour node, except if the goal could not be satisfied
                try:
                    neighbour = ActionNode.create_neighbour(action, node, world_state)
                except UnsatisfiableGoalEncountered:
                    continue

                neighbours.append(neighbour)

        neighbours.sort(key=_key_action_precedence)
        return neighbours

    @staticmethod
    def create_effect_table(actions: Iterable[Action]) -> Dict[str, List[Action]]:
        """Associate effects with appropriate actions.

        :param actions: valid action instances
        """
        effect_to_actions: DefaultDict[str, List[Action]] = defaultdict(list)

        for action in actions:
            for effect in action.effects:
                effect_to_actions[effect].append(action)

        effect_to_actions.default_factory = None
        return effect_to_actions

    def is_finished(self, node: ActionNode, goal: GoalNode, node_to_parent: Dict[ActionNode, ActionNode]) -> bool:
        """Return True if the given node is a solution to the path-finding problem (satisfies the goal)

        :param node: ActionNode object
        :param goal: GoalNode object
        :param node_to_parent: mapping from an ActionNode to its parent ActionNode
        :return:
        """
        if node.unsatisfied_state:
            return False

        # Avoid copying whole state dict, leverage property that keys(node) should include all keys involved in search
        current_state = {k: self.world_state[k] for k in node.current_state}

        while node is not goal:
            node.action.apply_effects(current_state, node.goal_state)
            node = node_to_parent[node]

        return goal.satisfies_goal_state(current_state)
