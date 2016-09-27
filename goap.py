from operator import attrgetter
from logging import getLogger
from enums import EvaluationState
from astar import AStarAlgorithm, PathNotFoundException
from utils import ListView

__all__ = "Goal", "Action", "Planner", "Director", "ActionNode", "GoalNode", "Goal"


logger = getLogger(__name__)


class Goal:

    state = {}
    priority = 0

    def get_relevance(self, world_state):
        return self.priority

    def is_satisfied(self, world_state):
        for key, value in self.state.items():
            if world_state[key] != value:
                return False

        return True


class Action:
    cost = 1
    precedence = 0

    effects = {}
    preconditions = {}

    apply_effects_on_exit = True

    def __repr__(self):
        return self.__class__.__name__

    def check_procedural_precondition(self, world_state, goal_state, is_planning=True):
        return True

    def get_status(self, world_state):
        return EvaluationState.success

    def get_cost(self):
        return self.cost

    def apply_effects(self, world_state, goal_state):
        """Apply action effects to state, resolving any variables from goal state

        :param world_state: state to modify
        :param goal_state: source for variables
        """
        for key, value in self.effects.items():
            if value is Ellipsis:
                value = goal_state[key]

            world_state[key] = value

    def on_enter(self, world_state, goal_state):
        pass

    def on_exit(self, world_state, goal_state):
        pass

    def preconditions_are_met(self, current_state):
        """Ensure that all preconditions are met in current state

        :param current_state: state to compare against
        """
        for key, value in self.preconditions.items():
            current_value = current_state[key]

            if current_value != value:
                return False

        return True


class NodeBase:
    def __init__(self, current_state=None, goal_state=None):
        if current_state is None:
            current_state = {}

        if goal_state is None:
            goal_state = {}

        self.current_state = current_state
        self.goal_state = goal_state

    @property
    def unsatisfied_keys(self):
        """Return the keys of the unsatisfied state symbols between the goal and current state"""
        current_state = self.current_state
        return [k for k, v in self.goal_state.items() if not current_state[k] == v]

    def satisfies_goal_state(self, state):
        """Determine if provided state satisfies required goal state

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


class GoalNode(NodeBase):
    """GOAP A* Goal Node"""

    def __repr__(self):
        return "<GOAPAStarGoalNode: {}>".format(self.goal_state)


class UnsatisfiableGoalEncountered(BaseException):
    pass


class ActionNode(NodeBase):
    """A* Node with associated GOAP action"""

    @classmethod
    def create_neighbour(cls, action, parent, world_state):
        current_state = parent.current_state.copy()
        goal_state = parent.goal_state.copy()
        cls._update_states_from_action(action, current_state, goal_state, world_state)

        return cls(action, current_state, goal_state)

    def __init__(self, action, current_state=None, goal_state=None):
        super().__init__(current_state, goal_state)

        self.action = action

    def __repr__(self):
        return "<GOAPAStarActionNode {}>".format(self.action)

    @staticmethod
    def _update_states_from_action(action, current_state, goal_state, world_state):
        """Update internal current and goal states, according to action effects and preconditions

        This step is used to backwards track from goal state, adding requirements of this action to goal, and
        applying effects to current state.
        A* search terminates once current state satisfies goal state

        Required for heuristic estimate.
        """
        # Get state of preconditions
        action_preconditions = action.preconditions

        # 1 Update current state from effects, resolve variables
        for key, value in action.effects.items():
            if value is Ellipsis:
                value = goal_state[key]

            current_state[key] = value

        # 2 Update goal state from action preconditions, resolve variables
        for key, value in action_preconditions.items():
            # If we are overwriting an existing goal, it must be satisfied, or else this path is unsolveable!
            if key in goal_state:
                current_goal = goal_state[key]
                if current_goal != current_state.get(key):
                    raise UnsatisfiableGoalEncountered()

            goal_state[key] = value
            current_state[key] = world_state[key]


_key_action_precedence = attrgetter("action.precedence")


class Planner(AStarAlgorithm):

    def __init__(self, action_classes, world_state):
        self.action_classes = action_classes
        self.world_state = world_state

        self.effects_to_actions = self.get_actions_by_effects(action_classes)

    def find_plan_for_goal(self, goal_state):
        """Find shortest plan to produce goal state

        :param goal_state: state of goal
        """
        world_state = self.world_state

        goal_node = GoalNode()
        goal_node.current_state = {k: world_state.get(k) for k in goal_state}
        goal_node.goal_state = goal_state

        node_path = self.find_path(goal_node)

        node_path_next = iter(node_path)
        next(node_path_next)

        plan_steps = [ActionPlanStep(node.action, next_node.goal_state) for next_node, node in
                      zip(node_path, node_path_next)]
        plan_steps.reverse()

        return ActionPlan(plan_steps)

    def get_h_score(self, node, goal):
        """Rough estimate of cost of node, based upon satisfaction of goal state

        :param node: node to evaluate heuristic
        """
        return len(node.unsatisfied_keys)

    def get_g_score(self, current, node):
        """Determine procedural (or static) cost of this action

        :param current: node to move from (unused)
        :param node: node to move to (unused)
        """
        # Update world states
        return node.action.get_cost()

    def get_neighbours(self, node):
        """Return new nodes for given node which satisfy missing state

        :param node: node performing request
        """
        effects_to_actions = self.effects_to_actions
        world_state = self.world_state
        goal_state = node.goal_state
        node_action = getattr(node, "action", None)

        neighbours = []
        for effect in node.unsatisfied_keys:
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

                # Try and create a neighbour node, exceppt if the goal could not be satisfied
                try:
                    neighbour = ActionNode.create_neighbour(action, node, world_state)

                except UnsatisfiableGoalEncountered:
                    continue

                neighbours.append(neighbour)

        neighbours.sort(key=_key_action_precedence)
        return neighbours

    @staticmethod
    def get_actions_by_effects(action_classes):
        """Associate effects with appropriate actions

        :param action_classes: valid action classes
        """
        mapping = {}

        for cls in action_classes:
            action = cls()

            for effect in action.effects:
                try:
                    effect_classes = mapping[effect]

                except KeyError:
                    effect_classes = mapping[effect] = []

                effect_classes.append(action)

        return mapping

    def is_finished(self, node, goal, node_to_parent):
        if node.unsatisfied_keys:
            return False

        # Avoid copying whole state dict, leverage property that keys(node) should include all keys involved in search
        current_state = {k: self.world_state[k] for k in node.current_state}

        while node is not goal:
            node.action.apply_effects(current_state, node.goal_state)
            node = node_to_parent[node]

        return goal.satisfies_goal_state(current_state)


class PlannerFailedException(Exception):
    pass


class ActionPlanStep:
    """Container object for bound action and its goal state"""

    def __init__(self, action, state):
        self.action = action
        self.goal_state = state

    def __repr__(self):
        return repr(self.action)


class ActionPlan:
    """Manager of a series of Actions which fulfil a goal state"""

    def __init__(self, plan_steps):
        self._plan_steps = plan_steps
        self.plan_steps = ListView(self._plan_steps)

        self._plan_steps_it = iter(plan_steps)

        self.current_plan_step = None

    def __repr__(self):
        def repr_step(step):
            return ("{!r}*" if step is self.current_plan_step else "{!r}").format(step)
        return "GOAPAIPlan :: {{{}}}".format(" -> ".join([repr_step(s) for s in self._plan_steps]))

    def update(self, world_state):
        """Update the plan, ensuring it is valid

        :param world_state: world_state object
        """
        finished_state = EvaluationState.success
        running_state = EvaluationState.running

        current_step = self.current_plan_step

        while True:
            # Before initialisation
            if current_step is not None:
                action = current_step.action
                goal_state = current_step.goal_state

                plan_state = action.get_status(world_state)

                # If the plan isn't finished, return its state (failure / running)
                if plan_state == running_state:
                    return running_state

                # Leave previous step
                action.on_exit(world_state, goal_state)

                if action.apply_effects_on_exit:
                    action.apply_effects(world_state, goal_state)

                # Return if not finished
                if plan_state != finished_state:
                    return plan_state

            # Get next step
            try:
                current_step = self.current_plan_step = next(self._plan_steps_it)

            except StopIteration:
                return EvaluationState.success

            action = current_step.action
            goal_state = current_step.goal_state

            # Check preconditions
            if not action.check_procedural_precondition(world_state, goal_state, is_planning=False):
                return EvaluationState.failure

            # Enter step
            action.on_enter(world_state, goal_state)

        return finished_state


class Director:
    """Determine and update GOAP plans for AI"""

    def __init__(self, planner, world_state, goals):
        self.world_state = world_state
        self.planner = planner

        self.goals = goals

        self._plan = None

    @property
    def sorted_goals(self):
        """Return sorted list of goals, if relevant"""
        # Update goals with sorted list
        world_state = self.world_state

        goal_pairs = []
        for goal in self.goals:
            relevance = goal.get_relevance(world_state)
            if relevance <= 0.0:
                continue

            goal_pairs.append((relevance, goal))

        goal_pairs.sort()

        return [g for r, g in goal_pairs]

    def find_best_plan(self):
        """Find best plan to satisfy most relevant, valid goal"""
        build_plan = self.planner.find_plan_for_goal

        # Try all goals to see if we can satisfy them
        for goal in self.sorted_goals:
            # Check the goal isn't satisfied already

            if goal.is_satisfied(self.world_state):
                continue

            try:
                return build_plan(goal.state)

            except PathNotFoundException:
                continue

        raise PlannerFailedException("Couldn't find suitable plan")

    def update(self):
        """Update current plan, or find new plan"""
        world_state = self.world_state

        # Rebuild plan
        if self._plan is None:
            try:
                self._plan = self.find_best_plan()

            except PlannerFailedException as err:
                logger.exception("Unexpected error in finding plan")

        else:
            plan_state = self._plan.update(world_state)

            if plan_state == EvaluationState.running:
                return

            elif plan_state == EvaluationState.failure:
                logger.warn("Plan failed during execution of '{}'".format(self._plan.current_plan_step))

            self._plan = None
            self.update()



