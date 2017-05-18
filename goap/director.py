from logging import getLogger

from .action import EvaluationState
from .astar import PathNotFoundException

logger = getLogger(__name__)


class NoPlanFoundError(Exception):
    pass


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

        goal_relevance_pairs = []

        for goal in self.goals:
            relevance = goal.get_relevance(world_state)
            if relevance <= 0.0:
                continue

            goal_relevance_pairs.append((relevance, goal))

        goal_relevance_pairs.sort()

        return [g for r, g in goal_relevance_pairs]

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

        raise NoPlanFoundError("Couldn't find suitable plan")

    def update(self):
        """Update current plan, or find new plan"""
        world_state = self.world_state

        # Rebuild plan
        if self._plan is None:
            try:
                self._plan = self.find_best_plan()

            except NoPlanFoundError:
                logger.exception("Unable to find viable plan")

            else:
                plan_state = self._plan.update(world_state)

                if plan_state == EvaluationState.running:
                    return

                elif plan_state == EvaluationState.failure:
                    logger.warning("Plan failed during execution of '{}'".format(self._plan.current_plan_step))

        self._plan = None
        self.update()
