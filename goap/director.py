from logging import getLogger
from typing import List, Sequence

from .action import StateType
from .astar import PathNotFoundException
from .planner import Planner, PlanStatus, ActionPlan, Goal

logger = getLogger(__name__)


class NoPlanFoundError(Exception):
    pass


class Director:
    """Determine and update GOAP plans for AI."""

    def __init__(self, planner: Planner, world_state: StateType, goals: Sequence[Goal]):
        self.world_state = world_state
        self.planner = planner
        self.goals = goals

        self._generator = self._execution_loop()

    def _execution_loop(self):
        while True:
            # Select plan
            try:
                plan = self.find_best_plan()
            except NoPlanFoundError:
                logger.exception("Unable to find viable plan")
                yield
                continue

            # Update plan
            while True:
                status = plan.update()
                if status == PlanStatus.running:
                    yield
                    continue

                if status == PlanStatus.failure:
                    logger.warning("Plan failed during execution of '{}'".format(plan.current_step))
                break

    @property
    def sorted_goals(self) -> List[Goal]:
        """Return sorted list of goals, if relevant."""
        world_state = self.world_state

        goal_relevance_pairs = []
        for goal in self.goals:
            relevance = goal.get_relevance(world_state)
            if relevance <= 0.0:
                continue

            goal_relevance_pairs.append((relevance, goal))
        goal_relevance_pairs.sort(reverse=True)

        return [g for r, g in goal_relevance_pairs]

    def find_best_plan(self) -> ActionPlan:
        """Find best plan to satisfy most relevant, valid goal."""
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
        """Update current plan, or find new plan."""
        next(self._generator)
