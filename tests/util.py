from typing import List

from goap.planner import PlanStep


def plan_as_contents(plan: List[PlanStep]):
    return [(p.action.__class__, p.services) for p in plan]
