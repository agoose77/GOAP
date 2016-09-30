import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../"))

from argparse import ArgumentParser
from goap.planner import ActionBase, GoalBase, Director, Planner


class GoTo(ActionBase):
    effects = {'at_location': ...}

    def on_enter(self, world_state, goal_state):
        query = goal_state['at_location']()
        print("Executing GOTO query: {}".format(query))


class GetAxe(ActionBase):
    def axe_finder():
        return "Find me an Axe, Gimli!"

    effects = {'has_axe': True}
    preconditions = {'at_location': axe_finder}

    def on_enter(self, world_state, goal_state):
        print("Collect ye olde AXE!")


class CutTrees(ActionBase):
    def trees_finder():
        return "Find me the forest of old!"

    effects = {"has_wood": True}
    preconditions = {'at_location': trees_finder, 'has_axe': True}

    def on_enter(self, world_state, goal_state):
        print("Cut dem treedem!")


class CutTreesGoal(GoalBase):
    priority = 1.0

    state = {"has_wood": True}


def apply_plan(plan, world_state):
    for step in plan.plan_steps:
        if step.action.apply_effects_on_exit:
            step.action.apply_effects(world_state, step.goal_state)


if __name__ == "__main__":
    parser = ArgumentParser(description="Run demo GOAP program")
    parser.add_argument("-graph", type=str)
    args = parser.parse_args()

    world_state = dict(at_location=None, has_axe=False, has_wood=False)

    actions = [a() for a in ActionBase.__subclasses__()]
    goals = [c() for c in GoalBase.__subclasses__()]

    planner = Planner(actions, world_state)
    director = Director(planner, world_state, goals)

    plan = director.find_best_plan()
    print("Initial State:", world_state)
    print("Plan:", plan)
    print("----Running Plan" + '-' * 34)

    if args.graph:
        from goap.visualise import visualise_plan
        visualise_plan(plan, args.graph)

    plan.update(world_state)

    print('-' * 50)
    print("Final State:", world_state)
