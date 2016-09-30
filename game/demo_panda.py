import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), "../"))

from time import monotonic
from random import random
from math import cos, sin, pi

from goap.planner import ActionBase, GoalBase, Director, Planner
from goap.fsm import FiniteStateMachine, State
from goap.sensors import SensorManager
from goap.enums import EvaluationState
from goap.memory import Record, RecordMemory
from direct.showbase.ShowBase import ShowBase


class DemoAction(ActionBase):

    def __init__(self, ai_manager):
        self.ai_manager = ai_manager


class GoTo(DemoAction):

    effects = {'at_location_symbol': ...}

    def on_enter(self, world_state, goal_state):
        target_symbol_name = goal_state['at_location_symbol']
        target = world_state[target_symbol_name]

        self.ai_manager.navigation_manager.target = target

    def get_status(self, world_state, goal_state):
        nav_manager = self.ai_manager.navigation_manager

        if nav_manager.distance_to_target < 0.5:
            nav_manager.target = None
            return EvaluationState.success

        return EvaluationState.running

    def on_exit(self, world_state, goal_state):
        self.ai_manager.navigation_manager.target = None


class Attack(DemoAction):

    apply_effects_on_exit = False

    effects = {"enemy_is_dead": True}
    preconditions = {"weapon_is_loaded": True, "in_weapons_range": True}

    def on_enter(self, world_state, goal_state):
        self.ai_manager.weapons_manager.start_firing()

    def on_exit(self, world_state, goal_state):
        self.ai_manager.weapons_manager.stop_firing()

    def get_status(self, world_state, goal_state):
        if not world_state['weapon_is_loaded']:
            return EvaluationState.failure

        target = world_state['target']

        if target is None:
            return EvaluationState.failure

        if target.get_python_tag('health') < 0:
            return EvaluationState.success

        else:
            return EvaluationState.running


class ReloadWeapon(DemoAction):
    """Reload weapon if we have ammo"""
    effects = {"weapon_is_loaded": True}
    preconditions = {"has_ammo": True}

    def on_enter(self, world_state, goal_state):
        self.ai_manager.weapons_manager.reload_weapon()

    def on_enter(self, world_state, goal_state):
        self.ai_manager.weapons_manager.reload_weapon()


class GetNearestAmmoPickup(DemoAction):
    """GOTO nearest ammo pickup in level"""
    effects = {"has_ammo": True}
    preconditions = {"at_location": 'pickup'}

    def on_enter(self, world_state, goal_state):
        goto_state = world_state['fsm'].states['GOTO']

        player = world_state['player']

        pickups = base.render.find_all_matches("**/=is_ammo")
        nearest_pickup = min(pickups, key=player.getDistance)
        world_state['pickup']

    def on_exit(self, world_state, goal_state):
        goto_state = world_state['fsm'].states['GOTO']
        ammo = goto_state.request.target.get_python_tag("ammo")

        world_state["ammo"] += ammo

    def get_status(self, world_state, goal_state):
        goto_state = world_state['fsm'].states['GOTO']
        return goto_state.request.status


class KillEnemyGoal(GoalBase):
    """Kill enemy if target exists"""
    state = {"target_is_dead": True}

    def get_relevance(self, world_state):
        if world_state["target"] is not None:
            return 0.7

        return 0.0


class ReloadWeaponGoal(GoalBase):
    """Reload weapon if not loaded"""

    priority = 0.45
    state = {"weapon_is_loaded": True}


class NoValidPathAvailable(BaseException):
    pass


class Blackboard:
    """Responsible for talking to subsystems without directly talking to them"""

    def __init__(self):
        self.goto_target = None

        self.request_reload = False
        self.current_weapon = None

        self.agent = None

        self.invalidate_path = False


class AIBase:
    """Main AI agent"""

    goal_set = []
    action_set = []

    def __init__(self):
        self.world_state = self.create_world_state()

        self.blackboard = Blackboard()
        self.working_memory = RecordMemory()
        self.sensor_manager = SensorManager(self)

        actions = self.load_action_set()
        self.planner = Planner(actions, self.world_state)

        goals = self.load_goal_set()
        self.director = Director(self.planner, self.world_state, goals)

    def create_world_state(self):
        return {}

    def load_action_set(self):
        return [cls(self) for cls in self.action_set]

    def load_goal_set(self):
        return [cls() for cls in self.goal_set]

    def update(self):
        self.director.update()


class AggressorAI(AIBase):

    action_set = list(DemoAction.__subclasses__())
    goal_set = list(GoalBase.__subclasses__())

    def create_world_state(self):
        return {''}


class DefenderAI(AIBase):

    action_set = list(DemoAction.__subclasses__())
    goal_set = list(GoalBase.__subclasses__())


class NavigationSystem:

    def __init__(self, blackboard):
        self.blackboard = blackboard
        self.blackboard['navigation_target'] = None

        self.target = None
        self.current_path = None

        self._distance_threshold = 0.5

    def _calculate_path(self, target):
        print("Recalculating path...")
        return None

    def _follow_path(self, path):
        player = self.blackboard['player']
        to_target = target.get_pos() - player.get_pos()

        # Update request
        distance = to_target.length()
        to_target.normalize()

        # TODO do actual A* and deal with moving targets
        if distance < self._distance_threshold:
            return

        else:
            player.set_pos(player.get_pos() + to_target * 0.15)

    def update(self):
        target = self.blackboard['navigation_target']

        # If the target has changed
        if target != self.target:
            self.target = target

            if target is None:
                self.current_path = None

            else:
                self.current_path = self._calculate_path(target)

        if self.current_path is None:
            return

        self._follow_path(self.current_path)


class WeaponManager:

    def __init__(self, blackboard):
        self.blackboard = blackboard

        self.shoot_time = 0.5
        self.last_fired_time = 0

    def update(self):
        # # TODO
        pass


class TargetSelector:

    def __init__(self, blackboard, world_state):
        self.world_state = world_state
        self.blackboard = blackboard

        self.agent = blackboard.agent
        blackboard['target'] = None

    def get_closest_enemy(self):
        enemies = base.render.find_all_matches('**/=is_enemy')

        if not enemies:
            return None

        return min(enemies, key=self.agent.getDistance)

    def update(self):
        blackboard = self.blackboard
        if blackboard['target'] is None:
            blackboard['target'] = self.get_closest_enemy()


class Application(ShowBase):

    def __init__(self):
        super().__init__()

        # Planner data
        # self.world_state = self.create_world_state()
        # self.world = self.load_world()
        #
        # self.world_state['player'] = self.world['player']
        # self.world_state['fsm'] = self.fsm

        self.ai = AggressorAI()

    def create_world_state(self):
        return {'in_weapons_range': False, 'target_in_range': False, 'has_ammo': False,
                'weapon_is_loaded': False, 'target_is_dead': False}

    def load_world(self):
        state = {}

        state['player'] = player = loader.loadModel('player.egg').get_child(0)
        player.reparent_to(base.render)
        player.set_pos(0, 0, 0)

        state['ammo'] = ammo = loader.loadModel('ammo.egg').get_child(0)
        ammo.reparent_to(base.render)
        ammo.set_pos(0, 15, 0)

        ammo.set_tag("is_ammo", '')
        ammo.set_python_tag("ammo", 5)

        enemies = state['enemies'] = []

        radius = 10
        for i in range(6):
            enemy = loader.loadModel('suzanne.egg').get_child(0)
            enemy.reparent_to(base.render)
            enemy.set_tag("is_enemy", '')
            enemy.set_python_tag('health', 100)

            theta = 2 * pi * random()
            x = cos(theta)
            y = sin(theta)
            enemy.set_pos(x * radius, y * radius, 0)

            enemies.append(enemy)

        base.cam.set_pos(0, 0, 200)
        base.cam.look_at(player)

        return state

    def update(self, task):
        self.ai.update()
        return task.cont


if __name__ == "__main__":
    app = Application()
    taskMgr.add(app.update, 'update')
    app.run()
