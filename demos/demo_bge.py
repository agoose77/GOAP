import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), "../"))

from collections.abc import MutableMapping
from time import monotonic

from goap.planner import ActionBase, GoalBase, Director, Planner
from goap.fsm import FiniteStateMachine, State
from goap.enums import EvaluationState


class GoToFuture:

    def __init__(self, target, threshold=0.5):
        self.target = target
        self.status = EvaluationState.running
        self.distance_to_target = -1.0
        self.threshold = threshold

    def on_completed(self):
        self.status = EvaluationState.success


class ChaseTarget(ActionBase):
    apply_effects_on_exit = False
    effects = {"in_weapons_range": True}

    def check_procedural_precondition(self, world_state, goal_state, is_planning=True):
        return world_state['target'] is not None

    def on_enter(self, world_state, goal_state):
        target = world_state['target']
        world_state['fsm'].states['GOTO'].request = GoToFuture(target)

    def get_status(self, world_state, goal_state):
        goto_state = world_state['fsm'].states['GOTO']
        distance = goto_state.request.distance_to_target

        if distance < 0.0 or distance > world_state['min_weapons_range']:
            return EvaluationState.running

        # XXX Stop GOTO (hack, instead make goto do this logic (goto point))
        goto_state.request = None
        return EvaluationState.success


class Attack(ActionBase):
    apply_effects_on_exit = False
    effects = {"target_is_dead": True}
    preconditions = {"weapon_is_loaded": True, "in_weapons_range": True}

    def on_enter(self, world_state, goal_state):
        world_state['fire_weapon'] = True

    def on_exit(self, world_state, goal_state):
        world_state['fire_weapon'] = False
        world_state['target'] = None # Not sure
        world_state['target'] = None

    def get_status(self, world_state, goal_state):
        if not world_state['weapon_is_loaded']:
            return EvaluationState.failure

        target = world_state['target']

        if target is None:
            return EvaluationState.failure

        if target.invalid or target['health'] < 0:
            return EvaluationState.success

        else:
            return EvaluationState.running


class ReloadWeapon(ActionBase):
    """Reload weapon if we have ammo"""
    effects = {"weapon_is_loaded": True}
    preconditions = {"has_ammo": True}


class GetNearestAmmoPickup(ActionBase):
    """GOTO nearest ammo pickup in level"""
    effects = {"has_ammo": True}

    def on_enter(self, world_state, goal_state):
        goto_state = world_state['fsm'].states['GOTO']

        player = world_state['player']
        nearest_pickup = min([o for o in player.scene.objects if "ammo" in o and "pickup" in o],
                             key=player.getDistanceTo)
        goto_state.request = GoToFuture(nearest_pickup)

    def on_exit(self, world_state, goal_state):
        goto_state = world_state['fsm'].states['GOTO']
        world_state["ammo"] += goto_state.request.target["ammo"]

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


class GameObjDict(MutableMapping):
    """Interface to KX_GameObject's properties as a true dictionary"""

    def __init__(self, obj):
        self._obj = obj

    def __getitem__(self, name):
        return self._obj[name]

    def __delitem__(self, name):
        del self._obj[name]

    def __setitem__(self, name, value):
        self._obj[name] = value

    def __iter__(self):
        return (k for k in self._obj.getPropertyNames())

    def __len__(self):
        return len(self._obj.getPropertyNames())


class GOTOState(State):
    name = "GOTO"

    def __init__(self, world_state):
        self.world_state = world_state
        self.request = None

    def update(self):
        request = self.request
        if request is None:
            return

        if request.status != EvaluationState.running:
            return

        player = self.world_state['player']
        to_target = request.target.worldPosition - player.worldPosition

        # Update request
        distance = to_target.length
        request.distance_to_target = distance

        if distance < request.threshold:
            request.on_completed()

        else:
            player.worldPosition += to_target.normalized() * 0.15


class AnimateState(State):
    name = "Animate"

    def __init__(self, world_state):
        self.world_state = world_state


class SystemManager:

    def __init__(self):
        self.systems = []

    def update(self):
        for system in self.systems:
            system.update()


class WeaponFireManager:

    def __init__(self, world_state):
        self.world_state = world_state
        world_state['fire_weapon'] = False

        self.shoot_time = 0.5
        self.last_fired_time = 0

    def update(self):
        if not self.world_state['fire_weapon']:
            return

        now = monotonic()
        if now - self.last_fired_time > self.shoot_time:
            self.last_fired_time = now

            target = self.world_state['target']

            target['health'] -= 10
            if target['health'] <= 0:
                target.endObject()
                self.world_state['fire_weapon'] = False

            self.world_state['ammo'] -= 1

            if not self.world_state['ammo']:
                self.world_state['has_ammo'] = False
                self.world_state['fire_weapon'] = False
                self.world_state['weapon_is_loaded'] = False


class TargetManager:

    def __init__(self, world_state):
        self.world_state = world_state
        self.player = world_state['player']

        world_state['target'] = None

    def get_closest_enemy(self):
        enemies = [o for o in self.player.scene.objects if "enemy" in o]

        if not enemies:
            return None

        return min(enemies, key=self.player.getDistanceTo)

    def update(self):
        world_state = self.world_state
        if world_state['target'] is None:
            world_state['target'] = self.get_closest_enemy()


def init(cont):
    own = cont.owner

    world_state = GameObjDict(own)
    fsm = FiniteStateMachine()

    goto_state = GOTOState(world_state)
    animate_state = AnimateState(world_state)

    fsm.add_state(goto_state)
    fsm.add_state(animate_state)

    world_state['player'] = own
    world_state['fsm'] = fsm

    sys_man = SystemManager()
    sys_man.systems.append(TargetManager(world_state))
    sys_man.systems.append(WeaponFireManager(world_state))

    actions = [a() for a in ActionBase.__subclasses__()]
    goals = [c() for c in GoalBase.__subclasses__()]

    planner = Planner(actions, world_state)
    director = Director(planner, world_state, goals)

    own['ai'] = director
    own['fsm'] = fsm
    own['system_manager'] = sys_man


def main(cont):
    own = cont.owner
    
    ai_manager = own['ai']
    fsm = own['fsm']
    sys_man = own['system_manager']

    sys_man.update()
    ai_manager.update()
    fsm.state.update()
