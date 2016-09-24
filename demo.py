from goap import Action, Goal, GoalManager, Planner
from fsm import FiniteStateMachine
from state import State
from time import monotonic
from enums import EvaluationState


class GOTORequest:

    def __init__(self, target):
        self.target = target
        self.status = EvaluationState.running
        self.distance_to_target = -1.0

    def on_completed(self):
        self.status = EvaluationState.success


class ChaseTarget(Action):
    apply_effects_on_exit = False
    effects = {"in_weapons_range": True}

    def check_procedural_precondition(self, world_state, goal_state, is_planning=True):
        return world_state['target'] is not None

    def on_enter(self, world_state, goal_state):
        target = world_state['target']
        world_state.fsm.states['GOTO'].request = GOTORequest(target)

    def get_status(self, world_state):
        goto_state = world_state.fsm.states['GOTO']
        distance = goto_state.request.distance_to_target

        if distance < 0.0 or distance > world_state['min_weapons_range']:
            return EvaluationState.running

        # XXX Stop GOTO (hack, instead make goto do this logic (goto point))
        goto_state.request = None
        return EvaluationState.success


class Attack(Action):
    apply_effects_on_exit = False
    effects = {"target_is_dead": True}
    preconditions = {"weapon_is_loaded": True, "in_weapons_range": True}

    def on_enter(self, world_state, goal_state):
        world_state['fire_weapon'] = True

    def on_exit(self, world_state, goal_state):
        world_state['fire_weapon'] = False
        world_state['target'] = None # Not sure
        world_state['target'] = None

    def get_status(self, world_state):
        if not world_state['weapon_is_loaded']:
            return EvaluationState.failure

        target = world_state['target']

        if target is None:
            return EvaluationState.failure

        if target.invalid or target['health'] < 0:
            return EvaluationState.success

        else:
            return EvaluationState.running


class ReloadWeapon(Action):
    """Reload weapon if we have ammo"""
    effects = {"weapon_is_loaded": True}
    preconditions = {"has_ammo": True}


class GetNearestAmmoPickup(Action):
    """GOTO nearest ammo pickup in level"""
    effects = {"has_ammo": True}

    def on_enter(self, world_state, goal_state):
        goto_state = world_state.fsm.states['GOTO']

        player = world_state.player
        nearest_pickup = min([o for o in player.scene.objects if "ammo" in o and "pickup" in o],
                             key=player.getDistanceTo)
        goto_state.request = GOTORequest(nearest_pickup)

    def on_exit(self, world_state, goal_state):
        goto_state = world_state.fsm.states['GOTO']
        world_state["ammo"] += goto_state.request.target["ammo"]

    def get_status(self, world_state):
        goto_state = world_state.fsm.states['GOTO']
        return goto_state.request.status


class KillEnemyGoal(Goal):
    """Kill enemy if target exists"""
    state = {"target_is_dead": True}

    def get_relevance(self, world_state):
        if world_state["target"] is not None:
            return 0.7

        return 0.0


class ReloadWeaponGoal(Goal):
    """Reload weapon if not loaded"""

    priority = 0.45
    state = {"weapon_is_loaded": True}


class GameObjDict:

    def __init__(self, obj):
        self._obj = obj

    def __getitem__(self, name):
        return self._obj[name]

    def __setitem__(self, name, value):
        self._obj[name] = value

    def copy(self):
        return dict(self.items())

    def get(self, name, default=None):
        try:
            return self[name]

        except KeyError:
            return default

    def setdefault(self, name, value):
        try:
            return self[name]

        except KeyError:
            self[name] = value
            return value

    def keys(self):
        return iter(self._obj.getPropertyNames())

    def update(self, other):
        for key, value in other.items():
            self._obj[key] = value

    def values(self):
        return (self._obj[x] for x in self.keys())

    def items(self):
        return ((x, y) for x, y in zip(self.keys(), self.values()))

    def __repr__(self):
        return repr(dict(self.items()))


class GOTOState(State):

    def __init__(self, world_state):
        super().__init__("GOTO")

        self.world_state = world_state
        self.request = None

    def update(self):
        request = self.request

        if request is None:
            return

        if request.status != EvaluationState.running:
            return

        player = self.world_state.player
        to_target = request.target.worldPosition - player.worldPosition

        # Update request
        distance = to_target.length
        request.distance_to_target = distance

        if distance < 0.5:
            request.status = EvaluationState.success

        else:
            player.worldPosition += to_target.normalized() * 0.15


class AnimateState(State):

    def __init__(self, world_state):
        super().__init__("Animate")

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
        if self.world_state['fire_weapon']:
            now = monotonic()
            if now - self.last_fired_time > self.shoot_time:
                self.last_fired_time = now
                self.world_state['target']['health'] -= 10
                if self.world_state['target']['health'] <= 0:
                    self.world_state['target'].endObject()
                    self.world_state['fire_weapon'] = False

                self.world_state['ammo'] -= 1

                if not self.world_state['ammo']:
                    self.world_state['has_ammo'] = False
                    self.world_state['fire_weapon'] = False
                    self.world_state['weapon_is_loaded'] = False


class TargetManager:

    def __init__(self, world_state):
        self.world_state = world_state
        self.player = world_state.player

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

    world_state.player = own
    world_state.fsm = fsm

    sys_man = SystemManager()
    sys_man.systems.append(TargetManager(world_state))
    sys_man.systems.append(WeaponFireManager(world_state))

    actions = Action.__subclasses__()
    goals = [c() for c in Goal.__subclasses__()]
    print(goals)
    planner = Planner(actions, world_state)
    goap_ai_manager = GoalManager(planner, world_state, goals)

    own['ai'] = goap_ai_manager
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
