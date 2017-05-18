from enum import auto, Enum


class EvaluationState(Enum):
    failure = auto()
    success = auto()
    running = auto()


class Forwarder:
    """Container class to allow a service-API for plugins"""

    def __init__(self, name):
        self.forward_effect_name = name


def expose(name):
    """Convenience function to return Forwarder object"""
    return Forwarder(name)


class ActionValidator(type):

    def __new__(metacls, cls_name, bases, attrs):
        if bases:
            # Validate precondition plugins
            try:
                preconditions = attrs['preconditions']
            except KeyError:
                pass

            else:
                # Overwrite effect plugins to ellipsis
                all_effects = attrs.get("effects", {})
                metacls._validate_preconditions(all_effects, preconditions)

        return super().__new__(metacls, cls_name, bases, attrs)

    def _validate_preconditions(all_effects, preconditions):
        for name, value in preconditions.items():
            if value is Ellipsis:
                raise ValueError("Invalid value for precondition '{}'".format(name))

            elif hasattr(value, 'forward_effect_name'):
                if value.forward_effect_name not in all_effects:
                    raise ValueError("Invalid plugin name for precondition '{}': {!r}".format(name, value.name))


class ActionBase(metaclass=ActionValidator):
    cost = 1
    precedence = 0

    effects = {}
    preconditions = {}

    apply_effects_on_exit = True

    def __repr__(self):
        return "<Action {!r}>".format(self.__class__.__name__)

    def apply_effects(self, world_state, goal_state):
        """Apply action effects to state, resolving any variables from goal state

        :param world_state: state to modify
        :param goal_state: source for variables
        """
        for key, value in self.effects.items():
            if value is Ellipsis:
                value = goal_state[key]

            world_state[key] = value

    def check_procedural_precondition(self, world_state, goal_state, is_planning=True):
        return True

    def get_status(self, world_state, goal_state):
        return EvaluationState.success

    def get_cost(self, world_state, goal_state):
        return self.cost

    def on_enter(self, world_state, goal_state):
        pass

    def on_exit(self, world_state, goal_state):
        pass
