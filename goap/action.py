from enum import auto, Enum
from typing import NamedTuple, ClassVar, Dict, Any, Tuple


StateType = Dict[str, Any]


class ActionStatus(Enum):
    failure = auto()
    success = auto()
    running = auto()


class EffectReference(NamedTuple):
    """Container class to allow a service-API for plugins."""

    forwarded_effect_name: str


def reference(name: str) -> EffectReference:
    """Convenience function to return Forwarder object."""
    return EffectReference(name)


# WARNING: Deprecated
expose = reference


class ActionValidator(type):
    """Metaclass to validate action classes at define time."""

    def __new__(mcs, cls_name: str, bases: Tuple[type], attrs: Dict[str, Any]):
        if bases:
            # Validate precondition plugins
            preconditions = attrs.get("preconditions", {})
            # Overwrite effect plugins to ellipsis
            all_effects = attrs.get("effects", {})
            mcs._validate_preconditions(all_effects, preconditions)

        return super().__new__(mcs, cls_name, bases, attrs)

    @staticmethod
    def _validate_preconditions(all_effects: StateType, preconditions: StateType):
        for name, value in preconditions.items():
            if value is Ellipsis:
                raise ValueError("Invalid value for precondition '{}'".format(name))

            elif hasattr(value, "forwarded_effect_name") and value.forwarded_effect_name not in all_effects:
                raise ValueError("Invalid plugin name for precondition '{}': {!r}".format(name, value.name))


class Action(metaclass=ActionValidator):
    effects: ClassVar[StateType] = {}
    preconditions: ClassVar[StateType] = {}

    cost = 1.0
    precedence = 0.0
    apply_effects_on_exit = True

    def apply_effects(self, world_state: StateType, goal_state: StateType):
        """Apply action effects to state, resolving any variables from goal state

        :param world_state: state to modify
        :param goal_state: source for variables
        """
        for key, value in self.effects.items():
            if value is Ellipsis:
                value = goal_state[key]

            world_state[key] = value

    def check_procedural_precondition(
        self, world_state: StateType, goal_state: StateType, is_planning: bool = True
    ) -> bool:
        return True

    def get_status(self, world_state: StateType, goal_state: StateType) -> ActionStatus:
        return ActionStatus.success

    def get_cost(self, world_state: StateType, goal_state: StateType) -> float:
        return self.cost

    def on_enter(self, world_state: StateType, goal_state: StateType):
        pass

    def on_exit(self, world_state: StateType, goal_state: StateType):
        pass

    def on_failure(self, world_state: StateType, goal_state: StateType):
        pass
