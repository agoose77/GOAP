from enum import auto, Enum
from typing import ClassVar, Dict, Any, Tuple, List

State = Dict[str, Any]


class EffectReference:
    """Dynamic precondition that references an effect."""

    def __init__(self, name):
        self.goap_effect_reference = name


def reference(name: str) -> EffectReference:
    """Convenience function to return Forwarder object."""
    return EffectReference(name)


class ActionValidator(type):
    """Metaclass to validate action classes at define time."""

    def __new__(mcs, cls_name: str, bases: Tuple[type], attrs: Dict[str, Any]):
        if bases:
            # Validate precondition plugins
            preconditions = attrs.get("preconditions", {})
            # Overwrite effect plugins to ellipsis
            effects = attrs.get("effects", {})
            # Compute names of service-like effects
            attrs["service_names"] = [k for k, v in effects.items() if v is Ellipsis]
            # Catch common errors with preconditions
            mcs._validate_preconditions(effects, preconditions)

        return super().__new__(mcs, cls_name, bases, attrs)

    @staticmethod
    def _validate_preconditions(all_effects: State, preconditions: State):
        for name, value in preconditions.items():
            if value is Ellipsis:
                raise ValueError(f"Preconditions cannot be services (...): '{name}'")

            elif (
                hasattr(value, "goap_effect_reference")
                and value.goap_effect_reference not in all_effects
            ):
                raise ValueError(
                    f"Invalid reference name for precondition '{name}': {value.name!r}"
                )


class Action(metaclass=ActionValidator):
    effects: ClassVar[State] = {}
    preconditions: ClassVar[State] = {}
    service_names: ClassVar[List[str]] = []

    cost = 1.0
    precedence = 0.0
    apply_effects_on_exit = True

    def check_procedural_precondition(self, services: State, is_planning: bool) -> bool:
        return True

    def get_cost(self, services) -> float:
        return self.cost
