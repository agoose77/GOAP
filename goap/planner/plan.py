from dataclasses import dataclass
from typing import Any

from ..action import Action

State = dict[str, Any]


@dataclass(frozen=True)
class PlanStep:
    action: Action
    services: State


class Plan:

    def __init__(self, actions):
        pass
