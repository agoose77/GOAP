from dataclasses import dataclass
from typing import Any, Dict

from ..action import Action

State = Dict[str, Any]


@dataclass(frozen=True)
class PlanStep:
    action: Action
    services: State
