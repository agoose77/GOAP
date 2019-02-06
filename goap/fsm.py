from abc import ABC, abstractmethod
from typing import Dict, List
from .utils import DictView


class State(ABC):
    @property
    @abstractmethod
    def name(self):
        pass

    def on_enter(self):
        pass

    def on_exit(self):
        pass


class FiniteStateMachine:
    def __init__(self):
        self._state: State = None
        self._states: Dict[str, State] = {}
        self.states = DictView(self._states)

    @property
    def state(self) -> State:
        return self._state

    @state.setter
    def state(self, state: State):
        if self._state is not None:
            self._state.on_exit()

        if state is not None:
            state.on_enter()

        self._state = state

    def add_state(self, state: State, set_default: bool = True):
        self._states[state.name] = state

        # Set default state if none set
        if set_default and self.state is None:
            self.state = state

    def remove_state(self, state: State):
        if self.state is state:
            self.state = None

        self._states.pop(state.name)


class PushDownAutomaton:
    def __init__(self):
        self._states: Dict[str, State] = {}
        self._stack: List[State] = []
        self.states = DictView(self._states)

    @property
    def state(self) -> State:
        if self._stack:
            return self._stack[-1]
        return None

    def push(self, state: State):
        if self._stack:
            self._stack[-1].on_exit()

        self._stack.append(state)
        state.on_enter()

    def pop(self) -> State:
        if not self._stack:
            raise ValueError("Stack empty")

        state = self._stack.pop()
        state.on_exit()

        return state

    def transition_to(self, state: State):
        if self._stack:
            self.pop()

        self.push(state)
