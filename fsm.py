from abc import ABC, abstractproperty

from .utils import DictView


class State(ABC):
    @abstractproperty
    def name(self):
        pass

    def on_enter(self):
        pass

    def on_exit(self):
        pass


class FiniteStateMachine:
    def __init__(self):
        self._state = None

        self._states = {}
        self.states = DictView(self._states)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state):
        if self._state is not None:
            self._state.on_exit()

        if state is not None:
            state.on_enter()

        self._state = state

    def add_state(self, state, set_default=True):
        self._states[state.name] = state

        # Set default state if none set
        if set_default and self._state is None:
            self._state = state
            state.on_enter()

    def remove_state(self, state):
        state.on_exit()

        if self._state is state:
            self._state = None

        self._states.pop(state.name)


class PushDownAutomaton:
    def __init__(self):
        self._states = {}
        self.states = DictView(self._states)
        self._stack = []

    @property
    def state(self):
        if self._stack:
            return self._stack[-1]
        return None

    def push(self, state):
        if self._stack:
            self._stack[-1].on_exit()

        self._stack.append(state)
        state.on_enter()

    def pop(self):
        if not self._stack:
            raise ValueError("Stack empty")

        state = self._stack.pop()
        state.on_exit()

        return state

    def transition_to(self, state):
        if self._stack:
            self.pop()

        self.push(state)
