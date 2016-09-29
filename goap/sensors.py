from abc import abstractmethod, ABC


class SensorBase(ABC):

    def __init__(self, ai):
        self.ai = ai

    @abstractmethod
    def update(self):
        pass


class SensorManager:

    def __init__(self, ai):
        self.ai = ai
        self._sensors = []

    def update(self):
        pass # TODO
