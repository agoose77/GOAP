from abc import abstractmethod, ABC


class SensorBase(ABC):

    def __init__(self, ai):
        self.ai = ai

    @abstractmethod
    def update(self):
        pass


class SensorManager:
    """Updates sensors and uses them to update world state using working memory & blackboard"""

    def __init__(self, ai):
        self._sensors = []

        self.ai = ai

    def update(self):
        pass
