from heapq import heappush, heappop


class PriorityElement:
    def __init__(self, element, score):
        self.value = element
        self.score = score
        self.removed = False

    def __lt__(self, other):
        return self.score < other.score


def _pass_through_key(x):
    return x


class PriorityQueue:
    def __init__(self, items=None, key=None):
        self._dict = {}
        self._heap = []

        if key is None:
            key = _pass_through_key

        self._key = key

        if items:
            for item in items:
                self.add(item)

    def __bool__(self):
        return bool(self._dict)

    def __contains__(self, key):
        return key in self._dict

    def __iter__(self):
        return iter(self._dict)

    def add(self, item):
        if item in self._dict:
            raise ValueError(f"{item} already in queue")

        element = PriorityElement(item, self._key(item))
        self._dict[item] = element
        heappush(self._heap, element)

    def pop(self):
        while True:
            element = heappop(self._heap)

            if not element.removed:
                element.removed = True
                value = element.value
                del self._dict[value]
                return value

    def remove(self, item):
        element = self._dict.pop(item)
        element.removed = True
