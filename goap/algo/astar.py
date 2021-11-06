from abc import ABC, abstractmethod
from collections import deque
from sys import float_info

from goap.priority_queue import PriorityQueue

__all__ = "PathNotFoundException", "AStarAlgorithm"


class PathNotFoundException(Exception):
    pass


class AStarAlgorithm(ABC):
    @abstractmethod
    def get_neighbours(self, node):
        raise NotImplementedError

    @abstractmethod
    def get_g_score(self, current, node):
        raise NotImplementedError

    @abstractmethod
    def get_h_score(self, node):
        raise NotImplementedError

    def find_path(self, start, end):
        g_scores = {start: 0}
        f_scores = {start: self.get_h_score(start)}
        parents = {}

        candidates = PriorityQueue([start], key=lambda n: f_scores[n])
        rejects = set()

        i = 0
        while candidates:
            current = candidates.pop()
            i += 1

            if self.is_finished(current, end, parents):
                return self.reconstruct_path(current, end, parents)

            rejects.add(current)

            for neighbour in self.get_neighbours(current):
                if neighbour in rejects:
                    continue

                tentative_g_score = g_scores[current] + self.get_g_score(
                    current, neighbour
                )
                tentative_is_better = tentative_g_score < g_scores.get(
                    neighbour, float_info.max
                )

                if neighbour in candidates and not tentative_is_better:
                    continue

                parents[neighbour] = current
                g_scores[neighbour] = tentative_g_score
                f_scores[neighbour] = tentative_g_score + self.get_h_score(neighbour)

                candidates.add(neighbour)

        raise PathNotFoundException("Couldn't find path for given nodes")

    @abstractmethod
    def is_finished(self, node, goal, parents):
        raise NotImplementedError

    def reconstruct_path(self, node, goal, parents):
        result = deque((node,))

        while True:
            try:
                node = parents[node]
            except KeyError:
                break
            result.appendleft(node)

        return result
