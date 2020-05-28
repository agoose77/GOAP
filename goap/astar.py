from abc import ABC, abstractmethod
from collections import deque
from sys import float_info

from .priority_queue import PriorityQueue

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
    def get_h_score(self, node, goal):
        raise NotImplementedError

    def find_path(self, goal, start=None):
        if start is None:
            start = goal

        node_to_g_score = {start: 0}
        node_to_f_score = {start: 0}
        node_to_parent = {}

        open_set = PriorityQueue([start], key=node_to_f_score.__getitem__)
        closed_set = set()

        while open_set:
            current = open_set.pop()

            if self.is_finished(current, goal, node_to_parent):
                return self.reconstruct_path(current, goal, node_to_parent)

            closed_set.add(current)

            for neighbour in self.get_neighbours(current):
                if neighbour in closed_set:
                    continue

                tentative_g_score = node_to_g_score[current] + self.get_g_score(
                    current, neighbour
                )
                tentative_is_better = tentative_g_score < node_to_g_score.get(
                    neighbour, float_info.max
                )

                if neighbour in open_set and not tentative_is_better:
                    continue

                node_to_parent[neighbour] = current
                node_to_g_score[neighbour] = tentative_g_score
                node_to_f_score[neighbour] = tentative_g_score + self.get_h_score(
                    neighbour, goal
                )

                open_set.add(neighbour)

        raise PathNotFoundException("Couldn't find path for given nodes")

    @abstractmethod
    def is_finished(self, current, goal, node_to_parent):
        raise NotImplementedError

    @staticmethod
    def reconstruct_path(node, goal, node_to_parent):
        result = deque((node,))

        try:
            while True:
                node = node_to_parent[node]
                result.appendleft(node)

        except KeyError:
            pass

        return result
