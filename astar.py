from abc import ABC, abstractmethod
from operator import attrgetter

from priority_queue import PriorityQueue


__all__ = "PathNotFoundException", "AStarAlgorithm"


class PathNotFoundException(Exception):
    pass


class AStarNode:
    g_score = 0
    f_score = 0


_fscore_key_func = attrgetter("f_score")


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

        open_set = PriorityQueue(start, key=_fscore_key_func)
        closed_set = set()

        is_complete = self.is_finished
        get_neighbours = self.get_neighbours
        get_g_score = self.get_g_score
        get_heuristic = self.get_h_score

        node_to_g_score = {}
        node_to_f_score = {}
        node_to_parent = {}

        while open_set:
            current = open_set.pop()

            if is_complete(current, goal, node_to_parent):
                return self.reconstruct_path(current, goal, node_to_parent)

            closed_set.add(current)

            for neighbour in get_neighbours(current):
                tentative_g_score = current.g_score + get_g_score(current, neighbour)

                tentative_is_better = tentative_g_score < neighbour.g_score
                in_open = neighbour in open_set
                in_closed = neighbour in closed_set

                if in_open and tentative_is_better:
                    open_set.remove(neighbour)

                if in_closed and tentative_is_better:
                    closed_set.remove(neighbour)

                if not in_open and not in_closed:
                    node_to_parent[neighbour] = current
                    node_to_g_score[neighbour] = tentative_g_score
                    node_to_f_score[neighbour] = tentative_g_score + get_heuristic(neighbour, goal)

                    open_set.add(neighbour)

        raise PathNotFoundException("Couldn't find path for given nodes")

    @abstractmethod
    def is_finished(self, current, goal, node_to_parent):
        raise NotImplementedError

    @staticmethod
    def reconstruct_path(node, goal, node_to_parent):
        result = [node]

        try:
            while True:
                node = node_to_parent[node]
                result.append(node)

        except KeyError:
            pass

        result.reverse()
        return result

