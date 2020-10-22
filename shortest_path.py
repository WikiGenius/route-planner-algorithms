# modification
import heapq
import math


class PathPlanner():
    """Construct a PathPlanner Object"""

    def __init__(self, M, start, goal):
        """ """
        self.map = M
        self.start = start
        self.goal = goal
        self.frontier = []  # use for priorty queue
        self.explored_check = set()
        self.f_score = dict()
        self.g_score = {start: 0}  # g: path cost
        self.cameFrom = {start: None}
        self.path, self.cost = self.run_Search()

    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            if current == None:
                break
            total_path.append(current)
        return list(reversed(total_path))

    def remove_choice(self):
        # A* remove_choice by pick min f
        result = heapq.heappop(self.frontier)
        f = result[0]
        current_node = result[1]
        return current_node, f

    def distance(self, node_1, node_2):
        """ Computes the Euclidean L2 Distance"""
        x0, y0 = self.map.intersections[node_1]
        x1, y1 = self.map.intersections[node_2]
        return math.sqrt(math.pow(x1 - x0, 2) + math.pow(y1 - y0, 2))

    def heuristic_cost_estimate(self, node):
        """ Returns the heuristic cost estimate of a node """
        return self.distance(node, self.goal)

    def get_neighbors(self, node):
        """Returns the neighbors of a node"""
        return self.map.roads[node]

    def run_Search(self):  # O(E + v log(v))
        """Using A* to find the shortest path with its cost"""
        # f = g + h
        self.f_score[self.start] = self.g_score[self.start] + \
            self.heuristic_cost_estimate(self.start)
        heapq.heappush(self.frontier, (self.f_score[self.start], self.start))

        while len(self.frontier) > 0:
            s, f = self.remove_choice()
            if s in self.explored_check:
                continue

            self.explored_check.add(s)
            if s == self.goal:
                return self.reconstruct_path(s), self.g_score[s]
            for neighbour in self.get_neighbors(s):
                if neighbour not in self.explored_check:
                    g = self.g_score[s] + self.distance(s, neighbour)
                    h = self.heuristic_cost_estimate(neighbour)
                    f = g + h
                    if neighbour not in self.cameFrom or f < self.f_score[neighbour]:
                        #                     print(h,g,f)
                        self.cameFrom[neighbour] = s
                        self.g_score[neighbour] = g
                        self.f_score[neighbour] = f
                        heapq.heappush(
                            self.frontier, (self.f_score[neighbour], neighbour))

        return None, -1
