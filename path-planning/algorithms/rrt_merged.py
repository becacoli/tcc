import random

from algorithms.common import Node, reconstruct_bidirectional_path
from utils.geometry import clamp_point, distance, is_collision_free, steer


class RRTMerge:
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5,
                 goal_sample_rate=0.05, connect_threshold=10, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.tree_start = [self.start]
        self.tree_goal = [self.goal]
        self.connect_threshold = connect_threshold
        self.obstacles = obstacles or []
        self.iterations = 0
        self.done = False
        self.last_sample = None

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.pos
        return (random.uniform(0, self.map_width), random.uniform(0, self.map_height))

    def _nearest(self, tree, point):
        return min(tree, key=lambda n: distance(n.pos, point))

    def _extend(self, tree, point):
        nearest = self._nearest(tree, point)
        new_point = steer(nearest.pos, point, self.step_size)
        new_point = clamp_point(new_point, (self.map_width, self.map_height))

        if is_collision_free(nearest.pos, new_point, self.obstacles):
            new_node = Node(*new_point, parent=nearest)
            tree.append(new_node)
            return new_node
        return None

    def _find_connection(self, new_node, other_tree):
        for node in other_tree:
            if distance(new_node.pos, node.pos) < self.connect_threshold:
                if is_collision_free(new_node.pos, node.pos, self.obstacles):
                    return node
        return None

    def get_all_nodes(self):
        return self.tree_start + self.tree_goal

    def get_tree_groups(self):
        return [self.tree_start, self.tree_goal]

    def step(self):
        if self.iterations >= self.max_iter:
            self.done = True
            return None

        rand_point = self.get_random_point()
        self.last_sample = rand_point
        new_node = self._extend(self.tree_start, rand_point)
        if new_node:
            matched = self._find_connection(new_node, self.tree_goal)
            if matched:
                self.done = True
                return reconstruct_bidirectional_path(new_node, matched)

        self.tree_start, self.tree_goal = self.tree_goal, self.tree_start
        self.iterations += 1
        return None

    def planning(self):
        while not self.done:
            path = self.step()
            if path:
                return path
        return None
