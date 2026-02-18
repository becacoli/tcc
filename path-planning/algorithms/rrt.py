import random

from algorithms.common import Node, build_path
from utils.geometry import clamp_point, distance, is_collision_free, steer


class RRT:
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5,
                 goal_sample_rate=0.05, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]
        self.obstacles = obstacles or []
        self.iterations = 0
        self.done = False
        self.last_sample = None

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.pos
        return (random.uniform(0, self.map_width), random.uniform(0, self.map_height))

    def _nearest(self, point):
        return min(self.tree, key=lambda n: distance(n.pos, point))

    def get_all_nodes(self):
        return self.tree

    def get_tree_groups(self):
        return [self.tree]

    def step(self):
        if self.iterations >= self.max_iter:
            self.done = True
            return None

        rand_point = self.get_random_point()
        self.last_sample = rand_point
        nearest = self._nearest(rand_point)
        new_point = steer(nearest.pos, rand_point, self.step_size)
        new_point = clamp_point(new_point, (self.map_width, self.map_height))

        if is_collision_free(nearest.pos, new_point, self.obstacles):
            new_node = Node(*new_point, parent=nearest)
            self.tree.append(new_node)

            if distance(new_node.pos, self.goal.pos) <= self.step_size:
                if is_collision_free(new_node.pos, self.goal.pos, self.obstacles):
                    goal_node = Node(self.goal.x, self.goal.y, parent=new_node)
                    self.done = True
                    return build_path(goal_node)

        self.iterations += 1
        return None

    def planning(self):
        while not self.done:
            path = self.step()
            if path:
                return path
        return None
