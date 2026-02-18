import enum
import random

from algorithms.common import Node, reconstruct_bidirectional_path
from utils.geometry import clamp_point, distance, is_collision_free, steer


class _Status(enum.Enum):
    TRAPPED = 1
    ADVANCED = 2
    REACHED = 3


class _Tree:
    def __init__(self):
        self.nodes = []

    def add(self, node):
        self.nodes.append(node)

    def nearest(self, point):
        return min(self.nodes, key=lambda n: distance(n.pos, point))


class RRTConnect:
    def __init__(self, start, goal, map_size, step_size=5, max_samples=500,
                 goal_sample_rate=0.05, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.step_size = step_size
        self.max_samples = max_samples
        self.goal_sample_rate = goal_sample_rate
        self.obstacles = obstacles or []
        self.samples_taken = 0
        self.trees = [_Tree(), _Tree()]
        self.trees[0].add(self.start)
        self.trees[1].add(self.goal)
        self._swapped = False
        self.done = False
        self.last_sample = None

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.pos
        return (random.uniform(0, self.map_width), random.uniform(0, self.map_height))

    def _extend(self, tree, target_point):
        nearest = tree.nearest(target_point)
        new_point = steer(nearest.pos, target_point, self.step_size)
        new_point = clamp_point(new_point, (self.map_width, self.map_height))
        if is_collision_free(nearest.pos, new_point, self.obstacles):
            new_node = Node(*new_point, parent=nearest)
            tree.add(new_node)
            if distance(new_point, target_point) < self.step_size:
                return new_node, _Status.REACHED
            return new_node, _Status.ADVANCED
        return None, _Status.TRAPPED

    def _connect(self, tree, target_node):
        status = _Status.ADVANCED
        last_node = None
        while status == _Status.ADVANCED:
            last_node, status = self._extend(tree, target_node.pos)
        return last_node, status

    def _swap_trees(self):
        self.trees[0], self.trees[1] = self.trees[1], self.trees[0]
        self._swapped = not self._swapped

    def _unswap(self):
        if self._swapped:
            self._swap_trees()

    def get_all_nodes(self):
        return self.trees[0].nodes + self.trees[1].nodes

    def get_tree_groups(self):
        return [self.trees[0].nodes, self.trees[1].nodes]

    def step(self):
        if self.samples_taken >= self.max_samples:
            self.done = True
            return None

        x_rand = self.get_random_point()
        self.last_sample = x_rand
        new_node, status = self._extend(self.trees[0], x_rand)
        if status != _Status.TRAPPED:
            connect_node, connect_status = self._connect(self.trees[1], new_node)
            if connect_status == _Status.REACHED:
                self._unswap()
                self.done = True
                return reconstruct_bidirectional_path(new_node, connect_node)

        self._swap_trees()
        self.samples_taken += 1
        return None

    def planning(self):
        while not self.done:
            path = self.step()
            if path:
                return path
        return None
