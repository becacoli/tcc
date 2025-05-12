import enum
import random
from utils.geometry import steer, distance, is_collision_free

class Status(enum.Enum):
    FAILED = 1
    TRAPPED = 2
    ADVANCED = 3
    REACHED = 4


class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


class Tree:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def get_nearest_node(self, point):
        return min(self.nodes, key=lambda node: distance((node.x, node.y), point))


class RRTBase:
    def __init__(self, start, goal, map_size, step_size=5, max_samples=500, goal_sample_rate=0.05, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.step_size = step_size
        self.max_samples = max_samples
        self.goal_sample_rate = goal_sample_rate
        self.obstacles = obstacles if obstacles else []
        self.samples_taken = 0
        self.trees = [Tree(), Tree()]
        self.trees[0].add_node(self.start)
        self.trees[1].add_node(self.goal)
        self.swapped = False

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (random.randint(0, self.map_width), random.randint(0, self.map_height))

    def extend(self, tree, x_rand):
        nearest = tree.get_nearest_node(x_rand)
        new_point = steer((nearest.x, nearest.y), x_rand, self.step_size)
        if is_collision_free((nearest.x, nearest.y), new_point, self.obstacles):
            new_node = Node(*new_point, parent=nearest)
            tree.add_node(new_node)
            if distance(new_point, x_rand) < self.step_size:
                return new_node, Status.REACHED
            return new_node, Status.ADVANCED
        return None, Status.TRAPPED

    def connect(self, tree, target):
        status = Status.ADVANCED
        last_node = None
        while status == Status.ADVANCED:
            last_node, status = self.extend(tree, (target.x, target.y))
        return last_node, status

    def swap_trees(self):
        self.trees[0], self.trees[1] = self.trees[1], self.trees[0]
        self.swapped = not self.swapped

    def unswap(self):
        if self.swapped:
            self.swap_trees()

    def reconstruct_path(self, node_start, node_goal):
        path_start = []
        while node_start:
            path_start.append((node_start.x, node_start.y))
            node_start = node_start.parent
        path_goal = []
        while node_goal:
            path_goal.append((node_goal.x, node_goal.y))
            node_goal = node_goal.parent
        return path_start[::-1] + path_goal


class RRTConnect(RRTBase):
    def __init__(self, start, goal, map_size, step_size=5, max_samples=500, goal_sample_rate=0.05, obstacles=None):
        super().__init__(start, goal, map_size, step_size, max_samples, goal_sample_rate, obstacles)
        
    def step(self):
        if self.samples_taken >= self.max_samples:
            return None  # Fim das iterações

        x_rand = self.get_random_point()
        new_node, status = self.extend(self.trees[0], x_rand)
        if status != Status.TRAPPED:
            connect_node, connect_status = self.connect(self.trees[1], new_node)
            if connect_status == Status.REACHED:
                self.unswap()
                return self.reconstruct_path(new_node, connect_node)

        self.swap_trees()
        self.samples_taken += 1
        return None

    def plan(self):
        for _ in range(self.max_samples):
            x_rand = self.get_random_point()
            new_node, status = self.extend(self.trees[0], x_rand)
            if status != Status.TRAPPED:
                connect_node, connect_status = self.connect(self.trees[1], new_node)
                if connect_status == Status.REACHED:
                    self.unswap()
                    return self.reconstruct_path(new_node, connect_node)
            self.swap_trees()
        return None
