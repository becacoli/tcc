import random
from utils.geometry import distance, steer, is_collision_free

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRTMerge:
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5, goal_sample_rate=0.05, connect_threshold=10, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate 
        self.tree_start = [self.start]
        self.tree_goal = [self.goal]
        self.connect_threshold = connect_threshold
        self.obstacles = obstacles if obstacles else []

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (random.randint(0, self.map_width), random.randint(0, self.map_height))

    def get_nearest_node(self, tree, point):
        return min(tree, key=lambda node: distance((node.x, node.y), point))

    def extend(self, tree, point):
        nearest = self.get_nearest_node(tree, point)
        new_point = steer((nearest.x, nearest.y), point, self.step_size)

        if is_collision_free((nearest.x, nearest.y), new_point, self.obstacles):
            new_node = Node(*new_point, parent=nearest)
            tree.append(new_node)
            return new_node
        return None

    def check_connection(self, new_node, other_tree):
        for node in other_tree:
            if distance((new_node.x, new_node.y), (node.x, node.y)) < self.connect_threshold:
                if is_collision_free((new_node.x, new_node.y), (node.x, node.y), self.obstacles):
                    return node
        return None

    def planning(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            new_node = self.extend(self.tree_start, rand_point)
            if new_node:
                matched_node = self.check_connection(new_node, self.tree_goal)
                if matched_node:
                    print("Conectado via merge passivo!")
                    return self.generate_path(new_node, matched_node)

            self.tree_start, self.tree_goal = self.tree_goal, self.tree_start

        return None

    def generate_path(self, node_start, node_goal):
        path_start = []
        while node_start is not None:
            path_start.append((node_start.x, node_start.y))
            node_start = node_start.parent
        path_goal = []
        while node_goal is not None:
            path_goal.append((node_goal.x, node_goal.y))
            node_goal = node_goal.parent
        return path_start[::-1] + path_goal
