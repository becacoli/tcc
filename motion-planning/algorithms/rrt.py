import random
from utils.geometry import distance, steer, is_collision_free

# Iniciar grafo

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT:
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5, goal_sample_rate=0.05, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]
        self.obstacles = obstacles if obstacles else []

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (random.randint(0, self.map_width), random.randint(0, self.map_height))

    def get_nearest_node(self, point):
        return min(self.tree, key=lambda node: distance((node.x, node.y), point))

    def planning(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest = self.get_nearest_node(rand_point)
            new_point = steer((nearest.x, nearest.y), rand_point, self.step_size)

            if is_collision_free((nearest.x, nearest.y), new_point, self.obstacles):
                new_node = Node(*new_point, parent=nearest)
                self.tree.append(new_node)

                if distance((new_node.x, new_node.y), (self.goal.x, self.goal.y)) < self.step_size:
                    print("Objetivo alcançado!")
                    return self.generate_path(new_node)

        return None 

    def generate_path(self, node):
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]  
