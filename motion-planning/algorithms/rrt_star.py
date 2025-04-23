import random
from utils.geometry import distance, steer, is_collision_free

class Node:
    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost

class RRTStar:
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5,
                 goal_sample_rate=0.05, obstacles=None, neighbor_radius=15):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]
        self.obstacles = obstacles if obstacles else []
        self.neighbor_radius = neighbor_radius

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (random.randint(0, self.map_width), random.randint(0, self.map_height))

    def get_nearest_node(self, point):
        return min(self.tree, key=lambda node: distance((node.x, node.y), point))

    def get_nearby_nodes(self, new_node):
        radius = self.neighbor_radius
        nearby = [node for node in self.tree if distance((node.x, node.y), (new_node.x, new_node.y)) <= radius]
        return nearby

    def planning(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest = self.get_nearest_node(rand_point)
            new_point = steer((nearest.x, nearest.y), rand_point, self.step_size)

            if not is_collision_free((nearest.x, nearest.y), new_point, self.obstacles):
                continue

            new_node = Node(*new_point)
            new_node.parent = nearest
            new_node.cost = nearest.cost + distance((nearest.x, nearest.y), new_point)

            nearby_nodes = self.get_nearby_nodes(new_node)

            for neighbor in nearby_nodes:
                if is_collision_free((neighbor.x, neighbor.y), new_point, self.obstacles):
                    temp_cost = neighbor.cost + distance((neighbor.x, neighbor.y), new_point)
                    if temp_cost < new_node.cost:
                        new_node.parent = neighbor
                        new_node.cost = temp_cost

            self.tree.append(new_node)

            # Rewire
            for neighbor in nearby_nodes:
                if neighbor == new_node.parent:
                    continue
                if is_collision_free((new_node.x, new_node.y), (neighbor.x, neighbor.y), self.obstacles):
                    temp_cost = new_node.cost + distance((new_node.x, new_node.y), (neighbor.x, neighbor.y))
                    if temp_cost < neighbor.cost:
                        neighbor.parent = new_node
                        neighbor.cost = temp_cost

            if distance((new_node.x, new_node.y), (self.goal.x, self.goal.y)) < self.step_size:
                if is_collision_free((new_node.x, new_node.y), (self.goal.x, self.goal.y), self.obstacles):
                    goal_node = Node(self.goal.x, self.goal.y, parent=new_node)
                    return self.generate_path(goal_node)

        return None

    def generate_path(self, node):
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]
