import heapq
from utils.geometry import distance, is_collision_free

class Node:
    def __init__(self, x, y, parent=None, g=0.0, f=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.g = g 
        self.f = f  # custo total estimado (g + h)

    def __lt__(self, other):  # necessário para a fila de prioridade
        return self.f < other.f

class AStar:
    def __init__(self, start, goal, map_size, obstacles=None):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.obstacles = obstacles if obstacles else []

    def get_neighbors(self, node):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]
        neighbors = []
        for dx, dy in directions:
            nx, ny = node.x + dx, node.y + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if is_collision_free((node.x, node.y), (nx, ny), self.obstacles):
                    neighbors.append((nx, ny))
        return neighbors

    def planning(self):
        open_list = []
        closed_set = set()
        heapq.heappush(open_list, self.start)

        while open_list:
            current = heapq.heappop(open_list)
            if (current.x, current.y) == (self.goal.x, self.goal.y):
                return self.generate_path(current)

            closed_set.add((current.x, current.y))

            for nx, ny in self.get_neighbors(current):
                if (nx, ny) in closed_set:
                    continue

                g = current.g + distance((current.x, current.y), (nx, ny))
                h = distance((nx, ny), (self.goal.x, self.goal.y))
                neighbor = Node(nx, ny, parent=current, g=g, f=g + h)

                heapq.heappush(open_list, neighbor)

        return None

    def generate_path(self, node):
        path = []
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]
