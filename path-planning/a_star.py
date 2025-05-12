import heapq
from utils.geometry import distance, is_collision_free
from utils.plotting import plot_path

class Node:
    def __init__(self, x, y, parent=None, g=0.0, f=0.0):
        self.x = x; self.y = y
        self.parent = parent
        self.g = g; self.f = f
    def __lt__(self, other):
        return self.f < other.f

class AStar:
    def __init__(self, start, goal, map_size, obstacles=None):
        self.start = Node(*start, g=0.0, f=0.0)
        self.goal  = Node(*goal)
        self.map_width, self.map_height = map_size
        self.obstacles = obstacles or []

    def get_neighbors(self, node):
        dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        result = []
        for dx, dy in dirs:
            nx, ny = node.x+dx, node.y+dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if is_collision_free((node.x,node.y),(nx,ny),self.obstacles):
                    result.append((nx, ny))
        return result

    def planning(self):
        open_heap = []
        closed = set()
        heapq.heappush(open_heap, self.start)

        while open_heap:
            current = heapq.heappop(open_heap)
            if (current.x, current.y) == (self.goal.x, self.goal.y):
                return self._reconstruct_path(current)
            closed.add((current.x, current.y))

            for nx, ny in self.get_neighbors(current):
                if (nx, ny) in closed:
                    continue
                g_new = current.g + distance((current.x,current.y),(nx,ny))
                h_new = distance((nx,ny),(self.goal.x,self.goal.y))
                f_new = g_new + h_new
                neighbor = Node(nx, ny, parent=current, g=g_new, f=f_new)
                heapq.heappush(open_heap, neighbor)

        return None

    def _reconstruct_path(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

def main():
   
    cols, rows = 15, 10
    start = (0, 0)
    goal  = (14, 9)         
    map_size = (cols, rows)


    obstacles = [
        (4,  3, 8,  6),  
        (10, 7, 13, 9)  
    ]

    planner = AStar(start, goal, map_size, obstacles)
    path = planner.planning()
    if not path:
        print("Nenhum caminho possível nesse mapa 15×10 com 2 obstáculos.")
        return

    plot_path(
        path,
        tree=[],
        start=start,
        goal=goal,
        obstacles=obstacles
    )

if __name__ == "__main__":
    main()