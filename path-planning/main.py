from algorithms.rrt_star import RRTStar
from utils.plotting import plot_path

start = (10, 10)
goal = (90, 90)
map_size = (100, 100)
obstacles = [
    (30, 20, 60, 40),
    (40, 60, 70, 80),
]

rrt_star = RRTStar(
    start, goal, map_size,
    max_iter=500, step_size=5, goal_sample_rate=0.05,
    obstacles=obstacles, neighbor_radius=15,
)

path = rrt_star.planning()
print("Caminho encontrado!" if path else "Caminho não encontrado.")

plot_path(path, rrt_star.get_all_nodes(), start, goal, obstacles,
          map_size=map_size, title="RRT*")

