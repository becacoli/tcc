from algorithms.rrt_connect import RRTConnect
from utils.plot_live import plot_live
from utils.plotting import plot_path

start = (10, 10)
goal = (90, 90)
map_size = (100, 100)
step_size = 5
max_samples = 500
goal_sample_rate = 0.05

obstacles = [
    (30, 20, 60, 40),
    (40, 60, 70, 80),
]

# Modo live — usa step() internamente
rrt_live = RRTConnect(start, goal, map_size, step_size=step_size,
                      max_samples=max_samples, goal_sample_rate=goal_sample_rate,
                      obstacles=obstacles)
plot_live(rrt_live, start, goal, obstacles, title="RRT-Connect ao Vivo")

# Modo estático
rrt = RRTConnect(start, goal, map_size, step_size=step_size,
                 max_samples=max_samples, goal_sample_rate=goal_sample_rate,
                 obstacles=obstacles)
path = rrt.planning()
print("Caminho encontrado!" if path else "Caminho não encontrado.")
plot_path(path, rrt.get_all_nodes(), start, goal, obstacles,
          map_size=map_size, title="RRT-Connect")
