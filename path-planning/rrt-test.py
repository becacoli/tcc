from algorithms.rrt_connect import RRTConnect
from utils.plotting import plot_path  
from utils.plot_live import plot_live
# Configuração
start = (10, 10)
goal = (90, 90)
map_size = (100, 100)
step_size = 5
max_samples = 500
goal_sample_rate = 0.05

obstacles = [
    (30, 20, 60, 40),
    (40, 60, 70, 80)
]

rrt = RRTConnect(start, goal, map_size, step_size=step_size,
                 max_samples=max_samples, goal_sample_rate=goal_sample_rate,
                 obstacles=obstacles)

path = rrt.plan()

if path:
    print("Caminho encontrado!")
else:
    print("Caminho não encontrado.")


# plot_path(path, rrt.trees[0].nodes + rrt.trees[1].nodes, start, goal, obstacles)

plot_live(rrt, start, goal, obstacles)
