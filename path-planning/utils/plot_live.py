import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def plot_live(rrt, start, goal, obstacles, interval=30):
    fig, ax = plt.subplots()
    ax.set_xlim(0, rrt.map_width)
    ax.set_ylim(0, rrt.map_height)
    ax.set_title("RRT-Connect ao Vivo")

    # Obstáculos
    for ox1, oy1, ox2, oy2 in obstacles:
        ax.add_patch(plt.Rectangle((ox1, oy1), ox2 - ox1, oy2 - oy1, color='gray'))

    # Start e Goal
    ax.plot(start[0], start[1], 'go')  
    ax.plot(goal[0], goal[1], 'ro')    

    drawn_edges = set()
    path_found = [False]  
    path_line = [None]

    def update(frame):
        if path_found[0]:
            return  

        path = rrt.step()

        # Desenhar novas arestas
        for tree in rrt.trees:
            for node in tree.nodes:
                if node.parent:
                    edge = ((node.x, node.y), (node.parent.x, node.parent.y))
                    if edge not in drawn_edges:
                        ax.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue', linewidth=0.5)
                        drawn_edges.add(edge)

        # Desenhar caminho final
        if path and not path_found[0]:
            px, py = zip(*path)
            path_line[0], = ax.plot(px, py, 'r-', linewidth=2)
            path_found[0] = True  # Marcar que já encontrou

    ani = FuncAnimation(fig, update, interval=interval, cache_frame_data=False)
    plt.show()
