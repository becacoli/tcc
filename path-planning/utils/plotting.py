import matplotlib.patches as patches
import matplotlib.pyplot as plt


def draw_obstacles(ax, obstacles):
    for x_min, y_min, x_max, y_max in obstacles or []:
        rect = patches.Rectangle(
            (x_min, y_min), x_max - x_min, y_max - y_min,
            linewidth=1, edgecolor="k", facecolor="gray",
        )
        ax.add_patch(rect)


def plot_path(path, nodes, start, goal, obstacles=None, map_size=None,
              title="RRT - Planejamento de Caminho", block=True):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")

    draw_obstacles(ax, obstacles)

    for node in nodes:
        if node.parent is not None:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y],
                    color="lightblue", linewidth=0.8)

    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color="red", linewidth=2, label="Caminho")

    ax.plot(*start, "go", markersize=10, label="Inicio")
    ax.plot(*goal, "ro", markersize=10, label="Objetivo")

    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    if map_size:
        ax.set_xlim(0, map_size[0])
        ax.set_ylim(0, map_size[1])
    
    if block:
        plt.show()  # Bloqueante - espera fechar
    else:
        plt.show(block=False)  # Não-bloqueante - mostra e continua
        plt.pause(0.1)  # Pequena pausa para renderizar
