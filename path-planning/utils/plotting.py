import matplotlib.patches as patches
import matplotlib.pyplot as plt


def draw_obstacles(ax, obstacles):
    for obs in obstacles or []:
        if isinstance(obs, tuple) and len(obs) == 4:
            x_min, y_min, x_max, y_max = obs
            rect = patches.Rectangle(
                (x_min, y_min), x_max - x_min, y_max - y_min,
                linewidth=1, edgecolor="k", facecolor="gray", alpha=0.35,
            )
            ax.add_patch(rect)
            continue

        # Shapely Polygon-like object
        if hasattr(obs, "exterior") and obs.exterior is not None:
            coords = list(obs.exterior.coords)
            poly = patches.Polygon(coords, closed=True, linewidth=1, edgecolor="k", facecolor="gray", alpha=0.35)
            ax.add_patch(poly)


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


def plot_planner_tree(path, tree_groups, start, goal, obstacles=None, map_size=None,
                      title="Planner Tree", block=True):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)

    draw_obstacles(ax, obstacles)

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    if not tree_groups:
        tree_groups = []

    for gi, nodes in enumerate(tree_groups):
        color = colors[gi % len(colors)]
        for node in nodes:
            if getattr(node, "parent", None) is not None:
                ax.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    color=color,
                    linewidth=0.7,
                    alpha=0.65,
                )

    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color="red", linewidth=2.2, label="Caminho")

    ax.plot(*start, "go", markersize=9, label="Inicio")
    ax.plot(*goal, "r*", markersize=12, label="Objetivo")
    ax.set_title(title)
    ax.legend(loc="upper left")

    if map_size:
        ax.set_xlim(0, map_size[0])
        ax.set_ylim(0, map_size[1])

    if block:
        plt.show()
    else:
        plt.show(block=False)
        plt.pause(0.1)
