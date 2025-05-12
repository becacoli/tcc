import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_path(path, tree, start, goal, obstacles=[]):
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_aspect('equal')

    # Desenhar obstáculos
    for obs in obstacles:
        x_min, y_min, x_max, y_max = obs
        rect = patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, linewidth=1, edgecolor='k', facecolor='gray')
        ax.add_patch(rect)

    # Desenhar árvore
    for node in tree:
        if node.parent is not None:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='lightblue', linewidth=0.8)

    # Desenhar caminho
    if path:
        x_vals, y_vals = zip(*path)
        plt.plot(x_vals, y_vals, color='red', linewidth=2, label="Caminho")

    # Início e objetivo
    plt.plot(start[0], start[1], "go", markersize=10, label="Início")
    plt.plot(goal[0], goal[1], "ro", markersize=10, label="Objetivo")

    plt.title("RRT - Planejamento de Caminho com Obstáculos")
    plt.legend()
    plt.grid(True)
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.show()
