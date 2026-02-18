import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from utils.plotting import draw_obstacles

# Cores para cada árvore (até 2 árvores)
_TREE_COLORS = ["#1f77b4", "#ff7f0e"]  # azul, laranja
_TREE_LABELS = ["Arvore inicio", "Arvore objetivo"]
_NODE_COLORS = ["#4a90d9", "#ffaa44"]


def plot_live(planner, start, goal, obstacles, interval=30, title="RRT ao Vivo"):
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_xlim(-2, planner.map_width + 2)
    ax.set_ylim(-2, planner.map_height + 2)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    draw_obstacles(ax, obstacles)

    ax.plot(*start, "go", markersize=12, zorder=5, label="Inicio")
    ax.plot(*goal, "r*", markersize=14, zorder=5, label="Objetivo")

    tree_groups = planner.get_tree_groups()
    is_bidirectional = len(tree_groups) > 1

    drawn_edges = set()
    prev_node_count = [0]
    path_found = [False]
    title_obj = ax.set_title(f"{title}  |  iter 0", fontsize=13)

    # Marcadores temporários (amostra + nós novos)
    temp_artists = []
    # Amostras acumuladas (ficam no gráfico como pontos pequenos)
    sample_scatter_x = []
    sample_scatter_y = []
    sample_plot = [None]

    def update(_frame):
        nonlocal temp_artists

        if path_found[0]:
            return

        # Apagar destaques temporários do frame anterior
        for art in temp_artists:
            art.remove()
        temp_artists = []

        path = planner.step()

        # Mostrar ponto amostrado (X amarelo grande = amostra atual)
        sample = getattr(planner, "last_sample", None)
        if sample:
            sample_scatter_x.append(sample[0])
            sample_scatter_y.append(sample[1])
            # Amostra atual: X amarelo grande temporário
            dot, = ax.plot(
                sample[0], sample[1], "x",
                color="#FFD700", markersize=8, markeredgewidth=2,
                alpha=0.9, zorder=3,
            )
            temp_artists.append(dot)
            # Acumular amostras passadas como pontos pequenos
            if sample_plot[0]:
                sample_plot[0].remove()
            sample_plot[0] = ax.scatter(
                sample_scatter_x, sample_scatter_y,
                c="#FFA500", s=6, alpha=0.3, zorder=1, label="_nolegend_",
            )

        # Iterar sobre cada grupo de árvore
        tree_groups_now = planner.get_tree_groups()
        for gi, group in enumerate(tree_groups_now):
            color = _TREE_COLORS[gi % 2]
            node_color = _NODE_COLORS[gi % 2]
            for node in group:
                if node.parent:
                    edge = (node.pos, node.parent.pos)
                    if edge not in drawn_edges:
                        ax.plot(
                            [node.x, node.parent.x],
                            [node.y, node.parent.y],
                            color=color, linewidth=0.6, alpha=0.7,
                        )
                        # Ponto do nó novo (destaque)
                        dot, = ax.plot(
                            node.x, node.y, "o",
                            color=node_color, markersize=4, alpha=0.9,
                        )
                        temp_artists.append(dot)
                        drawn_edges.add(edge)

        # Atualizar contador de iterações no título
        iters = getattr(planner, "iterations", getattr(planner, "samples_taken", 0))
        total = len(planner.get_all_nodes())
        title_obj.set_text(f"{title}  |  iter {iters}  |  {total} nos")

        # Caminho encontrado
        if path:
            px, py = zip(*path)
            ax.plot(px, py, "r-", linewidth=2.5, zorder=4, label="Caminho")
            ax.plot(px, py, "wo", markersize=3, zorder=4)
            path_found[0] = True
            title_obj.set_text(f"{title}  |  CAMINHO ENCONTRADO  |  {len(path)} pontos")
            ax.legend(loc="upper left", fontsize=9)

        if planner.done and not path_found[0]:
            title_obj.set_text(f"{title}  |  SEM CAMINHO apos {iters} iteracoes")
            ani.event_source.stop()

    # Legenda inicial para árvores bidirecionais
    if is_bidirectional:
        for gi in range(len(tree_groups)):
            ax.plot([], [], color=_TREE_COLORS[gi % 2], linewidth=2,
                    label=_TREE_LABELS[gi])
    else:
        ax.plot([], [], color=_TREE_COLORS[0], linewidth=2, label="Arvore")
    ax.plot([], [], "x", color="#FFD700", markersize=8, markeredgewidth=2,
            label="Amostra atual")
    ax.scatter([], [], c="#FFA500", s=12, alpha=0.5, label="Amostras anteriores")
    ax.legend(loc="upper left", fontsize=9)

    ani = FuncAnimation(fig, update, interval=interval, cache_frame_data=False)
    plt.show()
