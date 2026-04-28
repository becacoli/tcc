import math
import random

from algorithms.common import Node, build_path
from utils.geometry import (
    clamp_point,
    distance,
    is_collision_free,
    is_point_collision_free,
    steer,
)


class EST:
    """Ideia central:
    - escolher preferencialmente nós em regiões menos densas;
    - gerar uma amostra local próxima ao nó escolhido;
    - adicionar o novo nó quando a conexão local estiver livre de colisão.

    O parâmetro global_sample_rate permite uma versão híbrida:
    com probabilidade global_sample_rate, o algoritmo realiza uma expansão global
    semelhante ao RRT. Isso facilita a discussão de completude probabilística,
    pois garante amostragem com suporte em todo o espaço de configuração.
    """

    def __init__(
        self,
        start,
        goal,
        map_size,
        max_iter=500,
        step_size=5,
        goal_sample_rate=0.05,
        obstacles=None,
        density_radius=10.0,
        local_sample_radius=None,
        global_sample_rate=0.0,
        density_candidates=40,
    ):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_width, self.map_height = map_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.obstacles = obstacles or []
        self.density_radius = density_radius
        self.local_sample_radius = local_sample_radius or step_size
        self.global_sample_rate = global_sample_rate
        self.density_candidates = max(1, int(density_candidates))

        self.tree = [self.start]
        self.iterations = 0
        self.done = False
        self.last_sample = None

    def get_all_nodes(self):
        return self.tree

    def get_tree_groups(self):
        return [self.tree]

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return self.goal.pos
        return (random.uniform(0, self.map_width), random.uniform(0, self.map_height))

    def _nearest(self, point):
        return min(self.tree, key=lambda n: distance(n.pos, point))

    def _density(self, node):
        """Conta vizinhos em torno do nó para estimar densidade local."""
        return sum(
            1
            for other in self.tree
            if other is not node and distance(node.pos, other.pos) <= self.density_radius
        )

    def _select_node_by_density(self):
        """Seleciona nós de baixa densidade com maior probabilidade.

        Para manter o custo baixo, estima a densidade em um subconjunto
        aleatório da árvore. Isso preserva a ideia do EST e deixa os
        experimentos viáveis com milhares de iterações.
        """
        if len(self.tree) <= self.density_candidates:
            candidates = self.tree
        else:
            candidates = random.sample(self.tree, self.density_candidates)

        weights = []
        for node in candidates:
            density = self._density(node)
            weights.append(1.0 / (1.0 + density))

        total = sum(weights)
        if total <= 0:
            return random.choice(candidates)

        threshold = random.random() * total
        cumulative = 0.0
        for node, weight in zip(candidates, weights):
            cumulative += weight
            if cumulative >= threshold:
                return node
        return candidates[-1]

    def _sample_near(self, node):
        """Gera amostra local em disco ao redor do nó selecionado."""
        angle = random.uniform(0.0, 2.0 * math.pi)
        # sqrt deixa a distribuição aproximadamente uniforme na área do disco.
        radius = self.local_sample_radius * math.sqrt(random.random())
        sample = (
            node.x + radius * math.cos(angle),
            node.y + radius * math.sin(angle),
        )
        return clamp_point(sample, (self.map_width, self.map_height))

    def _try_add(self, parent, candidate):
        candidate = clamp_point(candidate, (self.map_width, self.map_height))
        if not is_point_collision_free(candidate, self.obstacles):
            return None
        if not is_collision_free(parent.pos, candidate, self.obstacles):
            return None

        new_node = Node(
            candidate[0],
            candidate[1],
            parent=parent,
            cost=parent.cost + distance(parent.pos, candidate),
        )
        self.tree.append(new_node)
        return new_node

    def _expand_global(self):
        q_rand = self.get_random_point()
        self.last_sample = q_rand
        q_near = self._nearest(q_rand)
        q_new = steer(q_near.pos, q_rand, self.step_size)
        return self._try_add(q_near, q_new)

    def _expand_est(self):
        # A meta é usada como viés ocasional, não como amostra local pura.
        # Usar o nó mais próximo da meta nesse caso acelera a convergência
        # sem remover a exploração por baixa densidade do EST.
        if random.random() < self.goal_sample_rate:
            q_near = self._nearest(self.goal.pos)
            q_new = steer(q_near.pos, self.goal.pos, self.step_size)
        else:
            q_near = self._select_node_by_density()
            q_new = self._sample_near(q_near)

        self.last_sample = q_new
        return self._try_add(q_near, q_new)

    def _try_connect_goal(self, node):
        if distance(node.pos, self.goal.pos) > self.step_size:
            return None

        if (
            is_point_collision_free(self.goal.pos, self.obstacles)
            and is_collision_free(node.pos, self.goal.pos, self.obstacles)
        ):
            goal_node = Node(
                self.goal.x,
                self.goal.y,
                parent=node,
                cost=node.cost + distance(node.pos, self.goal.pos),
            )
            self.done = True
            return build_path(goal_node)
        return None

    def step(self):
        if self.iterations >= self.max_iter:
            self.done = True
            return None

        if random.random() < self.global_sample_rate:
            new_node = self._expand_global()
        else:
            new_node = self._expand_est()

        self.iterations += 1

        if new_node is None:
            return None

        return self._try_connect_goal(new_node)

    def planning(self):
        while not self.done:
            path = self.step()
            if path:
                return path
        return None


class HybridEST(EST):
    """EST com componente de amostragem global. Existe uma chance fixa
    de amostragem global, além da expansão por baixa densidade do EST.
    """

    def __init__(self, *args, global_sample_rate=0.15, **kwargs):
        super().__init__(*args, global_sample_rate=global_sample_rate, **kwargs)
