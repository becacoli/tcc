import random

from algorithms.common import Node, build_path
from algorithms.rrt_star import RRTStar
from utils.geometry import clamp_point, distance, is_collision_free, steer


def _path_cost(path):
    if not path or len(path) < 2:
        return float("inf")
    return sum(distance(path[i], path[i + 1]) for i in range(len(path) - 1))


class RRTStarSmart(RRTStar):
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5,
                 goal_sample_rate=0.05, obstacles=None, neighbor_radius=15,
                 beacon_sample_rate=0.35, beacon_radius=None):
        super().__init__(
            start=start,
            goal=goal,
            map_size=map_size,
            max_iter=max_iter,
            step_size=step_size,
            goal_sample_rate=goal_sample_rate,
            obstacles=obstacles,
            neighbor_radius=neighbor_radius,
        )
        self.best_path = None
        self.best_cost = float("inf")
        self.beacon_sample_rate = beacon_sample_rate
        self.beacon_radius = beacon_radius if beacon_radius is not None else step_size * 2.0
        self.beacons = []

    def _sample_near_beacon(self):
        beacon = random.choice(self.beacons)
        sampled = (
            random.uniform(beacon[0] - self.beacon_radius, beacon[0] + self.beacon_radius),
            random.uniform(beacon[1] - self.beacon_radius, beacon[1] + self.beacon_radius),
        )
        return clamp_point(sampled, (self.map_width, self.map_height))

    def get_random_point(self):
        if self.beacons and random.random() < self.beacon_sample_rate:
            return self._sample_near_beacon()
        return super().get_random_point()

    def _shortcut_path(self, path):
        if not path:
            return path

        optimized = [path[0]]
        anchor_idx = 0
        while anchor_idx < len(path) - 1:
            next_idx = len(path) - 1
            while next_idx > anchor_idx + 1:
                if is_collision_free(path[anchor_idx], path[next_idx], self.obstacles):
                    break
                next_idx -= 1
            optimized.append(path[next_idx])
            anchor_idx = next_idx
        return optimized

    def _update_solution(self, path):
        optimized = self._shortcut_path(path)
        optimized_cost = _path_cost(optimized)
        if optimized_cost < self.best_cost:
            self.best_path = optimized
            self.best_cost = optimized_cost
            self.beacons = optimized[1:-1]
            return optimized
        return None

    def step(self):
        if self.iterations >= self.max_iter:
            self.done = True
            return None

        rand_point = self.get_random_point()
        self.last_sample = rand_point
        nearest = self._nearest(rand_point)
        new_point = steer(nearest.pos, rand_point, self.step_size)
        new_point = clamp_point(new_point, (self.map_width, self.map_height))

        if not is_collision_free(nearest.pos, new_point, self.obstacles):
            self.iterations += 1
            return None

        new_node = Node(*new_point, parent=nearest,
                        cost=nearest.cost + distance(nearest.pos, new_point))

        nearby = self._nearby(new_node)
        self._choose_best_parent(new_node, nearby)
        self.tree.append(new_node)
        self._rewire(new_node, nearby)

        improved_path = None
        if distance(new_node.pos, self.goal.pos) <= self.step_size:
            if is_collision_free(new_node.pos, self.goal.pos, self.obstacles):
                goal_node = Node(
                    self.goal.x,
                    self.goal.y,
                    parent=new_node,
                    cost=new_node.cost + distance(new_node.pos, self.goal.pos),
                )
                improved_path = self._update_solution(build_path(goal_node))

        self.iterations += 1
        if self.iterations >= self.max_iter:
            self.done = True
        return improved_path

    def planning(self):
        while not self.done:
            self.step()
        return self.best_path
