import math
import random

from algorithms.common import Node, build_path
from algorithms.rrt_star import RRTStar
from utils.geometry import clamp_point, distance, is_collision_free, steer


def _path_cost(path):
    if not path or len(path) < 2:
        return float("inf")
    return sum(distance(path[i], path[i + 1]) for i in range(len(path) - 1))


class InformedRRTStar(RRTStar):
    def __init__(self, start, goal, map_size, max_iter=500, step_size=5,
                 goal_sample_rate=0.05, obstacles=None, neighbor_radius=15):
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
        self.c_min = distance(start, goal)
        self.x_center = ((start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0)
        self.theta = math.atan2(goal[1] - start[1], goal[0] - start[0])

    def _sample_in_ellipse(self):
        radius = math.sqrt(random.random())
        angle = random.uniform(0.0, 2.0 * math.pi)
        x_ball = radius * math.cos(angle)
        y_ball = radius * math.sin(angle)

        major_axis = self.best_cost / 2.0
        focal_distance = self.c_min / 2.0
        minor_axis = math.sqrt(max(major_axis * major_axis - focal_distance * focal_distance, 0.0))

        x_local = major_axis * x_ball
        y_local = minor_axis * y_ball

        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        x_world = cos_t * x_local - sin_t * y_local + self.x_center[0]
        y_world = sin_t * x_local + cos_t * y_local + self.x_center[1]
        return clamp_point((x_world, y_world), (self.map_width, self.map_height))

    def get_random_point(self):
        if self.best_path is not None and math.isfinite(self.best_cost):
            for _ in range(20):
                point = self._sample_in_ellipse()
                if 0 <= point[0] <= self.map_width and 0 <= point[1] <= self.map_height:
                    return point
        return super().get_random_point()

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
                candidate_path = build_path(goal_node)
                candidate_cost = _path_cost(candidate_path)
                if candidate_cost < self.best_cost:
                    self.best_path = candidate_path
                    self.best_cost = candidate_cost
                    improved_path = candidate_path

        self.iterations += 1
        if self.iterations >= self.max_iter:
            self.done = True
        return improved_path

    def planning(self):
        while not self.done:
            self.step()
        return self.best_path
