import math
from shapely.geometry import LineString, box


def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def steer(from_point, to_point, step_size):
    dx = to_point[0] - from_point[0]
    dy = to_point[1] - from_point[1]
    dist = math.hypot(dx, dy)
    if dist == 0:
        return (from_point[0], from_point[1])
    if dist <= step_size:
        return (to_point[0], to_point[1])
    scale = step_size / dist
    return (from_point[0] + dx * scale, from_point[1] + dy * scale)


def clamp_point(point, map_size):
    x = min(max(point[0], 0), map_size[0])
    y = min(max(point[1], 0), map_size[1])
    return (x, y)


def is_collision_free(p1, p2, obstacles=None):
    if not obstacles:
        return True
    for obs in obstacles:
        if line_intersects_rect(p1, p2, obs):
            return False
    return True


def line_intersects_rect(p1, p2, rect):
    line = LineString([p1, p2])
    rect_shape = box(*rect)
    return line.intersects(rect_shape)
