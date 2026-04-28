import math
from shapely.geometry import LineString, Point, box


def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def respects_x_direction(p1, p2, mode="any", eps=1e-9):
    dx = p2[0] - p1[0]
    if mode == "right_only":
        return dx >= -eps
    if mode == "left_only":
        return dx <= eps
    return True


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


def is_collision_free(p1, p2, obstacles=None, clearance=0.0, allow_touches=False):
    if not obstacles:
        return True
    for obs in obstacles:
        if line_intersects_rect(p1, p2, obs, clearance=clearance, allow_touches=allow_touches):
            return False
    return True


def line_intersects_rect(p1, p2, rect, clearance=0.0, allow_touches=False):
    line = LineString([p1, p2])

    def _hits_geom(geom):
        if allow_touches:
            return line.crosses(geom) or line.within(geom) or line.overlaps(geom) or geom.contains(line)
        return line.intersects(geom)

    # Obstáculo representado como retângulo (AABB)
    if isinstance(rect, (tuple, list)) and len(rect) == 4:
        x_min, y_min, x_max, y_max = rect
        if clearance > 0:
            x_min -= clearance
            y_min -= clearance
            x_max += clearance
            y_max += clearance

        rect_shape = box(x_min, y_min, x_max, y_max)
        return _hits_geom(rect_shape)

    # Obstáculo representado como geometria Shapely (Polygon/Buffered)
    if hasattr(rect, "intersects"):
        geom = rect.buffer(clearance, join_style=2) if clearance > 0 else rect
        return _hits_geom(geom)

    raise TypeError("Obstacle format not supported for collision check")

def _obstacle_to_geometry(obstacle, clearance=0.0):
    """Converte AABB ou geometria Shapely em uma geometria com clearance opcional."""
    if isinstance(obstacle, (tuple, list)) and len(obstacle) == 4:
        x_min, y_min, x_max, y_max = obstacle
        geom = box(x_min, y_min, x_max, y_max)
    elif hasattr(obstacle, "intersects"):
        geom = obstacle
    else:
        raise TypeError("Obstacle format not supported for collision check")

    if clearance > 0:
        return geom.buffer(clearance, join_style=2)
    return geom


def is_point_collision_free(point, obstacles=None, clearance=0.0, allow_touches=False):
    """Verifica se uma configuração pontual está fora dos obstáculos.

    Essa função deixa explícita a hipótese usada nas provas de planejamento:
    uma configuração q pertence a X_free quando não intersecta nenhum obstáculo.
    """
    if not obstacles:
        return True

    p = Point(point[0], point[1])
    for obs in obstacles:
        geom = _obstacle_to_geometry(obs, clearance=clearance)
        if allow_touches:
            if geom.contains(p):
                return False
        else:
            if geom.intersects(p) or geom.contains(p):
                return False
    return True


def minimum_distance_to_obstacles(point, obstacles=None):
    """Retorna a menor distância de um ponto aos obstáculos."""
    if not obstacles:
        return float("inf")

    p = Point(point[0], point[1])
    distances = []
    for obs in obstacles:
        geom = _obstacle_to_geometry(obs, clearance=0.0)
        distances.append(p.distance(geom))
    return min(distances) if distances else float("inf")

