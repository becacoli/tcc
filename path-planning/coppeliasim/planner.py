import math
import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from algorithms.informed_rrt_star import InformedRRTStar
from algorithms.rrt import RRT
from algorithms.rrt_connect import RRTConnect
from algorithms.rrt_star import RRTStar
from algorithms.rrt_star_smart import RRTStarSmart
from utils.geometry import is_collision_free

ALGORITHMS = {
    "rrt": RRT,
    "rrt_star": RRTStar,
    "rrt_connect": RRTConnect,
    "informed_rrt_star": InformedRRTStar,
    "rrt_star_smart": RRTStarSmart,
}


def world_to_planner(x, y, map_size, world_bounds):
    map_w, map_h = map_size
    min_x, max_x, min_y, max_y = world_bounds
    px = (x - min_x) * map_w / (max_x - min_x)
    py = (y - min_y) * map_h / (max_y - min_y)
    return (px, py)


def planner_to_world(px, py, map_size, world_bounds):
    map_w, map_h = map_size
    min_x, max_x, min_y, max_y = world_bounds
    x = min_x + (px / map_w) * (max_x - min_x)
    y = min_y + (py / map_h) * (max_y - min_y)
    return (x, y)


def normalize_alias(alias):
    if not alias:
        return ""
    return alias.split("/")[-1]


def wall_name_matches(alias, prefix):
    return normalize_alias(alias).lower().startswith(prefix.lower())


def transform_point(matrix, point):
    x, y, z = point
    return (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )


def get_local_bbox_size(sim, handle):
    min_x = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x)
    min_y = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y)
    max_x = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x)
    max_y = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y)
    return (max_x - min_x, max_y - min_y)


def rect_from_handle_world(sim, handle):
    pos = sim.getObjectPosition(handle, -1)
    ori = sim.getObjectOrientation(handle, -1)
    yaw = ori[2]
    size_x, size_y = get_local_bbox_size(sim, handle)

    half_x = 0.5 * (abs(math.cos(yaw)) * size_x + abs(math.sin(yaw)) * size_y)
    half_y = 0.5 * (abs(math.sin(yaw)) * size_x + abs(math.cos(yaw)) * size_y)

    x_min = pos[0] - half_x
    x_max = pos[0] + half_x
    y_min = pos[1] - half_y
    y_max = pos[1] + half_y
    return (x_min, y_min, x_max, y_max)


def try_get_object_by_alias(sim, alias):
    try:
        return sim.getObject(f"/{alias}")
    except Exception:
        return -1


def compute_world_bounds_from_handle(sim, handle, margin=0.0):
    x_min, y_min, x_max, y_max = rect_from_handle_world(sim, handle)
    return (
        x_min + margin,
        x_max - margin,
        y_min + margin,
        y_max - margin,
    )


def point_inside_rect(point, rect):
    x, y = point
    x_min, y_min, x_max, y_max = rect
    return (x_min <= x <= x_max) and (y_min <= y <= y_max)


def inflate_obstacles(obstacles, inflate_by, map_size):
    if inflate_by <= 0:
        return obstacles
    map_w, map_h = map_size
    inflated = []
    for (x_min, y_min, x_max, y_max) in obstacles:
        ix_min = max(0.0, x_min - inflate_by)
        iy_min = max(0.0, y_min - inflate_by)
        ix_max = min(map_w, x_max + inflate_by)
        iy_max = min(map_h, y_max + inflate_by)
        inflated.append((ix_min, iy_min, ix_max, iy_max))
    return inflated


def read_scene_walls_as_obstacles(sim, map_size, world_bounds, walls_prefix):
    obstacles = []
    labels = []
    shape_handles = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)

    for handle in shape_handles:
        alias = sim.getObjectAlias(handle)
        if not wall_name_matches(alias, walls_prefix):
            continue

        x_min_w, y_min_w, x_max_w, y_max_w = rect_from_handle_world(sim, handle)
        p1 = world_to_planner(x_min_w, y_min_w, map_size, world_bounds)
        p2 = world_to_planner(x_max_w, y_max_w, map_size, world_bounds)

        x_min = min(p1[0], p2[0])
        y_min = min(p1[1], p2[1])
        x_max = max(p1[0], p2[0])
        y_max = max(p1[1], p2[1])

        obstacles.append((x_min, y_min, x_max, y_max))
        labels.append(normalize_alias(alias))

    return obstacles, labels


def _component_boxes_from_shape_viz(sim, handle):
    viz = sim.getShapeViz(handle, 0)
    vertices = viz.get("vertices") or []
    indices = viz.get("indices") or []
    if not vertices or not indices:
        return []

    matrix = sim.getObjectMatrix(handle, -1)
    points = [
        transform_point(matrix, (vertices[i], vertices[i + 1], vertices[i + 2]))
        for i in range(0, len(vertices), 3)
    ]
    adjacency = [set() for _ in range(len(points))]

    for i in range(0, len(indices), 3):
        tri = [int(indices[i]), int(indices[i + 1]), int(indices[i + 2])]
        for idx in tri:
            adjacency[idx].update(tri)

    visited = set()
    boxes = []
    for start_idx in range(len(points)):
        if start_idx in visited:
            continue

        stack = [start_idx]
        visited.add(start_idx)
        component = []
        while stack:
            idx = stack.pop()
            component.append(points[idx])
            for nxt in adjacency[idx]:
                if nxt not in visited:
                    visited.add(nxt)
                    stack.append(nxt)

        xs = [p[0] for p in component]
        ys = [p[1] for p in component]
        if xs and ys:
            boxes.append((min(xs), min(ys), max(xs), max(ys)))
    return boxes


def read_obstacle_object_as_rects(sim, object_path, map_size, world_bounds):
    handle = sim.getObject(object_path)
    try:
        world_rects = _component_boxes_from_shape_viz(sim, handle)
    except Exception:
        world_rects = []

    if not world_rects:
        world_rects = [rect_from_handle_world(sim, handle)]

    obstacles = []
    labels = []
    alias = normalize_alias(sim.getObjectAlias(handle))
    for x_min_w, y_min_w, x_max_w, y_max_w in world_rects:
        p1 = world_to_planner(x_min_w, y_min_w, map_size, world_bounds)
        p2 = world_to_planner(x_max_w, y_max_w, map_size, world_bounds)
        obstacles.append(
            (
                min(p1[0], p2[0]),
                min(p1[1], p2[1]),
                max(p1[0], p2[0]),
                max(p1[1], p2[1]),
            )
        )
        labels.append(alias)
    return obstacles, labels


def resolve_goal_from_object(sim, goal_object_path, map_size, world_bounds):
    if not goal_object_path:
        return None
    try:
        goal_handle = sim.getObject(goal_object_path)
    except Exception:
        return None

    goal_pos = sim.getObjectPosition(goal_handle, -1)
    return world_to_planner(goal_pos[0], goal_pos[1], map_size, world_bounds)


def plan_path(start, goal, map_size, obstacles, args):
    algo_name = getattr(args, "algo", "rrt_star")
    if algo_name not in ALGORITHMS:
        raise ValueError(f"Unsupported algorithm: {algo_name}")

    kwargs = {
        "step_size": args.step_size,
        "goal_sample_rate": args.goal_sample_rate,
        "obstacles": obstacles,
    }
    if algo_name == "rrt_connect":
        kwargs["max_samples"] = args.max_iter
    else:
        kwargs["max_iter"] = args.max_iter
    if algo_name in {"rrt_star", "informed_rrt_star", "rrt_star_smart"}:
        kwargs["neighbor_radius"] = args.neighbor_radius
    if algo_name == "rrt_star_smart":
        kwargs["beacon_sample_rate"] = getattr(args, "beacon_sample_rate", 0.35)
        kwargs["beacon_radius"] = getattr(args, "beacon_radius", None)

    planner = ALGORITHMS[algo_name](start, goal, map_size, **kwargs)
    return planner.planning()


def path_is_collision_free(path, obstacles):
    if not path or len(path) < 2:
        return False
    for i in range(len(path) - 1):
        if not is_collision_free(path[i], path[i + 1], obstacles):
            return False
    return True


def shortcut_path(path, obstacles):
    if not path:
        return path

    shortened = [path[0]]
    anchor_idx = 0
    while anchor_idx < len(path) - 1:
        next_idx = len(path) - 1
        while next_idx > anchor_idx + 1:
            if is_collision_free(path[anchor_idx], path[next_idx], obstacles):
                break
            next_idx -= 1
        shortened.append(path[next_idx])
        anchor_idx = next_idx
    return shortened


def densify_path(path, max_spacing):
    if not path or len(path) < 2:
        return path

    dense_path = [path[0]]
    for idx in range(len(path) - 1):
        start = path[idx]
        end = path[idx + 1]
        segment_length = math.hypot(end[0] - start[0], end[1] - start[1])
        num_segments = max(1, int(math.ceil(segment_length / max_spacing)))
        for step in range(1, num_segments + 1):
            t = step / num_segments
            dense_path.append(
                (
                    start[0] + (end[0] - start[0]) * t,
                    start[1] + (end[1] - start[1]) * t,
                )
            )
    return dense_path


def postprocess_path(path, obstacles, waypoint_spacing):
    if not path:
        return path

    simplified = shortcut_path(path, obstacles)
    densified = densify_path(simplified, waypoint_spacing)
    return densified if path_is_collision_free(densified, obstacles) else path


def plan_collision_free_path(start, goal, map_size, obstacles, args):
    for _ in range(max(1, args.plan_attempts)):
        path = plan_path(start, goal, map_size, obstacles, args)
        if path and path_is_collision_free(path, obstacles):
            return postprocess_path(path, obstacles, waypoint_spacing=max(args.step_size * 0.5, 0.08))
    return None
