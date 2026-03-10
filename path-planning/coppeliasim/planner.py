import math
import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from algorithms.rrt_star import RRTStar
from utils.geometry import is_collision_free


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
    planner = RRTStar(
        start=start,
        goal=goal,
        map_size=map_size,
        max_iter=args.max_iter,
        step_size=args.step_size,
        goal_sample_rate=args.goal_sample_rate,
        obstacles=obstacles,
        neighbor_radius=args.neighbor_radius,
    )
    return planner.planning()


def path_is_collision_free(path, obstacles):
    if not path or len(path) < 2:
        return False
    for i in range(len(path) - 1):
        if not is_collision_free(path[i], path[i + 1], obstacles):
            return False
    return True


def plan_collision_free_path(start, goal, map_size, obstacles, args):
    for _ in range(max(1, args.plan_attempts)):
        path = plan_path(start, goal, map_size, obstacles, args)
        if path and path_is_collision_free(path, obstacles):
            return path
    return None
