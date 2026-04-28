import os
import sys
import time
import math
from shapely.geometry import Polygon

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from algorithms.informed_rrt_star import InformedRRTStar
from algorithms.est import EST, HybridEST
from algorithms.rrt import RRT
from algorithms.rrt_connect import RRTConnect
from algorithms.rrt_star import RRTStar
from algorithms.rrt_star_smart import RRTStarSmart
from planning_context import (
    create_context_from_coppelia,
    planner_to_world_coords,
    world_to_planner_coords,
)
from utils.geometry import is_collision_free


ALGORITHMS = {
    "rrt": RRT,
    "rrt_star": RRTStar,
    "rrt_connect": RRTConnect,
    "informed_rrt_star": InformedRRTStar,
    "est": EST,
    "est_hybrid": HybridEST,
    "rrt_star_smart": RRTStarSmart,
}


def add_planner_arguments(parser):
    parser.add_argument(
        "--algo",
        choices=["rrt", "rrt_star", "rrt_connect", "informed_rrt_star", "rrt_star_smart", "est", "est_hybrid"],
        default="rrt",
    )
    parser.add_argument("--max-iter", type=int, default=1000)
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--goal-sample-rate", type=float, default=0.1)
    parser.add_argument("--density-radius", type=float, default=8.0)
    parser.add_argument("--local-sample-radius", type=float, default=None)
    parser.add_argument("--global-sample-rate", type=float, default=0.15)
    parser.add_argument("--density-candidates", type=int, default=40)
    parser.add_argument("--x-direction", choices=["any", "right_only", "left_only"], default="any")
    parser.add_argument("--neighbor-radius", type=float, default=5.0)
    parser.add_argument("--beacon-sample-rate", type=float, default=0.35)
    parser.add_argument("--beacon-radius", type=float, default=None)
    parser.add_argument("--map-width", type=int, default=100)
    parser.add_argument("--map-height", type=int, default=100)
    parser.add_argument("--wall-inflate", type=float, default=0.0)
    parser.add_argument("--allow-start-in-obstacle", action="store_true", default=False)
    parser.add_argument("--plan-attempts", type=int, default=5)
    parser.add_argument("--waypoint-spacing", type=float, default=0.12)
    parser.add_argument("--obstacle-model", choices=["aabb", "polygon"], default="polygon")
    parser.add_argument("--polygon-source", choices=["bbox", "vertices"], default="bbox")
    parser.add_argument("--robot-radius", type=float, default=0.0)
    parser.add_argument(
        "--obstacle-primitives",
        choices=["cuboid", "cuboid_spheroid", "all"],
        default="cuboid",
        help="Filtro de primitivas de obstáculos extraídas do CoppeliaSim",
    )


def extract_planning_context(sim, args):
    """Extract a self-contained planning context from CoppeliaSim."""
    context = create_context_from_coppelia(
        sim=sim,
        robot_path=args.robot_path,
        goal_object=args.goal_object,
        map_size_pixels=(args.map_width, args.map_height),
        wall_inflate=args.wall_inflate,
        obstacle_primitives=getattr(args, "obstacle_primitives", "cuboid"),
        allow_start_in_obstacle=args.allow_start_in_obstacle,
    )
    return context


def _inflate_rect(rect, inflate_by):
    x_min, y_min, x_max, y_max = rect
    return (x_min - inflate_by, y_min - inflate_by, x_max + inflate_by, y_max + inflate_by)


def _rect_world_to_planner(rect_world, context):
    x_min_w, y_min_w, x_max_w, y_max_w = rect_world
    p1 = world_to_planner_coords(x_min_w, y_min_w, context)
    p2 = world_to_planner_coords(x_max_w, y_max_w, context)
    return (
        min(p1[0], p2[0]),
        min(p1[1], p2[1]),
        max(p1[0], p2[0]),
        max(p1[1], p2[1]),
    )


def _polygon_world_to_planner(vertices_world, context):
    return [world_to_planner_coords(x, y, context) for (x, y) in vertices_world]


def _build_obstacles_for_planner(context, args):
    """Build obstacle list in the selected model (aabb/polygon)."""
    model = getattr(args, "obstacle_model", "aabb")
    polygon_source = getattr(args, "polygon_source", "bbox")
    robot_radius = max(0.0, float(getattr(args, "robot_radius", 0.0)))
    robot_radius_planner = _meters_to_planner_units(context, robot_radius) if robot_radius > 0 else 0.0

    if model == "polygon":
        details = context.metadata.get("obstacle_details", [])
        polygons = []
        for obs in details:
            if polygon_source == "bbox":
                vertices = obs.get("planner_bbox_vertices_xy", [])
            else:
                vertices = obs.get("vertices_xy", [])
            if len(vertices) >= 3:
                planner_vertices = _polygon_world_to_planner(vertices, context)
                poly = Polygon(planner_vertices)
                if robot_radius_planner > 0:
                    poly = poly.buffer(robot_radius_planner, join_style=2)
                polygons.append(poly)
        if polygons:
            return polygons

    rects_world = context.get_obstacles_as_rects()
    if robot_radius > 0:
        rects_world = [_inflate_rect(r, robot_radius) for r in rects_world]
    return [_rect_world_to_planner(r, context) for r in rects_world]


def _show_detailed_obstacle_info(obstacles, context, args):
    """Show detailed obstacle bounds and spacing info for debugging."""
    print(f"\n[OBSTACLE SPACING DEBUG]")
    print(f"  Robot radius (planner units): {_meters_to_planner_units(context, float(getattr(args, 'robot_radius', 0.0))):.2f}")
    
    if obstacles and hasattr(obstacles[0], 'bounds'):
        # Polygon mode
        bounds_list = [obs.bounds for obs in obstacles]
        bounds_list.sort(key=lambda b: b[0])  # Sort by x_min
        
        for i, bounds in enumerate(bounds_list):
            x_min, y_min, x_max, y_max = bounds
            w, h = x_max - x_min, y_max - y_min
            print(f"    Obs {i}: x=[{x_min:.1f}, {x_max:.1f}] y=[{y_min:.1f}, {y_max:.1f}] size {w:.1f}x{h:.1f}")
        
        # Show gaps between vertical walls
        for i in range(len(bounds_list) - 1):
            gap = bounds_list[i+1][0] - bounds_list[i][2]
            if gap > 0:
                print(f"    Gap between obs {i} and {i+1}: {gap:.1f} pixels")
    elif obstacles and isinstance(obstacles[0], tuple):
        # AABB mode
        for i, rect in enumerate(obstacles):
            x_min, y_min, x_max, y_max = rect
            w, h = x_max - x_min, y_max - y_min
            print(f"    Obs {i}: x=[{x_min:.1f}, {x_max:.1f}] y=[{y_min:.1f}, {y_max:.1f}] size {w:.1f}x{h:.1f}")


def _meters_to_planner_units(context, spacing_m):
    """Converte espaçamento em metros para unidades do mapa do planner."""
    world_w = context.world_bounds.max_x - context.world_bounds.min_x
    world_h = context.world_bounds.max_y - context.world_bounds.min_y
    map_w, map_h = context.map_size_pixels

    if world_w <= 0 or world_h <= 0:
        return spacing_m

    scale_x = map_w / world_w
    scale_y = map_h / world_h
    scale = 0.5 * (scale_x + scale_y)
    return spacing_m * scale


def plan_path_independent(context, args):
    """Run planner independently of control/state-estimation/mapping stack."""
    start = world_to_planner_coords(*context.robot_position(), context)
    goal = world_to_planner_coords(*context.goal_position(), context)
    obstacles = _build_obstacles_for_planner(context, args)
    map_size = context.map_size_pixels

    # DEBUG: Show map state for diagnostics
    if hasattr(args, 'debug') and args.debug:
        print(f"\n[PLANNER DEBUG]")
        print(f"  Map bounds: (0, 0) to {map_size}")
        print(f"  Start (planner): {start}")
        print(f"  Goal (planner): {goal}")
        print(f"  Obstacle count: {len(obstacles) if obstacles else 0}")
        if obstacles and isinstance(obstacles[0], tuple):
            # AABB mode
            for i, obs in enumerate(obstacles[:3]):
                x_min, y_min, x_max, y_max = obs
                w, h = x_max - x_min, y_max - y_min
                print(f"    Obstacle {i}: ({x_min:.1f}, {y_min:.1f}) size {w:.1f}x{h:.1f}")
        elif obstacles:
            # Polygon mode - show bounds
            for i, poly in enumerate(obstacles):
                bounds = poly.bounds  # (minx, miny, maxx, maxy)
                w, h = bounds[2] - bounds[0], bounds[3] - bounds[1]
                print(f"    Obstacle {i}: ({bounds[0]:.1f}, {bounds[1]:.1f}) size {w:.1f}x{h:.1f}")
        
        # Show detailed spacing info
        _show_detailed_obstacle_info(obstacles, context, args)


    algo_name = args.algo
    if algo_name not in ALGORITHMS:
        raise ValueError(f"Unsupported algorithm: {algo_name}")

    x_direction = getattr(args, "x_direction", "any")
    if x_direction != "any" and algo_name != "rrt":
        raise ValueError("x_direction is currently supported only for the rrt algorithm")

    algo_cls = ALGORITHMS[algo_name]
    t0 = time.time()

    kwargs = {
        "step_size": args.step_size,
        "goal_sample_rate": args.goal_sample_rate,
        "obstacles": obstacles,
    }

    if algo_name == "rrt":
        kwargs["x_direction"] = x_direction

    if algo_name == "rrt_connect":
        kwargs["max_samples"] = args.max_iter
    else:
        kwargs["max_iter"] = args.max_iter

    if algo_name in {"rrt_star", "informed_rrt_star", "rrt_star_smart"}:
        kwargs["neighbor_radius"] = args.neighbor_radius

    if algo_name in {"est", "est_hybrid"}:
        kwargs["density_radius"] = args.density_radius
        kwargs["local_sample_radius"] = args.local_sample_radius
        kwargs["density_candidates"] = args.density_candidates
        if algo_name == "est_hybrid":
            kwargs["global_sample_rate"] = args.global_sample_rate

    if algo_name == "rrt_star_smart":
        kwargs["beacon_sample_rate"] = args.beacon_sample_rate
        kwargs["beacon_radius"] = args.beacon_radius

    planner = None
    path_planner = None
    attempts = max(1, int(getattr(args, "plan_attempts", 1)))
    raw_waypoints = 0
    processed_waypoints = 0
    for _ in range(attempts):
        planner = algo_cls(start, goal, map_size, **kwargs)
        candidate = planner.planning()
        if candidate and path_is_collision_free(candidate, obstacles):
            # Alguns planejadores bidirecionais podem retornar goal->start.
            # Normaliza para start->goal para o controlador seguir corretamente.
            if _path_starts_near_goal(candidate, start, goal):
                candidate = list(reversed(candidate))

            raw_waypoints = len(candidate)
            spacing_m = max(0.03, float(getattr(args, "waypoint_spacing", 0.12)))
            spacing_planner = max(0.5, _meters_to_planner_units(context, spacing_m))
            path_planner = postprocess_path(
                candidate,
                obstacles,
                waypoint_spacing=spacing_planner,
            )
            processed_waypoints = len(path_planner) if path_planner else 0
            break

    planning_time = time.time() - t0

    iterations = getattr(planner, "iterations", getattr(planner, "samples_taken", 0))
    metrics = {
        "planning_time_s": planning_time,
        "iterations": iterations,
        "num_obstacles": len(obstacles),
        "plan_attempts": attempts,
        "raw_waypoints": raw_waypoints,
        "processed_waypoints": processed_waypoints,
        "start": start,
        "goal": goal,
        "map_size": map_size,
        "planner_obstacles": obstacles,
        "planner_obj": planner,
    }
    return path_planner, metrics


def planner_path_to_world(path_planner, context):
    return [planner_to_world_coords(px, py, context) for (px, py) in path_planner]


def _path_starts_near_goal(path, start, goal):
    if not path:
        return False
    start_dist_first = math.hypot(path[0][0] - start[0], path[0][1] - start[1])
    start_dist_last = math.hypot(path[-1][0] - start[0], path[-1][1] - start[1])
    goal_dist_first = math.hypot(path[0][0] - goal[0], path[0][1] - goal[1])
    goal_dist_last = math.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])

    # Invertido quando o primeiro ponto está mais próximo do goal do que do start,
    # e o último está mais próximo do start.
    return (goal_dist_first < start_dist_first) and (start_dist_last < goal_dist_last)


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
