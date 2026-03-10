import argparse
import math
import os
import sys
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from experiments.run_rrt_batch import SCENARIOS
from utils.geometry import distance

from planner import (
    compute_world_bounds_from_handle,
    inflate_obstacles,
    plan_collision_free_path,
    planner_to_world,
    point_inside_rect,
    read_scene_walls_as_obstacles,
    resolve_goal_from_object,
    try_get_object_by_alias,
    world_to_planner,
)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def build_parser():
    parser = argparse.ArgumentParser(
        description="Plan with RRT* and follow waypoints in CoppeliaSim (PioneerP3DX)."
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)

    parser.add_argument("--robot-path", default="/PioneerP3DX")
    parser.add_argument("--left-motor-path", default="/PioneerP3DX/leftMotor")
    parser.add_argument("--right-motor-path", default="/PioneerP3DX/rightMotor")

    parser.add_argument("--scenario", choices=list(SCENARIOS) + ["none"], default="none")
    parser.add_argument(
        "--map-width",
        type=float,
        default=None,
        help="Planner map width. If omitted in scenario=none, uses floor/world width in meters.",
    )
    parser.add_argument(
        "--map-height",
        type=float,
        default=None,
        help="Planner map height. If omitted in scenario=none, uses floor/world height in meters.",
    )

    parser.add_argument(
        "--goal-x",
        type=float,
        default=1.25,
        help="Goal X in world meters (same frame as Coppelia floor). Prefer --goal-world-x/--goal-world-y.",
    )
    parser.add_argument(
        "--goal-y",
        type=float,
        default=-1.60,
        help="Goal Y in world meters (same frame as Coppelia floor). Prefer --goal-world-x/--goal-world-y.",
    )
    parser.add_argument("--goal-world-x", type=float, default=None, help="Goal X in Coppelia world meters.")
    parser.add_argument("--goal-world-y", type=float, default=None, help="Goal Y in Coppelia world meters.")
    parser.add_argument(
        "--goal-object",
        default="",
        help="CoppeliaSim object path to use as goal in scenario=none (e.g. /Sphere). Use '' to disable.",
    )

    parser.add_argument("--world-min-x", type=float, default=-5.0)
    parser.add_argument("--world-max-x", type=float, default=5.0)
    parser.add_argument("--world-min-y", type=float, default=-5.0)
    parser.add_argument("--world-max-y", type=float, default=5.0)
    parser.add_argument("--walls-prefix", default="wall_")
    parser.add_argument("--use-scene-walls", action="store_true", default=True)
    parser.add_argument("--no-scene-walls", action="store_false", dest="use_scene_walls")
    parser.add_argument("--floor-alias", default="floor")
    parser.add_argument("--auto-world-bounds", action="store_true", default=True)
    parser.add_argument("--no-auto-world-bounds", action="store_false", dest="auto_world_bounds")
    parser.add_argument(
        "--world-margin",
        type=float,
        default=0.0,
        help="Margin (meters) removed from floor bounds when auto-world-bounds is enabled.",
    )

    parser.add_argument("--max-iter", type=int, default=5000)
    parser.add_argument("--step-size", type=float, default=0.12)
    parser.add_argument("--goal-sample-rate", type=float, default=0.1)
    parser.add_argument("--neighbor-radius", type=float, default=0.8)
    parser.add_argument(
        "--inflate",
        type=float,
        default=0.22,
        help="Obstacle inflation in planner units (meters when using world-scale).",
    )

    parser.add_argument("--wheel-radius", type=float, default=0.0975)
    parser.add_argument("--wheel-base", type=float, default=0.381)
    parser.add_argument("--linear-speed", type=float, default=0.25)
    parser.add_argument("--angular-gain", type=float, default=3.0)
    parser.add_argument("--max-ang-speed", type=float, default=2.0)
    parser.add_argument("--waypoint-tolerance", type=float, default=0.2)
    parser.add_argument("--goal-tolerance", type=float, default=0.15)
    parser.add_argument("--dt", type=float, default=0.05)

    parser.add_argument(
        "--stuck-timeout",
        type=float,
        default=2.0,
        help="Seconds without progress before replanning.",
    )
    parser.add_argument(
        "--stuck-distance-eps",
        type=float,
        default=0.03,
        help="Minimum progress distance in meters to reset stuck timer.",
    )
    parser.add_argument("--max-replans", type=int, default=5)
    parser.add_argument(
        "--plan-attempts",
        type=int,
        default=5,
        help="How many full planning retries to attempt until a collision-free path is found.",
    )
    return parser


def compute_obstacles_and_goal(args):
    if args.scenario == "none":
        map_size = (1.0, 1.0)
        obstacles = []
        goal = (args.goal_x, args.goal_y)
    else:
        cfg = SCENARIOS[args.scenario]
        obstacles = cfg["obstacles"]
        goal = cfg["goal"]
        map_size = cfg["map_size"]
    return map_size, obstacles, goal


def set_wheel_speeds(sim, left_motor, right_motor, v, omega, wheel_radius, wheel_base):
    left_lin = v - (wheel_base * omega / 2.0)
    right_lin = v + (wheel_base * omega / 2.0)

    left_rad_s = left_lin / wheel_radius
    right_rad_s = right_lin / wheel_radius

    sim.setJointTargetVelocity(left_motor, left_rad_s)
    sim.setJointTargetVelocity(right_motor, right_rad_s)


def main():
    args = build_parser().parse_args()

    world_bounds = (args.world_min_x, args.world_max_x, args.world_min_y, args.world_max_y)
    scenario_map_size, scenario_obstacles, goal = compute_obstacles_and_goal(args)

    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")

    robot = sim.getObject(args.robot_path)
    left_motor = sim.getObject(args.left_motor_path)
    right_motor = sim.getObject(args.right_motor_path)

    if args.auto_world_bounds:
        floor_handle = try_get_object_by_alias(sim, args.floor_alias)
        if floor_handle != -1:
            auto_bounds = compute_world_bounds_from_handle(
                sim,
                floor_handle,
                margin=args.world_margin,
            )
            if (auto_bounds[1] - auto_bounds[0]) > 0 and (auto_bounds[3] - auto_bounds[2]) > 0:
                world_bounds = auto_bounds
            else:
                print("Warning: invalid bounds from floor. Keeping manual world bounds.")
        else:
            print(
                f"Warning: floor '{args.floor_alias}' not found. "
                "Keeping manual world bounds."
            )

    # Default: planner uses same physical scale as CoppeliaSim world in scenario=none.
    world_map_size = (world_bounds[1] - world_bounds[0], world_bounds[3] - world_bounds[2])
    if args.map_width is not None and args.map_height is not None:
        map_size = (args.map_width, args.map_height)
    elif args.scenario == "none":
        map_size = world_map_size
    else:
        map_size = scenario_map_size

    if args.scenario == "none":
        if args.goal_world_x is not None and args.goal_world_y is not None:
            goal = world_to_planner(args.goal_world_x, args.goal_world_y, map_size, world_bounds)
        else:
            goal = world_to_planner(args.goal_x, args.goal_y, map_size, world_bounds)
            goal_from_object = resolve_goal_from_object(sim, args.goal_object, map_size, world_bounds)
            if goal_from_object is not None:
                goal = goal_from_object

    obstacles = scenario_obstacles
    wall_names = []
    if args.use_scene_walls:
        try:
            scene_obstacles, wall_names = read_scene_walls_as_obstacles(
                sim=sim,
                map_size=map_size,
                world_bounds=world_bounds,
                walls_prefix=args.walls_prefix,
            )
            if scene_obstacles:
                obstacles = scene_obstacles
        except Exception as exc:
            print(f"Warning: failed to read scene walls ({exc}). Using scenario obstacles.")

    obstacles = inflate_obstacles(obstacles, args.inflate, map_size)

    robot_pos = sim.getObjectPosition(robot, -1)
    start = world_to_planner(robot_pos[0], robot_pos[1], map_size, world_bounds)

    print("Connected to CoppeliaSim")
    print(f"  robot: {args.robot_path}")
    print(
        "  world bounds: "
        f"x[{world_bounds[0]:.2f}, {world_bounds[1]:.2f}] "
        f"y[{world_bounds[2]:.2f}, {world_bounds[3]:.2f}]"
    )
    print(f"  map size (planner): ({map_size[0]:.2f}, {map_size[1]:.2f})")
    print(f"  start (planner): ({start[0]:.2f}, {start[1]:.2f})")
    print(f"  goal  (planner): ({goal[0]:.2f}, {goal[1]:.2f})")
    if args.scenario == "none" and args.goal_world_x is not None and args.goal_world_y is not None:
        print(f"  goal (world): ({args.goal_world_x:.2f}, {args.goal_world_y:.2f})")
    elif args.scenario == "none" and args.goal_object == "":
        print(f"  goal (world): ({args.goal_x:.2f}, {args.goal_y:.2f})")
    if args.scenario == "none" and args.goal_object:
        print(f"  goal object: {args.goal_object}")
    print(f"  obstacles: {len(obstacles)}")
    print(f"  obstacle inflate: {args.inflate:.2f}")
    if wall_names:
        print(f"  scene walls detected: {', '.join(sorted(wall_names))}")

    if any(point_inside_rect(start, obs) for obs in obstacles):
        print("Warning: start is inside an obstacle.")
    if any(point_inside_rect(goal, obs) for obs in obstacles):
        print("Warning: goal is inside an obstacle.")

    path = plan_collision_free_path(start, goal, map_size, obstacles, args)
    if not path:
        print("No path found with current parameters.")
        return

    waypoints_world = [planner_to_world(px, py, map_size, world_bounds) for (px, py) in path]
    print(f"Path found with {len(path)} waypoints.")

    try:
        replans = 0
        waypoint_index = 0
        prev_pos = sim.getObjectPosition(robot, -1)
        last_progress_time = time.time()

        while waypoint_index < len(waypoints_world):
            wx, wy = waypoints_world[waypoint_index]
            pos = sim.getObjectPosition(robot, -1)
            ori = sim.getObjectOrientation(robot, -1)

            step_progress = distance((prev_pos[0], prev_pos[1]), (pos[0], pos[1]))
            if step_progress > args.stuck_distance_eps:
                last_progress_time = time.time()
            prev_pos = pos

            current_planner = world_to_planner(pos[0], pos[1], map_size, world_bounds)
            if any(point_inside_rect(current_planner, obs) for obs in obstacles):
                last_progress_time = 0.0

            dx = wx - pos[0]
            dy = wy - pos[1]
            d = math.hypot(dx, dy)

            tol = args.goal_tolerance if waypoint_index == len(waypoints_world) - 1 else args.waypoint_tolerance
            if d <= tol:
                waypoint_index += 1
                continue

            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - ori[2])

            omega = clamp(args.angular_gain * heading_error, -args.max_ang_speed, args.max_ang_speed)
            v = args.linear_speed * max(0.0, 1.0 - abs(heading_error) / math.pi)
            if abs(heading_error) > 1.2:
                v *= 0.5

            set_wheel_speeds(
                sim,
                left_motor,
                right_motor,
                v,
                omega,
                args.wheel_radius,
                args.wheel_base,
            )
            time.sleep(args.dt)

            if time.time() - last_progress_time > args.stuck_timeout:
                if replans >= args.max_replans:
                    print("Stuck detected and max replans reached. Stopping.")
                    break

                replans += 1
                print(f"Stuck detected. Replanning... ({replans}/{args.max_replans})")
                current_pos = sim.getObjectPosition(robot, -1)
                start = world_to_planner(current_pos[0], current_pos[1], map_size, world_bounds)
                path = plan_collision_free_path(start, goal, map_size, obstacles, args)
                if not path:
                    print("Replan failed: no path found.")
                    break

                waypoints_world = [planner_to_world(px, py, map_size, world_bounds) for (px, py) in path]
                waypoint_index = 0
                prev_pos = current_pos
                last_progress_time = time.time()
                print(f"Replan success. New path with {len(path)} waypoints.")

        set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)

        final_pos = sim.getObjectPosition(robot, -1)
        final_goal_world = waypoints_world[-1]
        final_err = distance((final_pos[0], final_pos[1]), final_goal_world)
        print(f"Done. Final goal error: {final_err:.3f} m")

    except KeyboardInterrupt:
        set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)
        print("Interrupted by user. Robot stopped.")


if __name__ == "__main__":
    main()
