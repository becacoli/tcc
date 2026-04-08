import math
import time

from utils.geometry import distance
from utils.geometry import is_collision_free


def add_controller_arguments(parser):
    parser.add_argument("--wheel-radius", type=float, default=0.0975)
    parser.add_argument("--wheel-base", type=float, default=0.381)
    parser.add_argument("--linear-speed", type=float, default=0.25)
    parser.add_argument("--angular-gain", type=float, default=3.0)
    parser.add_argument("--max-ang-speed", type=float, default=2.0)
    parser.add_argument("--heading-stop-threshold", type=float, default=0.7)
    parser.add_argument("--waypoint-tolerance", type=float, default=0.20)
    parser.add_argument("--goal-tolerance", type=float, default=0.15)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-exec-time", type=float, default=120.0)
    parser.add_argument("--lookahead-waypoints", type=int, default=3)
    parser.add_argument("--stuck-timeout", type=float, default=4.0)
    parser.add_argument("--stuck-progress-eps", type=float, default=0.02)
    parser.add_argument("--control-collision-check", action="store_true", default=True)
    parser.add_argument("--control-clearance", type=float, default=0.03)
    parser.add_argument("--tracking-clearance", type=float, default=None)
    parser.add_argument("--slowdown-clearance", type=float, default=0.08)
    parser.add_argument("--min-linear-speed-scale", type=float, default=0.35)
    parser.add_argument("--allow-zero-clearance-fallback", action="store_true", default=False)
    parser.add_argument("--max-zero-clearance-uses", type=int, default=8)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))


def set_wheel_speeds(sim, left_motor, right_motor, v, omega, wheel_radius, wheel_base):
    left_lin = v - (wheel_base * omega / 2.0)
    right_lin = v + (wheel_base * omega / 2.0)
    sim.setJointTargetVelocity(left_motor, left_lin / wheel_radius)
    sim.setJointTargetVelocity(right_motor, right_lin / wheel_radius)


def _choose_safe_target_index(pos_xy, waypoints_world, waypoint_idx, lookahead_idx, obstacles_world, clearance):
    """Escolhe o waypoint mais distante alcançável sem colisão no segmento atual->target."""
    if not obstacles_world:
        return lookahead_idx

    for idx in range(lookahead_idx, waypoint_idx - 1, -1):
        if is_collision_free(
            pos_xy,
            waypoints_world[idx],
            obstacles_world,
            clearance=clearance,
            allow_touches=True,
        ):
            return idx
    return None


def _distance_point_to_obstacle(point_xy, obstacle):
    px, py = point_xy

    # AABB obstacle
    if isinstance(obstacle, (tuple, list)) and len(obstacle) == 4:
        x_min, y_min, x_max, y_max = obstacle
        dx = max(x_min - px, 0.0, px - x_max)
        dy = max(y_min - py, 0.0, py - y_max)
        return math.hypot(dx, dy)

    # Shapely geometry
    if hasattr(obstacle, "distance"):
        try:
            from shapely.geometry import Point

            return obstacle.distance(Point(px, py))
        except Exception:
            return float("inf")

    return float("inf")


def _nearest_obstacle_distance(point_xy, obstacles_world):
    if not obstacles_world:
        return float("inf")
    return min(_distance_point_to_obstacle(point_xy, obs) for obs in obstacles_world)


def follow_world_path(
    sim,
    robot,
    left_motor,
    right_motor,
    waypoints_world,
    args,
    obstacles_world=None,
    max_exec_time_override=None,
):
    """Execute wheel control on already-planned world waypoints."""
    exec_start = time.time()
    max_exec_time = args.max_exec_time
    if max_exec_time_override is not None:
        max_exec_time = max(0.0, float(max_exec_time_override))

    waypoint_idx = 0
    last_progress_time = time.time()
    best_dist_to_current = float("inf")
    skipped_waypoints = 0
    zero_clearance_uses = 0
    tracking_clearance = args.tracking_clearance if args.tracking_clearance is not None else args.control_clearance

    while waypoint_idx < len(waypoints_world):
        if (time.time() - exec_start) > max_exec_time:
            set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)
            reason = "timeout" if max_exec_time_override is None else "window_elapsed"
            return {
                "success": False,
                "reason": reason,
                "waypoint_idx": waypoint_idx,
                "skipped_waypoints": skipped_waypoints,
                "execution_time_s": time.time() - exec_start,
            }

        # Tolerancia do waypoint corrente (para avancar no indice)
        current_x, current_y = waypoints_world[waypoint_idx]

        # Controle usa um lookahead para suavizar e evitar zig-zag em corredores
        lookahead_idx = min(
            waypoint_idx + max(0, args.lookahead_waypoints),
            len(waypoints_world) - 1,
        )

        pos = sim.getObjectPosition(robot, -1)
        ori = sim.getObjectOrientation(robot, -1)
        pos_xy = (pos[0], pos[1])

        # Se habilitado, escolhe alvo seguro para evitar cortar quina e colidir
        target_idx = lookahead_idx
        if args.control_collision_check:
            used_zero_clearance = False
            nearest_obs_dist = _nearest_obstacle_distance(pos_xy, obstacles_world)

            # Em corredor apertado, evita "cortar quina": mira no waypoint corrente.
            if nearest_obs_dist < max(0.0, 1.5 * tracking_clearance):
                lookahead_idx = waypoint_idx

            target_idx = _choose_safe_target_index(
                pos_xy,
                waypoints_world,
                waypoint_idx,
                lookahead_idx,
                obstacles_world,
                clearance=tracking_clearance,
            )

            # Fallback: relaxa margem para evitar bloqueio por conservadorismo numérico
            if (
                target_idx is None
                and tracking_clearance > 0
                and args.allow_zero_clearance_fallback
            ):
                target_idx = _choose_safe_target_index(
                    pos_xy,
                    waypoints_world,
                    waypoint_idx,
                    lookahead_idx,
                    obstacles_world,
                    clearance=0.0,
                )
                if target_idx is not None and getattr(args, "debug", False):
                    print("[control] fallback: using zero-clearance safe target")
                used_zero_clearance = target_idx is not None

            if target_idx is None:
                set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)
                return {
                    "success": False,
                    "reason": "collision_blocked",
                    "waypoint_idx": waypoint_idx,
                    "skipped_waypoints": skipped_waypoints,
                    "execution_time_s": time.time() - exec_start,
                }

            if used_zero_clearance:
                zero_clearance_uses += 1
                if zero_clearance_uses > max(0, int(args.max_zero_clearance_uses)):
                    set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)
                    return {
                        "success": False,
                        "reason": "unsafe_tracking",
                        "waypoint_idx": waypoint_idx,
                        "skipped_waypoints": skipped_waypoints,
                        "execution_time_s": time.time() - exec_start,
                    }
            else:
                zero_clearance_uses = 0

        target_x, target_y = waypoints_world[target_idx]

        # Distancia ao waypoint corrente (criterio de progresso)
        dist_to_current = math.hypot(current_x - pos_xy[0], current_y - pos_xy[1])

        # Distancia ao alvo de controle (lookahead)
        dx = target_x - pos_xy[0]
        dy = target_y - pos_xy[1]
        dist_to_target = math.hypot(dx, dy)

        # Atualiza marcador de progresso
        if dist_to_current < (best_dist_to_current - args.stuck_progress_eps):
            best_dist_to_current = dist_to_current
            last_progress_time = time.time()

        tolerance = args.goal_tolerance if waypoint_idx == len(waypoints_world) - 1 else args.waypoint_tolerance
        if dist_to_current <= tolerance:
            waypoint_idx += 1
            best_dist_to_current = float("inf")
            last_progress_time = time.time()
            continue

        # Se ficar muito tempo sem progresso, pula waypoint intermediario
        if (time.time() - last_progress_time) > args.stuck_timeout:
            if waypoint_idx < len(waypoints_world) - 1:
                waypoint_idx += 1
                skipped_waypoints += 1
                best_dist_to_current = float("inf")
                last_progress_time = time.time()
                if getattr(args, "debug", False):
                    print(f"[control] stuck recovery: skipping to waypoint {waypoint_idx}")
                continue

            # Se travou no waypoint final, encerra como timeout de controle
            set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)
            return {
                "success": False,
                "reason": "stuck_at_goal",
                "waypoint_idx": waypoint_idx,
                "skipped_waypoints": skipped_waypoints,
                "execution_time_s": time.time() - exec_start,
            }

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - ori[2])

        omega = clamp(args.angular_gain * heading_error, -args.max_ang_speed, args.max_ang_speed)
        v = args.linear_speed * max(0.0, 1.0 - abs(heading_error) / math.pi)

        # Reduz velocidade ao aproximar de obstaculos para evitar raspar parede.
        if args.control_collision_check and obstacles_world:
            nearest_obs_dist = _nearest_obstacle_distance(pos_xy, obstacles_world)
            slowdown_clearance = max(1e-6, float(args.slowdown_clearance))
            min_scale = clamp(float(args.min_linear_speed_scale), 0.0, 1.0)
            if nearest_obs_dist < slowdown_clearance:
                ratio = nearest_obs_dist / slowdown_clearance
                speed_scale = min_scale + (1.0 - min_scale) * clamp(ratio, 0.0, 1.0)
                v *= speed_scale

        if abs(heading_error) > args.heading_stop_threshold:
            v = 0.0

        set_wheel_speeds(sim, left_motor, right_motor, v, omega, args.wheel_radius, args.wheel_base)
        time.sleep(args.dt)

    set_wheel_speeds(sim, left_motor, right_motor, 0.0, 0.0, args.wheel_radius, args.wheel_base)
    final_pos = sim.getObjectPosition(robot, -1)
    final_goal = waypoints_world[-1]
    final_err = distance((final_pos[0], final_pos[1]), final_goal)

    return {
        "success": final_err <= args.goal_tolerance,
        "reason": "completed",
        "final_error_m": final_err,
        "waypoint_idx": len(waypoints_world),
        "skipped_waypoints": skipped_waypoints,
        "execution_time_s": time.time() - exec_start,
    }
