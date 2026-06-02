import argparse
import os
import sys
import time
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from shapely.geometry import Polygon

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from planner_isolated import (
    add_planner_arguments,
    extract_planning_context,
    plan_path_independent,
    planner_path_to_world,
)
from wheel_controller import add_controller_arguments, follow_world_path
from utils.plotting import plot_planner_tree


def _inflate_rect(rect, inflate_by):
    x_min, y_min, x_max, y_max = rect
    return (x_min - inflate_by, y_min - inflate_by, x_max + inflate_by, y_max + inflate_by)


def build_control_obstacles_world(context, args):
    """Constrói obstáculos no mundo para validação de segurança no controlador."""
    # Nao inflar aqui: o planner ja considera robot_radius.
    # No controlador usamos apenas control_clearance para evitar dupla inflacao.
    inflate_by = 0.0
    details = context.metadata.get("obstacle_details", [])

    if getattr(args, "obstacle_model", "aabb") == "polygon" and details:
        polygons = []
        for obs in details:
            vertices = obs.get("vertices_xy", [])
            if len(vertices) >= 3:
                poly = Polygon(vertices)
                if inflate_by > 0:
                    poly = poly.buffer(inflate_by, join_style=2)
                polygons.append(poly)
        return polygons

    rects = []
    for obs in details:
        b = obs.get("raw_bounds", obs.get("bounds", {}))
        if {"x_min", "y_min", "x_max", "y_max"}.issubset(b):
            rects.append((b["x_min"], b["y_min"], b["x_max"], b["y_max"]))

    if not rects:
        rects = context.get_obstacles_as_rects()

    if inflate_by > 0:
        rects = [_inflate_rect(r, inflate_by) for r in rects]
    return rects


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Navegacao no CoppeliaSim com arquitetura isolada: "
            "(1) extracao de contexto e planejamento independente "
            "(2) controle de rodas separado"
        )
    )

    # Conexao e objetos de cena
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--robot-path", default="/PioneerP3DX")
    parser.add_argument("--left-motor-path", default="/PioneerP3DX/leftMotor")
    parser.add_argument("--right-motor-path", default="/PioneerP3DX/rightMotor")
    parser.add_argument("--goal-object", default="/GoalConfiguration")

    # Planner layer args (isolated module)
    add_planner_arguments(parser)

    # Wheel-control layer args (separate module)
    add_controller_arguments(parser)

    # Diagnostico
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--plot-planner", action="store_true")
    parser.add_argument("--plot-block", action="store_true")
    parser.add_argument("--online-replan", action="store_true", default=False)
    parser.add_argument("--replan-period", type=float, default=1.0)
    parser.add_argument("--max-replans", type=int, default=60)
    parser.add_argument("--online-max-time", type=float, default=120.0)
    return parser


def _run_online_replanning(sim, robot, left_motor, right_motor, args):
    nav_start = time.time()
    replans = 0
    total_plan_time = 0.0

    print("Connected to CoppeliaSim")
    print("Planner+Control mode: ONLINE REPLANNING")
    print(f"  algorithm: {args.algo}")
    print(f"  obstacle model: {args.obstacle_model}")
    print(f"  obstacle primitives: {args.obstacle_primitives}")
    print(f"  replan period: {args.replan_period:.2f} s")

    while replans < max(1, args.max_replans):
        elapsed = time.time() - nav_start
        if elapsed > args.online_max_time:
            print("Control layer (wheel execution):")
            print("  success: False")
            print("  reason: online_timeout")
            print(f"  execution time: {elapsed:.3f} s")
            return

        # 1) Sense: fresh world/map state from simulator
        context = extract_planning_context(sim, args)
        robot_pos = context.robot_position()
        goal_pos = context.goal_position()
        robot_goal_dist = math.hypot(goal_pos[0] - robot_pos[0], goal_pos[1] - robot_pos[1])

        if robot_goal_dist <= args.goal_tolerance:
            print("Control layer (wheel execution):")
            print("  success: True")
            print("  reason: already_at_goal")
            print(f"  execution time: {elapsed:.3f} s")
            print(f"  replans: {replans}")
            return

        # 2) Plan with updated context
        path_planner, plan_metrics = plan_path_independent(context, args)
        total_plan_time += plan_metrics["planning_time_s"]
        replans += 1

        if args.debug:
            print(
                f"[online] cycle={replans} dist_goal={robot_goal_dist:.3f}m "
                f"plan_time={plan_metrics['planning_time_s']:.3f}s "
                f"waypoints={plan_metrics.get('processed_waypoints', 0)}"
            )

        if not path_planner:
            if args.debug:
                print("[online] no path in this cycle; retrying...")
            time.sleep(min(max(args.dt, 0.01), max(args.replan_period, 0.05)))
            continue

        # 3) Act for a short window, then replan again
        waypoints_world = planner_path_to_world(path_planner, context)
        control_obstacles_world = build_control_obstacles_world(context, args)

        remaining_total = max(0.0, args.online_max_time - (time.time() - nav_start))
        control_window = min(max(args.replan_period, 0.05), remaining_total)
        if control_window <= 0.0:
            break

        control_result = follow_world_path(
            sim,
            robot,
            left_motor,
            right_motor,
            waypoints_world,
            args,
            obstacles_world=control_obstacles_world,
            max_exec_time_override=control_window,
        )

        if control_result.get("success", False):
            print("Control layer (wheel execution):")
            print("  success: True")
            print(f"  reason: {control_result['reason']}")
            print(f"  execution time: {time.time() - nav_start:.3f} s")
            print(f"  replans: {replans}")
            print(f"  total planning time: {total_plan_time:.3f} s")
            if "final_error_m" in control_result:
                print(f"  final goal error: {control_result['final_error_m']:.3f} m")
            return

        if control_result.get("reason") == "window_elapsed":
            continue

        # For blocked/stuck cases, keep looping and trying to replan until budget ends.
        if args.debug:
            print(f"[online] controller reported: {control_result.get('reason')}")

    print("Control layer (wheel execution):")
    print("  success: False")
    print("  reason: max_replans_reached")
    print(f"  execution time: {time.time() - nav_start:.3f} s")
    print(f"  replans: {replans}")
    print(f"  total planning time: {total_plan_time:.3f} s")


def main():
    args = build_parser().parse_args()

    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")

    # Garante que a simulação está rodando — sem isso os comandos de motor
    # não fazem efeito e o robô fica parado / dispara stuck recovery.
    sim_state = sim.getSimulationState()
    stopped_val = getattr(sim, "simulation_stopped", 0)
    paused_val  = getattr(sim, "simulation_paused", 8)
    if sim_state == stopped_val:
        print("[info] Simulação parada — iniciando…")
        sim.startSimulation()
        import time as _time
        _time.sleep(0.5)
    elif sim_state == paused_val:
        print("[info] Simulação pausada — retomando…")
        sim.startSimulation()  # também retoma de pausa
        import time as _time
        _time.sleep(0.3)

    robot = sim.getObject(args.robot_path)
    left_motor = sim.getObject(args.left_motor_path)
    right_motor = sim.getObject(args.right_motor_path)

    if args.online_replan:
        _run_online_replanning(sim, robot, left_motor, right_motor, args)
        return

    # 1) PLANEJAMENTO ISOLADO
    context = extract_planning_context(sim, args)
    path_planner, plan_metrics = plan_path_independent(context, args)

    print("Connected to CoppeliaSim")
    print("Planner layer (isolated):")
    print(f"  algorithm: {args.algo}")
    print(f"  obstacle model: {args.obstacle_model}")
    print(f"  obstacle primitives: {args.obstacle_primitives}")
    print(f"  robot radius: {args.robot_radius:.3f}")
    print(f"  planning time: {plan_metrics['planning_time_s']:.4f} s")
    print(f"  iterations: {plan_metrics['iterations']}")
    print(f"  obstacles: {plan_metrics['num_obstacles']}")
    if "raw_waypoints" in plan_metrics:
        print(f"  raw waypoints: {plan_metrics['raw_waypoints']}")
    if "processed_waypoints" in plan_metrics:
        print(f"  processed waypoints: {plan_metrics['processed_waypoints']}")

    if not path_planner:
        print("No path found in planner layer.")
        return

    if args.plot_planner:
        planner_obj = plan_metrics.get("planner_obj")
        tree_groups = planner_obj.get_tree_groups() if planner_obj is not None else []
        plot_planner_tree(
            path=path_planner,
            tree_groups=tree_groups,
            start=plan_metrics["start"],
            goal=plan_metrics["goal"],
            obstacles=plan_metrics.get("planner_obstacles", []),
            map_size=plan_metrics.get("map_size"),
            title=f"{args.algo.upper()} - Tree + Path",
            block=args.plot_block,
        )

    # 2) CONTROLE DE RODAS (SEPARADO)
    waypoints_world = planner_path_to_world(path_planner, context)

    if args.debug:
        print(f"Path found with {len(path_planner)} waypoints (planner coords).")
        print("First waypoints in world coords:")
        for idx, (wx, wy) in enumerate(waypoints_world[:5]):
            print(f"  {idx}: ({wx:.3f}, {wy:.3f})")

    control_obstacles_world = build_control_obstacles_world(context, args)
    control_result = follow_world_path(
        sim,
        robot,
        left_motor,
        right_motor,
        waypoints_world,
        args,
        obstacles_world=control_obstacles_world,
    )

    print("Control layer (wheel execution):")
    print(f"  success: {control_result['success']}")
    print(f"  reason: {control_result['reason']}")
    print(f"  execution time: {control_result['execution_time_s']:.3f} s")
    if "final_error_m" in control_result:
        print(f"  final goal error: {control_result['final_error_m']:.3f} m")


if __name__ == "__main__":
    main()
