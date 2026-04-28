import argparse
import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

import csv
import json
import os
import random
import statistics
import time

from algorithms.rrt import RRT
from algorithms.rrt_connect import RRTConnect
from algorithms.informed_rrt_star import InformedRRTStar
from algorithms.est import EST, HybridEST
from algorithms.rrt_merged import RRTMerge
from algorithms.rrt_star import RRTStar
from algorithms.rrt_star_smart import RRTStarSmart
from experiments.metrics import PathPlanningMetrics
from utils.plot_live import plot_live
from utils.plotting import plot_path

ALGORITHMS = {
    "rrt": RRT,
    "rrt_star": RRTStar,
    "rrt_connect": RRTConnect,
    "rrt_merged": RRTMerge,
    "informed_rrt_star": InformedRRTStar,
    "est": EST,
    "est_hybrid": HybridEST,
    "rrt_star_smart": RRTStarSmart,
}

SCENARIOS = {
    "default": {
        "start": (10, 10),
        "goal": (90, 90),
        "map_size": (100, 100),
        "obstacles": [(30, 20, 60, 40), (40, 60, 70, 80)],
    },
    "simples": {
        "start": (10, 10),
        "goal": (90, 90),
        "map_size": (100, 100),
        "obstacles": [(30, 30, 50, 50), (60, 60, 80, 80)],
    },
    "moderado": {
        "start": (10, 10),
        "goal": (90, 90),
        "map_size": (100, 100),
        "obstacles": [
            (20, 20, 40, 40),
            (50, 10, 60, 70),
            (30, 60, 70, 75),
            (70, 30, 90, 50),
        ],
    },
    "complexo": {
        "start": (10, 10),
        "goal": (90, 90),
        "map_size": (100, 100),
        "obstacles": [
            (15, 15, 25, 85),
            (35, 15, 45, 75),
            (55, 25, 65, 85),
            (75, 15, 85, 75),
            (25, 40, 90, 50),
        ],
    },
    "corredor": {
        "start": (10, 50),
        "goal": (90, 50),
        "map_size": (100, 100),
        "obstacles": [
            (0, 0, 100, 35),
            (0, 65, 100, 100),
            (30, 35, 35, 55),
            (50, 45, 55, 65),
            (70, 35, 75, 55),
        ],
    },
    "direita_reta": {
        "start": (10, 50),
        "goal": (90, 50),
        "map_size": (100, 100),
        "obstacles": [
            (0, 0, 100, 35),
            (0, 65, 100, 100),
        ],
    },
    "esquerda_reta": {
        "start": (90, 50),
        "goal": (10, 50),
        "map_size": (100, 100),
        "obstacles": [
            (0, 0, 100, 35),
            (0, 65, 100, 100),
        ],
    },
    "vazio": {
        "start": (10, 10),
        "goal": (90, 90),
        "map_size": (100, 100),
        "obstacles": [],
    },
    "passagem_estreita": {
        "start": (10, 50),
        "goal": (90, 50),
        "map_size": (100, 100),
        "obstacles": [
            (0, 0, 100, 35),
            (0, 65, 100, 100),
            (35, 35, 45, 48),
            (35, 52, 45, 65),
            (60, 35, 70, 48),
            (60, 52, 70, 65),
        ],
    },
}


def _build_kwargs(name, args):
    base = dict(step_size=args.step_size, goal_sample_rate=args.goal_sample_rate, obstacles=None)
    if name == "rrt_connect":
        base["max_samples"] = args.max_iter
    else:
        base["max_iter"] = args.max_iter
    if name == "rrt":
        base["x_direction"] = args.x_direction
    if name in {"rrt_star", "informed_rrt_star", "rrt_star_smart"}:
        base["neighbor_radius"] = args.neighbor_radius
    if name in {"est", "est_hybrid"}:
        base["density_radius"] = args.density_radius
        base["local_sample_radius"] = args.local_sample_radius
        base["density_candidates"] = args.density_candidates
        if name == "est_hybrid":
            base["global_sample_rate"] = args.global_sample_rate
    if name == "rrt_merged":
        base["connect_threshold"] = args.connect_threshold
    if name == "rrt_star_smart":
        base["beacon_sample_rate"] = args.beacon_sample_rate
        base["beacon_radius"] = args.beacon_radius
    return base


def build_planner(name, start, goal, map_size, obstacles, args):
    kwargs = _build_kwargs(name, args)
    kwargs["obstacles"] = obstacles
    return ALGORITHMS[name](start, goal, map_size, **kwargs)


def _avg_std(values):
    if not values:
        return (None, None)
    if len(values) == 1:
        return (values[0], 0.0)
    return (statistics.mean(values), statistics.stdev(values))


def _fmt(avg, std, precision):
    if avg is None:
        return "n/a"
    return f"{avg:.{precision}f} +- {std:.{precision}f}"


def main():
    parser = argparse.ArgumentParser(description="Executa multiplas rodadas de RRT.")
    parser.add_argument("--algo", choices=list(ALGORITHMS), default="rrt")
    parser.add_argument(
        "--scenario",
        choices=list(SCENARIOS),
        default="default",
        help="Cenario a usar (veja --list-scenarios para a lista completa)",
    )
    parser.add_argument("--list-scenarios", action="store_true", help="Lista os cenarios disponiveis e sai")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iter", type=int, default=500)
    parser.add_argument("--step-size", type=float, default=5)
    parser.add_argument("--goal-sample-rate", type=float, default=0.05)
    parser.add_argument("--density-radius", type=float, default=10.0)
    parser.add_argument("--local-sample-radius", type=float, default=None)
    parser.add_argument("--global-sample-rate", type=float, default=0.15)
    parser.add_argument("--density-candidates", type=int, default=40)
    parser.add_argument("--x-direction", choices=["any", "right_only", "left_only"], default="any")
    parser.add_argument("--neighbor-radius", type=float, default=15)
    parser.add_argument("--connect-threshold", type=float, default=10)
    parser.add_argument("--beacon-sample-rate", type=float, default=0.35)
    parser.add_argument("--beacon-radius", type=float, default=None)
    parser.add_argument("--visual", choices=["none", "static", "live", "both"], default="none")
    parser.add_argument(
        "--save-json",
        default=None,
        help="Arquivo JSON de saida (padrao: ../results/<algo>_<cenario>_metrics.json)",
    )
    parser.add_argument(
        "--save-csv",
        default=None,
        help="Arquivo CSV de saida (padrao: ../results/<algo>_<cenario>_metrics.csv)",
    )
    args = parser.parse_args()

    if args.x_direction != "any" and args.algo != "rrt":
        parser.error("--x-direction atualmente e suportado apenas com --algo rrt")

    if args.list_scenarios:
        print("Cenarios disponiveis:")
        for name, config in SCENARIOS.items():
            print(f"\n  {name}:")
            print(f"    Start: {config['start']}")
            print(f"    Goal: {config['goal']}")
            print(f"    Map: {config['map_size']}")
            print(f"    Obstaculos: {len(config['obstacles'])}")
        return

    scenario = SCENARIOS[args.scenario]
    start = scenario["start"]
    goal = scenario["goal"]
    map_size = scenario["map_size"]
    obstacles = scenario["obstacles"]

    print(f"Cenario: {args.scenario}")
    print(f"  Start: {start}, Goal: {goal}, Map: {map_size}")
    print(f"  Obstaculos: {len(obstacles)}")
    if args.x_direction != "any":
        print(f"  X direction: {args.x_direction}")
    print()

    metrics = PathPlanningMetrics()
    run_results = []
    successes = 0

    for i in range(args.runs):
        seed = args.seed + i
        show_live = args.visual in ("live", "both")
        show_static = args.visual in ("static", "both")

        if show_live:
            random.seed(seed)
            planner = build_planner(args.algo, start, goal, map_size, obstacles, args)
            plot_live(planner, start, goal, obstacles, title=f"{args.algo} - {args.scenario} #{i + 1}")

        random.seed(seed)
        planner = build_planner(args.algo, start, goal, map_size, obstacles, args)
        start_time = time.time()
        path = planner.planning()
        planning_time = time.time() - start_time

        if show_static:
            plot_path(
                path,
                planner.get_all_nodes(),
                start,
                goal,
                obstacles,
                map_size=map_size,
                title=f"{args.algo} - {args.scenario} #{i + 1}",
            )

        found = path is not None
        all_nodes = planner.get_all_nodes()
        num_nodes = len(all_nodes) if all_nodes is not None else 0

        run_data = {
            "run": i + 1,
            "seed": seed,
            "algorithm": args.algo,
            "scenario": args.scenario,
            "x_direction": args.x_direction,
            "success": found,
            "planning_time": planning_time,
            "num_nodes": num_nodes,
            "path_length": None,
            "path_smoothness": None,
            "clearance": None,
            "straight_line_distance": None,
            "path_efficiency": None,
        }

        if found:
            successes += 1
            path_length = metrics.path_length(path)
            straight_line = metrics.euclidean_distance(start, goal)
            run_data["path_length"] = path_length
            run_data["path_smoothness"] = metrics.path_smoothness(path)
            run_data["clearance"] = metrics.path_clearance(path, obstacles)
            run_data["straight_line_distance"] = straight_line
            run_data["path_efficiency"] = (straight_line / path_length) if path_length > 0 else None

        run_results.append(run_data)
        print(f"Execucao {i + 1}/{args.runs}: {'ok' if found else 'falhou'}")

    print(f"Sucesso: {successes}/{args.runs}")

    success_runs = [r for r in run_results if r["success"]]
    if success_runs:
        t_avg, t_std = _avg_std([r["planning_time"] for r in success_runs])
        l_avg, l_std = _avg_std([r["path_length"] for r in success_runs if r["path_length"] is not None])
        s_avg, s_std = _avg_std([r["path_smoothness"] for r in success_runs if r["path_smoothness"] is not None])
        c_avg, c_std = _avg_std([r["clearance"] for r in success_runs if r["clearance"] is not None])
        n_avg, n_std = _avg_std([r["num_nodes"] for r in success_runs])
        e_avg, e_std = _avg_std([r["path_efficiency"] for r in success_runs if r["path_efficiency"] is not None])

        print("\nResumo das metricas (apenas execucoes com sucesso):")
        print(f"  Tempo de planejamento (s): {_fmt(t_avg, t_std, 5)}")
        print(f"  Comprimento do caminho:    {_fmt(l_avg, l_std, 3)}")
        print(f"  Suavidade:                 {_fmt(s_avg, s_std, 3)}")
        print(f"  Folga minima:              {_fmt(c_avg, c_std, 3)}")
        print(f"  Numero de nos:             {_fmt(n_avg, n_std, 2)}")
        print(f"  Eficiencia (reta/caminho): {_fmt(e_avg, e_std, 3)}")

    output_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "results"))
    os.makedirs(output_dir, exist_ok=True)

    json_path = args.save_json or os.path.join(output_dir, f"{args.algo}_{args.scenario}_metrics.json")
    csv_path = args.save_csv or os.path.join(output_dir, f"{args.algo}_{args.scenario}_metrics.csv")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(run_results, f, indent=2)

    fieldnames = [
        "run",
        "seed",
        "algorithm",
        "scenario",
        "x_direction",
        "success",
        "planning_time",
        "num_nodes",
        "path_length",
        "path_smoothness",
        "clearance",
        "straight_line_distance",
        "path_efficiency",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(run_results)

    print("\nResultados detalhados salvos em:")
    print(f"  JSON: {json_path}")
    print(f"  CSV:  {csv_path}")


if __name__ == "__main__":
    main()
