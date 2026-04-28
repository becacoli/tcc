import argparse
import os
import sys

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

import csv
import random
import statistics
import time

from algorithms.est import EST, HybridEST
from algorithms.informed_rrt_star import InformedRRTStar
from algorithms.rrt import RRT
from algorithms.rrt_connect import RRTConnect
from algorithms.rrt_star import RRTStar
from experiments.metrics import PathPlanningMetrics
from experiments.run_rrt_batch import SCENARIOS


ALGORITHMS = {
    "rrt": RRT,
    "rrt_connect": RRTConnect,
    "rrt_star": RRTStar,
    "informed_rrt_star": InformedRRTStar,
    "est": EST,
    "est_hybrid": HybridEST,
}


def build_kwargs(name, args):
    kwargs = {
        "step_size": args.step_size,
        "goal_sample_rate": args.goal_sample_rate,
        "obstacles": None,
    }
    if name == "rrt_connect":
        kwargs["max_samples"] = args.max_iter
    else:
        kwargs["max_iter"] = args.max_iter

    if name in {"rrt_star", "informed_rrt_star"}:
        kwargs["neighbor_radius"] = args.neighbor_radius

    if name in {"est", "est_hybrid"}:
        kwargs["density_radius"] = args.density_radius
        kwargs["local_sample_radius"] = args.local_sample_radius
        kwargs["density_candidates"] = args.density_candidates
        if name == "est_hybrid":
            kwargs["global_sample_rate"] = args.global_sample_rate

    return kwargs


def mean_std(values):
    if not values:
        return None, None
    if len(values) == 1:
        return values[0], 0.0
    return statistics.mean(values), statistics.stdev(values)


def run_one(name, scenario_name, args):
    scenario = SCENARIOS[scenario_name]
    start = scenario["start"]
    goal = scenario["goal"]
    map_size = scenario["map_size"]
    obstacles = scenario["obstacles"]

    metrics = PathPlanningMetrics()
    rows = []

    for run_idx in range(args.runs):
        seed = args.seed + run_idx
        random.seed(seed)

        kwargs = build_kwargs(name, args)
        kwargs["obstacles"] = obstacles

        planner = ALGORITHMS[name](start, goal, map_size, **kwargs)

        t0 = time.time()
        path = planner.planning()
        elapsed = time.time() - t0

        success = path is not None
        row = {
            "algorithm": name,
            "scenario": scenario_name,
            "run": run_idx + 1,
            "seed": seed,
            "success": success,
            "planning_time": elapsed,
            "num_nodes": len(planner.get_all_nodes()),
            "path_length": None,
            "path_smoothness": None,
            "clearance": None,
            "path_efficiency": None,
        }

        if success:
            length = metrics.path_length(path)
            straight = metrics.euclidean_distance(start, goal)
            row["path_length"] = length
            row["path_smoothness"] = metrics.path_smoothness(path)
            row["clearance"] = metrics.path_clearance(path, obstacles)
            row["path_efficiency"] = straight / length if length > 0 else None

        rows.append(row)

    return rows


def summarize(rows):
    success_rows = [r for r in rows if r["success"]]
    success_rate = len(success_rows) / len(rows) if rows else 0.0

    summary = {
        "algorithm": rows[0]["algorithm"],
        "scenario": rows[0]["scenario"],
        "runs": len(rows),
        "success_rate": success_rate,
    }

    for key in ["planning_time", "num_nodes", "path_length", "path_smoothness", "clearance", "path_efficiency"]:
        values = [r[key] for r in success_rows if r[key] is not None]
        avg, std = mean_std(values)
        summary[f"{key}_mean"] = avg
        summary[f"{key}_std"] = std

    return summary


def main():
    parser = argparse.ArgumentParser(description="Executa matriz de comparação para o TCC.")
    parser.add_argument("--algorithms", nargs="+", default=["rrt", "rrt_connect", "rrt_star", "est", "est_hybrid"])
    parser.add_argument("--scenarios", nargs="+", default=["simples", "moderado", "corredor", "passagem_estreita"])
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iter", type=int, default=2000)
    parser.add_argument("--step-size", type=float, default=5)
    parser.add_argument("--goal-sample-rate", type=float, default=0.05)
    parser.add_argument("--neighbor-radius", type=float, default=15)
    parser.add_argument("--density-radius", type=float, default=10)
    parser.add_argument("--local-sample-radius", type=float, default=5)
    parser.add_argument("--global-sample-rate", type=float, default=0.15)
    parser.add_argument("--density-candidates", type=int, default=40)
    parser.add_argument("--raw-csv", default="../results/matrix_raw.csv")
    parser.add_argument("--summary-csv", default="../results/matrix_summary.csv")
    args = parser.parse_args()

    raw_rows = []
    summary_rows = []

    for scenario in args.scenarios:
        for algo in args.algorithms:
            print(f"Rodando {algo} em {scenario} ({args.runs} execuções)")
            rows = run_one(algo, scenario, args)
            raw_rows.extend(rows)
            summary_rows.append(summarize(rows))

    raw_fields = [
        "algorithm",
        "scenario",
        "run",
        "seed",
        "success",
        "planning_time",
        "num_nodes",
        "path_length",
        "path_smoothness",
        "clearance",
        "path_efficiency",
    ]
    summary_fields = list(summary_rows[0].keys()) if summary_rows else []

    import os
    os.makedirs(os.path.dirname(args.raw_csv), exist_ok=True)
    os.makedirs(os.path.dirname(args.summary_csv), exist_ok=True)

    with open(args.raw_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=raw_fields)
        writer.writeheader()
        writer.writerows(raw_rows)

    with open(args.summary_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(summary_rows)

    print("Arquivos gerados:")
    print(f"  {args.raw_csv}")
    print(f"  {args.summary_csv}")


if __name__ == "__main__":
    main()
