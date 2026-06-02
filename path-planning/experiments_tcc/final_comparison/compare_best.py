"""Comparação final dos algoritmos usando seus melhores parâmetros.

Lê os JSONs gerados pelo sweep (sweep_<algo>_<scene>_best.json) e executa
N rodadas de cada algoritmo com seus melhores hiperparâmetros para gerar
estatística robusta para os boxplots do TCC.

Uso:
  python compare_best.py --scene cena4 --runs 20
  python compare_best.py --scene cena4 --runs 30 --algos rrt rrt_connect est

Output:
  experiments_tcc/results/comparison_<scene>.csv
"""

import argparse
import csv
import json
import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TCC_ROOT   = os.path.dirname(os.path.dirname(SCRIPT_DIR))
COPPELIA   = os.path.join(TCC_ROOT, "coppeliasim")
SWEEP_DIR  = os.path.join(TCC_ROOT, "experiments_tcc", "parameter_sweep")
for _d in [TCC_ROOT, COPPELIA, SWEEP_DIR]:
    if _d not in sys.path:
        sys.path.insert(0, _d)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sweep import (
    build_args_for_combo,
    run_single,
    aggregate,
)
from run_coppelia_batch import reset_and_verify, _sim_is_stopped, _wait_sim_state


def load_best(algo: str, scene: str) -> dict:
    results_dir = os.path.join(TCC_ROOT, "experiments_tcc", "results")
    path = os.path.join(results_dir, f"sweep_{algo}_{scene}_best.json")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Best params não encontrados: {path}\n"
            f"Rode antes: python experiments_tcc/parameter_sweep/sweep.py "
            f"--algo {algo} --scene {scene}"
        )
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def main():
    parser = argparse.ArgumentParser(description="Comparação final dos melhores configs.")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--runs", type=int, default=20,
                        help="Rodadas por algoritmo (recomendado >= 20 para boxplot)")
    parser.add_argument("--algos", nargs="+",
                        default=["rrt", "rrt_connect", "est"],
                        help="Algoritmos a comparar")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--settle-time", type=float, default=2.0)
    args = parser.parse_args()

    # Carrega melhores params de cada algoritmo
    best_configs = {}
    for algo in args.algos:
        cfg = load_best(algo, args.scene)
        best_configs[algo] = cfg
        print(f"  [{algo}]  melhor combo: {cfg['combo']}")
    print()

    # Conexão
    print(f"Conectando ao CoppeliaSim {args.host}:{args.port}...")
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")
    if _sim_is_stopped(sim):
        sim.startSimulation()
        _wait_sim_state(sim, target_stopped=False, timeout=10.0)
        time.sleep(args.settle_time)

    # CSV
    results_dir = os.path.join(TCC_ROOT, "experiments_tcc", "results")
    csv_path = os.path.join(results_dir, f"comparison_{args.scene}.csv")
    fieldnames = [
        "scene", "algo", "run_idx",
        "success", "reason",
        "planning_time_s", "execution_time_s", "final_error_m",
        "iterations", "processed_waypoints", "skipped_waypoints",
        "min_clearance_m",
    ]
    f_csv = open(csv_path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(f_csv, fieldnames=fieldnames)
    writer.writeheader()

    # Loop principal
    all_results = {}
    for algo in args.algos:
        cfg = best_configs[algo]
        plan_args = build_args_for_combo(algo, cfg["combo"])

        robot = sim.getObject(plan_args.robot_path)
        lm    = sim.getObject(plan_args.left_motor_path)
        rm    = sim.getObject(plan_args.right_motor_path)
        goal  = sim.getObject(plan_args.goal_object)

        print(f"\n{'='*60}\nAlgoritmo: {algo}  ({args.runs} rodadas)\n{'='*60}")

        runs = []
        for ri in range(1, args.runs + 1):
            print(f"  run {ri}/{args.runs}...", end=" ", flush=True)
            if ri > 1:
                ok = reset_and_verify(sim, robot, goal, settle_time=args.settle_time)
                if not ok:
                    print("[reset falhou]")
                    continue
                robot = sim.getObject(plan_args.robot_path)
                lm    = sim.getObject(plan_args.left_motor_path)
                rm    = sim.getObject(plan_args.right_motor_path)
                goal  = sim.getObject(plan_args.goal_object)

            try:
                result = run_single(sim, robot, lm, rm, plan_args)
            except Exception as e:
                print(f"[exception: {e}]")
                result = {
                    "success": False, "reason": "exception",
                    "planning_time_s": 0.0, "execution_time_s": 0.0,
                    "final_error_m": float("nan"),
                    "iterations": 0, "processed_waypoints": 0,
                    "skipped_waypoints": 0, "min_clearance_m": float("nan"),
                }
            mark = "✓" if result["success"] else "✗"
            print(f"{mark} plan={result['planning_time_s']:.3f}s")
            runs.append(result)

            row = {"scene": args.scene, "algo": algo, "run_idx": ri}
            row.update(result)
            writer.writerow(row)
            f_csv.flush()

        all_results[algo] = runs

    f_csv.close()

    # Resumo
    print(f"\n{'='*70}")
    print(f"COMPARAÇÃO FINAL  (cena {args.scene})")
    print(f"{'='*70}")
    print(f"{'Algo':<15} {'Sucesso':<12} {'Plan (s)':<18} {'Exec (s)':<18}")
    print("-" * 70)
    for algo, runs in all_results.items():
        stats = aggregate(runs)
        succ = f"{stats['n_success']}/{stats['n_runs']} ({stats['success_rate']*100:.0f}%)"
        plan = f"{stats['planning_time_s_mean']:.3f}±{stats['planning_time_s_std']:.3f}"
        exec_ = f"{stats['execution_time_s_mean']:.3f}±{stats['execution_time_s_std']:.3f}"
        print(f"{algo:<15} {succ:<12} {plan:<18} {exec_:<18}")
    print(f"\n  CSV salvo: {csv_path}")
    print(f"\n  Gere os boxplots com:")
    print(f"     python experiments_tcc/plots/boxplots.py --scene {args.scene}")


if __name__ == "__main__":
    raise SystemExit(main())
