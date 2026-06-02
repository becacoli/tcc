"""Parameter sweep para encontrar a "melhor forma" de cada algoritmo.

Metodologia (orientador):
  Fase 1 — Para cada algoritmo, varia 1-2 hiperparâmetros principais e
           mede as 5 métricas. Identifica a configuração ótima.
  Fase 2 — Roda comparação final usando os melhores parâmetros de cada um.

Este script implementa a Fase 1.

Uso:
  python sweep.py --algo rrt           --scene cena4 --runs 5
  python sweep.py --algo rrt_connect   --scene cena4 --runs 5
  python sweep.py --algo est           --scene cena4 --runs 5
  python sweep.py --algo est --scene cena4 --runs 3 --quick   # sweep mais curto

Output:
  experiments_tcc/results/sweep_<algo>_<scene>.csv  (todas as combinações)
  experiments_tcc/results/sweep_<algo>_<scene>_best.json  (melhor config)

Pré-requisitos:
  - CoppeliaSim aberto com a cena correspondente
  - Simulação rodando (▶)
"""

import argparse
import csv
import itertools
import json
import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TCC_ROOT   = os.path.dirname(os.path.dirname(SCRIPT_DIR))
COPPELIA   = os.path.join(TCC_ROOT, "coppeliasim")
for _d in [TCC_ROOT, COPPELIA, SCRIPT_DIR]:
    if _d not in sys.path:
        sys.path.insert(0, _d)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from rrt_navigation import build_parser, build_control_obstacles_world
from planner_isolated import (
    extract_planning_context,
    plan_path_independent,
    planner_path_to_world,
)
from wheel_controller import follow_world_path
from run_coppelia_batch import reset_and_verify, _sim_is_stopped, _wait_sim_state


# ---------------------------------------------------------------------------
# Grids de parâmetros por algoritmo
# ---------------------------------------------------------------------------
# Cada grid define quais parâmetros variar. Mantemos os parâmetros de
# controle (wheel_controller) constantes para isolar o efeito do planejador.

COMMON_FIXED = {
    "robot_path":       "/PioneerP3DX",
    "left_motor_path":  "/PioneerP3DX/leftMotor",
    "right_motor_path": "/PioneerP3DX/rightMotor",
    "goal_object":      "/GoalConfiguration",
    "obstacle_model":   "polygon",
    "polygon_source":   "vertices",
    "lookahead_waypoints": 1,
    "robot_radius":     0.18,
    "wall_inflate":     0.00,
    # Controle igual para todos:
    "waypoint_spacing": 0.06,
    "linear_speed":     0.12,
}

# Grid completo (usa quando NÃO passa --quick)
GRIDS = {
    "rrt": {
        "step_size":        [0.8, 1.0, 1.2, 1.5, 2.0],
        "goal_sample_rate": [0.05, 0.10, 0.15, 0.20, 0.30],
        "max_iter":         [12000],
        "plan_attempts":    [25],
    },
    "rrt_connect": {
        "step_size":        [0.8, 1.0, 1.2, 1.5, 2.0],
        "goal_sample_rate": [0.05, 0.10, 0.15, 0.20],
        "max_iter":         [8000],
        "plan_attempts":    [20],
    },
    "est": {
        "local_sample_radius":  [5, 10, 15, 20],
        "density_candidates":   [20, 30, 40],
        "density_radius":       [4],
        "global_sample_rate":   [0.0],
        "goal_sample_rate":     [0.15],
        "step_size":            [1.2],
        "max_iter":             [5000],
        "plan_attempts":        [3],
    },
}

# Grid reduzido para teste rápido (--quick)
GRIDS_QUICK = {
    "rrt": {
        "step_size":        [1.0, 1.5],
        "goal_sample_rate": [0.10, 0.20],
        "max_iter":         [12000],
        "plan_attempts":    [25],
    },
    "rrt_connect": {
        "step_size":        [1.0, 1.5],
        "goal_sample_rate": [0.10, 0.15],
        "max_iter":         [8000],
        "plan_attempts":    [20],
    },
    "est": {
        "local_sample_radius":  [10, 15, 20],
        "density_candidates":   [30],
        "density_radius":       [4],
        "global_sample_rate":   [0.0],
        "goal_sample_rate":     [0.15],
        "step_size":            [1.2],
        "max_iter":             [5000],
        "plan_attempts":        [3],
    },
}


def build_args_for_combo(algo: str, combo: dict) -> "argparse.Namespace":
    """Converte uma combinação de hiperparâmetros num Namespace pro planner."""
    parser = build_parser()
    cli = ["--algo", algo if algo != "rrt_connect" else "rrt_connect"]

    # Constantes
    for k, v in COMMON_FIXED.items():
        cli.append(f"--{k.replace('_', '-')}")
        cli.append(str(v))

    # Hiperparâmetros desta combinação
    for k, v in combo.items():
        cli.append(f"--{k.replace('_', '-')}")
        cli.append(str(v))

    args = parser.parse_args(cli)
    args.plot_planner = False
    args.plot_block   = False
    args.online_replan = False
    args.debug = False
    return args


def run_single(sim, robot, lm, rm, args) -> dict:
    """Executa planejamento + controle e retorna métricas."""
    context = extract_planning_context(sim, args)
    path_planner, plan_metrics = plan_path_independent(context, args)
    planning_time = plan_metrics["planning_time_s"]

    if not path_planner:
        return {
            "success": False,
            "reason": "no_path",
            "planning_time_s": planning_time,
            "execution_time_s": 0.0,
            "final_error_m": float("nan"),
            "iterations": plan_metrics.get("iterations", 0),
            "processed_waypoints": 0,
            "skipped_waypoints": 0,
            "min_clearance_m": float("nan"),
        }

    waypoints_world = planner_path_to_world(path_planner, context)
    control_obstacles = build_control_obstacles_world(context, args)
    ctrl = follow_world_path(
        sim, robot, lm, rm,
        waypoints_world, args,
        obstacles_world=control_obstacles,
    )
    return {
        "success": ctrl.get("success", False),
        "reason": ctrl.get("reason", "unknown"),
        "planning_time_s": planning_time,
        "execution_time_s": ctrl.get("execution_time_s", 0.0),
        "final_error_m": ctrl.get("final_error_m", float("nan")),
        "iterations": plan_metrics.get("iterations", 0),
        "processed_waypoints": plan_metrics.get("processed_waypoints", 0),
        "skipped_waypoints": ctrl.get("skipped_waypoints", 0),
        "min_clearance_m": plan_metrics.get("path_min_clearance_m", float("nan")),
    }


def aggregate(runs: list) -> dict:
    """Agrega N runs em média ± std das métricas."""
    def _stats(values):
        vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
        if not vals:
            return float("nan"), float("nan")
        mean = sum(vals) / len(vals)
        var  = sum((v - mean) ** 2 for v in vals) / len(vals)
        return mean, math.sqrt(var)

    n = len(runs)
    successes = sum(1 for r in runs if r["success"])
    out = {
        "n_runs":       n,
        "n_success":    successes,
        "success_rate": successes / n if n else 0.0,
    }
    for k in ("planning_time_s", "execution_time_s", "final_error_m",
              "processed_waypoints", "min_clearance_m"):
        mean, std = _stats([r[k] for r in runs])
        out[f"{k}_mean"] = mean
        out[f"{k}_std"]  = std
    return out


def score_combo(stats: dict, weight_success: float = 1000.0) -> float:
    """Score combinado para escolher a melhor configuração.

    Quanto MAIOR melhor.
    Fórmula: success_rate * weight − (planning_time + 0.5*final_error)
    Falhar custa muito (weight=1000), entre as que funcionam preferimos
    a mais rápida e mais precisa.
    """
    if stats["success_rate"] < 1e-6:
        return -1e9
    base = stats["success_rate"] * weight_success
    penalty = stats.get("planning_time_s_mean", 0.0) or 0.0
    err     = stats.get("final_error_m_mean", 0.0) or 0.0
    if math.isnan(err):
        err = 1.0  # penaliza se não computou
    return base - penalty - 0.5 * err


def main():
    parser = argparse.ArgumentParser(description="Parameter sweep para TCC.")
    parser.add_argument("--algo", required=True, choices=["rrt", "rrt_connect", "est"])
    parser.add_argument("--scene", required=True, help="Nome da cena (ex: cena4)")
    parser.add_argument("--runs", type=int, default=3, help="Rodadas por combinação")
    parser.add_argument("--quick", action="store_true", help="Usar grid reduzido")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--no-reset", action="store_true",
                        help="Não reinicia simulação entre runs (não recomendado)")
    parser.add_argument("--settle-time", type=float, default=2.0)
    args = parser.parse_args()

    grids = GRIDS_QUICK if args.quick else GRIDS
    grid  = grids[args.algo]
    keys  = list(grid.keys())
    combos = [dict(zip(keys, vals)) for vals in itertools.product(*[grid[k] for k in keys])]

    print(f"Sweep: algoritmo={args.algo}  cena={args.scene}  combinações={len(combos)}  runs/comb={args.runs}")
    total = len(combos) * args.runs
    print(f"Total de rodadas: {total}  (estimativa: {total * 15 / 60:.0f}-{total * 25 / 60:.0f} min)")
    print()

    # Output paths
    results_dir = os.path.join(TCC_ROOT, "experiments_tcc", "results")
    os.makedirs(results_dir, exist_ok=True)
    csv_path  = os.path.join(results_dir, f"sweep_{args.algo}_{args.scene}.csv")
    json_path = os.path.join(results_dir, f"sweep_{args.algo}_{args.scene}_best.json")

    # Conexão CoppeliaSim
    print(f"Conectando ao CoppeliaSim {args.host}:{args.port}...")
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")

    # Garante simulação rodando
    if _sim_is_stopped(sim):
        print("Iniciando simulação...")
        sim.startSimulation()
        _wait_sim_state(sim, target_stopped=False, timeout=10.0)
        time.sleep(args.settle_time)

    # CSV header
    fieldnames = [
        "scene", "algo", "combo_idx", "run_idx",
        *keys,
        "success", "reason",
        "planning_time_s", "execution_time_s", "final_error_m",
        "iterations", "processed_waypoints", "skipped_waypoints",
        "min_clearance_m",
    ]
    f_csv = open(csv_path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(f_csv, fieldnames=fieldnames)
    writer.writeheader()

    # Loop principal
    best_combo = None
    best_score = -1e9
    best_stats = None
    aggregated_results = []

    for ci, combo in enumerate(combos, 1):
        print(f"[{ci}/{len(combos)}] {combo}")
        runs_results = []
        plan_args = build_args_for_combo(args.algo, combo)
        robot = sim.getObject(plan_args.robot_path)
        lm    = sim.getObject(plan_args.left_motor_path)
        rm    = sim.getObject(plan_args.right_motor_path)
        goal  = sim.getObject(plan_args.goal_object)

        for run_i in range(1, args.runs + 1):
            print(f"   run {run_i}/{args.runs}...", end=" ", flush=True)
            # Reset entre runs (após a 1ª)
            if not args.no_reset and run_i > 1:
                ok = reset_and_verify(sim, robot, goal, settle_time=args.settle_time)
                if not ok:
                    print("[ERRO reset, pulando]")
                    continue
                robot = sim.getObject(plan_args.robot_path)
                lm    = sim.getObject(plan_args.left_motor_path)
                rm    = sim.getObject(plan_args.right_motor_path)
                goal  = sim.getObject(plan_args.goal_object)

            t0 = time.time()
            try:
                result = run_single(sim, robot, lm, rm, plan_args)
            except Exception as e:
                print(f"[ERRO: {e}]")
                result = {
                    "success": False, "reason": "exception",
                    "planning_time_s": 0.0, "execution_time_s": 0.0,
                    "final_error_m": float("nan"),
                    "iterations": 0, "processed_waypoints": 0,
                    "skipped_waypoints": 0, "min_clearance_m": float("nan"),
                }
            elapsed = time.time() - t0
            mark = "✓" if result["success"] else "✗"
            print(f"{mark} plan={result['planning_time_s']:.2f}s  total={elapsed:.1f}s")
            runs_results.append(result)

            row = {"scene": args.scene, "algo": args.algo,
                   "combo_idx": ci, "run_idx": run_i}
            row.update(combo)
            row.update(result)
            writer.writerow(row)
            f_csv.flush()

        # Agrega e avalia score
        stats = aggregate(runs_results)
        score = score_combo(stats)
        aggregated_results.append({
            "combo": combo, "stats": stats, "score": score,
        })
        print(f"   → success={stats['n_success']}/{stats['n_runs']}  "
              f"plan_mean={stats.get('planning_time_s_mean', float('nan')):.3f}s  "
              f"score={score:.3f}")

        if score > best_score:
            best_score = score
            best_combo = combo
            best_stats = stats

    f_csv.close()

    # Salva melhor configuração
    best_output = {
        "algo":   args.algo,
        "scene":  args.scene,
        "combo":  best_combo,
        "stats":  best_stats,
        "score":  best_score,
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(best_output, f, indent=2)

    # Resumo
    print()
    print("=" * 70)
    print(f"MELHOR CONFIGURAÇÃO  ({args.algo} / {args.scene})")
    print("=" * 70)
    print(f"  Combo: {best_combo}")
    print(f"  Score: {best_score:.3f}")
    print(f"  Sucesso: {best_stats['n_success']}/{best_stats['n_runs']} "
          f"({best_stats['success_rate']*100:.0f}%)")
    print(f"  Plan time: {best_stats['planning_time_s_mean']:.3f} "
          f"± {best_stats['planning_time_s_std']:.3f} s")
    print(f"  Erro final: {best_stats['final_error_m_mean']:.3f} m")
    print(f"  Clearance: {best_stats['min_clearance_m_mean']:.3f} m")
    print()
    print(f"  CSV completo:  {csv_path}")
    print(f"  JSON melhor:   {json_path}")


if __name__ == "__main__":
    raise SystemExit(main())
