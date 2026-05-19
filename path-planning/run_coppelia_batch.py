"""Executa múltiplas rodadas de cada preset no CoppeliaSim e coleta estatísticas.

Pré-requisito: CoppeliaSim com a cena aberta e simulação em execução.

Exemplos:
  python run_coppelia_batch.py --list
  python run_coppelia_batch.py --presets rrt rrt_connect_fast --runs 5
  python run_coppelia_batch.py --presets all --runs 3 --output results_coppelia.csv
  python run_coppelia_batch.py --presets rrt est_hybrid_safe --runs 5 --no-reset
"""

import argparse
import csv
import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
COPPELIASIM_DIR = os.path.join(SCRIPT_DIR, "coppeliasim")
for _d in [SCRIPT_DIR, COPPELIASIM_DIR]:
    if _d not in sys.path:
        sys.path.insert(0, _d)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Importa PRESETS e parser de cada módulo isolado
from run_planner_command import PRESETS
from rrt_navigation import build_parser, build_control_obstacles_world
from planner_isolated import (
    extract_planning_context,
    plan_path_independent,
    planner_path_to_world,
)
from wheel_controller import follow_world_path


# ---------------------------------------------------------------------------
# Utilitários de simulação
# ---------------------------------------------------------------------------

def _sim_is_stopped(sim) -> bool:
    """Retorna True se a simulação está parada."""
    state = sim.getSimulationState()
    # sim.simulation_stopped == 0 na maioria das versões
    stopped_val = getattr(sim, "simulation_stopped", 0)
    return state == stopped_val


def _wait_sim_state(sim, target_stopped: bool, timeout: float = 15.0) -> bool:
    """Aguarda simulação parar (target_stopped=True) ou iniciar (False)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _sim_is_stopped(sim) == target_stopped:
            return True
        time.sleep(0.1)
    return False


def reset_simulation(sim, settle_time: float = 1.5) -> bool:
    """Para e reinicia a simulação; retorna True se bem-sucedido."""
    sim.stopSimulation()
    if not _wait_sim_state(sim, target_stopped=True, timeout=15.0):
        print("  [aviso] timeout aguardando simulação parar")
        return False
    time.sleep(0.2)
    sim.startSimulation()
    if not _wait_sim_state(sim, target_stopped=False, timeout=15.0):
        print("  [aviso] timeout aguardando simulação iniciar")
        return False
    time.sleep(settle_time)  # aguarda dinâmica estabilizar
    return True


# ---------------------------------------------------------------------------
# Conversão preset → Namespace
# ---------------------------------------------------------------------------

def preset_to_args(preset_name: str) -> argparse.Namespace:
    """Converte a lista de flags do preset em um Namespace do argparse."""
    cli_args = PRESETS[preset_name]
    parser = build_parser()
    # Desliga plot para não bloquear durante batch
    args = parser.parse_args(cli_args)
    args.plot_planner = False
    args.plot_block = False
    args.online_replan = False
    return args


# ---------------------------------------------------------------------------
# Execução de uma única rodada
# ---------------------------------------------------------------------------

def run_single(sim, robot, left_motor, right_motor, args) -> dict:
    """
    Executa planejamento + controle e retorna um dict de métricas.
    Campos: success, reason, planning_time_s, execution_time_s,
            final_error_m, iterations, processed_waypoints, skipped_waypoints.
    """
    # --- Planejamento ---
    t0 = time.time()
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
        }

    # --- Controle ---
    waypoints_world = planner_path_to_world(path_planner, context)
    control_obstacles = build_control_obstacles_world(context, args)
    ctrl = follow_world_path(
        sim, robot, left_motor, right_motor,
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
    }


# ---------------------------------------------------------------------------
# Estatísticas
# ---------------------------------------------------------------------------

def compute_stats(runs: list[dict]) -> dict:
    """Calcula média e desvio-padrão de cada métrica numérica."""
    def _stats(values):
        vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
        if not vals:
            return float("nan"), float("nan")
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return mean, math.sqrt(var)

    n = len(runs)
    successes = sum(1 for r in runs if r["success"])

    keys = ["planning_time_s", "execution_time_s", "final_error_m",
            "iterations", "processed_waypoints", "skipped_waypoints"]
    stats = {
        "n_runs": n,
        "n_success": successes,
        "success_rate": successes / n if n > 0 else 0.0,
    }
    for k in keys:
        values = [r[k] for r in runs]
        mean, std = _stats(values)
        stats[f"{k}_mean"] = mean
        stats[f"{k}_std"] = std

    # Média apenas das rodadas bem-sucedidas (mais representativo)
    success_runs = [r for r in runs if r["success"]]
    for k in ["execution_time_s", "final_error_m", "skipped_waypoints"]:
        values = [r[k] for r in success_runs]
        mean, std = _stats(values)
        stats[f"{k}_succ_mean"] = mean
        stats[f"{k}_succ_std"] = std

    return stats


# ---------------------------------------------------------------------------
# Impressão de tabela
# ---------------------------------------------------------------------------

def print_summary(all_stats: dict[str, dict]):
    sep = "=" * 110
    print(f"\n{sep}")
    print("RESULTADOS — CoppeliaSim Batch")
    print(sep)
    header = (
        f"{'Preset':<28} {'Sucesso':<10} {'t_plan(s)':<14} {'t_exec(s)':<14} "
        f"{'err_final(m)':<15} {'waypts':<8} {'skip':<6}"
    )
    print(header)
    print("-" * 110)

    for preset, s in all_stats.items():
        success_str = f"{s['n_success']}/{s['n_runs']} ({s['success_rate']*100:.0f}%)"
        tp_mean = s["planning_time_s_mean"]
        tp_std  = s["planning_time_s_std"]
        te_mean = s["execution_time_s_succ_mean"]
        te_std  = s["execution_time_s_succ_std"]
        fe_mean = s["final_error_m_succ_mean"]
        fe_std  = s["final_error_m_succ_std"]
        wp      = s["processed_waypoints_mean"]
        sk      = s["skipped_waypoints_succ_mean"]

        def _fmt(mean, std):
            if math.isnan(mean):
                return "  —  "
            if math.isnan(std) or std < 1e-9:
                return f"{mean:6.3f}"
            return f"{mean:6.3f}±{std:.3f}"

        print(
            f"{preset:<28} {success_str:<10} "
            f"{_fmt(tp_mean, tp_std):<14} "
            f"{_fmt(te_mean, te_std):<14} "
            f"{_fmt(fe_mean, fe_std):<15} "
            f"{wp:<8.0f} "
            f"{_fmt(sk, float('nan'))}"
        )
    print(sep)


# ---------------------------------------------------------------------------
# Salvamento CSV
# ---------------------------------------------------------------------------

def save_csv(all_runs: dict[str, list[dict]], output_path: str):
    """Salva todas as rodadas brutas em CSV."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fieldnames = [
        "preset", "run", "success", "reason",
        "planning_time_s", "execution_time_s", "final_error_m",
        "iterations", "processed_waypoints", "skipped_waypoints",
    ]
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for preset, runs in all_runs.items():
            for i, r in enumerate(runs, 1):
                row = {"preset": preset, "run": i}
                row.update(r)
                writer.writerow(row)
    print(f"\n✓ Dados brutos salvos em: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Batch de experimentos CoppeliaSim.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--presets", nargs="+",
        metavar="PRESET",
        help="Presets a executar (use 'all' para todos). "
             f"Disponíveis: {', '.join(PRESETS.keys())}",
    )
    parser.add_argument("--runs", type=int, default=5, help="Número de rodadas por preset (padrão: 5)")
    parser.add_argument("--output", default="results_coppelia.csv", help="Arquivo CSV de saída")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--settle-time", type=float, default=1.5,
                        help="Segundos de espera após reiniciar a simulação (padrão: 1.5)")
    parser.add_argument("--no-reset", action="store_true",
                        help="Não reinicia a simulação entre rodadas (você reseta manualmente)")
    parser.add_argument("--list", action="store_true", help="Lista os presets disponíveis e sai")
    args = parser.parse_args()

    if args.list:
        print("Presets disponíveis:")
        for name in PRESETS:
            print(f"  {name}")
        return 0

    if not args.presets:
        parser.error("Informe --presets PRESET [PRESET ...] ou use --list")

    selected = list(PRESETS.keys()) if "all" in args.presets else args.presets
    for p in selected:
        if p not in PRESETS:
            parser.error(f"Preset desconhecido: '{p}'. Use --list para ver os disponíveis.")

    # Conecta ao CoppeliaSim
    print(f"Conectando ao CoppeliaSim em {args.host}:{args.port}...")
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")
    print("Conectado.\n")

    all_runs: dict[str, list[dict]] = {}
    all_stats: dict[str, dict] = {}

    for preset_name in selected:
        print(f"{'='*60}")
        print(f"Preset: {preset_name}  ({args.runs} rodadas)")
        print(f"{'='*60}")
        preset_args = preset_to_args(preset_name)

        robot     = sim.getObject(preset_args.robot_path)
        left_mot  = sim.getObject(preset_args.left_motor_path)
        right_mot = sim.getObject(preset_args.right_motor_path)

        runs = []
        for run_idx in range(1, args.runs + 1):
            print(f"\n  Rodada {run_idx}/{args.runs}...", flush=True)

            # Reinicia simulação entre rodadas (exceto na primeira)
            if run_idx > 1 and not args.no_reset:
                ok = reset_simulation(sim, settle_time=args.settle_time)
                if not ok:
                    print("  [erro] falha ao reiniciar simulação; abortando este preset")
                    break
                # Atualiza handles após reinício (por segurança)
                robot     = sim.getObject(preset_args.robot_path)
                left_mot  = sim.getObject(preset_args.left_motor_path)
                right_mot = sim.getObject(preset_args.right_motor_path)
            elif run_idx == 1 and not args.no_reset:
                # Garante que a simulação está rodando antes da primeira rodada
                state = sim.getSimulationState()
                if _sim_is_stopped(sim):
                    print("  [info] Iniciando simulação...")
                    sim.startSimulation()
                    _wait_sim_state(sim, target_stopped=False, timeout=10.0)
                    time.sleep(args.settle_time)

            result = run_single(sim, robot, left_mot, right_mot, preset_args)
            runs.append(result)

            status = "✓" if result["success"] else "✗"
            print(
                f"  {status} plan={result['planning_time_s']:.2f}s  "
                f"exec={result['execution_time_s']:.2f}s  "
                f"err={result['final_error_m']:.3f}m  "
                f"motivo={result['reason']}"
            )

        all_runs[preset_name] = runs
        all_stats[preset_name] = compute_stats(runs)

    print_summary(all_stats)
    save_csv(all_runs, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
