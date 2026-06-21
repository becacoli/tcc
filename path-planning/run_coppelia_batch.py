"""Executa múltiplas rodadas de cada preset no CoppeliaSim e coleta estatísticas.

Pré-requisito: CoppeliaSim com a cena aberta e simulação em execução.

Exemplos:
  python run_coppelia_batch.py --list
  python run_coppelia_batch.py --presets rrt rrt_connect --runs 5
  python run_coppelia_batch.py --presets all --runs 3 --output results_coppelia.csv
  python run_coppelia_batch.py --presets rrt rrt_connect est --runs 5 --no-reset
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

from run_planner_command import PRESETS as PRESETS_DEFAULT
from run_planner_command_fair import PRESETS as PRESETS_FAIR
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
    state = sim.getSimulationState()
    stopped_val = getattr(sim, "simulation_stopped", 0)
    return state == stopped_val


def _wait_sim_state(sim, target_stopped: bool, timeout: float = 15.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _sim_is_stopped(sim) == target_stopped:
            return True
        time.sleep(0.1)
    return False


def _dist_robot_to_goal(sim, robot, goal_obj) -> float:
    """Distância euclidiana entre o robô e o goal no espaço do mundo."""
    rpos = sim.getObjectPosition(robot, -1)
    gpos = sim.getObjectPosition(goal_obj, -1)
    return math.hypot(rpos[0] - gpos[0], rpos[1] - gpos[1])


def reset_and_verify(
    sim,
    robot,
    goal_obj,
    settle_time: float = 2.0,
    verify_timeout: float = 12.0,
    near_goal_threshold: float = 0.25,
) -> bool:
    """
    Para e reinicia a simulação.
    Aguarda até o robô estar longe do goal (sinal de que voltou ao início).
    Retorna True em caso de sucesso (ou se o robô nunca esteve perto do goal).
    """
    sim.stopSimulation()
    if not _wait_sim_state(sim, target_stopped=True, timeout=15.0):
        print("  [aviso] timeout aguardando simulação parar")
        return False
    time.sleep(0.3)
    sim.startSimulation()
    if not _wait_sim_state(sim, target_stopped=False, timeout=15.0):
        print("  [aviso] timeout aguardando simulação iniciar")
        return False

    # Espera mínima para física estabilizar
    time.sleep(settle_time)

    # Verifica se o robô saiu da área do goal (ou nunca estava lá)
    deadline = time.time() + verify_timeout
    while time.time() < deadline:
        dist = _dist_robot_to_goal(sim, robot, goal_obj)
        if dist > near_goal_threshold:
            return True  # Resetou corretamente — robô está longe do goal
        time.sleep(0.25)

    dist = _dist_robot_to_goal(sim, robot, goal_obj)
    if dist <= near_goal_threshold:
        print(
            f"  [aviso] Robô ainda a {dist:.3f}m do goal após reset "
            f"({verify_timeout:.0f}s). "
            "Verifique se a posição inicial da cena foi salva corretamente."
        )
    return True  # Continua mesmo assim; run_single pode detectar o problema


# ---------------------------------------------------------------------------
# Conversão preset → Namespace
# ---------------------------------------------------------------------------

def preset_to_args(preset_name: str, presets: dict) -> argparse.Namespace:
    cli_args = presets[preset_name]
    parser = build_parser()
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
        "min_clearance_m": plan_metrics.get("path_min_clearance_m", float("nan")),
    }


# ---------------------------------------------------------------------------
# Estatísticas
# ---------------------------------------------------------------------------

def compute_stats(runs: list[dict]) -> dict:
    def _stats(values):
        vals = [v for v in values if not (isinstance(v, float) and math.isnan(v))]
        if not vals:
            return float("nan"), float("nan")
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return mean, math.sqrt(var)

    n = len(runs)
    successes = sum(1 for r in runs if r["success"])
    stats = {
        "n_runs": n,
        "n_success": successes,
        "success_rate": successes / n if n > 0 else 0.0,
    }
    for k in ("planning_time_s", "execution_time_s", "final_error_m",
               "iterations", "processed_waypoints", "skipped_waypoints",
               "min_clearance_m"):
        mean, std = _stats([r[k] for r in runs])
        stats[f"{k}_mean"] = mean
        stats[f"{k}_std"] = std

    ok_runs = [r for r in runs if r["success"]]
    for k in ("execution_time_s", "final_error_m", "skipped_waypoints",
              "min_clearance_m"):
        mean, std = _stats([r[k] for r in ok_runs])
        stats[f"{k}_succ_mean"] = mean
        stats[f"{k}_succ_std"] = std
    return stats


# ---------------------------------------------------------------------------
# Impressão de tabela
# ---------------------------------------------------------------------------

def print_summary(all_stats: dict[str, dict]):
    sep = "=" * 120
    print(f"\n{sep}")
    print("RESULTADOS — CoppeliaSim Batch")
    print(sep)
    print(
        f"{'Preset':<28} {'Sucesso':<10} {'t_plan(s)':<14} {'t_exec(s)':<14} "
        f"{'err_final(m)':<15} {'clearance(m)':<14} {'waypts':<8} {'skip':<6}"
    )
    print("-" * 120)

    for preset, s in all_stats.items():
        success_str = f"{s['n_success']}/{s['n_runs']} ({s['success_rate']*100:.0f}%)"
        tp_mean = s["planning_time_s_mean"]
        tp_std  = s["planning_time_s_std"]
        te_mean = s["execution_time_s_succ_mean"]
        te_std  = s["execution_time_s_succ_std"]
        fe_mean = s["final_error_m_succ_mean"]
        fe_std  = s["final_error_m_succ_std"]
        cl_mean = s["min_clearance_m_succ_mean"]
        cl_std  = s["min_clearance_m_succ_std"]
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
            f"{_fmt(cl_mean, cl_std):<14} "
            f"{wp:<8.0f} "
            f"{_fmt(sk, float('nan'))}"
        )
    print(sep)


# ---------------------------------------------------------------------------
# Salvamento CSV (incremental)
# ---------------------------------------------------------------------------

def save_csv(all_runs: dict[str, list[dict]], output_path: str, scene: str = ""):
    """Salva todas as rodadas brutas em CSV (sobrescreve — safe para saves incrementais)."""
    parent = os.path.dirname(os.path.abspath(output_path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    fieldnames = [
        "scene", "preset", "run", "success", "reason",
        "planning_time_s", "execution_time_s", "final_error_m",
        "iterations", "processed_waypoints", "skipped_waypoints",
        "min_clearance_m",
    ]
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for preset, runs in all_runs.items():
            for i, r in enumerate(runs, 1):
                row = {"scene": scene, "preset": preset, "run": i}
                row.update(r)
                writer.writerow(row)
    print(f"  ✓ CSV salvo: {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Batch de experimentos CoppeliaSim.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--fair", action="store_true",
        help="Usa presets equalizados (run_planner_command_fair.py) em vez dos padrão.",
    )
    parser.add_argument(
        "--presets", nargs="+", metavar="PRESET",
        help="Presets a executar (use 'all' para todos).",
    )
    parser.add_argument("--runs", type=int, default=5, help="Número de rodadas por preset (padrão: 5)")
    parser.add_argument("--output", default="results_coppelia.csv", help="Arquivo CSV de saída")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument(
        "--settle-time", type=float, default=2.0,
        help="Segundos de espera após reiniciar a simulação (padrão: 2.0)",
    )
    parser.add_argument(
        "--no-reset", action="store_true",
        help="Não reinicia a simulação entre rodadas (você reseta manualmente)",
    )
    parser.add_argument(
        "--scene", default="",
        help="Nome da cena (ex: cena1, cena2). Adicionado ao CSV e ao nome do arquivo.",
    )
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Sobrescreve o arquivo de saída sem perguntar.",
    )
    parser.add_argument("--list", action="store_true", help="Lista os presets disponíveis e sai")
    args = parser.parse_args()

    # Seleciona o dicionário de presets
    PRESETS = PRESETS_FAIR if args.fair else PRESETS_DEFAULT

    if args.list:
        label = "equalizados" if args.fair else "padrão"
        print(f"Presets disponíveis ({label}):")
        for name in PRESETS:
            print(f"  {name}")
        return 0

    if not args.presets:
        parser.error("Informe --presets PRESET [PRESET ...] ou use --list")

    selected = list(PRESETS.keys()) if "all" in args.presets else args.presets
    for p in selected:
        if p not in PRESETS:
            parser.error(f"Preset desconhecido: '{p}'. Use --list para ver os disponíveis.")

    # Se --scene foi passado, embutir no nome do arquivo de saída automaticamente
    output_path = args.output
    if args.scene:
        base, ext = os.path.splitext(output_path)
        output_path = f"{base}_{args.scene}{ext}"

    # Avisa se o arquivo já existe
    if os.path.exists(output_path) and not args.overwrite:
        print(f"[aviso] O arquivo '{output_path}' já existe e será sobrescrito.")
        resp = input("         Continuar? [s/N] ").strip().lower()
        if resp not in ("s", "sim", "y", "yes"):
            print("Abortado.")
            return 1

    print(f"Conectando ao CoppeliaSim em {args.host}:{args.port}...")
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")
    print("Conectado.\n")

    all_runs: dict[str, list[dict]] = {}
    all_stats: dict[str, dict] = {}
    very_first_run = True  # Controla se é a primeira rodada de toda a sessão

    for preset_name in selected:
        print(f"{'='*60}")
        print(f"Preset: {preset_name}  ({args.runs} rodadas)")
        print(f"{'='*60}")
        preset_args = preset_to_args(preset_name, PRESETS)

        def _get_handles():
            return (
                sim.getObject(preset_args.robot_path),
                sim.getObject(preset_args.left_motor_path),
                sim.getObject(preset_args.right_motor_path),
                sim.getObject(preset_args.goal_object),
            )

        robot, left_mot, right_mot, goal_obj = _get_handles()

        runs = []
        for run_idx in range(1, args.runs + 1):
            print(f"\n  Rodada {run_idx}/{args.runs}...", flush=True)

            if not args.no_reset:
                if very_first_run:
                    # Apenas garante que a simulação está rodando
                    if _sim_is_stopped(sim):
                        print("  [info] Iniciando simulação...")
                        sim.startSimulation()
                        _wait_sim_state(sim, target_stopped=False, timeout=10.0)
                        time.sleep(args.settle_time)
                    very_first_run = False
                else:
                    # Toda rodada após a primeira: reset + verificação de posição
                    ok = reset_and_verify(
                        sim, robot, goal_obj,
                        settle_time=args.settle_time,
                    )
                    if not ok:
                        print("  [erro] falha ao reiniciar simulação; abortando este preset")
                        break
                    robot, left_mot, right_mot, goal_obj = _get_handles()

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

        # ► Salva após CADA preset (protege contra Ctrl+C)
        save_csv(all_runs, output_path, scene=args.scene)

    print_summary(all_stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
