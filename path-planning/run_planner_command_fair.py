"""Presets com parâmetros equalizados para comparação justa entre algoritmos.

Diferenças em relação ao run_planner_command.py:
  - Todos com robot_radius = 0.18
  - Todos com step_size = 1.2
  - Todos com goal_sample_rate = 0.15
  - Todos com max_iter = 12000
  - Todos com plan_attempts = 25
  - Todos com waypoint_spacing = 0.06
  - Todos com linear_speed = 0.12

Os únicos parâmetros que diferem entre os algoritmos são os exclusivos
do HybridEST (density_radius, local_sample_radius, density_candidates)
e o alpha (global_sample_rate), que define a variante.

Exemplos:
  python run_planner_command_fair.py list
  python run_planner_command_fair.py rrt_fair --dry-run
  python run_planner_command_fair.py est_hybrid_exploratory_fair
"""

import argparse
import subprocess
import sys


COMMON_SCENE_ARGS = [
    "--robot-path",        "/PioneerP3DX",
    "--left-motor-path",   "/PioneerP3DX/leftMotor",
    "--right-motor-path",  "/PioneerP3DX/rightMotor",
    "--goal-object",       "/GoalConfiguration",
    "--obstacle-model",    "polygon",
    "--polygon-source",    "vertices",
    "--lookahead-waypoints", "1",
    "--debug",
    "--plot-planner",
]

# Parâmetros compartilhados por todos os algoritmos
COMMON_ALGO_ARGS = [
    "--robot-radius",      "0.18",
    "--wall-inflate",      "0.00",
    "--step-size",         "1.2",
    "--goal-sample-rate",  "0.15",
    "--max-iter",          "12000",
    "--plan-attempts",     "25",
    "--waypoint-spacing",  "0.06",
    "--linear-speed",      "0.12",
]


PRESETS = {
    "rrt_fair": [
        "--algo", "rrt",
        *COMMON_SCENE_ARGS,
        *COMMON_ALGO_ARGS,
    ],
    "rrt_connect_fair": [
        "--algo", "rrt_connect",
        *COMMON_SCENE_ARGS,
        *COMMON_ALGO_ARGS,
    ],
    "est_hybrid_exploratory_fair": [
        "--algo", "est_hybrid",
        *COMMON_SCENE_ARGS,
        *COMMON_ALGO_ARGS,
        "--density-radius",      "4",
        "--local-sample-radius", "3",
        "--density-candidates",  "30",
        "--global-sample-rate",  "0.70",
    ],
    "est_hybrid_safe_fair": [
        "--algo", "est_hybrid",
        *COMMON_SCENE_ARGS,
        *COMMON_ALGO_ARGS,
        "--density-radius",      "4",
        "--local-sample-radius", "3",
        "--density-candidates",  "40",
        "--global-sample-rate",  "0.35",
    ],
}


def format_command(command):
    return " ".join(f'"{part}"' if " " in part else part for part in command)


def main():
    parser = argparse.ArgumentParser(
        description="Executa presets equalizados de planejamento no CoppeliaSim.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("preset", choices=["list", *PRESETS.keys()])
    parser.add_argument("--dry-run", action="store_true",
                        help="Mostra o comando sem executar.")
    args, extra_args = parser.parse_known_args()

    if args.preset == "list":
        print("Presets disponiveis (equalizados):")
        for name in PRESETS:
            print(f"  {name}")
        return 0

    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]

    command = [
        sys.executable,
        "coppeliasim/rrt_navigation.py",
        *PRESETS[args.preset],
        *extra_args,
    ]

    print(format_command(command))
    if args.dry_run:
        return 0

    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
