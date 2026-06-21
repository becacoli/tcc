"""Run named CoppeliaSim planner command presets.

Examples:
  python run_planner_command.py list
  python run_planner_command.py rrt_connect_fast
  python run_planner_command.py est --dry-run
"""

import argparse
import subprocess
import sys


COMMON_SCENE_ARGS = [
    "--robot-path",
    "/PioneerP3DX",
    "--left-motor-path",
    "/PioneerP3DX/leftMotor",
    "--right-motor-path",
    "/PioneerP3DX/rightMotor",
    "--goal-object",
    "/GoalConfiguration",
    "--obstacle-model",
    "polygon",
    "--polygon-source",
    "vertices",
    "--lookahead-waypoints",
    "1",
    "--debug",
    "--plot-planner",
]


PRESETS = {
    "rrt": [
        "--algo",
        "rrt",
        *COMMON_SCENE_ARGS,
        "--robot-radius",
        "0.18",
        "--wall-inflate",
        "0.00",
        "--step-size",
        "1.2",
        "--goal-sample-rate",
        "0.15",
        "--max-iter",
        "12000",
        "--plan-attempts",
        "25",
        "--waypoint-spacing",
        "0.06",
        "--linear-speed",
        "0.12",
    ],
    "rrt_star": [
        "--algo",
        "rrt_star",
        *COMMON_SCENE_ARGS,
        "--robot-radius",
        "0.18",
        "--wall-inflate",
        "0.00",
        "--step-size",
        "1.2",
        "--goal-sample-rate",
        "0.15",
        "--neighbor-radius",
        "5.0",
        "--max-iter",
        "12000",
        "--plan-attempts",
        "25",
        "--waypoint-spacing",
        "0.06",
        "--linear-speed",
        "0.12",
    ],
    "informed_rrt_star": [
        "--algo",
        "informed_rrt_star",
        *COMMON_SCENE_ARGS,
        "--robot-radius",
        "0.18",
        "--wall-inflate",
        "0.00",
        "--step-size",
        "1.2",
        "--goal-sample-rate",
        "0.15",
        "--neighbor-radius",
        "5.0",
        "--max-iter",
        "12000",
        "--plan-attempts",
        "25",
        "--waypoint-spacing",
        "0.06",
        "--linear-speed",
        "0.12",
    ],
    "rrt_connect_fast": [
        "--algo",
        "rrt_connect",
        *COMMON_SCENE_ARGS,
        "--robot-radius",
        "0.18",
        "--wall-inflate",
        "0.00",
        "--step-size",
        "1.5",
        "--goal-sample-rate",
        "0.15",
        "--max-iter",
        "8000",
        "--plan-attempts",
        "20",
        "--waypoint-spacing",
        "0.06",
        "--linear-speed",
        "0.12",
    ],
    "rrt_connect_precise": [
        "--algo",
        "rrt_connect",
        *COMMON_SCENE_ARGS,
        "--robot-radius",
        "0.18",
        "--wall-inflate",
        "0.01",
        "--step-size",
        "1.2",
        "--goal-sample-rate",
        "0.15",
        "--max-iter",
        "12000",
        "--plan-attempts",
        "25",
        "--waypoint-spacing",
        "0.06",
        "--linear-speed",
        "0.12",
    ],
    "est": [
        # EST puro: global_sample_rate=0.0, depende só da densidade.
        # Aumentamos local_sample_radius para 15px (0.75m) — dá mais
        # chance de progredir em espaços abertos. Em mazes ainda falha.
        "--algo",
        "est",
        *COMMON_SCENE_ARGS,
        "--robot-radius",
        "0.18",
        "--wall-inflate",
        "0.00",
        "--step-size",
        "1.2",
        "--density-radius",
        "4",
        "--local-sample-radius",
        "15",                   # antes: 3 (era pequeno demais)
        "--global-sample-rate",
        "0.0",
        "--density-candidates",
        "30",
        "--goal-sample-rate",
        "0.15",
        "--max-iter",
        "5000",
        "--plan-attempts",
        "3",
        "--waypoint-spacing",
        "0.06",
        "--linear-speed",
        "0.12",
    ],
}


def format_command(command):
    return " ".join(f'"{part}"' if " " in part else part for part in command)


def main():
    parser = argparse.ArgumentParser(description="Executa presets de planejamento no CoppeliaSim.")
    parser.add_argument("preset", choices=["list", *PRESETS.keys()])
    parser.add_argument("--dry-run", action="store_true", help="Mostra o comando sem executar.")
    args, extra_args = parser.parse_known_args()

    if args.preset == "list":
        print("Presets disponiveis:")
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
