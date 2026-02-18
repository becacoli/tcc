import argparse
import random

from algorithms.rrt import RRT
from algorithms.rrt_connect import RRTConnect
from algorithms.rrt_merged import RRTMerge
from algorithms.rrt_star import RRTStar
from utils.plot_live import plot_live
from utils.plotting import plot_path

ALGORITHMS = {
    "rrt": RRT,
    "rrt_star": RRTStar,
    "rrt_connect": RRTConnect,
    "rrt_merged": RRTMerge,
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
            (0, 0, 100, 35),      # Parede inferior
            (0, 65, 100, 100),    # Parede superior  
            (30, 35, 35, 55),     # Obstáculo 1 (passagem embaixo)
            (50, 45, 55, 65),     # Obstáculo 2 (passagem em cima)
            (70, 35, 75, 55),     # Obstáculo 3 (passagem embaixo)
        ],
    },
    "vazio": {
        "start": (10, 10),
        "goal": (90, 90),
        "map_size": (100, 100),
        "obstacles": [],
    },
}


def _build_kwargs(name, args):
    base = dict(step_size=args.step_size, goal_sample_rate=args.goal_sample_rate,
                obstacles=None)
    if name == "rrt_connect":
        base["max_samples"] = args.max_iter
    else:
        base["max_iter"] = args.max_iter
    if name == "rrt_star":
        base["neighbor_radius"] = args.neighbor_radius
    if name == "rrt_merged":
        base["connect_threshold"] = args.connect_threshold
    return base


def build_planner(name, start, goal, map_size, obstacles, args):
    kwargs = _build_kwargs(name, args)
    kwargs["obstacles"] = obstacles
    return ALGORITHMS[name](start, goal, map_size, **kwargs)


def main():
    parser = argparse.ArgumentParser(description="Executa multiplas rodadas de RRT.")
    parser.add_argument("--algo", choices=list(ALGORITHMS), default="rrt")
    parser.add_argument("--scenario", choices=list(SCENARIOS), default="default",
                        help="Cenário a usar (default, simples, moderado, complexo, corredor, vazio)")
    parser.add_argument("--list-scenarios", action="store_true",
                        help="Lista os cenários disponíveis e sai")
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iter", type=int, default=500)
    parser.add_argument("--step-size", type=float, default=5)
    parser.add_argument("--goal-sample-rate", type=float, default=0.05)
    parser.add_argument("--neighbor-radius", type=float, default=15)
    parser.add_argument("--connect-threshold", type=float, default=10)
    parser.add_argument("--visual", choices=["none", "static", "live", "both"], default="none")
    args = parser.parse_args()

    if args.list_scenarios:
        print("Cenários disponíveis:")
        for name, config in SCENARIOS.items():
            print(f"\n  {name}:")
            print(f"    Start: {config['start']}")
            print(f"    Goal: {config['goal']}")
            print(f"    Map: {config['map_size']}")
            print(f"    Obstáculos: {len(config['obstacles'])}")
        return

    scenario = SCENARIOS[args.scenario]
    start = scenario["start"]
    goal = scenario["goal"]
    map_size = scenario["map_size"]
    obstacles = scenario["obstacles"]
    
    print(f"Cenário: {args.scenario}")
    print(f"  Start: {start}, Goal: {goal}, Map: {map_size}")
    print(f"  Obstáculos: {len(obstacles)}")
    print()

    successes = 0
    for i in range(args.runs):
        seed = args.seed + i
        show_live = args.visual in ("live", "both")
        show_static = args.visual in ("static", "both")

        if show_live:
            random.seed(seed)
            planner = build_planner(args.algo, start, goal, map_size, obstacles, args)
            plot_live(planner, start, goal, obstacles, 
                     title=f"{args.algo} - {args.scenario} #{i+1}")

        random.seed(seed)
        planner = build_planner(args.algo, start, goal, map_size, obstacles, args)
        path = planner.planning()

        if show_static:
            plot_path(path, planner.get_all_nodes(), start, goal, obstacles,
                      map_size=map_size, title=f"{args.algo} - {args.scenario} #{i+1}")

        found = path is not None
        if found:
            successes += 1
        print(f"Execucao {i+1}/{args.runs}: {'ok' if found else 'falhou'}")

    print(f"Sucesso: {successes}/{args.runs}")


if __name__ == "__main__":
    main()
