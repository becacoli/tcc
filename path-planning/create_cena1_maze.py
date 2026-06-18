"""Cena 1 — Espaço aberto com obstáculos dispersos (pilares).

Layout (5 m × 5 m, y↑  x→):

  ╔══════════════════════════════════════╗  boundary ±2.45 m
  ║  S(-1.8,+1.8)                        ║
  ║           [A]        [H]   [B]       ║
  ║                  [G]                 ║
  ║   [F]    [D]                         ║
  ║               [E]    [C]             ║
  ║                            G(+1.8,-1.8)║
  ╚══════════════════════════════════════╝

  8 pilares quadrados 0.4 × 0.4 m dispersos.
  Sem passagens estreitas — múltiplos caminhos viáveis.

  Mostra:  EST explora mais uniformemente, menos iterações desperdiçadas.
           RRT repete amostras em regiões já visitadas.

Uso:
  python create_cena1_maze.py
  python create_cena1_maze.py --remove
  python create_cena1_maze.py --dry-run
"""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
COPPELIASIM_DIR = os.path.join(SCRIPT_DIR, "coppeliasim")
for _d in [SCRIPT_DIR, COPPELIASIM_DIR]:
    if _d not in sys.path:
        sys.path.insert(0, _d)

# ---------------------------------------------------------------------------
WALL_HEIGHT    = 0.50
WALL_THICKNESS = 0.06
WALL_Z         = 0.25
PILLAR_SIZE    = 0.40   # pilares quadrados 0.4 × 0.4 m
FLOOR_NAME     = "floor"
GOAL_ALIAS     = "GoalConfiguration"
GOAL_POS       = [1.8, -1.8, 0.05]
ROBOT_PATH     = "/PioneerP3DX"
ROBOT_Z        = 0.14
ROBOT_START    = (-1.8, 1.8)

# Pilares: lista de (nome, cx, cy)
PILLAR_DEFS = [
    ("obs_A", -1.20,  0.80),
    ("obs_B",  0.80,  1.20),
    ("obs_C",  1.20, -0.50),
    ("obs_D", -0.50, -0.50),
    ("obs_E",  0.50, -1.00),
    ("obs_F", -1.50, -1.00),
    ("obs_G",  0.00,  0.50),
    ("obs_H", -0.30,  1.40),
]

BOUNDARY_SEGMENTS = [
    {"name": "boundary_N", "axis": "x", "cx":  0.00, "cy":  2.45, "length": 5.00},
    {"name": "boundary_S", "axis": "x", "cx":  0.00, "cy": -2.45, "length": 5.00},
    {"name": "boundary_W", "axis": "y", "cx": -2.45, "cy":  0.00, "length": 5.00},
    {"name": "boundary_E", "axis": "y", "cx":  2.45, "cy":  0.00, "length": 5.00},
]

PIONEER_MODEL_CANDIDATES = [
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\PioneerP3DX.ttm",
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSim\models\robots\mobile\PioneerP3DX.ttm",
    r"C:\Program Files (x86)\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\PioneerP3DX.ttm",
]

# ---------------------------------------------------------------------------
# Helpers compartilhados
# ---------------------------------------------------------------------------

def _make_static(sim, h):
    for param in [getattr(sim, "shapeintparam_static", None), 3]:
        if param is not None:
            try:
                sim.setObjectInt32Param(h, param, 1)
                return
            except Exception:
                pass
    try:
        sim.setShapeMass(h, 0.0)
    except Exception:
        pass


def _create_shape(sim, alias, size, pos):
    """Cria cuboid de tamanho arbitrário, removendo duplicata se existir."""
    try:
        sim.removeObject(sim.getObject(f"/{alias}"))
    except Exception:
        pass
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, -1, pos)
    _make_static(sim, h)
    return h


def _fix_floor_name(sim):
    try:
        sim.getObject(f"/{FLOOR_NAME}")
        print(f"  ✓ Chão '{FLOOR_NAME}' detectado  (bounds ±2.5 m)")
        return
    except Exception:
        pass
    for candidate in ["/ResizableFloor_5_25", "/Floor", "/Plane", "/ResizableFloor", "/ground"]:
        try:
            h = sim.getObject(candidate)
            sim.setObjectAlias(h, FLOOR_NAME)
            print(f"  ✓ Chão '{candidate.lstrip('/')}' → '{FLOOR_NAME}'")
            return
        except Exception:
            continue
    # Fallback: maior shape da cena
    try:
        handles = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
        best_h, best_area = None, 0.0
        for h in handles:
            try:
                sx = (sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_x)
                    - sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_min_x))
                sy = (sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_y)
                    - sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_min_y))
                if sx * sy > best_area:
                    best_area, best_h = sx * sy, h
            except Exception:
                pass
        if best_h and best_area > 4.0:
            old = sim.getObjectAlias(best_h)
            sim.setObjectAlias(best_h, FLOOR_NAME)
            print(f"  ✓ Chão '{old}' (área≈{best_area:.1f}m²) → '{FLOOR_NAME}'")
            return
    except Exception:
        pass
    print(f"  [aviso] Chão não encontrado — planner usará limites padrão (±5 m)")


def _position_robot(sim, xy, model_path=None):
    target = [xy[0], xy[1], ROBOT_Z]
    try:
        h = sim.getObject(ROBOT_PATH)
        sim.setObjectPosition(h, -1, target)
        print(f"  ✓ {ROBOT_PATH} → {target}")
        return
    except Exception:
        pass
    resolved = next((p for p in PIONEER_MODEL_CANDIDATES if os.path.exists(p)), model_path)
    if not resolved:
        print("  [aviso] PioneerP3DX.ttm não encontrado.")
        return
    h = sim.loadModel(resolved)
    sim.setObjectPosition(h, -1, target)
    print(f"  ✓ PioneerP3DX carregado em {target}")


def _ensure_sim_stopped(sim):
    try:
        return sim.getSimulationState() == getattr(sim, "simulation_stopped", 0)
    except Exception:
        return True


# ---------------------------------------------------------------------------
# Criação
# ---------------------------------------------------------------------------

def create_scene(sim, dry_run=False):
    print("Chão:\n")
    if not dry_run:
        _fix_floor_name(sim)
    else:
        print(f"  [dry-run] garantir nome '{FLOOR_NAME}'")
    print()

    print("Pilares (wall_obs_*):\n")
    for name, cx, cy in PILLAR_DEFS:
        alias = f"wall_{name}"
        size  = [PILLAR_SIZE, PILLAR_SIZE, WALL_HEIGHT]
        pos   = [cx, cy, WALL_Z]
        if dry_run:
            print(f"  [P] {alias:<22}  size=[{size[0]:.2f}×{size[1]:.2f}]  pos=[{cx:+.2f},{cy:+.2f}]")
        else:
            _create_shape(sim, alias, size, pos)
            print(f"  ✓ {alias}")

    print("\nParedes de borda:\n")
    for seg in BOUNDARY_SEGMENTS:
        alias = f"wall_{seg['name']}"
        if seg["axis"] == "x":
            size = [seg["length"], WALL_THICKNESS, WALL_HEIGHT]
        else:
            size = [WALL_THICKNESS, seg["length"], WALL_HEIGHT]
        pos = [seg["cx"], seg["cy"], WALL_Z]
        if dry_run:
            print(f"  [B] {alias:<22}  pos=[{pos[0]:+.2f},{pos[1]:+.2f}]")
        else:
            _create_shape(sim, alias, size, pos)
            print(f"  ✓ {alias}")

    if dry_run:
        print(f"\n  [goal]  /{GOAL_ALIAS}  {GOAL_POS}")
        print(f"  [robot] {ROBOT_PATH}  xy={ROBOT_START}")
        return

    # Goal
    try:
        gh = sim.getObject(f"/{GOAL_ALIAS}")
        sim.setObjectPosition(gh, -1, GOAL_POS)
        print(f"\n  ✓ /{GOAL_ALIAS}  → {GOAL_POS}")
    except Exception:
        gh = sim.createDummy(0.10)
        sim.setObjectAlias(gh, GOAL_ALIAS)
        sim.setObjectPosition(gh, -1, GOAL_POS)
        print(f"\n  ✓ /{GOAL_ALIAS} criado em {GOAL_POS}")

    print()
    _position_robot(sim, ROBOT_START)

    print(
        f"\n  Cena 1 — espaço aberto"
        f"\n  Start : {ROBOT_START}"
        f"\n  Goal  : {GOAL_POS[:2]}"
        f"\n  Obs   : {len(PILLAR_DEFS)} pilares 0.4×0.4 m"
        f"\n  Sem passagens estreitas → baseline de comparação"
    )


# ---------------------------------------------------------------------------
# Remoção
# ---------------------------------------------------------------------------

def remove_scene(sim):
    removed = 0
    try:
        handles = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    except Exception:
        handles = []

    to_remove = []
    for h in handles:
        try:
            alias = sim.getObjectAlias(h)
            short = alias.split("/")[-1]
            if short.lower().startswith("wall_"):
                to_remove.append((h, short))
        except Exception:
            pass

    for h, short in to_remove:
        try:
            sim.removeObject(h)
            print(f"  ✗ {short}")
            removed += 1
        except Exception:
            pass

    if removed == 0:
        known = ([f"wall_{s['name']}" for s in BOUNDARY_SEGMENTS] +
                 [f"wall_obs_{n}" for n, *_ in PILLAR_DEFS] +
                 [f"wall_{n}" for n, *_ in PILLAR_DEFS])
        for alias in known:
            try:
                sim.removeObject(sim.getObject(f"/{alias}"))
                print(f"  ✗ {alias} (fallback)")
                removed += 1
            except Exception:
                pass

    _fix_floor_name(sim)
    print(f"\n  {removed} objetos removidos.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Cria/remove Cena 1 (espaço aberto).")
    parser.add_argument("--remove",  action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    if args.dry_run and not args.remove:
        print("=== DRY RUN — Cena 1 ===\n")
        create_scene(None, dry_run=True)
        return

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")
    print("Conectado.\n")

    if not _ensure_sim_stopped(sim):
        print("[ATENÇÃO] Pare a simulação antes de continuar!")
        if input("  Continuar? [s/N] ").strip().lower() not in ("s", "sim", "y"):
            return

    if args.remove:
        print("Removendo Cena 1...\n")
        remove_scene(sim)
    else:
        print("Criando Cena 1 (espaço aberto)...\n")
        create_scene(sim)
        print("\n  Próximos passos: Ctrl+S → ▶ →")
        print("  python run_coppelia_batch.py --presets rrt rrt_connect_fast est_hybrid_safe --runs 20 --scene cena1")


if __name__ == "__main__":
    main()
