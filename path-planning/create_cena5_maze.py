"""Cena 5 — Corredor obrigatório + labirinto interior.

Robô começa fora, atravessa um corredor estreito (0.8 m), depois navega num
mini-labirinto até alcançar o goal protegido por uma câmara em "U".

Layout (5 m × 5 m, y↑ x→):

  ╔══════════════════════════════════════════╗  boundary ±2.45 m
  ║                                          ║
  ║   ┌─────────┐                            ║
  ║   │  goal   │                            ║  câmara em "U"
  ║   │ (0,1.0) │                            ║  (entrada por baixo)
  ║   └─[gap]───┘                            ║
  ║                                          ║
  ║         OBSTÁCULO          OBSTÁCULO     ║  bloqueios laterais
  ║                                          ║
  ║───────────       ───────────────────     ║  divisor y=-0.5
  ║          │       │                       ║  (gap = corredor)
  ║          │       │                       ║
  ║          │       │                       ║
  ║          │  COR. │                       ║  corredor estreito
  ║          │       │                       ║  0.8m de largura
  ║          │  S    │                       ║
  ║          │       │                       ║
  ╚══════════════════════════════════════════╝
                                              S(-0.8, -2.2)

  Percurso obrigatório:
    S → sobe pelo corredor (forced narrow passage)
      → emerge no espaço aberto em y > -0.5
      → contorna obstáculos laterais
      → entra na câmara do goal por baixo (gap x ∈ [-0.2, +0.2])
      → chega ao goal (0, +1.0)

  Diferencial p/ a comparação de algoritmos:
    - Corredor de 0.8m → RRT-Connect domina (busca bidirecional)
    - Câmara com gap único → tester de exploração focada
    - EST puro tende a falhar (sem global sampling pra achar o gap da câmara)

Uso:
  python create_cena5_maze.py
  python create_cena5_maze.py --remove
  python create_cena5_maze.py --dry-run
"""

import argparse
import math
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
FLOOR_NAME     = "floor"
GOAL_ALIAS     = "GoalConfiguration"
GOAL_POS       = [0.0, 1.0, 0.05]
ROBOT_PATH     = "/PioneerP3DX"
ROBOT_Z        = 0.14
ROBOT_START    = (-0.8, -2.20)

WALL_SEGMENTS = [
    # ── CORREDOR estreito 0.8m (y de -2.45 a -0.5) ───────────────────────────
    # Parede esquerda do corredor: x=-1.2, y ∈ [-2.45, -0.5]  cy=-1.475 len=1.95
    {"name": "corr_left",   "axis": "y", "cx": -1.20, "cy": -1.475, "length": 1.95},
    # Parede direita do corredor: x=-0.4, y ∈ [-2.45, -0.5]  cy=-1.475 len=1.95
    {"name": "corr_right",  "axis": "y", "cx": -0.40, "cy": -1.475, "length": 1.95},

    # ── DIVISOR em y=-0.5 com gap alinhado com corredor (x ∈ [-1.2, -0.4]) ──
    # Esquerda: x ∈ [-2.45, -1.20]  cx=-1.825 len=1.25
    {"name": "div_left",    "axis": "x", "cx": -1.825, "cy": -0.50, "length": 1.25},
    # Direita: x ∈ [-0.40, +2.45]   cx=+1.025 len=2.85
    {"name": "div_right",   "axis": "x", "cx":  1.025, "cy": -0.50, "length": 2.85},

    # ── OBSTÁCULOS LATERAIS no meio do espaço aberto ─────────────────────────
    # Bloqueia atalho direto: força contornar para acessar a câmara do goal
    {"name": "obs_left",    "axis": "y", "cx": -1.50, "cy":  0.40, "length": 1.20},  # y ∈ [-0.2, +1.0]
    {"name": "obs_right",   "axis": "y", "cx":  1.50, "cy":  0.40, "length": 1.20},  # y ∈ [-0.2, +1.0]

    # ── CÂMARA em "U" envolvendo o goal (0, +1.0) ─────────────────────────────
    # Topo:    y=+1.4, x ∈ [-0.5, +0.5]   cx=0 len=1.0
    {"name": "chamber_top", "axis": "x", "cx":  0.00, "cy":  1.40, "length": 1.00},
    # Esq.:    x=-0.5, y ∈ [+0.6, +1.4]   cy=+1.0 len=0.8
    {"name": "chamber_lft", "axis": "y", "cx": -0.50, "cy":  1.00, "length": 0.80},
    # Dir.:    x=+0.5, y ∈ [+0.6, +1.4]   cy=+1.0 len=0.8
    {"name": "chamber_rgt", "axis": "y", "cx":  0.50, "cy":  1.00, "length": 0.80},
    # Fundo com gap centralizado: 2 segmentos de 0.3m, gap x ∈ [-0.2, +0.2] = 0.4m
    {"name": "chamber_btL", "axis": "x", "cx": -0.35, "cy":  0.60, "length": 0.30},  # x ∈ [-0.5, -0.2]
    {"name": "chamber_btR", "axis": "x", "cx":  0.35, "cy":  0.60, "length": 0.30},  # x ∈ [+0.2, +0.5]
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
# Helpers
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
        print(f"  ✓ Chão '{FLOOR_NAME}' detectado")
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
    print(f"  [aviso] Chão não encontrado")


def _position_robot(sim, xy, model_path=None):
    target = [xy[0], xy[1], ROBOT_Z]
    # Orienta o robô apontando para o goal — evita rotação grande inicial
    gx, gy = GOAL_POS[0], GOAL_POS[1]
    yaw = math.atan2(gy - xy[1], gx - xy[0])
    orientation = [0.0, 0.0, yaw]
    try:
        h = sim.getObject(ROBOT_PATH)
        sim.setObjectPosition(h, -1, target)
        sim.setObjectOrientation(h, -1, orientation)
        print(f"  ✓ {ROBOT_PATH} → {target}  yaw={math.degrees(yaw):+.1f}°")
        return
    except Exception:
        pass
    resolved = next((p for p in PIONEER_MODEL_CANDIDATES if os.path.exists(p)), model_path)
    if not resolved:
        print("  [aviso] PioneerP3DX.ttm não encontrado.")
        return
    h = sim.loadModel(resolved)
    sim.setObjectPosition(h, -1, target)
    sim.setObjectOrientation(h, -1, orientation)
    print(f"  ✓ PioneerP3DX carregado em {target}  yaw={math.degrees(yaw):+.1f}°")


def _ensure_sim_stopped(sim):
    try:
        return sim.getSimulationState() == getattr(sim, "simulation_stopped", 0)
    except Exception:
        return True


# ---------------------------------------------------------------------------
# Criação / Remoção
# ---------------------------------------------------------------------------

def create_scene(sim, dry_run=False):
    print("Chão:\n")
    if not dry_run:
        _fix_floor_name(sim)
    print()

    print("Corredor + divisor + obstáculos + câmara:\n")
    for seg in WALL_SEGMENTS:
        alias = f"wall_{seg['name']}"
        if seg["axis"] == "x":
            size = [seg["length"], WALL_THICKNESS, WALL_HEIGHT]
        else:
            size = [WALL_THICKNESS, seg["length"], WALL_HEIGHT]
        pos = [seg["cx"], seg["cy"], WALL_Z]
        if dry_run:
            print(f"  [W] {alias:<24} pos=[{pos[0]:+.2f},{pos[1]:+.2f}] size={size[0]:.2f}×{size[1]:.2f}")
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
            print(f"  [B] {alias:<24} pos=[{pos[0]:+.2f},{pos[1]:+.2f}]")
        else:
            _create_shape(sim, alias, size, pos)
            print(f"  ✓ {alias}")

    if dry_run:
        print(f"\n  [goal]  /{GOAL_ALIAS}  {GOAL_POS}")
        print(f"  [robot] {ROBOT_PATH}  xy={ROBOT_START}")
        return

    try:
        gh = sim.getObject(f"/{GOAL_ALIAS}")
        sim.setObjectPosition(gh, -1, GOAL_POS)
        print(f"\n  ✓ /{GOAL_ALIAS} → {GOAL_POS}")
    except Exception:
        gh = sim.createDummy(0.10)
        sim.setObjectAlias(gh, GOAL_ALIAS)
        sim.setObjectPosition(gh, -1, GOAL_POS)
        print(f"\n  ✓ /{GOAL_ALIAS} criado em {GOAL_POS}")

    print()
    _position_robot(sim, ROBOT_START)

    print(
        f"\n  Cena 5 — corredor + labirinto + câmara do goal"
        f"\n  Start  : {ROBOT_START}   (dentro do corredor)"
        f"\n  Goal   : {GOAL_POS[:2]}   (dentro da câmara em U)"
        f"\n  Largura do corredor : 0.80 m"
        f"\n  Gap da câmara       : 0.40 m"
    )


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
    _fix_floor_name(sim)
    print(f"\n  {removed} objetos removidos.")


def main():
    parser = argparse.ArgumentParser(description="Cria/remove Cena 5 (corredor + labirinto).")
    parser.add_argument("--remove",  action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    if args.dry_run and not args.remove:
        print("=== DRY RUN — Cena 5 ===\n")
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
        print("Removendo Cena 5...\n")
        remove_scene(sim)
    else:
        print("Criando Cena 5 (corredor + labirinto)...\n")
        create_scene(sim)
        print("\n  Próximos passos: Ctrl+S → ▶ →")
        print("  python run_coppelia_batch.py --presets rrt rrt_connect_fast est est_hybrid_exploratory --runs 10 --scene cena5")


if __name__ == "__main__":
    main()
