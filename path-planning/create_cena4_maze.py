"""Cria a Cena 4 — labirinto 2-anéis com múltiplas entradas.

Chão: deve se chamar "floor" (minúsculo) para que o planner detecte os limites.

Layout top-view (5 m × 5 m, y↑ x→):

  ╔══════════════════════════════════════════╗  boundary ±2.45 m
  ║                                          ║
  ║   ──[gap topo x±0.4]──   ──────────      ║  anel externo  ±1.6 m
  ║   │                                │     ║  (4 entradas de 0.8m)
  ║   │                                │     ║
  ║   │   ──[gap top x±0.4]──          │     ║  anel interno  ±0.6 m
  ║   gap                              gap   ║  (2 entradas de 0.8m
  ║   esq                G(0,0)        dir   ║   topo e fundo)
  ║   │   ──[gap bot x±0.4]──          │     ║
  ║   │                                │     ║
  ║   │                                │     ║
  ║   ──[gap inf x+0.6→+1.4]─                ║  ← entrada principal
  ║                              S↗          ║
  ╚══════════════════════════════════════════╝
                                S (+2.0, -2.0)

  Múltiplas entradas no anel externo (4):
    TOPO   y=+1.6 : x ∈ [-0.4, +0.4]     (0.8m)
    FUNDO  y=-1.6 : x ∈ [+0.6, +1.4]     (0.8m, alinhado com robô)
    ESQ.   x=-1.6 : y ∈ [-0.4, +0.4]     (0.8m)
    DIR.   x=+1.6 : y ∈ [-1.4, -0.6]     (0.8m, alinhado com robô)

  Múltiplas entradas no anel interno (2):
    TOPO   y=+0.6 : x ∈ [-0.4, +0.4]     (0.8m)
    FUNDO  y=-0.6 : x ∈ [-0.4, +0.4]     (0.8m)

  Larguras (todas seguras para step_size=1.2m do preset):
    Corredor externo (boundary → anel ext) : 0.82m → C-space 9.2px  ✓
    Corredor ext → int                     : 0.94m → C-space 11.6px ✓
    Gaps (todos)                           : 0.80m → C-space 8.8px  ✓

  Várias rotas possíveis — diferencia algoritmos por:
    - Como exploram opções (RRT amostra global, EST prioriza fronteira)
    - Comprimento do caminho escolhido (rotas curtas vs. longas)

Uso:
  python create_cena4_maze.py               # cria (início padrão: bottom_right)
  python create_cena4_maze.py --remove      # remove TODAS as paredes wall_*
  python create_cena4_maze.py --dry-run     # mostra posições sem criar
  python create_cena4_maze.py --start-pos top_left
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
# Parâmetros das paredes
# ---------------------------------------------------------------------------
WALL_THICKNESS = 0.06
WALL_HEIGHT    = 0.50
WALL_Z         = 0.25

# ---------------------------------------------------------------------------
# Labirinto — 3 anéis em espiral
# ---------------------------------------------------------------------------
#
#  Anel externo  ±1.8 m  — canto inf-dir ABERTO (x>+0.8 e y<0 livres)
#  Anel médio   ±1.15 m  — gap TOPO-DIR  y=+1.15, x: +0.10 → +0.80 (0.7 m)
#  Anel interno  ±0.5 m  — gap INFERIOR  y=-0.5,  x: -0.3 → +0.3  (0.6 m)
#
#  Corredor ext→méd = 1.80−1.15−0.06 = 0.59 m
#  Corredor méd→int = 1.15−0.50−0.06 = 0.59 m
#  Folga física     = 0.59 − 0.44(robô) = 0.15 m  ✓
#
#  Gap em pixels (mapa 100×100 px, escala 20 px/m):
#    Externo inf. = 0.7m = 14px → 14 − 2×3.6 =  6.8px livres  ✓
#    Médio topo   = 0.7m = 14px → 14 − 2×3.6 =  6.8px livres  ✓
#    Interno inf. = 0.6m = 12px → 12 − 2×3.6 =  4.8px livres  ✓  (mín ≈ 4px)

WALL_SEGMENTS = [
    # ── Anel externo (±1.6m) — 4 ENTRADAS (0.8m cada) ───────────────────────
    #
    #  TOPO   y=+1.6 : gap x ∈ [-0.4, +0.4]   (0.8m)
    #  FUNDO  y=-1.6 : gap x ∈ [+0.6, +1.4]   (0.8m — alinhado com robô)
    #  ESQ.   x=-1.6 : gap y ∈ [-0.4, +0.4]   (0.8m)
    #  DIR.   x=+1.6 : gap y ∈ [-1.4, -0.6]   (0.8m — alinhado com robô)
    #
    #  Corredor externo  : 2.45 − 1.60 − 0.03 = 0.82m → C-space 9.2px ✓
    #  Gaps              : 0.80m = 16px → C-space 8.8px ✓
    #
    # TOPO
    {"name": "outer_top_L",   "axis": "x", "cx": -1.00, "cy":  1.60, "length": 1.20},
    {"name": "outer_top_R",   "axis": "x", "cx":  1.00, "cy":  1.60, "length": 1.20},
    # FUNDO (gap deslocado p/ direita, alinhado com robô em x=+2.0)
    {"name": "outer_bot_L",   "axis": "x", "cx": -0.50, "cy": -1.60, "length": 2.20},
    {"name": "outer_bot_R",   "axis": "x", "cx":  1.50, "cy": -1.60, "length": 0.20},
    # ESQUERDA
    {"name": "outer_left_T",  "axis": "y", "cx": -1.60, "cy":  1.00, "length": 1.20},
    {"name": "outer_left_B",  "axis": "y", "cx": -1.60, "cy": -1.00, "length": 1.20},
    # DIREITA (gap deslocado p/ baixo, alinhado com robô em y=-2.0)
    {"name": "outer_right_T", "axis": "y", "cx":  1.60, "cy":  0.50, "length": 2.20},
    {"name": "outer_right_B", "axis": "y", "cx":  1.60, "cy": -1.50, "length": 0.20},

    # ── Anel interno (±0.6m) — 2 ENTRADAS (0.8m cada) ───────────────────────
    #
    #  TOPO  y=+0.6 : gap x ∈ [-0.4, +0.4]   (0.8m)
    #  FUNDO y=-0.6 : gap x ∈ [-0.4, +0.4]   (0.8m)
    #  ESQ.  x=-0.6 : completo
    #  DIR.  x=+0.6 : completo
    #
    #  Corredor ext→int  : 1.60 − 0.60 − 0.06 = 0.94m → C-space 11.6px ✓
    #  Gaps              : 0.80m = 16px → C-space 8.8px ✓
    #
    # TOPO (2 segmentos curtos de 0.2m)
    {"name": "inner_top_L",   "axis": "x", "cx": -0.50, "cy":  0.60, "length": 0.20},
    {"name": "inner_top_R",   "axis": "x", "cx":  0.50, "cy":  0.60, "length": 0.20},
    # FUNDO (2 segmentos curtos de 0.2m)
    {"name": "inner_bot_L",   "axis": "x", "cx": -0.50, "cy": -0.60, "length": 0.20},
    {"name": "inner_bot_R",   "axis": "x", "cx":  0.50, "cy": -0.60, "length": 0.20},
    # ESQUERDA e DIREITA (paredes completas)
    {"name": "inner_left",    "axis": "y", "cx": -0.60, "cy":  0.00, "length": 1.20},
    {"name": "inner_right",   "axis": "y", "cx":  0.60, "cy":  0.00, "length": 1.20},
]

BOUNDARY_SEGMENTS = [
    {"name": "boundary_N", "axis": "x", "cx":  0.00, "cy":  2.45, "length": 5.00},
    {"name": "boundary_S", "axis": "x", "cx":  0.00, "cy": -2.45, "length": 5.00},
    {"name": "boundary_W", "axis": "y", "cx": -2.45, "cy":  0.00, "length": 5.00},
    {"name": "boundary_E", "axis": "y", "cx":  2.45, "cy":  0.00, "length": 5.00},
]

GOAL_POS   = [0.0, 0.0, 0.05]
GOAL_ALIAS = "GoalConfiguration"
ROBOT_PATH = "/PioneerP3DX"
ROBOT_Z    = 0.14

# Posições de início — robô FORA do anel externo (±1.6 m)
# Corredor externo:  boundary ±2.45 m  ←→  anel ext ±1.6 m  (0.82 m de largura)
START_POSITIONS = {
    # Posições afastadas do canto (>= 0.40m de folga para paredes)
    # — corpo retangular da PioneerP3DX precisa de espaço para girar
    "bottom_right": ( 1.80, -2.00),   # padrão — afastado 0.65m da borda E
    "left":         (-2.00,  0.00),   # lado esquerdo (alinhado com gap esq)
    "top_left":     (-1.80,  2.00),   # canto sup-esq
    "top":          ( 0.00,  2.00),   # topo (alinhado com gap topo)
    "right":        ( 2.00,  0.00),   # lado direito
    "bottom":       ( 0.00, -2.00),   # fundo centro
}
DEFAULT_START = "bottom_right"

# O extractor (coppelia.py) usa floor_alias="floor" (minúsculo)
FLOOR_NAME = "floor"

PIONEER_MODEL_CANDIDATES = [
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\PioneerP3DX.ttm",
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSim\models\robots\mobile\PioneerP3DX.ttm",
    r"C:\Program Files (x86)\CoppeliaRobotics\CoppeliaSimEdu\models\robots\mobile\PioneerP3DX.ttm",
    r"C:\Program Files (x86)\CoppeliaRobotics\CoppeliaSim\models\robots\mobile\PioneerP3DX.ttm",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _wall_size(seg):
    return ([seg["length"], WALL_THICKNESS, WALL_HEIGHT] if seg["axis"] == "x"
            else [WALL_THICKNESS, seg["length"], WALL_HEIGHT])


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


def _create_wall(sim, alias, size, pos):
    # Remove se já existir — EVITA DUPLICATAS entre runs consecutivos
    try:
        old = sim.getObject(f"/{alias}")
        sim.removeObject(old)
    except Exception:
        pass
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(h, alias)
    sim.setObjectPosition(h, -1, pos)
    _make_static(sim, h)
    return h


def _ensure_sim_stopped(sim):
    try:
        return sim.getSimulationState() == getattr(sim, "simulation_stopped", 0)
    except Exception:
        return True


def _fix_floor_name(sim):
    """
    Garante que o chão se chama exatamente FLOOR_NAME ("floor" minúsculo),
    pois coppelia.py usa floor_alias="floor" e faz getObject("/floor").
    """
    # Já tem o nome correto?
    try:
        sim.getObject(f"/{FLOOR_NAME}")
        print(f"  ✓ Chão já se chama '{FLOOR_NAME}'  (planner detectará ±2.5 m)")
        return
    except Exception:
        pass

    # Tenta renomear pelos nomes mais comuns do CoppeliaSim
    candidates = [
        "/ResizableFloor_5_25", "/Floor", "/Plane",
        "/Floor_renamed", "/ResizableFloor", "/ground", "/floor0",
    ]
    for candidate in candidates:
        try:
            h = sim.getObject(candidate)
            sim.setObjectAlias(h, FLOOR_NAME)
            print(f"  ✓ Chão '{candidate.lstrip('/')}' → renomeado para '{FLOOR_NAME}'")
            return
        except Exception:
            continue

    # Última tentativa: varrer todos os shapes e renomear o maior (provável chão)
    try:
        handles = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
        largest_h = None
        largest_area = 0.0
        for h in handles:
            try:
                sx = (sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_x)
                    - sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_min_x))
                sy = (sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_y)
                    - sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_min_y))
                area = sx * sy
                if area > largest_area:
                    largest_area = area
                    largest_h = h
            except Exception:
                pass
        if largest_h is not None and largest_area > 4.0:
            old_name = sim.getObjectAlias(largest_h)
            sim.setObjectAlias(largest_h, FLOOR_NAME)
            print(f"  ✓ Chão '{old_name}' (maior shape, área≈{largest_area:.1f}m²) → '{FLOOR_NAME}'")
            return
    except Exception:
        pass

    print(f"  [aviso] Chão não encontrado — planner usará limites default (±5 m)")


# ---------------------------------------------------------------------------
# Robô
# ---------------------------------------------------------------------------

def _find_model_path(user_path):
    if user_path and os.path.exists(user_path):
        return user_path
    for p in PIONEER_MODEL_CANDIDATES:
        if os.path.exists(p):
            return p
    return None


def _position_robot(sim, xy, model_path=None):
    import math
    target = [xy[0], xy[1], ROBOT_Z]
    # Orienta o robô apontando para o goal — evita rotação grande no início
    # que travaria o controlador em corredores estreitos
    gx, gy = GOAL_POS[0], GOAL_POS[1]
    yaw = math.atan2(gy - xy[1], gx - xy[0])   # ângulo em rad (eixo z)
    orientation = [0.0, 0.0, yaw]

    try:
        h = sim.getObject(ROBOT_PATH)
        sim.setObjectPosition(h, -1, target)
        sim.setObjectOrientation(h, -1, orientation)
        print(f"  ✓ {ROBOT_PATH} posicionado em {target}  yaw={math.degrees(yaw):+.1f}°")
        return
    except Exception:
        pass
    resolved = _find_model_path(model_path)
    if resolved is None:
        print("  [aviso] PioneerP3DX.ttm não encontrado.\n"
              "  Use --robot-model <caminho.ttm>")
        return
    print(f"  Carregando: {resolved}")
    h = sim.loadModel(resolved)
    sim.setObjectPosition(h, -1, target)
    sim.setObjectOrientation(h, -1, orientation)
    print(f"  ✓ PioneerP3DX carregado em {target}  yaw={math.degrees(yaw):+.1f}°")


# ---------------------------------------------------------------------------
# Criação
# ---------------------------------------------------------------------------

def create_maze(sim, dry_run=False, start_key=DEFAULT_START, robot_model=None):
    xy = START_POSITIONS[start_key]

    if dry_run:
        print(f"  [chão]  será renomeado para '{FLOOR_NAME}' (sem criar novo)\n")
    else:
        print("Chão:\n")
        _fix_floor_name(sim)
        print()

    all_segments = [
        ("wall_maze_",  WALL_SEGMENTS),
        ("wall_",       BOUNDARY_SEGMENTS),
    ]

    for prefix, segments in all_segments:
        label = "Paredes do labirinto" if prefix == "wall_maze_" else "Paredes de borda"
        print(f"{label}:\n")
        for seg in segments:
            alias = f"{prefix}{seg['name']}"
            size  = _wall_size(seg)
            pos   = [seg["cx"], seg["cy"], WALL_Z]
            if dry_run:
                ax = "H" if seg["axis"] == "x" else "V"
                print(f"  [{ax}] {alias:<32}  "
                      f"size=[{size[0]:.2f},{size[1]:.2f},{size[2]:.2f}]  "
                      f"pos=[{pos[0]:+.3f},{pos[1]:+.3f}]")
            else:
                _create_wall(sim, alias, size, pos)
                print(f"  ✓ {alias}")
        print()

    if dry_run:
        print(f"  [goal]  /{GOAL_ALIAS}  {GOAL_POS}")
        print(f"  [robot] {ROBOT_PATH}  xy={xy}  z={ROBOT_Z}  (--start-pos {start_key})")
    else:
        try:
            gh = sim.getObject(f"/{GOAL_ALIAS}")
            sim.setObjectPosition(gh, -1, GOAL_POS)
            print(f"  ✓ /{GOAL_ALIAS}  → {GOAL_POS}")
        except Exception:
            gh = sim.createDummy(0.10)
            sim.setObjectAlias(gh, GOAL_ALIAS)
            sim.setObjectPosition(gh, -1, GOAL_POS)
            print(f"  ✓ /{GOAL_ALIAS}  criado em {GOAL_POS}")
        print()
        _position_robot(sim, xy, robot_model)

    print(
        f"\n  Corredores:"
        f"\n    Fora → externo  : 0.62 m  (boundary ±2.45 m ↔ externo ±1.80 m)"
        f"\n    Externo → médio : 0.59 m  (±1.80 m ↔ ±1.15 m)"
        f"\n    Médio → interno : 0.59 m  (±1.15 m ↔ ±0.50 m)"
        f"\n    Espaço interno  : 1.0 × 1.0 m"
        f"\n    Robô Ø = 0.44 m  →  folga 0.15 m em cada corredor  ✓"
        f"\n"
        f"\n  Gaps:"
        f"\n    Externo esq.  x = -1.80  y: -0.50 → +0.50  (1.0 m)"
        f"\n    Médio topo-d. y = +1.15  x: +0.15 → +0.65  (0.5 m)"
        f"\n    Interno inf.  y = -0.50  x: -0.20 → +0.20  (0.4 m)"
        f"\n"
        f"\n  Posições de início (FORA do anel externo):"
    )
    for key, pos in START_POSITIONS.items():
        marker = "  ◄ atual" if key == start_key else ""
        print(f"    --start-pos {key:<12}  xy={pos}{marker}")


# ---------------------------------------------------------------------------
# Remoção
# ---------------------------------------------------------------------------

def remove_maze(sim):
    """Remove TODAS as paredes wall_* da cena (de qualquer versão do script)."""
    removed = 0

    # Abordagem 1: varrer todos os shapes (mesmo método do extractor coppelia.py)
    try:
        shape_handles = sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        )
    except Exception as e:
        print(f"  [aviso] getObjectsInTree falhou ({e}); usando fallback por nome...")
        shape_handles = []

    # Coleta primeiro para evitar invalidação de handles durante remoção
    walls_to_remove = []
    for h in shape_handles:
        try:
            alias = sim.getObjectAlias(h)
            # Normaliza: "/wall_maze_outer_top" → "wall_maze_outer_top"
            short = alias.split("/")[-1] if alias else ""
            if short.lower().startswith("wall_"):
                walls_to_remove.append((h, short))
        except Exception:
            pass

    for h, short in walls_to_remove:
        try:
            sim.removeObject(h)
            print(f"  ✗ {short} removido")
            removed += 1
        except Exception as e:
            print(f"  – {short} erro: {e}")

    # Abordagem 2 (fallback): remove por nome explícito
    if removed == 0 and not walls_to_remove:
        known = (
            [f"wall_maze_{s['name']}" for s in WALL_SEGMENTS] +
            [f"wall_{s['name']}"      for s in BOUNDARY_SEGMENTS] +
            # Nomes de versões anteriores
            ["wall_maze_mid_top",    "wall_maze_mid_bot",
             "wall_maze_mid_left",   "wall_maze_mid_right",
             "wall_maze_outer_top",  "wall_maze_outer_bot",
             "wall_maze_outer_left", "wall_maze_outer_right",
             "wall_maze_inner_top",  "wall_maze_inner_bot",
             "wall_maze_inner_left", "wall_maze_inner_right",
             "wall_boundary_N", "wall_boundary_S",
             "wall_boundary_W", "wall_boundary_E",
             "wall_N", "wall_S", "wall_E", "wall_W"]
        )
        for alias in known:
            try:
                h = sim.getObject(f"/{alias}")
                sim.removeObject(h)
                print(f"  ✗ /{alias} removido (fallback)")
                removed += 1
            except Exception:
                pass

    _fix_floor_name(sim)
    print(f"\n  {removed} paredes removidas.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Cria/remove o labirinto 3-anéis (Cena 4).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--remove",  action="store_true",
                        help="Remove TODAS as paredes wall_* da cena.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Mostra posições sem criar nada.")
    parser.add_argument("--start-pos", default=DEFAULT_START,
                        choices=list(START_POSITIONS.keys()),
                        help=f"Posição inicial do robô (padrão: {DEFAULT_START}).")
    parser.add_argument("--robot-model", default=None, metavar="ARQUIVO.ttm")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    if args.dry_run and not args.remove:
        print("=== DRY RUN ===\n")
        create_maze(None, dry_run=True, start_key=args.start_pos)
        return

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    print(f"Conectando a {args.host}:{args.port} ...")
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")
    print("Conectado.\n")

    if not _ensure_sim_stopped(sim):
        print("[ATENÇÃO] Pare a simulação (■) antes de continuar!\n")
        if input("  Continuar mesmo assim? [s/N] ").strip().lower() not in ("s","sim","y","yes"):
            print("Abortado.")
            return

    if args.remove:
        print("Removendo Cena 4...\n")
        remove_maze(sim)
    else:
        print(f"Criando labirinto 3-anéis (Cena 4) — início: {args.start_pos}...\n")
        create_maze(sim, start_key=args.start_pos, robot_model=args.robot_model)
        print(
            "\n──────────────────────────────────────────────────────────\n"
            "Próximos passos:\n"
            "  1. Pare a simulação (■) se estiver rodando.\n"
            "  2. Salve a cena (Ctrl+S).\n"
            "  3. Inicie a simulação (▶).\n"
            "  4. Teste com 1 rodada:\n"
            "       python run_coppelia_batch.py --presets rrt_connect_fast --runs 1 --scene cena4\n"
            "  5. Se o robô andar, rode completo:\n"
            "       python run_coppelia_batch.py --presets rrt rrt_connect_fast est --runs 20 --scene cena4\n"
            "  6. Para testar diferentes pontos de início:\n"
            "       python create_cena4_maze.py --start-pos right\n"
            "       (Ctrl+S → ▶ → rodar batch)\n"
            "──────────────────────────────────────────────────────────"
        )


if __name__ == "__main__":
    main()
