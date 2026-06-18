"""Cria a cena3: adiciona obstáculos pequenos dispersos à cena atual do CoppeliaSim.

Pré-requisito: CoppeliaSim com a cena da cena1 aberta e simulação PARADA.

Uso:
  python create_cena3.py                  # adiciona obstáculos com layout padrão
  python create_cena3.py --dry-run        # mostra posições sem criar nada
  python create_cena3.py --remove         # remove todos wall_scatter_ da cena
  python create_cena3.py --size 0.30      # obstáculos de 30cm de lado
  python create_cena3.py --n-obstacles 20 # 20 obstáculos em vez de 16

Após rodar:
  1. Verifique visualmente na cena do CoppeliaSim se os obstáculos estão bem postos.
  2. Salve a cena como "cena3.ttt" (File → Save scene as…).
  3. Rode o batch: python run_coppelia_batch.py --presets rrt rrt_connect_fast
       est_hybrid_exploratory est_hybrid_safe --runs 10 --scene cena3
"""

import argparse
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


# ---------------------------------------------------------------------------
# Layout normalizado de obstáculos (0..1 no espaço do mundo)
# Cada entrada: (x_norm, y_norm)
# O script calcula a posição real baseado nos limites do chão.
# ---------------------------------------------------------------------------

# 16 posições distribuídas em 4 colunas × 4 linhas com offset alternado ("tijoleira")
# Evitam as bordas (margem 15%) e criam caminhos alternativos entre eles.
_LAYOUT_4x4 = [
    # col 1  (x≈20%)
    (0.18, 0.20), (0.18, 0.45), (0.18, 0.68), (0.18, 0.85),
    # col 2  (x≈38%, offset y)
    (0.38, 0.30), (0.38, 0.55), (0.38, 0.78),
    # col 3  (x≈58%)
    (0.58, 0.18), (0.58, 0.42), (0.58, 0.65), (0.58, 0.88),
    # col 4  (x≈75%, offset y)
    (0.75, 0.28), (0.75, 0.52), (0.75, 0.75),
    # extras para distribuição mais irregular
    (0.28, 0.12), (0.48, 0.92),
]

# Posições adicionais (usadas quando --n-obstacles > 16)
_EXTRA_POSITIONS = [
    (0.10, 0.55), (0.90, 0.45), (0.32, 0.38),
    (0.65, 0.30), (0.85, 0.70), (0.45, 0.60),
]


def _get_floor_bounds(sim):
    """Tenta detectar bounds do chão; retorna (x_min, x_max, y_min, y_max)."""
    try:
        floor = sim.getObject("/floor")
    except Exception:
        try:
            floor = sim.getObject("/Floor")
        except Exception:
            print("  [aviso] Chão não encontrado, usando bounds padrão ±3.0m")
            return (-3.0, 3.0, -3.0, 3.0)

    x_min_l = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_min_x)
    x_max_l = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_max_x)
    y_min_l = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_min_y)
    y_max_l = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_max_y)
    matrix = sim.getObjectMatrix(floor, -1)

    def _t(px, py):
        wx = matrix[0]*px + matrix[1]*py + matrix[3]
        wy = matrix[4]*px + matrix[5]*py + matrix[7]
        return wx, wy

    corners = [_t(x_min_l, y_min_l), _t(x_max_l, y_min_l),
               _t(x_max_l, y_max_l), _t(x_min_l, y_max_l)]
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    return (min(xs), max(xs), min(ys), max(ys))


def _get_robot_goal_positions(sim):
    """Retorna (robot_xy, goal_xy) ou (None, None) se não encontrado."""
    robot_xy = goal_xy = None
    for path in ("/PioneerP3DX", "/Pioneer_p3dx"):
        try:
            h = sim.getObject(path)
            p = sim.getObjectPosition(h, -1)
            robot_xy = (p[0], p[1])
            break
        except Exception:
            continue
    for path in ("/GoalConfiguration", "/Goal", "/goal"):
        try:
            h = sim.getObject(path)
            p = sim.getObjectPosition(h, -1)
            goal_xy = (p[0], p[1])
            break
        except Exception:
            continue
    return robot_xy, goal_xy


def _dist_to_segment(px, py, ax, ay, bx, by):
    """Distância do ponto (px,py) ao segmento (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax)*dx + (py - ay)*dy) / (dx*dx + dy*dy)))
    return math.hypot(px - (ax + t*dx), py - (ay + t*dy))


def compute_positions(
    n_obstacles: int,
    bounds,
    robot_xy,
    goal_xy,
    obs_size: float,
    min_clearance_start_goal: float = 0.60,
):
    """
    Converte layout normalizado em coordenadas mundo.
    Remove posições muito próximas do robô/goal.
    """
    x_min, x_max, y_min, y_max = bounds
    # Margem para não encostar nas paredes externas
    margin = 0.20
    xw = x_max - x_min
    yw = y_max - y_min

    all_norm = _LAYOUT_4x4 + _EXTRA_POSITIONS
    positions = []
    for xn, yn in all_norm:
        wx = x_min + margin + xn * (xw - 2*margin)
        wy = y_min + margin + yn * (yw - 2*margin)

        # Verifica distância ao robô e ao goal
        if robot_xy:
            if math.hypot(wx - robot_xy[0], wy - robot_xy[1]) < min_clearance_start_goal + obs_size:
                continue
        if goal_xy:
            if math.hypot(wx - goal_xy[0], wy - goal_xy[1]) < min_clearance_start_goal + obs_size:
                continue
        # Verifica distância ao segmento direto robô→goal (deixa corredor central livre)
        if robot_xy and goal_xy:
            d = _dist_to_segment(wx, wy, robot_xy[0], robot_xy[1], goal_xy[0], goal_xy[1])
            if d < 0.40:  # Mantém uma faixa de 40cm livre ao longo do caminho direto
                continue

        positions.append((wx, wy))
        if len(positions) >= n_obstacles:
            break

    if len(positions) < n_obstacles:
        print(f"  [aviso] Só foi possível posicionar {len(positions)} obstáculos "
              f"(pedido: {n_obstacles}). Reduza --n-obstacles ou a margem de clearance.")
    return positions


def remove_scatter_obstacles(sim):
    """Remove todos os objetos com nome wall_scatter_* da cena."""
    try:
        handles = sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        )
    except Exception as e:
        print(f"  [erro] Não foi possível listar objetos: {e}")
        return 0

    removed = 0
    for h in handles:
        try:
            alias = sim.getObjectAlias(h)
        except Exception:
            continue
        name = alias.split("/")[-1].lower()
        if name.startswith("wall_scatter"):
            sim.removeObject(h)
            removed += 1
            print(f"  ✗ Removido: {alias}")
    return removed


def create_scatter_obstacles(
    sim,
    positions,
    obs_size: float = 0.25,
    obs_height: float = 0.35,
    floor_z: float = 0.0,
    color_rgb: tuple = (0.7, 0.35, 0.1),
):
    """Cria cuboides pequenos dispersos e os nomeia wall_scatter_NN."""
    created = []
    z_center = floor_z + obs_height / 2.0

    for i, (wx, wy) in enumerate(positions, 1):
        try:
            handle = sim.createPrimitiveShape(
                sim.primitiveshape_cuboid,
                [obs_size, obs_size, obs_height],
                0,
            )
            alias = f"wall_scatter_{i:02d}"
            sim.setObjectAlias(handle, alias)
            sim.setObjectPosition(handle, -1, [wx, wy, z_center])

            # Cor laranja-acastanhado para distinguir das paredes originais
            try:
                sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse,
                                  list(color_rgb))
            except Exception:
                pass  # cor não é crítica

            # Torna o objeto estático (não afetado pela física)
            try:
                sim.setObjectInt32Param(
                    handle, sim.shapeintparam_static, 1
                )
                sim.setObjectInt32Param(
                    handle, sim.shapeintparam_respondable, 1
                )
            except Exception:
                pass

            created.append((alias, wx, wy))
            print(f"  ✓ {alias}  em  ({wx:+.3f}, {wy:+.3f}, {z_center:.3f})")
        except Exception as e:
            print(f"  [erro] Obstáculo {i}: {e}")

    return created


def main():
    parser = argparse.ArgumentParser(
        description="Adiciona obstáculos dispersos à cena atual do CoppeliaSim (cena3).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--size",  type=float, default=0.25,
                        help="Tamanho (lado) de cada obstáculo em metros (padrão: 0.25)")
    parser.add_argument("--height", type=float, default=0.35,
                        help="Altura de cada obstáculo em metros (padrão: 0.35)")
    parser.add_argument("--n-obstacles", type=int, default=16,
                        help="Número de obstáculos a criar (padrão: 16)")
    parser.add_argument("--min-clearance", type=float, default=0.60,
                        help="Distância mínima de cada obstáculo ao robô/goal em metros (padrão: 0.60)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Mostra posições calculadas sem criar objetos")
    parser.add_argument("--remove", action="store_true",
                        help="Remove todos wall_scatter_* da cena e sai")
    args = parser.parse_args()

    print(f"Conectando ao CoppeliaSim em {args.host}:{args.port}...")
    client = RemoteAPIClient(args.host, args.port)
    sim = client.getObject("sim")
    print("Conectado.\n")

    if args.remove:
        n = remove_scatter_obstacles(sim)
        print(f"\n✓ {n} obstáculo(s) removido(s).")
        return 0

    # Detecta geometria da cena
    bounds = _get_floor_bounds(sim)
    print(f"Limites do mapa: x=[{bounds[0]:.2f}, {bounds[1]:.2f}]  "
          f"y=[{bounds[2]:.2f}, {bounds[3]:.2f}]")

    robot_xy, goal_xy = _get_robot_goal_positions(sim)
    if robot_xy:
        print(f"Robô em: ({robot_xy[0]:.3f}, {robot_xy[1]:.3f})")
    if goal_xy:
        print(f"Goal em: ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})")
    print()

    positions = compute_positions(
        n_obstacles=args.n_obstacles,
        bounds=bounds,
        robot_xy=robot_xy,
        goal_xy=goal_xy,
        obs_size=args.size,
        min_clearance_start_goal=args.min_clearance,
    )

    print(f"{len(positions)} posições calculadas:\n")
    for i, (x, y) in enumerate(positions, 1):
        print(f"  {i:2d}. ({x:+.3f}, {y:+.3f})")

    if args.dry_run:
        print("\n[--dry-run] Nenhum objeto criado.")
        return 0

    print(f"\nCriando {len(positions)} obstáculos na cena...\n")
    created = create_scatter_obstacles(
        sim, positions,
        obs_size=args.size,
        obs_height=args.height,
    )

    print(f"\n{'='*60}")
    print(f"✓ {len(created)} obstáculo(s) criado(s) com sucesso!")
    print()
    print("Próximos passos:")
    print("  1. Verifique a cena no CoppeliaSim (os obstáculos devem aparecer em")
    print("     laranja-marrom dispersos pela área).")
    print("  2. Se precisar remover algum manualmente, clique nele e apague.")
    print("  3. Salve como cena3: File → Save scene as… → cena3.ttt")
    print("  4. Rode o batch:")
    print("     python run_coppelia_batch.py \\")
    print("       --presets rrt rrt_connect_fast est_hybrid_exploratory est_hybrid_safe \\")
    print("       --runs 10 --scene cena3")
    print(f"{'='*60}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
