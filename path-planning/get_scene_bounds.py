"""Imprime as dimensões reais da cena atual do CoppeliaSim.

Uso:
  python get_scene_bounds.py

Pré-requisito: CoppeliaSim aberto com a cena carregada.
"""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
COPPELIASIM_DIR = os.path.join(SCRIPT_DIR, "coppeliasim")
for _d in [SCRIPT_DIR, COPPELIASIM_DIR]:
    if _d not in sys.path:
        sys.path.insert(0, _d)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient("localhost", 23000)
sim = client.getObject("sim")

# Tenta encontrar o chão
floor = None
for name in ("/floor", "/Floor", "/ResizableFloor_5_25"):
    try:
        floor = sim.getObject(name)
        print(f"Chão encontrado: {name}")
        break
    except Exception:
        continue

if floor is None:
    print("Chão não encontrado. Tentando detectar pelos limites do robô...")
    sys.exit(1)

min_x = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_min_x)
max_x = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_max_x)
min_y = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_min_y)
max_y = sim.getObjectFloatParam(floor, sim.objfloatparam_objbbox_max_y)
matrix = sim.getObjectMatrix(floor, -1)


def transform(px, py):
    wx = matrix[0] * px + matrix[1] * py + matrix[3]
    wy = matrix[4] * px + matrix[5] * py + matrix[7]
    return wx, wy


corners = [
    transform(min_x, min_y),
    transform(max_x, min_y),
    transform(max_x, max_y),
    transform(min_x, max_y),
]

xs = [c[0] for c in corners]
ys = [c[1] for c in corners]

w = max(xs) - min(xs)
h = max(ys) - min(ys)

print(f"\nDimensão real:   {w:.2f} m x {h:.2f} m")
print(f"Mapa discreto:   100 x 100 pixels")
print(f"Resolução:       {w/100:.4f} m/pixel x {h/100:.4f} m/pixel")
print(f"\n(use estes valores na tabela do TCC)")
