"""Aplica textura procedural (madeira, pedra, etc.) no chão do CoppeliaSim.

Gera uma textura usando PIL e a aplica como imagem no chão. Mais realista
que aplicar só uma cor sólida.

Uso:
  python apply_floor_texture.py --pattern wood        # tábuas de madeira clara
  python apply_floor_texture.py --pattern wood_dark   # tábuas escuras
  python apply_floor_texture.py --pattern stone       # pedra
  python apply_floor_texture.py --pattern marble      # mármore
  python apply_floor_texture.py --pattern tile        # azulejo branco
  python apply_floor_texture.py --pattern carpet      # carpete bege

Pré-requisitos:
  pip install pillow
"""

import argparse
import math
import os
import random
import sys

try:
    from PIL import Image, ImageDraw, ImageFilter
except ImportError:
    print("[ERRO] biblioteca PIL não instalada.")
    print("Execute: pip install pillow")
    sys.exit(1)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEXTURES_DIR = os.path.join(SCRIPT_DIR, "textures")


# ---------------------------------------------------------------------------
# Geradores de textura
# ---------------------------------------------------------------------------

def generate_wood_planks(width=512, height=512, plank_width=64, light=True):
    """Gera tábuas de madeira simulada."""
    img = Image.new("RGB", (width, height))
    draw = ImageDraw.Draw(img)

    if light:
        base_color = (180, 130, 80)   # madeira clara
        dark_color = (130, 80, 40)
        line_color = (90, 50, 25)
    else:
        base_color = (100, 60, 30)    # madeira escura
        dark_color = (60, 35, 15)
        line_color = (30, 15, 5)

    # Cada tábua
    for i, x in enumerate(range(0, width, plank_width)):
        # Variação aleatória de tom por tábua
        variation = random.randint(-25, 25)
        r = max(0, min(255, base_color[0] + variation))
        g = max(0, min(255, base_color[1] + variation))
        b = max(0, min(255, base_color[2] + variation))
        draw.rectangle([x, 0, x + plank_width, height], fill=(r, g, b))

        # Linhas de divisão entre tábuas
        if x > 0:
            draw.line([(x, 0), (x, height)], fill=line_color, width=2)

        # Veios de madeira (linhas horizontais sutis)
        for _ in range(8):
            y = random.randint(0, height)
            grain_r = max(0, r - random.randint(15, 35))
            grain_g = max(0, g - random.randint(15, 35))
            grain_b = max(0, b - random.randint(15, 35))
            draw.line([(x, y), (x + plank_width, y)],
                      fill=(grain_r, grain_g, grain_b), width=1)

    # Suaviza levemente pra parecer mais natural
    img = img.filter(ImageFilter.SMOOTH)
    return img


def generate_stone(width=512, height=512):
    """Gera pedra cinza com variação."""
    img = Image.new("RGB", (width, height), (140, 140, 140))
    draw = ImageDraw.Draw(img)
    for _ in range(3000):
        x, y = random.randint(0, width), random.randint(0, height)
        c = random.randint(120, 170)
        draw.point((x, y), fill=(c, c, c))
    img = img.filter(ImageFilter.SMOOTH)
    return img


def generate_marble(width=512, height=512):
    """Gera mármore branco com veios cinza."""
    img = Image.new("RGB", (width, height), (245, 240, 235))
    draw = ImageDraw.Draw(img)
    # Veios irregulares
    for _ in range(30):
        x1 = random.randint(0, width)
        y1 = random.randint(0, height)
        x2 = x1 + random.randint(-100, 100)
        y2 = y1 + random.randint(-100, 100)
        draw.line([(x1, y1), (x2, y2)],
                  fill=(180, 175, 170), width=random.randint(1, 3))
    img = img.filter(ImageFilter.GaussianBlur(radius=1))
    return img


def generate_tile(width=512, height=512, tile_size=64):
    """Gera azulejos brancos com grout cinza."""
    img = Image.new("RGB", (width, height), (240, 240, 240))
    draw = ImageDraw.Draw(img)
    for x in range(0, width, tile_size):
        draw.line([(x, 0), (x, height)], fill=(150, 150, 150), width=2)
    for y in range(0, height, tile_size):
        draw.line([(0, y), (width, y)], fill=(150, 150, 150), width=2)
    return img


def generate_carpet(width=512, height=512):
    """Gera textura de carpete bege."""
    img = Image.new("RGB", (width, height), (200, 180, 150))
    draw = ImageDraw.Draw(img)
    for _ in range(8000):
        x, y = random.randint(0, width), random.randint(0, height)
        c_r = 200 + random.randint(-30, 30)
        c_g = 180 + random.randint(-30, 30)
        c_b = 150 + random.randint(-30, 30)
        draw.point((x, y), fill=(c_r, c_g, c_b))
    return img


PATTERNS = {
    "wood":       lambda: generate_wood_planks(light=True),
    "wood_dark":  lambda: generate_wood_planks(light=False),
    "stone":      generate_stone,
    "marble":     generate_marble,
    "tile":       generate_tile,
    "carpet":     generate_carpet,
}


# ---------------------------------------------------------------------------
# Aplicação no CoppeliaSim
# ---------------------------------------------------------------------------

def apply_texture_to_floor(sim, texture_path):
    """Aplica a textura no chão da cena ativa."""
    # Procura o objeto floor
    try:
        floor = sim.getObject('/floor')
    except Exception:
        try:
            # Tenta alternativas
            for name in ['/Floor', '/ResizableFloor_5_25', '/Plane']:
                try:
                    floor = sim.getObject(name)
                    sim.setObjectAlias(floor, 'floor')
                    break
                except Exception:
                    continue
            else:
                print("[ERRO] Não consegui encontrar o chão (floor).")
                return False
        except Exception:
            return False

    # Carrega a textura como imagem na cena
    try:
        # Cria textura a partir do arquivo
        result = sim.createTexture(texture_path, 0)
        # result pode ser uma tupla (shape_handle, texture_id, resolution)
        if isinstance(result, (list, tuple)):
            if len(result) >= 2:
                texture_id = result[1]
            else:
                texture_id = result[0]
            # Remove o shape auxiliar criado
            try:
                if len(result) > 0:
                    sim.removeObject(result[0])
            except Exception:
                pass
        else:
            texture_id = result

        # Aplica a textura no chão
        sim.setShapeTexture(
            floor,
            texture_id,
            sim.texturemap_plane,
            4,            # opções (1=interpoladar, 4=repete textura)
            [5.0, 5.0],   # repetições (5x5 vezes pela área)
        )
        print(f"  ✓ Textura aplicada no chão")
        return True
    except Exception as e:
        print(f"  [erro ao aplicar textura] {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Aplica textura procedural no chão do CoppeliaSim.")
    parser.add_argument(
        "--pattern",
        default="wood",
        choices=list(PATTERNS.keys()) + ["list"],
        help="Padrão de textura (use 'list' pra ver opções).",
    )
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    if args.pattern == "list":
        print("\nPadrões disponíveis:")
        for p in PATTERNS:
            print(f"  - {p}")
        return 0

    # Gera a textura
    os.makedirs(TEXTURES_DIR, exist_ok=True)
    texture_path = os.path.join(TEXTURES_DIR, f"floor_{args.pattern}.png")
    print(f"Gerando textura '{args.pattern}'...")
    img = PATTERNS[args.pattern]()
    img.save(texture_path)
    print(f"  ✓ Salvo em: {texture_path}")

    # Conecta ao CoppeliaSim
    print(f"\nConectando ao CoppeliaSim...")
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        client = RemoteAPIClient(args.host, args.port)
        sim = client.getObject('sim')
        print("✓ Conectado.\n")
    except Exception as e:
        print(f"[ERRO] Não consegui conectar: {e}")
        return 1

    if not apply_texture_to_floor(sim, texture_path):
        print("\n⚠️  Se não funcionou, aplica manualmente:")
        print("   1. Clica no 'floor' na hierarquia")
        print("   2. Texture / geometry properties → Texture")
        print(f"   3. Carrega o arquivo: {texture_path}")
        return 1

    print("\n🎉 Textura aplicada com sucesso!\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
