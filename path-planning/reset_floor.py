"""Reseta o chão do CoppeliaSim para cor branca (sem textura).

Use ANTES de aplicar uma nova textura, para garantir que ela apareça.

Uso:
  python reset_floor.py
"""

import sys


def main():
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        client = RemoteAPIClient('localhost', 23000)
        sim = client.getObject('sim')
        print("✓ Conectado ao CoppeliaSim\n")
    except Exception as e:
        print(f"[ERRO] {e}")
        return 1

    # Procura o chão
    floor = None
    for name in ['/floor', '/Floor', '/ResizableFloor_5_25', '/Plane']:
        try:
            floor = sim.getObject(name)
            print(f"✓ Chão encontrado em: {name}")
            break
        except Exception:
            continue

    if floor is None:
        print("[ERRO] Nenhum chão encontrado.")
        return 1

    # 1) Remove a textura
    try:
        sim.setShapeTexture(floor, -1, sim.texturemap_plane, 0, [0.0, 0.0])
        print("  ✓ Textura removida")
    except Exception as e:
        print(f"  [aviso] Falha ao remover textura: {e}")

    # 2) Aplica cor BRANCA (importante: textura aparece sobre cor neutra)
    try:
        sim.setShapeColor(floor, '', sim.colorcomponent_ambient_diffuse,
                          [1.0, 1.0, 1.0])
        sim.setShapeColor(floor, '', sim.colorcomponent_specular,
                          [0.2, 0.2, 0.2])
        sim.setShapeColor(floor, '', sim.colorcomponent_emission,
                          [0.0, 0.0, 0.0])
        print("  ✓ Cor resetada para BRANCO")
    except Exception as e:
        print(f"  [erro] Falha ao resetar cor: {e}")
        return 1

    print("\n🎉 Chão resetado!")
    print("\nAgora você pode:")
    print("  1) Aplicar uma textura: python apply_floor_texture.py --pattern wood")
    print("  2) Ou aplicar um tema:  python beautify_scene.py --theme wood_full")
    return 0


if __name__ == "__main__":
    sys.exit(main())
