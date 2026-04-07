"""
Descobre objetos disponíveis na cena CoppeliaSim

Execute com CoppeliaSim rodando (scene.ttt carregada):
  python discover_scene_objects.py

Mostrará:
  • Nome de cada objeto
  • Tipo de objeto
  • Posição (x, y, z)
"""

import sys
sys.path.insert(0, ".")

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    print("Erro: RemoteAPI não encontrada")
    print("Copie coppeliasim_zmqremoteapi_client.py para este diretório")
    sys.exit(1)


def main():
    print("\n" + "=" * 70)
    print("DESCOBRIR OBJETOS DO COPPELIA")
    print("=" * 70)
    
    # Conectar
    try:
        client = RemoteAPIClient("localhost", 23000)
        sim = client.getObject("sim")
        print("\n✅ Conectado ao CoppeliaSim")
    except Exception as e:
        print(f"\n❌ Erro: {e}")
        print("Certifique-se de que:")
        print("  1. CoppeliaSim está aberto")
        print("  2. Cenário está carregado")
        print("  3. Simulação está rodando (Play)")
        return 1
    
    print("\n📋 OBJETOS NA CENA:")
    print("-" * 70)
    
    try:
        # Listar todos os objetos
        all_objects = sim.getObjects(sim.handle_all, None)
        
        print(f"\nTotal: {len(all_objects)} objetos\n")
        
        for obj_handle in all_objects:
            try:
                # Nome
                name = sim.getObjectName(obj_handle)
                
                # Tipo
                obj_type = sim.getObjectType(obj_handle)
                type_str = {
                    0: "Shape",
                    1: "Joint",
                    2: "Graph",
                    3: "Camera",
                    4: "Light",
                    5: "Dummy",
                    6: "Proximity Sensor",
                    7: "Ray",
                    8: "Revolute Joint",
                    9: "Prismatic Joint",
                    10: "Spherical Joint",
                    11: "Path",
                    12: "Octree",
                    13: "PointCloud",
                    14: "Force Sensor",
                    15: "Accelerometer",
                    16: "Gyro",
                    17: "Filter",
                    18: "Composite",
                }.get(obj_type, f"Tipo {obj_type}")
                
                # Posição
                try:
                    pos = sim.getObjectPosition(obj_handle, -1)
                    pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                except:
                    pos_str = "N/A"
                
                # Imprime
                print(f"  • {name:<30} | {type_str:<20} | Pos: {pos_str}")
                
            except Exception as e:
                print(f"  (erro ao ler objeto: {e})")
    
    except Exception as e:
        print(f"Erro listando objetos: {e}")
        return 1
    
    print("\n" + "=" * 70)
    print("\n💡 USE O NOME EXATO para os testes.")
    print("\nExemplo:")
    print("  • Se o robô é 'PioneerP3DX' use:")
    print("    python test_planning_coppelia.py --robot /PioneerP3DX")
    print("\n  • Se o alvo é 'Target' use:")
    print("    python test_planning_coppelia.py --goal /Target")
    print("\n")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
