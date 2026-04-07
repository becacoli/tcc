"""
Exemplos de uso do PlanningContext

Demonstra:
1. Criação manual de contexto (sem CoppeliaSim)
2. Uso recorrente (múltiplos planejamentos com os mesmos dados)
3. Integração com algoritmos RRT
4. Loop recorrente para comparação justa
5. Isolamento de dados do planejador
"""

import sys
import os

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(CURRENT_DIR))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from planning_context import (
    PlanningContext, PlannerContextBuilder, Pose, Obstacle, WorldBounds,
    world_to_planner_coords, planner_to_world_coords
)
from algorithms.rrt_star import RRTStar
from utils.geometry import is_collision_free


def example_1_manual_context():
    """
    Exemplo 1: Criação manual de contexto sem CoppeliaSim.
    
    Útil para:
    - Testes offline
    - Reproduzir cenários
    - Integração com outras ferramentas
    """
    print("=" * 70)
    print("EXEMPLO 1: Criação Manual de Contexto")
    print("=" * 70)
    
    # Cria contexto usando builder
    builder = PlannerContextBuilder()
    
    # Definir robô e objetivo
    builder.set_robot_pose(x=1.0, y=1.0, theta=0.0)
    builder.set_goal_pose(x=3.0, y=3.0, theta=0.0)
    
    # Definir mundo
    builder.set_world_bounds(min_x=0.0, max_x=5.0, min_y=0.0, max_y=5.0)
    
    # Adicionar obstáculos (centro, largura, altura)
    builder.add_obstacle(cx=2.5, cy=1.5, width=1.0, height=0.5, name="wall_1")
    builder.add_obstacle(cx=1.5, cy=3.0, width=0.5, height=1.0, name="wall_2")
    
    # Tamanho do mapa para planejamento
    builder.set_map_size_pixels(width=100, height=100)
    
    # Metadata (informações extras)
    builder.set_metadata("scenario", "manual_test")
    builder.set_metadata("description", "Cenário simples com 2 obstáculos")
    
    context = builder.build()
    print(f"\n{context}")
    
    print(f"\nRobot pose (completo): {context.robot_pose_full()}")
    print(f"Goal pose (completo):  {context.goal_pose_full()}")
    print(f"\nObstáculos:")
    for obs in context.obstacles:
        print(f"  - {obs}")
    
    print(f"\nWorldbounds: {context.world_bounds}")
    print(f"Map size: {context.map_size_pixels}")
    
    return context


def example_2_coordinate_conversion():
    """
    Exemplo 2: Conversão entre coordenadas mundo <-> planejador.
    
    Mundo: coordenadas reais x,y
    Planejador: coordenadas discretizadas (0..map_width, 0..map_height)
    """
    print("\n" + "=" * 70)
    print("EXEMPLO 2: Conversão de Coordenadas")
    print("=" * 70)
    
    context = example_1_manual_context()
    
    # Ponto no mundo
    world_x, world_y = 2.5, 2.5
    
    # Converte para coordenadas do planejador
    planner_x, planner_y = world_to_planner_coords(world_x, world_y, context)
    print(f"\nMundo ({world_x}, {world_y}) → Planejador ({planner_x:.1f}, {planner_y:.1f})")
    
    # Converte de volta
    back_x, back_y = planner_to_world_coords(planner_x, planner_y, context)
    print(f"Planejador ({planner_x:.1f}, {planner_y:.1f}) → Mundo ({back_x:.2f}, {back_y:.2f})")
    
    # Robot e goal no espaço do planejador
    r_x, r_y = world_to_planner_coords(*context.robot_position(), context)
    g_x, g_y = world_to_planner_coords(*context.goal_position(), context)
    
    print(f"\nRobot no planejador: ({r_x:.1f}, {r_y:.1f})")
    print(f"Goal no planejador:  ({g_x:.1f}, {g_y:.1f})")


def example_3_planning_with_context():
    """
    Exemplo 3: Usar contexto para planejamento com RRT*.
    """
    print("\n" + "=" * 70)
    print("EXEMPLO 3: Planejamento com Contexto (RRT*)")
    print("=" * 70)
    
    context = example_1_manual_context()
    
    # Extrai dados necessários para RRT*
    start = world_to_planner_coords(*context.robot_position(), context)
    goal = world_to_planner_coords(*context.goal_position(), context)
    obstacles = context.get_obstacles_as_rects()
    map_size = context.map_size_pixels
    
    print(f"\nParâmetros do RRT*:")
    print(f"  Start: {start}")
    print(f"  Goal:  {goal}")
    print(f"  Obstáculos: {len(obstacles)}")
    print(f"  Map size: {map_size}")
    
    # Executa planejamento
    rrt_star = RRTStar(
        start=start,
        goal=goal,
        map_size=map_size,
        obstacles=obstacles,
        max_iter=1000,
        step_size=0.5,
        goal_sample_rate=0.1,
        neighbor_radius=5.0
    )
    
    path = rrt_star.planning()
    
    if path:
        print(f"\n✓ Caminho encontrado com {len(path)} waypoints")
        
        # Converte caminho para coordenadas do mundo
        world_path = []
        for px, py in path:
            wx, wy = planner_to_world_coords(px, py, context)
            world_path.append((wx, wy))
        
        print(f"\nPrimeiros waypoints (mundo):")
        for i, (x, y) in enumerate(world_path[:5]):
            print(f"  {i}: ({x:.2f}, {y:.2f})")
        if len(world_path) > 5:
            print(f"  ... ({len(world_path) - 5} mais)")
    else:
        print(f"\n✗ Caminho NÃO encontrado!")


def example_4_recurrent_planning():
    """
    Exemplo 4: Loop recorrente de planejamento.
    
    Simula múltiplas execuções de planejamento com os mesmos dados,
    permitindo comparar algoritmos fairmente.
    """
    print("\n" + "=" * 70)
    print("EXEMPLO 4: Loop Recorrente de Planejamento")
    print("=" * 70)
    print("Simulando múltiplos planejamentos com os MESMOS dados\n")
    
    # Cria contexto uma vez
    builder = PlannerContextBuilder()
    builder.set_robot_pose(1.0, 1.0, 0.0)
    builder.set_goal_pose(4.0, 4.0, 0.0)
    builder.set_world_bounds(0.0, 5.0, 0.0, 5.0)
    builder.add_obstacle(2.5, 2.5, 1.0, 1.0, "obstacle_1")
    builder.set_map_size_pixels(100, 100)
    context = builder.build()
    
    # Dados para planejamento
    start = world_to_planner_coords(*context.robot_position(), context)
    goal = world_to_planner_coords(*context.goal_position(), context)
    obstacles = context.get_obstacles_as_rects()
    map_size = context.map_size_pixels
    
    # Simula múltiplas execuções com diferentes configurações
    configs = [
        {"name": "RRT* (config 1)", "max_iter": 500, "step_size": 0.5},
        {"name": "RRT* (config 2)", "max_iter": 1000, "step_size": 0.7},
        {"name": "RRT* (config 3)", "max_iter": 1500, "step_size": 0.5},
    ]
    
    results = []
    
    for config in configs:
        print(f"Executando: {config['name']}")
        print(f"  Iterações: {config['max_iter']}, Step size: {config['step_size']}")
        
        rrt_star = RRTStar(
            start=start,
            goal=goal,
            map_size=map_size,
            obstacles=obstacles,
            max_iter=config['max_iter'],
            step_size=config['step_size'],
            goal_sample_rate=0.1,
            neighbor_radius=5.0
        )
        
        path = rrt_star.planning()
        
        if path:
            # Calcula tamanho do caminho
            path_length = 0.0
            for i in range(len(path) - 1):
                dx = path[i+1][0] - path[i][0]
                dy = path[i+1][1] - path[i][1]
                path_length += (dx**2 + dy**2)**0.5
            
            results.append({
                "config": config['name'],
                "success": True,
                "waypoints": len(path),
                "path_length": path_length,
                "iterations": rrt_star.iterations
            })
            print(f"  ✓ Sucesso! Waypoints: {len(path)}, Comprimento: {path_length:.2f}\n")
        else:
            results.append({
                "config": config['name'],
                "success": False,
                "iterations": rrt_star.iterations
            })
            print(f"  ✗ Falhou!\n")
    
    # Resumo comparativo
    print("\n" + "=" * 70)
    print("COMPARAÇÃO DE RESULTADOS")
    print("=" * 70)
    
    for result in results:
        if result['success']:
            print(f"{result['config']}: "
                  f"Waypoints={result['waypoints']}, "
                  f"Path_length={result['path_length']:.2f}, "
                  f"Iterations={result['iterations']}")
        else:
            print(f"{result['config']}: FALHOU (Iterations={result['iterations']})")


def example_5_data_isolation():
    """
    Exemplo 5: Demonstração de isolamento de dados.
    
    Mostra como o PlanningContext isola o planejador de:
    - Controle do robô
    - Estimação de estados
    - Mapeamento
    - Outras partes da pilha de navegação
    """
    print("\n" + "=" * 70)
    print("EXEMPLO 5: Isolamento de Dados")
    print("=" * 70)
    print("""
O PlanningContext DESACOPLA o planejador de outras componentes:

┌─────────────────────────────────────────────────────────────┐
│ CoppeliaSim / Sensor / Estimador                           │
│ (dados brutos: posição, obstáculos, mapa)                 │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ↓ (converter)
┌────────────────────────────────────────────────────────────┐
│ PlanningContext (interface limpa e independente)          │
│  - robot_pose: (xr, yr, θr)                              │
│  - goal_pose: (xg, yg, θg)                               │
│  - obstacles: [lista com centros e geometria]            │
│  - world_bounds: limites do mundo                        │
└────────────────────┬──────────────────────────────────────┘
                     │
                     ↓
┌────────────────────────────────────────────────────────────┐
│ Algoritmos de Planejamento (RRT, RRT*, etc.)             │
│ (sem conhecer sobre controladores, mapeamento, etc.)      │
└────────────────────────────────────────────────────────────┘

Benefícios:
  ✓ Avaliar algoritmos fairmente (sem viés)
  ✓ Comparar diferentes configurações
  ✓ Reutilizar dados para múltiplas execuções
  ✓ Isolar problemas (se falha, é no planejador)
  ✓ Integrar dados de diferentes fontes
        """)
    
    # Exemplo concreto
    print("Exemplo concreto:")
    print("-" * 70)
    
    context = example_1_manual_context()
    
    print(f"\n✓ Dados prontos para planejamento (independentes):")
    print(f"  - Robot ready: {context.robot_pose_full()}")
    print(f"  - Goal ready:  {context.goal_pose_full()}")
    print(f"  - Obstacles ready: {len(context.obstacles)} obstáculos")
    print(f"  - World ready: {context.world_bounds}")
    
    print(f"\n✓ Algoritmos podem ser implementados sem conhecer:")
    print(f"  - Como os dados foram coletados")
    print(f"  - Se há controlador ou estimador")
    print(f"  - Como o mapeamento funciona")
    print(f"  - Dinâmica do robô")


if __name__ == "__main__":
    print("\n" + "█" * 70)
    print("EXEMPLOS DE USO: PlanningContext - Isolamento do Planejador")
    print("█" * 70)
    
    example_1_manual_context()
    example_2_coordinate_conversion()
    example_3_planning_with_context()
    example_4_recurrent_planning()
    example_5_data_isolation()
    
    print("\n" + "█" * 70)
    print("FIM DOS EXEMPLOS")
    print("█" * 70 + "\n")
