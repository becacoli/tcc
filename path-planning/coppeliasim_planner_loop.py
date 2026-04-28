import argparse
import json
import time
import sys
import os
from pathlib import Path

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = CURRENT_DIR
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from planning_context import (
    PlanningContext, world_to_planner_coords, planner_to_world_coords,
    create_context_from_coppelia
)
from algorithms.rrt_star import RRTStar
from algorithms.rrt import RRT
from algorithms.rrt_connect import RRTConnect
from utils.geometry import distance


class RecurrentPlannerLoop:
    """
    Loop recorrente de planejamento.
    
    A cada iteração:
    1. Extrai dados atuais do CoppeliaSim
    2. Expõe claramente no contexto
    3. Executa planejamento
    4. Coleta métricas
    """
    
    def __init__(self, sim, algorithm_name="rrt_star", max_iterations=1000,
                 step_size=0.5, goal_sample_rate=0.1, neighbor_radius=5.0,
                 world_bounds=None, map_size_pixels=(100, 100),
                 wall_inflate=0.0, robot_path="/PioneerP3DX",
                 goal_object="/GoalConfiguration", cuboid_only=True,
                 obstacle_primitives=None):
        """
        Args:
            sim: Simulador CoppeliaSim
            algorithm_name: "rrt", "rrt_star", ou "rrt_connect"
            max_iterations: Máximo de iterações do algoritmo
            step_size: Tamanho do passo do RRT
            goal_sample_rate: Taxa de amostragem do objetivo
            neighbor_radius: Raio de vizinhança (para RRT*)
            map_size_pixels: Tamanho do mapa de planejamento
            wall_inflate: Inflação dos obstáculos (margem de segurança)
        """
        self.sim = sim
        self.algorithm_name = algorithm_name
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.neighbor_radius = neighbor_radius
        self.world_bounds = world_bounds
        self.map_size_pixels = map_size_pixels
        self.wall_inflate = wall_inflate
        self.robot_path = robot_path
        self.goal_object = goal_object
        self.cuboid_only = cuboid_only
        self.obstacle_primitives = obstacle_primitives or ("cuboid" if cuboid_only else "all")
        
        # Métricas
        self.metrics = []
    
    def run_loop(self, num_iterations=5, delay_between_plans=0.0):
        """
        Executa loop recorrente de planejamento.
        
        Args:
            num_iterations: Quantas vezes extrair dados e planejar
            delay_between_plans: Espera entre iterações (segundos)
        """
        print("\n" + "=" * 80)
        print("LOOP RECORRENTE DE PLANEJAMENTO - PlanningContext")
        print("=" * 80)
        
        for iteration in range(num_iterations):
            print(f"\n{'─' * 80}")
            print(f"ITERAÇÃO {iteration + 1} / {num_iterations}")
            print(f"{'─' * 80}")
            
            try:
                # =========== PASSO 1: Extrai contexto ===========
                context = self._extract_planning_context()
                
                # =========== PASSO 2: Executa planejamento ===========
                path, planning_metrics = self._plan_with_context(context)
                
                # =========== PASSO 3: Coleta métricas ===========
                self._collect_metrics(iteration, context, path, planning_metrics)
                
                # =========== PASSO 4: Exibe resultado ===========
                self._display_results(iteration, context, path, planning_metrics)
                
            except Exception as e:
                print(f"✗ Erro na iteração {iteration + 1}: {e}")
                import traceback
                traceback.print_exc()
            
            if iteration < num_iterations - 1 and delay_between_plans > 0:
                print(f"\nEsperando {delay_between_plans:.1f}s antes da próxima iteração...")
                time.sleep(delay_between_plans)
        
        # =========== RESUME FINAL ===========
        self._display_summary()
    
    def _extract_planning_context(self) -> PlanningContext:
        """Extrai contexto atual do CoppeliaSim."""
        print("\n📍 EXTRAINDO CONTEXTO DO COPPELIA...")
        
        context = create_context_from_coppelia(
            sim=self.sim,
            robot_path=self.robot_path,
            goal_object=self.goal_object,
            world_bounds=self.world_bounds,
            map_size_pixels=self.map_size_pixels,
            wall_inflate=self.wall_inflate,
            cuboid_only=self.cuboid_only,
            obstacle_primitives=self.obstacle_primitives,
        )
        
        return context

    def _build_snapshot(self, context: PlanningContext) -> dict:
        """Snapshot explícito para avaliação sem viés da pilha de navegação."""
        return context.to_snapshot()
    
    def _plan_with_context(self, context: PlanningContext):
        """Executa planejamento com o contexto extraído."""
        print("\n🚀 EXECUTANDO PLANEJAMENTO...")
        print(f"   Algoritmo: {self.algorithm_name}")
        
        # Converte para coordenadas do planejador
        start = world_to_planner_coords(*context.robot_position(), context)
        goal = world_to_planner_coords(*context.goal_position(), context)
        obstacles = context.get_obstacles_as_rects()
        map_size = context.map_size_pixels
        
        print(f"   Start (planner): {start}")
        print(f"   Goal (planner):  {goal}")
        print(f"   Obstacles: {len(obstacles)}")
        
        # Seleciona algoritmo
        algorithm_class = {
            "rrt": RRT,
            "rrt_star": RRTStar,
            "rrt_connect": RRTConnect,
        }.get(self.algorithm_name, RRTStar)
        
        # Executa
        t0 = time.time()

        planner_kwargs = {
            "step_size": self.step_size,
            "goal_sample_rate": self.goal_sample_rate,
            "obstacles": obstacles,
        }

        if self.algorithm_name == "rrt_connect":
            planner_kwargs["max_samples"] = self.max_iterations
        else:
            planner_kwargs["max_iter"] = self.max_iterations

        if self.algorithm_name == "rrt_star":
            planner_kwargs["neighbor_radius"] = self.neighbor_radius

        planner = algorithm_class(
            start=start,
            goal=goal,
            map_size=map_size,
            **planner_kwargs,
        )
        
        path = planner.planning()
        t1 = time.time()
        
        planning_metrics = {
            "time_ms": (t1 - t0) * 1000,
            "iterations": planner.iterations,
            "nodes": len(planner.get_all_nodes()),
            "success": path is not None,
            "path_length": self._calculate_path_length(path) if path else None,
            "waypoints": len(path) if path else 0,
        }
        
        return path, planning_metrics
    
    def _calculate_path_length(self, path):
        """Calcula comprimento total do caminho."""
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += distance(path[i], path[i+1])
        
        return length
    
    def _collect_metrics(self, iteration, context, path, planning_metrics):
        """Coleta métricas para análise posterior."""
        snapshot = self._build_snapshot(context)
        robot_goal_dist = distance(
            context.robot_position(), 
            context.goal_position()
        )
        
        metric_entry = {
            "iteration": iteration + 1,
            "robot_pose": snapshot["robot_pose"],
            "goal_pose": snapshot["goal_pose"],
            "obstacles": snapshot["obstacles"],
            "num_obstacles": len(context.obstacles),
            "robot_goal_distance": robot_goal_dist,
            **planning_metrics,
        }
        
        self.metrics.append(metric_entry)
    
    def _display_results(self, iteration, context, path, planning_metrics):
        """Exibe resultados da iteração."""
        print("\n📊 RESULTADOS:")
        snapshot = self._build_snapshot(context)
        
        # Contexto
        print(f"\n   Contexto Extraído:")
        rp = snapshot["robot_pose"]
        gp = snapshot["goal_pose"]
        print(f"     Robot (xr, yr, θr): ({rp['xr']:.3f}, {rp['yr']:.3f}, {rp['theta_r']:.3f})")
        print(f"     Goal  (xg, yg, θg): ({gp['xg']:.3f}, {gp['yg']:.3f}, {gp['theta_g']:.3f})")
        print(f"     Obstáculos: {len(context.obstacles)}")
        for obs in snapshot["obstacles"]:
            print(
                "       - "
                f"{obs['name']}: centro=({obs['cx']:.3f}, {obs['cy']:.3f}), "
                f"geom=({obs['width']:.3f} x {obs['height']:.3f})"
            )
        
        # Planejamento
        if planning_metrics['success']:
            print(f"\n   ✓ PLANEJAMENTO SUCESSO!")
            print(f"     Tempo: {planning_metrics['time_ms']:.2f}ms")
            print(f"     Iterações: {planning_metrics['iterations']}")
            print(f"     Nós na árvore: {planning_metrics['nodes']}")
            print(f"     Waypoints no caminho: {planning_metrics['waypoints']}")
            print(f"     Comprimento do caminho: {planning_metrics['path_length']:.3f}")
            
            # Mostra alguns waypoints em coordenadas mundo
            if len(path) > 0:
                print(f"\n     Primeiros waypoints (mundo):")
                for i, (px, py) in enumerate(path[:3]):
                    wx, wy = planner_to_world_coords(px, py, context)
                    print(f"       {i}: ({wx:.3f}, {wy:.3f})")
                if len(path) > 3:
                    print(f"       ... ({len(path) - 3} mais)")
        else:
            print(f"\n   ✗ PLANEJAMENTO FALHOU!")
            print(f"     Tempo: {planning_metrics['time_ms']:.2f}ms")
            print(f"     Iterações até limite: {planning_metrics['iterations']}")
    
    def _display_summary(self):
        """Exibe resumo de todas as iterações."""
        print("\n" + "=" * 80)
        print("RESUMO FINAL - COMPARAÇÃO DE ITERAÇÕES")
        print("=" * 80)
        
        if not self.metrics:
            print("Sem métricas coletadas.")
            return
        
        successful = [m for m in self.metrics if m['success']]
        failed = [m for m in self.metrics if not m['success']]
        
        print(f"\nTotal de iterações: {len(self.metrics)}")
        print(f"Planejamentos bem-sucedidos: {len(successful)} ({100*len(successful)/len(self.metrics):.0f}%)")
        print(f"Planejamentos falhados: {len(failed)} ({100*len(failed)/len(self.metrics):.0f}%)")
        
        if successful:
            times = [m['time_ms'] for m in successful]
            path_lengths = [m['path_length'] for m in successful if m['path_length']]
            waypoints = [m['waypoints'] for m in successful]
            
            print(f"\nTempo de planejamento (planejamentos bem-sucedidos):")
            print(f"  Média: {sum(times)/len(times):.2f}ms")
            print(f"  Min: {min(times):.2f}ms")
            print(f"  Max: {max(times):.2f}ms")
            
            if path_lengths:
                print(f"\nComprimento do caminho:")
                print(f"  Média: {sum(path_lengths)/len(path_lengths):.3f}")
                print(f"  Min: {min(path_lengths):.3f}")
                print(f"  Max: {max(path_lengths):.3f}")
            
            print(f"\nWaypoints por caminho (bem-sucedidos):")
            print(f"  Média: {sum(waypoints)/len(waypoints):.1f}")
            print(f"  Min: {min(waypoints)}")
            print(f"  Max: {max(waypoints)}")
        
        print("\n" + "=" * 80)
    
    def save_metrics(self, output_file="planning_metrics.json"):
        """Salva métricas em arquivo JSON."""
        with open(output_file, 'w') as f:
            json.dump(self.metrics, f, indent=2)
        print(f"\n✓ Métricas salvas em: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Loop recorrente de planejamento com PlanningContext"
    )
    parser.add_argument("--host", default="localhost",
                       help="Host do CoppeliaSim")
    parser.add_argument("--port", type=int, default=23000,
                       help="Port do CoppeliaSim")
    parser.add_argument("--algo", choices=["rrt", "rrt_star", "rrt_connect"],
                       default="rrt_star", help="Algoritmo de planejamento")
    parser.add_argument("--iterations", type=int, default=5,
                       help="Número de iterações do loop")
    parser.add_argument("--max-iter", type=int, default=1000,
                       help="Máximo de iterações do algoritmo")
    parser.add_argument("--step-size", type=float, default=0.5,
                       help="Tamanho do passo")
    parser.add_argument("--robot-path", default="/PioneerP3DX",
                       help="Caminho do objeto robô")
    parser.add_argument("--goal-object", default="/GoalConfiguration",
                       help="Caminho do objeto objetivo")
    parser.add_argument("--world-bounds", nargs=4, type=float, default=None,
                       help="Limites do mundo (min_x max_x min_y max_y)")
    parser.add_argument("--map-width", type=int, default=100,
                       help="Largura do mapa de planejamento")
    parser.add_argument("--map-height", type=int, default=100,
                       help="Altura do mapa de planejamento")
    parser.add_argument("--wall-inflate", type=float, default=0.0,
                       help="Inflação dos obstáculos")
    parser.add_argument("--cuboid-only", action="store_true", default=True,
                       help="Usa apenas obstáculos primitive cuboid")
    parser.add_argument(
        "--obstacle-primitives",
        choices=["cuboid", "cuboid_spheroid", "all"],
        default="cuboid",
        help="Filtro de primitivas de obstáculos extraídas do CoppeliaSim",
    )
    parser.add_argument("--save-metrics", default="planning_metrics.json",
                       help="Arquivo para salvar métricas (JSON)")
    parser.add_argument("--delay", type=float, default=1.0,
                       help="Delay entre iterações (segundos)")
    
    args = parser.parse_args()
    
    print(f"\n{'█' * 80}")
    print("PLANNER CONTEXT - LOOP RECORRENTE")
    print(f"{'█' * 80}")
    print(f"\nConectando ao CoppeliaSim em {args.host}:{args.port}...")
    
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        client = RemoteAPIClient(args.host, args.port)
        sim = client.getObject("sim")
    except Exception as e:
        print(f"✗ Erro ao conectar: {e}")
        return
    
    # Cria loop de planejamento
    loop = RecurrentPlannerLoop(
        sim=sim,
        algorithm_name=args.algo,
        max_iterations=args.max_iter,
        step_size=args.step_size,
        world_bounds=tuple(args.world_bounds) if args.world_bounds else None,
        map_size_pixels=(args.map_width, args.map_height),
        wall_inflate=args.wall_inflate,
        robot_path=args.robot_path,
        goal_object=args.goal_object,
        cuboid_only=args.cuboid_only,
        obstacle_primitives=args.obstacle_primitives,
    )
    
    # Executa loop
    try:
        loop.run_loop(
            num_iterations=args.iterations,
            delay_between_plans=args.delay
        )
    finally:
        # Salva métricas
        if args.save_metrics:
            loop.save_metrics(args.save_metrics)


if __name__ == "__main__":
    main()
