import numpy as np
import time
from typing import List, Tuple, Dict


class PathPlanningMetrics:
    """Calcula métricas para avaliação de algoritmos de planejamento"""
    
    @staticmethod
    def path_length(path: List[Tuple[float, float]]) -> float:
        """
        Calcula o comprimento total do caminho
        
        Args:
            path: Lista de pontos (x, y)
            
        Returns:
            Comprimento do caminho em unidades de distância
        """
        if not path or len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            length += np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        return length
    
    @staticmethod
    def path_smoothness(path: List[Tuple[float, float]]) -> float:
        """
        Calcula a suavidade do caminho (soma das mudanças de ângulo)
        Quanto menor, mais suave é o caminho
        
        Args:
            path: Lista de pontos (x, y)
            
        Returns:
            Soma dos ângulos de mudança em radianos
        """
        if not path or len(path) < 3:
            return 0.0
        
        total_angle_change = 0.0
        
        for i in range(1, len(path) - 1):
            # Vetores entre pontos consecutivos
            v1 = np.array(path[i]) - np.array(path[i-1])
            v2 = np.array(path[i+1]) - np.array(path[i])
            
            # Normalizar vetores
            v1_norm = v1 / (np.linalg.norm(v1) + 1e-10)
            v2_norm = v2 / (np.linalg.norm(v2) + 1e-10)
            
            # Calcular ângulo entre vetores
            dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
            angle = np.arccos(dot_product)
            
            total_angle_change += angle
        
        return total_angle_change
    
    @staticmethod
    def number_of_nodes(tree: List) -> int:
        """
        Conta o número de nós gerados pelo algoritmo
        
        Args:
            tree: Árvore gerada pelo algoritmo
            
        Returns:
            Número de nós
        """
        if tree is None:
            return 0
        return len(tree)
    
    @staticmethod
    def euclidean_distance(point1: Tuple[float, float], 
                          point2: Tuple[float, float]) -> float:
        """
        Calcula a distância euclidiana entre dois pontos
        
        Args:
            point1: Primeiro ponto (x, y)
            point2: Segundo ponto (x, y)
            
        Returns:
            Distância euclidiana
        """
        return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
    @staticmethod
    def path_clearance(path: List[Tuple[float, float]], 
                       obstacles: List[Tuple[float, float, float, float]]) -> float:
        """
        Calcula a distância mínima do caminho aos obstáculos
        
        Args:
            path: Lista de pontos (x, y)
            obstacles: Lista de obstáculos (x_min, y_min, x_max, y_max)
            
        Returns:
            Distância mínima aos obstáculos
        """
        if not path or not obstacles:
            return float('inf')
        
        min_clearance = float('inf')
        
        for point in path:
            x, y = point
            for obs in obstacles:
                x_min, y_min, x_max, y_max = obs
                
                # Encontrar ponto mais próximo no obstáculo
                closest_x = np.clip(x, x_min, x_max)
                closest_y = np.clip(y, y_min, y_max)
                
                # Calcular distância
                distance = np.sqrt((x - closest_x)**2 + (y - closest_y)**2)
                min_clearance = min(min_clearance, distance)
        
        return min_clearance


class ExperimentRunner:
    """Executa experimentos e coleta métricas"""
    
    def __init__(self):
        self.results = []
    
    def run_algorithm(self, algorithm, algorithm_name: str, 
                     start: Tuple, goal: Tuple, **kwargs) -> Dict:
        """
        Executa um algoritmo e coleta métricas
        
        Args:
            algorithm: Classe do algoritmo a ser testado
            algorithm_name: Nome do algoritmo
            start: Ponto inicial
            goal: Ponto objetivo
            **kwargs: Argumentos adicionais para o algoritmo
            
        Returns:
            Dicionário com resultados e métricas
        """
        print(f"\n{'='*60}")
        print(f"Executando: {algorithm_name}")
        print(f"{'='*60}")
        
        # Inicializar algoritmo
        algo_instance = algorithm(start, goal, **kwargs)
        
        # Medir tempo de planejamento
        start_time = time.time()
        path = algo_instance.planning()
        planning_time = time.time() - start_time
        
        # Coletar métricas
        metrics = PathPlanningMetrics()
        
        result = {
            'algorithm': algorithm_name,
            'success': path is not None and len(path) > 0,
            'planning_time': planning_time,
            'path': path,
            'tree': getattr(algo_instance, 'tree', None)
        }
        
        if result['success']:
            result['path_length'] = metrics.path_length(path)
            result['path_smoothness'] = metrics.path_smoothness(path)
            result['num_nodes'] = metrics.number_of_nodes(result['tree'])
            result['clearance'] = metrics.path_clearance(
                path, 
                kwargs.get('obstacles', [])
            )
            
            print(f"✓ Sucesso!")
            print(f"  - Tempo de planejamento: {planning_time:.4f}s")
            print(f"  - Comprimento do caminho: {result['path_length']:.2f}")
            print(f"  - Suavidade: {result['path_smoothness']:.4f}")
            print(f"  - Número de nós: {result['num_nodes']}")
            print(f"  - Folga mínima: {result['clearance']:.2f}")
        else:
            print(f"✗ Falha - caminho não encontrado")
        
        self.results.append(result)
        return result
    
    def compare_algorithms(self, algorithms: List[Tuple], 
                          start: Tuple, goal: Tuple, 
                          num_runs: int = 5, **kwargs) -> Dict:
        """
        Compara múltiplos algoritmos executando várias vezes
        
        Args:
            algorithms: Lista de tuplas (classe_algoritmo, nome)
            start: Ponto inicial
            goal: Ponto objetivo
            num_runs: Número de execuções por algoritmo
            **kwargs: Argumentos para os algoritmos
            
        Returns:
            Dicionário com estatísticas comparativas
        """
        comparison = {}
        
        for algo_class, algo_name in algorithms:
            print(f"\n{'#'*60}")
            print(f"Testando {algo_name} ({num_runs} execuções)")
            print(f"{'#'*60}")
            
            times = []
            lengths = []
            smoothness_values = []
            nodes = []
            successes = 0
            
            for run in range(num_runs):
                print(f"\nRun {run + 1}/{num_runs}")
                result = self.run_algorithm(algo_class, algo_name, start, goal, **kwargs)
                
                if result['success']:
                    successes += 1
                    times.append(result['planning_time'])
                    lengths.append(result['path_length'])
                    smoothness_values.append(result['path_smoothness'])
                    nodes.append(result['num_nodes'])
            
            # Calcular estatísticas
            comparison[algo_name] = {
                'success_rate': successes / num_runs * 100,
                'avg_time': np.mean(times) if times else None,
                'std_time': np.std(times) if times else None,
                'avg_length': np.mean(lengths) if lengths else None,
                'std_length': np.std(lengths) if lengths else None,
                'avg_smoothness': np.mean(smoothness_values) if smoothness_values else None,
                'avg_nodes': np.mean(nodes) if nodes else None,
            }
        
        # Imprimir resumo comparativo
        print(f"\n{'='*80}")
        print("RESUMO COMPARATIVO")
        print(f"{'='*80}")
        print(f"{'Algoritmo':<20} {'Taxa Sucesso':<15} {'Tempo (s)':<15} {'Comprimento':<15}")
        print(f"{'-'*80}")
        
        for algo_name, stats in comparison.items():
            avg_time = stats['avg_time']
            std_time = stats['std_time']
            avg_length = stats['avg_length']
            std_length = stats['std_length']

            if avg_time is None:
                time_str = "   n/a       "
            else:
                time_str = f"{avg_time:>8.4f}±{(std_time or 0):>.4f}"

            if avg_length is None:
                length_str = "   n/a     "
            else:
                length_str = f"{avg_length:>8.2f}±{(std_length or 0):>.2f}"

            print(f"{algo_name:<20} "
                  f"{stats['success_rate']:>6.1f}%      "
                  f"{time_str}  "
                  f"{length_str}")
        
        return comparison
    
    def save_results(self, filename: str):
        """Salva resultados em arquivo"""
        import json
        import os
        
        # Converter para formato serializável
        serializable_results = []
        for result in self.results:
            r = result.copy()
            r['path'] = list(r['path']) if r['path'] else None
            r['tree'] = None  # Árvore é muito grande para serializar
            serializable_results.append(r)
        
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w') as f:
            json.dump(serializable_results, f, indent=2)
        
        print(f"\n✓ Resultados salvos em: {filename}")


# Exemplo de uso
if __name__ == "__main__":
    # Testar métricas básicas
    metrics = PathPlanningMetrics()
    
    # Caminho de exemplo
    path = [(0, 0), (1, 1), (2, 1), (3, 2)]
    
    print(f"Comprimento: {metrics.path_length(path):.2f}")
    print(f"Suavidade: {metrics.path_smoothness(path):.4f}")
