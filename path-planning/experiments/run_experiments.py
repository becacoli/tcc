import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from algorithms.rrt import RRT
from algorithms.rrt_star import RRTStar
from algorithms.rrt_connect import RRTConnect
from algorithms.a_star import AStar
from experiments.metrics import ExperimentRunner
from utils.plotting import plot_path
import matplotlib.pyplot as plt
import numpy as np


def create_test_scenarios():
    """Define cenários de teste"""
    scenarios = {
        'simples': {
            'start': (10, 10),
            'goal': (90, 90),
            'map_size': (100, 100),
            'obstacles': [
                (30, 30, 50, 50),
                (60, 60, 80, 80)
            ]
        },
        'moderado': {
            'start': (10, 10),
            'goal': (90, 90),
            'map_size': (100, 100),
            'obstacles': [
                (20, 20, 40, 40),
                (50, 10, 60, 70),
                (30, 60, 70, 75),
                (70, 30, 90, 50)
            ]
        },
        'complexo': {
            'start': (10, 10),
            'goal': (90, 90),
            'map_size': (100, 100),
            'obstacles': [
                (15, 15, 25, 85),
                (35, 15, 45, 75),
                (55, 25, 65, 85),
                (75, 15, 85, 75),
                (25, 40, 90, 50)
            ]
        },
        'corredor': {
            'start': (10, 50),
            'goal': (90, 50),
            'map_size': (100, 100),
            'obstacles': [
                (0, 0, 100, 30),
                (0, 70, 100, 100),
                (40, 30, 45, 70),
                (55, 30, 60, 70)
            ]
        }
    }
    return scenarios


def run_comparative_experiments():
    print("="*80)
    print("EXPERIMENTOS COMPARATIVOS - ALGORITMOS DE PLANEJAMENTO")
    print("="*80)
    
    # Definir cenários
    scenarios = create_test_scenarios()
    
    # Algoritmos a testar
    algorithms = [
        (RRT, "RRT"),
        (RRTStar, "RRT*"),
        # (RRTConnect, "RRT-Connect"),  
    ]
    
    # Parâmetros comuns
    common_params = {
        'max_iter': 500,
        'step_size': 5,
        'goal_sample_rate': 0.1,
    }
    rrt_star_params = {
        'neighbor_radius': 15,
    }
    visualize_paths = True
    
    # Runner para experimentos
    runner = ExperimentRunner()
    
    # Resultados por cenário
    all_results = {}
    
    # Executar experimentos para cada cenário
    for scenario_name, scenario_config in scenarios.items():
        print(f"\n{'#'*80}")
        print(f"CENÁRIO: {scenario_name.upper()}")
        print(f"{'#'*80}")
        
        # Combinar parâmetros
        params = {**common_params, **scenario_config}
        params.pop("start", None)
        params.pop("goal", None)

        # Comparar algoritmos (3 execuções por algoritmo)
        results = {}
        for algo_class, algo_name in algorithms:
            algo_params = dict(params)
            if algo_class is RRTStar:
                algo_params.update(rrt_star_params)

            results[algo_name] = runner.compare_algorithms(
                algorithms=[(algo_class, algo_name)],
                start=scenario_config['start'],
                goal=scenario_config['goal'],
                num_runs=3,
                **algo_params
            )[algo_name]

            if visualize_paths:
                print(f"  → Gerando visualização para {algo_name} - {scenario_name}")
                algo_instance = algo_class(
                    scenario_config['start'],
                    scenario_config['goal'],
                    **algo_params,
                )
                path = algo_instance.planning()
                if path:
                    plot_path(
                        path,
                        algo_instance.get_all_nodes(),
                        scenario_config['start'],
                        scenario_config['goal'],
                        scenario_config['obstacles'],
                        map_size=scenario_config['map_size'],
                        title=f"{algo_name} - {scenario_name}",
                        block=False,  # Não-bloqueante - mostra todas as janelas
                    )
                else:
                    print(f"  ✗ Sem caminho para visualizar")
        
        all_results[scenario_name] = results
    
    # Salvar resultados
    runner.save_results('../results/experiment_results.json')
    
    # Gerar visualizações
    print("\n" + "="*80)
    print("GERANDO VISUALIZAÇÕES")
    print("="*80)
    generate_comparison_plots(all_results, scenarios)
    
    # Se gerou plot_path, manter janelas abertas
    if visualize_paths:
        print("\n" + "="*80)
        print("Janelas de visualização abertas. Pressione ENTER para fechar todas...")
        input()
        plt.close('all')
    
    print("\n✓ Experimentos concluídos!")


def generate_comparison_plots(results, scenarios):
    """Gera gráficos comparativos"""
    
    # Criar diretório para resultados
    os.makedirs('../results', exist_ok=True)
    
    scenario_names = list(results.keys())
    algorithm_names = list(results[scenario_names[0]].keys())
    
    # 1. Tempo de planejamento por cenário
    fig, ax = plt.subplots(figsize=(10, 6))
    
    x = np.arange(len(scenario_names))
    width = 0.8 / len(algorithm_names)
    
    for i, algo_name in enumerate(algorithm_names):
        times = [results[scenario][algo_name]['avg_time'] 
                 if results[scenario][algo_name]['avg_time'] is not None else 0
                 for scenario in scenario_names]
        
        ax.bar(x + i * width, times, width, label=algo_name)
    
    ax.set_xlabel('Cenário')
    ax.set_ylabel('Tempo médio (s)')
    ax.set_title('Tempo de Planejamento por Cenário')
    ax.set_xticks(x + width * (len(algorithm_names) - 1) / 2)
    ax.set_xticklabels(scenario_names)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('../results/planning_time_comparison.png', dpi=300)
    print("✓ Gráfico salvo: planning_time_comparison.png")
    plt.close()
    
    # 2. Comprimento do caminho por cenário
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for i, algo_name in enumerate(algorithm_names):
        lengths = [results[scenario][algo_name]['avg_length'] 
                   if results[scenario][algo_name]['avg_length'] is not None else 0
                   for scenario in scenario_names]
        
        ax.bar(x + i * width, lengths, width, label=algo_name)
    
    ax.set_xlabel('Cenário')
    ax.set_ylabel('Comprimento médio do caminho')
    ax.set_title('Comprimento do Caminho por Cenário')
    ax.set_xticks(x + width * (len(algorithm_names) - 1) / 2)
    ax.set_xticklabels(scenario_names)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('../results/path_length_comparison.png', dpi=300)
    print("✓ Gráfico salvo: path_length_comparison.png")
    plt.close()
    
    # 3. Taxa de sucesso
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for i, algo_name in enumerate(algorithm_names):
        success_rates = [results[scenario][algo_name]['success_rate'] 
                         for scenario in scenario_names]
        
        ax.bar(x + i * width, success_rates, width, label=algo_name)
    
    ax.set_xlabel('Cenário')
    ax.set_ylabel('Taxa de Sucesso (%)')
    ax.set_title('Taxa de Sucesso por Cenário')
    ax.set_xticks(x + width * (len(algorithm_names) - 1) / 2)
    ax.set_xticklabels(scenario_names)
    ax.set_ylim([0, 105])
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('../results/success_rate_comparison.png', dpi=300)
    print("✓ Gráfico salvo: success_rate_comparison.png")
    plt.close()
    
    # 4. Tabela resumo
    print("\n" + "="*80)
    print("TABELA RESUMO - TODOS OS CENÁRIOS")
    print("="*80)
    
    for algo_name in algorithm_names:
        print(f"\n{algo_name}:")
        print(f"{'Cenário':<15} {'Sucesso%':<12} {'Tempo(s)':<12} {'Comprimento':<15}")
        print("-" * 60)
        
        for scenario in scenario_names:
            stats = results[scenario][algo_name]
            avg_time = stats['avg_time']
            avg_length = stats['avg_length']

            time_str = "  n/a  " if avg_time is None else f"{avg_time:>6.4f}"
            length_str = "  n/a  " if avg_length is None else f"{avg_length:>8.2f}"

            print(f"{scenario:<15} "
                  f"{stats['success_rate']:>6.1f}%      "
                      f"{time_str}      "
                      f"{length_str}")


if __name__ == "__main__":
    run_comparative_experiments()
