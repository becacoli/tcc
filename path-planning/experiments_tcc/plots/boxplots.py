"""Gera os 5 boxplots de comparação final dos algoritmos (TCC).

Lê o CSV de comparison_<scene>.csv gerado por compare_best.py e produz
5 figuras separadas, uma para cada métrica:

  1. planning_time_s   - tempo de planejamento
  2. execution_time_s  - tempo de execução do controle
  3. final_error_m     - erro final em relação ao goal
  4. processed_waypoints - tamanho do caminho (proxy para path length)
  5. min_clearance_m   - clearance mínimo (segurança)

Cada figura tem 3 boxplots lado a lado (RRT, RRT-Connect, EST).

Uso:
  python boxplots.py --scene cena4
  python boxplots.py --scene cena4 --show         # mostra na tela
  python boxplots.py --scene cena4 --format pdf   # exporta PDF
"""

import argparse
import csv
import os
import sys
from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TCC_ROOT   = os.path.dirname(os.path.dirname(SCRIPT_DIR))

# ---------------------------------------------------------------------------
# Configuração das métricas
# ---------------------------------------------------------------------------
METRICS = [
    {
        "key":       "planning_time_s",
        "label":     "Tempo de planejamento (s)",
        "title":     "Tempo de planejamento",
        "filename":  "01_planning_time",
        "log_scale": True,
    },
    {
        "key":       "execution_time_s",
        "label":     "Tempo de execução (s)",
        "title":     "Tempo de execução",
        "filename":  "02_execution_time",
        "log_scale": False,
    },
    {
        "key":       "final_error_m",
        "label":     "Erro final ao goal (m)",
        "title":     "Erro final",
        "filename":  "03_final_error",
        "log_scale": False,
    },
    {
        "key":       "processed_waypoints",
        "label":     "Nº de waypoints do caminho",
        "title":     "Tamanho do caminho",
        "filename":  "04_path_length",
        "log_scale": False,
    },
    {
        "key":       "min_clearance_m",
        "label":     "Clearance mínimo (m)",
        "title":     "Margem de segurança",
        "filename":  "05_clearance",
        "log_scale": False,
    },
]

ALGO_ORDER  = ["rrt", "rrt_connect", "est"]
ALGO_LABELS = {
    "rrt":         "RRT",
    "rrt_connect": "RRT-Connect",
    "est":         "EST",
}
ALGO_COLORS = {
    "rrt":         "#4C72B0",   # azul
    "rrt_connect": "#55A868",   # verde
    "est":         "#C44E52",   # vermelho
}


def load_csv(csv_path: str) -> dict:
    """Lê o CSV e retorna dict {metric_key: {algo: [values]}}."""
    data = defaultdict(lambda: defaultdict(list))
    with open(csv_path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            algo = row["algo"]
            success = row["success"].lower() in ("true", "1", "yes")
            for m in METRICS:
                k = m["key"]
                if row.get(k) is None or row[k] == "":
                    continue
                try:
                    v = float(row[k])
                except ValueError:
                    continue
                # Filtra runs com falha (NaN, 0 em métricas que precisam de path)
                if k in ("execution_time_s", "final_error_m",
                         "processed_waypoints", "min_clearance_m"):
                    if not success:
                        continue
                if v != v:  # NaN
                    continue
                data[k][algo].append(v)
    return data


def plot_one(metric: dict, data_by_algo: dict, output_path: str,
             show: bool = False, format_: str = "png"):
    """Gera UM boxplot para uma métrica específica."""
    fig, ax = plt.subplots(figsize=(7, 5))
    box_data    = []
    box_labels  = []
    box_colors  = []
    for algo in ALGO_ORDER:
        vals = data_by_algo.get(algo, [])
        if not vals:
            continue
        box_data.append(vals)
        box_labels.append(f"{ALGO_LABELS[algo]}\n(n={len(vals)})")
        box_colors.append(ALGO_COLORS[algo])

    if not box_data:
        print(f"  [aviso] sem dados para {metric['key']}")
        plt.close(fig)
        return

    bp = ax.boxplot(box_data, labels=box_labels,
                    patch_artist=True, showmeans=True,
                    meanprops=dict(marker="D", markerfacecolor="white",
                                   markeredgecolor="black", markersize=7),
                    medianprops=dict(color="black", linewidth=1.5))

    for patch, color in zip(bp["boxes"], box_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    ax.set_ylabel(metric["label"], fontsize=12)
    ax.set_title(metric["title"], fontsize=13, fontweight="bold")
    ax.grid(axis="y", alpha=0.3, linestyle="--")
    if metric["log_scale"]:
        ax.set_yscale("log")

    plt.tight_layout()
    plt.savefig(f"{output_path}.{format_}", dpi=150, bbox_inches="tight")
    print(f"  ✓ {output_path}.{format_}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True)
    parser.add_argument("--csv", default=None,
                        help="Caminho do CSV (padrão: results/comparison_<scene>.csv)")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--format", default="png", choices=["png", "pdf", "svg"])
    args = parser.parse_args()

    csv_path = args.csv or os.path.join(
        TCC_ROOT, "experiments_tcc", "results", f"comparison_{args.scene}.csv"
    )
    if not os.path.exists(csv_path):
        print(f"[ERRO] CSV não encontrado: {csv_path}")
        return 1

    out_dir = os.path.join(TCC_ROOT, "experiments_tcc", "plots", f"boxplots_{args.scene}")
    os.makedirs(out_dir, exist_ok=True)

    print(f"Lendo: {csv_path}")
    data = load_csv(csv_path)
    print(f"Algoritmos encontrados: {list({a for m in data.values() for a in m})}")
    print(f"Gerando 5 boxplots em: {out_dir}\n")

    for m in METRICS:
        plot_one(m, data[m["key"]],
                 os.path.join(out_dir, m["filename"]),
                 show=args.show, format_=args.format)

    print(f"\nPronto! Use as figuras de {out_dir} no capítulo de resultados do TCC.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
