"""Gera gráficos de média ± desvio-padrão consolidados para o TCC.

Diferente dos boxplots, esse script gera UM gráfico por métrica mostrando
todas as 4 cenas lado a lado, com 3 barras (RRT, RRT-Connect, EST) por cena.

Visual mais limpo, mais elegante e coerente com a apresentação por tabela.

Uso:
  python summary_plots.py                 # gera os 2 gráficos principais
  python summary_plots.py --all-metrics   # gera todos os 5
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TCC_ROOT   = os.path.dirname(os.path.dirname(SCRIPT_DIR))

# ---------------------------------------------------------------------------
# Estilo acadêmico
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "font.family":       "serif",
    "font.size":         13,
    "axes.titlesize":    15,
    "axes.labelsize":    14,
    "xtick.labelsize":   12,
    "ytick.labelsize":   12,
    "legend.fontsize":   12,
    "axes.titleweight":  "bold",
    "axes.linewidth":    1.2,
    "figure.facecolor":  "white",
    "axes.facecolor":    "white",
    "savefig.facecolor": "white",
    "savefig.edgecolor": "none",
    "axes.grid":         True,
    "grid.alpha":        0.35,
    "grid.linestyle":    "--",
    "grid.linewidth":    0.6,
})

# ---------------------------------------------------------------------------
# Configuração
# ---------------------------------------------------------------------------

# Mapeamento: arquivo CSV → nome da cena no TCC
SCENE_FILE_TO_LABEL = {
    "cena1": "Cena 1",
    "cena2": "Cena 2",
    "cena5": "Cena 3",   # importante! cena5 = Cena 3 no texto
    "cena4": "Cena 4",
}
SCENE_ORDER = ["cena1", "cena2", "cena5", "cena4"]

ALGO_ORDER  = ["rrt", "rrt_connect", "est"]
ALGO_LABELS = {
    "rrt":         "RRT",
    "rrt_connect": "RRT-Connect",
    "est":         "EST",
}

# Paletas de cores disponíveis (escolha com --palette)
PALETTES = {
    "default": {
        "rrt":         "#4C72B0",   # azul seaborn
        "rrt_connect": "#55A868",   # verde seaborn
        "est":         "#C44E52",   # vermelho seaborn
    },
    "nordic": {
        "rrt":         "#5E81AC",   # azul nórdico suave
        "rrt_connect": "#A3BE8C",   # verde sage
        "est":         "#BF616A",   # vermelho nórdico
    },
    "berry": {
        "rrt":         "#264653",   # azul-petróleo escuro
        "rrt_connect": "#2A9D8F",   # turquesa vibrante
        "est":         "#E76F51",   # coral aquecido
    },
    "sunset": {
        "rrt":         "#3D5A80",   # azul marinho
        "rrt_connect": "#98C1D9",   # azul claro
        "est":         "#EE6C4D",   # laranja vivo
    },
    "earthy": {
        "rrt":         "#6B705C",   # verde oliva
        "rrt_connect": "#A5A58D",   # sage neutro
        "est":         "#CB997E",   # taupe rosado
    },
    "pastel": {
        "rrt":         "#A8DADC",   # azul gelo
        "rrt_connect": "#81B29A",   # verde menta
        "est":         "#F2A0A1",   # rosa pêssego
    },
    "material": {
        "rrt":         "#1976D2",   # azul material
        "rrt_connect": "#388E3C",   # verde material
        "est":         "#D32F2F",   # vermelho material
    },
    "ocean": {
        "rrt":         "#003049",   # azul profundo
        "rrt_connect": "#669BBC",   # azul oceano
        "est":         "#FCBF49",   # amarelo dourado
    },
    "purple_pop": {
        "rrt":         "#7209B7",   # roxo vibrante
        "rrt_connect": "#4361EE",   # azul royal
        "est":         "#F72585",   # rosa choque
    },
    "academic": {
        "rrt":         "#264653",   # azul-petróleo
        "rrt_connect": "#8AB17D",   # verde oliva claro
        "est":         "#E9C46A",   # mostarda
    },
}

ALGO_COLORS = PALETTES["nordic"]   # default mais bonito que o original

METRICS = {
    "planning_time_s": {
        "label":     "Tempo de planejamento (s)",
        "title":     "Tempo médio de planejamento por algoritmo e cenário",
        "filename":  "summary_planning_time",
        "log_scale": True,
    },
    "min_clearance_m": {
        "label":     "Clearance mínima (m)",
        "title":     "Clearance mínima média por algoritmo e cenário",
        "filename":  "summary_clearance",
        "log_scale": False,
    },
    "execution_time_s": {
        "label":     "Tempo de execução (s)",
        "title":     "Tempo médio de execução por algoritmo e cenário",
        "filename":  "summary_execution_time",
        "log_scale": False,
    },
    "final_error_m": {
        "label":     "Erro final (m)",
        "title":     "Erro final médio por algoritmo e cenário",
        "filename":  "summary_final_error",
        "log_scale": False,
    },
}


def load_data():
    """Carrega dados de todos os CSVs em uma estrutura aninhada."""
    results_dir = os.path.join(TCC_ROOT, "experiments_tcc", "results")
    data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    # data[metric][algo][scene] = [values...]

    for scene_key in SCENE_ORDER:
        csv_path = os.path.join(results_dir, f"comparison_{scene_key}.csv")
        if not os.path.exists(csv_path):
            print(f"  [aviso] CSV não encontrado: {csv_path}")
            continue

        with open(csv_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                algo = row["algo"]
                success = row["success"].lower() in ("true", "1", "yes")
                for metric in METRICS:
                    val_str = row.get(metric)
                    if not val_str:
                        continue
                    try:
                        val = float(val_str)
                    except ValueError:
                        continue
                    if math.isnan(val):
                        continue
                    if metric in ("execution_time_s", "final_error_m",
                                  "min_clearance_m"):
                        if not success:
                            continue
                    data[metric][algo][scene_key].append(val)
    return data


def compute_stats(values):
    if not values:
        return float("nan"), float("nan")
    arr = np.asarray(values, dtype=float)
    return float(np.mean(arr)), float(np.std(arr, ddof=0))


def plot_metric(metric_key, metric_cfg, data, output_dir, format_="png"):
    """Gera UM gráfico de barras com média ± desvio-padrão."""
    n_scenes = len(SCENE_ORDER)
    n_algos = len(ALGO_ORDER)

    bar_width = 0.25
    x_base = np.arange(n_scenes)

    fig, ax = plt.subplots(figsize=(9, 5.5), dpi=150)

    for i, algo in enumerate(ALGO_ORDER):
        means, stds = [], []
        for scene in SCENE_ORDER:
            vals = data[metric_key][algo].get(scene, [])
            m, s = compute_stats(vals)
            means.append(m)
            stds.append(s)

        x = x_base + (i - 1) * bar_width
        bars = ax.bar(
            x, means, bar_width,
            yerr=stds, capsize=5,
            label=ALGO_LABELS[algo],
            color=ALGO_COLORS[algo],
            alpha=0.85,
            edgecolor="#222222",
            linewidth=1.0,
            error_kw=dict(ecolor="#333333", elinewidth=1.5, alpha=0.8),
        )

        # Anota o valor da média acima de cada barra
        for bar, mean_val in zip(bars, means):
            if math.isnan(mean_val):
                continue
            height = bar.get_height()
            # Formato adaptado à escala da métrica
            if metric_key == "min_clearance_m":
                lbl = f"{mean_val:.3f}"
            elif metric_key == "final_error_m":
                lbl = f"{mean_val:.3f}"
            elif metric_key == "execution_time_s":
                lbl = f"{mean_val:.1f}"
            else:
                lbl = f"{mean_val:.2f}"

            offset = height * 0.02 if not metric_cfg["log_scale"] else height * 0.1
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                height + offset,
                lbl,
                ha="center", va="bottom",
                fontsize=9, color="#222222",
            )

    ax.set_xticks(x_base)
    ax.set_xticklabels([SCENE_FILE_TO_LABEL[s] for s in SCENE_ORDER])
    ax.set_ylabel(metric_cfg["label"], fontsize=13)
    ax.set_title(metric_cfg["title"], fontsize=14, pad=12)

    if metric_cfg["log_scale"]:
        ax.set_yscale("log")

    ax.legend(loc="best", frameon=True, edgecolor="#666666")
    ax.grid(axis="y", alpha=0.35, linestyle="--", linewidth=0.6)
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    plt.tight_layout()
    output_path = os.path.join(output_dir, f"{metric_cfg['filename']}.{format_}")
    plt.savefig(output_path, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"  ✓ {output_path}")
    plt.close(fig)
    return output_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--all-metrics", action="store_true",
                        help="Gera todas as 4 métricas (padrão: só plan + clearance)")
    parser.add_argument("--format", default="png", choices=["png", "pdf", "svg"])
    parser.add_argument("--palette", default="nordic",
                        choices=list(PALETTES.keys()) + ["list"],
                        help="Paleta de cores (use 'list' pra ver opções)")
    args = parser.parse_args()

    if args.palette == "list":
        print("\nPaletas disponíveis:")
        for name, colors in PALETTES.items():
            preview = " | ".join(f"{algo}: {col}" for algo, col in colors.items())
            print(f"  {name:<14} → {preview}")
        print()
        return 0

    # Aplica a paleta escolhida globalmente
    global ALGO_COLORS
    ALGO_COLORS = PALETTES[args.palette]
    print(f"🎨 Usando paleta: {args.palette}\n")

    output_dir = os.path.join(TCC_ROOT, "experiments_tcc", "plots", "summary")
    os.makedirs(output_dir, exist_ok=True)

    print("Carregando dados dos 4 cenários...")
    data = load_data()
    print("✓ Dados carregados.\n")

    # Define quais métricas gerar
    if args.all_metrics:
        keys_to_plot = list(METRICS.keys())
    else:
        keys_to_plot = ["planning_time_s", "min_clearance_m"]

    print(f"Gerando gráficos em: {output_dir}\n")
    for key in keys_to_plot:
        cfg = METRICS[key]
        plot_metric(key, cfg, data, output_dir, format_=args.format)

    print(f"\n🎉 Pronto! {len(keys_to_plot)} gráfico(s) salvo(s) em:")
    print(f"   {output_dir}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
