"""Gera os 5 boxplots de comparação final dos algoritmos (TCC).

Lê o CSV de comparison_<scene>.csv gerado por compare_best.py e produz
5 figuras separadas, uma para cada métrica, com estilo acadêmico
(fundo branco, fonte legível, anotações de mediana, etc.).

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
# Estilo geral para publicação acadêmica
# ---------------------------------------------------------------------------
plt.rcParams.update({
    "font.family":       "serif",          # estilo acadêmico (Times-like)
    "font.size":         13,
    "axes.titlesize":    15,
    "axes.labelsize":    14,
    "xtick.labelsize":   12,
    "ytick.labelsize":   11,
    "axes.titleweight":  "bold",
    "axes.linewidth":    1.2,
    "figure.facecolor":  "white",          # fundo branco
    "axes.facecolor":    "white",
    "savefig.facecolor": "white",
    "savefig.edgecolor": "none",
    "axes.grid":         True,
    "grid.alpha":        0.4,
    "grid.linestyle":    "--",
    "grid.linewidth":    0.6,
})

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
        "unit":      "s",
        "fmt":       "{:.3f}",
    },
    {
        "key":       "execution_time_s",
        "label":     "Tempo de execução (s)",
        "title":     "Tempo de execução",
        "filename":  "02_execution_time",
        "log_scale": False,
        "unit":      "s",
        "fmt":       "{:.2f}",
    },
    {
        "key":       "final_error_m",
        "label":     "Erro final ao objetivo (m)",
        "title":     "Erro final",
        "filename":  "03_final_error",
        "log_scale": False,
        "unit":      "m",
        "fmt":       "{:.3f}",
    },
    {
        "key":       "processed_waypoints",
        "label":     "Nº de waypoints do caminho",
        "title":     "Tamanho do caminho",
        "filename":  "04_path_length",
        "log_scale": False,
        "unit":      "",
        "fmt":       "{:.0f}",
    },
    {
        "key":       "min_clearance_m",
        "label":     "Clearance mínima (m)",
        "title":     "Margem de segurança (clearance)",
        "filename":  "05_clearance",
        "log_scale": False,
        "unit":      "m",
        "fmt":       "{:.4f}",
    },
]

ALGO_ORDER  = ["rrt", "rrt_connect", "est"]
ALGO_LABELS = {
    "rrt":         "RRT",
    "rrt_connect": "RRT-Connect",
    "est":         "EST",
}
# Cores sóbrias adequadas a publicação acadêmica
ALGO_COLORS = {
    "rrt":         "#4C72B0",   # azul moderado
    "rrt_connect": "#55A868",   # verde moderado
    "est":         "#C44E52",   # vermelho moderado
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
                if k in ("execution_time_s", "final_error_m",
                         "processed_waypoints", "min_clearance_m"):
                    if not success:
                        continue
                if v != v:  # NaN
                    continue
                data[k][algo].append(v)
    return data


def _median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return float("nan")
    if n % 2 == 1:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) / 2


def plot_one(metric: dict, data_by_algo: dict, output_path: str,
             show: bool = False, format_: str = "png"):
    """Gera UM boxplot para uma métrica específica."""
    fig, ax = plt.subplots(figsize=(8, 5.5), dpi=150)
    box_data    = []
    box_labels  = []
    box_colors  = []
    medians     = []

    for algo in ALGO_ORDER:
        vals = data_by_algo.get(algo, [])
        if not vals:
            continue
        box_data.append(vals)
        box_labels.append(f"{ALGO_LABELS[algo]}\n(n={len(vals)})")
        box_colors.append(ALGO_COLORS[algo])
        medians.append(_median(vals))

    if not box_data:
        print(f"  [aviso] sem dados para {metric['key']}")
        plt.close(fig)
        return

    bp = ax.boxplot(
        box_data,
        labels=box_labels,
        patch_artist=True,
        showmeans=True,
        widths=0.55,
        meanprops=dict(
            marker="D",
            markerfacecolor="white",
            markeredgecolor="black",
            markersize=7,
        ),
        medianprops=dict(color="black", linewidth=2.0),
        whiskerprops=dict(color="#333333", linewidth=1.2),
        capprops=dict(color="#333333", linewidth=1.2),
        flierprops=dict(
            marker="o",
            markerfacecolor="white",
            markeredgecolor="#666666",
            markersize=5,
            alpha=0.6,
        ),
    )

    for patch, color in zip(bp["boxes"], box_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.75)
        patch.set_edgecolor("#222222")
        patch.set_linewidth(1.2)

    # Anotações com valor da mediana acima de cada box
    fmt = metric["fmt"]
    unit = metric["unit"]
    y_min, y_max = ax.get_ylim()
    for i, med in enumerate(medians, start=1):
        if med != med:  # NaN
            continue
        label = fmt.format(med)
        if unit:
            label = f"{label}{(' ' + unit) if unit else ''}"
        if metric["log_scale"]:
            offset_factor = 1.20
            y_pos = med * offset_factor
        else:
            y_pos = med + (y_max - y_min) * 0.04
        ax.annotate(
            label,
            xy=(i, med),
            xytext=(i, y_pos),
            ha="center",
            va="bottom",
            fontsize=9,
            color="#222222",
            fontweight="bold",
        )

    ax.set_ylabel(metric["label"], fontsize=13)
    ax.set_title(metric["title"], fontsize=14, pad=12)
    ax.grid(axis="y", alpha=0.35, linestyle="--", linewidth=0.6)
    ax.set_axisbelow(True)

    if metric["log_scale"]:
        ax.set_yscale("log")
        ax.set_ylabel(metric["label"] + " — escala logarítmica", fontsize=12)

    # Remove eixo superior/direito para visual mais limpo
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    plt.tight_layout()
    out_full = f"{output_path}.{format_}"
    plt.savefig(out_full, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"  ✓ {out_full}")
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
    algos_found = sorted({a for m in data.values() for a in m})
    print(f"Algoritmos encontrados: {algos_found}")
    print(f"Gerando 5 boxplots em: {out_dir}\n")

    for m in METRICS:
        plot_one(m, data[m["key"]],
                 os.path.join(out_dir, m["filename"]),
                 show=args.show, format_=args.format)

    print(f"\nPronto! Use as figuras de {out_dir} no capítulo de resultados do TCC.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
