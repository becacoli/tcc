"""Gera gráficos comparativos a partir dos CSVs produzidos por run_coppelia_batch.py.

Uso:
  python plot_coppelia_results.py results_coppelia.csv
  python plot_coppelia_results.py batch_rrt.csv batch_est.csv --output figs/
  python plot_coppelia_results.py results_coppelia.csv --latex
  python plot_coppelia_results.py results_coppelia.csv --no-show
"""

import argparse
import csv
import math
import os
import sys


# ---------------------------------------------------------------------------
# Metadados visuais por preset
# ---------------------------------------------------------------------------

PRESET_LABELS = {
    "rrt":                    "RRT",
    "rrt_star":               "RRT*",
    "informed_rrt_star":      "Informed RRT*",
    "rrt_connect_fast":       "RRT-Connect\n(rápido)",
    "rrt_connect_precise":    "RRT-Connect\n(preciso)",
    "est_hybrid_exploratory": "HybridEST\n(α=0.70)",
    "est_hybrid_safe":        "HybridEST\n(α=0.35)",
}

PRESET_COLORS = {
    "rrt":                    "#1976D2",
    "rrt_star":               "#0D47A1",
    "informed_rrt_star":      "#3949AB",
    "rrt_connect_fast":       "#29B6F6",
    "rrt_connect_precise":    "#0288D1",
    "est_hybrid_exploratory": "#FFA726",
    "est_hybrid_safe":        "#E65100",
}

# Ordem preferida de exibição (RRT antes de EST)
DISPLAY_ORDER = [
    "rrt",
    "rrt_connect_fast",
    "rrt_connect_precise",
    "rrt_star",
    "informed_rrt_star",
    "est_hybrid_exploratory",
    "est_hybrid_safe",
]


# ---------------------------------------------------------------------------
# Carregamento de CSV
# ---------------------------------------------------------------------------

def load_csv(paths: list[str]) -> dict[str, list[dict]]:
    """Carrega um ou mais CSVs e agrupa por preset."""
    runs_by_preset: dict[str, list[dict]] = {}
    for path in paths:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                preset = row["preset"]
                run = {
                    "success":            row["success"].lower() in ("true", "1"),
                    "reason":             row["reason"],
                    "planning_time_s":    _float(row["planning_time_s"]),
                    "execution_time_s":   _float(row["execution_time_s"]),
                    "final_error_m":      _float(row["final_error_m"]),
                    "iterations":         _float(row["iterations"]),
                    "processed_waypoints":_float(row["processed_waypoints"]),
                    "skipped_waypoints":  _float(row["skipped_waypoints"]),
                }
                runs_by_preset.setdefault(preset, []).append(run)
    return runs_by_preset


def _float(s: str) -> float:
    try:
        return float(s)
    except (ValueError, TypeError):
        return float("nan")


# ---------------------------------------------------------------------------
# Estatísticas
# ---------------------------------------------------------------------------

def _mean_std(values: list[float]) -> tuple[float, float]:
    vals = [v for v in values if not math.isnan(v)]
    if not vals:
        return float("nan"), float("nan")
    mean = sum(vals) / len(vals)
    variance = sum((v - mean) ** 2 for v in vals) / len(vals)
    return mean, math.sqrt(variance)


def compute_stats(runs: list[dict]) -> dict:
    n = len(runs)
    n_ok = sum(1 for r in runs if r["success"])
    ok_runs = [r for r in runs if r["success"]]

    stats = {
        "n": n,
        "n_success": n_ok,
        "success_rate": n_ok / n if n else 0.0,
    }

    for key in ("planning_time_s", "iterations", "processed_waypoints"):
        m, s = _mean_std([r[key] for r in runs])
        stats[f"{key}_mean"] = m
        stats[f"{key}_std"] = s

    for key in ("execution_time_s", "final_error_m", "skipped_waypoints"):
        m, s = _mean_std([r[key] for r in ok_runs])
        stats[f"{key}_mean"] = m
        stats[f"{key}_std"] = s

    return stats


def stats_all(runs_by_preset: dict) -> dict[str, dict]:
    return {p: compute_stats(r) for p, r in runs_by_preset.items()}


# ---------------------------------------------------------------------------
# Ordenação dos presets conforme DISPLAY_ORDER
# ---------------------------------------------------------------------------

def sorted_presets(all_stats: dict) -> list[str]:
    ordered = [p for p in DISPLAY_ORDER if p in all_stats]
    # Adiciona presets desconhecidos no fim
    for p in all_stats:
        if p not in ordered:
            ordered.append(p)
    return ordered


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _bar_ax(ax, presets, values, errors, ylabel, title, colors, ylim_bottom=0):
    import numpy as np
    x = np.arange(len(presets))
    labels = [PRESET_LABELS.get(p, p) for p in presets]
    bars = ax.bar(x, values, color=colors, width=0.55, zorder=3,
                  yerr=[e if not math.isnan(e) else 0 for e in errors],
                  capsize=4, error_kw={"elinewidth": 1.2, "ecolor": "#444"})
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8.5)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.set_ylim(bottom=ylim_bottom)
    ax.grid(axis="y", alpha=0.3, zorder=0)
    ax.spines[["top", "right"]].set_visible(False)
    # Valores sobre as barras
    for bar, val in zip(bars, values):
        if not math.isnan(val):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + bar.get_height() * 0.02 + 0.001,
                f"{val:.2f}" if val < 10 else f"{val:.1f}",
                ha="center", va="bottom", fontsize=7.5,
            )


def generate_plots(
    runs_by_preset: dict,
    all_stats: dict,
    output_dir: str,
    show: bool = True,
):
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[aviso] matplotlib não encontrado — gráficos não serão gerados.")
        return []

    os.makedirs(output_dir, exist_ok=True)
    presets = sorted_presets(all_stats)
    colors = [PRESET_COLORS.get(p, "#888") for p in presets]
    saved = []

    # ------------------------------------------------------------------
    # Figura 1: Visão geral (2×2)
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle("Comparação CoppeliaSim — Cena 1", fontsize=13, fontweight="bold", y=0.98)

    # 1a) Tempo de planejamento
    _bar_ax(
        axes[0, 0], presets,
        [all_stats[p]["planning_time_s_mean"] for p in presets],
        [all_stats[p]["planning_time_s_std"] for p in presets],
        "Tempo (s)", "Tempo de Planejamento", colors,
    )

    # 1b) Taxa de sucesso
    import numpy as np
    x = np.arange(len(presets))
    success_rates = [all_stats[p]["success_rate"] * 100 for p in presets]
    n_labels = [f"n={all_stats[p]['n']}" for p in presets]
    bars = axes[0, 1].bar(x, success_rates, color=colors, width=0.55, zorder=3)
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels([PRESET_LABELS.get(p, p) for p in presets], fontsize=8.5)
    axes[0, 1].set_ylabel("Taxa (%)", fontsize=9)
    axes[0, 1].set_title("Taxa de Sucesso", fontsize=10, fontweight="bold")
    axes[0, 1].set_ylim(0, 115)
    axes[0, 1].grid(axis="y", alpha=0.3, zorder=0)
    axes[0, 1].spines[["top", "right"]].set_visible(False)
    for bar, rate, nl in zip(bars, success_rates, n_labels):
        axes[0, 1].text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 2,
            f"{rate:.0f}%\n({nl})",
            ha="center", va="bottom", fontsize=7.5,
        )

    # 1c) Tempo de execução (apenas rodadas com sucesso)
    _bar_ax(
        axes[1, 0], presets,
        [all_stats[p]["execution_time_s_mean"] for p in presets],
        [all_stats[p]["execution_time_s_std"] for p in presets],
        "Tempo (s)", "Tempo de Execução (rodadas bem-sucedidas)", colors,
    )

    # 1d) Erro final ao goal
    _bar_ax(
        axes[1, 1], presets,
        [all_stats[p]["final_error_m_mean"] for p in presets],
        [all_stats[p]["final_error_m_std"] for p in presets],
        "Erro (m)", "Erro Final ao Goal", colors,
    )

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    path1 = os.path.join(output_dir, "coppelia_overview.png")
    fig.savefig(path1, dpi=180, bbox_inches="tight")
    saved.append(path1)
    print(f"✓ {path1}")

    # ------------------------------------------------------------------
    # Figura 2: Tempo de planejamento em escala logarítmica
    # ------------------------------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(9, 5))
    plan_means = [all_stats[p]["planning_time_s_mean"] for p in presets]
    plan_stds  = [all_stats[p]["planning_time_s_std"]  for p in presets]
    x = np.arange(len(presets))
    bars = ax2.bar(x, plan_means, color=colors, width=0.55, zorder=3,
                   yerr=[s if not math.isnan(s) else 0 for s in plan_stds],
                   capsize=4, error_kw={"elinewidth": 1.2, "ecolor": "#444"})
    ax2.set_yscale("log")
    ax2.set_xticks(x)
    ax2.set_xticklabels([PRESET_LABELS.get(p, p) for p in presets], fontsize=9)
    ax2.set_ylabel("Tempo (s) — escala log", fontsize=10)
    ax2.set_title("Tempo de Planejamento (escala logarítmica)", fontsize=11, fontweight="bold")
    ax2.grid(axis="y", which="both", alpha=0.3, zorder=0)
    ax2.spines[["top", "right"]].set_visible(False)
    for bar, val in zip(bars, plan_means):
        if not math.isnan(val):
            ax2.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() * 1.15,
                f"{val:.2f}s" if val < 10 else f"{val:.1f}s",
                ha="center", va="bottom", fontsize=8,
            )
    plt.tight_layout()
    path2 = os.path.join(output_dir, "coppelia_planning_time_log.png")
    fig2.savefig(path2, dpi=180, bbox_inches="tight")
    saved.append(path2)
    print(f"✓ {path2}")

    # ------------------------------------------------------------------
    # Figura 3: Distribuição individual por rodada (strip plot)
    # ------------------------------------------------------------------
    fig3, axes3 = plt.subplots(1, 2, figsize=(12, 5))
    fig3.suptitle("Distribuição por Rodada — CoppeliaSim", fontsize=12, fontweight="bold")

    for ax, key, ylabel, title in [
        (axes3[0], "planning_time_s", "Tempo (s)", "Tempo de Planejamento por Rodada"),
        (axes3[1], "execution_time_s", "Tempo (s)", "Tempo de Execução por Rodada"),
    ]:
        import random as _random
        for xi, preset in enumerate(presets):
            vals = [r[key] for r in runs_by_preset[preset] if not math.isnan(r[key])]
            ok_mask = [r["success"] for r in runs_by_preset[preset] if not math.isnan(r[key])]
            color = PRESET_COLORS.get(preset, "#888")
            jitter = [xi + _random.uniform(-0.18, 0.18) for _ in vals]
            for jx, v, ok in zip(jitter, vals, ok_mask):
                ax.scatter(jx, v, color=color if ok else "#ccc",
                           edgecolors="#555" if ok else "#bbb",
                           linewidths=0.5, s=40, zorder=4, alpha=0.85)
            if vals:
                mean = sum(vals) / len(vals)
                ax.plot([xi - 0.25, xi + 0.25], [mean, mean],
                        color=color, linewidth=2.0, zorder=5)
        ax.set_xticks(range(len(presets)))
        ax.set_xticklabels([PRESET_LABELS.get(p, p) for p in presets], fontsize=8.5)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.grid(axis="y", alpha=0.3, zorder=0)
        ax.spines[["top", "right"]].set_visible(False)

    # Legenda manual
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#555",
               markersize=7, label="Bem-sucedido"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#ccc",
               markersize=7, label="Falhou"),
        Line2D([0], [0], color="#555", linewidth=2, label="Média"),
    ]
    axes3[1].legend(handles=legend_elements, fontsize=8, loc="upper right")

    plt.tight_layout()
    path3 = os.path.join(output_dir, "coppelia_runs_distribution.png")
    fig3.savefig(path3, dpi=180, bbox_inches="tight")
    saved.append(path3)
    print(f"✓ {path3}")

    if show:
        plt.show()
    else:
        plt.close("all")

    return saved


# ---------------------------------------------------------------------------
# Tabela LaTeX
# ---------------------------------------------------------------------------

def print_latex_table(all_stats: dict, presets: list[str]):
    def _fmt(mean, std):
        if math.isnan(mean):
            return r"\text{---}"
        if math.isnan(std) or std < 1e-9:
            return f"{mean:.2f}"
        return f"{mean:.2f} \\pm {std:.2f}"

    print()
    print(r"% --- Tabela gerada por plot_coppelia_results.py ---")
    print(r"\begin{table}[ht]")
    print(r"\centering")
    print(r"\caption{Comparação de desempenho no CoppeliaSim --- Cena 1}")
    print(r"\label{tab:coppelia_results}")
    print(r"\begin{tabular}{lrrrr}")
    print(r"\toprule")
    print(r"Preset & Sucesso & $t_\text{plan}$ (s) & $t_\text{exec}$ (s) & Erro final (m) \\")
    print(r"\midrule")
    for p in presets:
        s = all_stats[p]
        label = PRESET_LABELS.get(p, p).replace("\n", " ")
        succ  = f"{s['n_success']}/{s['n']} ({s['success_rate']*100:.0f}\\%)"
        tp    = _fmt(s["planning_time_s_mean"], s["planning_time_s_std"])
        te    = _fmt(s["execution_time_s_mean"], s["execution_time_s_std"])
        fe    = _fmt(s["final_error_m_mean"],    s["final_error_m_std"])
        print(f"  {label:<30} & {succ:<14} & ${tp}$ & ${te}$ & ${fe}$ \\\\")
    print(r"\bottomrule")
    print(r"\end{tabular}")
    print(r"\end{table}")
    print()


# ---------------------------------------------------------------------------
# Tabela texto
# ---------------------------------------------------------------------------

def print_text_table(all_stats: dict, presets: list[str]):
    sep = "=" * 105
    print(f"\n{sep}")
    print("ESTATÍSTICAS — CoppeliaSim")
    print(sep)
    print(
        f"{'Preset':<28} {'n':<5} {'Sucesso':<10} "
        f"{'t_plan(s)':<16} {'t_exec(s)':<16} {'err(m)':<14} {'waypts':<8} {'skip':<6}"
    )
    print("-" * 105)

    def _f(m, s):
        if math.isnan(m):
            return "   —   "
        if math.isnan(s) or s < 1e-9:
            return f"{m:6.3f}      "
        return f"{m:6.3f}±{s:.3f}"

    for p in presets:
        s = all_stats[p]
        succ = f"{s['n_success']}/{s['n']} ({s['success_rate']*100:.0f}%)"
        print(
            f"{p:<28} {s['n']:<5} {succ:<10} "
            f"{_f(s['planning_time_s_mean'],  s['planning_time_s_std']):<16} "
            f"{_f(s['execution_time_s_mean'], s['execution_time_s_std']):<16} "
            f"{_f(s['final_error_m_mean'],    s['final_error_m_std']):<14} "
            f"{s['processed_waypoints_mean']:<8.0f} "
            f"{_f(s['skipped_waypoints_mean'], float('nan'))}"
        )
    print(sep)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Gera gráficos e tabelas a partir dos CSVs do run_coppelia_batch.py.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("csvs", nargs="+", metavar="CSV", help="Arquivos CSV de entrada")
    parser.add_argument("--output", "-o", default="results/figs",
                        help="Diretório de saída para os PNGs (padrão: results/figs)")
    parser.add_argument("--no-show", action="store_true",
                        help="Não exibe as janelas — apenas salva os PNGs")
    parser.add_argument("--latex", action="store_true",
                        help="Imprime tabela LaTeX pronta para copiar no TCC")
    args = parser.parse_args()

    # Verifica arquivos
    for path in args.csvs:
        if not os.path.isfile(path):
            print(f"[erro] Arquivo não encontrado: {path}")
            return 1

    runs_by_preset = load_csv(args.csvs)
    if not runs_by_preset:
        print("[erro] Nenhum dado carregado.")
        return 1

    all_stats = stats_all(runs_by_preset)
    presets   = sorted_presets(all_stats)

    print_text_table(all_stats, presets)

    if args.latex:
        print_latex_table(all_stats, presets)

    saved = generate_plots(
        runs_by_preset, all_stats,
        output_dir=args.output,
        show=not args.no_show,
    )

    if saved:
        print(f"\n{len(saved)} gráfico(s) salvos em '{args.output}/'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
