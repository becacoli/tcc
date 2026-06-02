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
    "rrt":                        "RRT",
    "rrt_star":                   "RRT*",
    "informed_rrt_star":          "Informed RRT*",
    "rrt_connect_fast":           "RRT-Connect\n(rápido)",
    "rrt_connect_precise":        "RRT-Connect\n(preciso)",
    "est":                        "EST\n(puro)",
    "est_hybrid_exploratory":     "HybridEST\n(α=0.70)",
    "est_hybrid_safe":            "HybridEST\n(α=0.35)",
    # variantes "_fair"
    "rrt_fair":                       "RRT\n(fair)",
    "rrt_connect_fair":               "RRT-Connect\n(fair)",
    "est_hybrid_exploratory_fair":    "HybridEST\n(α=0.70 fair)",
    "est_hybrid_safe_fair":           "HybridEST\n(α=0.35 fair)",
}

PRESET_COLORS = {
    "rrt":                        "#1976D2",
    "rrt_star":                   "#0D47A1",
    "informed_rrt_star":          "#3949AB",
    "rrt_connect_fast":           "#29B6F6",
    "rrt_connect_precise":        "#0288D1",
    "est":                        "#FF7043",
    "est_hybrid_exploratory":     "#FFA726",
    "est_hybrid_safe":            "#E65100",
    "rrt_fair":                   "#42A5F5",
    "rrt_connect_fair":           "#4DD0E1",
    "est_hybrid_exploratory_fair":"#FFB74D",
    "est_hybrid_safe_fair":       "#BF360C",
}

# Ordem preferida de exibição
DISPLAY_ORDER = [
    "rrt", "rrt_fair",
    "rrt_connect_fast", "rrt_connect_precise", "rrt_connect_fair",
    "rrt_star", "informed_rrt_star",
    "est",
    "est_hybrid_exploratory", "est_hybrid_exploratory_fair",
    "est_hybrid_safe", "est_hybrid_safe_fair",
]


# ---------------------------------------------------------------------------
# Carregamento de CSV
# ---------------------------------------------------------------------------

def load_csv(paths: list[str]) -> dict[str, list[dict]]:
    """
    Carrega um ou mais CSVs e agrupa por preset.
    Se houver coluna 'scene', agrupa por 'scene/preset' para separar cenas.
    Compatível com CSVs antigos (sem coluna 'scene').
    """
    runs_by_preset: dict[str, list[dict]] = {}
    for path in paths:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                scene = row.get("scene", "").strip()
                preset = row["preset"]
                key = f"{scene}/{preset}" if scene else preset
                run = {
                    "scene":               scene,
                    "success":             row["success"].lower() in ("true", "1"),
                    "reason":              row["reason"],
                    "planning_time_s":     _float(row["planning_time_s"]),
                    "execution_time_s":    _float(row["execution_time_s"]),
                    "final_error_m":       _float(row["final_error_m"]),
                    "iterations":          _float(row["iterations"]),
                    "processed_waypoints": _float(row["processed_waypoints"]),
                    "skipped_waypoints":   _float(row["skipped_waypoints"]),
                    # campo novo — nan se CSV antigo não tiver a coluna
                    "min_clearance_m":     _float(row.get("min_clearance_m", "nan")),
                }
                runs_by_preset.setdefault(key, []).append(run)
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

    for key in ("execution_time_s", "final_error_m", "skipped_waypoints",
                "min_clearance_m"):
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
    """
    Ordena as chaves de all_stats.
    Chaves podem ser 'preset' (sem cena) ou 'cena/preset'.
    Agrupa por cena e dentro de cada cena segue DISPLAY_ORDER.
    """
    def _sort_key(key):
        parts = key.split("/", 1)
        scene = parts[0] if len(parts) == 2 else ""
        preset = parts[-1]
        order = DISPLAY_ORDER.index(preset) if preset in DISPLAY_ORDER else len(DISPLAY_ORDER)
        return (scene, order)

    return sorted(all_stats.keys(), key=_sort_key)


# ---------------------------------------------------------------------------
# Rótulos de exibição
# ---------------------------------------------------------------------------

def _label(key: str) -> str:
    """Converte chave 'cena/preset' ou 'preset' em rótulo legível para gráficos."""
    scene, sep, preset = key.partition("/")
    if sep:  # tem cena
        base = PRESET_LABELS.get(preset, preset)
        return f"{scene}\n{base}"
    # sem cena — key inteira é o preset
    return PRESET_LABELS.get(key, key)


def _preset_name(key: str) -> str:
    """Extrai o nome do preset de 'cena/preset' ou 'preset'."""
    return key.split("/", 1)[-1]


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def _bar_ax(ax, presets, values, errors, ylabel, title, colors, ylim_bottom=0):
    import numpy as np
    x = np.arange(len(presets))
    labels = [_label(p) for p in presets]
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
    colors = [PRESET_COLORS.get(_preset_name(p), "#888") for p in presets]
    saved = []

    # Título dinâmico baseado nas cenas presentes nos dados
    scenes_present = sorted({k.split("/")[0] for k in presets if "/" in k} or {""})
    if scenes_present == [""]:
        fig_title = "Comparação CoppeliaSim"
    else:
        fig_title = "Comparação CoppeliaSim — " + ", ".join(s.upper() for s in scenes_present if s)

    # ------------------------------------------------------------------
    # Figura 1: Visão geral (2×2)
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(fig_title, fontsize=13, fontweight="bold", y=0.98)

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
    axes[0, 1].set_xticklabels([_label(p) for p in presets], fontsize=8.5)
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
    ax2.set_xticklabels([_label(p) for p in presets], fontsize=9)
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
        ax.set_xticklabels([_label(p) for p in presets], fontsize=8.5)
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

    # ------------------------------------------------------------------
    # Figura 4: Clearance mínima do caminho (m)
    # ------------------------------------------------------------------
    # Só gera se houver dados de clearance (campo adicionado depois da v1)
    clearance_means = [all_stats[p]["min_clearance_m_mean"] for p in presets]
    if any(not math.isnan(v) for v in clearance_means):
        fig4, ax4 = plt.subplots(figsize=(9, 5))
        clearance_stds = [all_stats[p]["min_clearance_m_std"] for p in presets]
        x = np.arange(len(presets))
        bars4 = ax4.bar(
            x, clearance_means, color=colors, width=0.55, zorder=3,
            yerr=[s if not math.isnan(s) else 0 for s in clearance_stds],
            capsize=4, error_kw={"elinewidth": 1.2, "ecolor": "#444"},
        )
        ax4.set_xticks(x)
        ax4.set_xticklabels([_label(p) for p in presets], fontsize=9)
        ax4.set_ylabel("Clearance mínima (m)", fontsize=10)
        ax4.set_title(
            "Clearance Mínima do Caminho Planejado\n"
            "(distância ao obstáculo mais próximo — maior é mais seguro)",
            fontsize=11, fontweight="bold",
        )
        ax4.grid(axis="y", alpha=0.3, zorder=0)
        ax4.spines[["top", "right"]].set_visible(False)
        for bar, val in zip(bars4, clearance_means):
            if not math.isnan(val):
                ax4.text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 0.002,
                    f"{val:.3f}m",
                    ha="center", va="bottom", fontsize=8,
                )
        plt.tight_layout()
        path4 = os.path.join(output_dir, "coppelia_clearance.png")
        fig4.savefig(path4, dpi=180, bbox_inches="tight")
        saved.append(path4)
        print(f"✓ {path4}")

    # ------------------------------------------------------------------
    # Figura 5: Timeline por rodada (útil para 1 ou 2 presets)
    # ------------------------------------------------------------------
    saved += _plot_per_run(runs_by_preset, presets, colors, output_dir, show)

    if show:
        plt.show()
    else:
        plt.close("all")

    return saved


def _plot_per_run(runs_by_preset, presets, colors, output_dir, show):
    """Gráfico de barras por rodada individual para cada preset."""
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        return []

    saved = []
    for preset, color in zip(presets, colors):
        runs = runs_by_preset[preset]
        n = len(runs)
        if n == 0:
            continue

        plan_times = [r["planning_time_s"] for r in runs]
        exec_times = [r["execution_time_s"] for r in runs]
        successes  = [r["success"] for r in runs]
        bar_colors = [color if ok else "#cccccc" for ok in successes]

        fig, axes = plt.subplots(2, 1, figsize=(max(8, n * 0.7 + 2), 7), sharex=True)
        label = _label(preset).replace("\n", " ")
        fig.suptitle(f"Resultados por Rodada — {label}", fontsize=12, fontweight="bold")

        x = np.arange(1, n + 1)

        # ── Planejamento ──────────────────────────────────────────────
        bars0 = axes[0].bar(x, plan_times, color=bar_colors, width=0.6, zorder=3)
        valid_plan = [t for t in plan_times if not math.isnan(t)]
        if valid_plan:
            mean_p = sum(valid_plan) / len(valid_plan)
            axes[0].axhline(mean_p, color=color, linewidth=1.8,
                            linestyle="--", label=f"Média: {mean_p:.2f}s")
            axes[0].legend(fontsize=8)
        axes[0].set_ylabel("Tempo (s)", fontsize=9)
        axes[0].set_title("Tempo de Planejamento", fontsize=10)
        axes[0].grid(axis="y", alpha=0.3, zorder=0)
        axes[0].spines[["top", "right"]].set_visible(False)
        for bar, val in zip(bars0, plan_times):
            if not math.isnan(val):
                axes[0].text(bar.get_x() + bar.get_width() / 2,
                             bar.get_height() * 1.02,
                             f"{val:.2f}", ha="center", va="bottom", fontsize=7)

        # ── Execução ──────────────────────────────────────────────────
        bars1 = axes[1].bar(x, exec_times, color=bar_colors, width=0.6, zorder=3)
        ok_exec = [t for t, ok in zip(exec_times, successes)
                   if ok and not math.isnan(t)]
        if ok_exec:
            mean_e = sum(ok_exec) / len(ok_exec)
            axes[1].axhline(mean_e, color=color, linewidth=1.8,
                            linestyle="--", label=f"Média (sucesso): {mean_e:.2f}s")
            axes[1].legend(fontsize=8)
        axes[1].set_ylabel("Tempo (s)", fontsize=9)
        axes[1].set_title("Tempo de Execução", fontsize=10)
        axes[1].set_xlabel("Rodada", fontsize=9)
        axes[1].set_xticks(x)
        axes[1].grid(axis="y", alpha=0.3, zorder=0)
        axes[1].spines[["top", "right"]].set_visible(False)
        for bar, val, ok in zip(bars1, exec_times, successes):
            if not math.isnan(val) and val > 0:
                axes[1].text(bar.get_x() + bar.get_width() / 2,
                             bar.get_height() * 1.02,
                             f"{val:.1f}", ha="center", va="bottom", fontsize=7)

        # Legenda de cores
        from matplotlib.patches import Patch
        legend_elems = [
            Patch(facecolor=color,   label="Sucesso"),
            Patch(facecolor="#cccccc", label="Falhou"),
        ]
        axes[0].legend(handles=legend_elems + axes[0].get_legend_handles_labels()[0],
                       fontsize=8, loc="upper right")

        # Taxa de sucesso no título
        rate = sum(successes) / n * 100
        fig.text(0.5, 0.01,
                 f"Taxa de sucesso: {sum(successes)}/{n} ({rate:.0f}%)",
                 ha="center", fontsize=9, color="#444")

        plt.tight_layout(rect=[0, 0.03, 1, 0.97])
        safe_name = preset.replace("/", "_")
        path = os.path.join(output_dir, f"per_run_{safe_name}.png")
        fig.savefig(path, dpi=180, bbox_inches="tight")
        saved.append(path)
        print(f"✓ {path}")
        if not show:
            plt.close(fig)

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

    # Verifica se há dados de clearance para incluir coluna extra
    has_clearance = any(
        not math.isnan(all_stats[p].get("min_clearance_m_mean", float("nan")))
        for p in presets
    )
    col_spec = "lrrrrr" if has_clearance else "lrrrr"
    header_extra = r" & $c_\text{min}$ (m)" if has_clearance else ""

    print()
    print(r"% --- Tabela gerada por plot_coppelia_results.py ---")
    print(r"\begin{table}[ht]")
    print(r"\centering")
    print(r"\caption{Comparação de desempenho no CoppeliaSim}")
    print(r"\label{tab:coppelia_results}")
    print(f"\\begin{{tabular}}{{{col_spec}}}")
    print(r"\toprule")
    print(
        r"Algoritmo & Sucesso & $t_\text{plan}$ (s) & $t_\text{exec}$ (s) "
        + r"& Erro final (m)" + header_extra + r" \\"
    )
    print(r"\midrule")
    for p in presets:
        s = all_stats[p]
        label = _label(p).replace("\n", " ")
        succ  = f"{s['n_success']}/{s['n']} ({s['success_rate']*100:.0f}\\%)"
        tp    = _fmt(s["planning_time_s_mean"], s["planning_time_s_std"])
        te    = _fmt(s["execution_time_s_mean"], s["execution_time_s_std"])
        fe    = _fmt(s["final_error_m_mean"],    s["final_error_m_std"])
        clearance_col = ""
        if has_clearance:
            cl = _fmt(s.get("min_clearance_m_mean", float("nan")),
                      s.get("min_clearance_m_std",  float("nan")))
            clearance_col = f" & ${cl}$"
        print(f"  {label:<30} & {succ:<14} & ${tp}$ & ${te}$ & ${fe}${clearance_col} \\\\")
    print(r"\bottomrule")
    print(r"\end{tabular}")
    print(r"\end{table}")
    print()


# ---------------------------------------------------------------------------
# Tabela texto
# ---------------------------------------------------------------------------

def print_text_table(all_stats: dict, presets: list[str]):
    sep = "=" * 118
    print(f"\n{sep}")
    print("ESTATÍSTICAS — CoppeliaSim")
    print(sep)
    print(
        f"{'Preset':<28} {'n':<5} {'Sucesso':<10} "
        f"{'t_plan(s)':<16} {'t_exec(s)':<16} {'err(m)':<14} "
        f"{'clearance(m)':<14} {'waypts':<8} {'skip':<6}"
    )
    print("-" * 118)

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
            f"{_f(s.get('min_clearance_m_mean', float('nan')), s.get('min_clearance_m_std', float('nan'))):<14} "
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
