#!/usr/bin/env python3
"""Fine 1D Urgency (u) Sweep to Center Both Zheng et al. (2021) / NBDL 2g and 3g Corridors."""

import json
import os
import shutil
import sys
import time
from concurrent.futures import ProcessPoolExecutor

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from run_zheng_2g_3g_urgency_sweep import (
    ART_DIR,
    G_LEVELS,
    T_IMPACT,
    T_SETTLE,
    T_TOTAL,
    compute_engagement,
    run_single_case,
)

OUT_JSON = os.path.join(_ROOT, "logs", "zheng_2g_3g_fine_corridor_sweep.json")
OUT_PNG = os.path.join(_ROOT, "results", "zheng_2g_3g_fine_corridor_sweep.png")

FINE_URGENCY_LIST = [0.010, 0.015, 0.020, 0.025, 0.030, 0.035]
CORRIDORS = {
    2.0: {"lo": 33.0, "hi": 40.0, "center": 36.5, "label": "NBDL 2g corridor (33–40°)"},
    3.0: {"lo": 40.0, "hi": 47.0, "center": 43.5, "label": "NBDL 3g corridor (40–47°)"},
}
SPLEN_REFS = {
    2.0: {"val": 0.50, "label": "Zheng Fig. 6 2g splen. cap. peak (~0.50)"},
    3.0: {"val": 0.60, "label": "Zheng Fig. 6 3g splen. cap. peak (~0.60)"},
}


def interp_exact_u(u_vals: list[float], pk_vals: list[float], target: float):
    """Linearly interpolate u where monotonically decreasing pk(u) crosses target."""
    for i in range(len(u_vals) - 1):
        y0, y1 = pk_vals[i], pk_vals[i + 1]
        if (y0 >= target >= y1) or (y0 <= target <= y1):
            if abs(y1 - y0) < 1e-12:
                return float(u_vals[i])
            frac = (target - y0) / (y1 - y0)
            return float(u_vals[i] + frac * (u_vals[i + 1] - u_vals[i]))
    return None


def make_fine_comparison_plot(results_by_key: dict, u_star: float):
    fig, axes = plt.subplots(2, 2, figsize=(14.5, 9.5), dpi=180, sharex=True)
    cmap = plt.get_cmap("viridis")
    non_star_indices = [i for i, u in enumerate(FINE_URGENCY_LIST) if abs(u - u_star) > 1e-9]
    color_map = {}
    for rank, idx in enumerate(non_star_indices):
        color_map[FINE_URGENCY_LIST[idx]] = cmap(0.10 + 0.78 * (rank / max(1, len(non_star_indices) - 1)))
    color_map[u_star] = "#dc2626"

    for col, g_level in enumerate(G_LEVELS):
        ax_ang = axes[0, col]
        ax_act = axes[1, col]

        corr = CORRIDORS[g_level]
        ax_ang.axhspan(corr["lo"], corr["hi"], color="#f59e0b", alpha=0.18, label=corr["label"])
        ax_ang.axhline(corr["lo"], color="#d97706", linestyle=":", linewidth=1.0, alpha=0.8)
        ax_ang.axhline(corr["hi"], color="#d97706", linestyle=":", linewidth=1.0, alpha=0.8)
        ax_ang.axhline(corr["center"], color="#b45309", linestyle="--", linewidth=1.3, alpha=0.9, label=f"Center ({corr['center']:.1f}°)")

        sref = SPLEN_REFS[g_level]
        ax_act.axhline(sref["val"], color="#dc2626", linestyle="--", linewidth=1.3, alpha=0.9, label=sref["label"])

        ax_ang.axvspan(37.5, 212.5, color="#cbd5e1", alpha=0.18)
        ax_act.axvspan(37.5, 212.5, color="#cbd5e1", alpha=0.18)

        for u in FINE_URGENCY_LIST:
            r = results_by_key[(g_level, u)]
            is_star = abs(u - u_star) <= 1e-9
            c = color_map[u]
            lw = 2.8 if is_star else 1.5
            zord = 5 if is_star else 3
            tag = " ★ u*" if is_star else ""
            lbl = f"u={u:.3f} (e={r['engagement']:.3f}) | pk={r['peak_flex_deg']:.1f}°{tag}"
            ax_ang.plot(r["t_imp_ms"], r["pitch_rel_deg"], color=c, linewidth=lw, zorder=zord, label=lbl)
            ax_act.plot(
                r["t_imp_ms"],
                r["splen_cap_act"],
                color=c,
                linewidth=lw,
                zorder=zord,
                label=f"u={u:.3f} (e={r['engagement']:.3f}) | pk={r['peak_splen_cap']:.2f}{tag}",
            )

        ax_ang.set_title(f"{int(g_level)}g Frontal Sled Pulse — Head C.G. Sagittal Rotation vs T1", fontsize=11.5, fontweight="bold")
        ax_ang.set_ylabel("Sagittal Flexion Rel. to Settle (°)", fontsize=10)
        ax_ang.grid(True, linestyle=":", alpha=0.55)
        ax_ang.set_xlim(0.0, 350.0)
        ax_ang.legend(loc="upper left", fontsize=7.6, framealpha=0.92)

        ax_act.set_title(f"{int(g_level)}g Frontal Sled Pulse — Splenius Capitis Activation", fontsize=11.5, fontweight="bold")
        ax_act.set_xlabel("Time Since Impact Onset (ms) [after 0.35 s quiet pre-settle]", fontsize=10)
        ax_act.set_ylabel("Mean Splenius Capitis Activation", fontsize=10)
        ax_act.grid(True, linestyle=":", alpha=0.55)
        ax_act.set_xlim(0.0, 350.0)
        max_splen = max(results_by_key[(g_level, u)]["peak_splen_cap"] for u in FINE_URGENCY_LIST)
        ax_act.set_ylim(0.0, max(0.75, max_splen + 0.08))
        ax_act.legend(loc="upper right", fontsize=7.6, framealpha=0.92)

    fig.suptitle(
        f"C11 + e(u) Fine Corridor Search (u = 0.010 -> 0.035) | Optimal u* = {u_star:.3f} (e = {compute_engagement(u_star):.4f})",
        fontsize=13.5,
        fontweight="bold",
        y=0.985,
    )
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, dpi=180)
    plt.close(fig)
    if os.path.isdir(ART_DIR):
        shutil.copy2(OUT_PNG, os.path.join(ART_DIR, os.path.basename(OUT_PNG)))


def main():
    t_wall0 = time.time()
    specs = [{"g_level": g, "urgency": u, "sign_x": -1.0} for g in G_LEVELS for u in FINE_URGENCY_LIST]
    print(f"Running {len(specs)} fine-corridor simulations across 6 local performance cores...", flush=True)

    results = []
    with ProcessPoolExecutor(max_workers=6) as pool:
        for res in pool.map(run_single_case, specs):
            results.append(res)
            print(
                f"[DONE] {int(res['g_level'])}g | u={res['urgency']:.3f} (e={res['engagement']:.4f}) | "
                f"pk_flex={res['peak_flex_deg']:+6.2f}° @ {res['t_peak_ms']:5.1f}ms | "
                f"Vmax={res['peak_vel_dps']:5.1f}°/s | splen={res['peak_splen_cap']:.3f} @ {res['t_peak_splen_ms']:5.1f}ms | "
                f"tx_end={res['spine_tx_end_m']:+.4f}m | settle={res['pitch_at_settle_deg']:+.2f}° | "
                f"{res['wall_s']:.1f}s | stop={res['stop_reason']}",
                flush=True,
            )

    results_by_key = {(r["g_level"], r["urgency"]): r for r in results}

    scores = []
    for u in FINE_URGENCY_LIST:
        r2 = results_by_key[(2.0, u)]
        r3 = results_by_key[(3.0, u)]
        err_2g = r2["peak_flex_deg"] - CORRIDORS[2.0]["center"]
        err_3g = r3["peak_flex_deg"] - CORRIDORS[3.0]["center"]
        comb_err = abs(err_2g) + abs(err_3g)
        in_2g = CORRIDORS[2.0]["lo"] <= r2["peak_flex_deg"] <= CORRIDORS[2.0]["hi"]
        in_3g = CORRIDORS[3.0]["lo"] <= r3["peak_flex_deg"] <= CORRIDORS[3.0]["hi"]
        scores.append({
            "urgency": u,
            "engagement": round(compute_engagement(u), 6),
            "pk_2g_deg": r2["peak_flex_deg"],
            "err_2g_deg": round(err_2g, 2),
            "in_2g_corridor": bool(in_2g),
            "pk_3g_deg": r3["peak_flex_deg"],
            "err_3g_deg": round(err_3g, 2),
            "in_3g_corridor": bool(in_3g),
            "combined_abs_err_deg": round(comb_err, 2),
        })

    best_row = min(scores, key=lambda x: x["combined_abs_err_deg"])
    u_star = float(best_row["urgency"])

    u_exact_2g = interp_exact_u(FINE_URGENCY_LIST, [results_by_key[(2.0, u)]["peak_flex_deg"] for u in FINE_URGENCY_LIST], CORRIDORS[2.0]["center"])
    u_exact_3g = interp_exact_u(FINE_URGENCY_LIST, [results_by_key[(3.0, u)]["peak_flex_deg"] for u in FINE_URGENCY_LIST], CORRIDORS[3.0]["center"])

    make_fine_comparison_plot(results_by_key, u_star)

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    summary_rows = [
        {k: v for k, v in r.items() if k not in ("t_imp_ms", "pitch_rel_deg", "pitch_abs_deg", "pitch_vel_dps", "splen_cap_act", "scm_act", "spine_tx_imp_m")}
        for r in results
    ]
    payload = {
        "meta": {
            "controller": "C11 + e(u)",
            "t_settle_s": T_SETTLE,
            "t_impact_s": T_IMPACT,
            "t_total_s": T_TOTAL,
            "pulse_ms": [37.5, 100.0, 212.5],
            "g_levels": G_LEVELS,
            "urgency_list": FINE_URGENCY_LIST,
            "u_star": u_star,
            "e_star": round(compute_engagement(u_star), 6),
            "u_interp_2g_center_36_5": round(u_exact_2g, 5) if u_exact_2g is not None else None,
            "u_interp_3g_center_43_5": round(u_exact_3g, 5) if u_exact_3g is not None else None,
            "total_wall_s": round(time.time() - t_wall0, 2),
        },
        "corridor_scores": scores,
        "summary": summary_rows,
        "cases": results,
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print("\n=== CORRIDOR CENTERING SCOREBOARD ===")
    print(f"{'u':>6} | {'e(u)':>6} | {'2g PkFlex':>9} | {'2g Err':>7} | {'In2g':>4} | {'3g PkFlex':>9} | {'3g Err':>7} | {'In3g':>4} | {'|E2|+|E3|':>9}")
    print("-" * 82)
    for s in scores:
        star = " <-- u*" if abs(s["urgency"] - u_star) <= 1e-9 else ""
        print(
            f"{s['urgency']:6.3f} | {s['engagement']:6.4f} | {s['pk_2g_deg']:+8.2f}° | {s['err_2g_deg']:+6.2f}° | "
            f"{str(s['in_2g_corridor']):>4} | {s['pk_3g_deg']:+8.2f}° | {s['err_3g_deg']:+6.2f}° | "
            f"{str(s['in_3g_corridor']):>4} | {s['combined_abs_err_deg']:8.2f}°{star}"
        )
    print(f"\nInterpolated exact 2g center (36.5°): u = {u_exact_2g:.5f}" if u_exact_2g is not None else "\n2g center outside grid")
    print(f"Interpolated exact 3g center (43.5°): u = {u_exact_3g:.5f}" if u_exact_3g is not None else "3g center outside grid")
    print(f"Best grid u* = {u_star:.3f} (e = {compute_engagement(u_star):.4f})")

    print("\n=== FULL SUMMARY TABLE ===")
    hdr = f"{'G':>3} | {'u':>5} | {'e(u)':>6} | {'Settle':>7} | {'PkFlex':>7} | {'t_pk':>7} | {'Vmax':>8} | {'SplenPk':>7} | {'t_spl':>7} | {'SCM_Pk':>7} | {'MaxAct':>7} | {'tx_end':>8}"
    print(hdr)
    print("-" * len(hdr))
    for g in G_LEVELS:
        for u in FINE_URGENCY_LIST:
            r = results_by_key[(g, u)]
            print(
                f"{int(g):>2}g | {u:5.3f} | {r['engagement']:6.4f} | {r['pitch_at_settle_deg']:+6.2f}° | "
                f"{r['peak_flex_deg']:+6.2f}° | {r['t_peak_ms']:5.1f}ms | "
                f"{r['peak_vel_dps']:6.1f}°/s | {r['peak_splen_cap']:7.4f} | {r['t_peak_splen_ms']:5.1f}ms | "
                f"{r['peak_scm']:7.4f} | {r['max_neck_act']:7.4f} | {r['spine_tx_end_m']:+7.4f}m"
            )
    print(f"\nSaved JSON: {OUT_JSON}")
    print(f"Saved PNG : {OUT_PNG}")
    print(f"Total wall time: {time.time() - t_wall0:.2f} s")


if __name__ == "__main__":
    main()
