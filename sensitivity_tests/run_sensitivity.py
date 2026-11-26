#!/usr/bin/env python3
"""Full-Model Block-by-Block Bode Sensitivity & Internal-Probe Suite (reusing unit_tests probe culture).

Runs 100% FULL MODEL (every biological subsystem ON) One-At-A-Time (OAT) sweeps on APEO, LatEC, or YawMA,
records internal C++ pathway torques (VCR, CCR, Buckle, QP) and upper/lower joint split (q1/q2) at every
frequency, ranks parameter sensitivities, and renders Happee-2017-Fig.5-style Bode sensitivity & torque-budget plots.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _env_k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_env_k, "1")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in (_REPO_ROOT, _THIS_DIR, os.path.join(_REPO_ROOT, "scripts"), os.path.join(_REPO_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sensitivity_tests.bode_xray import print_xray, run_bode_probed

RESULTS_DIR = os.path.join(_THIS_DIR, "results")

# Default full-model baselines (all subsystems ON, anatomical vertebral inertias, ISO 2631)
BASELINES = {
    "APEO": {
        "K_sc": 0.10, "z_sc": 0.010, "p_sc": 25.0,
        "K_phas": 0.05, "z_phas": 0.010, "p_phas": 4.0,
        "K_ton": 0.25, "p_ton": 3.0, "Ki_ton": 0.05,
        "Kp_prop_pitch": 0.05, "Kp_proprioception": 0.002,
        "Kp_buckle": 0.80, "Kd_buckle": 0.05,
        "vcr_upper_share_pitch": 0.44,
        "srs_k_srs": 0.50, "srs_thix_min": 0.12,
    },
    "LatEC": {
        "K_sc_roll": 0.06, "z_sc_roll": 0.012, "p_sc_roll": 6.0,
        "K_phas_roll": 0.02, "z_phas_roll": 0.010, "p_phas_roll": 4.0,
        "K_ton_roll": 0.25, "p_ton_roll": 2.5, "Ki_ton_roll": 0.02,
        "Kp_prop_roll": 0.03, "Kp_proprioception_roll": 0.002,
        "Kp_buckle_roll": 0.15, "Kd_buckle_roll": 0.03,
        "srs_k_srs": 0.50, "srs_thix_min": 0.12,
    },
    "YawMA": {
        "K_sc_yaw": 0.65, "z_sc_yaw": 0.008, "p_sc_yaw": 20.0,
        "K_ton_yaw": 0.80, "p_ton_yaw": 10.0,
        "Kp_prop_yaw": 0.25, "Kp_proprioception_yaw": 0.005,
        "Kp_buckle_yaw": 0.70, "Kd_buckle_yaw": 0.05,
        "srs_k_srs": 0.50, "srs_thix_min": 0.12,
    },
}

# Block-grouped OAT probes: (block_name, param_key, lo_val, hi_val)
# Special keys: "__bushing_k__" and "__bushing_c__" scale disc bushings in-memory to quantify plant stiffness sensitivity!
SWEEP_SPECS = {
    "APEO": [
        ("B14_Canal",  "K_sc",                  0.03,  0.40),
        ("B14_Canal",  "p_sc",                  5.00, 80.00),
        ("B14_Phasic", "K_phas",                0.01,  0.40),
        ("B14_Tonic",  "K_ton",                 0.05,  1.20),
        ("B14_Tonic",  "p_ton",                 1.00, 15.00),
        ("B19_CCR",    "Kp_prop_pitch",         0.01,  0.40),
        ("B20_Buckle", "Kp_buckle",             0.15,  2.50),
        ("B20_Buckle", "Kd_buckle",             0.01,  0.20),
        ("B20_Split",  "vcr_upper_share_pitch", 0.15,  0.85),
        ("B03_SRS",    "srs_k_srs",             0.10,  3.00),
        ("B02_DiscK",  "__bushing_k__",         0.30,  1.50),
        ("B02_DiscC",  "__bushing_c__",         0.30,  1.50),
    ],
    "LatEC": [
        ("B14_Canal",  "K_sc_roll",             0.02,  0.30),
        ("B14_Canal",  "p_sc_roll",             1.50, 20.00),
        ("B14_Phasic", "K_phas_roll",           0.005, 0.20),
        ("B14_Tonic",  "K_ton_roll",            0.05,  0.80),
        ("B14_Tonic",  "p_ton_roll",            0.80, 12.00),
        ("B19_CCR",    "Kp_prop_roll",          0.01,  0.25),
        ("B20_Buckle", "Kp_buckle_roll",        0.04,  1.20),
        ("B20_Buckle", "Kd_buckle_roll",        0.01,  0.15),
        ("B03_SRS",    "srs_k_srs",             0.10,  3.00),
        ("B02_DiscK",  "__bushing_k__",         0.30,  1.50),
        ("B02_DiscC",  "__bushing_c__",         0.30,  1.50),
    ],
    "YawMA": [
        ("B14_Canal",  "K_sc_yaw",              0.20,  1.80),
        ("B14_Canal",  "p_sc_yaw",              3.00, 40.00),
        ("B14_Canal",  "z_sc_yaw",              0.002, 0.030),
        ("B14_Tonic",  "K_ton_yaw",             0.15,  3.00),
        ("B19_CCR",    "Kp_prop_yaw",           0.05,  1.20),
        ("B20_Buckle", "Kp_buckle_yaw",         0.20,  2.00),
        ("B20_Buckle", "Kd_buckle_yaw",         0.015, 0.15),
        ("B03_SRS",    "srs_k_srs",             0.10,  3.00),
        ("B02_DiscK",  "__bushing_k__",         0.30,  1.50),
        ("B02_DiscC",  "__bushing_c__",         0.30,  1.50),
    ],
}


def _eval_task(item):
    tag, mode, ov, bk, bc, exact_15s = item
    res = run_bode_probed(mode, ov, exact_15s=exact_15s, bushing_k_scale=bk, bushing_c_scale=bc)
    return tag, res


def plot_sensitivity_grid(axis: str, base_res: dict, groups: list, out_png: str):
    n = len(groups)
    ncols = min(4, n)
    nrows = (n + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows * 2, ncols, figsize=(4.2 * ncols, 3.0 * nrows * 2), dpi=130, sharex="col")
    if ncols == 1:
        axes = np.asarray(axes).reshape(nrows * 2, 1)

    freqs = [r["f_hz"] for r in base_res["per_freq"]]
    tgt_g = [r["tgt_g"] for r in base_res["per_freq"]]
    tgt_p = [r["tgt_ph"] for r in base_res["per_freq"]]
    base_g = [r["sim_g"] for r in base_res["per_freq"]]
    base_p = [r["sim_ph"] for r in base_res["per_freq"]]

    for idx, grp in enumerate(groups):
        r_blk = idx // ncols
        c_blk = idx % ncols
        ax_g = axes[r_blk * 2, c_blk]
        ax_p = axes[r_blk * 2 + 1, c_blk]

        ax_g.loglog(freqs, tgt_g, "k-o", lw=2.0, ms=4, label="Target")
        ax_g.loglog(freqs, base_g, "b--s", lw=1.6, ms=3.5, label=f"Base ({base_res['total_score']:.0f})")
        ax_p.semilogx(freqs, tgt_p, "k-o", lw=2.0, ms=4)
        ax_p.semilogx(freqs, base_p, "b--s", lw=1.6, ms=3.5)

        for tag_key, color, ls in (("lo", "#2ca02c", "-"), ("hi", "#d62728", "-")):
            res = grp[tag_key]
            if res and not res.get("aborted"):
                sg = [r["sim_g"] for r in res["per_freq"]]
                sp = []
                for r in res["per_freq"]:
                    # unwrap close to target for clean plotting
                    pe = ((r["sim_ph"] - r["tgt_ph"] + 180.0) % 360.0) - 180.0
                    sp.append(r["tgt_ph"] + pe)
                ax_g.loglog(freqs, sg, color=color, ls=ls, lw=1.6, label=f"{tag_key}={grp[tag_key+'_val']} ({res['total_score']:.0f})")
                ax_p.semilogx(freqs, sp, color=color, ls=ls, lw=1.6)

        ax_g.set_title(f"{grp['block']}: {grp['param']}", fontsize=9, fontweight="bold")
        ax_g.grid(True, which="both", alpha=0.3)
        ax_p.grid(True, which="both", alpha=0.3)
        ax_g.legend(fontsize=7, loc="best")
        if c_blk == 0:
            ax_g.set_ylabel("Gain")
            ax_p.set_ylabel("Phase (deg)")
        ax_p.set_xlabel("Freq (Hz)")

    # Hide unused slots
    for idx in range(n, nrows * ncols):
        r_blk, c_blk = idx // ncols, idx % ncols
        axes[r_blk * 2, c_blk].axis("off")
        axes[r_blk * 2 + 1, c_blk].axis("off")

    fig.suptitle(f"Full-Model Block Sensitivity Grid — {axis} (All Subsystems ON)", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_png)
    plt.close(fig)


def plot_torque_budget(axis: str, res: dict, out_png: str):
    if res.get("aborted"):
        return
    pf = res["per_freq"]
    freqs = [f"{r['f_hz']:.2f}" for r in pf]
    x = np.arange(len(freqs))
    w = 0.20
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5), dpi=130)
    ax1.bar(x - 1.5 * w, [r["tau_vcr_g"] for r in pf], w, label="VCR (dbg_vcr)")
    ax1.bar(x - 0.5 * w, [r["tau_ccr_g"] for r in pf], w, label="CCR (dbg_ccr)")
    ax1.bar(x + 0.5 * w, [r["tau_bkl_g"] for r in pf], w, label="Anti-Buckle")
    ax1.bar(x + 1.5 * w, [r["tau_qp_g"] for r in pf], w, label="QP Delivered")
    ax1.set_xticks(x); ax1.set_xticklabels(freqs)
    ax1.set_xlabel("Frequency (Hz)"); ax1.set_ylabel("Harmonic Torque Gain (Nm/unit)")
    ax1.set_title(f"{axis} Internal Torque Budget (TOT={res['total_score']:.0f}, thix={res['thix_mean']:.2f})")
    ax1.legend(fontsize=8); ax1.grid(True, axis="y", alpha=0.3)

    ax2.bar(x - 0.2, [r["q1_g"] for r in pf], 0.35, label="Upper Joint q1")
    ax2.bar(x + 0.2, [r["q2_g"] for r in pf], 0.35, label="Lower Joint q2")
    ax2.plot(x, [r["tgt_g"] for r in pf], "k-o", lw=2, label="Target Total")
    ax2.plot(x, [r["sim_g"] for r in pf], "b--s", lw=2, label="Sim Total (q1+q2)")
    ax2.set_xticks(x); ax2.set_xticklabels(freqs)
    ax2.set_xlabel("Frequency (Hz)"); ax2.set_ylabel("Angular Gain")
    ax2.set_title(f"{axis} Upper vs Lower Neck Joint Split (S-Bend X-Ray)")
    ax2.legend(fontsize=8); ax2.grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=["APEO", "LatEC", "YawMA"], required=True)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--exact", action="store_true", help="Run exact 15.8s Bode instead of 10s fast Bode")
    ap.add_argument("--params-json", default="", help="Optional JSON file with axis_params or params baseline")
    ap.add_argument("--subset", default="", help="Comma-separated param names to sweep (default: all)")
    args = ap.parse_args()

    os.makedirs(RESULTS_DIR, exist_ok=True)
    base_ov = dict(BASELINES[args.axis])
    if args.params_json and os.path.exists(args.params_json):
        doc = json.load(open(args.params_json, "r", encoding="utf-8"))
        base_ov.update(doc.get("params", doc.get("axis_params", doc)))

    specs = list(SWEEP_SPECS[args.axis])
    if args.subset:
        keep = {s.strip() for s in args.subset.split(",") if s.strip()}
        specs = [s for s in specs if s[1] in keep]

    tasks = [("BASE", args.axis, dict(base_ov), 1.0, 1.0, args.exact)]
    for blk, pname, lo, hi in specs:
        for side, val in (("lo", lo), ("hi", hi)):
            ov = dict(base_ov)
            bk, bc = 1.0, 1.0
            if pname == "__bushing_k__":
                bk = val
            elif pname == "__bushing_c__":
                bc = val
            else:
                ov[pname] = val
            tasks.append((f"{blk}|{pname}|{side}|{val}", args.axis, ov, bk, bc, args.exact))

    t0 = time.time()
    print(f"[SENSITIVITY:{args.axis}] Running {len(tasks)} full-model probed Bode evals on {args.workers} workers (exact={args.exact})...", flush=True)
    results_map = {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futs = {pool.submit(_eval_task, tk): tk[0] for tk in tasks}
        for fut in as_completed(futs):
            tag, res = fut.result()
            results_map[tag] = res
            print(f"  done [{len(results_map):02d}/{len(tasks):02d}] {tag:34s} | TOT={res.get('total_score', 1e6):8.1f} cls={res.get('classic_score', 0):7.1f} G={res.get('gain_loss', 0):6.0f} P={res.get('phase_loss', 0):6.0f} rv={res.get('rv', 0):+.3f} hf={res.get('hf_rms', 0):.2f} ({res.get('wall_s', 0):.0f}s)", flush=True)

    base_res = results_map["BASE"]
    print_xray(f"BASELINE ({args.axis})", base_res)

    groups = []
    ranking = []
    for blk, pname, lo, hi in specs:
        r_lo = results_map[f"{blk}|{pname}|lo|{lo}"]
        r_hi = results_map[f"{blk}|{pname}|hi|{hi}"]
        groups.append({"block": blk, "param": pname, "lo_val": lo, "hi_val": hi, "lo": r_lo, "hi": r_hi})
        b_tot = base_res["total_score"]
        d_lo = r_lo["total_score"] - b_tot
        d_hi = r_hi["total_score"] - b_tot
        best_dir = "LOWER" if d_lo < min(0.0, d_hi) else ("HIGHER" if d_hi < min(0.0, d_lo) else "STAY")
        ranking.append({
            "block": blk, "param": pname,
            "base_val": 1.0 if pname.startswith("__") else base_ov.get(pname, None),
            "lo_val": lo, "lo_tot": r_lo["total_score"], "lo_cls": r_lo.get("classic_score", 0), "d_lo": round(d_lo, 1),
            "hi_val": hi, "hi_tot": r_hi["total_score"], "hi_cls": r_hi.get("classic_score", 0), "d_hi": round(d_hi, 1),
            "best_gain_pts": round(min(d_lo, d_hi), 1), "direction": best_dir,
        })

    ranking.sort(key=lambda x: x["best_gain_pts"])
    print(f"\n=== RANKED BLOCK SENSITIVITY TABLE ({args.axis}, Base TOT={base_res['total_score']:.1f}) ===")
    print("  Block       | Parameter             |   Base |    Low ->     TOT (   dTOT) |   High ->     TOT (   dTOT) | Move")
    for r in ranking:
        bv = f"{r['base_val']:.3g}" if r["base_val"] is not None else "  n/a"
        print(
            f"  {r['block']:11s} | {r['param']:21s} | {bv:>6s} | "
            f"{r['lo_val']:6.3g} -> {r['lo_tot']:7.1f} ({r['d_lo']:+7.1f}) | "
            f"{r['hi_val']:6.3g} -> {r['hi_tot']:7.1f} ({r['d_hi']:+7.1f}) | {r['direction']}"
        )

    out_json = os.path.join(RESULTS_DIR, f"sensitivity_{args.axis}.json")
    out_grid = os.path.join(RESULTS_DIR, f"sensitivity_{args.axis}_grid.png")
    out_tb = os.path.join(RESULTS_DIR, f"torque_budget_{args.axis}.png")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump({"axis": args.axis, "wall_s": round(time.time() - t0, 1), "baseline_params": base_ov, "baseline": base_res, "ranking": ranking, "all_results": results_map}, f, indent=2)
    plot_sensitivity_grid(args.axis, base_res, groups, out_grid)
    plot_torque_budget(args.axis, base_res, out_tb)
    print(f"\nSaved:\n  JSON: {out_json}\n  GRID: {out_grid}\n  TORQ: {out_tb}\nTotal wall={time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
