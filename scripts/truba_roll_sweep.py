#!/usr/bin/env python3
"""Wide Log-Space / Order-of-Magnitude Sweep for Roll (LatEC) on TRUBA.

Sweeps 28-56 candidates spanning orders of magnitude of K_sc_roll, K_phas_roll,
K_ton_roll, p_sc_roll, p_phas_roll, and Kp_prop_roll on exact 15.8s LatEC,
prints a sorted table with Gain/Phase loss & rv, and publishes the best to
logs/phase_a3_v3/best_roll_10d.json so queued Roll CMA-ES warm-starts from it.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in (_REPO_ROOT, _THIS_DIR, os.path.join(_REPO_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import scripts.truba_phase_a3_exact_roi_cmaes as a3ex


def build_sweep_candidates(n_cand: int = 28, seed: int = 20261190):
    rng = np.random.default_rng(seed)
    _, base_p = a3ex.load_base_params()
    shared = a3ex.merge_shared_params(base_p, exclude_axis="roll")

    # 12 structured anchors spanning pure-soft (fn~1.0Hz) through moderate to stiff
    anchors = [
        {"K_sc_roll": 0.05, "p_sc_roll": 3.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.0, "Kp_prop_roll": 0.0,  "Kp_proprioception_roll": 0.0,   "Kp_buckle_roll": 0.2, "Kd_buckle_roll": 0.03},
        {"K_sc_roll": 0.15, "p_sc_roll": 3.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.0, "Kp_prop_roll": 0.05, "Kp_proprioception_roll": 0.002, "Kp_buckle_roll": 0.6, "Kd_buckle_roll": 0.05},
        {"K_sc_roll": 0.30, "p_sc_roll": 4.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.1, "Kp_prop_roll": 0.10, "Kp_proprioception_roll": 0.003, "Kp_buckle_roll": 1.2, "Kd_buckle_roll": 0.08},
        {"K_sc_roll": 0.05, "p_sc_roll": 3.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.0, "Kp_prop_roll": 0.0,  "Kp_proprioception_roll": 0.0,   "Kp_buckle_roll": 3.0, "Kd_buckle_roll": 0.05},
        {"K_sc_roll": 0.20, "p_sc_roll": 5.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.2, "Kp_prop_roll": 0.15, "Kp_proprioception_roll": 0.005, "Kp_buckle_roll": 2.5, "Kd_buckle_roll": 0.08},
        {"K_sc_roll": 0.50, "p_sc_roll": 5.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.3, "Kp_prop_roll": 0.20, "Kp_proprioception_roll": 0.005, "Kp_buckle_roll": 1.8, "Kd_buckle_roll": 0.10},
        {"K_sc_roll": 0.40, "p_sc_roll": 2.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.0, "Kp_prop_roll": 0.05, "Kp_proprioception_roll": 0.002, "Kp_buckle_roll": 0.3, "Kd_buckle_roll": 0.04},
        {"K_sc_roll": 0.80, "p_sc_roll": 2.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.2, "Kp_prop_roll": 0.20, "Kp_proprioception_roll": 0.005, "Kp_buckle_roll": 1.0, "Kd_buckle_roll": 0.08},
        {"K_sc_roll": 0.30, "p_sc_roll": 8.0,  "K_phas_roll": 0.1, "p_phas_roll": 4.0, "K_ton_roll": 0.2, "Kp_prop_roll": 0.10, "Kp_proprioception_roll": 0.004, "Kp_buckle_roll": 1.5, "Kd_buckle_roll": 0.08},
        {"K_sc_roll": 1.00, "p_sc_roll": 6.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.5, "Kp_prop_roll": 0.30, "Kp_proprioception_roll": 0.008, "Kp_buckle_roll": 2.0, "Kd_buckle_roll": 0.12},
        {"K_sc_roll": 1.50, "p_sc_roll": 8.0,  "K_phas_roll": 0.0, "p_phas_roll": 4.0, "K_ton_roll": 0.5, "Kp_prop_roll": 0.50, "Kp_proprioception_roll": 0.010, "Kp_buckle_roll": 2.5, "Kd_buckle_roll": 0.15},
        {"K_sc_roll": 0.20, "p_sc_roll": 15.0, "K_phas_roll": 0.2, "p_phas_roll": 6.0, "K_ton_roll": 0.5, "Kp_prop_roll": 0.15, "Kp_proprioception_roll": 0.004, "Kp_buckle_roll": 4.0, "Kd_buckle_roll": 0.06},
    ]

    # Log-uniform / linear ranges spanning orders of magnitude (including soft low-gain regime!)
    specs = [
        ("K_sc_roll",              0.03,   3.0, True),
        ("z_sc_roll",              0.001, 0.050, True),
        ("p_sc_roll",              0.50,  40.0, True),
        ("K_phas_roll",            0.005, 1.20, True),
        ("z_phas_roll",            0.001, 0.040, True),
        ("p_phas_roll",            0.80,  25.0, True),
        ("K_ton_roll",             0.02,  3.00, True),
        ("p_ton_roll",             2.00,  30.0, True),
        ("Ki_ton_roll",            0.00,  1.20, False),
        ("Kp_prop_roll",           0.02,  2.00, True),
        ("Kp_proprioception_roll", 0.001, 0.040, True),
        ("Kp_buckle_roll",         0.10,  5.00, True),
        ("Kd_buckle_roll",         0.02,  0.40, True),
        ("tau_lead_pole_roll",     0.004, 0.028, False),
    ]

    n_lhs = max(0, n_cand - len(anchors))
    lhs_cols = {}
    for name, lo, hi, is_log in specs:
        perm = rng.permutation(n_lhs)
        u = (perm + rng.uniform(0.0, 1.0, size=n_lhs)) / max(1, n_lhs)
        if is_log:
            vals = np.exp(np.log(lo) + u * (np.log(hi) - np.log(lo)))
        else:
            vals = lo + u * (hi - lo)
        lhs_cols[name] = vals

    cands = []
    for idx in range(n_cand):
        ap = {
            "K_sc_roll": 4.20, "z_sc_roll": 0.014, "p_sc_roll": 5.5,
            "K_phas_roll": 0.50, "z_phas_roll": 0.010, "p_phas_roll": 8.0,
            "K_ton_roll": 0.35, "p_ton_roll": 10.0, "Ki_ton_roll": 0.02,
            "Kp_prop_roll": 0.30, "Kp_proprioception_roll": 0.014,
            "Kp_buckle_roll": 1.00, "Kd_buckle_roll": 0.32, "tau_lead_pole_roll": 0.014,
        }
        if idx < len(anchors):
            ap.update(anchors[idx])
        else:
            j = idx - len(anchors)
            for name, _, _, _ in specs:
                ap[name] = float(lhs_cols[name][j])
        full_p = dict(shared)
        full_p.update(ap)
        cands.append((idx, ap, full_p))
    return cands


def _eval(item):
    idx, ap, full_p = item
    res = a3ex.run_bode_exact(full_p, "LatEC", wall_limit_s=2400.0)
    scored = a3ex.score_roll_or_yaw(idx, ap, res, "LatEC")
    scored["gain_loss"] = res.get("gain_loss", 0.0)
    scored["phase_loss"] = res.get("phase_loss", 0.0)
    return scored


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=28)
    ap.add_argument("--workers", type=int, default=28)
    ap.add_argument("--seed", type=int, default=20261190)
    args = ap.parse_args()

    t0 = time.time()
    cands = build_sweep_candidates(args.n, args.seed)
    print(f"[ROLL-SWEEP] Evaluating {len(cands)} log-space LatEC candidates on {args.workers} workers...", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_eval, cands))

    results.sort(key=lambda r: float(r["loss"]))
    print("\n=== SORTED ROLL SWEEP RESULTS (Top 15) ===", flush=True)
    for r in results[:15]:
        p = r["axis_params"]
        print(
            f"#{r['idx']:02d} ab={int(r['aborted'])} | LatEC={r['total_score']:8.1f} cls={r.get('classic_score',0):8.1f} "
            f"G={r.get('gain_loss',0):6.0f} P={r.get('phase_loss',0):6.0f} rv={r.get('corr_vel',0):+.4f} hf={r.get('hf_rms',0):.2f} | "
            f"Ksc={p['K_sc_roll']:.2f} psc={p['p_sc_roll']:.1f} Kph={p['K_phas_roll']:.2f} pph={p['p_phas_roll']:.1f} "
            f"Kton={p['K_ton_roll']:.2f} Kpr={p['Kp_prop_roll']:.2f}",
            flush=True,
        )

    best = results[0]
    pub = a3ex.publish_axis_best(a3ex.BEST_ROLL_JSON, best, "roll_sweep", 0, "loss")
    a3ex.atomic_write_json(
        os.path.join(a3ex.STATE_DIR, "roll_sweep_results.json"),
        {"wall_s": round(time.time() - t0, 1), "results": results},
    )
    print(f"[ROLL-SWEEP] Done in {time.time()-t0:.1f}s | Best LatEC={best['total_score']:.1f} | published={pub}", flush=True)


if __name__ == "__main__":
    main()
