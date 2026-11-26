#!/usr/bin/env python3
"""Wide Log-Space / Order-of-Magnitude Sweep for Yaw (YawMA) on TRUBA (anatomical vertebral inertias)."""
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


def build_yaw_sweep_candidates(n_cand: int = 28, seed: int = 20261199):
    rng = np.random.default_rng(seed)
    _, base_p = a3ex.load_base_params()
    shared = a3ex.merge_shared_params(base_p, exclude_axis="yaw")

    anchors = [
        {"K_sc_yaw": 1.55, "z_sc_yaw": 0.022, "p_sc_yaw": 1.25, "K_ton_yaw": 15.7, "p_ton_yaw": 19.5, "Kp_prop_yaw": 12.9, "Kp_proprioception_yaw": 0.042, "Kp_buckle_yaw": 1.23},
        {"K_sc_yaw": 0.60, "z_sc_yaw": 0.015, "p_sc_yaw": 3.00, "K_ton_yaw": 4.0,  "p_ton_yaw": 15.0, "Kp_prop_yaw": 3.0,  "Kp_proprioception_yaw": 0.020, "Kp_buckle_yaw": 1.00},
        {"K_sc_yaw": 1.00, "z_sc_yaw": 0.015, "p_sc_yaw": 5.00, "K_ton_yaw": 8.0,  "p_ton_yaw": 15.0, "Kp_prop_yaw": 6.0,  "Kp_proprioception_yaw": 0.030, "Kp_buckle_yaw": 1.50},
        {"K_sc_yaw": 0.30, "z_sc_yaw": 0.010, "p_sc_yaw": 8.00, "K_ton_yaw": 2.0,  "p_ton_yaw": 10.0, "Kp_prop_yaw": 1.5,  "Kp_proprioception_yaw": 0.015, "Kp_buckle_yaw": 2.00},
        {"K_sc_yaw": 2.50, "z_sc_yaw": 0.020, "p_sc_yaw": 2.00, "K_ton_yaw": 12.0, "p_ton_yaw": 20.0, "Kp_prop_yaw": 10.0, "Kp_proprioception_yaw": 0.050, "Kp_buckle_yaw": 1.00},
        {"K_sc_yaw": 0.80, "z_sc_yaw": 0.010, "p_sc_yaw": 1.50, "K_ton_yaw": 10.0, "p_ton_yaw": 15.0, "Kp_prop_yaw": 8.0,  "Kp_proprioception_yaw": 0.025, "Kp_buckle_yaw": 0.60},
    ]

    specs = [
        ("K_sc_yaw",              0.15,  5.00, True),
        ("z_sc_yaw",              0.002, 0.050, True),
        ("p_sc_yaw",              0.50,  25.0, True),
        ("K_ton_yaw",             0.50,  22.0, True),
        ("p_ton_yaw",             3.00,  35.0, True),
        ("Kp_prop_yaw",           0.30,  20.0, True),
        ("Kp_proprioception_yaw", 0.003, 0.080, True),
        ("Kp_buckle_yaw",         0.20,  4.00, True),
    ]

    n_lhs = max(0, n_cand - len(anchors))
    lhs_cols = {}
    for name, lo, hi, is_log in specs:
        perm = rng.permutation(n_lhs)
        u = (perm + rng.uniform(0.0, 1.0, size=n_lhs)) / max(1, n_lhs)
        vals = np.exp(np.log(lo) + u * (np.log(hi) - np.log(lo))) if is_log else (lo + u * (hi - lo))
        lhs_cols[name] = vals

    cands = []
    for idx in range(n_cand):
        ap = dict(anchors[0])
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
    res = a3ex.run_bode_exact(full_p, "YawMA", wall_limit_s=2400.0)
    scored = a3ex.score_roll_or_yaw(idx, ap, res, "YawMA")
    scored["gain_loss"] = res.get("gain_loss", 0.0)
    scored["phase_loss"] = res.get("phase_loss", 0.0)
    return scored


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=28)
    ap.add_argument("--workers", type=int, default=28)
    ap.add_argument("--seed", type=int, default=20261199)
    args = ap.parse_args()

    t0 = time.time()
    cands = build_yaw_sweep_candidates(args.n, args.seed)
    print(f"[YAW-SWEEP] Evaluating {len(cands)} log-space YawMA candidates on {args.workers} workers...", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_eval, cands))

    results.sort(key=lambda r: float(r["loss"]))
    print("\n=== SORTED YAW SWEEP RESULTS (Top 15) ===", flush=True)
    for r in results[:15]:
        p = r["axis_params"]
        print(
            f"#{r['idx']:02d} ab={int(r['aborted'])} | YawMA={r['total_score']:8.1f} cls={r.get('classic_score',0):8.1f} "
            f"G={r.get('gain_loss',0):6.0f} P={r.get('phase_loss',0):6.0f} rv={r.get('corr_vel',0):+.4f} hf={r.get('hf_rms',0):.2f} | "
            f"Ksc={p['K_sc_yaw']:.2f} psc={p['p_sc_yaw']:.1f} Kton={p['K_ton_yaw']:.1f} Kpr={p['Kp_prop_yaw']:.1f} Kbk={p['Kp_buckle_yaw']:.2f}",
            flush=True,
        )

    best = results[0]
    pub = a3ex.publish_axis_best(a3ex.BEST_YAW_JSON, best, "yaw_sweep", 0, "loss")
    a3ex.atomic_write_json(
        os.path.join(a3ex.STATE_DIR, "yaw_sweep_results.json"),
        {"wall_s": round(time.time() - t0, 1), "results": results},
    )
    print(f"[YAW-SWEEP] Done in {time.time()-t0:.1f}s | Best YawMA={best['total_score']:.1f} | published={pub}", flush=True)


if __name__ == "__main__":
    main()
