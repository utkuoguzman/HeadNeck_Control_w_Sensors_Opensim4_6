#!/usr/bin/env python3
"""Wide Multi-Decade / Order-of-Magnitude Sweep for Pitch (APEO ISO -p_arr + 2g/3g Sled) on TRUBA.

Evaluates 56 exact 15.8s APEO (-p_arr ISO 2631) + 2g/3g sled candidates spanning orders of magnitude
of K_sc, K_phas, K_ton, Ki_ton, Kp_prop_pitch, Kp_proprioception, Kp_buckle, Kd_buckle, srs_k_srs,
p_sc, p_phas, tau_lead_pole, and publishes the best to logs/phase_a3_v3/best_pitch_24d.json.
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


def build_pitch_sweep_candidates(n_cand: int = 56, seed: int = 20261195):
    rng = np.random.default_rng(seed)
    _, base_p = a3ex.load_base_params()
    shared = a3ex.merge_shared_params(base_p, exclude_axis="pitch")

    # 14 multi-decade structured anchors spanning pure-soft (fn~0.9Hz) through Happee B8EC to stiff
    anchors = [
        {"K_sc": 0.05, "p_sc": 20.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.0, "Ki_ton": 0.0, "Kp_prop_pitch": 0.0, "Kp_proprioception": 0.0,   "Kp_buckle": 0.2, "Kd_buckle": 0.03, "srs_k_srs": 0.0},
        {"K_sc": 0.15, "p_sc": 25.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.3, "Ki_ton": 0.0, "Kp_prop_pitch": 0.1, "Kp_proprioception": 0.003, "Kp_buckle": 0.5, "Kd_buckle": 0.06, "srs_k_srs": 0.5},
        {"K_sc": 0.25, "p_sc": 30.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.6, "Ki_ton": 0.1, "Kp_prop_pitch": 0.2, "Kp_proprioception": 0.005, "Kp_buckle": 1.0, "Kd_buckle": 0.10, "srs_k_srs": 1.5},
        {"K_sc": 0.40, "p_sc": 25.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 1.0, "Ki_ton": 0.2, "Kp_prop_pitch": 0.3, "Kp_proprioception": 0.008, "Kp_buckle": 1.5, "Kd_buckle": 0.12, "srs_k_srs": 2.0},
        {"K_sc": 0.15, "p_sc": 20.0, "K_phas": 0.15, "p_phas": 4.0, "K_ton": 0.4, "Ki_ton": 0.0, "Kp_prop_pitch": 0.1, "Kp_proprioception": 0.003, "Kp_buckle": 2.0, "Kd_buckle": 0.08, "srs_k_srs": 0.5},
        {"K_sc": 0.30, "p_sc": 40.0, "K_phas": 0.30, "p_phas": 4.0, "K_ton": 0.8, "Ki_ton": 0.2, "Kp_prop_pitch": 0.2, "Kp_proprioception": 0.005, "Kp_buckle": 2.5, "Kd_buckle": 0.10, "srs_k_srs": 1.0},
        {"K_sc": 0.60, "p_sc": 40.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 1.5, "Ki_ton": 0.3, "Kp_prop_pitch": 0.5, "Kp_proprioception": 0.010, "Kp_buckle": 2.0, "Kd_buckle": 0.15, "srs_k_srs": 3.0},
        {"K_sc": 0.60, "p_sc": 40.0, "K_phas": 0.25, "p_phas": 4.0, "K_ton": 2.0, "Ki_ton": 0.5, "Kp_prop_pitch": 0.8, "Kp_proprioception": 0.015, "Kp_buckle": 2.0, "Kd_buckle": 0.20, "srs_k_srs": 4.0},
        {"K_sc": 0.10, "p_sc": 15.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.2, "Ki_ton": 0.0, "Kp_prop_pitch": 0.05,"Kp_proprioception": 0.002, "Kp_buckle": 3.5, "Kd_buckle": 0.05, "srs_k_srs": 0.0},
        {"K_sc": 0.20, "p_sc": 10.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.5, "Ki_ton": 0.0, "Kp_prop_pitch": 0.15,"Kp_proprioception": 0.004, "Kp_buckle": 0.3, "Kd_buckle": 0.04, "srs_k_srs": 1.0},
        {"K_sc": 0.50, "p_sc": 10.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.3, "Ki_ton": 0.0, "Kp_prop_pitch": 0.10,"Kp_proprioception": 0.002, "Kp_buckle": 1.0, "Kd_buckle": 0.05, "srs_k_srs": 0.0},
        {"K_sc": 1.00, "p_sc": 8.0,  "K_phas": 0.00, "p_phas": 4.0, "K_ton": 1.0, "Ki_ton": 0.2, "Kp_prop_pitch": 0.30,"Kp_proprioception": 0.006, "Kp_buckle": 1.5, "Kd_buckle": 0.10, "srs_k_srs": 2.0},
        {"K_sc": 0.05, "p_sc": 15.0, "K_phas": 0.00, "p_phas": 4.0, "K_ton": 0.0, "Ki_ton": 0.0, "Kp_prop_pitch": 0.0, "Kp_proprioception": 0.0,   "Kp_buckle": 4.0, "Kd_buckle": 0.05, "srs_k_srs": 0.0},
        {"K_sc": 0.30, "p_sc": 60.0, "K_phas": 0.50, "p_phas": 6.0, "K_ton": 1.0, "Ki_ton": 0.5, "Kp_prop_pitch": 0.4, "Kp_proprioception": 0.008, "Kp_buckle": 0.8, "Kd_buckle": 0.08, "srs_k_srs": 2.5},
    ]

    specs = [
        ("K_sc",                 0.03,  2.50, True),
        ("z_sc",                 0.001, 0.050, True),
        ("p_sc",                 2.00, 100.0, True),
        ("K_phas",               0.005, 1.50, True),
        ("z_phas",               0.001, 0.040, True),
        ("p_phas",               0.80,  25.0, True),
        ("oto_reweight_a0_mps2", 4.00,  16.0, False),
        ("oto_reweight_tau_s",   0.06,  0.35, False),
        ("phasic_oto_tanh_mps2", 6.00,  25.0, False),
        ("K_ton",                0.05,  4.00, True),
        ("p_ton",                3.00,  35.0, True),
        ("Ki_ton",               0.00,  2.00, False),
        ("Kp_prop_pitch",        0.02,  2.00, True),
        ("Kp_proprioception",    0.001, 0.040, True),
        ("tau_lead_pole",        0.003, 0.028, False),
        ("Kp_task_pitch",        6.00,  28.0, False),
        ("Kd_task_pitch",        1.20,  7.00, False),
        ("Ki_task_pitch",       10.00,  45.0, False),
        ("Kp_buckle",            0.10,  5.00, True),
        ("Kd_buckle",            0.02,  0.40, True),
        ("srs_k_srs",            0.10,  8.00, True),
        ("srs_delta_srs",        0.010, 0.030, False),
        ("srs_thix_min",         0.08,  0.40, False),
        ("srs_delta_thix_break", 0.005, 0.035, False),
    ]

    n_lhs = max(0, n_cand - len(anchors))
    lhs_cols = {}
    for name, lo, hi, is_log in specs:
        perm = rng.permutation(n_lhs)
        u = (perm + rng.uniform(0.0, 1.0, size=n_lhs)) / max(1, n_lhs)
        vals = np.exp(np.log(lo) + u * (np.log(hi) - np.log(lo))) if is_log else (lo + u * (hi - lo))
        lhs_cols[name] = vals

    default_ap = {
        "K_sc": 0.60, "z_sc": 0.010, "p_sc": 40.0,
        "K_phas": 0.20, "z_phas": 0.010, "p_phas": 4.0,
        "oto_reweight_a0_mps2": 9.0, "oto_reweight_tau_s": 0.18, "phasic_oto_tanh_mps2": 14.0,
        "K_ton": 2.0, "p_ton": 15.0, "Ki_ton": 0.5,
        "Kp_prop_pitch": 0.80, "Kp_proprioception": 0.015, "tau_lead_pole": 0.012,
        "Kp_task_pitch": 13.0, "Kd_task_pitch": 3.5, "Ki_task_pitch": 30.0,
        "Kp_buckle": 2.0, "Kd_buckle": 0.25,
        "srs_k_srs": 4.0, "srs_delta_srs": 0.020, "srs_thix_min": 0.14, "srs_delta_thix_break": 0.012,
    }

    cands = []
    for idx in range(n_cand):
        ap = dict(default_ap)
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
    bode = a3ex.run_bode_exact(full_p, "APEO", wall_limit_s=2400.0)
    if bode.get("aborted", False):
        return {
            "idx": idx, "aborted": True, "loss": float(bode["total_score"]),
            "apeo_total": float(bode["total_score"]), "apeo_classic": float(bode["classic_score"]),
            "apeo_gain_loss": 0.0, "apeo_phase_loss": 0.0, "apeo_rv": 0.0, "apeo_hf": 99.0,
            "n_pk": [9, 9, 9, 9], "vol60_end": 0.0, "axis_params": ap,
            "abort_reason": bode.get("abort_reason", ""),
        }
    # Full 4 sleds + VOL60 bundle
    sled_res = a3ex._dispatch({"kind": "sled_bundle", "idx": idx, "params": full_p})["res"]
    scored = a3ex.score_pitch_candidate(idx, ap, bode, sled_res)
    return scored


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=56)
    ap.add_argument("--workers", type=int, default=56)
    ap.add_argument("--seed", type=int, default=20261195)
    args = ap.parse_args()

    t0 = time.time()
    cands = build_pitch_sweep_candidates(args.n, args.seed)
    print(f"[PITCH-SWEEP ISO] Evaluating {len(cands)} multi-decade APEO(-p_arr)+Sled+VOL60 candidates on {args.workers} workers...", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(_eval, cands))

    results.sort(key=lambda r: float(r["loss"]))
    print("\n=== SORTED PITCH SWEEP RESULTS (Top 20) ===", flush=True)
    for r in results[:20]:
        p = r["axis_params"]
        print(
            f"#{r['idx']:02d} ab={int(r['aborted'])} | Loss={r['loss']:8.1f} APEO={r.get('apeo_total',0):8.1f} "
            f"G={r.get('apeo_gain_loss',0):6.0f} P={r.get('apeo_phase_loss',0):6.0f} rv={r.get('apeo_rv',0):+.3f} "
            f"nPk={r.get('n_pk',[])} v60={r.get('vol60_end',0):.1f} | "
            f"Ksc={p['K_sc']:.2f} psc={p['p_sc']:.0f} Kph={p['K_phas']:.2f} Kton={p['K_ton']:.1f} "
            f"Kpr={p['Kp_prop_pitch']:.2f} Kbk={p['Kp_buckle']:.2f} ksrs={p['srs_k_srs']:.1f}",
            flush=True,
        )

    best = results[0]
    pub = a3ex.publish_axis_best(a3ex.BEST_PITCH_JSON, best, "pitch_sweep_iso", 0, "loss")
    a3ex.atomic_write_json(
        os.path.join(a3ex.STATE_DIR, "pitch_sweep_iso_results.json"),
        {"wall_s": round(time.time() - t0, 1), "results": results},
    )
    print(f"[PITCH-SWEEP ISO] Done in {time.time()-t0:.1f}s | Best Loss={best['loss']:.1f} APEO={best.get('apeo_total',0):.1f} | published={pub}", flush=True)


if __name__ == "__main__":
    main()
