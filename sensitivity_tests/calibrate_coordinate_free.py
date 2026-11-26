#!/usr/bin/env python3
"""Coupled 3-Axis Local Calibrator for the Coordinate-Free Geometric & Tensor-Network Controller.

Evaluates APEO, LatEC, and YawMA simultaneously on ONE unified coordinate-free parameter vector
(100% Full Model — all biological subsystems ON, exact 15.8 s evaluation windows), maintains a live
JSON blackboard (`sensitivity_tests/results/coord_free_blackboard.json`), and supports:
  1. `--mode probe` : Multi-seed smart physics-informed grid/probe batch to map the 3-axis landscape
  2. `--mode cma`   : Compact covariance-matrix adaptation (CMA-ES) driving max(APEO, LatEC, YawMA) < 5,000
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Dict, List, Tuple

for _env_k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_env_k, "1")

import cma
import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in (_REPO_ROOT, _THIS_DIR, os.path.join(_REPO_ROOT, "scripts"), os.path.join(_REPO_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sensitivity_tests.bode_xray import print_xray, run_bode_probed

RESULTS_DIR = os.path.join(_THIS_DIR, "results")
BLACKBOARD_JSON = os.path.join(RESULTS_DIR, "coord_free_blackboard.json")
MODES = ("APEO", "LatEC", "YawMA")

# Compact 25D Coordinate-Free Covariant Tensor-Pencil Search Space: (name, x0, lo, hi, sigma0)
COORD_FREE_SPECS: List[Tuple[str, float, float, float, float]] = [
    # 1. Brainstem Tonic VCR Tensor Pencil {Kg3, Kdisc3, Gmu3_eq}
    ("alpha_grav_ton",     -6.50,  -15.00, 12.00,  1.50),
    ("c_disc_ton",         48.00,   -5.00, 75.00,  6.00),
    ("gamma_musc_ton",     -2.00,  -15.00, 15.00,  1.50),
    ("p_ton",              16.50,    2.00, 35.00,  3.50),
    ("ki_grav_ton",         0.00,    0.00,  0.80,  0.08),
    # 2. Brainstem Semicircular Canal VCR Tensor Pencil + Graviceptive Tilt-Dumping Pole P_sc
    ("omega_sc",           12.00,    0.00, 48.00,  5.00),
    ("c_disc_sc",           3.50,   -8.00, 20.00,  2.00),
    ("beta_musc_sc",       -0.80,   -6.00,  6.00,  0.80),
    ("z_sc",                0.014,   0.001, 0.040, 0.004),
    ("p_sc",                1.30,    0.40, 30.00,  1.20),
    ("c_omega_p_sc",        0.50,    0.00,  1.50,  0.15),
    # 3. Brainstem Phasic Otolith VCR Tensor Pencil {Kg3, Gmu3_eq}
    ("alpha_inert_phas",   46.00,    0.00, 85.00,  6.00),
    ("gamma_musc_phas",   -62.00, -130.00, 20.00,  8.00),
    ("z_phas",              0.016,   0.000, 0.045, 0.005),
    ("p_phas",              4.50,    1.50, 16.00,  1.50),
    # 4. Spinal CCR Covariant Pencil {Kdisc3, Gmu3_eq, Kg3} & Fiber/Synergy Network
    ("c_disc_ccr",         38.60,    0.00, 70.00,  5.00),
    ("gamma_musc_ccr",      0.00,  -15.00, 15.00,  1.50),
    ("c_grav_ccr",         -7.20,  -20.00,  5.00,  1.50),
    ("kappa_ia_local",      0.018,   0.001, 0.060, 0.006),
    ("kappa_ia_synergy",    0.006,   0.000, 0.050, 0.006),
    ("z_ccr",               0.016,   0.002, 0.040, 0.005),
    # 5. Chain Nullspace Anti-Buckling, Synaptic Lead & Cross-Bridge SRS
    ("kappa_buckle",        1.25,    0.15,  3.50,  0.30),
    ("beta_buckle",         0.34,    0.02,  0.80,  0.08),
    ("tau_lead_pole",       0.013,   0.004, 0.025, 0.003),
    ("srs_k_srs",           9.00,    0.50, 15.00,  2.00),
]


def _eval_single_mode(task):
    cand_id, mode, params, exact_15s, collect_xray = task
    res = run_bode_probed(mode, params, exact_15s=exact_15s, collect_xray=collect_xray)
    return cand_id, mode, res


def coupled_objective(scores: Dict[str, float]) -> float:
    """Smooth barrier-augmented sum driving ALL THREE Bodes < 5,000 simultaneously."""
    s_ap = float(scores["APEO"])
    s_la = float(scores["LatEC"])
    s_ya = float(scores["YawMA"])
    base_sum = s_ap + s_la + s_ya
    # Smooth quadratic-linear excess above 4,850 strongly prioritizes bringing lagging axes < 5,000
    excess_pen = 0.0
    for s in (s_ap, s_la, s_ya):
        ex = max(0.0, s - 4850.0)
        excess_pen += 2.5 * ex + (ex * ex) / 4000.0
    return base_sum + excess_pen


def load_blackboard() -> dict:
    if os.path.exists(BLACKBOARD_JSON):
        try:
            with open(BLACKBOARD_JSON, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {
        "updated_at": "",
        "best_coupled_obj": 1e12,
        "best_max3": 1e12,
        "best_sum3": 1e12,
        "all_under_5000": False,
        "best_params": {},
        "best_axes": {},
        "history": [],
    }


def save_blackboard(bb: dict) -> None:
    os.makedirs(RESULTS_DIR, exist_ok=True)
    bb["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    tmp = f"{BLACKBOARD_JSON}.tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(bb, f, indent=2)
    os.replace(tmp, BLACKBOARD_JSON)


def evaluate_population(
    candidates: List[Tuple[str, Dict[str, float]]],
    workers: int,
    exact_15s: bool,
    bb: dict,
    collect_xray: bool = True,
) -> List[dict]:
    tasks = []
    for cid, p in candidates:
        for m in MODES:
            tasks.append((cid, m, dict(p), exact_15s, collect_xray))

    by_cid: Dict[str, Dict[str, dict]] = {cid: {} for cid, _ in candidates}
    pmap = {cid: p for cid, p in candidates}

    with ProcessPoolExecutor(max_workers=workers) as pool:
        futs = {pool.submit(_eval_single_mode, tk): tk for tk in tasks}
        for fut in as_completed(futs):
            cid, m, res = fut.result()
            by_cid[cid][m] = res
            print(
                f"    [{cid}:{m:5s}] TOT={res.get('total_score', 1.2e6):8.1f} "
                f"(cls={res.get('classic_score', 0):7.1f} G={res.get('gain_loss', 0):5.0f} "
                f"P={res.get('phase_loss', 0):5.0f} xt={res.get('xtalk_pen', 0):5.0f} "
                f"hfP={res.get('hf_pen', 0):4.0f}) rv={res.get('rv', 0):+.3f} hf={res.get('hf_rms', 0):.2f} ({res.get('wall_s', 0):.0f}s)",
                flush=True,
            )

    out_records = []
    for cid, p in candidates:
        axes = by_cid[cid]
        scores = {m: float(axes[m].get("total_score", 1.2e6)) for m in MODES}
        max3 = max(scores.values())
        sum3 = sum(scores.values())
        obj = coupled_objective(scores)
        rec = {
            "id": cid,
            "obj": round(obj, 1),
            "max3": round(max3, 1),
            "sum3": round(sum3, 1),
            "APEO": round(scores["APEO"], 1),
            "LatEC": round(scores["LatEC"], 1),
            "YawMA": round(scores["YawMA"], 1),
            "rv": {m: axes[m].get("rv", 0.0) for m in MODES},
            "xt": {m: axes[m].get("xtalk_pen", 0.0) for m in MODES},
            "hf": {m: axes[m].get("hf_rms", 0.0) for m in MODES},
            "params": pmap[cid],
            "axes_detail": axes,
        }
        out_records.append(rec)

        # Update blackboard if improved on coupled objective or max3
        bb["history"].append({k: v for k, v in rec.items() if k != "axes_detail"})
        if obj < bb["best_coupled_obj"]:
            bb["best_coupled_obj"] = round(obj, 1)
            bb["best_max3"] = round(max3, 1)
            bb["best_sum3"] = round(sum3, 1)
            bb["all_under_5000"] = bool(max3 < 5000.0)
            bb["best_id"] = cid
            bb["best_params"] = dict(pmap[cid])
            bb["best_axes"] = axes
            print(
                f"  >>> NEW CHAMPION [{cid}]: APEO={scores['APEO']:.1f} | LatEC={scores['LatEC']:.1f} | "
                f"YawMA={scores['YawMA']:.1f} | MAX3={max3:.1f} | SUM3={sum3:.1f} <<<",
                flush=True,
            )
    save_blackboard(bb)
    return out_records


def build_smart_probes(base_p: Dict[str, float]) -> List[Tuple[str, Dict[str, float]]]:
    """Curated physics-informed initial cohort spanning soft/medium/stiff canal, phasic, SRS & tonic regimes."""
    p0 = dict(base_p)
    probes = [
        ("CF_0_base", dict(p0)),
        # Soft SRS + moderate canal/phasic (favors LatEC & YawMA low-freq gain + smooth APEO)
        ("CF_1_soft_srs", {
            **p0,
            "srs_k_srs": 0.60, "omega_sc": 6.0, "c_disc_sc": 0.08, "p_sc": 8.0,
            "alpha_inert_phas": 0.60, "c_disc_phas": 0.10, "p_phas": 4.0,
            "alpha_grav_ton": 0.70, "omega_ton": 3.5, "c_disc_ton": 0.40, "p_ton": 12.0,
            "c_disc_ccr": 0.30, "kappa_ia_local": 0.004, "kappa_ia_synergy": 0.003,
            "kappa_buckle": 0.50, "beta_buckle": 0.05,
        }),
        # Balanced covariant pencil (matches P26 Pitch ~4100 + Y7_1 Yaw ~4836 + R_best Roll ~2000 ratios!)
        ("CF_2_covariant_match", {
            **p0,
            "srs_k_srs": 3.50,
            "omega_sc": 19.0, "c_disc_sc": 0.12, "c_grav_sc": 0.05, "p_sc": 18.0, "z_sc": 0.012,
            "alpha_inert_phas": 5.50, "c_disc_phas": 0.00, "p_phas": 3.50, "z_phas": 0.020,
            "alpha_grav_ton": 0.65, "omega_ton": 6.5, "c_disc_ton": 0.35, "p_ton": 14.0, "ki_grav_ton": 0.40,
            "c_disc_ccr": 0.45, "kappa_ia_local": 0.008, "kappa_ia_synergy": 0.006,
            "kappa_buckle": 0.85, "beta_buckle": 0.10, "vcr_compliant_share": 0.35,
        }),
        # High-phasic / crisp Pitch+Roll tilt + nimble Yaw (c_grav separates Tilt from Yaw!)
        ("CF_3_tilt_yaw_split", {
            **p0,
            "srs_k_srs": 5.50,
            "omega_sc": 14.0, "c_disc_sc": 0.25, "c_grav_sc": 0.18, "p_sc": 28.0, "z_sc": 0.012, "c_omega_p_sc": 1.20,
            "alpha_inert_phas": 8.50, "c_disc_phas": 1.20, "p_phas": 3.40, "z_phas": 0.022,
            "alpha_grav_ton": 0.72, "omega_ton": 8.0, "c_disc_ton": 0.80, "p_ton": 15.0, "ki_grav_ton": 0.45,
            "c_disc_ccr": 0.55, "kappa_ia_local": 0.010, "kappa_ia_synergy": 0.006,
            "kappa_buckle": 0.90, "beta_buckle": 0.12, "vcr_compliant_share": 0.40,
        }),
        # Low-pole canal + moderate phasic (targets LatEC 0.12-0.5 Hz & YawMA 0.5-2 Hz phase!)
        ("CF_4_lowpole_sc", {
            **p0,
            "srs_k_srs": 1.50,
            "omega_sc": 12.0, "c_disc_sc": 0.20, "c_grav_sc": 0.12, "p_sc": 6.50, "z_sc": 0.014, "c_omega_p_sc": 0.50,
            "alpha_inert_phas": 3.00, "c_disc_phas": 0.50, "p_phas": 3.80, "z_phas": 0.016,
            "alpha_grav_ton": 0.68, "omega_ton": 5.0, "c_disc_ton": 0.60, "p_ton": 12.0, "ki_grav_ton": 0.35,
            "c_disc_ccr": 0.35, "kappa_ia_local": 0.006, "kappa_ia_synergy": 0.004,
            "kappa_buckle": 0.65, "beta_buckle": 0.06, "vcr_compliant_share": 0.25,
        }),
    ]
    return probes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["probe", "cma"], default="probe")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--popsize", type=int, default=6)
    ap.add_argument("--gens", type=int, default=12)
    ap.add_argument("--fast10", action="store_true", help="Use 10s APEO/YawMA (+15.8s LatEC) for rapid initial screening")
    ap.add_argument("--reset", action="store_true", help="Reset blackboard before running")
    args = ap.parse_args()

    exact_15s = not args.fast10
    bb = {} if args.reset else load_blackboard()
    if not bb or "best_coupled_obj" not in bb:
        bb = {
            "updated_at": "",
            "best_coupled_obj": 1e12,
            "best_max3": 1e12,
            "best_sum3": 1e12,
            "all_under_5000": False,
            "best_params": {},
            "best_axes": {},
            "history": [],
        }

    base_p = {nm: x0 for nm, x0, _, _, _ in COORD_FREE_SPECS}
    if bb.get("best_params"):
        base_p.update(bb["best_params"])

    if args.mode == "probe":
        probes = build_smart_probes(base_p)
        print(f"=== Running {len(probes)} Coupled 3-Axis Coordinate-Free Probes ({len(probes)*3} sims, exact_15s={exact_15s}) ===", flush=True)
        recs = evaluate_population(probes, workers=args.workers, exact_15s=exact_15s, bb=bb)
        print("\n=== PROBE SUMMARY ===")
        for r in sorted(recs, key=lambda x: x["obj"]):
            print(f"  {r['id']:22s} | APEO={r['APEO']:7.1f} LatEC={r['LatEC']:7.1f} YawMA={r['YawMA']:7.1f} | MAX3={r['max3']:7.1f} SUM3={r['sum3']:8.1f}")
        if bb.get("best_axes"):
            for m in MODES:
                print_xray(f"BEST [{bb.get('best_id', 'best')}]", bb["best_axes"][m])
        return

    # Mode: CMA-ES in normalized [0, 1] coordinates (sigma_i = s0 / (hi - lo))
    names = [s[0] for s in COORD_FREE_SPECS]
    lo = np.array([s[2] for s in COORD_FREE_SPECS], dtype=float)
    hi = np.array([s[3] for s in COORD_FREE_SPECS], dtype=float)
    span = hi - lo
    u0 = np.clip(np.array([(base_p[s[0]] - s[2]) / (s[3] - s[2]) for s in COORD_FREE_SPECS], dtype=float), 0.02, 0.98)
    stds = [float(s[4] / (s[3] - s[2])) for s in COORD_FREE_SPECS]

    es = cma.CMAEvolutionStrategy(
        u0.tolist(),
        1.0,
        {
            "popsize": args.popsize,
            "bounds": [0.0, 1.0],
            "CMA_stds": stds,
            "seed": 42,
            "verbose": -9,
        },
    )

    print(f"=== Starting {len(names)}D Coupled 3-Axis Coordinate-Free CMA-ES (pop={args.popsize}, gens={args.gens}, exact_15s={exact_15s}) ===", flush=True)
    for gen in range(1, args.gens + 1):
        t_g = time.time()
        us = es.ask()
        cands = []
        for idx, u in enumerate(us):
            x = lo + np.clip(np.asarray(u, dtype=float), 0.0, 1.0) * span
            p = {nm: float(val) for nm, val in zip(names, x)}
            cands.append((f"G{gen:02d}_C{idx:02d}", p))

        recs = evaluate_population(cands, workers=args.workers, exact_15s=exact_15s, bb=bb, collect_xray=False)
        objs = [r["obj"] for r in recs]
        es.tell(us, objs)
        best_g = min(recs, key=lambda r: r["obj"])
        print(
            f"[Gen {gen:02d}/{args.gens:02d} in {time.time()-t_g:.0f}s] GenBest={best_g['id']} "
            f"(APEO={best_g['APEO']:.1f}, LatEC={best_g['LatEC']:.1f}, YawMA={best_g['YawMA']:.1f}, MAX3={best_g['max3']:.1f}) | "
            f"AllTimeBest MAX3={bb['best_max3']:.1f} SUM3={bb['best_sum3']:.1f}",
            flush=True,
        )
        if bb.get("all_under_5000") and exact_15s:
            print("\n*** ALL THREE FULL-MODEL BODES ARE < 5,000 ON ONE COORDINATE-FREE VECTOR! ***", flush=True)
            for m in MODES:
                print_xray(f"WINNER [{bb.get('best_id')}]", bb["best_axes"][m])
            break


if __name__ == "__main__":
    main()
