#!/usr/bin/env python3
"""Unified 3-Axis 10-Parameter CMA-ES Optimization (Option A).

Features:
- Popsize = 22 (2 TPA candidates + 10 orthogonal Haar antipodal mirrored pairs = 22)
- Active covariance update (CMA_active = True) + CMAAdaptSigmaTPA (CMA_elitist = False)
- Zero surrogate model, zero local GP, zero pre-sweep
- Unweighted sum of APEO + LatEC + YawMA Classic Bode scores + Pitch cross-talk penalty:
    score = bode_apeo + bode_latec + bode_yawma
            + 100.0 * max_pitch_dev_latec_deg**2
            + 100.0 * max_pitch_dev_yawma_deg**2
- All 3 perturbations start at t = 0.8 s, ramp 1.5 s, run 15.0 s (target_time = 15.8 s),
  evaluated identically via scripts.generate_standard_plot.run_axis_worker.
"""

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

import cma
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from scripts.generate_standard_plot import run_axis_worker

PARAM_NAMES = [
    "tau_lead_pole",
    "K_sc",
    "p_sc",
    "z_sc",
    "K_phas",
    "z_phas",
    "Kp_prop_pitch",
    "z_ccr",
    "Kd_task_pitch",
    "Ki_task_pitch",
]

X0 = [
    0.014,   # 0: tau_lead_pole
    4.55,    # 1: K_sc
    58.0,    # 2: p_sc
    0.020,   # 3: z_sc
    10.8,    # 4: K_phas
    0.008,   # 5: z_phas
    0.52,    # 6: Kp_prop_pitch
    0.016,   # 7: z_ccr
    0.85,    # 8: Kd_task_pitch
    2.00,    # 9: Ki_task_pitch
]

LOWER_BOUNDS = [
    0.008,   # tau_lead_pole
    3.50,    # K_sc
    45.0,    # p_sc
    0.010,   # z_sc
    8.50,    # K_phas
    0.004,   # z_phas
    0.40,    # Kp_prop_pitch
    0.006,   # z_ccr
    0.45,    # Kd_task_pitch
    1.00,    # Ki_task_pitch
]

UPPER_BOUNDS = [
    0.022,   # tau_lead_pole
    5.50,    # K_sc
    75.0,    # p_sc
    0.026,   # z_sc
    13.50,   # K_phas
    0.015,   # z_phas
    0.75,    # Kp_prop_pitch
    0.024,   # z_ccr
    1.25,    # Kd_task_pitch
    3.80,    # Ki_task_pitch
]

CMA_STDS = [
    0.0025,  # tau_lead_pole
    0.25,    # K_sc
    4.50,    # p_sc
    0.0030,  # z_sc
    0.60,    # K_phas
    0.0020,  # z_phas
    0.05,    # Kp_prop_pitch
    0.0030,  # z_ccr
    0.12,    # Kd_task_pitch
    0.40,    # Ki_task_pitch
]

LOG_DIR = os.path.join(_ROOT, "logs")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_10p_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_10p_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_10p_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_10p_history.json")


def ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=True):
    """2 TPA slots + 10 Haar-orthogonal antipodal mirrored pairs in R^10."""
    n_dim = len(lb)
    raw_ask = es.ask()

    candidates = []
    if use_tpa and popsize >= 4:
        candidates.append(np.clip(np.asarray(raw_ask[0], dtype=float), lb, ub))
        candidates.append(np.clip(np.asarray(raw_ask[1], dtype=float), lb, ub))
        half = (popsize - 2) // 2
    else:
        half = popsize // 2

    n_blocks = int(math.ceil(float(half) / float(n_dim)))
    eigvals, b_mat = np.linalg.eigh(es.C)
    d_vec = np.sqrt(np.maximum(eigvals, 1e-16))
    mean_int = es.mean
    sigma = es.sigma

    z_dirs = []
    for _ in range(n_blocks):
        if n_dim == 1:
            q_mat = np.array([[1.0 if np.random.randn() >= 0.0 else -1.0]])
        else:
            raw_g = np.random.randn(n_dim, n_dim)
            q_mat, r_mat = np.linalg.qr(raw_g)
            d_signs = np.sign(np.diag(r_mat))
            d_signs[d_signs == 0.0] = 1.0
            q_mat = q_mat * d_signs

        chi_lengths = np.linalg.norm(np.random.randn(n_dim, n_dim), axis=0)
        for col_idx in range(n_dim):
            z_dirs.append(q_mat[:, col_idx] * chi_lengths[col_idx])
            if len(z_dirs) >= half:
                break
        if len(z_dirs) >= half:
            break

    coord_scale = np.asarray(es.sigma_vec.scaling, dtype=float) if hasattr(es, "sigma_vec") else np.ones(n_dim)
    for k in range(half):
        step_int = coord_scale * np.dot(b_mat, d_vec * z_dirs[k])
        for sgn in (+1.0, -1.0):
            x_int = mean_int + sgn * sigma * step_int
            x_ext = es.gp.pheno(x_int, into_bounds=es.boundary_handler.repair, archive=es.sent_solutions)
            x_ext = np.clip(x_ext, lb, ub)
            candidates.append(x_ext)

    return candidates[:popsize]


def _sim_job(job):
    cand_idx, mode, overrides = job
    res = run_axis_worker((mode, overrides))
    return {
        "cand_idx": cand_idx,
        "mode": mode,
        "aborted": bool(res["aborted"]),
        "abort_time_s": float(res["abort_time_s"]),
        "classic_bode_score": float(res["classic_bode_score"]),
        "gain_loss_pts": float(res["gain_loss_pts"]),
        "phase_loss_pts": float(res["phase_loss_pts"]),
        "pitch_crosstalk_penalty": float(res["pitch_crosstalk_penalty"]),
        "total_score": float(res["total_score"]),
        "full_pitch_dev_max_deg": float(res["full_pitch_dev_max_deg"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "corr_vel": float(res["corr_vel"]),
    }


def main():
    parser = argparse.ArgumentParser(description="22-Popsize Active Orthogonal-Mirrored TPA 3-Axis 10D CMA-ES")
    parser.add_argument("--gens", type=int, default=6, help="Number of CMA-ES generations to run")
    parser.add_argument("--workers", type=int, default=14, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4201, help="Random seed for CMA-ES and Haar QR sampling")
    parser.add_argument("--resume", action="store_true", help="Resume from existing logs/cmaes_3axis_10p_history.json")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    popsize = 22
    modes = ("APEO", "LatEC", "YawMA")

    cma_opts = {
        "popsize": popsize,
        "bounds": [LOWER_BOUNDS, UPPER_BOUNDS],
        "CMA_stds": CMA_STDS,
        "CMA_active": True,
        "CMA_elitist": False,
        "AdaptSigma": cma.sigma_adaptation.CMAAdaptSigmaTPA,
        "verbose": -9,
        "seed": args.seed,
    }
    es = cma.CMAEvolutionStrategy(list(X0), 1.0, cma_opts)

    global_best_score = float("inf")
    global_best_entry = None
    history_records = []
    start_gen = 1

    if args.resume and os.path.exists(ALL_GENS_JSON):
        with open(ALL_GENS_JSON, "r", encoding="utf-8") as f:
            history_records = json.load(f)
        for rec in history_records:
            _ = ask_orthogonal_mirrored_nd(es, popsize, LOWER_BOUNDS, UPPER_BOUNDS, use_tpa=True)
            logged_cands = [
                np.array([float(c["params"][k]) for k in PARAM_NAMES], dtype=float)
                for c in rec["candidates"]
            ]
            scores = [float(c["total_score"]) for c in rec["candidates"]]
            es.tell(logged_cands, scores)
            gb = rec["gen_best"]
            if float(gb["total_score"]) < global_best_score:
                global_best_score = float(gb["total_score"])
                global_best_entry = gb
        start_gen = len(history_records) + 1
        print(
            f"=== Resumed from {len(history_records)} completed generations "
            f"(sigma={es.sigma:.4f}, GlobalBest={global_best_score:,.2f}) -> starting Gen {start_gen} ===",
            flush=True,
        )

    if not history_records:
        with open(LEDGER_PATH, "w", encoding="utf-8") as f:
            f.write(
                "# 3-Axis 10-Parameter CMA-ES Ledger (Popsize=22: 2 TPA + 10 Mirrored Pairs, Active CMA, Zero Surrogate/GP)\n\n"
                f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | t_delay=0.8s, ramp=1.5s, pert_dur=15.0s (T=15.8s)\n\n"
                "| Gen | Best Total | APEO Bode | LatEC Bode | YawMA Bode | PitchXTalk (L/Y deg & pts) | σ | Params |\n"
                "|---:|---:|---:|---:|---:|---|---:|---|\n"
            )
    else:
        with open(LEDGER_PATH, "w", encoding="utf-8") as f:
            f.write(
                "# 3-Axis 10-Parameter CMA-ES Ledger (Popsize=22: 2 TPA + 10 Mirrored Pairs, Active CMA, Zero Surrogate/GP)\n\n"
                f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | t_delay=0.8s, ramp=1.5s, pert_dur=15.0s (T=15.8s)\n\n"
                "| Gen | Best Total | APEO Bode | LatEC Bode | YawMA Bode | PitchXTalk (L/Y deg & pts) | σ | Params |\n"
                "|---:|---:|---:|---:|---:|---|---:|---|\n"
            )
            for rec in history_records:
                gb = rec["gen_best"]
                p_str = ", ".join(f"{k}={gb['params'][k]}" for k in PARAM_NAMES)
                f.write(
                    f"| {rec['gen']} | {gb['total_score']:,.2f} | {gb['bode_apeo']:,.2f} | "
                    f"{gb['bode_latec']:,.2f} | {gb['bode_yawma']:,.2f} | "
                    f"{gb['pitch_dev_latec_deg']:.2f}°/{gb['pitch_dev_yawma_deg']:.2f}° "
                    f"({gb['xtalk_latec']+gb['xtalk_yawma']:,.1f}) | {rec['sigma']:.3f} | {p_str} |\n"
                )

    print(
        f"=== Starting 10D 3-Axis CMA-ES (Pop={popsize} [2 TPA + 10 Mirrored Pairs], Active=True, Zero Surrogate/GP, "
        f"Workers={args.workers}, Gens={start_gen}..{args.gens}) ===",
        flush=True,
    )

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(start_gen, args.gens + 1):
            t0 = time.time()
            candidates = ask_orthogonal_mirrored_nd(es, popsize, LOWER_BOUNDS, UPPER_BOUNDS, use_tpa=True)
            if gen == 1:
                candidates[0] = np.clip(np.asarray(X0, dtype=float), LOWER_BOUNDS, UPPER_BOUNDS)

            jobs = []
            for idx, x in enumerate(candidates):
                overrides = [(name, f"{float(val):.8g}") for name, val in zip(PARAM_NAMES, x)]
                for m in modes:
                    jobs.append((idx, m, overrides))

            per_cand = {idx: {} for idx in range(popsize)}
            futs = [pool.submit(_sim_job, j) for j in jobs]
            n_total_jobs = len(jobs)
            for done_cnt, fut in enumerate(as_completed(futs), 1):
                r = fut.result()
                per_cand[r["cand_idx"]][r["mode"]] = r
                if done_cnt % 11 == 0 or done_cnt == n_total_jobs:
                    print(
                        f"  [Gen {gen:02d}] {done_cnt:2d}/{n_total_jobs} sims done ({time.time() - t0:.0f}s) — "
                        f"last cand={r['cand_idx']:02d} {r['mode']}: Bode={r['classic_bode_score']:,.1f}, "
                        f"PitchMax={r['full_pitch_dev_max_deg']:.2f}°",
                        flush=True,
                    )

            cand_summaries = []
            scores = []
            for idx in range(popsize):
                res_apeo = per_cand[idx]["APEO"]
                res_latec = per_cand[idx]["LatEC"]
                res_yawma = per_cand[idx]["YawMA"]

                bode_apeo = res_apeo["classic_bode_score"]
                bode_latec = res_latec["classic_bode_score"]
                bode_yawma = res_yawma["classic_bode_score"]
                xtalk_latec = res_latec["pitch_crosstalk_penalty"]
                xtalk_yawma = res_yawma["pitch_crosstalk_penalty"]
                total_score = bode_apeo + bode_latec + bode_yawma + xtalk_latec + xtalk_yawma
                scores.append(total_score)

                entry = {
                    "gen": gen,
                    "slot": idx,
                    "slot_type": "TPA+" if idx == 0 else ("TPA-" if idx == 1 else f"Pair{(idx-2)//2 + 1}{'+' if idx % 2 == 0 else '-'}"),
                    "params": {k: round(float(v), 6) for k, v in zip(PARAM_NAMES, candidates[idx])},
                    "total_score": round(total_score, 2),
                    "bode_apeo": round(bode_apeo, 2),
                    "bode_latec": round(bode_latec, 2),
                    "bode_yawma": round(bode_yawma, 2),
                    "xtalk_latec": round(xtalk_latec, 2),
                    "xtalk_yawma": round(xtalk_yawma, 2),
                    "pitch_dev_latec_deg": round(res_latec["full_pitch_dev_max_deg"], 3),
                    "pitch_dev_yawma_deg": round(res_yawma["full_pitch_dev_max_deg"], 3),
                    "hf_noise_apeo": round(res_apeo["hf_noise_rms"], 2),
                    "hf_noise_latec": round(res_latec["hf_noise_rms"], 2),
                    "hf_noise_yawma": round(res_yawma["hf_noise_rms"], 2),
                    "aborted": any(per_cand[idx][m]["aborted"] for m in modes),
                }
                cand_summaries.append(entry)

            es.tell(candidates, scores)
            best_i = int(np.argmin(scores))
            gen_best = cand_summaries[best_i]

            if gen_best["total_score"] < global_best_score:
                global_best_score = gen_best["total_score"]
                global_best_entry = gen_best
                with open(CHAMPION_JSON, "w", encoding="utf-8") as f:
                    json.dump(global_best_entry, f, indent=2)

            history_records.append({
                "gen": gen,
                "wall_s": round(time.time() - t0, 1),
                "sigma": round(float(es.sigma), 4),
                "gen_best": gen_best,
                "global_best_score": global_best_score,
                "candidates": cand_summaries,
            })
            with open(ALL_GENS_JSON, "w", encoding="utf-8") as f:
                json.dump(history_records, f, indent=2)
            with open(STATE_JSON, "w", encoding="utf-8") as f:
                json.dump({
                    "last_gen": gen,
                    "sigma": float(es.sigma),
                    "mean": {k: float(v) for k, v in zip(PARAM_NAMES, es.mean)},
                    "global_best": global_best_entry,
                }, f, indent=2)

            p_str = ", ".join(f"{k}={gen_best['params'][k]}" for k in PARAM_NAMES)
            with open(LEDGER_PATH, "a", encoding="utf-8") as f:
                f.write(
                    f"| {gen} | {gen_best['total_score']:,.2f} | {gen_best['bode_apeo']:,.2f} | "
                    f"{gen_best['bode_latec']:,.2f} | {gen_best['bode_yawma']:,.2f} | "
                    f"{gen_best['pitch_dev_latec_deg']:.2f}°/{gen_best['pitch_dev_yawma_deg']:.2f}° "
                    f"({gen_best['xtalk_latec']+gen_best['xtalk_yawma']:,.1f}) | {es.sigma:.3f} | {p_str} |\n"
                )

            dt = time.time() - t0
            print(
                f"[Gen {gen:02d}/{args.gens:02d} | {dt:5.1f}s | σ={es.sigma:.3f}] "
                f"GenBest={gen_best['total_score']:,.2f} ({gen_best['slot_type']}) "
                f"[APEO={gen_best['bode_apeo']:,.1f}, LatEC={gen_best['bode_latec']:,.1f}, YawMA={gen_best['bode_yawma']:,.1f}, "
                f"XTalk={gen_best['xtalk_latec']+gen_best['xtalk_yawma']:,.1f} ({gen_best['pitch_dev_latec_deg']:.2f}°/{gen_best['pitch_dev_yawma_deg']:.2f}°)] "
                f"| GlobalBest={global_best_score:,.2f}",
                flush=True,
            )
            print(f"   Params: {p_str}", flush=True)


if __name__ == "__main__":
    main()
