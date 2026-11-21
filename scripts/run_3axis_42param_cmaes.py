#!/usr/bin/env python3
"""Unified 3-Axis 43-Parameter CMA-ES Optimization (All Controller/Compensator/Fusimotor Params).

Features:
- Popsize = 88 (2 TPA candidates + 43 orthogonal Haar antipodal mirrored pairs = 88)
- Active covariance update (CMA_active = True) + CMAAdaptSigmaTPA (CMA_elitist = False)
- Zero surrogate model, zero local GP, zero pre-sweep
- Unweighted sum of APEO + LatEC + YawMA Classic Bode scores + Pitch cross-talk penalty
- Seeds the 10 previously-optimized parameters from logs/cmaes_3axis_10p_champion.json if provided.
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

# (name, x0_default, lb, ub, std)
PARAM_SPECS = [
    ("tau_lead_pole",     0.015359,  0.004,  0.030,  0.0020),
    # Pitch Vestibular
    ("K_sc",              4.775912,  2.50,   7.50,   0.22),
    ("z_sc",              0.016343,  0.005,  0.035,  0.0025),
    ("p_sc",              56.051918, 30.0,   90.0,   4.0),
    ("K_phas",            10.777236, 6.0,    18.0,   0.55),
    ("z_phas",            0.008539,  0.002,  0.025,  0.0020),
    ("p_phas",            4.711937,  2.0,    12.0,   1.0),
    ("K_ton",             5.259843,  2.0,    10.0,   0.8),
    ("p_ton",             16.227225, 5.0,    35.0,   3.0),
    # Roll Vestibular
    ("K_sc_roll",         3.5,       1.5,    8.0,    0.6),
    ("z_sc_roll",         0.018,     0.005,  0.045,  0.004),
    ("p_sc_roll",         60.0,      30.0,   100.0,  9.0),
    ("K_phas_roll",       13.5,      6.0,    24.0,   2.2),
    ("z_phas_roll",       0.012,     0.003,  0.035,  0.003),
    ("p_phas_roll",       4.5,       1.5,    12.0,   1.0),
    ("K_ton_roll",        0.90,      0.2,    3.5,    0.20),
    ("p_ton_roll",        3.2,       1.0,    10.0,   0.8),
    # Yaw Vestibular
    ("K_sc_yaw",          1.32,      0.4,    4.0,    0.30),
    ("z_sc_yaw",          0.016,     0.004,  0.045,  0.0035),
    ("p_sc_yaw",          3.5,       1.0,    15.0,   0.8),
    ("K_ton_yaw",         6.2,       2.0,    12.0,   1.0),
    ("p_ton_yaw",         16.227225, 5.0,    35.0,   3.0),
    # CCR / Proprioception
    ("Kp_prop_pitch",     0.537304,  0.25,   1.20,   0.05),
    ("z_ccr",             0.020081,  0.004,  0.045,  0.0025),
    ("Kp_prop_roll",      2.10,      0.80,   5.00,   0.40),
    ("z_ccr_roll",        0.005,     0.001,  0.030,  0.0020),
    ("Kp_prop_yaw",       14.0,      4.0,    30.0,   2.5),
    ("z_ccr_yaw",         0.050,     0.010,  0.120,  0.010),
    ("p_ccr",             99.919586, 40.0,   160.0,  15.0),
    ("Kp_proprioception", 0.050,     0.015,  0.150,  0.010),
    # Cortical Task PIDs
    ("Kp_task_pitch",     3.790117,  1.50,   7.00,   0.55),
    ("Kd_task_pitch",     0.744122,  0.25,   1.60,   0.10),
    ("Ki_task_pitch",     1.678854,  0.80,   6.50,   0.35),
    ("Kp_task_roll",      1.20,      0.40,   4.00,   0.25),
    ("Kd_task_roll",      1.35,      0.40,   3.50,   0.25),
    ("Ki_task_roll",      0.45,      0.05,   2.50,   0.12),
    ("Kp_task_yaw",       0.55,      0.15,   2.50,   0.12),
    ("Kd_task_yaw",       0.05,      0.01,   0.50,   0.020),
    ("Ki_task_yaw",       0.05,      0.005,  0.50,   0.020),
    # Fusimotor & Anti-Buckling
    ("K_gamma_dyn",       0.92,      0.0,    1.0,    0.15),
    ("K_gamma_stat",      0.98,      0.0,    1.0,    0.15),
    ("Kp_buckle",         1.5,       0.4,    4.0,    0.30),
    ("Kd_buckle",         0.15,      0.03,   0.50,   0.035),
]

GEN2_CHAMPION_10P = {
    "tau_lead_pole": 0.011024, "K_sc": 4.678367, "p_sc": 53.925437, "z_sc": 0.018324,
    "K_phas": 10.710151, "z_phas": 0.0081, "Kp_prop_pitch": 0.475065, "z_ccr": 0.01894,
    "Kd_task_pitch": 0.702159, "Ki_task_pitch": 2.030608,
}

GEN6_BEST_10P = {
    "tau_lead_pole": 0.015529, "K_sc": 4.833826, "p_sc": 59.637673, "z_sc": 0.015797,
    "K_phas": 10.685056, "z_phas": 0.006493, "Kp_prop_pitch": 0.586823, "z_ccr": 0.019993,
    "Kd_task_pitch": 0.828411, "Ki_task_pitch": 1.834523,
}

PARAM_NAMES = [s[0] for s in PARAM_SPECS]
X0_DEFAULT = [s[1] for s in PARAM_SPECS]
LOWER_BOUNDS = [s[2] for s in PARAM_SPECS]
UPPER_BOUNDS = [s[3] for s in PARAM_SPECS]
CMA_STDS = [s[4] for s in PARAM_SPECS]

LOG_DIR = os.path.join(_ROOT, "logs")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_43p_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_43p_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_43p_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_43p_history.json")


def ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=True):
    """2 TPA slots + orthogonal Haar antipodal mirrored pairs in R^N."""
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
        "pitch_crosstalk_penalty": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "total_score": float(res["total_score"]),
        "full_pitch_dev_max_deg": float(res["full_pitch_dev_max_deg"]),
        "post_pitch_dev_max_deg": float(res["post_pitch_dev_max_deg"]),
        "post_roll_max_deg": float(res["post_roll_max_deg"]),
        "post_yaw_rel_max_deg": float(res["post_yaw_rel_max_deg"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
    }


def main():
    parser = argparse.ArgumentParser(description="88-Popsize Active Orthogonal-Mirrored TPA 3-Axis 43D CMA-ES")
    parser.add_argument("--gens", type=int, default=12, help="Number of CMA-ES generations to run")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4301, help="Random seed")
    parser.add_argument("--seed-from-champion", default="", help="Optional 10P champion JSON to seed matching parameters")
    parser.add_argument("--seed-from-state", default="", help="Optional 10P state JSON (mean dict) to center X0")
    parser.add_argument("--resume", action="store_true", help="Resume from existing logs/cmaes_3axis_43p_history.json")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    def _overlay(base_vec, param_dict):
        vec = list(base_vec)
        for idx, name in enumerate(PARAM_NAMES):
            if name in param_dict:
                vec[idx] = float(np.clip(float(param_dict[name]), LOWER_BOUNDS[idx], UPPER_BOUNDS[idx]))
        return np.asarray(vec, dtype=float)

    x0 = list(X0_DEFAULT)
    if args.seed_from_state and os.path.exists(args.seed_from_state):
        st = json.load(open(args.seed_from_state, "r", encoding="utf-8"))
        m_params = st.get("mean", {})
        x0 = list(_overlay(x0, m_params))
        print(f"=== Centered X0 from {args.seed_from_state} ({len(m_params)} params, last_gen={st.get('last_gen')}) ===", flush=True)
    elif args.seed_from_champion and os.path.exists(args.seed_from_champion):
        champ = json.load(open(args.seed_from_champion, "r", encoding="utf-8"))
        c_params = champ.get("params", {})
        x0 = list(_overlay(x0, c_params))
        print(f"=== Centered X0 from {args.seed_from_champion} (score={champ.get('total_score')}) ===", flush=True)

    popsize = 2 + 2 * len(PARAM_NAMES)  # 88
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
    es = cma.CMAEvolutionStrategy(x0, 1.0, cma_opts)

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
                np.clip(
                    np.array([float(c["params"][k]) for k in PARAM_NAMES], dtype=float),
                    LOWER_BOUNDS,
                    UPPER_BOUNDS,
                )
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
            f"=== Resumed 43D CMA-ES from Gen {len(history_records)} "
            f"(sigma={es.sigma:.4f}, GlobalBest={global_best_score:,.2f}) -> Gen {start_gen} ===",
            flush=True,
        )

    with open(LEDGER_PATH, "w", encoding="utf-8") as f:
        f.write(
            f"# 3-Axis {len(PARAM_NAMES)}-Parameter CMA-ES Ledger (Popsize={popsize}: 2 TPA + {len(PARAM_NAMES)} Mirrored Pairs)\n\n"
            f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | t_delay=0.8s, ramp=1.5s, pert_dur=15.0s (T=15.8s)\n\n"
            "| Gen | Best Total | APEO Bode | LatEC Bode | YawMA Bode | PitchXTalk (L/Y deg & pts) | σ |\n"
            "|---:|---:|---:|---:|---:|---|---:|\n"
        )
        for rec in history_records:
            gb = rec["gen_best"]
            f.write(
                f"| {rec['gen']} | {gb['total_score']:,.2f} | {gb['bode_apeo']:,.2f} | "
                f"{gb['bode_latec']:,.2f} | {gb['bode_yawma']:,.2f} | "
                f"{gb['pitch_dev_latec_deg']:.2f}°/{gb['pitch_dev_yawma_deg']:.2f}° "
                f"({gb['xtalk_latec']+gb['xtalk_yawma']:,.1f}) | {rec['sigma']:.3f} |\n"
            )

    print(
        f"=== Starting {len(PARAM_NAMES)}D 3-Axis CMA-ES (Pop={popsize}, Active=True, Zero Surrogate/GP, "
        f"Workers={args.workers}, Gens={start_gen}..{args.gens}) ===",
        flush=True,
    )

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(start_gen, args.gens + 1):
            t0 = time.time()
            candidates = ask_orthogonal_mirrored_nd(es, popsize, LOWER_BOUNDS, UPPER_BOUNDS, use_tpa=True)
            if gen == 1:
                candidates[0] = _overlay(X0_DEFAULT, GEN2_CHAMPION_10P)
                candidates[1] = _overlay(X0_DEFAULT, GEN6_BEST_10P)

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
                if done_cnt % 44 == 0 or done_cnt == n_total_jobs:
                    print(
                        f"  [Gen {gen:02d}] {done_cnt:3d}/{n_total_jobs} sims done ({time.time() - t0:.0f}s) — "
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

                cand_summaries.append({
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
                    "aborted": any(per_cand[idx][m]["aborted"] for m in modes),
                })

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

            with open(LEDGER_PATH, "a", encoding="utf-8") as f:
                f.write(
                    f"| {gen} | {gen_best['total_score']:,.2f} | {gen_best['bode_apeo']:,.2f} | "
                    f"{gen_best['bode_latec']:,.2f} | {gen_best['bode_yawma']:,.2f} | "
                    f"{gen_best['pitch_dev_latec_deg']:.2f}°/{gen_best['pitch_dev_yawma_deg']:.2f}° "
                    f"({gen_best['xtalk_latec']+gen_best['xtalk_yawma']:,.1f}) | {es.sigma:.3f} |\n"
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


if __name__ == "__main__":
    main()
