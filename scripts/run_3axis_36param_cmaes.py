#!/usr/bin/env python3
"""Stage 2: 36-Parameter 2-Wave CMA-ES (Popsize=74, 222 sims/gen on 112 cores).

Automatically runs ROI & convergence triage on the completed 43-parameter history
(logs/cmaes_3axis_43p_history.json), freezes the 7 lowest-ROI / converged parameters at
their 43P champion values, and runs active CMA-ES on the remaining 36 parameters
(2 TPA + 36 Haar-orthogonal mirrored pairs = 74 candidates -> 222 sims/gen = 2 waves).
"""

import argparse
import json
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

from scripts.run_3axis_42param_cmaes import (
    ALL_GENS_JSON as HIST_43_JSON,
    CHAMPION_JSON as CHAMP_43_JSON,
    LOG_DIR,
    PARAM_SPECS,
    STATE_JSON as STATE_43_JSON,
    _sim_job,
    ask_orthogonal_mirrored_nd,
)

TRIAGE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_triage.json")
TRIAGE_MD = os.path.join(LOG_DIR, "cmaes_3axis_36p_triage.md")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_36p_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_history.json")


def compute_43p_triage(n_freeze=7, save=True):
    """Compute ROI + convergence metrics from 43P history and pick n_freeze params to freeze."""
    hist = json.load(open(HIST_43_JSON, "r", encoding="utf-8"))
    champ_43 = json.load(open(CHAMP_43_JSON, "r", encoding="utf-8"))
    state_43 = json.load(open(STATE_43_JSON, "r", encoding="utf-8"))

    names = [s[0] for s in PARAM_SPECS]
    x0s = np.array([s[1] for s in PARAM_SPECS], dtype=float)
    lbs = np.array([s[2] for s in PARAM_SPECS], dtype=float)
    ubs = np.array([s[3] for s in PARAM_SPECS], dtype=float)
    stds = np.array([s[4] for s in PARAM_SPECS], dtype=float)
    mean_cur = state_43["mean"]

    X_all, y_all, y_apeo, y_lat, y_yaw, y_xt = [], [], [], [], [], []
    gen_means = []
    for rec in hist:
        cands = rec["candidates"]
        Xc = np.array([[c["params"][k] for k in names] for c in cands], dtype=float)
        yc = np.array([c["total_score"] for c in cands], dtype=float)
        top_half = np.argsort(yc)[: max(1, len(yc) // 2)]
        gen_means.append(np.mean(Xc[top_half], axis=0))
        for c in cands:
            if not c["aborted"] and c["total_score"] < 35000.0:
                X_all.append([c["params"][k] for k in names])
                y_all.append(c["total_score"])
                y_apeo.append(c["bode_apeo"])
                y_lat.append(c["bode_latec"])
                y_yaw.append(c["bode_yawma"])
                y_xt.append(c["xtalk_latec"] + c["xtalk_yawma"])

    X_all = np.asarray(X_all, dtype=float)
    y_all = np.asarray(y_all, dtype=float)
    gen_means = np.asarray(gen_means, dtype=float)

    Z = (X_all - np.mean(X_all, axis=0)) / (np.std(X_all, axis=0) + 1e-12)
    yc_norm = y_all - np.mean(y_all)
    beta = np.linalg.solve(Z.T @ Z + 10.0 * np.eye(len(names)), Z.T @ yc_norm)

    def _corr(a, b):
        sa, sb = np.std(a), np.std(b)
        return 0.0 if (sa < 1e-12 or sb < 1e-12) else float(abs(np.corrcoef(a, b)[0, 1]))

    max_axis_corr = np.array([
        max(_corr(X_all[:, i], y_apeo), _corr(X_all[:, i], y_lat), _corr(X_all[:, i], y_yaw), _corr(X_all[:, i], y_xt))
        for i in range(len(names))
    ])
    tail_k = min(4, len(gen_means))
    rec_drift = np.std(gen_means[-tail_k:], axis=0) / stds
    tot_shift = np.abs(np.array([mean_cur[k] for k in names]) - x0s) / stds

    ranked = sorted(range(len(names)), key=lambda i: abs(beta[i]))
    frozen_idx = [i for i in ranked if max_axis_corr[i] < 0.18 and rec_drift[i] < 0.20][:n_freeze]
    if len(frozen_idx) < n_freeze:
        for i in ranked:
            if i not in frozen_idx:
                frozen_idx.append(i)
                if len(frozen_idx) == n_freeze:
                    break
    frozen_set = set(frozen_idx)

    champ_params = champ_43["params"]
    gen_last_best = hist[-1]["gen_best"]["params"]
    sigma_scale = float(np.clip(state_43.get("sigma", 1.0), 0.35, 1.0))

    frozen_params = {}
    active_specs = []
    for i, (name, x0_def, lb, ub, std) in enumerate(PARAM_SPECS):
        if i in frozen_set:
            frozen_params[name] = float(np.clip(float(champ_params[name]), lb, ub))
        else:
            x_center = float(np.clip(float(mean_cur[name]), lb, ub))
            active_specs.append([name, x_center, float(lb), float(ub), float(std * sigma_scale)])

    audit_rows = []
    for r_i, i in enumerate(ranked, 1):
        name = names[i]
        audit_rows.append({
            "rank": r_i,
            "name": name,
            "roi_beta_pts_per_sigma": round(float(abs(beta[i])), 2),
            "max_axis_corr": round(float(max_axis_corr[i]), 3),
            "recent_drift_sigma": round(float(rec_drift[i]), 3),
            "total_shift_sigma": round(float(tot_shift[i]), 3),
            "champ_val": round(float(champ_params[name]), 6),
            "mean_val": round(float(mean_cur[name]), 6),
            "status": "FROZEN" if i in frozen_set else "ACTIVE",
        })

    triage = {
        "source_43p_gens": len(hist),
        "source_43p_global_best": float(champ_43["total_score"]),
        "source_43p_sigma": float(state_43["sigma"]),
        "frozen_params": frozen_params,
        "active_specs": active_specs,
        "seed_slot0_43p_champ": {s[0]: float(np.clip(float(champ_params[s[0]]), s[2], s[3])) for s in active_specs},
        "seed_slot1_43p_last_best": {s[0]: float(np.clip(float(gen_last_best[s[0]]), s[2], s[3])) for s in active_specs},
        "audit": audit_rows,
    }

    if save:
        with open(TRIAGE_JSON, "w", encoding="utf-8") as f:
            json.dump(triage, f, indent=2)
        with open(TRIAGE_MD, "w", encoding="utf-8") as f:
            f.write(
                f"# 43P -> 36P Triage Audit (Source: {len(hist)} gens, Best={champ_43['total_score']:,.2f}, σ={state_43['sigma']:.3f})\n\n"
                f"**Frozen (7):** {', '.join(f'`{k}`={v:.6g}' for k, v in frozen_params.items())}\n\n"
                "| Rank | Param | ROI (`pts/σ`) | MaxAxisCorr | RecDrift (`σ`) | TotalShift (`σ`) | 43P Champ | 43P Mean | Status |\n"
                "|---:|---|---:|---:|---:|---:|---:|---:|---|\n"
            )
            for row in audit_rows:
                f.write(
                    f"| {row['rank']} | `{row['name']}` | {row['roi_beta_pts_per_sigma']:.1f} | "
                    f"{row['max_axis_corr']:.3f} | {row['recent_drift_sigma']:.3f} | "
                    f"{row['total_shift_sigma']:.2f} | {row['champ_val']} | {row['mean_val']} | {row['status']} |\n"
                )
    return triage


def main():
    parser = argparse.ArgumentParser(description="74-Popsize Active Orthogonal-Mirrored TPA 3-Axis 36D CMA-ES (2 Waves on 112 Cores)")
    parser.add_argument("--gens", type=int, default=12, help="Target total 36P generations")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4361, help="Random seed")
    parser.add_argument("--resume", action="store_true", help="Resume from logs/cmaes_3axis_36p_history.json")
    parser.add_argument("--dry-triage", action="store_true", help="Compute/print 43P->36P triage without running simulations")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    if args.resume and os.path.exists(TRIAGE_JSON):
        triage = json.load(open(TRIAGE_JSON, "r", encoding="utf-8"))
        print(f"=== Loaded existing 36P triage from {TRIAGE_JSON} ===", flush=True)
    else:
        triage = compute_43p_triage(n_freeze=7, save=not args.dry_triage)
        print(f"=== Generated 43P -> 36P triage (source gens={triage['source_43p_gens']}, best={triage['source_43p_global_best']:,.2f}) ===", flush=True)

    print("Frozen (7):", ", ".join(f"{k}={v:.6g}" for k, v in triage["frozen_params"].items()), flush=True)
    if args.dry_triage:
        return

    active_specs = triage["active_specs"]
    active_names = [s[0] for s in active_specs]
    x0 = [float(s[1]) for s in active_specs]
    lb = [float(s[2]) for s in active_specs]
    ub = [float(s[3]) for s in active_specs]
    stds = [float(s[4]) for s in active_specs]
    frozen_items = [(k, f"{float(v):.8g}") for k, v in triage["frozen_params"].items()]

    popsize = 2 + 2 * len(active_names)  # 74 -> 222 sims/gen = 2 waves on 112 cores
    modes = ("APEO", "LatEC", "YawMA")

    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        "popsize": popsize,
        "bounds": [lb, ub],
        "CMA_stds": stds,
        "CMA_active": True,
        "CMA_elitist": False,
        "AdaptSigma": cma.sigma_adaptation.CMAAdaptSigmaTPA,
        "verbose": -9,
        "seed": args.seed,
    })

    global_best_score = float("inf")
    global_best_entry = None
    history_records = []
    start_gen = 1

    if args.resume and os.path.exists(ALL_GENS_JSON):
        with open(ALL_GENS_JSON, "r", encoding="utf-8") as f:
            history_records = json.load(f)
        for rec in history_records:
            _ = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=True)
            logged_cands = [
                np.clip(np.array([float(c["params"][k]) for k in active_names], dtype=float), lb, ub)
                for c in rec["candidates"]
            ]
            es.tell(logged_cands, [float(c["total_score"]) for c in rec["candidates"]])
            gb = rec["gen_best"]
            if float(gb["total_score"]) < global_best_score:
                global_best_score = float(gb["total_score"])
                global_best_entry = gb
        start_gen = len(history_records) + 1
        print(f"=== Resumed 36D CMA-ES from Gen {len(history_records)} (sigma={es.sigma:.4f}, GlobalBest={global_best_score:,.2f}) -> Gen {start_gen} ===", flush=True)

    with open(LEDGER_PATH, "w", encoding="utf-8") as f:
        f.write(
            f"# 3-Axis 36-Parameter CMA-ES Ledger (Popsize={popsize}: 2 TPA + {len(active_names)} Mirrored Pairs = 222 sims/gen)\n\n"
            f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Frozen(7): {', '.join(triage['frozen_params'].keys())}\n\n"
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
        f"=== Starting 36D 3-Axis CMA-ES (Pop={popsize}, Active=True, Zero Surrogate/GP, "
        f"Workers={args.workers}, Gens={start_gen}..{args.gens}) ===",
        flush=True,
    )

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(start_gen, args.gens + 1):
            t0 = time.time()
            candidates = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=True)
            if gen == 1:
                candidates[0] = np.clip(np.array([triage["seed_slot0_43p_champ"][k] for k in active_names], dtype=float), lb, ub)
                candidates[1] = np.clip(np.array([triage["seed_slot1_43p_last_best"][k] for k in active_names], dtype=float), lb, ub)

            jobs = []
            for idx, x in enumerate(candidates):
                overrides = [(name, f"{float(val):.8g}") for name, val in zip(active_names, x)] + frozen_items
                for m in modes:
                    jobs.append((idx, m, overrides))

            per_cand = {idx: {} for idx in range(popsize)}
            futs = [pool.submit(_sim_job, j) for j in jobs]
            n_total_jobs = len(jobs)
            for done_cnt, fut in enumerate(as_completed(futs), 1):
                r = fut.result()
                per_cand[r["cand_idx"]][r["mode"]] = r
                if done_cnt % 37 == 0 or done_cnt == n_total_jobs:
                    print(
                        f"  [36P Gen {gen:02d}] {done_cnt:3d}/{n_total_jobs} sims done ({time.time() - t0:.0f}s) — "
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

                full_params = dict(triage["frozen_params"])
                for k, v in zip(active_names, candidates[idx]):
                    full_params[k] = round(float(v), 6)

                cand_summaries.append({
                    "gen": gen,
                    "slot": idx,
                    "slot_type": "TPA+" if idx == 0 else ("TPA-" if idx == 1 else f"Pair{(idx-2)//2 + 1}{'+' if idx % 2 == 0 else '-'}"),
                    "params": full_params,
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
                    "mean": {k: float(v) for k, v in zip(active_names, es.mean)},
                    "frozen_params": triage["frozen_params"],
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
                f"[36P Gen {gen:02d}/{args.gens:02d} | {dt:5.1f}s | σ={es.sigma:.3f}] "
                f"GenBest={gen_best['total_score']:,.2f} ({gen_best['slot_type']}) "
                f"[APEO={gen_best['bode_apeo']:,.1f}, LatEC={gen_best['bode_latec']:,.1f}, YawMA={gen_best['bode_yawma']:,.1f}, "
                f"XTalk={gen_best['xtalk_latec']+gen_best['xtalk_yawma']:,.1f} ({gen_best['pitch_dev_latec_deg']:.2f}°/{gen_best['pitch_dev_yawma_deg']:.2f}°)] "
                f"| GlobalBest={global_best_score:,.2f}",
                flush=True,
            )


if __name__ == "__main__":
    main()
