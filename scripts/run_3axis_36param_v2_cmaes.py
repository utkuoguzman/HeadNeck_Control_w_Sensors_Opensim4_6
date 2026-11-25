#!/usr/bin/env python3
"""Stage 3: 36-Parameter v2 2-Wave CMA-ES with Axis-Decoupled Compensators (Pop=74, 222 sims/gen).

Decouples 3 previously-shared axes (+6 params):
  - tau_lead_pole -> (tau_lead_pole [pitch], tau_lead_pole_roll, tau_lead_pole_yaw)
  - p_ccr         -> (p_ccr [pitch], p_ccr_roll, p_ccr_yaw)
  - Kd_buckle     -> (Kd_buckle [pitch], Kd_buckle_roll, Kd_buckle_yaw)

Freezes 13 lowest-ROI / converged parameters (7 from 43P->36P + 6 from 36P audit) at
the 36P Champion values, leaving 36 active parameters -> Pop=74 (2 TPA + 36 mirrored pairs)
-> 222 sims/gen = 2 waves on 112 cores.
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
    LOG_DIR,
    PARAM_SPECS,
    _sim_job,
    ask_orthogonal_mirrored_nd,
)

CHAMP_36_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_champion.json")
STATE_36_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_state.json")
HIST_36_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_history.json")

TRIAGE_V2_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_v2_triage.json")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_v2_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_v2_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_36p_v2_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_v2_history.json")

FROZEN_13_DEFAULTS = {
    # Original 7 frozen from 43P -> 36P triage
    "K_phas": 10.7113,
    "p_ton": 20.2239,
    "K_sc_yaw": 1.4433,
    "z_sc_yaw": 0.01933,
    "z_ccr_roll": 0.007426,
    "Kp_task_pitch": 4.65154,
    "K_gamma_dyn": 0.857744,
    # Additional 6 lowest-ROI / converged from 36P audit
    "Kp_prop_pitch": 0.50201,
    "z_ccr": 0.01973,
    "Kp_task_yaw": 0.39575,
    "p_ton_yaw": 16.34,
    "Kp_task_roll": 1.0968,
    "Ki_task_roll": 0.47524,
}

NEW_AXIS_SPECS = [
    ("tau_lead_pole_roll", 0.015804, 0.004, 0.030, 0.0022),
    ("tau_lead_pole_yaw",  0.015804, 0.004, 0.030, 0.0022),
    ("p_ccr_roll",         83.865,   40.0,  160.0, 12.0),
    ("p_ccr_yaw",          83.865,   40.0,  160.0, 12.0),
    ("Kd_buckle_roll",     0.18766,  0.03,  0.50,  0.04),
    ("Kd_buckle_yaw",      0.18766,  0.03,  0.50,  0.04),
]

PARENT_MAP = {
    "tau_lead_pole_roll": "tau_lead_pole",
    "tau_lead_pole_yaw": "tau_lead_pole",
    "p_ccr_roll": "p_ccr",
    "p_ccr_yaw": "p_ccr",
    "Kd_buckle_roll": "Kd_buckle",
    "Kd_buckle_yaw": "Kd_buckle",
}


def _with_decoupled_aliases(pdict):
    out = dict(pdict)
    for child, parent in PARENT_MAP.items():
        if child not in out and parent in out:
            out[child] = float(out[parent])
    return out


def build_or_load_v2_config(save=True):
    """Build or load the 36 active + 13 frozen split seeded from 36P results."""
    if os.path.exists(TRIAGE_V2_JSON):
        return json.load(open(TRIAGE_V2_JSON, "r", encoding="utf-8"))

    champ_36 = json.load(open(CHAMP_36_JSON, "r", encoding="utf-8"))
    state_36 = json.load(open(STATE_36_JSON, "r", encoding="utf-8"))
    hist_36 = json.load(open(HIST_36_JSON, "r", encoding="utf-8"))

    champ_p = _with_decoupled_aliases(champ_36["params"])
    last_best_p = _with_decoupled_aliases(hist_36[-1]["gen_best"]["params"])
    mean_p = _with_decoupled_aliases({**champ_p, **state_36.get("mean", {})})
    sigma_scale = float(np.clip(state_36.get("sigma", 0.88), 0.45, 1.0))

    spec_map = {s[0]: list(s) for s in PARAM_SPECS}
    # Widen p_sc_roll UB from 100.0 -> 140.0 (Champ reached 95.85)
    spec_map["p_sc_roll"][3] = 140.0

    frozen_params = {}
    for k, default_v in FROZEN_13_DEFAULTS.items():
        _, _, lb, ub, _ = spec_map[k]
        raw_v = float(champ_p.get(k, default_v))
        frozen_params[k] = float(np.clip(raw_v, lb, ub))

    active_specs = []
    for name, _, lb, ub, std in PARAM_SPECS:
        if name in FROZEN_13_DEFAULTS:
            continue
        _, _, lb, ub, std = spec_map[name]
        x_center = float(np.clip(float(mean_p.get(name, _)), lb, ub))
        active_specs.append([name, x_center, float(lb), float(ub), float(std * sigma_scale)])

    for name, def_x0, lb, ub, std in NEW_AXIS_SPECS:
        x_center = float(np.clip(float(mean_p.get(name, def_x0)), lb, ub))
        active_specs.append([name, x_center, float(lb), float(ub), float(std)])

    assert len(frozen_params) == 13, f"Expected 13 frozen, got {len(frozen_params)}"
    assert len(active_specs) == 36, f"Expected 36 active, got {len(active_specs)}"

    cfg = {
        "source_36p_gens": len(hist_36),
        "source_36p_global_best": float(champ_36["total_score"]),
        "source_36p_sigma": float(state_36.get("sigma", 1.0)),
        "frozen_params": frozen_params,
        "active_specs": active_specs,
        "seed_slot0_36p_champ": {s[0]: float(np.clip(float(champ_p[s[0]]), s[2], s[3])) for s in active_specs},
        "seed_slot1_36p_last_best": {s[0]: float(np.clip(float(last_best_p[s[0]]), s[2], s[3])) for s in active_specs},
    }
    if save:
        with open(TRIAGE_V2_JSON, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def main():
    parser = argparse.ArgumentParser(description="74-Popsize Axis-Decoupled 36D v2 CMA-ES (2 Waves on 112 Cores)")
    parser.add_argument("--gens", type=int, default=24, help="Target total 36P-v2 generations")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4362, help="Random seed")
    parser.add_argument("--resume", action="store_true", help="Resume from logs/cmaes_3axis_36p_v2_history.json")
    parser.add_argument("--dry-check", action="store_true", help="Verify 36 active + 13 frozen split without running sims")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    cfg = build_or_load_v2_config(save=not args.dry_check)
    active_specs = cfg["active_specs"]
    active_names = [s[0] for s in active_specs]
    x0 = [float(s[1]) for s in active_specs]
    lb = [float(s[2]) for s in active_specs]
    ub = [float(s[3]) for s in active_specs]
    stds = [float(s[4]) for s in active_specs]
    frozen_items = [(k, f"{float(v):.8g}") for k, v in cfg["frozen_params"].items()]

    print(
        f"=== 36P-v2 Axis-Decoupled Setup: {len(active_names)} active + {len(frozen_items)} frozen "
        f"(seeded from 36P Best={cfg['source_36p_global_best']:,.2f}, Gen {cfg['source_36p_gens']}) ===",
        flush=True,
    )
    if args.dry_check:
        print("Active(36):", active_names)
        print("Frozen(13):", cfg["frozen_params"])
        return

    popsize = 2 + 2 * len(active_names)  # 74 -> 222 sims/gen
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
        print(f"=== Resumed 36P-v2 from Gen {len(history_records)} (sigma={es.sigma:.4f}, GlobalBest={global_best_score:,.2f}) -> Gen {start_gen} ===", flush=True)

    with open(LEDGER_PATH, "w", encoding="utf-8") as f:
        f.write(
            f"# 3-Axis 36-Param v2 CMA-ES Ledger (Axis-Decoupled tau_lead_pole_*/p_ccr_*/Kd_buckle_*, Pop={popsize}, 222 sims/gen)\n\n"
            f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Frozen(13): {', '.join(cfg['frozen_params'].keys())}\n\n"
            "| Gen | Best Total | APEO Bode | LatEC Bode | YawMA Bode | XTalk Post-5s (pts) | HF Jitter (pts) | σ |\n"
            "|---:|---:|---:|---:|---:|---:|---:|---:|\n"
        )
        for rec in history_records:
            gb = rec["gen_best"]
            xt_sum = gb.get("xtalk_apeo", 0.0) + gb["xtalk_latec"] + gb["xtalk_yawma"]
            hf_sum = gb.get("hf_apeo", 0.0) + gb.get("hf_latec", 0.0) + gb.get("hf_yawma", 0.0)
            f.write(
                f"| {rec['gen']} | {gb['total_score']:,.2f} | {gb['bode_apeo']:,.2f} | "
                f"{gb['bode_latec']:,.2f} | {gb['bode_yawma']:,.2f} | "
                f"{xt_sum:,.1f} | {hf_sum:,.1f} | {rec['sigma']:.3f} |\n"
            )

    print(
        f"=== Starting 36P-v2 3-Axis CMA-ES (Pop={popsize}, Active=True, Zero Surrogate/GP, "
        f"Workers={args.workers}, Gens={start_gen}..{args.gens}) ===",
        flush=True,
    )

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(start_gen, args.gens + 1):
            t0 = time.time()
            candidates = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=True)
            if gen == 1:
                candidates[0] = np.clip(np.array([cfg["seed_slot0_36p_champ"][k] for k in active_names], dtype=float), lb, ub)
                candidates[1] = np.clip(np.array([cfg["seed_slot1_36p_last_best"][k] for k in active_names], dtype=float), lb, ub)

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
                        f"  [36Pv2 Gen {gen:02d}] {done_cnt:3d}/{n_total_jobs} sims done ({time.time() - t0:.0f}s) — "
                        f"last cand={r['cand_idx']:02d} {r['mode']}: Bode={r['classic_bode_score']:,.1f}, "
                        f"XTalk={r['pitch_crosstalk_penalty']:,.1f}, HF={r['hf_penalty']:,.1f}, Total={r['total_score']:,.1f}",
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
                xtalk_apeo = res_apeo["pitch_crosstalk_penalty"]
                xtalk_latec = res_latec["pitch_crosstalk_penalty"]
                xtalk_yawma = res_yawma["pitch_crosstalk_penalty"]
                hf_apeo = res_apeo["hf_penalty"]
                hf_latec = res_latec["hf_penalty"]
                hf_yawma = res_yawma["hf_penalty"]
                total_score = (
                    bode_apeo + bode_latec + bode_yawma
                    + xtalk_apeo + xtalk_latec + xtalk_yawma
                    + hf_apeo + hf_latec + hf_yawma
                )
                scores.append(total_score)

                full_params = dict(cfg["frozen_params"])
                for k, v in zip(active_names, candidates[idx]):
                    full_params[k] = float(f"{float(v):.8g}")

                cand_summaries.append({
                    "gen": gen,
                    "slot": idx,
                    "slot_type": "TPA+" if idx == 0 else ("TPA-" if idx == 1 else f"Pair{(idx-2)//2 + 1}{'+' if idx % 2 == 0 else '-'}"),
                    "params": full_params,
                    "total_score": round(total_score, 2),
                    "bode_apeo": round(bode_apeo, 2),
                    "bode_latec": round(bode_latec, 2),
                    "bode_yawma": round(bode_yawma, 2),
                    "xtalk_apeo": round(xtalk_apeo, 2),
                    "xtalk_latec": round(xtalk_latec, 2),
                    "xtalk_yawma": round(xtalk_yawma, 2),
                    "hf_apeo": round(hf_apeo, 2),
                    "hf_latec": round(hf_latec, 2),
                    "hf_yawma": round(hf_yawma, 2),
                    "roll_dev_apeo_deg": round(res_apeo["post_roll_max_deg"], 3),
                    "roll_dev_yawma_deg": round(res_yawma["post_roll_max_deg"], 3),
                    "pitch_dev_latec_deg": round(res_latec["post_pitch_dev_max_deg"], 3),
                    "pitch_dev_yawma_deg": round(res_yawma["post_pitch_dev_max_deg"], 3),
                    "hf_rms_apeo": round(res_apeo["hf_noise_rms"], 3),
                    "hf_rms_latec": round(res_latec["hf_noise_rms"], 3),
                    "hf_rms_yawma": round(res_yawma["hf_noise_rms"], 3),
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
                    "frozen_params": cfg["frozen_params"],
                    "global_best": global_best_entry,
                }, f, indent=2)

            xt_tot = gen_best["xtalk_apeo"] + gen_best["xtalk_latec"] + gen_best["xtalk_yawma"]
            hf_tot = gen_best["hf_apeo"] + gen_best["hf_latec"] + gen_best["hf_yawma"]
            with open(LEDGER_PATH, "a", encoding="utf-8") as f:
                f.write(
                    f"| {gen} | {gen_best['total_score']:,.2f} | {gen_best['bode_apeo']:,.2f} | "
                    f"{gen_best['bode_latec']:,.2f} | {gen_best['bode_yawma']:,.2f} | "
                    f"{xt_tot:,.1f} | {hf_tot:,.1f} | {es.sigma:.3f} |\n"
                )

            dt = time.time() - t0
            print(
                f"[36Pv2 Gen {gen:02d}/{args.gens:02d} | {dt:5.1f}s | σ={es.sigma:.3f}] "
                f"GenBest={gen_best['total_score']:,.2f} ({gen_best['slot_type']}) "
                f"[APEO={gen_best['bode_apeo']:,.1f}, LatEC={gen_best['bode_latec']:,.1f}, YawMA={gen_best['bode_yawma']:,.1f}, "
                f"XTalk={xt_tot:,.1f} (rA={gen_best['roll_dev_apeo_deg']:.2f}°/rY={gen_best['roll_dev_yawma_deg']:.2f}°/"
                f"pL={gen_best['pitch_dev_latec_deg']:.2f}°/pY={gen_best['pitch_dev_yawma_deg']:.2f}°), "
                f"HF={hf_tot:,.1f} ({gen_best['hf_rms_apeo']:.2f}/{gen_best['hf_rms_latec']:.2f}/{gen_best['hf_rms_yawma']:.2f}°/s)] "
                f"| GlobalBest={global_best_score:,.2f}",
                flush=True,
            )


if __name__ == "__main__":
    main()
