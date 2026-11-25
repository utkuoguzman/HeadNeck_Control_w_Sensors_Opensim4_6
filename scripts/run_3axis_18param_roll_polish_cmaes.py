#!/usr/bin/env python3
"""Stage 5: 18-Parameter Roll-Subspace 1-Wave CSA CMA-ES Polish (Pop=36, 108 sims/gen).

Option 2 + Option 1:
  - Freezes all 35 Pitch, Yaw, and Fusimotor parameters at the 36P-v2 Gen 30 Champion (10,556.77).
  - Unlocks newly C++-decoupled Kp_proprioception_roll & Kp_buckle_roll alongside 16 Roll params (18 active).
  - Lowers z_phas_roll and z_ccr_roll lower bounds to 0.0.
  - Uses standard CSA (no TPA) with sigma0=0.45 and 18 pure orthogonal mirrored pairs (Pop=36 -> 108 sims/gen = 1 wave).
  - Still evaluates all 3 axes (APEO, LatEC, YawMA) so Pitch/Yaw and cross-talk cannot regress.
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
    _sim_job,
    ask_orthogonal_mirrored_nd,
)

CHAMP_V2_JSON = os.path.join(LOG_DIR, "cmaes_3axis_36p_v2_champion.json")

CONFIG_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_config.json")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_history.json")

# (name, parent_key_in_v2_champ, lb, ub, std)
ACTIVE_ROLL_18_SPECS = [
    ("K_sc_roll",               "K_sc_roll",          1.50,  9.00,  0.50),
    ("z_sc_roll",               "z_sc_roll",          0.002, 0.050, 0.0040),
    ("p_sc_roll",               "p_sc_roll",          30.0,  160.0, 10.0),
    ("K_phas_roll",             "K_phas_roll",        4.00,  24.0,  1.80),
    ("z_phas_roll",             "z_phas_roll",        0.0,   0.035, 0.0030),
    ("p_phas_roll",             "p_phas_roll",        1.50,  14.0,  0.90),
    ("K_ton_roll",              "K_ton_roll",         0.15,  3.50,  0.22),
    ("p_ton_roll",              "p_ton_roll",         0.60,  10.0,  0.70),
    ("Kp_prop_roll",            "Kp_prop_roll",       0.50,  6.50,  0.45),
    ("z_ccr_roll",              "z_ccr_roll",         0.0,   0.035, 0.0025),
    ("p_ccr_roll",              "p_ccr_roll",         30.0,  160.0, 10.0),
    ("Kp_proprioception_roll",  "Kp_proprioception",  0.003, 0.120, 0.0080),
    ("Kp_task_roll",            "Kp_task_roll",       0.30,  4.00,  0.28),
    ("Kd_task_roll",            "Kd_task_roll",       0.40,  3.50,  0.25),
    ("Ki_task_roll",            "Ki_task_roll",       0.02,  2.50,  0.15),
    ("tau_lead_pole_roll",      "tau_lead_pole_roll", 0.004, 0.035, 0.0022),
    ("Kp_buckle_roll",          "Kp_buckle",          0.20,  4.00,  0.30),
    ("Kd_buckle_roll",          "Kd_buckle_roll",     0.03,  0.55,  0.035),
]


def build_or_load_roll18_config(save=True):
    """Build or load the 18 active Roll + 35 frozen Pitch/Yaw config from 36P-v2 Champion."""
    if os.path.exists(CONFIG_JSON):
        return json.load(open(CONFIG_JSON, "r", encoding="utf-8"))

    champ_v2 = json.load(open(CHAMP_V2_JSON, "r", encoding="utf-8"))
    champ_p = dict(champ_v2["params"])
    champ_p["Kp_proprioception_yaw"] = float(champ_p["Kp_proprioception"])
    champ_p["Kp_buckle_yaw"] = float(champ_p["Kp_buckle"])

    active_specs = []
    active_names = []
    for name, parent_k, lb, ub, std in ACTIVE_ROLL_18_SPECS:
        x_center = float(np.clip(float(champ_p[parent_k]), float(lb), float(ub)))
        active_specs.append([name, x_center, float(lb), float(ub), float(std)])
        active_names.append(name)

    active_set = set(active_names)
    frozen_params = {
        k: float(v)
        for k, v in champ_p.items()
        if k not in active_set
    }

    assert len(active_specs) == 18, f"Expected 18 active, got {len(active_specs)}"
    assert len(frozen_params) == 35, f"Expected 35 frozen, got {len(frozen_params)}"
    assert len(set(frozen_params) & active_set) == 0, "Overlap between frozen and active params"
    assert len(set(frozen_params) | active_set) == 53, f"Expected 53 total params, got {len(set(frozen_params) | active_set)}"

    seed_slot0 = [float(s[1]) for s in active_specs]
    seed_slot1 = list(seed_slot0)
    idx_zpr = active_names.index("z_phas_roll")
    idx_kpr = active_names.index("Kp_proprioception_roll")
    seed_slot1[idx_zpr] = 0.0
    seed_slot1[idx_kpr] = float(np.clip(0.70 * seed_slot0[idx_kpr], active_specs[idx_kpr][2], active_specs[idx_kpr][3]))

    cfg = {
        "source_v2_gen": int(champ_v2["gen"]),
        "source_v2_global_best": float(champ_v2["total_score"]),
        "active_specs": active_specs,
        "frozen_params": frozen_params,
        "seed_slot0_champ": {k: float(v) for k, v in zip(active_names, seed_slot0)},
        "seed_slot1_zerophas_probe": {k: float(v) for k, v in zip(active_names, seed_slot1)},
    }
    if save:
        with open(CONFIG_JSON, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def main():
    parser = argparse.ArgumentParser(description="36-Popsize 18D Roll-Subspace 1-Wave CSA CMA-ES (108 sims/gen)")
    parser.add_argument("--gens", type=int, default=45, help="Target total Roll18P generations")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4318, help="Random seed")
    parser.add_argument("--resume", action="store_true", help="Resume from logs/cmaes_3axis_18p_roll_history.json")
    parser.add_argument("--dry-check", action="store_true", help="Verify 18 active + 35 frozen split without running sims")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    cfg = build_or_load_roll18_config(save=not args.dry_check)
    active_specs = cfg["active_specs"]
    active_names = [s[0] for s in active_specs]
    x0 = [float(s[1]) for s in active_specs]
    lb = [float(s[2]) for s in active_specs]
    ub = [float(s[3]) for s in active_specs]
    stds = [float(s[4]) for s in active_specs]
    frozen_items = [(k, f"{float(v):.8g}") for k, v in cfg["frozen_params"].items()]

    print(
        f"=== Roll18P Subspace Setup: {len(active_names)} active + {len(frozen_items)} frozen "
        f"(seeded from 36P-v2 Best={cfg['source_v2_global_best']:,.2f}, Gen {cfg['source_v2_gen']}) ===",
        flush=True,
    )
    if args.dry_check:
        print("Active(18):", active_names)
        print("Frozen(35):", cfg["frozen_params"])
        return

    popsize = 2 * len(active_names)  # 36 -> 108 sims/gen = 1 wave on 112 cores
    modes = ("APEO", "LatEC", "YawMA")

    es = cma.CMAEvolutionStrategy(x0, 0.45, {
        "popsize": popsize,
        "bounds": [lb, ub],
        "CMA_stds": stds,
        "CMA_active": True,
        "CMA_elitist": False,
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
            _ = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=False)
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
        print(f"=== Resumed Roll18P from Gen {len(history_records)} (sigma={es.sigma:.4f}, GlobalBest={global_best_score:,.2f}) -> Gen {start_gen} ===", flush=True)

    with open(LEDGER_PATH, "w", encoding="utf-8") as f:
        f.write(
            f"# 3-Axis 18-Param Roll-Subspace 1-Wave CSA CMA-ES Ledger (Pop={popsize}, 108 sims/gen)\n\n"
            f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Active(18): {', '.join(active_names)}\n\n"
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
        f"=== Starting Roll18P 3-Axis CSA CMA-ES (Pop={popsize}, 108 sims/gen, CSA sigma0=0.45, "
        f"Workers={args.workers}, Gens={start_gen}..{args.gens}) ===",
        flush=True,
    )

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for gen in range(start_gen, args.gens + 1):
            t0 = time.time()
            candidates = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=False)
            if gen == 1:
                candidates[0] = np.clip(np.array([cfg["seed_slot0_champ"][k] for k in active_names], dtype=float), lb, ub)
                candidates[1] = np.clip(np.array([cfg["seed_slot1_zerophas_probe"][k] for k in active_names], dtype=float), lb, ub)

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
                if done_cnt % 36 == 0 or done_cnt == n_total_jobs:
                    print(
                        f"  [Roll18P Gen {gen:02d}] {done_cnt:3d}/{n_total_jobs} sims done ({time.time() - t0:.0f}s) — "
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

                if gen == 1 and idx == 0:
                    stype = "SeedChamp"
                elif gen == 1 and idx == 1:
                    stype = "ZeroPhasProbe"
                else:
                    stype = f"Pair{(idx // 2) + 1}{'+' if idx % 2 == 0 else '-'}"

                cand_summaries.append({
                    "gen": gen,
                    "slot": idx,
                    "slot_type": stype,
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
                f"[Roll18P Gen {gen:02d}/{args.gens:02d} | {dt:5.1f}s | σ={es.sigma:.3f}] "
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
