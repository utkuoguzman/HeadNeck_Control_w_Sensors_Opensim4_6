#!/usr/bin/env python3
"""Stage 8 (Option A Stage 1): Fast LatEC-Only 18D Roll Clean-Basin CSA CMA-ES (Pop=36, 36 sims/gen).

Evaluates ONLY mode='LatEC' (36 sims/gen on 112 cores, 1 wave, zero Pitch/Yaw guardrail drag)
while keeping all 35 Pitch/Yaw/Fusimotor parameters frozen at Gen 35.
Seeded at the best clean Roll candidate from HN_RollCln (logs/best_clean_roll_seed.json)
with a hard stability wall above 3.5 deg/s Roll HF RMS.
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
from scripts.run_3axis_18p_roll_clean_basin_cmaes import ROLL_CLEAN_18_SPECS

SEED_JSON = os.path.join(LOG_DIR, "best_clean_roll_seed.json")

CONFIG_JSON = os.path.join(LOG_DIR, "cmaes_latec_only_18p_clean_config.json")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_latec_only_18p_clean_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_latec_only_18p_clean_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_latec_only_18p_clean_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_latec_only_18p_clean_history.json")


def build_or_load_config(save=True):
    """Build or load the 18 active Roll + 35 frozen Pitch/Yaw/Fusimotor config from best_clean_roll_seed.json."""
    if os.path.exists(CONFIG_JSON):
        return json.load(open(CONFIG_JSON, "r", encoding="utf-8"))

    seed_data = json.load(open(SEED_JSON, "r", encoding="utf-8"))
    p1 = {k: float(v) for k, v in seed_data["params"].items()}

    if "second_seed" in seed_data and "params" in seed_data["second_seed"]:
        p2 = {k: float(v) for k, v in seed_data["second_seed"]["params"].items()}
        second_label = f"Gen{seed_data['second_seed']['source_gen']}Slot{seed_data['second_seed']['source_slot']}"
    else:
        p2 = {s[0]: float(s[1]) for s in ROLL_CLEAN_18_SPECS}
        second_label = "GROUP_OSIM_ROLL18"

    active_specs = []
    active_names = []
    seed_slot1 = []
    for name, _x0_osim, lb, ub, std in ROLL_CLEAN_18_SPECS:
        x_center = float(np.clip(float(p1[name]), float(lb), float(ub)))
        active_specs.append([name, x_center, float(lb), float(ub), float(std)])
        active_names.append(name)
        s1_val = float(np.clip(float(p2.get(name, p1[name])), float(lb), float(ub)))
        seed_slot1.append(s1_val)

    active_set = set(active_names)
    frozen_params = {k: float(v) for k, v in p1.items() if k not in active_set}

    assert len(active_specs) == 18, f"Expected 18 active, got {len(active_specs)}"
    assert len(frozen_params) == 35, f"Expected 35 frozen, got {len(frozen_params)}"
    assert len(set(frozen_params) & active_set) == 0, "Overlap between frozen and active params"
    assert len(set(frozen_params) | active_set) == 53, f"Expected 53 total params, got {len(set(frozen_params) | active_set)}"

    seed_slot0 = [float(s[1]) for s in active_specs]
    cfg = {
        "source_gen": int(seed_data.get("source_gen", 0)),
        "source_slot": int(seed_data.get("source_slot", 0)),
        "source_bode_latec": float(seed_data.get("bode_latec", 0.0)),
        "source_hf_rms_latec": float(seed_data.get("hf_rms_latec", 0.0)),
        "second_seed_label": second_label,
        "active_specs": active_specs,
        "frozen_params": frozen_params,
        "seed_slot0": {k: float(v) for k, v in zip(active_names, seed_slot0)},
        "seed_slot1": {k: float(v) for k, v in zip(active_names, seed_slot1)},
    }
    if save:
        with open(CONFIG_JSON, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def compute_latec_clean_rank_score(bode_latec, xtalk_latec, hf_latec, hf_rms_latec):
    """LatEC canonical total + hard stability wall above 3.5 deg/s Roll HF RMS."""
    latec_canon = float(bode_latec + xtalk_latec + hf_latec)
    roll_wall = 0.0 if hf_rms_latec <= 3.5 else (20000.0 + 3000.0 * (hf_rms_latec - 3.5) ** 2)
    return float(latec_canon + roll_wall)


def main():
    parser = argparse.ArgumentParser(description="Fast LatEC-Only 18D Roll Clean-Basin CSA CMA-ES (36 sims/gen)")
    parser.add_argument("--gens", type=int, default=80, help="Target total generations")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4618, help="Random seed")
    parser.add_argument("--resume", action="store_true", help="Resume from logs/cmaes_latec_only_18p_clean_history.json")
    parser.add_argument("--dry-check", action="store_true", help="Verify 18 active + 35 frozen split without running sims")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    np.random.seed(args.seed)

    cfg = build_or_load_config(save=not args.dry_check)
    active_specs = cfg["active_specs"]
    active_names = [s[0] for s in active_specs]
    x0 = [float(s[1]) for s in active_specs]
    lb = [float(s[2]) for s in active_specs]
    ub = [float(s[3]) for s in active_specs]
    stds = [float(s[4]) for s in active_specs]
    frozen_items = [(k, f"{float(v):.8g}") for k, v in cfg["frozen_params"].items()]

    print(
        f"=== LatEC-Only Clean-Basin 18D Setup: {len(active_names)} active + {len(frozen_items)} frozen "
        f"(Seed Gen{cfg['source_gen']}Slot{cfg['source_slot']}: BodeL={cfg['source_bode_latec']:.2f}, RollHF={cfg['source_hf_rms_latec']:.3f} deg/s) ===",
        flush=True,
    )
    if args.dry_check:
        print("Active(18):", active_names)
        print("Frozen(35):", len(frozen_items))
        return

    popsize = 2 * len(active_names)  # 36 -> 36 LatEC sims/gen on 112 cores

    es = cma.CMAEvolutionStrategy(x0, 0.50, {
        "popsize": popsize,
        "bounds": [lb, ub],
        "CMA_stds": stds,
        "CMA_active": True,
        "CMA_elitist": False,
        "verbose": -9,
        "seed": args.seed,
    })

    history = []
    global_best = None
    start_gen = 1

    if args.resume and os.path.exists(ALL_GENS_JSON):
        history = json.load(open(ALL_GENS_JSON, "r", encoding="utf-8"))
        for g_rec in history:
            cands = g_rec["candidates"]
            X_hist = [np.asarray([c["params"][k] for k in active_names], dtype=float) for c in cands]
            f_hist = [float(c["rank_score"]) for c in cands]
            es.ask()
            es.tell(X_hist, f_hist)
            for c in cands:
                if global_best is None or c["rank_score"] < global_best["rank_score"]:
                    global_best = c
        start_gen = len(history) + 1
        print(f"=== Resumed from Gen {len(history)} (sigma={es.sigma:.4f}) ===", flush=True)

    if not os.path.exists(LEDGER_PATH) or start_gen == 1:
        with open(LEDGER_PATH, "w", encoding="utf-8") as f:
            f.write("# LatEC-Only 18-Param Clean-Basin 1-Wave CSA CMA-ES Ledger (Pop=36, 36 sims/gen)\n\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Active(18): {', '.join(active_names)}\n\n")
            f.write("| Gen | Rank Score | LatEC Canon | LatEC Bode | LatEC XTalk | LatEC HF (pts) | Roll HF (°/s) | Clean<=3.5 | Elapsed (s) | σ |\n")
            f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")

    for gen in range(start_gen, args.gens + 1):
        t0 = time.time()
        candidates = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=False)
        if gen == 1:
            candidates[0] = np.asarray([cfg["seed_slot0"][k] for k in active_names], dtype=float)
            candidates[1] = np.asarray([cfg["seed_slot1"][k] for k in active_names], dtype=float)

        jobs = []
        for idx, vec in enumerate(candidates):
            ovr = frozen_items + [(k, f"{float(v):.8g}") for k, v in zip(active_names, vec)]
            jobs.append((idx, "LatEC", ovr))

        results_by_cand = {}
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            futs = [ex.submit(_sim_job, job) for job in jobs]
            for fut in as_completed(futs):
                res = fut.result()
                results_by_cand[res["cand_idx"]] = res

        gen_records = []
        f_vals = []
        clean_count = 0
        for idx, vec in enumerate(candidates):
            r_l = results_by_cand[idx]
            bode_l = float(r_l["classic_bode_score"])
            xtalk_l = float(r_l["pitch_crosstalk_penalty"])
            hf_l = float(r_l["hf_penalty"])
            latec_canon = bode_l + xtalk_l + hf_l
            hf_rms_l = float(r_l["hf_noise_rms"])
            aborted = bool(r_l["aborted"])

            if (not aborted) and hf_rms_l <= 3.5:
                clean_count += 1

            rank_score = compute_latec_clean_rank_score(bode_l, xtalk_l, hf_l, hf_rms_l)
            f_vals.append(rank_score)

            full_p = dict(cfg["frozen_params"])
            for k, v in zip(active_names, vec):
                full_p[k] = float(v)

            rec = {
                "gen": gen,
                "slot": idx,
                "params": full_p,
                "rank_score": round(rank_score, 2),
                "latec_canon": round(latec_canon, 2),
                "bode_latec": round(bode_l, 2),
                "xtalk_latec": round(xtalk_l, 2),
                "hf_latec": round(hf_l, 2),
                "hf_rms_latec": round(hf_rms_l, 3),
                "aborted": aborted,
            }
            gen_records.append(rec)
            if global_best is None or rec["rank_score"] < global_best["rank_score"]:
                global_best = rec

        es.tell(candidates, f_vals)
        gen_best = min(gen_records, key=lambda c: c["rank_score"])
        dt = time.time() - t0

        history.append({
            "gen": gen,
            "sigma": float(es.sigma),
            "clean_count": clean_count,
            "elapsed_s": round(dt, 1),
            "candidates": gen_records,
        })
        with open(ALL_GENS_JSON, "w", encoding="utf-8") as f:
            json.dump(history, f, indent=2)
        with open(CHAMPION_JSON, "w", encoding="utf-8") as f:
            json.dump(global_best, f, indent=2)
        with open(STATE_JSON, "w", encoding="utf-8") as f:
            json.dump({
                "last_gen": gen,
                "sigma": float(es.sigma),
                "clean_count": clean_count,
                "mean": {k: float(v) for k, v in zip(active_names, es.mean)},
            }, f, indent=2)

        with open(LEDGER_PATH, "a", encoding="utf-8") as f:
            f.write(
                f"| {gen} | {gen_best['rank_score']:,.2f} | {gen_best['latec_canon']:,.2f} | "
                f"{gen_best['bode_latec']:,.2f} | {gen_best['xtalk_latec']:,.2f} | {gen_best['hf_latec']:,.2f} | "
                f"{gen_best['hf_rms_latec']:.3f} | {clean_count}/{popsize} | {dt:.1f} | {es.sigma:.3f} |\n"
            )

        print(
            f"Gen {gen:02d} ({dt:.1f}s) | Clean={clean_count}/{popsize} | Rank={gen_best['rank_score']:,.2f} "
            f"BodeL={gen_best['bode_latec']:,.2f} | RollHF={gen_best['hf_rms_latec']:.3f} deg/s | sigma={es.sigma:.3f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
