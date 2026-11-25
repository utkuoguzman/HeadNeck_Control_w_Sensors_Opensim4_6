#!/usr/bin/env python3
"""Stage 7: Clean-Basin Hard-Wall 18D Roll-Subspace 1-Wave CSA CMA-ES (Pop=36, 108 sims/gen).

Starts INSIDE the non-oscillating Clean Basin (GROUP_OSIM_ROLL18, Roll HF RMS = 1.741 deg/s)
with all 35 Pitch/Yaw/Fusimotor parameters frozen at the Gen 35 Champion (APEO=1,809, YawMA=1,378).
Unlocks all 18 Roll parameters with a hard stability wall above 3.5 deg/s so CMA-ES cannot
drift into the ~15-20 Hz Hopf limit-cycle basin (10+ deg/s) while minimizing LatEC Bode score.
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

CHAMP_G35_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_champion.json")

CONFIG_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_clean_config.json")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_clean_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_clean_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_clean_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_18p_roll_clean_history.json")

# (name, x0_osim_clean, lb, ub, std)
ROLL_CLEAN_18_SPECS = [
    ("K_sc_roll",              3.5,       1.00,  9.00,  0.55),
    ("z_sc_roll",              0.018,     0.002, 0.050, 0.0040),
    ("p_sc_roll",              60.0,      20.0,  160.0, 12.0),
    ("K_phas_roll",            13.5,      2.00,  24.0,  2.20),
    ("z_phas_roll",            0.012,     0.0,   0.035, 0.0035),
    ("p_phas_roll",            4.5,       1.00,  14.0,  1.00),
    ("K_ton_roll",             0.90,      0.10,  4.00,  0.30),
    ("p_ton_roll",             3.20,      0.40,  12.0,  0.90),
    ("Kp_prop_roll",           2.10,      0.30,  7.00,  0.55),
    ("z_ccr_roll",             0.005,     0.0,   0.035, 0.0030),
    ("p_ccr_roll",             99.919586, 20.0,  160.0, 15.0),
    ("Kp_proprioception_roll", 0.050,     0.003, 0.120, 0.010),
    ("Kp_task_roll",           1.20,      0.20,  5.00,  0.35),
    ("Kd_task_roll",           1.35,      0.20,  3.50,  0.30),
    ("Ki_task_roll",           0.45,      0.02,  3.00,  0.20),
    ("tau_lead_pole_roll",     0.008,     0.004, 0.045, 0.0035),
    ("Kp_buckle_roll",         1.50,      0.20,  4.00,  0.30),
    ("Kd_buckle_roll",         0.15,      0.02,  0.55,  0.040),
]

TOP6_LOWJIT_OVERRIDES = {
    "K_sc_roll": 2.4428,
    "z_sc_roll": 0.0180,
    "tau_lead_pole_roll": 0.02951,
    "z_phas_roll": 0.0,
    "K_phas_roll": 7.2216,
    "p_phas_roll": 3.5806,
}


def build_or_load_config(save=True):
    """Build or load the 18 active Roll-clean + 35 frozen Gen 35 Pitch/Yaw/Fusimotor config."""
    if os.path.exists(CONFIG_JSON):
        return json.load(open(CONFIG_JSON, "r", encoding="utf-8"))

    champ_g35 = json.load(open(CHAMP_G35_JSON, "r", encoding="utf-8"))
    champ_p = {k: float(v) for k, v in champ_g35["params"].items()}

    active_specs = []
    active_names = []
    seed_slot1 = []
    for name, x0_clean, lb, ub, std in ROLL_CLEAN_18_SPECS:
        x_center = float(np.clip(float(x0_clean), float(lb), float(ub)))
        active_specs.append([name, x_center, float(lb), float(ub), float(std)])
        active_names.append(name)
        s1_raw = TOP6_LOWJIT_OVERRIDES.get(name, champ_p[name])
        seed_slot1.append(float(np.clip(float(s1_raw), float(lb), float(ub))))

    active_set = set(active_names)
    frozen_params = {k: float(v) for k, v in champ_p.items() if k not in active_set}

    assert len(active_specs) == 18, f"Expected 18 active, got {len(active_specs)}"
    assert len(frozen_params) == 35, f"Expected 35 frozen, got {len(frozen_params)}"
    assert len(set(frozen_params) & active_set) == 0, "Overlap between frozen and active params"
    assert len(set(frozen_params) | active_set) == 53, f"Expected 53 total params, got {len(set(frozen_params) | active_set)}"

    seed_slot0 = [float(s[1]) for s in active_specs]
    cfg = {
        "source_g35_total_score": float(champ_g35["total_score"]),
        "source_g35_hf_rms_latec": float(champ_g35["hf_rms_latec"]),
        "active_specs": active_specs,
        "frozen_params": frozen_params,
        "seed_slot0_osim_roll18": {k: float(v) for k, v in zip(active_names, seed_slot0)},
        "seed_slot1_top6_lowjit": {k: float(v) for k, v in zip(active_names, seed_slot1)},
    }
    if save:
        with open(CONFIG_JSON, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def compute_clean_rank_score(canonical_total, bode_apeo, bode_yawma, hf_rms_apeo, hf_rms_latec, hf_rms_yawma):
    """Hard stability wall above 3.5 deg/s Roll HF RMS + Pitch/Yaw Bode & HF guardrails."""
    roll_wall = 0.0 if hf_rms_latec <= 3.5 else (20000.0 + 3000.0 * (hf_rms_latec - 3.5) ** 2)
    bode_guard = 10.0 * max(0.0, bode_apeo - 2500.0) + 10.0 * max(0.0, bode_yawma - 2500.0)
    hf_guard = 500.0 * max(0.0, hf_rms_apeo - 7.5) ** 2 + 500.0 * max(0.0, hf_rms_yawma - 2.5) ** 2
    return float(canonical_total + roll_wall + bode_guard + hf_guard)


def main():
    parser = argparse.ArgumentParser(description="Clean-Basin Hard-Wall 18D Roll 1-Wave CSA CMA-ES (108 sims/gen)")
    parser.add_argument("--gens", type=int, default=50, help="Target total generations")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4518, help="Random seed")
    parser.add_argument("--resume", action="store_true", help="Resume from logs/cmaes_3axis_18p_roll_clean_history.json")
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
        f"=== Roll Clean-Basin Hard-Wall 18D Setup: {len(active_names)} active + {len(frozen_items)} frozen "
        f"(x0=GROUP_OSIM_ROLL18, frozen=Gen35 Pitch/Yaw) ===",
        flush=True,
    )
    if args.dry_check:
        print("Active(18):", active_names)
        print("Frozen(35):", len(frozen_items))
        return

    popsize = 2 * len(active_names)  # 36 -> 108 sims/gen = 1 wave on 112 cores
    modes = ("APEO", "LatEC", "YawMA")

    es = cma.CMAEvolutionStrategy(x0, 0.65, {
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
            f.write("# 3-Axis 18-Param Clean-Basin Hard-Wall 1-Wave CSA CMA-ES Ledger (Pop=36, 108 sims/gen)\n\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Active(18): {', '.join(active_names)}\n\n")
            f.write("| Gen | Rank Score | Canon Total | APEO Bode | LatEC Bode | YawMA Bode | Roll HF (°/s) | Pitch HF | Yaw HF | Clean<=3.5 | σ |\n")
            f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")

    for gen in range(start_gen, args.gens + 1):
        t0 = time.time()
        candidates = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=False)
        if gen == 1:
            candidates[0] = np.asarray([cfg["seed_slot0_osim_roll18"][k] for k in active_names], dtype=float)
            candidates[1] = np.asarray([cfg["seed_slot1_top6_lowjit"][k] for k in active_names], dtype=float)

        jobs = []
        for idx, vec in enumerate(candidates):
            ovr = frozen_items + [(k, f"{float(v):.8g}") for k, v in zip(active_names, vec)]
            for mode in modes:
                jobs.append((idx, mode, ovr))

        results_by_cand = {idx: {} for idx in range(len(candidates))}
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            futs = [ex.submit(_sim_job, job) for job in jobs]
            for fut in as_completed(futs):
                res = fut.result()
                results_by_cand[res["cand_idx"]][res["mode"]] = res

        gen_records = []
        f_vals = []
        clean_count = 0
        for idx, vec in enumerate(candidates):
            r_a = results_by_cand[idx]["APEO"]
            r_l = results_by_cand[idx]["LatEC"]
            r_y = results_by_cand[idx]["YawMA"]

            bode_a = float(r_a["classic_bode_score"])
            bode_l = float(r_l["classic_bode_score"])
            bode_y = float(r_y["classic_bode_score"])
            xtalk = float(r_a["pitch_crosstalk_penalty"] + r_l["pitch_crosstalk_penalty"] + r_y["pitch_crosstalk_penalty"])
            hf = float(r_a["hf_penalty"] + r_l["hf_penalty"] + r_y["hf_penalty"])
            canon_total = bode_a + bode_l + bode_y + xtalk + hf

            hf_rms_a = float(r_a["hf_noise_rms"])
            hf_rms_l = float(r_l["hf_noise_rms"])
            hf_rms_y = float(r_y["hf_noise_rms"])
            aborted = bool(r_a["aborted"] or r_l["aborted"] or r_y["aborted"])

            if (not aborted) and hf_rms_l <= 3.5:
                clean_count += 1

            rank_score = compute_clean_rank_score(canon_total, bode_a, bode_y, hf_rms_a, hf_rms_l, hf_rms_y)
            f_vals.append(rank_score)

            full_p = dict(cfg["frozen_params"])
            for k, v in zip(active_names, vec):
                full_p[k] = float(v)

            rec = {
                "gen": gen,
                "slot": idx,
                "params": full_p,
                "rank_score": round(rank_score, 2),
                "total_score": round(canon_total, 2),
                "bode_apeo": round(bode_a, 2),
                "bode_latec": round(bode_l, 2),
                "bode_yawma": round(bode_y, 2),
                "xtalk_total": round(xtalk, 2),
                "hf_total": round(hf, 2),
                "hf_rms_apeo": round(hf_rms_a, 3),
                "hf_rms_latec": round(hf_rms_l, 3),
                "hf_rms_yawma": round(hf_rms_y, 3),
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
                f"| {gen} | {gen_best['rank_score']:,.2f} | {gen_best['total_score']:,.2f} | "
                f"{gen_best['bode_apeo']:,.2f} | {gen_best['bode_latec']:,.2f} | {gen_best['bode_yawma']:,.2f} | "
                f"{gen_best['hf_rms_latec']:.3f} | {gen_best['hf_rms_apeo']:.3f} | {gen_best['hf_rms_yawma']:.3f} | "
                f"{clean_count}/{popsize} | {es.sigma:.3f} |\n"
            )

        print(
            f"Gen {gen:02d} ({dt:.1f}s) | Clean={clean_count}/{popsize} | Rank={gen_best['rank_score']:,.2f} "
            f"Canon={gen_best['total_score']:,.2f} | RollHF={gen_best['hf_rms_latec']:.3f} | "
            f"Bode(A/L/Y)={gen_best['bode_apeo']:.0f}/{gen_best['bode_latec']:.0f}/{gen_best['bode_yawma']:.0f} | sigma={es.sigma:.3f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
