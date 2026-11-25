#!/usr/bin/env python3
"""Stage 6: 10-Parameter Culprit-Only Roll-Jitter Polish 1-Wave CSA CMA-ES (Pop=20, 60 sims/gen).

Freezes 43 non-culprit parameters at the Roll18P Gen 35 Champion (9,416.54).
Unlocks only the 10 parameters with |Delta hf_rms_latec| >= 0.35 deg/s in the
53-parameter LatEC jitter sensitivity sweep:
  Roll18 Top-6: K_sc_roll, z_sc_roll, tau_lead_pole_roll, z_phas_roll, K_phas_roll, p_phas_roll
  Frozen35 (>=0.35 deg/s): Kp_prop_yaw, Kp_buckle, z_ccr_yaw, tau_lead_pole

Uses 1-wave CSA CMA-ES (Pop=20 -> 60 sims/gen on 112 cores) with:
  - Extra Roll HF jitter pressure above 3.5 deg/s: + 35.0 * max(0, hf_rms_latec - 3.5)^2
  - Guardrails keeping APEO & YawMA Bode < 2,600 and Pitch/Yaw HF jitter low.
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

CONFIG_JSON = os.path.join(LOG_DIR, "cmaes_3axis_jitter_culprit_config.json")
STATE_JSON = os.path.join(LOG_DIR, "cmaes_3axis_jitter_culprit_state.json")
CHAMPION_JSON = os.path.join(LOG_DIR, "cmaes_3axis_jitter_culprit_champion.json")
LEDGER_PATH = os.path.join(LOG_DIR, "cmaes_3axis_jitter_culprit_ledger.md")
ALL_GENS_JSON = os.path.join(LOG_DIR, "cmaes_3axis_jitter_culprit_history.json")

# (name, lb, ub, std, top6_probe_val_or_None)
CULPRIT_10_SPECS = [
    ("K_sc_roll",          1.50,  9.00,  0.45,   2.4428),
    ("z_sc_roll",          0.002, 0.050, 0.0035, 0.0180),
    ("tau_lead_pole_roll", 0.004, 0.045, 0.0030, 0.02951),
    ("z_phas_roll",        0.0,   0.035, 0.0025, 0.0),
    ("K_phas_roll",        4.00,  24.0,  1.50,   7.2216),
    ("p_phas_roll",        1.50,  14.0,  0.80,   3.5806),
    ("Kp_prop_yaw",        3.00,  30.0,  1.50,   None),
    ("Kp_buckle",          0.40,  4.00,  0.25,   None),
    ("z_ccr_yaw",          0.0,   0.120, 0.0050, None),
    ("tau_lead_pole",      0.004, 0.035, 0.0020, None),
]


def build_or_load_config(save=True):
    """Build or load the 10 active culprits + 43 frozen Gen 35 parameters."""
    if os.path.exists(CONFIG_JSON):
        return json.load(open(CONFIG_JSON, "r", encoding="utf-8"))

    champ_g35 = json.load(open(CHAMP_G35_JSON, "r", encoding="utf-8"))
    champ_p = {k: float(v) for k, v in champ_g35["params"].items()}

    active_specs = []
    active_names = []
    seed_slot1 = []
    for name, lb, ub, std, probe_val in CULPRIT_10_SPECS:
        x_center = float(np.clip(champ_p[name], float(lb), float(ub)))
        active_specs.append([name, x_center, float(lb), float(ub), float(std)])
        active_names.append(name)
        seed_slot1.append(float(np.clip(probe_val, float(lb), float(ub))) if probe_val is not None else x_center)

    active_set = set(active_names)
    frozen_params = {k: float(v) for k, v in champ_p.items() if k not in active_set}

    assert len(active_specs) == 10, f"Expected 10 active, got {len(active_specs)}"
    assert len(frozen_params) == 43, f"Expected 43 frozen, got {len(frozen_params)}"
    assert len(set(frozen_params) & active_set) == 0, "Overlap between frozen and active params"
    assert len(set(frozen_params) | active_set) == 53, "Expected 53 total params"

    seed_slot0 = [float(s[1]) for s in active_specs]
    cfg = {
        "source_g35_total_score": float(champ_g35["total_score"]),
        "source_g35_hf_rms_latec": float(champ_g35["hf_rms_latec"]),
        "active_specs": active_specs,
        "frozen_params": frozen_params,
        "seed_slot0_g35": {k: float(v) for k, v in zip(active_names, seed_slot0)},
        "seed_slot1_top6_lowjit": {k: float(v) for k, v in zip(active_names, seed_slot1)},
    }
    if save:
        with open(CONFIG_JSON, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def compute_rank_score(canonical_total, bode_apeo, bode_yawma, hf_rms_apeo, hf_rms_latec, hf_rms_yawma):
    """Canonical total + Roll HF jitter pressure + Pitch/Yaw Bode & HF guardrails."""
    roll_jit_pen = 35.0 * max(0.0, hf_rms_latec - 3.5) ** 2
    bode_guard = 2.0 * max(0.0, bode_apeo - 2600.0) + 2.0 * max(0.0, bode_yawma - 2600.0)
    hf_guard = 25.0 * max(0.0, hf_rms_apeo - 7.0) ** 2 + 25.0 * max(0.0, hf_rms_yawma - 2.0) ** 2
    return float(canonical_total + roll_jit_pen + bode_guard + hf_guard)


def main():
    parser = argparse.ArgumentParser(description="20-Popsize 10D Roll-Jitter Culprit 1-Wave CSA CMA-ES (60 sims/gen)")
    parser.add_argument("--gens", type=int, default=40, help="Target total generations")
    parser.add_argument("--workers", type=int, default=112, help="Parallel OpenSim worker processes")
    parser.add_argument("--seed", type=int, default=4410, help="Random seed")
    parser.add_argument("--resume", action="store_true", help="Resume from logs/cmaes_3axis_jitter_culprit_history.json")
    parser.add_argument("--dry-check", action="store_true", help="Verify 10 active + 43 frozen split without running sims")
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
        f"=== Roll-Jitter Culprit Subspace: {len(active_names)} active + {len(frozen_items)} frozen "
        f"(G35 Total={cfg['source_g35_total_score']:,.2f}, hf_rms_latec={cfg['source_g35_hf_rms_latec']:.3f} deg/s) ===",
        flush=True,
    )
    if args.dry_check:
        print("Active(10):", active_names)
        print("Frozen(43):", len(frozen_items))
        return

    popsize = 2 * len(active_names)  # 20 -> 60 sims/gen = 1 wave on 112 cores
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
            f.write("# 3-Axis 10-Param Roll-Jitter Culprit 1-Wave CSA CMA-ES Ledger (Pop=20, 60 sims/gen)\n\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Active(10): {', '.join(active_names)}\n\n")
            f.write("| Gen | Rank Score | Canon Total | APEO Bode | LatEC Bode | YawMA Bode | Roll HF RMS (°/s) | Pitch HF RMS | Yaw HF RMS | σ |\n")
            f.write("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")

    for gen in range(start_gen, args.gens + 1):
        t0 = time.time()
        candidates = ask_orthogonal_mirrored_nd(es, popsize, lb, ub, use_tpa=False)
        if gen == 1:
            candidates[0] = np.asarray([cfg["seed_slot0_g35"][k] for k in active_names], dtype=float)
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

            rank_score = compute_rank_score(canon_total, bode_a, bode_y, hf_rms_a, hf_rms_l, hf_rms_y)
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

        history.append({"gen": gen, "sigma": float(es.sigma), "elapsed_s": round(dt, 1), "candidates": gen_records})
        with open(ALL_GENS_JSON, "w", encoding="utf-8") as f:
            json.dump(history, f, indent=2)
        with open(CHAMPION_JSON, "w", encoding="utf-8") as f:
            json.dump(global_best, f, indent=2)
        with open(STATE_JSON, "w", encoding="utf-8") as f:
            json.dump({"last_gen": gen, "sigma": float(es.sigma), "mean": {k: float(v) for k, v in zip(active_names, es.mean)}}, f, indent=2)

        with open(LEDGER_PATH, "a", encoding="utf-8") as f:
            f.write(
                f"| {gen} | {gen_best['rank_score']:,.2f} | {gen_best['total_score']:,.2f} | "
                f"{gen_best['bode_apeo']:,.2f} | {gen_best['bode_latec']:,.2f} | {gen_best['bode_yawma']:,.2f} | "
                f"{gen_best['hf_rms_latec']:.3f} | {gen_best['hf_rms_apeo']:.3f} | {gen_best['hf_rms_yawma']:.3f} | {es.sigma:.3f} |\n"
            )

        print(
            f"Gen {gen:02d} ({dt:.1f}s) | Rank={gen_best['rank_score']:,.2f} Canon={gen_best['total_score']:,.2f} | "
            f"RollHF={gen_best['hf_rms_latec']:.3f} deg/s | Bode(A/L/Y)={gen_best['bode_apeo']:.0f}/{gen_best['bode_latec']:.0f}/{gen_best['bode_yawma']:.0f} | sigma={es.sigma:.3f}",
            flush=True,
        )


if __name__ == "__main__":
    main()
