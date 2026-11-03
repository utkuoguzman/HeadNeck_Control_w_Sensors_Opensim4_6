import os
import sys
import json
import math
import time
import random
import subprocess
import numpy as np
import cma
from concurrent.futures import ThreadPoolExecutor, as_completed

sys.path.append(os.getcwd())

FIXED_W_N = 22.0
FIXED_ZETA = 0.707
FIXED_KI = 40.2786
HAPPEE_BENCHMARK = 2803.88

TARGETS_ORDERED = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
]

PARAM_NAMES = [
    "K_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",
    "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat",
    "Kp_task_pitch", "Kd_task_pitch", "Ki_task_pitch"
]


def evaluate_multisine_candidate(candidate, w_idx, gen_idx):
    worker_label = f"W{w_idx+1}_MS_G{gen_idx}"
    res_file = os.path.join("logs", "candidates_multisine", f"{worker_label}.json")
    if os.path.exists(res_file):
        try:
            with open(res_file, "r", encoding="utf-8") as f_res:
                d_res = json.load(f_res)
            if "total_score" in d_res and "params" in d_res and len(d_res["params"]) == len(candidate):
                if (abs(d_res["params"][0] - candidate[0]) < 1e-6 and
                    abs(d_res["params"][1] - candidate[1]) < 1e-5 and
                    abs(d_res["params"][4] - candidate[4]) < 1e-5):
                    return float(d_res["total_score"])
        except Exception:
            pass

    cmd = [
        sys.executable, "cmaes_worker_multisine.py",
        str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
        *[str(v) for v in candidate],
        worker_label
    ]
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=600
        )
        if result.returncode == 0:
            lines = [l.strip() for l in result.stdout.strip().split("\n") if l.strip()]
            score = float(lines[-1])
            print(f"[COMPLETE] [{worker_label}] finished 6-freq multisine! Total Score: {score:.2f}", flush=True)
            return score
        else:
            print(f"[FAIL] [{worker_label}]: {result.stderr}", file=sys.stderr)
            return 1200000.0 + random.uniform(0, 100.0)
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] [{worker_label}]", file=sys.stderr)
        return 1200000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"[ERROR] [{worker_label}]: {e}", file=sys.stderr)
        return 1200000.0 + random.uniform(0, 100.0)


def update_multisine_ledger(gen_records):
    os.makedirs("logs", exist_ok=True)
    prog_lines = [
        "| Campaign Milestone | 0.324 Hz | 0.783 Hz | 1.099 Hz | 2.446 Hz | 5.583 Hz | 7.927 Hz | Total Score |",
        "| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
        "| **Happee (2017) Benchmark** | -2.5 dB<br>/ 39.1° | +0.0 dB<br>/ 3.0° | +0.7 dB<br>/ 9.1° | -0.9 dB<br>/ 0.1° | -0.7 dB<br>/ 6.7° | +1.0 dB<br>/ 14.8° | **`2803.88`** |",
    ]

    md_lines = [
        "# Multisine CMA-ES Optimization Ledger (4-Generation Comparison vs. Single-Sine Phase 2.4)",
        "",
        "## 1. Multisine Excitation & Demodulation Configuration",
        r"- **Excitation:** 6-Frequency Schroeder-Phase Multisine (`0.324, 0.783, 1.099, 2.446, 5.583, 7.927 Hz`, equal-RMS scale factor $1.0 / \sqrt{6} = 0.408248$, $400\,\text{Hz}$ `SimmSpline`).",
        r"- **Window:** $1.5\,\text{s}$ zero-excitation equilibrium check + $1.5\,\text{s}$ $C^2$ quintic ramp + $6.173\,\text{s}$ steady-state evaluation window ($2\times$ full cycles of $0.324\,\text{Hz}$, total $9.173\,\text{s}$ vs. $25.4\,\text{s}$ single-sine sum).",
        r"- **Demodulation:** Exact 14-column multi-harmonic linear least-squares (`1, t, sin(w_k t + phi_k), cos(w_k t + phi_k)`).",
        "",
    ]

    for rec in gen_records:
        g = rec["gen"]
        w_best = rec["best_worker"]
        score = rec["best_score"]
        dur = rec["duration_min"]
        n_pass = rec["n_pass"]
        mean_pass = rec["mean_pass"]
        med_pass = rec["median_pass"]
        d_json = rec["champ_json"]
        pf = d_json["frequencies"]
        tel = d_json["telemetry"]

        row_cells = []
        for f_str in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]:
            fd = pf[f_str]
            row_cells.append(f"{fd['gain_db_diff']:+.1f} dB<br>/ {fd['phase_err_deg']:.1f}°")
        prog_lines.append(
            f"| **Multisine Gen {g}** (`{w_best}`) | " + " | ".join(row_cells) + f" | **`{score:.2f}`** |"
        )

        md_lines.extend([
            "---",
            f"## Multisine Generation {g} — Champion `{w_best}` (Total Score: `{score:.2f} pts`, Duration: `{dur:.1f} min`)",
            f"- **Pass Rate:** `{n_pass} / 28` | **Mean Pass Score:** `{mean_pass:.2f}` | **Median Pass Score:** `{med_pass:.2f}`",
            f"- **6-DOF Kinematics:** Pre-Perturbation Drift = `{tel['pre_pitch_drift_deg']:.4f}°` | Post `pitch` = `[{tel['post_pitch_min_deg']:+.2f}°, {tel['post_pitch_max_deg']:+.2f}°]` | `pitch1` = `[{tel['p1_min_deg']:+.2f}°, {tel['p1_max_deg']:+.2f}°]` | `pitch2` = `[{tel['p2_min_deg']:+.2f}°, {tel['p2_max_deg']:+.2f}°]` | Max `|roll|` = `{tel['max_abs_roll_deg']:.3f}°` | Max `|yaw|` = `{tel['max_abs_yaw_deg']:.3f}°`",
            f"- **CNS Observers:** Mahony Pitch RMSE = `{tel['mahony_pitch_rmse_deg']:.3f}°` (`r = {tel['mahony_pitch_corr']:+.4f}`) | Canal Luenberger RMSE = `{tel['canal_luen_rmse_deg_s']:.2f} deg/s` (`r = {tel['canal_luen_corr']:+.4f}`) | Max Spindle `dL` = `{tel['spindle_dL_max_mm']:.3f} mm`",
            "",
            "| Freq (`Hz`) | Sim / Target Gain (`dB`) | Sim / Target Phase (`err`) | Gain Loss | Phase Loss | Freq Score |",
            "| :---: | :---: | :---: | :---: | :---: | :---: |",
        ])
        for f_str in ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]:
            fd = pf[f_str]
            md_lines.append(
                f"| **`{f_str}`** | `{fd['sim_gain']:.1f}` / `{fd['target_gain']:.1f}` (`{fd['gain_db_diff']:+.1f} dB`) | `{fd['sim_phase']:.1f}°` / `{fd['target_phase']:.1f}°` (`{fd['phase_err_deg']:.1f}°`) | `{fd['gain_loss']:.1f}` | `{fd['phase_loss']:.1f}` | **`{fd['score']:.1f}`** |"
            )
        md_lines.append("")

    with open("logs/multisine_bode_progress_matrix.md", "w", encoding="utf-8") as f:
        f.write("\n".join(prog_lines) + "\n")

    with open("logs/multisine_master_ledger.md", "w", encoding="utf-8") as f:
        f.write("\n".join(md_lines) + "\n")


def main():
    max_gens = int(os.environ.get("MULTISINE_STOP_GEN", "4"))
    n_workers = int(os.environ.get("PHASE2_4_WORKERS", "14"))

    with open("data/prescreen_rational_champion.json", "r", encoding="utf-8") as f:
        seed_data = json.load(f)

    x0 = np.array(seed_data["x0"], dtype=float)
    sigma0 = np.array(seed_data["sigma0"], dtype=float)

    lower_bounds = [
        0.010,  5.000,  0.010,  0.001, 15.000,  0.200,
        1.000,  0.030,  0.001, 20.000,  0.050,  0.050,
        0.000,  0.000,  0.500
    ]
    upper_bounds = [
        3.000, 150.000,  1.500,  0.600, 160.000, 28.000,
       65.000,   2.000,  0.500, 240.000,  1.500,  1.500,
       30.000,  12.000, 65.000
    ]

    print("=" * 92, flush=True)
    print(f"STARTING 4-GENERATION MULTISINE CMA-ES CAMPAIGN (Pop=28, Workers={n_workers})", flush=True)
    print("Identical Seed x0 & Sigma0 to Single-Sine Phase 2.4 Campaign for Direct A/B Comparison", flush=True)
    print("=" * 92, flush=True)

    # First evaluate the exact seed x0 on Multisine for reference
    print("\n[SEED CHECK] Evaluating Initial Seed x0 on 6-Frequency Multisine...", flush=True)
    t_seed0 = time.time()
    seed_ms_score = evaluate_multisine_candidate(x0, -1, 0)
    print(f"[SEED CHECK] Seed x0 Multisine Score = {seed_ms_score:,.2f} pts (in {time.time() - t_seed0:.1f}s vs 52,399.62 pts Single-Sine)", flush=True)

    opts = {
        "bounds": [lower_bounds, upper_bounds],
        "CMA_stds": list(sigma0),
        "popsize": 28,
        "seed": 2026,
        "verbose": -1,
    }
    es = cma.CMAEvolutionStrategy(x0, 1.0, opts)

    gen_records = []
    all_time_best_score = float("inf")
    all_time_best_worker = None
    all_time_best_params = None

    for gen in range(1, max_gens + 1):
        t_gen_start = time.time()
        print(f"\n==================== STARTING MULTISINE GENERATION {gen} of {max_gens} ====================", flush=True)
        solutions = es.ask()
        scores = [0.0] * len(solutions)

        with ThreadPoolExecutor(max_workers=n_workers) as executor:
            fut_to_idx = {
                executor.submit(evaluate_multisine_candidate, solutions[i], i, gen): i
                for i in range(len(solutions))
            }
            for fut in as_completed(fut_to_idx):
                idx = fut_to_idx[fut]
                scores[idx] = fut.result()

        es.tell(solutions, scores)

        best_idx = int(np.argmin(scores))
        best_score = float(scores[best_idx])
        best_worker = f"W{best_idx+1}_MS_G{gen}"
        dur_min = (time.time() - t_gen_start) / 60.0

        pass_scores = [s for s in scores if s < 150000.0]
        n_pass = len(pass_scores)
        mean_pass = float(np.mean(pass_scores)) if pass_scores else float("nan")
        med_pass = float(np.median(pass_scores)) if pass_scores else float("nan")

        if best_score < all_time_best_score:
            all_time_best_score = best_score
            all_time_best_worker = best_worker
            all_time_best_params = [float(v) for v in solutions[best_idx]]

        champ_file = os.path.join("logs", "candidates_multisine", f"{best_worker}.json")
        with open(champ_file, "r", encoding="utf-8") as f_c:
            champ_json = json.load(f_c)

        gen_records.append({
            "gen": gen,
            "best_worker": best_worker,
            "best_score": best_score,
            "duration_min": dur_min,
            "n_pass": n_pass,
            "mean_pass": mean_pass,
            "median_pass": med_pass,
            "champ_json": champ_json,
        })
        update_multisine_ledger(gen_records)

        print(
            f"[MULTISINE GEN {gen} COMPLETE] Best: {best_worker} = {best_score:,.2f} pts | "
            f"Pass: {n_pass}/28 | Mean Pass: {mean_pass:,.2f} | Median: {med_pass:,.2f} | Duration: {dur_min:.2f} min",
            flush=True
        )

    print("\n" + "=" * 92, flush=True)
    print(f"MULTISINE 4-GEN CAMPAIGN COMPLETE! All-Time Multisine Champion: {all_time_best_worker} ({all_time_best_score:,.2f} pts)", flush=True)
    print("=" * 92, flush=True)

    # Cross-evaluate the Multisine All-Time Champion on the 6 Single-Sine Frequencies for exact apples-to-apples comparison!
    if all_time_best_params is not None:
        print(f"\n[CROSS-VALIDATION] Evaluating Multisine Champion ({all_time_best_worker}) on 6 Separate Single-Sine Frequencies...", flush=True)
        single_sine_total = 0.0
        for f_val, t_gain, t_phase in TARGETS_ORDERED:
            label = f"{all_time_best_worker}_SingleSineCheck"
            cmd = [
                sys.executable, "cmaes_worker_p22.py",
                str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
                *[str(v) for v in all_time_best_params],
                str(f_val), str(t_gain), str(t_phase),
                label
            ]
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
            lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
            f_score = float(lines[-1])
            single_sine_total += f_score
            print(f"  Single-Sine {f_val:5.3f} Hz Score: {f_score:8.1f} pts", flush=True)
        print(f"[CROSS-VALIDATION] {all_time_best_worker} Total Single-Sine Score = {single_sine_total:,.2f} pts (vs Multisine Score {all_time_best_score:,.2f} pts)", flush=True)


if __name__ == "__main__":
    main()
