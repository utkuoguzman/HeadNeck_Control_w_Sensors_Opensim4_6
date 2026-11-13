import os
import sys
import json
import math
import time
import glob
import shutil
import random
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import cma

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
PLOT_SCRIPT = os.path.join(ART_DIR, "scratch", "plot_3panel_multisine.py")

# Fixed Voluntary Pitch Task Gains (from Champion) & Biological 1:1 Fusimotor Coactivation:
FIXED_KP_TASK_PITCH = 0.969350
FIXED_KD_TASK_PITCH = 0.363497
FIXED_KI_TASK_PITCH = 29.528023
FIXED_K_GAMMA_DYN   = 1.0
FIXED_K_GAMMA_STAT  = 1.0

# 4 Optimized Reflex Gains: [K_ton, K_sc, K_phas, kp_prop]
PARAM_4_NAMES = ["K_ton", "K_sc", "K_phas", "kp_prop"]

# Physical bounds for [K_ton, K_sc, K_phas, kp_prop]:
# kp_prop >= 0.12 ensures C0-C1 (pitch2) has sufficient local stretch reflex stiffness to prevent S-buckling aborts.
PHYS_LB = np.array([0.10, 0.015,  0.25, 0.120], dtype=float)
PHYS_UB = np.array([3.00, 0.600, 20.00, 0.450], dtype=float)
LOG_LB  = np.log10(PHYS_LB)
LOG_UB  = np.log10(PHYS_UB)

# Initial center: reduced K_phas (from 15 -> 5.0) and slightly higher kp_prop (0.20) + K_sc (0.080) to damp 2.446 Hz resonance:
X0_PHYS = np.array([0.65, 0.080, 5.00, 0.20], dtype=float)
X0_LOG  = np.log10(X0_PHYS)
SIGMA0_LOG = np.array([0.20, 0.22, 0.28, 0.14], dtype=float)

# Generation-1 smart physics seeds:
GEN1_PHYSICS_PROBES = [
    [0.80, 0.060, 15.00, 0.18],  # Slot 0: Exact RATIONAL_TEST_FIXED anchor (74,833.25 pts)
    [0.65, 0.080,  5.00, 0.20],  # Slot 1: Lower K_phas=5.0 (champion was 4.94), kp_prop=0.20, K_sc=0.080
    [0.55, 0.065,  3.50, 0.22],  # Slot 2: Lower K_phas=3.50, stronger C0-C1 stretch support kp_prop=0.22
    [0.75, 0.100,  7.50, 0.20],  # Slot 3: Intermediate K_phas=7.50, higher canal damping K_sc=0.100
    [0.45, 0.055,  1.80, 0.19],  # Slot 4: Low K_phas=1.80, gentle K_ton=0.45 to test 2.446 Hz & 5.583 Hz suppression
    [0.95, 0.120,  4.50, 0.24],  # Slot 5: Stronger K_ton=0.95 + K_sc=0.12 + kp_prop=0.24
]


def _write_fallback_json(res_file, worker_label, phys, score, reason):
    os.makedirs(os.path.dirname(res_file), exist_ok=True)
    K_ton, K_sc, K_phas, kp_prop = [float(v) for v in phys]
    fallback = {
        "worker": worker_label,
        "total_score": float(score),
        "params": [
            K_ton, 15.0, K_sc, 0.015, 50.0, K_phas, 4.0,
            kp_prop, 0.015, 100.0, 1.0, 1.0,
            FIXED_KP_TASK_PITCH, FIXED_KD_TASK_PITCH, FIXED_KI_TASK_PITCH,
        ],
        "frequencies": {},
        "telemetry": {
            "aborted": True,
            "abort_time_s": 0.0,
            "abort_reason": reason,
            "bode_score_without_hf": float(score),
            "hf_penalty": 0.0,
            "dc_drift_penalty": 0.0,
            "dc_offset_rel_deg": 0.0,
            "post_pitch_mean_deg": 0.0,
        },
    }
    with open(res_file, "w", encoding="utf-8") as f:
        json.dump(fallback, f, indent=2)


def evaluate_4param_candidate(x_log, w_idx, gen_idx):
    phys = np.clip(10.0 ** np.asarray(x_log, dtype=float), PHYS_LB, PHYS_UB)
    K_ton, K_sc, K_phas, kp_prop = [float(v) for v in phys]
    worker_label = f"W{w_idx+1:02d}_4P_G{gen_idx:02d}"
    res_file = os.path.join(REPO_ROOT, "logs", "candidates_multisine", f"{worker_label}.json")

    if os.path.exists(res_file):
        try:
            with open(res_file, "r", encoding="utf-8") as f_res:
                d_res = json.load(f_res)
            if "total_score" in d_res and "params" in d_res and len(d_res["params"]) >= 8:
                p = d_res["params"]
                if (
                    abs(p[0] - K_ton) < 1e-5
                    and abs(p[2] - K_sc) < 1e-5
                    and abs(p[5] - K_phas) < 1e-4
                    and abs(p[7] - kp_prop) < 1e-5
                ):
                    sc = float(d_res["total_score"])
                    print(
                        f"[CACHED]   [{worker_label}] K_ton={K_ton:.4f}, K_sc={K_sc:.4f}, K_phas={K_phas:.3f}, kp_prop={kp_prop:.4f} -> {sc:,.2f} pts",
                        flush=True,
                    )
                    return sc
        except Exception:
            pass

    env = os.environ.copy()
    env["SIMB_NUM_THREADS"] = "1"
    env["OMP_NUM_THREADS"] = "1"
    env["MKL_NUM_THREADS"] = "1"
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["SAVE_TS"] = "1"

    cmd = [
        sys.executable,
        os.path.join(REPO_ROOT, "util", "cmaes_worker_multisine.py"),
        f"{K_ton:.8f}",
        f"{K_sc:.8f}",
        f"{K_phas:.8f}",
        f"{kp_prop:.8f}",
        f"{FIXED_K_GAMMA_DYN:.4f}",
        f"{FIXED_K_GAMMA_STAT:.4f}",
        f"{FIXED_KP_TASK_PITCH:.6f}",
        f"{FIXED_KD_TASK_PITCH:.6f}",
        f"{FIXED_KI_TASK_PITCH:.6f}",
        worker_label,
    ]
    try:
        t0 = time.time()
        result = subprocess.run(
            cmd,
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=900,
        )
        dt = time.time() - t0
        if result.returncode == 0:
            lines = [l.strip() for l in result.stdout.strip().split("\n") if l.strip()]
            score = float(lines[-1])
            print(
                f"[COMPLETE] [{worker_label}] ({dt:5.1f}s) K_ton={K_ton:.4f}, K_sc={K_sc:.4f}, K_phas={K_phas:.3f}, kp_prop={kp_prop:.4f} -> Score: {score:,.2f} pts",
                flush=True,
            )
            return score
        else:
            sc_fail = 1800000.0 + random.uniform(0, 100.0)
            _write_fallback_json(res_file, worker_label, phys, sc_fail, "subprocess_error")
            print(f"[FAIL] [{worker_label}]: {result.stderr[-300:]}", file=sys.stderr, flush=True)
            return sc_fail
    except subprocess.TimeoutExpired:
        sc_to = 1800000.0 + random.uniform(0, 100.0)
        _write_fallback_json(res_file, worker_label, phys, sc_to, "timeout_900s")
        print(f"[TIMEOUT] [{worker_label}]", file=sys.stderr, flush=True)
        return sc_to
    except Exception as e:
        sc_err = 1800000.0 + random.uniform(0, 100.0)
        _write_fallback_json(res_file, worker_label, phys, sc_err, str(e))
        print(f"[ERROR] [{worker_label}]: {e}", file=sys.stderr, flush=True)
        return sc_err


def write_4param_ledger(gen_records, all_time_best):
    os.makedirs(os.path.join(REPO_ROOT, "logs"), exist_ok=True)
    lines = [
        "# 4-Parameter Reflex Gain CMA-ES Optimization Ledger (`z_sc = z_ccr = 15.0 ms`)",
        "",
        f"- **All-Time Best Champion:** `{all_time_best['worker']}` — **`{all_time_best['score']:,.2f} pts`**",
        f"- **Best Gains:** `K_ton = {all_time_best['phys'][0]:.4f}`, `K_sc = {all_time_best['phys'][1]:.4f}`, `K_phas = {all_time_best['phys'][2]:.3f}`, `kp_prop = {all_time_best['phys'][3]:.4f}`",
        "",
        "| Gen | Champion | Total Score | Bode Score | >=10 Hz Tremor | DC/Drift Pen | Mean Pitch | K_ton | K_sc | K_phas | kp_prop | Pass | Min |",
        "| :-: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |",
    ]
    for r in gen_records:
        tel = r["champ_json"].get("telemetry", {})
        bode_s = tel.get("bode_score_without_hf", r["best_score"])
        hf_p = tel.get("hf_penalty", 0.0)
        dc_p = tel.get("dc_drift_penalty", 0.0)
        mean_p = tel.get("dc_offset_rel_deg", tel.get("post_pitch_mean_deg", 0.0))
        ph = r["best_phys"]
        lines.append(
            f"| **{r['gen']}** | `{r['best_worker']}` | **`{r['best_score']:,.2f}`** | `{bode_s:,.2f}` | `{hf_p:,.2f}` | `{dc_p:,.2f}` | `{mean_p:+.2f}°` | "
            f"`{ph[0]:.4f}` | `{ph[1]:.4f}` | `{ph[2]:.3f}` | `{ph[3]:.4f}` | `{r['n_pass']}/{r['popsize']}` | `{r['duration_min']:.2f}` |"
        )
    ledger_path = os.path.join(REPO_ROOT, "logs", "cmaes_4param_ledger.md")
    with open(ledger_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
    max_gens = int(os.environ.get("CMAES_4P_GENS", "6"))
    popsize = int(os.environ.get("CMAES_4P_POPSIZE", "20"))
    n_workers = int(os.environ.get("CMAES_4P_WORKERS", "20"))

    print("=" * 98, flush=True)
    print(
        f"LAUNCHING 4-PARAMETER REFLEX GAIN CMA-ES (Log10-Space, Mirrored+Active CMA, Pop={popsize}, Workers={n_workers}, Gens={max_gens})",
        flush=True,
    )
    print(
        f"Fixed Constants: z_sc=z_ccr=15.0ms | K_gamma_dyn=K_gamma_stat=1.0 | Kp_task={FIXED_KP_TASK_PITCH}, Kd_task={FIXED_KD_TASK_PITCH}, Ki_task={FIXED_KI_TASK_PITCH}",
        flush=True,
    )
    print("=" * 98, flush=True)

    opts = {
        "bounds": [list(LOG_LB), list(LOG_UB)],
        "CMA_stds": list(SIGMA0_LOG),
        "popsize": popsize,
        "CMA_mirrors": True,
        "CMA_mirrormethod": 2,
        "CMA_active": True,
        "seed": 202604,
        "verbose": -1,
    }

    es = cma.CMAEvolutionStrategy(list(X0_LOG), 1.0, opts)
    gen_records = []
    all_time_best = {
        "score": float("inf"),
        "worker": None,
        "phys": list(X0_PHYS),
    }

    for gen in range(1, max_gens + 1):
        t_gen0 = time.time()
        solutions = es.ask()

        if gen == 1:
            for slot_idx, probe_phys in enumerate(GEN1_PHYSICS_PROBES):
                if slot_idx < len(solutions):
                    probe_clipped = np.clip(np.array(probe_phys, dtype=float), PHYS_LB + 1e-6, PHYS_UB - 1e-6)
                    solutions[slot_idx] = np.log10(probe_clipped)
        elif all_time_best["worker"] is not None:
            best_clipped = np.clip(np.array(all_time_best["phys"], dtype=float), PHYS_LB + 1e-6, PHYS_UB - 1e-6)
            solutions[0] = np.log10(best_clipped)

        print(f"\n==================== STARTING 4-PARAM GENERATION {gen} of {max_gens} (20 Cores) ====================", flush=True)
        scores = [0.0] * len(solutions)
        with ThreadPoolExecutor(max_workers=n_workers) as executor:
            fut_to_idx = {
                executor.submit(evaluate_4param_candidate, solutions[i], i, gen): i
                for i in range(len(solutions))
            }
            for fut in as_completed(fut_to_idx):
                idx = fut_to_idx[fut]
                scores[idx] = fut.result()

        es.tell(solutions, scores)

        best_idx = int(np.argmin(scores))
        best_score = float(scores[best_idx])
        best_worker = f"W{best_idx+1:02d}_4P_G{gen:02d}"
        best_phys = [float(v) for v in np.clip(10.0 ** np.asarray(solutions[best_idx]), PHYS_LB, PHYS_UB)]
        dur_min = (time.time() - t_gen0) / 60.0

        pass_scores = [s for s in scores if s < 500000.0]
        n_pass = len(pass_scores)
        mean_pass = float(np.mean(pass_scores)) if pass_scores else float("nan")
        med_pass = float(np.median(pass_scores)) if pass_scores else float("nan")

        champ_file = os.path.join(REPO_ROOT, "logs", "candidates_multisine", f"{best_worker}.json")
        if os.path.exists(champ_file):
            with open(champ_file, "r", encoding="utf-8") as f_c:
                champ_json = json.load(f_c)
        else:
            champ_json = {"telemetry": {"bode_score_without_hf": best_score}}

        improved = best_score < all_time_best["score"]
        if improved:
            all_time_best["score"] = best_score
            all_time_best["worker"] = best_worker
            all_time_best["phys"] = best_phys
            with open(os.path.join(REPO_ROOT, "logs", "cmaes_4param_best_champion.json"), "w", encoding="utf-8") as f_b:
                json.dump(champ_json, f_b, indent=2)
            if os.path.exists(PLOT_SCRIPT) and champ_json.get("frequencies"):
                try:
                    subprocess.run([sys.executable, PLOT_SCRIPT, best_worker], cwd=REPO_ROOT, check=False)
                except Exception as e_plt:
                    print(f"[WARN] Plot generation failed: {e_plt}", flush=True)

        gen_records.append({
            "gen": gen,
            "popsize": popsize,
            "best_worker": best_worker,
            "best_score": best_score,
            "best_phys": best_phys,
            "duration_min": dur_min,
            "n_pass": n_pass,
            "mean_pass": mean_pass,
            "median_pass": med_pass,
            "champ_json": champ_json,
        })
        write_4param_ledger(gen_records, all_time_best)

        print(
            f"[GEN {gen:02d} COMPLETE] Gen Best: {best_worker} = {best_score:,.2f} pts "
            f"(All-Time Best: {all_time_best['worker']} = {all_time_best['score']:,.2f} pts) | "
            f"Gains=[K_ton={best_phys[0]:.4f}, K_sc={best_phys[1]:.4f}, K_phas={best_phys[2]:.3f}, kp_prop={best_phys[3]:.4f}] | "
            f"Pass: {n_pass}/{popsize} | Median: {med_pass:,.2f} | Time: {dur_min:.2f} min",
            flush=True,
        )


if __name__ == "__main__":
    main()
