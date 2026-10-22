import os
import sys
import subprocess
import concurrent.futures
import json
import math
import numpy as np
import cma
import datetime
import random

# ==============================================================================
# PHASE 2.1 CONSOLIDATED CAMPAIGN (100% Sensor-Driven + Per-Sensor Luenberger)
# Target: Beat Happee (2017) Model Score (3,835.00)
# Key Features:
#  - Vestibular Luenberger Observer + 72-Muscle Per-Spindle Luenberger Observer
#  - 6-DOF Cervical Jacobian Decoder + Multisensory Mahony Spatial Observer
#  - Active Zero Placement via Lead Ratio Coupling (r_lead_sc, r_lead_ccr in [0.10, 0.55])
#  - Barrier-Free Asynchronous Candidate Cascade across 18 parallel workers
#    (Fast P-cores immediately advance to the next frequency without waiting for E-cores)
# ==============================================================================

FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 40.2786

# 6 Happee (2017) Experimental Frequencies (Longest-Processing-Time First / LPT order:
# schedules the heavy 0.31 Hz jobs first across physical cores so P-cores absorb extra
# long jobs and no E-core stragglers occur at the end of a generation)
TARGETS = [
    (0.31, 0.893,    4.5),
    (0.78, 0.840,  -98.9),
    (1.11, 0.814, -112.6),
    (5.52, 0.509, -194.6),
    (2.48, 0.655, -163.7),
    (7.95, 0.443, -247.7)
]

# Recalibrated Physiological Parameter Bounds for Sensor-Level Luenberger Pipeline
lower_bounds = np.array([
    0.45,   # 0: r_ton (K_ton / K_sc)
     8.0,   # 1: p_ton (rad/s)
    30.0,   # 2: K_sc (Nm/(rad/s))
    0.10,   # 3: r_lead_sc (z_sc / p_sc)
    25.0,   # 4: p_sc (rad/s)
     1.0,   # 5: K_phas (Nm/(m/s^2))
    25.0,   # 6: p_phas (rad/s)
    0.25,   # 7: kp (spindle stiffness gain)
    0.10,   # 8: r_lead_ccr (z_ccr / p_ccr)
    15.0,   # 9: p_ccr (rad/s)
    0.15,   # 10: k_gamma_dyn (x100 pulses/s -> 15 to 120 pps)
    0.15    # 11: k_gamma_stat (x100 pulses/s -> 15 to 120 pps)
])

upper_bounds = np.array([
    0.95,   # 0: r_ton
    40.0,   # 1: p_ton
    95.0,   # 2: K_sc
    0.55,   # 3: r_lead_sc
    90.0,   # 4: p_sc
    14.0,   # 5: K_phas
    85.0,   # 6: p_phas
    1.30,   # 7: kp
    0.55,   # 8: r_lead_ccr
    75.0,   # 9: p_ccr
    1.20,   # 10: k_gamma_dyn
    1.20    # 11: k_gamma_stat
])

def evaluate_freq(candidate, target_idx, w_idx, gen_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = int(target_sim_time * 240 * 2.5)
    worker_label = f"W{w_idx+1}_P21G{gen_idx}"
    
    try:
        cmd = [
            sys.executable, "cmaes_worker_p15.py",
            str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
            *[str(v) for v in candidate],
            str(freq), str(t_gain), str(t_phase),
            worker_label
        ]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=dynamic_timeout
        )
        if result.returncode == 0 and result.stdout.strip():
            score = float(result.stdout.strip().split('\n')[-1])
            return score
        else:
            return 100000.0 + random.uniform(0, 100.0)
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] on {freq}Hz [{worker_label}]", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"[ERROR] on {freq}Hz [{worker_label}]: {e}", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)

def main():
    # Recalibrated Initial Vector x0 based on 6-Frequency Bode Verification
    x0 = np.array([
        0.78,   # 0: r_ton (K_ton = 45.24)
        22.0,   # 1: p_ton
        58.0,   # 2: K_sc
        0.22,   # 3: r_lead_sc (z_sc = 12.1 rad/s)
        55.0,   # 4: p_sc
         5.5,   # 5: K_phas
        55.0,   # 6: p_phas
        0.60,   # 7: kp
        0.25,   # 8: r_lead_ccr (z_ccr = 9.5 rad/s)
        38.0,   # 9: p_ccr
        0.48,   # 10: k_dyn (48 pps)
        0.38    # 11: k_stat (38 pps)
    ])
    
    # Recalibrated Search Sigmas (~12-15% of parameter range)
    sigma0 = np.array([
        0.07,   # r_ton
        4.0,    # p_ton
        8.0,    # K_sc
        0.06,   # r_lead_sc
        8.0,    # p_sc
        1.5,    # K_phas
        7.0,    # p_phas
        0.12,   # kp
        0.06,   # r_lead_ccr
        6.0,    # p_ccr
        0.12,   # k_dyn
        0.12    # k_stat
    ])
    
    max_total_generations = 30
    target_stop_generation = int(os.environ.get("PHASE2_1_STOP_GEN", "30"))
    
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': max_total_generations,
        'seed': 2026
    })
    
    os.makedirs("logs", exist_ok=True)
    os.makedirs("logs/candidates", exist_ok=True)
    
    # Check for completed Phase 2.1 generations if resuming
    completed_gens = 0
    global_best_score = float('inf')
    global_best_sol = x0.copy()
    
    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_1_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            cands = es.ask()
            es.tell(cands, sdata["all_scores"])
            completed_gens = g
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = cands[sdata["best_candidate_idx"] - 1]
            print(f"[REPLAY] Phase 2.1 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    print(f"\nPhase 2.1 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Initial Candidate x0:\n{x0}")
    print(f"Sigma0:\n{sigma0}")
    
    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        print(f"\n{'='*35} STARTING PHASE 2.1 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)
        
        candidates = es.ask()
        all_scores = [0.0] * len(candidates)
        
        # Barrier-Free Asynchronous LPT Cascade across 14 physical cores (6 P-cores + 8 E-cores):
        # Submit frequency 0 (0.31 Hz, longest job) first across 14 workers to avoid Hyper-Threading
        # FPU contention and ensure fast P-cores absorb the remaining 0.31 Hz jobs before E-cores finish.
        with concurrent.futures.ThreadPoolExecutor(max_workers=14) as executor:
            future_to_task = {}
            for c_idx in range(len(candidates)):
                fut = executor.submit(evaluate_freq, candidates[c_idx], 0, c_idx, iteration)
                future_to_task[fut] = (c_idx, 0)
                
            while future_to_task:
                done, _ = concurrent.futures.wait(
                    future_to_task.keys(),
                    return_when=concurrent.futures.FIRST_COMPLETED
                )
                for fut in done:
                    c_idx, t_idx = future_to_task.pop(fut)
                    freq, _, _ = TARGETS[t_idx]
                    score = fut.result()
                    
                    if score < 50000.0:
                        all_scores[c_idx] += score
                        print(f"[PASS] [W{c_idx+1}_P21G{iteration}] passed {freq}Hz ({t_idx+1}/{len(TARGETS)}) (Score: {score:.1f} | Cum: {all_scores[c_idx]:.1f})", flush=True)
                        if t_idx + 1 < len(TARGETS):
                            next_fut = executor.submit(evaluate_freq, candidates[c_idx], t_idx + 1, c_idx, iteration)
                            future_to_task[next_fut] = (c_idx, t_idx + 1)
                        else:
                            print(f"[COMPLETE] [W{c_idx+1}_P21G{iteration}] finished all 6 frequencies! Total Score: {all_scores[c_idx]:.2f}", flush=True)
                    else:
                        penalty = (len(TARGETS) - t_idx) * 20000.0
                        all_scores[c_idx] += (score + penalty)
                        print(f"[EARLY STOPPING] [W{c_idx+1}_P21G{iteration}] failed on {freq}Hz (Score: {score:.1f}). Penalty added.", flush=True)
                        
        es.tell(candidates, all_scores)
        
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds()
        gen_best_idx = int(np.argmin(all_scores))
        gen_best_score = float(all_scores[gen_best_idx])
        gen_best_sol = candidates[gen_best_idx]
        
        r_ton_val = float(gen_best_sol[0])
        K_sc_val = float(gen_best_sol[2])
        K_ton_val = r_ton_val * K_sc_val
        
        r_lead_sc_val = float(gen_best_sol[3])
        p_sc_val = float(gen_best_sol[4])
        z_sc_val = r_lead_sc_val * p_sc_val
        
        r_lead_ccr_val = float(gen_best_sol[8])
        p_ccr_val = float(gen_best_sol[9])
        z_ccr_val = r_lead_ccr_val * p_ccr_val
        
        gen_summary = {
            "phase": 2.1,
            "generation": iteration,
            "start_time": gen_start_time.isoformat(),
            "end_time": datetime.datetime.now().isoformat(),
            "duration_min": gen_duration / 60.0,
            "best_candidate_idx": int(gen_best_idx + 1),
            "best_score": float(gen_best_score),
            "r_ton": r_ton_val,
            "best_params": {
                "r_ton": r_ton_val,
                "K_ton": K_ton_val,
                "p_ton": float(gen_best_sol[1]),
                "K_sc": K_sc_val,
                "r_lead_sc": r_lead_sc_val,
                "z_sc": z_sc_val,
                "p_sc": p_sc_val,
                "K_phas": float(gen_best_sol[5]),
                "p_phas": float(gen_best_sol[6]),
                "kp": float(gen_best_sol[7]),
                "r_lead_ccr": r_lead_ccr_val,
                "z_ccr": z_ccr_val,
                "p_ccr": p_ccr_val,
                "k_dyn": float(gen_best_sol[10]),
                "k_stat": float(gen_best_sol[11])
            },
            "all_scores": [float(s) for s in all_scores]
        }
        
        with open(f"logs/phase2_1_gen_{iteration}_summary.json", "w", encoding="utf-8") as f_sum:
            json.dump(gen_summary, f_sum, indent=2)
            
        if gen_best_score < global_best_score:
            global_best_score = gen_best_score
            global_best_sol = gen_best_sol
            print(f"\n>>> NEW PHASE 2.1 RECORD in Gen {iteration}: {global_best_score:.2f} (Candidate W{gen_best_idx+1}_P21G{iteration}) <<<", flush=True)
            
        print(f"\nPhase 2.1 Generation {iteration} completed in {gen_duration/60.0:.1f} mins.", flush=True)
        print(f"Best Candidate: W{gen_best_idx+1}_P21G{iteration} | Score: {gen_best_score:.2f}", flush=True)
        print(f"r_ton: {r_ton_val:.3f} | K_ton: {K_ton_val:.2f} | K_sc: {K_sc_val:.2f} | z_sc: {z_sc_val:.2f} | z_ccr: {z_ccr_val:.2f}", flush=True)

    print("\n" + "="*90, flush=True)
    print(f"PHASE 2.1 TARGET GENERATIONS (1..{target_stop_generation}) COMPLETE!", flush=True)
    print(f"Final Global Best Score: {global_best_score:.2f}", flush=True)
    print("="*90, flush=True)

if __name__ == "__main__":
    main()
