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
# PHASE 2.1 CONSOLIDATED CAMPAIGN
# Target: Beat Happee (2017) Model Score (3,835.00)
# Key Features:
#  - Active Zero Placement via Lead Ratio Coupling (r_lead_sc, r_lead_ccr in [0.10, 0.45])
#  - Canal Gain Bounded to [35, 95] Nm/(rad/s)
#  - Spindle Stiffness Gain Bounded to [0.80, 1.70]
#  - Dead k_v cleanly eliminated from controller & worker
#  - Sensible, well-scaled sigma0 vector across all 12 dimensions
#  - Initialized with Phase 2 best parameters + analytical resonance zeros
# ==============================================================================

FIXED_W_N = 0.0
FIXED_ZETA = 0.0
FIXED_KI = 0.0

# 6 Happee (2017) Experimental Frequencies (High-to-low early screening)
TARGETS = [
    (7.95, 0.443, -247.7),
    (5.52, 0.509, -194.6),
    (2.48, 0.655, -163.7),
    (1.11, 0.814, -112.6),
    (0.78, 0.840,  -98.9),
    (0.31, 0.893,    4.5)
]

# Consolidated Physiological Parameter Bounds
lower_bounds = np.array([
    0.50,   # 0: r_ton (K_ton / K_sc)
    10.0,   # 1: p_ton (rad/s)
    35.0,   # 2: K_sc (Nm/(rad/s))
    0.10,   # 3: r_lead_sc (z_sc / p_sc) -> 30-50 deg lead
    20.0,   # 4: p_sc (rad/s)
     2.0,   # 5: K_phas (Nm/(m/s^2))
    30.0,   # 6: p_phas (rad/s)
    0.80,   # 7: kp (spindle stiffness gain)
    0.10,   # 8: r_lead_ccr (z_ccr / p_ccr) -> 30-50 deg lead
    10.0,   # 9: p_ccr (rad/s)
    0.20,   # 10: k_gamma_dyn
    0.20    # 11: k_gamma_stat
])

upper_bounds = np.array([
    0.95,   # 0: r_ton
    40.0,   # 1: p_ton
    95.0,   # 2: K_sc (bounded away from 120 ceiling)
    0.45,   # 3: r_lead_sc
    70.0,   # 4: p_sc
    15.0,   # 5: K_phas
    90.0,   # 6: p_phas
    1.70,   # 7: kp (bounded away from stiffness blowout)
    0.45,   # 8: r_lead_ccr
    50.0,   # 9: p_ccr
    2.00,   # 10: k_gamma_dyn
    2.00    # 11: k_gamma_stat
])

def evaluate_freq(candidate, target_idx, w_idx, gen_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = int(target_sim_time * 240 * 2.0)
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
    # Initial vector: Phase 2 champions + analytical lead zeros
    x0 = np.array([
        0.80,   # 0: r_ton
        22.0,   # 1: p_ton
        55.0,   # 2: K_sc
        0.20,   # 3: r_lead_sc (z_sc = 0.20 * 42 = 8.4 rad/s)
        42.0,   # 4: p_sc
         6.0,   # 5: K_phas
        60.0,   # 6: p_phas
        1.30,   # 7: kp
        0.25,   # 8: r_lead_ccr (z_ccr = 0.25 * 25 = 6.25 rad/s)
        25.0,   # 9: p_ccr
        0.95,   # 10: k_dyn
        0.98    # 11: k_stat
    ])
    
    # Sensible standard deviations: ~10% of parameter range
    sigma0 = np.array([
        0.05,   # r_ton
        3.0,    # p_ton
        6.0,    # K_sc
        0.04,   # r_lead_sc
        5.0,    # p_sc
        1.2,    # K_phas
        6.0,    # p_phas
        0.10,   # kp
        0.04,   # r_lead_ccr
        4.0,    # p_ccr
        0.15,   # k_dyn
        0.15    # k_stat
    ])
    
    num_generations = 25
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': num_generations,
        'seed': 2026
    })
    
    os.makedirs("logs", exist_ok=True)
    os.makedirs("logs/candidates", exist_ok=True)
    
    # Check for completed Phase 2.1 generations if resuming
    completed_gens = 0
    global_best_score = float('inf')
    global_best_sol = x0.copy()
    
    for g in range(1, num_generations + 1):
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
            
    print(f"\nPhase 2.1 Initialized. Starting from Generation {completed_gens + 1} of {num_generations}")
    print(f"Initial Candidate x0:\n{x0}")
    print(f"Sigma0:\n{sigma0}")
    
    iteration = completed_gens
    while not es.stop() and iteration < num_generations:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        print(f"\n{'='*35} STARTING PHASE 2.1 GENERATION {iteration} of {num_generations} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}")
        
        candidates = es.ask()
        all_scores = [0.0] * len(candidates)
        active_candidates = list(range(len(candidates)))
        
        for t_idx, (freq, t_gain, t_phase) in enumerate(TARGETS):
            if not active_candidates:
                break
                
            print(f"\n--- Evaluating {len(active_candidates)} candidates on {freq}Hz (Phase 2.1 Gen {iteration}) ---")
            
            with concurrent.futures.ThreadPoolExecutor(max_workers=18) as executor:
                future_to_c = {
                    executor.submit(evaluate_freq, candidates[c_idx], t_idx, c_idx, iteration): c_idx
                    for c_idx in active_candidates
                }
                
                next_active = []
                for future in concurrent.futures.as_completed(future_to_c):
                    c_idx = future_to_c[future]
                    score = future.result()
                    
                    if score < 50000.0:
                        all_scores[c_idx] += score
                        next_active.append(c_idx)
                        print(f"[PASS] [W{c_idx+1}_P21G{iteration}] passed {freq}Hz (Score: {score:.1f})")
                    else:
                        penalty = (len(TARGETS) - t_idx) * 20000.0
                        all_scores[c_idx] += (score + penalty)
                        print(f"[EARLY STOPPING] [W{c_idx+1}_P21G{iteration}] failed on {freq}Hz (Score: {score:.1f}). Penalty added.")
                        
            active_candidates = next_active
            
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
            print(f"\n>>> NEW PHASE 2.1 RECORD in Gen {iteration}: {global_best_score:.2f} (Candidate W{gen_best_idx+1}_P21G{iteration}) <<<")
            
        print(f"\nPhase 2.1 Generation {iteration} completed in {gen_duration/60.0:.1f} mins.")
        print(f"Best Candidate: W{gen_best_idx+1}_P21G{iteration} | Score: {gen_best_score:.2f}")
        print(f"r_ton: {r_ton_val:.3f} | K_ton: {K_ton_val:.2f} | K_sc: {K_sc_val:.2f} | z_sc: {z_sc_val:.2f} | z_ccr: {z_ccr_val:.2f}")

    print("\n" + "="*90)
    print("PHASE 2.1 CAMPAIGN COMPLETE!")
    print(f"Final Global Best Score: {global_best_score:.2f}")
    print("="*90)

if __name__ == "__main__":
    main()

