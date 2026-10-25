"""
Phase 2 Fresh Optimization Campaign: 25 Generations (P2_G1 through P2_G25).
Calibrated specifically for the new physiological plant:
  - True 35ms reflex latency (axonal transport + polysynaptic transmission + EMD)
  - 3% baseline antagonistic muscle co-contraction (u_min = 0.03)
  - Responsive Otolith Tilt Innovation (Kp_oto = 0.25 rad/s)
  - Biological Ratio Coupling: K_ton = r_ton * K_sc (r_ton in [0.40, 1.20])
Fresh CMA-ES initialization with unconstrained spherical covariance exploration.
"""

import cma
import concurrent.futures
import subprocess
import sys
import numpy as np
import math
import os
import random
import datetime
import json

class Tee(object):
    def __init__(self, name, mode):
        os.makedirs(os.path.dirname(name), exist_ok=True)
        self.file = open(name, mode, encoding="utf-8")
        self.stdout = sys.stdout
        sys.stdout = self
    def __del__(self):
        try:
            sys.stdout = self.stdout
            self.file.close()
        except Exception:
            pass
    def write(self, data):
        self.file.write(data)
        try:
            self.stdout.write(data)
        except UnicodeEncodeError:
            self.stdout.write(data.encode('ascii', errors='replace').decode('ascii'))
        self.file.flush()
        self.stdout.flush()
    def flush(self):
        self.file.flush()
        self.stdout.flush()

# Tee stdout only to avoid subprocess fileno issues
sys.stdout = Tee("logs/phase2_campaign_25gens.log", "a")

print("="*90)
print("PHASE 2 FRESH OPTIMIZATION CAMPAIGN: 25 GENERATIONS (GEN 1 THROUGH 25)")
print("Model Plant: 35ms True Delay | 3% Co-Contraction | Kp_oto=0.25 | r_ton Coupling")
print(f"Started at: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
print("="*90)

FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 40.2786

TARGETS = [
    (7.95, 0.443, -247.7),  # F6
    (5.52, 0.509, -194.6),  # F5
    (2.48, 0.655, -163.7),  # F4
    (1.11, 0.814, -112.6),  # F3
    (0.78, 0.840,  -98.9),  # F2
    (0.31, 0.893,    4.5)   # F1
]

# Bounds for [r_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, k_dyn, k_stat]
lower_bounds = np.array([
     0.40,   5.0,
    20.0,  10.0,  10.0,
     1.0,  15.0,
     0.5,  50.0,   5.0,
     0.1,   0.1
])

upper_bounds = np.array([
     1.20,  50.0,
   120.0, 600.0, 100.0,
    20.0, 120.0,
     6.0, 800.0,  60.0,
     3.0,   3.0
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
    worker_label = f"W{w_idx+1}_P2G{gen_idx}"
    
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
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=dynamic_timeout
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
    # Calibrated Initial Mean
    x0 = np.array([
        0.80,  # r_ton
        18.0,  # p_ton
        60.0,  # K_sc
       120.0,  # z_sc
        55.0,  # p_sc
         6.0,  # K_phas
        52.0,  # p_phas
         1.65, # kp_prop
       600.0,  # z_ccr
        22.0,  # p_ccr
         0.88, # k_dyn
         1.05  # k_stat
    ])
    
    # Healthy Uncorrelated Exploration Sigmas (~10-15% parameter range)
    sigma0 = np.array([
        0.10,  # r_ton
        3.0,   # p_ton
       10.0,   # K_sc
       40.0,   # z_sc
        8.0,   # p_sc
        1.5,   # K_phas
        8.0,   # p_phas
        0.30,  # kp_prop
       80.0,   # z_ccr
        3.5,   # p_ccr
        0.15,  # k_dyn
        0.15   # k_stat
    ])
    
    num_generations = 25
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': num_generations,
        'seed': 100
    })
    
    global_best_score = float('inf')
    global_best_sol = x0.copy()
    
    iteration = 0
    while not es.stop() and iteration < num_generations:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        print(f"\n{'='*35} STARTING PHASE 2 GENERATION {iteration} of {num_generations} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}")
        
        candidates = es.ask()
        all_scores = [0.0] * len(candidates)
        active_candidates = list(range(len(candidates)))
        
        for t_idx, (freq, t_gain, t_phase) in enumerate(TARGETS):
            if not active_candidates:
                break
                
            print(f"\n--- Evaluating {len(active_candidates)} candidates on {freq}Hz (Phase 2 Gen {iteration}) ---")
            
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
                        print(f"[PASS] [W{c_idx+1}_P2G{iteration}] passed {freq}Hz (Score: {score:.1f})")
                    else:
                        penalty = (len(TARGETS) - t_idx) * 20000.0
                        all_scores[c_idx] += (score + penalty)
                        print(f"[EARLY STOPPING] [W{c_idx+1}_P2G{iteration}] failed on {freq}Hz (Score: {score:.1f}). Penalty added.")
                        
            active_candidates = next_active
            
        es.tell(candidates, all_scores)
        
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds()
        gen_best_idx = int(np.argmin(all_scores))
        gen_best_score = float(all_scores[gen_best_idx])
        gen_best_sol = candidates[gen_best_idx]
        
        r_ton_val = float(gen_best_sol[0])
        K_sc_val = float(gen_best_sol[2])
        K_ton_val = r_ton_val * K_sc_val
        
        gen_summary = {
            "phase": 2,
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
                "z_sc": float(gen_best_sol[3]),
                "p_sc": float(gen_best_sol[4]),
                "K_phas": float(gen_best_sol[5]),
                "p_phas": float(gen_best_sol[6]),
                "kp": float(gen_best_sol[7]),
                "z_ccr": float(gen_best_sol[8]),
                "p_ccr": float(gen_best_sol[9]),
                "k_dyn": float(gen_best_sol[10]),
                "k_stat": float(gen_best_sol[11])
            },
            "all_scores": [float(s) for s in all_scores]
        }
        
        os.makedirs("logs", exist_ok=True)
        with open(f"logs/phase2_gen_{iteration}_summary.json", "w", encoding="utf-8") as f_sum:
            json.dump(gen_summary, f_sum, indent=2)
            
        if gen_best_score < global_best_score:
            global_best_score = gen_best_score
            global_best_sol = gen_best_sol
            print(f"\n>>> NEW PHASE 2 RECORD in Gen {iteration}: {global_best_score:.2f} (Candidate W{gen_best_idx+1}_P2G{iteration}) <<<")
            
        print(f"\nPhase 2 Generation {iteration} completed in {gen_duration/60.0:.1f} mins.")
        print(f"Best Candidate: W{gen_best_idx+1}_P2G{iteration} | Score: {gen_best_score:.2f}")
        print(f"r_ton: {r_ton_val:.3f} | K_ton: {K_ton_val:.2f} | K_sc: {K_sc_val:.2f}")

    print("\nPhase 2 25-Generation Campaign Complete!")
    print(f"Final Global Best Score: {global_best_score:.2f}")

if __name__ == "__main__":
    main()

