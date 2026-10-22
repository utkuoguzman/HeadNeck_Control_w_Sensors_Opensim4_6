"""
Resume Phase 1.5 CMA-ES Multi-Generation Campaign from Generation 35 to 50
Targets Happee (2017) Human Experimental Frequency Responses.
Automatically inherits the adapted CMA-ES state from Generation 34.
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
        sys.stdout = self.stdout
        self.file.close()
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

timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
Tee(f"logs/cmaes_p15_resume_gen35_{timestamp}.log", "w")

print("="*90)
print("PHASE 1.5 RESUMPTION: EXECUTING GENERATIONS 35 THROUGH 50")
print("Targeting Happee (2017) Real Experimental Bode Curves (Angular Phase & Gain)")
print(f"Resumed at: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
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

best_score = 466.21
# Seed from Gen 34 Champion W17_G34
x0 = np.array([
    72.13,    # K_ton
    14.64,    # p_ton
    53.43,    # K_sc
   552.35,    # z_sc
    46.46,    # p_sc
     4.57,    # K_phas
    60.26,    # p_phas
     2.19,    # kp_prop
   515.68,    # z_ccr
    19.26,    # p_ccr
     0.88,    # k_gamma_dyn
     1.06     # k_gamma_stat
])

if os.path.exists("logs/gen_34_summary.json"):
    try:
        with open("logs/gen_34_summary.json", "r") as f:
            g34 = json.load(f)
            p = g34["best_params"]
            x0 = np.array([
                p["K_ton"], p["p_ton"],
                p["K_sc"], p["z_sc"], p["p_sc"],
                p["K_phas"], p["p_phas"],
                p["kp"], p["z_ccr"], p["p_ccr"],
                p["k_dyn"], p["k_stat"]
            ])
            print(f"Loaded Gen 34 Champion seed with score: {g34['best_score']:.2f}")
    except Exception as e:
        print(f"Note: Error parsing gen_34_summary.json ({e}), using default seed.")

sigma0 = np.array([
     1.38, 0.46, 1.14, 25.0, 1.5, 0.28, 1.1, 0.11, 25.0, 0.85, 0.027, 0.027
])
initial_sigma = 1.0

try:
    if os.path.exists("outcmaes/stddev.dat"):
        with open("outcmaes/stddev.dat", "r") as f_std:
            lines = [l.strip() for l in f_std if l.strip() and not l.startswith("%")]
            if lines:
                last_line = lines[-1].split()
                if len(last_line) >= 17:
                    initial_sigma = float(last_line[2])
                    adapted_stds = [float(v) for v in last_line[5:17]]
                    sigma0 = np.array(adapted_stds)
                    print(f"Inherited adapted CMA-ES sigma={initial_sigma:.4f} and coordinate stds from outcmaes/stddev.dat")
except Exception as e:
    print(f"Note: Could not parse outcmaes/stddev.dat ({e}), using calibrated defaults.")

lower_bounds = np.array([
    20.0,   5.0,
    15.0,   5.0,  10.0,
     1.0,  20.0,
     0.5,   5.0,   5.0,
     0.1,   0.1
])

upper_bounds = np.array([
    150.0,  50.0,
    100.0, 800.0,  80.0,
     20.0, 120.0,
      8.0, 800.0,  60.0,
      4.0,   4.0
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
    worker_label = f"W{w_idx+1}_G{gen_idx}"
    
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
            stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=dynamic_timeout
        )
        
        if result.returncode == 0 and result.stdout.strip():
            score = float(result.stdout.strip().split('\n')[-1])
            return score
        else:
            return 100000.0 + random.uniform(0, 100.0)
    except subprocess.TimeoutExpired:
        print(f"TIMEOUT on {freq}Hz [{worker_label}]", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"ERROR on {freq}Hz [{worker_label}]: {e}", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)

def main():
    es = cma.CMAEvolutionStrategy(x0, initial_sigma, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': 16, # Gen 35 to 50
        'seed': 46
    })
    
    iteration = 34 # Next iteration is 35
    global_best_score = best_score
    global_best_sol = x0.copy()

    while not es.stop():
        iteration += 1
        gen_start_time = datetime.datetime.now()
        print(f"\n{'='*35} STARTING GENERATION {iteration} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}")
        
        solutions = es.ask()
        scores = np.zeros(len(solutions))
        active_candidates = list(range(len(solutions)))
        
        for target_idx in range(len(TARGETS)):
            if not active_candidates:
                break
                
            freq = TARGETS[target_idx][0]
            print(f"\n--- Evaluating {len(active_candidates)} candidates on {freq}Hz (Gen {iteration}) ---")
            
            with concurrent.futures.ThreadPoolExecutor(max_workers=18) as executor:
                futures = {}
                for idx in active_candidates:
                    futures[executor.submit(evaluate_freq, solutions[idx], target_idx, idx, iteration)] = idx
                    
                for future in concurrent.futures.as_completed(futures):
                    idx = futures[future]
                    score = future.result()
                    scores[idx] += score
                    
                    if score > 500000.0:
                        remaining_freqs = (len(TARGETS) - 1) - target_idx
                        penalty = remaining_freqs * 1000000.0
                        scores[idx] += penalty
                        active_candidates.remove(idx)
                        print(f"[EARLY STOPPING] [W{idx+1}_G{iteration}] failed on {freq}Hz (Score: {score:.1f}). Penalty added.")
                    else:
                        print(f"[PASS] [W{idx+1}_G{iteration}] passed {freq}Hz (Score: {score:.1f})")
                        
        es.tell(solutions, scores)
        es.logger.add()
        es.disp()
        
        gen_best_idx = np.argmin(scores)
        gen_best_sol = solutions[gen_best_idx]
        gen_best_score = scores[gen_best_idx]
        gen_end_time = datetime.datetime.now()
        gen_duration = (gen_end_time - gen_start_time).total_seconds()
        
        print(f"\n{'='*40} PHASE 1.5 GENERATION {iteration} SUMMARY {'='*40}")
        print(f"Duration: {gen_duration/60:.1f} min ({gen_duration:.1f} s)")
        print(f"Gen Best Score: {gen_best_score:.2f}")
        print(f"K_ton={gen_best_sol[0]:.2f}, p_ton={gen_best_sol[1]:.2f}")
        print(f"K_sc={gen_best_sol[2]:.2f}, z_sc={gen_best_sol[3]:.2f}, p_sc={gen_best_sol[4]:.2f}")
        print(f"K_phas={gen_best_sol[5]:.2f}, p_phas={gen_best_sol[6]:.2f}")
        print(f"kp={gen_best_sol[7]:.2f}, z_ccr={gen_best_sol[8]:.2f}, p_ccr={gen_best_sol[9]:.2f}")
        print(f"k_dyn={gen_best_sol[10]:.2f}, k_stat={gen_best_sol[11]:.2f}")
        
        gen_summary = {
            "generation": iteration,
            "start_time": gen_start_time.isoformat(),
            "end_time": gen_end_time.isoformat(),
            "duration_min": gen_duration / 60.0,
            "best_candidate_idx": int(gen_best_idx + 1),
            "best_score": float(gen_best_score),
            "best_params": {
                "K_ton": float(gen_best_sol[0]), "p_ton": float(gen_best_sol[1]),
                "K_sc": float(gen_best_sol[2]), "z_sc": float(gen_best_sol[3]), "p_sc": float(gen_best_sol[4]),
                "K_phas": float(gen_best_sol[5]), "p_phas": float(gen_best_sol[6]),
                "kp": float(gen_best_sol[7]), "z_ccr": float(gen_best_sol[8]), "p_ccr": float(gen_best_sol[9]),
                "k_dyn": float(gen_best_sol[10]), "k_stat": float(gen_best_sol[11])
            },
            "all_scores": [float(s) for s in scores]
        }
        with open(f"logs/gen_{iteration}_summary.json", "w", encoding="utf-8") as f_sum:
            json.dump(gen_summary, f_sum, indent=2)
        
        if gen_best_score < global_best_score:
            global_best_score = gen_best_score
            global_best_sol = gen_best_sol.copy()
            with open("logs/cmaes_p15_best.txt", "w", encoding="utf-8") as f_best:
                f_best.write(f"Generation: {iteration}\n")
                f_best.write(f"Candidate: W{gen_best_idx+1}_G{iteration}\n")
                f_best.write(f"Total Score: {gen_best_score:.4f}\n")
                f_best.write(f"K_ton={gen_best_sol[0]:.6f}, p_ton={gen_best_sol[1]:.6f}\n")
                f_best.write(f"K_sc={gen_best_sol[2]:.6f}, z_sc={gen_best_sol[3]:.6f}, p_sc={gen_best_sol[4]:.6f}\n")
                f_best.write(f"K_phas={gen_best_sol[5]:.6f}, p_phas={gen_best_sol[6]:.6f}\n")
                f_best.write(f"kp={gen_best_sol[7]:.6f}, z_ccr={gen_best_sol[8]:.6f}, p_ccr={gen_best_sol[9]:.6f}\n")
                f_best.write(f"k_dyn={gen_best_sol[10]:.6f}, k_stat={gen_best_sol[11]:.6f}\n")
            print(f"*** NEW ALL-TIME PROJECT CHAMPION RECORD: {gen_best_score:.2f} (Gen {iteration}) ***")

if __name__ == "__main__":
    main()

