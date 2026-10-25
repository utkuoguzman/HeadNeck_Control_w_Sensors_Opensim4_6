import cma
import concurrent.futures
import subprocess
import sys
import numpy as np
import math
import os
import random
import datetime

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
Tee(f"logs/cmaes_tf_global_{timestamp}.log", "w")

print("="*90)
print("GLOBAL CMA-ES: CONTINUOUS TRANSFER FUNCTION OPTIMIZATION WITH EARLY STOPPING")
print("="*90)

# Fixed Task parameters
FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 40.2786

# Frequencies: We evaluate from highest to lowest for early stopping!
# Order: F6, F5, F4, F3, F2
TARGETS = [
    (7.95, 0.443, -29.8),  # F6
    (5.52, 0.509, -29.0),  # F5
    (2.48, 0.655, -20.6),  # F4
    (1.11, 0.814, -10.9),  # F3
    (0.78, 0.840,  -8.1),  # F2
    (0.31, 0.893,   2.5)   # F1 (Re-enabled for final fine-tuning)
]

# Initialized with Gen 5 best parameters (Score: 5132.50)
x0 = np.array([
    15.3215, 33.8800,  3.8980,  # K_ton, z_ton, p_ton
    16.4291,  9.0505, 56.8736,  # K_sc, z_sc, p_sc 
     1.3441, 12.5447, 10.3109,  # K_phas, z_phas, p_phas 
     1.2193, 22.0746, 55.8447,  # kp_prop, z_ccr, p_ccr 
     3.6906,  2.5544            # k_dyn, k_stat 
])

# Widened sigma for broader parameter exploration and faster convergence
sigma0 = np.array([
    2.0,  4.0, 0.8,    # K_ton, z, p (~10-15% exploration width)
    2.5,  2.0, 6.0,    # K_sc, z, p 
    0.25, 2.0, 1.5,    # K_phas, z, p 
    0.20, 3.0, 6.0,    # kp, z_ccr, p_ccr 
    0.4,  0.4          # k_dyn, k_stat 
])

# Bounds
lower_bounds = np.array([
    0.1, 0.001, 0.001,
    0.1, 0.001, 0.001,
    0.1, 0.001, 0.001,
    0.01, 0.1, 0.1,
    0.0, 0.0
])
upper_bounds = np.array([
    200.0, 100.0, 100.0,
    100.0, 100.0, 100.0,
    100.0, 100.0, 100.0,
    10.0, 100.0, 100.0,
    10.0, 10.0
])

def evaluate_freq(candidate, target_idx, w_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    # Calculate timeout based on freq:
    # Phase 1 (1.5s settle) + Phase 2 (1 cycle ramp) + Phase 3 (>= 2 periods of 1.8Hz)
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = int(target_sim_time * 240 * 2.0) # 240x real-time + 100% buffer
    
    try:
        cmd = [
            sys.executable, "cmaes_worker_tf.py",
            str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
            *[str(v) for v in candidate],
            str(freq), str(t_gain), str(t_phase),
            f"W{w_idx+1}"
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
        print(f"?? TIMEOUT on {freq}Hz")
        return 100000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"?? ERROR on {freq}Hz: {e}")
        return 100000.0 + random.uniform(0, 100.0)

def main():
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': 40,
        'seed': 42
    })
    
    iteration = 0
    global_best_score = 5132.50
    global_best_sol = x0.copy()

    while not es.stop():
        iteration += 1
        solutions = es.ask()
        scores = np.zeros(len(solutions))
        
        # Parallel evaluation with Early Stopping cascade
        # We start with target_idx = 0 (F6) and go down to 5 (F1)
        active_candidates = list(range(len(solutions)))
        
        for target_idx in range(len(TARGETS)):
            if not active_candidates:
                break
                
            freq = TARGETS[target_idx][0]
            print(f"\n--- Evaluating {len(active_candidates)} candidates on {freq}Hz ---")
            
            with concurrent.futures.ThreadPoolExecutor(max_workers=18) as executor:
                futures = {}
                for idx in active_candidates:
                    futures[executor.submit(evaluate_freq, solutions[idx], target_idx, idx)] = idx
                    
                for future in concurrent.futures.as_completed(futures):
                    idx = futures[future]
                    score = future.result()
                    scores[idx] += score
                    
                    if score > 500000.0:
                        # EARLY STOPPING: This candidate failed a fast frequency.
                        # It doesn't get to run the slow frequencies!
                        # Add a massive penalty for the remaining frequencies.
                        remaining_freqs = (len(TARGETS) - 1) - target_idx
                        penalty = remaining_freqs * 1000000.0
                        scores[idx] += penalty
                        active_candidates.remove(idx)
                        print(f"[EARLY STOPPING] [W{idx+1}] failed on {freq}Hz (Score: {score:.1f}). Penalty added.")
                    else:
                        print(f"[PASS] [W{idx+1}] passed {freq}Hz (Score: {score:.1f})")
                        
        es.tell(solutions, scores)
        es.logger.add()
        es.disp()
        
        gen_best_idx = np.argmin(scores)
        gen_best_sol = solutions[gen_best_idx]
        gen_best_score = scores[gen_best_idx]
        
        print(f"\n{'='*40} GENERATION {iteration} SUMMARY {'='*40}")
        print(f"Gen Best Score: {gen_best_score:.2f}")
        print(f"K_ton={gen_best_sol[0]:.2f}, z_ton={gen_best_sol[1]:.2f}, p_ton={gen_best_sol[2]:.2f}")
        print(f"K_sc={gen_best_sol[3]:.2f}, z_sc={gen_best_sol[4]:.2f}, p_sc={gen_best_sol[5]:.2f}")
        print(f"K_phas={gen_best_sol[6]:.2f}, z_phas={gen_best_sol[7]:.2f}, p_phas={gen_best_sol[8]:.2f}")
        print(f"kp={gen_best_sol[9]:.2f}, z_ccr={gen_best_sol[10]:.2f}, p_ccr={gen_best_sol[11]:.2f}")
        print(f"k_dyn={gen_best_sol[12]:.2f}, k_stat={gen_best_sol[13]:.2f}")
        
        if gen_best_score < global_best_score:
            global_best_score = gen_best_score
            global_best_sol = gen_best_sol
            print(f"[NEW ALL-TIME BEST SCORE: {global_best_score:.2f}] Saving to logs/cmaes_tf_global_best.txt...")
            with open("logs/cmaes_tf_global_best.txt", "w") as f:
                f.write(f"Generation {iteration} Best Parameters:\n")
                f.write("-" * 50 + "\n")
                f.write(f"Total Score: {global_best_score:.2f}\n")
                f.write(f"K_ton={global_best_sol[0]:.4f}, z_ton={global_best_sol[1]:.4f}, p_ton={global_best_sol[2]:.4f}\n")
                f.write(f"K_sc={global_best_sol[3]:.4f}, z_sc={global_best_sol[4]:.4f}, p_sc={global_best_sol[5]:.4f}\n")
                f.write(f"K_phas={global_best_sol[6]:.4f}, z_phas={global_best_sol[7]:.4f}, p_phas={global_best_sol[8]:.4f}\n")
                f.write(f"kp={global_best_sol[9]:.4f}, z_ccr={global_best_sol[10]:.4f}, p_ccr={global_best_sol[11]:.4f}\n")
                f.write(f"k_dyn={global_best_sol[12]:.4f}, k_stat={global_best_sol[13]:.4f}\n")

if __name__ == "__main__":
    main()
