import cma
import concurrent.futures
import subprocess
import sys
import numpy as np
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
        self.stdout.write(data)
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
FIXED_KI = 6.54

# Frequencies: We evaluate from highest to lowest for early stopping!
# Order: F6, F5, F4, F3, F2, F1
TARGETS = [
    (7.95, 0.443, -29.8),  # F6
    (5.52, 0.509, -29.0),  # F5
    (2.48, 0.655, -20.6),  # F4
    (1.11, 0.814, -10.9),  # F3
    (0.78, 0.840,  -8.1),  # F2
    (0.31, 0.893,   2.5)   # F1
]

# Educated starting parameters based on VCR/CCR biological pathways
x0 = np.array([
    2.0, 5.0, 1.0,   # K_ton, z_ton, p_ton (Tonic: Angular Pos -> Torque. DC gain = 10.0, low-pass)
    0.5, 1.0, 1.0,   # K_sc, z_sc, p_sc (SC: Angular Vel -> Torque. Flat damping = 0.5)
    0.1, 1.0, 5.0,   # K_phas, z_phas, p_phas (Phasic: Lin Acc -> Torque. Low-pass inertia comp)
    0.02, 100.0, 10.0, # kp_prop, z_ccr, p_ccr (CCR: Low high-freq gain, DC gain = 1.0)
    0.13, 0.45       # k_dyn, k_stat (CCR: Spindle sensitivity drives)
])

sigma0 = np.array([
    1.0, 2.0, 0.5,   # K_ton, z, p
    0.5, 1.0, 1.0,   # K_sc, z, p
    0.1, 1.0, 2.0,  # K_phas, z, p
    0.1, 5.0, 5.0,   # kp, z_ccr, p_ccr
    0.1, 0.2        # k_dyn, k_stat
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
    5.0, 100.0, 100.0,
    5.0, 5.0
])

def evaluate_freq(candidate, target_idx, w_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    # Calculate timeout based on freq (slower freq = longer simulation)
    # F6 (7.95Hz) runs 0.37s -> ~2 mins
    # F1 (0.31Hz) runs 9.6s -> ~40 mins
    # The worker simulates a 0.5s settle delay + 2 periods (2.0 / freq)
    target_sim_time = 0.5 + (2.0 / freq)
    dynamic_timeout = int(target_sim_time * 240 * 1.5) # 240x real-time + 50% buffer
    
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
        'seed': 42
    })
    
    iteration = 0
    while not es.stop():
        iteration += 1
        solutions = es.ask()
        scores = np.zeros(len(solutions))
        
        # Parallel evaluation with Early Stopping cascade
        # We start with target_idx = 0 (F6) and go down to 5 (F1)
        active_candidates = list(range(len(solutions)))
        
        for target_idx in range(6):
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
                        remaining_freqs = 5 - target_idx
                        penalty = remaining_freqs * 1000000.0
                        scores[idx] += penalty
                        active_candidates.remove(idx)
                        print(f"?? EARLY STOPPING [W{idx+1}] failed on {freq}Hz (Score: {score:.1f}). Penalty added.")
                    else:
                        print(f"? SUCCESS [W{idx+1}] passed {freq}Hz (Score: {score:.1f})")
                        
        es.tell(solutions, scores)
        es.logger.add()
        es.disp()
        
        best_idx = np.argmin(scores)
        best_sol = solutions[best_idx]
        best_score = scores[best_idx]
        
        print(f"\n{'='*40} GENERATION {iteration} SUMMARY {'='*40}")
        print(f"Best Total Score: {best_score:.1f}")
        print(f"K_ton={best_sol[0]:.2f}, z_ton={best_sol[1]:.2f}, p_ton={best_sol[2]:.2f}")
        print(f"K_sc={best_sol[3]:.2f}, z_sc={best_sol[4]:.2f}, p_sc={best_sol[5]:.2f}")
        print(f"K_phas={best_sol[6]:.2f}, z_phas={best_sol[7]:.2f}, p_phas={best_sol[8]:.2f}")
        print(f"kp={best_sol[9]:.2f}, z_ccr={best_sol[10]:.2f}, p_ccr={best_sol[11]:.2f}")
        print(f"k_dyn={best_sol[12]:.2f}, k_stat={best_sol[13]:.2f}")
        
        with open("logs/cmaes_tf_global_best.txt", "w") as f:
            f.write(f"Generation {iteration} Best Parameters:\n")
            f.write("-" * 50 + "\n")
            f.write(f"Total Score: {best_score:.2f}\n")
            f.write(f"K_ton={best_sol[0]:.4f}, z_ton={best_sol[1]:.4f}, p_ton={best_sol[2]:.4f}\n")
            f.write(f"K_sc={best_sol[3]:.4f}, z_sc={best_sol[4]:.4f}, p_sc={best_sol[5]:.4f}\n")
            f.write(f"K_phas={best_sol[6]:.4f}, z_phas={best_sol[7]:.4f}, p_phas={best_sol[8]:.4f}\n")
            f.write(f"kp={best_sol[9]:.4f}, z_ccr={best_sol[10]:.4f}, p_ccr={best_sol[11]:.4f}\n")
            f.write(f"k_dyn={best_sol[12]:.4f}, k_stat={best_sol[13]:.4f}\n")

if __name__ == "__main__":
    main()
