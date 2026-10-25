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
Tee(f"logs/cmaes_p15_resumed_{timestamp}.log", "w")

print("="*90)
print("PHASE 1.5 RESUMPTION: SMART OPTIMIZATION WITH 14:15 DEADLINE SAFEGUARD")
print("="*90)

# Fixed Task parameters (calibrated in Phase 1)
FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 40.2786

# Target frequencies: Evaluated from highest to lowest for early stopping!
TARGETS = [
    (7.95, 0.443, -29.8),  # F6
    (5.52, 0.509, -29.0),  # F5
    (2.48, 0.655, -20.6),  # F4
    (1.11, 0.814, -10.9),  # F3
    (0.78, 0.840,  -8.1),  # F2
    (0.31, 0.893,   2.5)   # F1
]

# Initial vector x0: Anchored to Gen 6 All-Time Champion (Score: 88.35)
x0 = np.array([
    72.7457,   # K_ton
    15.5817,   # p_ton
    49.9993,   # K_sc
   399.0455,   # z_sc  <-- Gen 6 Champion Lead Zero
    39.5120,   # p_sc
     5.8847,   # K_phas
    65.4598,   # p_phas
     2.6128,   # kp_prop
   322.1083,   # z_ccr <-- Gen 6 Champion Lead Zero
    22.5190,   # p_ccr
     0.8045,   # k_gamma_dyn
     1.2110    # k_gamma_stat
])

# Covariance standard deviations:
# Focused around the discovered optimal valley (z_sc in [350, 450], z_ccr in [270, 370])
sigma0 = np.array([
     2.0,      # K_ton
     0.7,      # p_ton
     1.5,      # K_sc
    45.0,      # z_sc
     1.2,      # p_sc
     0.35,     # K_phas
     1.5,      # p_phas
     0.14,     # kp_prop
    45.0,      # z_ccr
     1.1,      # p_ccr
     0.038,    # k_gamma_dyn
     0.039     # k_gamma_stat
])

# Bounds
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

def evaluate_freq(candidate, target_idx, w_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = int(target_sim_time * 240 * 2.0)
    
    try:
        cmd = [
            sys.executable, "cmaes_worker_p15.py",
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
        print(f"?? TIMEOUT on {freq}Hz", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"?? ERROR on {freq}Hz: {e}", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)

def main():
    # Allow up to 3 generations (Gen 8, Gen 9, Gen 10)
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': 3,
        'seed': 43
    })
    
    iteration = 7 # Offset: starts at Generation 8
    global_best_score = 88.35 # Seeded with all-time best from Gen 6
    global_best_sol = x0.copy()
    
    # 14:15 deadline safeguard
    deadline = datetime.datetime.now().replace(hour=14, minute=15, second=0, microsecond=0)

    while not es.stop():
        # Check deadline before starting a generation: need at least 70 mins
        now = datetime.datetime.now()
        time_left = (deadline - now).total_seconds()
        if iteration >= 9 and time_left < 70 * 60:
            print(f"\n[DEADLINE SAFEGUARD] Current time: {now.strftime('%H:%M:%S')}. Time until 14:15 is {time_left/60:.1f} mins (< 70 mins).")
            print(f"[DEADLINE SAFEGUARD] Completed 9-generation milestone cleanly! Stopping before deadline.")
            break

        iteration += 1
        gen_start_time = datetime.datetime.now()
        print(f"\n{'='*35} STARTING GENERATION {iteration} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}")
        print(f"Time remaining until 14:15: {(deadline - gen_start_time).total_seconds()/60:.1f} minutes")
        
        solutions = es.ask()
        scores = np.zeros(len(solutions))
        
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
        
        if gen_best_score < global_best_score:
            global_best_score = gen_best_score
            global_best_sol = gen_best_sol
            print(f"[NEW ALL-TIME BEST SCORE: {global_best_score:.2f}] Saving to logs/cmaes_p15_best.txt...")
            with open("logs/cmaes_p15_best.txt", "w") as f:
                f.write(f"Phase 1.5 Generation {iteration} Best Parameters:\n")
                f.write(f"{'-'*50}\n")
                f.write(f"Total Score: {global_best_score:.2f}\n")
                f.write(f"K_ton={gen_best_sol[0]:.4f}, p_ton={gen_best_sol[1]:.4f}\n")
                f.write(f"K_sc={gen_best_sol[2]:.4f}, z_sc={gen_best_sol[3]:.4f}, p_sc={gen_best_sol[4]:.4f}\n")
                f.write(f"K_phas={gen_best_sol[5]:.4f}, p_phas={gen_best_sol[6]:.4f}\n")
                f.write(f"kp={gen_best_sol[7]:.4f}, z_ccr={gen_best_sol[8]:.4f}, p_ccr={gen_best_sol[9]:.4f}\n")
                f.write(f"k_dyn={gen_best_sol[10]:.4f}, k_stat={gen_best_sol[11]:.4f}\n")

    print("\n" + "="*90)
    print(f"OPTIMIZATION FINISHED AT {datetime.datetime.now().strftime('%H:%M:%S')}")
    print(f"Total Generations Executed: {iteration} of {iteration}")
    print(f"Final Champion Score: {global_best_score:.2f}")
    print("="*90)

if __name__ == "__main__":
    main()
