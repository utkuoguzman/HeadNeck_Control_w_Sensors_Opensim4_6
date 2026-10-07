import cma
import concurrent.futures
import subprocess
import sys
import numpy as np
import os
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
        self.file.flush()
        self.stdout.write(data)
        self.stdout.flush()
    def flush(self):
        self.file.flush()
        self.stdout.flush()

timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
sys.stdout = Tee(f"logs/cmaes_opt_6_independent_{timestamp}.log", "a")

# ============================================================
# 6 INDEPENDENT CMA-ES OPTIMIZERS (Meta-Optimizer)
# Optimizing 6 separate 'Golden Sets' simultaneously.
# Popsize = 9 per frequency. 54 Tasks total per generation.
# Interleaved queue guarantees exactly 3 cores per frequency!
# ============================================================

PARAM_NAMES = ["G_ton", "G_sc", "G_phas", "kp_prop", "k_gamma_dyn", "k_gamma_stat"]

FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 6.54

TARGETS = [
    (0.31, 87.52, 4.52),
    (0.78, 239.64, -98.89),
    (1.11, 329.00, -112.63),
    (2.48, 417.12, -163.72),
    (5.52, 504.98, -194.58),
    (7.95, 600.59, -247.68)
]

def evaluate_single_simulation(args):
    x, gen_idx, freq_idx, w_idx, freq, t_gain, t_phase = args
    g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat = x

    if g_ton < 0.0 or g_sc < 0.0 or g_phas < 0.0 or kp_prop < 0.0:
        return freq_idx, w_idx, 100000.0

    target_sim_time = 0.5 + 2.0 / freq
    dynamic_timeout = int(target_sim_time * 240 * 1.5) # 240x real-time + 50% buffer

    try:
        result = subprocess.run(
            [
                sys.executable, "cmaes_worker.py",
                str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
                str(g_ton), str(g_sc), str(g_phas),
                str(kp_prop), str(k_gamma_dyn), str(k_gamma_stat),
                str(freq), str(t_gain), str(t_phase),
                f"F{freq_idx+1}-W{w_idx+1}"
            ],
            stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=dynamic_timeout
        )

        if result.returncode == 0 and result.stdout.strip():
            score = float(result.stdout.strip().split('\n')[-1])
            if score >= 90000.0:
                print(f"🛑 POOR FIT [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz (Score: {score:.1f})")
            else:
                print(f"✅ SUCCESS [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz (Score: {score:.1f})")
            return freq_idx, w_idx, score
        else:
            print(f"💥 CRASHED [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz: Gton={g_ton:.2f}, Gsc={g_sc:.2f}")
            return freq_idx, w_idx, 100000.0
            
    except subprocess.TimeoutExpired:
        print(f"⏳ TIMEOUT [F{freq_idx+1}-W{w_idx+1}] at {freq:.2f}Hz")
        return freq_idx, w_idx, 100000.0

def main():
    print("=" * 90)
    print(f"CMA-ES META-OPTIMIZER: 6 Independent Frequencies (Fixed Task: w_n={FIXED_W_N}, zeta={FIXED_ZETA}, Ki={FIXED_KI})")
    print(f"Searching 6 Reflex Parameters: {PARAM_NAMES}")
    print("=" * 90)

    x0_list = [
        [91.64, 2.63, 6.41, 1.44, 37.3, 124.6],   # 0.31 Hz
        [26.39, 23.96, 2.13, 0.41, 40.1, 113.8],  # 0.78 Hz
        [11.06, 11.28, 8.34, 0.06, 36.1, 77.7],   # 1.11 Hz
        [78.80, 24.94, 5.93, 1.21, 20.3, 91.5],   # 2.48 Hz
        [44.09, 11.20, 7.58, 2.50, 71.9, 124.6],  # 5.52 Hz
        [36.79, 5.18, 7.33, 1.76, 26.7, 92.0]     # 7.95 Hz
    ]
    sigma0_list = [1.0, 0.5, 0.5, 1.0, 1.0, 0.5] # 0.5 for stable ones, 1.0 for those pushing new boundaries 

    bounds = [
        [   5.0,  0.5,   0.5,    0.01,         5.0,         50.0  ],  # LOWER (Widened G_phas and k_g_dyn)
        [ 200.0, 50.0,  10.0,    5.00,        80.0,        150.0  ]   # UPPER (Widened G_sc and kp_prop)
    ]

    opts = {
        'bounds': bounds,
        'popsize': 9,  # 9 Workers per frequency
        'CMA_active': True,
        'CMA_mirrors': 0.5,
        'CMA_stds': [10.0, 3.8, 0.75, 0.3, 4.5, 4.9]
    }

    # Instantiate 6 completely independent optimizers with specific starting points!
    optimisers = [cma.CMAEvolutionStrategy(x0_list[i], sigma0_list[i], opts) for i in range(6)]

    with concurrent.futures.ThreadPoolExecutor(max_workers=18) as executor:
        iteration = 0
        # Stop only if all 6 optimizers have converged or we hit gen 45
        while iteration < 45 and not all(es.stop() for es in optimisers):
            iteration += 1
            
            # Ask 9 solutions from each of the 6 optimizers
            solutions_list = [es.ask() if not es.stop() else [] for es in optimisers]
            
            tasks = []
            # Interleave tasks: W1 for F1, W1 for F2... W2 for F1...
            # This perfectly balances 3 cores per frequency over 18 cores!
            for w_idx in range(9):
                for freq_idx in range(6):
                    if not optimisers[freq_idx].stop():
                        sol = solutions_list[freq_idx][w_idx]
                        freq, t_gain, t_phase = TARGETS[freq_idx]
                        tasks.append((sol, iteration, freq_idx, w_idx, freq, t_gain, t_phase))
            
            if not tasks:
                break # All done

            # Submit all tasks
            futures = [executor.submit(evaluate_single_simulation, arg) for arg in tasks]
            
            # Reconstruct scores
            scores_list = [[0.0]*9 for _ in range(6)]
            for future in concurrent.futures.as_completed(futures):
                freq_idx, w_idx, score = future.result()
                scores_list[freq_idx][w_idx] = score
                
            print(f"\n{'='*40} GENERATION {iteration} SUMMARY {'='*40}")
            
            # Tell each optimizer and log the best
            with open("logs/cmaes_6_independent_best.txt", "w") as f:
                f.write(f"Generation {iteration} Best Parameters:\n")
                f.write("-" * 50 + "\n")
                
                for freq_idx in range(6):
                    if not optimisers[freq_idx].stop():
                        optimisers[freq_idx].tell(solutions_list[freq_idx], scores_list[freq_idx])
                        optimisers[freq_idx].logger.add()
                        
                        best_idx = np.argmin(scores_list[freq_idx])
                        best_sol = solutions_list[freq_idx][best_idx]
                        best_score = scores_list[freq_idx][best_idx]
                        freq = TARGETS[freq_idx][0]
                        
                        print(f"Freq {freq:.2f}Hz: SSE={best_score:.1f} | G_ton={best_sol[0]:.2f}, G_sc={best_sol[1]:.2f}, G_phas={best_sol[2]:.2f}, kp_p={best_sol[3]:.2f}, k_g_dyn={best_sol[4]:.1f}, k_g_stat={best_sol[5]:.1f}")
                        
                        f.write(f"Freq {freq:.2f}Hz | SSE: {best_score:.2f}\n")
                        f.write(f"G_ton={best_sol[0]:.4f}, G_sc={best_sol[1]:.4f}, G_phas={best_sol[2]:.4f}, kp_prop={best_sol[3]:.4f}, k_gamma_dyn={best_sol[4]:.4f}, k_gamma_stat={best_sol[5]:.4f}\n\n")

if __name__ == "__main__":
    main()
