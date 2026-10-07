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
sys.stdout = Tee(f"logs/cmaes_opt_18core_6dim_flat_{timestamp}.log", "a")

# ============================================================
# FLATTENED 54-TASK ARCHITECTURE
# 9 Population Size x 6 Frequencies = 54 Independent Tasks.
# 18 Cores process them in exactly 3 waves.
# Task order guarantees exactly 3 cores run each frequency!
# ============================================================

PARAM_NAMES = ["G_ton", "G_sc", "G_phas", "kp_prop", "k_gamma_dyn", "k_gamma_stat"]

FIXED_W_N = 10.1
FIXED_ZETA = 1.98
FIXED_KI = 6.54

def evaluate_single_freq(args):
    x, gen_idx, ind_idx, freq, t_gain, t_phase, abort_flags = args
    
    # If a previous frequency for this individual crashed, instantly abort to save CPU!
    if abort_flags[ind_idx]:
        return ind_idx, 100000.0

    g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat = x

    if g_ton < 0.0 or g_sc < 0.0 or g_phas < 0.0 or kp_prop < 0.0:
        abort_flags[ind_idx] = True
        return ind_idx, 100000.0

    try:
        result = subprocess.run(
            [
                sys.executable, "cmaes_worker.py",
                str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
                str(g_ton), str(g_sc), str(g_phas),
                str(kp_prop), str(k_gamma_dyn), str(k_gamma_stat),
                str(freq), str(t_gain), str(t_phase),
                f"G{gen_idx:02d}-W{ind_idx:02d}"
            ],
            stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=600
        )

        if result.returncode == 0 and result.stdout.strip():
            score = float(result.stdout.strip().split('\n')[-1])
            
            if score >= 90000.0: 
                abort_flags[ind_idx] = True
                print(f"🛑 EARLY ABORT [G{gen_idx:02d}-W{ind_idx:02d}] at {freq:.2f}Hz: Gsc={g_sc:.2f}, kp_prop={kp_prop:.2f}")
                return ind_idx, 100000.0
                
            return ind_idx, score
        else:
            abort_flags[ind_idx] = True
            print(f"💥 CRASHED [G{gen_idx:02d}-W{ind_idx:02d}] at {freq:.2f}Hz: Gton={g_ton:.2f}, Gsc={g_sc:.2f}")
            return ind_idx, 100000.0
            
    except subprocess.TimeoutExpired:
        abort_flags[ind_idx] = True
        print(f"⏳ TIMEOUT [G{gen_idx:02d}-W{ind_idx:02d}] at {freq:.2f}Hz")
        return ind_idx, 100000.0

def main():
    print("=" * 90)
    print(f"CMA-ES: 18-Core FLATTENED Optimization (Fixed Task: w_n={FIXED_W_N}, zeta={FIXED_ZETA}, Ki={FIXED_KI})")
    print(f"Searching 6 Reflex Parameters: {PARAM_NAMES}")
    print("=" * 90)

    x0 = [16.84, 3.13, 6.22, 0.56, 38.11, 87.47]
    sigma0 = 2.0 

    bounds = [
        [   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ],  # LOWER
        [ 200.0, 25.0,  10.0,    2.50,        80.0,        150.0  ]   # UPPER
    ]

    opts = {
        'bounds': bounds,
        'popsize': 9,  # EXACTLY 9 INDIVIDUALS
        'CMA_active': True,
        'CMA_mirrors': 0.5,
        'CMA_stds': [10.0, 3.8, 0.75, 0.3, 4.5, 4.9]
    }

    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

    targets = [
        (0.31, 87.52, 4.52),
        (0.78, 239.64, -98.89),
        (1.11, 329.00, -112.63),
        (2.48, 417.12, -163.72),
        (5.52, 504.98, -194.58),
        (7.95, 600.59, -247.68)
    ]

    with concurrent.futures.ThreadPoolExecutor(max_workers=18) as executor:
        iteration = 0
        while not es.stop() and iteration < 45:
            iteration += 1
            solutions = es.ask()
            
            # Shared memory array for thread-safe early aborts
            abort_flags = [False] * 9
            scores = [0.0] * 9
            
            tasks = []
            # We iterate ind_idx outer, freq_idx inner.
            # This generates: [Ind0-Freq0, Ind0-Freq1... Ind1-Freq0...]
            # Since the executor pulls 18 at a time, the first 18 are exactly Ind0, Ind1, Ind2 (all 6 freqs each)
            # This PERFECTLY guarantees 3 cores per frequency continuously!
            for ind_idx in range(9):
                for freq, t_gain, t_phase in targets:
                    tasks.append( (solutions[ind_idx], iteration, ind_idx, freq, t_gain, t_phase, abort_flags) )
            
            # Submit all 54 tasks
            futures = [executor.submit(evaluate_single_freq, arg) for arg in tasks]
            
            # Wait and accumulate scores
            for future in concurrent.futures.as_completed(futures):
                ind_idx, score = future.result()
                scores[ind_idx] += score
                
            es.tell(solutions, scores)
            es.logger.add()
            es.disp()
            
            best_idx = np.argmin(scores)
            best_sol = solutions[best_idx]
            
            print(f"Gen {iteration}: Best Bode SSE = {scores[best_idx]:.1f} | G_ton={best_sol[0]:.3f}, G_sc={best_sol[1]:.3f}, G_phas={best_sol[2]:.3f}, kp_prop={best_sol[3]:.3f}, k_gamma_dyn={best_sol[4]:.3f}, k_gamma_stat={best_sol[5]:.3f}\n")
            
            with open("logs/cmaes_18core_flat_best.txt", "w") as f:
                f.write(f"Fixed Task: omega_n={FIXED_W_N}, zeta={FIXED_ZETA}, Ki={FIXED_KI}\n")
                f.write(f"G_ton={best_sol[0]:.4f}, G_sc={best_sol[1]:.4f}, G_phas={best_sol[2]:.4f}\n")
                f.write(f"kp_prop={best_sol[3]:.4f}, k_gamma_dyn={best_sol[4]:.4f}, k_gamma_stat={best_sol[5]:.4f}\n")
                f.write(f"Bode_SSE={scores[best_idx]:.2f}\n")

if __name__ == "__main__":
    main()
