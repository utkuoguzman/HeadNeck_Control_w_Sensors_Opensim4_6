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
sys.stdout = Tee(f"logs/cmaes_opt_bandwidth_8dim_{timestamp}.log", "a")

# ============================================================
# ROUND 4: Bandwidth-Scheduled (omega_n) Optimization
# Optimizing 8 parameters. Kp and Kd are dynamically linked 
# by omega_n to guarantee zeta = 1.0 (no vibrations!).
# ============================================================

PARAM_NAMES = ["omega_n", "zeta", "Ki", "G_ton", "G_sc", "G_phas", "kp_prop", "k_gamma_dyn", "k_gamma_stat"]

def objective_wrapper(args):
    x, gen_idx, w_idx = args
    omega_n, zeta, ki, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat = x

    # Non-negativity check
    if omega_n < 0.0 or ki < 0.0 or g_ton < 0.0 or g_sc < 0.0 or g_phas < 0.0 or kp_prop < 0.0:
        return 100000.0

    targets = [
        (1.12, 1.30, 29),
        (2.12, 0.90, 75),
        (3.12, 0.95, 126),
        (4.12, 0.96, 175),
        (8.12, 1.20, -78),  
        (12.12, 0.85, 33)   
    ]

    total_sse = 0.0
    for freq_idx, (freq, t_gain, t_phase) in enumerate(targets):
        try:
            result = subprocess.run(
                [
                    sys.executable, "cmaes_worker.py",
                    str(omega_n), str(zeta), str(ki),
                    str(g_ton), str(g_sc), str(g_phas),
                    str(kp_prop), str(k_gamma_dyn), str(k_gamma_stat),
                    str(freq), str(t_gain), str(t_phase),
                    f"G{gen_idx:02d}-W{w_idx:02d}"
                ],
                stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=300
            )

            if result.returncode == 0 and result.stdout.strip():
                score = float(result.stdout.strip().split('\n')[-1])
                total_sse += score

                # Cascaded early rejection
                if score >= 90000.0: # ONLY abort on physical crash (100000.0) or timeout
                    remaining_freqs = len(targets) - (freq_idx + 1)
                    penalized_score = total_sse + remaining_freqs * 3500.0
                    print(f"Eval EARLY ABORT at {freq:.2f}Hz: w_n={omega_n:.1f}, Gsc={g_sc:.2f} => Est Score: {penalized_score:.1f}")
                    return penalized_score
            else:
                print(f"Eval CRASHED at {freq:.2f}Hz: w_n={omega_n:.1f}, Gton={g_ton:.2f}")
                return 100000.0
        except subprocess.TimeoutExpired:
            print(f"Eval TIMEOUT at {freq:.2f}Hz")
            return 100000.0

    return total_sse

def main():
    print("=" * 90)
    print("CMA-ES: Bandwidth-Scheduled Bode Optimization (9 Dimensions)")
    print(f"Searching: {PARAM_NAMES}")
    print("=" * 90)

    # Initial Guess (x0)
    # omega_n = 9.0 gives Kp ~ 81.0, Kd ~ 18.0 (critically damped)
    x0 = [9.0, 0.425, 15.0, 11.2, 4.3, 2.6, 0.2, 58.7, 94.8]
    sigma0 = 2.0 

    bounds = [
        # w_n,   Ki,  G_ton, G_sc, G_phas, kp_prop, k_gamma_dyn, k_gamma_stat
        [ 4.0,  0.3, 10.0,   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ],  # LOWER
        [20.0, 0.8, 200.0,  25.0,  5.0,  10.0,    2.50,        80.0,        150.0  ]   # UPPER
    ]

    opts = {
        'bounds': bounds,
        'popsize': 15, 
        'CMA_active': True,
        'CMA_mirrors': 0.5,
        'CMA_stds': [2.0, 0.2, 10.0, 3.8, 0.75, 0.68, 0.3, 4.5, 4.9]
    }

    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)

    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
        iteration = 0
        while not es.stop() and iteration < 45:
            iteration += 1
            solutions = es.ask()
            
            args_list = [(sol, iteration, i+1) for i, sol in enumerate(solutions)]
            scores = list(executor.map(objective_wrapper, args_list))
            
            es.tell(solutions, scores)
            es.logger.add()
            es.disp()
            
            best_idx = np.argmin(scores)
            best_sol = solutions[best_idx]
            
            w_n = best_sol[0]
            kp = w_n**2
            kd = 2.0 * best_sol[1] * w_n
            print(f"Gen {iteration}: Best Bode SSE = {scores[best_idx]:.1f} | w_n={w_n:.2f}, zeta={best_sol[1]:.3f} (Kp={kp:.1f},Kd={kd:.1f}), Ki={best_sol[2]:.1f}, G_ton={best_sol[3]:.3f}, G_sc={best_sol[4]:.3f}, G_phas={best_sol[5]:.3f}\n")
            
            with open("logs/cmaes_bandwidth_bode_best.txt", "w") as f:
                f.write(f"omega_n={best_sol[0]:.4f}, zeta={best_sol[1]:.4f}, Ki={best_sol[2]:.4f}\n")
                f.write(f"G_ton={best_sol[3]:.4f}, G_sc={best_sol[4]:.4f}, G_phas={best_sol[5]:.4f}\n")
                f.write(f"kp_prop={best_sol[6]:.4f}, k_gamma_dyn={best_sol[7]:.4f}, k_gamma_stat={best_sol[8]:.4f}\n")
                f.write(f"Bode_SSE={scores[best_idx]:.2f}\n")

if __name__ == "__main__":
    main()
