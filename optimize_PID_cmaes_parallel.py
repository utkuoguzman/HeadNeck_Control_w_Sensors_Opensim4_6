import cma
import concurrent.futures
import subprocess
import sys
import numpy as np
import random

import os
import datetime

class Tee(object):
    def __init__(self, name, mode):
        os.makedirs(os.path.dirname(name), exist_ok=True)
        self.file = open(name, mode)
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
sys.stdout = Tee(f"logs/cmaes_optimization_{timestamp}.log", "a")


def objective_wrapper(args):
    x, gen_idx, w_idx = args
    kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat = x
    
    if kp < 2.0 or ki < 0.0 or kd < 0.0 or g_ton < 0.0 or g_sc < 0.0 or g_phas < 0.0:
        return 100000.0
        
    targets = [
        # (freq, gain, phase, timeout_seconds)
        # We process highest frequency first because it is fastest!
        (1.124195, 149.430277, -81.089109, 250),   # Tiny amplitude (1.2cm). 
        (0.823652, 106.268348, -85.537188, 350),   # Small amplitude (2cm). 
        (0.603456, 81.029187, -86.129080, 500),    # Medium amplitude (4cm). 
        (0.442127, 63.821646, -82.694072, 800),    # Large amplitude (7.6cm). 
        (0.323929, 50.868070, -77.524752, 2000)    # Massive amplitude (14cm).
    ]
    
    total_sse = 0.0
    for freq_idx, (freq, target_gain, target_phase, max_timeout) in enumerate(targets):
        try:
            result = subprocess.run(
                [sys.executable, "cmaes_worker.py", str(kp), str(ki), str(kd), str(g_ton), str(g_sc), str(g_phas), str(kp_prop), str(k_gamma_dyn), str(k_gamma_stat), str(freq), str(target_gain), str(target_phase), f"G{gen_idx:02d}-W{w_idx:02d}"],
                stdout=subprocess.PIPE,
                text=True,
                timeout=max_timeout
            )
            
            if result.returncode == 0 and result.stdout.strip():
                score = float(result.stdout.strip().split('\n')[-1])
                total_sse += score
                
                # --- CASCADED EARLY REJECTION ---
                # If high frequencies fail, skip simulating slow 0.32Hz/0.44Hz runs to save 50% CPU time!
                if (freq_idx == 0 and score > 3000.0) or (freq_idx == 1 and total_sse > 5500.0):
                    remaining_freqs = len(targets) - (freq_idx + 1)
                    penalized_score = total_sse + remaining_freqs * 3500.0
                    print(f"Eval EARLY ABORT at {freq:.2f}Hz: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f}, Gsc={g_sc:.2f}, Gphas={g_phas:.2f} => Est Score: {penalized_score:.1f}")
                    return penalized_score
            else:
                print(f"Eval CRASHED at {freq:.2f}Hz: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f}, Gton={g_ton:.2f}, Gsc={g_sc:.2f}, Gphas={g_phas:.2f}, Kp_prop={kp_prop:.3f}, K_gamma_dyn={k_gamma_dyn:.1f}, K_gamma_stat={k_gamma_stat:.1f}")
                return 100000.0 + np.random.rand() * 100.0
        except subprocess.TimeoutExpired:
            print(f"Eval TIMEOUT at {freq:.2f}Hz: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f}, Gton={g_ton:.2f}, Gsc={g_sc:.2f}, Gphas={g_phas:.2f}, Kp_prop={kp_prop:.3f}, K_gamma_dyn={k_gamma_dyn:.1f}, K_gamma_stat={k_gamma_stat:.1f}")
            return 100000.0 + np.random.rand() * 100.0
        except Exception as e:
            print(f"Eval ERROR at {freq:.2f}Hz: {e}")
            return 100000.0 + np.random.rand() * 100.0
            
    print(f"Eval SUCCESS: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f}, Gton={g_ton:.2f}, Gsc={g_sc:.2f}, Gphas={g_phas:.2f}, Kp_prop={kp_prop:.3f}, K_gamma_dyn={k_gamma_dyn:.1f}, K_gamma_stat={k_gamma_stat:.1f} => Bode Error: {total_sse:.1f}")
    return total_sse

if __name__ == "__main__":
    print("Starting ENHANCED Active+Mirrored Parallel CMA-ES Optimization (Della Santina Canals)...")
    # Warm start from the absolute best individual (Bode Error: 7873.7)
    x0 = [65.6, 36.6, 14.1, 12.15, 1.48, 4.89, 0.052, 36.0, 97.2]
    sigma0 = 1.0 
    
    # Biomechanically bounded ranges preventing unviable regions
    bounds = [
        [10.0, 5.0, 1.0, 0.5, 0.1, 0.5, 0.05, 10.0, 30.0], 
        [150.0, 100.0, 40.0, 35.0, 8.0, 10.0, 2.5, 90.0, 180.0]
    ]
    
    opts = {
        'bounds': bounds,
        'popsize': 12,
        'CMA_active': True,
        'CMA_mirrors': 0.5,
        'CMA_stds': [5.0, 2.4, 2.8, 3.8, 0.75, 0.68, 0.3, 4.5, 4.9]
    }
    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)
    
    # We use ThreadPoolExecutor because subprocess.run releases the GIL anyway.
    # 16 workers mapping perfectly to popsize=16 with SIMB_NUM_THREADS=1
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
        iteration = 0
        while not es.stop() and iteration < 50:
            iteration += 1
            print(f"\n--- Generation {iteration} ---")
            solutions = es.ask()
            
            # Map solutions to isolated background subprocesses
            tasks = [(x, iteration, i+1) for i, x in enumerate(solutions)]
            fitnesses = list(executor.map(objective_wrapper, tasks))
            
            es.tell(solutions, fitnesses)
            es.disp()
            
        res = es.result
        print("\n===============================")
        print("Optimization Finished!")
        print(f"Best Kp: {res.xbest[0]:.2f}")
        print(f"Best Ki: {res.xbest[1]:.2f}")
        print(f"Best Kd: {res.xbest[2]:.2f}")
        print(f"Best Bode SSE: {res.fbest:.1f}")
