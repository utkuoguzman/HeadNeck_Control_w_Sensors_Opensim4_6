import cma
import concurrent.futures
import subprocess
import sys
import numpy as np
import os

def evaluate_candidate(cand, wname):
    # cand has 6 elements: wn_p, z_p, wn_r, z_r, wn_y, z_y
    # Ensure they are within reasonable physical bounds (positive)
    if any(x < 0.1 for x in cand) or any(cand[i] > 20.0 for i in [0,2,4]) or any(cand[i] > 5.0 for i in [1,3,5]):
        return 1e6
        
    cmd = [
        "C:\\Users\\User\\miniforge3\\envs\\opensim_env\\python.exe",
        "worker_fast_minjerk.py",
        wname,
        str(cand[0]), str(cand[1]), str(cand[2]),
        str(cand[3]), str(cand[4]), str(cand[5])
    ]
    
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        out = res.stdout.strip().split('\n')[-1]
        score = float(out)
    except Exception as e:
        score = 1e6
    return score

if __name__ == '__main__':
    print("="*80)
    print("OPTIMIZING LQR COGNITIVE WEIGHTS (Wp, Wv for P/R/Y)")
    print("="*80)

    # Initial guess: 
    # If wn = 4.0, Wp = 4^4 = 256. 
    # If zeta = 1.0, Wv = (1.0^2 - 0.5) * 4 * sqrt(256) = 0.5 * 4 * 16 = 32
    x0 = np.array([256.0, 32.0, 256.0, 32.0, 256.0, 32.0])
    sigma0 = 50.0
    
    es = cma.CMAEvolutionStrategy(x0, sigma0, {
        'popsize': 12,
        'bounds': [0.1, [160000.0, 160000.0, 160000.0, 160000.0, 160000.0, 160000.0]], # Allow very high weights
        'tolfun': 1e-4,
    })
    
    generation = 1
    with concurrent.futures.ProcessPoolExecutor(max_workers=6) as executor:
        while not es.stop():
            candidates = es.ask()
            
            futures = []
            for i, cand in enumerate(candidates):
                wname = f"W{i}"
                futures.append(executor.submit(evaluate_candidate, cand, wname))
                
            scores = [f.result() for f in futures]
            
            es.tell(candidates, scores)
            es.disp()
            
            best_idx = np.argmin(scores)
            best_score = scores[best_idx]
            best_cand = candidates[best_idx]
            print(f"Gen {generation} Best Score: {best_score:.6f}")
            print(f"Gen {generation} Best Params: Wp_p={best_cand[0]:.2f}, Wv_p={best_cand[1]:.2f}, Wp_r={best_cand[2]:.2f}, Wv_r={best_cand[3]:.2f}, Wp_y={best_cand[4]:.2f}, Wv_y={best_cand[5]:.2f}")
            
            with open("logs/fast_minjerk_best.txt", "w") as f:
                f.write(f"Score: {best_score}\nParams: {list(best_cand)}\n")
                
            generation += 1
            if generation > 50:
                break
                
    print("Optimization finished.")

