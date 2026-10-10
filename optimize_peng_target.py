import cma
import numpy as np
import subprocess
import os

# Decision variables:
# 1. Wp_base (LQR position cost base)
# 2. Wv_base (LQR velocity cost base)
# 3. k_vigor (Continuous vigor multiplier)
# 4. tau_traj (Brain's internal 2nd-order critically damped trajectory planner time constant)

def run_worker(params, worker_id):
    worker_name = f"W-{worker_id}"
    cmd = [
        "python", "worker_peng_target.py",
        worker_name,
        str(params[0]),
        str(params[1]),
        str(params[2]),
        str(params[3])
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    try:
        score = float(result.stdout.strip())
        return score
    except:
        return 1e6

if __name__ == '__main__':
    # Initial guess
    x0 = [400.0, 40.0, 0.5, 0.15]
    # Standard deviations
    sigma0 = 0.2
    
    opts = cma.CMAOptions()
    opts['bounds'] = [[1.0, 1.0, 0.0, 0.05], [5000.0, 500.0, 10.0, 1.0]]
    opts['popsize'] = 12
    opts['maxiter'] = 30
    
    es = cma.CMAEvolutionStrategy(x0, sigma0, opts)
    
    iteration = 0
    while not es.stop():
        solutions = es.ask()
        
        # In a real run, you can use multiprocessing here
        fitness = [run_worker(x, i) for i, x in enumerate(solutions)]
        
        es.tell(solutions, fitness)
        es.disp()
        iteration += 1
        
    print("Optimization Complete.")
    print("Best solution found:")
    best_x = es.result.xbest
    print(f"Wp_base = {best_x[0]}")
    print(f"Wv_base = {best_x[1]}")
    print(f"k_vigor = {best_x[2]}")
    print(f"tau_traj = {best_x[3]}")
    print(f"Best fitness = {es.result.fbest}")
