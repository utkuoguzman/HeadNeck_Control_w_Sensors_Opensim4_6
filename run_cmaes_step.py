import cma
import subprocess
import numpy as np

def objective_function(x):
    Kp, Ki, Kd = x
    
    # Penalize negative gains
    if Kp < 0 or Ki < 0 or Kd < 0:
        return 1e6 + np.sum(np.abs(x))
        
    cmd = ["python", "cmaes_step_worker.py", str(Kp), str(Ki), str(Kd)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    try:
        err = float(result.stdout.strip())
        if err == 1e6:
            # If it crashed, add a penalty based on the magnitude of the gains so the optimizer knows to reduce them!
            return 1e6 + np.sum(np.abs(x)) * 1000.0
        return err
    except:
        return 1e6 + np.sum(np.abs(x)) * 1000.0

# Start with a VERY soft controller that won't crash the QP solver!
x0 = [15.0, 1.0, 1.0]
sigma0 = 5.0

print("Starting CMA-ES for Step Inputs (Kp, Ki, Kd)")
es = cma.CMAEvolutionStrategy(x0, sigma0, {'bounds': [0, None], 'popsize': 6})

generation = 0
while not es.stop() and generation < 15:
    solutions = es.ask()
    fitnesses = [objective_function(x) for x in solutions]
    es.tell(solutions, fitnesses)
    es.disp()
    
    best_x = es.result.xbest
    best_f = es.result.fbest
    print(f"Gen {generation}: Best Error = {best_f:.2f} | Kp={best_x[0]:.2f}, Ki={best_x[1]:.2f}, Kd={best_x[2]:.2f}")
    
    # Save checkpoint
    with open('logs/cmaes_step_best.txt', 'w') as f:
        f.write(f"Kp={best_x[0]}, Ki={best_x[1]}, Kd={best_x[2]}")
        
    generation += 1
