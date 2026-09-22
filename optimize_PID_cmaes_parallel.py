import cma
import concurrent.futures
import subprocess
import sys

def objective_wrapper(x):
    kp, ki, kd = x
    
    if kp < 2.0 or ki < 0.0 or kd < 0.0:
        return 100000.0
        
    try:
        # Spawn a completely isolated process to run the OpenSim simulation
        # This guarantees that if the C++ integrator explodes on wild parameters,
        # it won't take down the multiprocessing pool.
        result = subprocess.run(
            [sys.executable, "cmaes_worker.py", str(kp), str(ki), str(kd)],
            capture_output=True,
            text=True,
            timeout=600 # 10 minutes max per evaluation
        )
        
        if result.returncode == 0 and result.stdout.strip():
            score = float(result.stdout.strip().split('\n')[-1])
            print(f"Eval: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} => Bode Error: {score:.1f}")
            return score
        else:
            print(f"Eval CRASHED: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} => Bad Parameters")
            return 100000.0 + np.random.rand() * 100.0 # Add noise to prevent flatline
    except subprocess.TimeoutExpired:
        print(f"Eval TIMEOUT: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f}")
        return 100000.0 + np.random.rand() * 100.0
    except Exception as e:
        print(f"Eval ERROR: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} => {e}")
        return 100000.0 + np.random.rand() * 100.0

if __name__ == "__main__":
    print("Starting BULLETPROOF Parallel Bode Plot CMA-ES Optimization...")
    x0 = [50.0, 40.0, 5.0]
    sigma0 = 10.0 
    
    es = cma.CMAEvolutionStrategy(x0, sigma0, {'bounds': [[2.0, 0.0, 0.0], [500.0, 500.0, 50.0]], 'popsize': 20})
    
    # We use ThreadPoolExecutor because subprocess.run releases the GIL anyway.
    # Set to 20 workers to completely saturate all P-cores, E-cores, and Hyperthreads.
    with concurrent.futures.ThreadPoolExecutor(max_workers=20) as executor:
        iteration = 0
        while not es.stop() and iteration < 500:
            iteration += 1
            print(f"\n--- Generation {iteration} ---")
            solutions = es.ask()
            
            # Map solutions to isolated background subprocesses
            fitnesses = list(executor.map(objective_wrapper, solutions))
            
            es.tell(solutions, fitnesses)
            es.disp()
            
        res = es.result
        print("\n===============================")
        print("Optimization Finished!")
        print(f"Best Kp: {res.xbest[0]:.2f}")
        print(f"Best Ki: {res.xbest[1]:.2f}")
        print(f"Best Kd: {res.xbest[2]:.2f}")
        print(f"Best Bode SSE: {res.fbest:.1f}")
