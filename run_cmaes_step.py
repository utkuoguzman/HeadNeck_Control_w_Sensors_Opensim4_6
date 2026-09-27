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
sys.stdout = Tee(f"logs/cmaes_step_optimization_{timestamp}.log", "a")


def objective_wrapper(args):
    x, gen_idx, w_idx = args
    Kp, Ki, Kd = x

    # Simple bounds check
    if Kp < 0 or Ki < 0 or Kd < 0:
        return 1e6 + np.sum(np.abs(x))

    worker_id = f"G{gen_idx:02d}-W{w_idx:02d}"
    cmd = [sys.executable, "cmaes_step_worker.py", str(Kp), str(Ki), str(Kd), worker_id]

    try:
        # Run the worker; stream its stdout line-by-line in real time
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        score = None
        for line in proc.stdout:
            line = line.rstrip()
            if line.startswith("SCORE:"):
                score = float(line.split(":", 1)[1])
            else:
                print(line)

        proc.wait()

        if proc.returncode != 0 or score is None:
            print(f"Eval CRASHED: {worker_id} | Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f}")
            return 1e6 + np.sum(np.abs(x)) * 1000.0

        if score >= 1e6:
            print(f"Eval CRASHED: {worker_id} | Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f} => Integrator failure")
            return score + np.sum(np.abs(x)) * 1000.0

        print(f"Eval SUCCESS: {worker_id} | Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f} => Score={score:.1f}")
        return score

    except Exception as e:
        print(f"Eval ERROR {worker_id}: {e}")
        return 1e6 + np.sum(np.abs(x)) * 1000.0


if __name__ == "__main__":
    print("Starting Parallel CMA-ES for Step-Input PID optimisation (Kp, Ki, Kd)...")
    # Soft starting point that should not crash the integrator
    x0 = [15.0, 1.0, 1.0]
    sigma0 = 5.0
    es = cma.CMAEvolutionStrategy(x0, sigma0, {"bounds": [0, None], "popsize": 6})

    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
        generation = 0
        while not es.stop() and generation < 15:
            generation += 1
            print(f"\n--- Generation {generation} ---")
            solutions = es.ask()
            tasks = [(x, generation, i + 1) for i, x in enumerate(solutions)]
            fitnesses = list(executor.map(objective_wrapper, tasks))
            es.tell(solutions, fitnesses)
            es.disp()
            best_x = es.result.xbest
            best_f = es.result.fbest
            print(f"Gen {generation}: Best Error = {best_f:.2f} | Kp={best_x[0]:.2f}, Ki={best_x[1]:.2f}, Kd={best_x[2]:.2f}")
            # Checkpoint
            with open('logs/cmaes_step_best.txt', 'w') as f:
                f.write(f"Kp={best_x[0]}, Ki={best_x[1]}, Kd={best_x[2]}, Error={best_f}")

    res = es.result
    print("\n===============================")
    print("Optimization Finished!")
    print(f"Best Kp: {res.xbest[0]:.2f}")
    print(f"Best Ki: {res.xbest[1]:.2f}")
    print(f"Best Kd: {res.xbest[2]:.2f}")
    print(f"Best Step Error: {res.fbest:.1f}")
