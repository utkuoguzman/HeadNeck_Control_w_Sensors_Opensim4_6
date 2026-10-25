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
sys.stdout = Tee(f"logs/cmaes_step_round3_{timestamp}.log", "a")

# ============================================================
# ROUND 3: Re-optimize PID with tuned reflexes from Round 2
# ============================================================
# Frozen reflex parameters (Round 2c: Gen 8 Biologically Excellent):
FROZEN_G_TON = 8.76
FROZEN_G_SC = 4.33
FROZEN_G_PHAS = 6.06
FROZEN_KP_PROP = 0.441
FROZEN_K_GAMMA_DYN = 52.36
FROZEN_K_GAMMA_STAT = 92.37


def objective_wrapper(args):
    x, gen_idx, w_idx = args
    Kp, Ki, Kd = x

    if Kp < 0 or Ki < 0 or Kd < 0:
        return 1e6 + np.sum(np.abs(x))

    worker_id = f"G{gen_idx:02d}-W{w_idx:02d}"
    cmd = [
        sys.executable, "cmaes_step_worker.py",
        str(Kp), str(Ki), str(Kd),
        str(FROZEN_G_TON), str(FROZEN_G_SC), str(FROZEN_G_PHAS),
        str(FROZEN_KP_PROP), str(FROZEN_K_GAMMA_DYN), str(FROZEN_K_GAMMA_STAT),
        worker_id,
    ]

    try:
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
            return score

        print(f"Eval SUCCESS: {worker_id} | Kp={Kp:.2f}, Ki={Ki:.2f}, Kd={Kd:.2f} => Score={score:.1f}")
        return score

    except Exception as e:
        print(f"Eval ERROR {worker_id}: {e}")
        return 1e6


if __name__ == "__main__":
    print("=" * 65)
    print("ROUND 3: Step-Input PID Re-Optimization (with tuned reflexes)")
    print(f"Frozen Reflexes: G_ton={FROZEN_G_TON}, G_sc={FROZEN_G_SC}, "
          f"G_phas={FROZEN_G_PHAS}")
    print(f"                 kp_prop={FROZEN_KP_PROP}, "
          f"k_gamma_dyn={FROZEN_K_GAMMA_DYN}, k_gamma_stat={FROZEN_K_GAMMA_STAT}")
    print("Searching: Kp, Ki, Kd")
    print("=" * 65)

    # Start softer than Round 1 since reflexes now provide damping
    x0 = [696.79, 179.12, 120.92]
    sigma0 = 50.0
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
            print(f"Gen {generation}: Best Error = {best_f:.2f} | "
                  f"Kp={best_x[0]:.2f}, Ki={best_x[1]:.2f}, Kd={best_x[2]:.2f}")
            with open('logs/cmaes_step_round3_best.txt', 'w') as f:
                f.write(f"Kp={best_x[0]}, Ki={best_x[1]}, Kd={best_x[2]}\n")
                f.write(f"G_ton={FROZEN_G_TON}, G_sc={FROZEN_G_SC}, "
                        f"G_phas={FROZEN_G_PHAS}\n")
                f.write(f"kp_prop={FROZEN_KP_PROP}, k_gamma_dyn={FROZEN_K_GAMMA_DYN}, "
                        f"k_gamma_stat={FROZEN_K_GAMMA_STAT}\n")
                f.write(f"Step Error={best_f}\n")

    res = es.result
    print("\n" + "=" * 65)
    print("Round 3 Optimization Finished!")
    print(f"Best PID: Kp={res.xbest[0]:.2f}, Ki={res.xbest[1]:.2f}, Kd={res.xbest[2]:.2f}")
    print(f"Best Step Error: {res.fbest:.2f}")
    print("=" * 65)

