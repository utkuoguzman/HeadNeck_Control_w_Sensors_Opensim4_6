import time
import numpy as np
from scipy.stats import qmc
import multiprocessing as mp
import sys
import os

sys.path.append("scratch")
from cervical_chadwick_blana_simulator import fast_eval_candidate

def main():
    lower_bounds = np.array([
        0.05, 2.0, 0.05, 0.001, 15.0, 0.20,
        8.0, 0.15, 0.001, 5.0, 0.10, 0.01
    ])
    upper_bounds = np.array([
        2.50, 120.0, 15.0, 0.60, 150.0, 15.0,
        80.0, 5.00, 0.65, 220.0, 1.50, 1.50
    ])

    sampler = qmc.Sobol(d=12, scramble=True, seed=2026)
    n_samples = 1000
    samples = qmc.scale(sampler.random(n_samples), lower_bounds, upper_bounds)

    t0 = time.perf_counter()
    with mp.Pool(14) as pool:
        scores = pool.map(fast_eval_candidate, [s.tolist() for s in samples], chunksize=25)
    t1 = time.perf_counter()

    elapsed = t1 - t0
    rate = n_samples / elapsed
    est_64k = 65536 / rate
    print(f"Evaluated {n_samples} samples in {elapsed:.2f} s ({rate:.1f} evals/s across 14 cores).")
    print(f"Estimated time for 65,536 samples: {est_64k:.1f} s ({est_64k/60.0:.2f} min).")
    best_idx = np.argmin(scores)
    print(f"Best score in batch: {scores[best_idx]:.2f} pts")

if __name__ == '__main__':
    main()
