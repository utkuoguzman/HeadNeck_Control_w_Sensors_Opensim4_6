import time
import math
import cmath
import json
import numpy as np
from scipy.stats import qmc
from test_integrated_prescreen import surrogate_score, OPT_PLANT, TARGETS

# Bounds for the 12 parameters
lower_bounds = np.array([
    0.05,   # 0: r_ton
    2.0,    # 1: p_ton
    0.20,   # 2: K_sc
    0.001,  # 3: r_lead_sc
    15.0,   # 4: p_sc
    0.20,   # 5: K_phas
    15.0,   # 6: p_phas
    0.15,   # 7: kp
    0.001,  # 8: r_lead_ccr
    5.0,    # 9: p_ccr
    0.10,   # 10: k_dyn
    0.01    # 11: k_stat
])

upper_bounds = np.array([
    2.50,   # 0: r_ton
    80.0,   # 1: p_ton
    30.0,   # 2: K_sc
    0.60,   # 3: r_lead_sc
    100.0,  # 4: p_sc
    20.0,   # 5: K_phas
    90.0,   # 6: p_phas
    5.00,   # 7: kp
    0.65,   # 8: r_lead_ccr
    90.0,   # 9: p_ccr
    1.50,   # 10: k_dyn
    1.50    # 11: k_stat
])

n_samples = 65536 # 2^16 Sobol sequence

print(f"Generating {n_samples:,} Sobol quasi-random samples in 12 dimensions...")
t0 = time.perf_counter()
sampler = qmc.Sobol(d=12, scramble=True, seed=42)
unit_samples = sampler.random(n_samples)
sobol_samples = qmc.scale(unit_samples, lower_bounds, upper_bounds)
t_gen = time.perf_counter() - t0
print(f"Generated {n_samples:,} Sobol samples in {t_gen*1000:.1f} ms!")

print(f"Evaluating all {n_samples:,} candidates with Section 3 Surrogate...")
t1 = time.perf_counter()
scores = [surrogate_score(c) for c in sobol_samples]
t_eval = time.perf_counter() - t1
evals_per_sec = n_samples / t_eval

ranked_indices = np.argsort(scores)

print("\n" + "="*85)
print(f"Global Sobol Sweep Completed in {t_eval:.2f} seconds ({evals_per_sec:,.0f} evals/sec)!")
print(f"Best Global Score Found:    {scores[ranked_indices[0]]:.2f} pts")
print(f"Median Score across Space:  {np.median(scores):.2f} pts")
print(f"Worst Score in Space:       {scores[ranked_indices[-1]]:.2f} pts")
print("="*85)

print("\n--- Top 5 Global Candidates from 65,536 Sobol Sweep ---")
param_names = ["r_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas", "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat"]
for rank in range(5):
    idx = ranked_indices[rank]
    cand = sobol_samples[idx]
    sc = scores[idx]
    print(f"\nRank #{rank+1} (Score: {sc:.2f} pts):")
    for name, val in zip(param_names, cand):
        print(f"  {name:<12}: {val:.4f}")

