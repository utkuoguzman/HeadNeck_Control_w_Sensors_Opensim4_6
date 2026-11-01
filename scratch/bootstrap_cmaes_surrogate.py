import json
import math
import cmath
import os
import time
import numpy as np
import cma
from rom_spline_surrogate import evaluate_rom_surrogate, TARGETS

# Load Master Ledger to grab exact champion parameters
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

# Find champion of the paused campaign (Gen 10 all-time best: W15_P23G10)
g10 = [g for g in ledger["generations"] if g["generation"] == 10][0]
p10 = g10["params"]

print("="*85)
print(f"Loaded Paused Campaign Champion: {g10['candidate']} (Score: {g10['score']:.2f})")
print("="*85)

# The 12 optimization parameters:
# 0: r_ton (K_ton / K_sc)
# 1: p_ton (rad/s)
# 2: K_sc (Nm/(rad/s))
# 3: r_lead_sc (z_sc / p_sc)
# 4: p_sc (rad/s)
# 5: K_phas (Nm/(m/s^2))
# 6: p_phas (rad/s)
# 7: kp (spindle gain)
# 8: r_lead_ccr (z_ccr / p_ccr)
# 9: p_ccr (rad/s)
# 10: k_dyn (dynamic gamma velocity sensitivity)
# 11: k_stat (static gamma tension sensitivity)

x0 = np.array([
    p10["r_ton"],       # 0: r_ton = 0.4015
    p10["p_ton"],       # 1: p_ton = 39.68
    p10["K_sc"],        # 2: K_sc = 1.63
    p10["r_lead_sc"],   # 3: r_lead_sc = 0.0311
    p10["p_sc"],        # 4: p_sc = 33.97
    p10["K_phas"],      # 5: K_phas = 7.43
    p10["p_phas"],      # 6: p_phas = 15.99
    p10["kp"],          # 7: kp = 1.097
    p10["r_lead_ccr"],  # 8: r_lead_ccr = 0.1438
    p10["p_ccr"],       # 9: p_ccr = 66.78
    p10["k_dyn"],       # 10: k_dyn = 1.20
    p10["k_stat"]       # 11: k_stat = 0.3516
])

# Expanded physiological parameter bounds
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
    1.50,   # 10: k_dyn (expanded headroom)
    1.50    # 11: k_stat
])

sigma0 = np.array([
    0.08,   # r_ton
    4.0,    # p_ton
    1.5,    # K_sc
    0.03,   # r_lead_sc
    6.0,    # p_sc
    1.5,    # K_phas
    6.0,    # p_phas
    0.25,   # kp
    0.03,   # r_lead_ccr
    6.0,    # p_ccr
    0.10,   # k_dyn
    0.10    # k_stat
])

def cand_to_param_dict(cand):
    r_ton = cand[0]
    p_ton = cand[1]
    K_sc = cand[2]
    r_lead_sc = cand[3]
    p_sc = cand[4]
    K_phas = cand[5]
    p_phas = cand[6]
    kp = cand[7]
    r_lead_ccr = cand[8]
    p_ccr = cand[9]
    k_dyn = cand[10]
    k_stat = cand[11]
    
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    return {
        "r_ton": r_ton,
        "K_ton": K_ton,
        "p_ton": p_ton,
        "K_sc": K_sc,
        "r_lead_sc": r_lead_sc,
        "z_sc": z_sc,
        "p_sc": p_sc,
        "K_phas": K_phas,
        "p_phas": p_phas,
        "kp": kp,
        "r_lead_ccr": r_lead_ccr,
        "z_ccr": z_ccr,
        "p_ccr": p_ccr,
        "k_dyn": k_dyn,
        "k_stat": k_stat
    }

def surrogate_objective(cand):
    # Boundary penalty
    penalty = 0.0
    cand_clamped = np.clip(cand, lower_bounds, upper_bounds)
    diff = cand - cand_clamped
    if np.any(diff != 0.0):
        penalty += np.sum(diff**2) * 10000.0
        
    p_dict = cand_to_param_dict(cand_clamped)
    score, _ = evaluate_rom_surrogate(p_dict)
    return score + penalty

# Test initial candidate score
p10_dict = cand_to_param_dict(x0)
score_init, bode_init = evaluate_rom_surrogate(p10_dict)
print(f"Initial Champion Score on Surrogate: {score_init:.2f} pts")

# Setup Ultra-Fast Surrogate CMA-ES
opts = {
    'bounds': [lower_bounds.tolist(), upper_bounds.tolist()],
    'CMA_stds': sigma0.tolist(),
    'popsize': 64,
    'maxiter': 250,
    'verb_disp': 25,
    'seed': 42
}

print(f"\nLaunching Surrogate CMA-ES Optimization (Popsize: 64, Max Generations: 250)...")
t_start = time.perf_counter()

es = cma.CMAEvolutionStrategy(x0, 0.5, opts)

best_score_history = []
while not es.stop():
    candidates = es.ask()
    scores = [surrogate_objective(c) for c in candidates]
    es.tell(candidates, scores)
    
    best_cand = es.result.xfavorite
    best_score = es.result.fbest
    best_score_history.append(best_score)
    
    if es.countiter % 25 == 0 or es.countiter == 1:
        print(f"  [Surrogate Gen {es.countiter:3d}/250] Current Best Score: {best_score:9.2f} pts")

t_elapsed = time.perf_counter() - t_start
total_evals = es.countevals

print("\n" + "="*85)
print(f"Surrogate CMA-ES Optimization Completed in {t_elapsed:.2f} seconds!")
print(f"Total Surrogate Candidates Evaluated: {total_evals:,} ({total_evals/t_elapsed:,.0f} evals/sec)")
print(f"Initial Champion Score:               {score_init:9.2f} pts")
print(f"New Optimized Champion Score:         {es.result.fbest:9.2f} pts (Target: < 2803.88)")
print("="*85)

opt_cand = es.result.xbest
opt_p = cand_to_param_dict(opt_cand)

final_score, final_bode = evaluate_rom_surrogate(opt_p)

print("\n--- Optimized Parameter Vector (Bootstrap Seed for Full Campaign) ---")
for k, v in opt_p.items():
    print(f"  {k:<12}: {v:10.4f}")

print("\n--- Predicted 6-Frequency Bode Response vs. Happee (2017) Benchmark ---")
print(f"{'Freq (Hz)':<10} | {'Target Gain':<14} | {'Target Phase':<14} | {'Pred Gain (dB)':<16} | {'Pred Phase Err':<16} | {'Freq Score':<12}")
print("-"*85)
for freq, (t_gain, t_phase) in TARGETS.items():
    b = final_bode[freq]
    print(f"{freq:<10.3f} | {t_gain:>11.2f}  | {t_phase:>11.2f}° | {b['gain_db']:>13.1f} dB | {b['phase_err']:>13.1f}° | {b['score']:>9.1f} pts")
print("="*85)

# Export the bootstrap candidate vector
export_data = {
    "bootstrap_timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    "optimizer": "Surrogate-ROM-CMA-ES",
    "total_surrogate_evaluations": total_evals,
    "elapsed_seconds": round(t_elapsed, 2),
    "initial_score": round(score_init, 2),
    "optimized_surrogate_score": round(final_score, 2),
    "target_benchmark": 2803.88,
    "optimized_params": opt_p,
    "bode_prediction": {str(k): v for k, v in final_bode.items()}
}

output_file = "logs/cmaes_surrogate_bootstrap_champion.json"
with open(output_file, "w", encoding="utf-8") as f:
    json.dump(export_data, f, indent=2)

print(f"\nExported bootstrap champion to {output_file} successfully!")
