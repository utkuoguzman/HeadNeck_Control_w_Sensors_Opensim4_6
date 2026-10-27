import json
import math
import cmath
import numpy as np
import scipy.optimize as opt

# 1. Load the Master Ledger containing Gen 1..11 ground truth OpenSim results
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

# Canonical Targets
TARGETS = {
    0.324: (86.55, -7.03),
    0.783: (236.93, -96.96),
    1.099: (319.74, -109.23),
    2.446: (423.34, -161.34),
    5.583: (503.30, -201.38),
    7.927: (572.69, -247.62)
}

delay_vcr = 0.010 # 10 ms
delay_ccr = 0.015 # 15 ms

def compute_surrogate_bode(params, plant_params, freq):
    # params: dict of reflex parameters
    # plant_params: [J_eff, B_pass, K_pass, M_drive, tau_act]
    J_eff, B_pass, K_pass, M_drive, tau_act = plant_params
    
    K_ton = params["K_ton"]
    p_ton = params["p_ton"]
    K_sc = params["K_sc"]
    z_sc = params["z_sc"]
    p_sc = params["p_sc"]
    K_phas = params["K_phas"]
    p_phas = params["p_phas"]
    kp = params["kp"]
    z_ccr = params["z_ccr"]
    p_ccr = params["p_ccr"]
    k_dyn = params.get("k_dyn", 0.25)
    k_stat = params.get("k_stat", 0.40)
    
    omega = 2.0 * math.pi * freq
    s = 1j * omega
    
    # Muscle activation dynamics
    H_act = 1.0 / (tau_act * s + 1.0)
    
    # Semicircular canal (rate feedback)
    H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
    
    # Tonic otolith (angle feedback)
    H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
    
    # Phasic otolith (acceleration feedforward)
    H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
    
    # Cervicocollic reflex (CCR) with dynamic gamma velocity scaling
    # Spindle dynamic lead: (1 + k_dyn * (s/omega_ref))
    # In C++, CCR filter is: kp * (s + z_ccr)/(s + p_ccr)
    H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
    
    # Total reflexive impedance on pitch angle theta
    Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
    
    # Plant mechanical impedance
    Z_plant = - (omega**2) * J_eff + s * B_pass + K_pass
    
    # Driving torque from sled acceleration
    # Inertial drive minus phasic otolith feedback
    T_drive = - (omega**2) * (M_drive - K_phas * H_phas * H_act)
    
    # Closed-loop transfer function: Theta / X_sled
    Theta_over_X = T_drive / (Z_plant + Z_reflex)
    
    # Angular velocity gain in deg/m: |s * Theta| / |s * X_sled| = |Theta| / |X_sled|
    gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
    
    # Phase in degrees (with OpenSim angle wrap-around to [-270, 90])
    phase_rad = cmath.phase(Theta_over_X)
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    
    return gain_deg_m, phase_deg

def score_candidate(params, plant_params):
    total_score = 0.0
    bode_results = {}
    for freq, (t_gain, t_phase) in TARGETS.items():
        sim_gain, sim_phase = compute_surrogate_bode(params, plant_params, freq)
        
        sim_gain_safe = max(1e-6, sim_gain)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        
        phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        f_score = gain_loss + phase_loss
        total_score += f_score
        bode_results[freq] = {
            "gain_db": round(gain_db_diff, 1),
            "phase_err": round(abs(phase_diff), 1),
            "score": round(f_score, 1)
        }
    return total_score, bode_results

print(f"Loaded {len(ledger['generations'])} generations from ledger.")

# Build training set from OpenSim ground truth
train_gens = [1, 3, 5, 7, 9]
test_gens = [2, 4, 6, 8, 10, 11]

def plant_loss(plant_params):
    loss = 0.0
    for g_data in ledger["generations"]:
        g = g_data["generation"]
        if g not in train_gens:
            continue
        params = g_data["params"]
        true_bode = g_data["bode"]
        
        for freq_str, b_true in true_bode.items():
            freq = float(freq_str)
            sim_gain, sim_phase = compute_surrogate_bode(params, plant_params, freq)
            t_gain, t_phase = TARGETS[freq]
            
            sim_gain_safe = max(1e-6, sim_gain)
            gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
            phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
            
            # Loss against OpenSim recorded gain_db and phase_err
            true_gain_db = b_true["gain_db"]
            true_phase_err = b_true["phase_err"]
            
            loss += (gain_db_diff - true_gain_db)**2 + (abs(phase_diff) - true_phase_err)**2
    return loss

# Bounds for plant parameters: [J_eff, B_pass, K_pass, M_drive, tau_act]
# Physical expectations:
# J_eff ~ 0.05 to 0.25 kg m^2
# B_pass ~ 0.2 to 2.5 Nms/rad
# K_pass ~ 5.0 to 40.0 Nm/rad
# M_drive ~ 0.3 to 1.5 kg m
# tau_act ~ 0.010 to 0.040 s
bounds = [
    (0.02, 0.35),   # J_eff
    (0.10, 3.50),   # B_pass
    (2.0, 50.0),    # K_pass
    (0.10, 2.50),   # M_drive
    (0.005, 0.050)  # tau_act
]

p0 = [0.126, 0.80, 15.0, 0.675, 0.025]

print("Fitting OpenSim Cervical Plant Parameters using Train Generations (1, 3, 5, 7, 9)...")
res = opt.minimize(plant_loss, p0, bounds=bounds, method='L-BFGS-B')
opt_plant = res.x

print("\n--- Identified OpenSim Cervical Plant Parameters ---")
print(f"  J_eff (Effective Inertia):       {opt_plant[0]:.4f} kg m^2")
print(f"  B_pass (Passive Damping):       {opt_plant[1]:.4f} Nms/rad")
print(f"  K_pass (Passive Bushing/Spine): {opt_plant[2]:.4f} Nm/rad")
print(f"  M_drive (Base Inertial Drive):  {opt_plant[3]:.4f} kg m")
print(f"  tau_act (Muscle Act Dynamics):  {opt_plant[4]*1000.0:.1f} ms")
print(f"  Fit Residual Loss:              {res.fun:.2f}")

# Benchmark Speed
import time
t_start = time.perf_counter()
n_evals = 10000
for _ in range(n_evals):
    _ = score_candidate(ledger["generations"][9]["params"], opt_plant)
t_elapsed = time.perf_counter() - t_start
time_per_eval_us = (t_elapsed / n_evals) * 1e6

print(f"\n--- Surrogate Execution Speed ---")
print(f"  Evaluated {n_evals:,} full 6-frequency candidates in {t_elapsed:.3f} s")
print(f"  Time per candidate: {time_per_eval_us:.2f} microseconds ({1.0/(t_elapsed/n_evals):,.0f} evaluations/sec)")

# Validation Across ALL Generations
print("\n" + "="*80)
print(f"{'Gen':<5} | {'Candidate':<12} | {'OpenSim Score':<14} | {'Surrogate Score':<16} | {'Abs Error':<10} | {'Rel Err %':<10}")
print("="*80)

total_rel_err = 0.0
for g_data in ledger["generations"]:
    g = g_data["generation"]
    cand = g_data["candidate"]
    true_score = g_data["score"]
    surr_score, surr_bode = score_candidate(g_data["params"], opt_plant)
    
    abs_err = abs(surr_score - true_score)
    rel_err = (abs_err / true_score) * 100.0
    total_rel_err += rel_err
    
    tag = " [TEST]" if g in test_gens else " [TRAIN]"
    print(f"G{g:<3} | {cand:<12} | {true_score:>13.2f} | {surr_score:>15.2f} | {abs_err:>9.2f} | {rel_err:>8.1f}%{tag}")

mean_rel_err = total_rel_err / len(ledger["generations"])
print("="*80)
print(f"Mean Relative Prediction Error across all 11 Generations: {mean_rel_err:.2f}%")

