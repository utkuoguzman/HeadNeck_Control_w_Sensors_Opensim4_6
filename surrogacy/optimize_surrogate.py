"""
surrogacy/optimize_surrogate.py
Step 2 of the Differentiable Neuromotor Surrogate Pipeline.

Uses Black's closed-loop feedback formula on top of the exact empirical OpenSim
passive plant response (surrogacy/opensim_passive_plant.json) to solve for optimal
neuromuscular reflex parameters (VCR, CCR, transport delays) via multi-start
L-BFGS-B gradient descent in under 10 seconds.

Outputs:
    surrogacy/exact_plant_surrogate_solution.json: The optimal reflex parameter vector.
"""

import json
import math
import cmath
import os
import sys
import numpy as np
import scipy.optimize as opt
import pandas as pd

# Load exact OpenSim passive plant measurements
passive_file = os.path.join(os.path.dirname(__file__), "opensim_passive_plant.json")
with open(passive_file, "r") as f:
    passive_data = json.load(f)

# Load canonical Happee 2017 experimental data
data_file = "extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv"
df = pd.read_csv(data_file)
targets = {round(r['Frequency_Hz'], 3): (r['Exp_Gain_deg_per_m'], r['Exp_Phase_deg']) for _, r in df.iterrows()}

freqs = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]

# Biomechanical constants
m_h = 4.30 * 0.15 # skull mass * height = 0.645 Nm/(m/s^2)
tau_act = 0.030 # 30 ms muscle activation dynamics
delay_vcr = 0.010 # 10 ms
delay_ccr = 0.015 # 15 ms

# Build complex G_passive for each frequency
G_passive = {}
for f in freqs:
    d = passive_data[str(f)]
    gain_rad_m = d['gain_deg_m'] * (math.pi / 180.0)
    phase_rad = math.radians(d['phase_deg'])
    G_passive[f] = cmath.rect(gain_rad_m, phase_rad)

def closed_loop_tf(params, freq):
    # params: [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr]
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr = params
    
    omega = 2.0 * math.pi * freq
    s = 1j * omega
    
    H_act = 1.0 / (tau_act * s + 1.0)
    H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
    H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
    H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
    H_ccr = ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
    
    # Reflex feedback torque per unit angle theta (in rad):
    Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + kp * H_ccr) * H_act
    
    # Inertial mass drive torque per unit sled displacement:
    T_mass = m_h * (omega**2)
    
    # Phasic otolith feedforward cancellation torque:
    T_phas = K_phas * H_phas * H_act * (omega**2)
    feedforward_factor = 1.0 - (T_phas / T_mass)
    
    # Exact Black's feedback formula:
    G_pass = G_passive[freq]
    L = G_pass * (Z_reflex / T_mass)
    G_closed = (G_pass * feedforward_factor) / (1.0 + L)
    
    gain_deg_m = abs(G_closed) * (180.0 / math.pi)
    phase_deg = math.degrees(cmath.phase(G_closed))
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    return gain_deg_m, phase_deg

def objective(params):
    loss = 0.0
    for f in freqs:
        tg, tp = targets[f]
        sg, sp = closed_loop_tf(params, f)
        
        # 1.0 dB Gain Error == 10.0 deg Phase Error == 100 pts
        gain_db_diff = 20.0 * math.log10(max(1e-6, sg) / tg)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        
        phase_diff = (sp - tp + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        loss += (gain_loss + phase_loss)
    return loss

# Physiological search bounds per Happee (2017)
bounds_phys = [
    (1.80, 15.0), # 0: K_ton
    (2.0, 40.0),  # 1: p_ton
    (0.10, 10.0), # 2: K_sc
    (0.5, 30.0),  # 3: z_sc
    (15.0, 100.0),# 4: p_sc
    (0.01, 10.0), # 5: K_phas
    (10.0, 60.0), # 6: p_phas
    (0.45, 3.0),  # 7: kp
    (0.5, 30.0),  # 8: z_ccr
    (15.0, 90.0)  # 9: p_ccr
]

def main():
    print("="*95)
    print("DIFFERENTIABLE SURROGATE OPTIMIZATION USING EXACT OPENSIM MULTI-BODY PLANT")
    print("="*95)

    best_res = None
    best_score = float('inf')

    np.random.seed(2026)
    seeds = [
        np.array([1.80, 2.0, 0.55, 10.0, 50.0, 0.87, 35.0, 0.45, 12.0, 45.0]),
        np.array([2.50, 5.0, 1.20, 5.0, 30.0, 0.50, 40.0, 0.70, 5.0, 60.0]),
        np.array([1.80, 2.0, 1.50, 15.0, 40.0, 0.05, 60.0, 0.45, 0.5, 88.0]),
        np.array([3.00, 10.0, 2.00, 8.0, 50.0, 1.00, 30.0, 0.80, 10.0, 50.0])
    ]

    for _ in range(20):
        rnd = np.array([np.random.uniform(b[0], b[1]) for b in bounds_phys])
        seeds.append(rnd)

    print(f"Running multi-start L-BFGS-B gradient descent across {len(seeds)} starting points...")
    for i, s0 in enumerate(seeds):
        try:
            res = opt.minimize(objective, s0, method='L-BFGS-B', bounds=bounds_phys, options={'maxiter': 500})
            if res.fun < best_score:
                best_score = res.fun
                best_res = res
                print(f"  [Improvement] Start {i+1:2d}: Score = {best_score:.1f}")
        except Exception:
            pass

    print("\n" + "="*95)
    print(f"OPTIMAL SURROGATE SCORE: {best_score:.1f} pts")
    print("="*95)
    p_opt = best_res.x
    names = ["K_ton", "p_ton", "K_sc", "z_sc", "p_sc", "K_phas", "p_phas", "kp", "z_ccr", "p_ccr"]
    print("\nOptimal Parameter Set:")
    for n, val in zip(names, p_opt):
        print(f"  {n:<10}: {val:8.3f}")

    r_ton = p_opt[0] / p_opt[2]
    r_lead_sc = p_opt[3] / p_opt[4]
    r_lead_ccr = p_opt[8] / p_opt[9]
    print(f"\nCoupling Ratios:")
    print(f"  r_ton      (K_ton / K_sc)  : {r_ton:.3f}")
    print(f"  r_lead_sc  (z_sc / p_sc)   : {r_lead_sc:.3f}")
    print(f"  r_lead_ccr (z_ccr / p_ccr) : {r_lead_ccr:.3f}")

    print("\nPredicted Closed-Loop Bode Frequency Response vs Happee Target:")
    print(f"{'Freq [Hz]':<10} | {'Pred Gain':<10} | {'Tgt Gain':<10} | {'Gain Diff':<10} | {'Pred Phase':<10} | {'Tgt Phase':<10} | {'Phase Diff':<11} | {'Score':<8}")
    print("-" * 95)
    tot = 0.0
    for f in freqs:
        sg, sp = closed_loop_tf(p_opt, f)
        tg, tp = targets[f]
        g_diff = 20.0 * math.log10(max(1e-6, sg) / tg)
        p_diff = (sp - tp + 180.0) % 360.0 - 180.0
        g_loss = ((g_diff / 1.0) ** 2) * 100.0
        p_loss = ((p_diff / 10.0) ** 2) * 100.0
        sc = g_loss + p_loss
        tot += sc
        print(f"{f:<10.3f} | {sg:<10.1f} | {tg:<10.1f} | {g_diff:+10.2f}dB | {sp:<10.1f}° | {tp:<10.1f}° | {abs(p_diff):11.1f}° | {sc:<8.1f}")
    print("-" * 95)
    print(f"TOTAL PREDICTED SCORE: {tot:.1f} pts")

    opt_data = {
        "score": best_score,
        "params": list(p_opt),
        "r_ton": r_ton,
        "r_lead_sc": r_lead_sc,
        "r_lead_ccr": r_lead_ccr,
        "k_dyn": 0.25,
        "k_stat": 0.40
    }
    out_sol = os.path.join(os.path.dirname(__file__), "exact_plant_surrogate_solution.json")
    with open(out_sol, "w") as fp:
        json.dump(opt_data, fp, indent=2)
    print(f"Saved optimal solution to {out_sol}")

if __name__ == '__main__':
    main()

