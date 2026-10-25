import numpy as np
import scipy.optimize as opt
import pandas as pd
import cmath
import math
import subprocess
import json
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed

# Load aligned experimental target data
df = pd.read_csv("extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv")
targets = {}
for _, row in df.iterrows():
    targets[round(row['Frequency_Hz'], 3)] = (row['Exp_Gain_deg_per_m'], row['Exp_Phase_deg'])

I_eff = 0.126
m = 4.5
h = 0.15
mgh = m * 9.81 * h
K_passive = 12.0
B_passive = 0.80
tau_act = 0.030
delay_vcr = 0.010
delay_ccr = 0.015

def transfer_function(params, freq):
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr = params
    omega = 2.0 * math.pi * freq
    s = 1j * omega
    H_act = 1.0 / (tau_act * s + 1.0)
    H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
    H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
    H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
    H_ccr = ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
    Z_fb = (K_sc * H_sc * s + K_ton * H_ton + kp * H_ccr) * H_act
    Z_plant = I_eff * (s**2) + B_passive * s + (K_passive - mgh)
    T_drive = (m * h * (omega**2) - K_phas * H_phas * (omega**2) * H_act)
    Theta_over_X = T_drive / (Z_plant + Z_fb)
    Gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
    phase_rad = cmath.phase(Theta_over_X)
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    return Gain_deg_m, phase_deg

def objective(params):
    loss = 0.0
    for f, (t_gain, t_phase) in targets.items():
        sg, sp = transfer_function(params, f)
        gain_db_diff = 20.0 * math.log10(max(1e-6, sg) / t_gain)
        phase_diff = (sp - t_phase + 180.0) % 360.0 - 180.0
        loss += ((gain_db_diff / 1.0) ** 2) * 100.0 + ((phase_diff / 10.0) ** 2) * 100.0
    return loss

# 1. Unconstrained Surrogate Solution
x0 = np.array([2.0, 10.0, 0.55, 10.0, 50.0, 0.87, 35.0, 0.81, 12.0, 45.0])
bounds_unconstrained = [
    (0.05, 15.0), (2.0, 40.0), (0.05, 10.0), (0.5, 30.0), (15.0, 100.0),
    (0.05, 10.0), (10.0, 60.0), (0.05, 3.0), (0.5, 30.0), (15.0, 90.0)
]
res_unconstrained = opt.minimize(objective, x0, method='L-BFGS-B', bounds=bounds_unconstrained)

# 2. Gravity-Stable Constrained Solution (K_ton >= 1.8, kp >= 0.45 per Happee 2017)
bounds_stable = [
    (1.80, 15.0), (2.0, 40.0), (0.10, 10.0), (0.5, 30.0), (15.0, 100.0),
    (0.05, 10.0), (10.0, 60.0), (0.45, 3.0), (0.5, 30.0), (15.0, 90.0)
]
res_stable = opt.minimize(objective, x0, method='L-BFGS-B', bounds=bounds_stable)

print("Surrogate Gradient Descent Results:")
print(f"  Unconstrained Score: {res_unconstrained.fun:.1f} | K_ton={res_unconstrained.x[0]:.2f}, K_sc={res_unconstrained.x[2]:.2f}, kp={res_unconstrained.x[7]:.2f}")
print(f"  Stable-Bounded Score: {res_stable.fun:.1f} | K_ton={res_stable.x[0]:.2f}, K_sc={res_stable.x[2]:.2f}, kp={res_stable.x[7]:.2f}")

def evaluate_in_opensim(label, params, freq):
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr = params
    t_gain, t_phase = targets[freq]
    worker_label = f"GD2_{label}_{freq:.2f}Hz"
    
    r_ton = K_ton / K_sc
    r_lead_sc = z_sc / p_sc
    r_lead_ccr = z_ccr / p_ccr
    k_dyn = 0.25
    k_stat = 0.40
    
    cmd = [
        r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        str(r_ton), str(p_ton), str(K_sc), str(r_lead_sc), str(p_sc),
        str(K_phas), str(p_phas), str(kp), str(r_lead_ccr), str(p_ccr),
        str(k_dyn), str(k_stat),
        str(freq), str(t_gain), str(t_phase),
        worker_label
    ]
    
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        if res.returncode == 0:
            lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
            score = float(lines[-1])
            res_file = os.path.join("logs", "candidates", f"{worker_label}_{freq:.2f}Hz.json")
            if os.path.exists(res_file):
                with open(res_file, "r") as fp:
                    d = json.load(fp)
                return label, freq, d
            return label, freq, {"score": score}
        else:
            print(f"[FAIL] {worker_label}: {res.stderr}", file=sys.stderr)
            return label, freq, None
    except Exception as e:
        print(f"[ERROR] {worker_label}: {e}", file=sys.stderr)
        return label, freq, None

def main():
    test_freqs = [7.927, 5.583, 2.446, 1.099, 0.783, 0.324]
    print(f"\nEvaluating Gradient-Descent Solutions in OpenSim across {test_freqs}...")
    sys.stdout.flush()
    
    tasks = []
    with ThreadPoolExecutor(max_workers=10) as executor:
        for f in test_freqs:
            tasks.append(executor.submit(evaluate_in_opensim, "Unconstrained", res_unconstrained.x, f))
            tasks.append(executor.submit(evaluate_in_opensim, "StableBounded", res_stable.x, f))
            
        results = []
        for future in as_completed(tasks):
            res = future.result()
            results.append(res)
            
    print("\n" + "=" * 115)
    print(f"{'Candidate':<15} | {'Freq [Hz]':<10} | {'Sim Gain':<10} | {'Tgt Gain':<10} | {'Gain Diff':<10} | {'Sim Phase':<10} | {'Tgt Phase':<10} | {'Phase Diff':<11} | {'Score':<8}")
    print("-" * 115)
    
    results.sort(key=lambda x: (x[0], x[1]))
    totals = {}
    for label, freq, d in results:
        if d:
            sg = d.get('sim_gain', 0.0)
            tg = d.get('target_gain', 0.0)
            sp = d.get('sim_phase', 0.0)
            tp = d.get('target_phase', 0.0)
            g_diff = d.get('gain_db_diff', 0.0)
            p_diff = d.get('phase_err_deg', 0.0)
            score = d.get('score', 0.0)
            totals[label] = totals.get(label, 0.0) + score
            print(f"{label:<15} | {freq:<10.3f} | {sg:<10.1f} | {tg:<10.1f} | {g_diff:+10.2f}dB | {sp:<10.1f}° | {tp:<10.1f}° | {p_diff:11.1f}° | {score:<8.1f}")
        else:
            print(f"{label:<15} | {freq:<10.3f} | FAILED")
            
    print("-" * 115)
    print("TOTAL SCORE ACROSS 6 FREQUENCIES IN OPENSIM:")
    for label, tot in sorted(totals.items(), key=lambda x: x[1]):
        print(f"  {label:<15}: {tot:.1f} pts")
    sys.stdout.flush()

if __name__ == "__main__":
    main()

