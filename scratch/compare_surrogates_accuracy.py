import math
import cmath
import json
import numpy as np

# 1. Ground Truth from cervical_fast_sim.exe (Exact OpenSim compiled C++ on current model)
# Recorded from task-33154:
OPENSIM_GROUND_TRUTH = {
    0.324: {"gain": 20.52,  "phase": -5.09,   "score": 15637.91},
    0.783: {"gain": 139.22, "phase": -88.73,  "score": 2200.64},
    1.099: {"gain": 207.62, "phase": -99.21,  "score": 1506.87},
    2.446: {"gain": 703.65, "phase": -161.58, "score": 1947.87},
    5.583: {"gain": 887.19, "phase": 25.82,   "score": 20059.33},
    7.927: {"gain": 694.47, "phase": 5.79,    "score": 11642.33}
}

TARGETS = [
    (0.324, 86.55,   -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

w3_params = [
    0.5769, 76.4989, 0.5302, 0.2458, 96.0501, 
    4.2943, 17.5282, 1.2839, 0.0687, 177.0660, 
    0.7189, 1.1773
]

# --- 2. SURROGATE A: Frequency-Domain ROM ---
def eval_surrogate_rom(params):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = params
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr

    J_eff = 0.1633
    B_pass = 1.46
    K_pass = 5.20
    M_drive = 0.778
    tau_act = 0.012
    k_geom = 1.0

    delay_vcr = 0.010
    delay_ccr = 0.015

    results = {}
    for freq, t_gain, t_phase in TARGETS:
        omega = 2.0 * math.pi * freq
        s = 1j * omega

        H_act = 1.0 / (tau_act * s + 1.0)
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
        H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)

        Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
        Z_plant = - (omega**2) * J_eff + s * B_pass + K_pass
        # Negative sign due to coordinate inversion in OpenSim sled vs head tilt
        T_drive = - (omega**2) * (M_drive - K_phas * H_phas * H_act)

        Theta_over_X = (T_drive / (Z_plant + Z_reflex)) * k_geom
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0

        results[freq] = {"gain": gain_deg_m, "phase": phase_deg}
    return results

# --- 3. SURROGATE B: 8-Joint / 72-Muscle Chadwick-Blana Simulator ---
import sys
sys.path.append("scratch")
from cervical_chadwick_blana_simulator import simulate_chadwick_blana_frequency

def eval_surrogate_chadwick(params):
    results = {}
    for freq, t_gain, t_phase in TARGETS:
        f_score, g_db, p_err, sim_g, sim_p = simulate_chadwick_blana_frequency(params, freq, t_gain, t_phase)
        results[freq] = {"gain": sim_g, "phase": sim_p, "score": f_score}
    return results

print("Evaluating Surrogate A (Frequency-Domain ROM)...")
rom_res = eval_surrogate_rom(w3_params)

print("Evaluating Surrogate B (8-Joint / 72-Muscle Chadwick-Blana ODE)...")
cb_res = eval_surrogate_chadwick(w3_params)

print("\nDetailed Comparison against OpenSim Ground Truth:")
print(f"{'Freq (Hz)':<10} | {'OpenSim Truth':<22} | {'Surrogate A (ROM)':<22} | {'Surrogate B (8-Joint ODE)':<22}")
print(f"{'':<10} | {'Gain':<9} {'Phase':<12} | {'Gain':<9} {'Phase':<12} | {'Gain':<9} {'Phase':<12}")
print("-" * 85)

for freq, _, _ in TARGETS:
    gt_g = OPENSIM_GROUND_TRUTH[freq]["gain"]
    gt_p = OPENSIM_GROUND_TRUTH[freq]["phase"]
    
    rom_g = rom_res[freq]["gain"]
    rom_p = rom_res[freq]["phase"]
    
    cb_g = cb_res[freq]["gain"]
    cb_p = cb_res[freq]["phase"]

    print(f"{freq:<10.3f} | {gt_g:<9.1f} {gt_p:>6.1f}°     | {rom_g:<9.1f} {rom_p:>6.1f}°     | {cb_g:<9.1f} {cb_p:>6.1f}°")


