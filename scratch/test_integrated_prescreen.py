import json
import math
import cmath
import os
import time
import numpy as np

# Refined plant parameters from test_offline_surrogate_refined
# [J_eff, B_pass, K_pass, M_drive, tau_act, k_geom]
OPT_PLANT = [0.6000, 0.3636, 2.8558, 0.0050, 0.0085, 1.348]

TARGETS = {
    0.324: (86.55, -7.03),
    0.783: (236.93, -96.96),
    1.099: (319.74, -109.23),
    2.446: (423.34, -161.34),
    5.583: (503.30, -201.38),
    7.927: (572.69, -247.62)
}

delay_vcr = 0.010
delay_ccr = 0.015

def surrogate_score(cand):
    # cand: 12-vector
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = cand
    
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    J_eff, B_pass, K_pass, M_drive, tau_act, k_geom = OPT_PLANT
    
    total_score = 0.0
    for freq, (t_gain, t_phase) in TARGETS.items():
        omega = 2.0 * math.pi * freq
        s = 1j * omega
        
        H_act = 1.0 / (tau_act * s + 1.0)
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
        H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
        
        Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
        Z_plant = - (omega**2) * J_eff + s * B_pass + K_pass
        T_drive = - (omega**2) * (M_drive - K_phas * H_phas * H_act)
        
        Theta_over_X = (T_drive / (Z_plant + Z_reflex)) * k_geom
        
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0
        
        sim_gain_safe = max(1e-6, gain_deg_m)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        
        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        total_score += (gain_loss + phase_loss)
    return total_score

# Test timing on 72 candidates
test_pool = [np.random.uniform(0.1, 1.0, 12) for _ in range(72)]
t0 = time.perf_counter()
scores = [surrogate_score(c) for c in test_pool]
dt_ms = (time.perf_counter() - t0) * 1000.0

print(f"Pre-screening evaluated {len(test_pool)} candidates in {dt_ms:.2f} ms ({dt_ms/len(test_pool)*1000.0:.2f} us per candidate)!")
print(f"Scores min: {min(scores):.1f}, median: {np.median(scores):.1f}, max: {max(scores):.1f}")

