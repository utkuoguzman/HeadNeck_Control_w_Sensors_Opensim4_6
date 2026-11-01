import json
import math
import cmath
import time
import numpy as np
from scipy.stats import qmc

# Load champion W7_P23G13
with open("logs/candidates/W7_P23G13.json", "r", encoding="utf-8") as f:
    cand_data = json.load(f)

# Candidate vector:
# [r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat]
p = cand_data["params"]
K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, k_dyn, k_stat = p
r_ton = K_ton / K_sc
r_lead_sc = z_sc / p_sc
r_lead_ccr = z_ccr / p_ccr

champ_vec = np.array([r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat])

SURR_DELAY_VCR = 0.010
SURR_DELAY_CCR = 0.015
SURR_OPT_PLANT = (0.0185, 0.45, 12.5, 0.145, 0.012, 1.0)
TARGETS = [(0.324, (86.55, -7.03)), (0.783, (236.93, -96.96)), (1.099, (319.74, -109.23)), (2.446, (423.34, -161.34)), (5.583, (503.30, -201.38)), (7.927, (572.69, -247.62))]

def surrogate_score(cand):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = cand
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    J_eff, B_pass, K_pass, M_drive, tau_act, k_geom = SURR_OPT_PLANT
    
    total_score = 0.0
    bode = {}
    for freq, (t_gain, t_phase) in TARGETS:
        omega = 2.0 * math.pi * freq
        s = 1j * omega
        
        H_act = 1.0 / (tau_act * s + 1.0)
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_ccr = ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * SURR_DELAY_CCR)
        
        Z_reflex = (K_sc * H_sc + K_ton * H_ton + kp * (1.0 + k_dyn * 0.5) * H_ccr) * H_act * k_geom
        Z_plant = J_eff * (s**2) + B_pass * s + K_pass
        
        T_drive = (omega**2) * (M_drive - 0.015 * K_phas * H_phas * H_act)
        Theta_over_X = T_drive / (Z_plant + Z_reflex)
        
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0
        
        gain_db_diff = 20.0 * math.log10(max(1e-6, gain_deg_m) / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        total_score += (gain_loss + phase_loss)
        bode[freq] = (gain_db_diff, abs(phase_diff))
    return total_score, bode

print(f"Baseline W7_P23G13 (p_ccr={p_ccr:.1f} rad/s):")
base_sc, base_bode = surrogate_score(champ_vec)
print(f"  Algebraic Surrogate Score: {base_sc:.1f} pts")
print(f"  7.927 Hz: Gain Err: {base_bode[7.927][0]:.1f} dB | Phase Err: {base_bode[7.927][1]:.1f}°")

print("\n1D Parameter Sweep: Varying p_ccr from 50 to 200 rad/s on W7_P23G13:")
for test_pccr in [50, 70, 90, 110, 130, 150, 170, 200]:
    cand = champ_vec.copy()
    cand[9] = test_pccr
    sc, bode = surrogate_score(cand)
    print(f"  p_ccr = {test_pccr:>3d} rad/s -> Score: {sc:>8.1f} pts | 7.927 Hz Phase Err: {bode[7.927][1]:>5.1f}° | 5.583 Hz Phase Err: {bode[5.583][1]:>5.1f}°")

