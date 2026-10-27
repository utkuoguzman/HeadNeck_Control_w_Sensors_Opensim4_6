import json
import math
import cmath
import numpy as np
import scipy.optimize as opt

# Load ledger
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

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
tau_act = 0.015

def get_reflex_terms(params, freq):
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
    
    omega = 2.0 * math.pi * freq
    s = 1j * omega
    
    H_act = 1.0 / (tau_act * s + 1.0)
    H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
    H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
    H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
    H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
    
    Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
    T_phas = K_phas * H_phas * H_act
    return Z_reflex, T_phas

# Identify Z_plant(j omega_k) and M_drive(omega_k) directly for each frequency k
# Each frequency has 3 real parameters: Re(Z_k), Im(Z_k), and M_drive_k
freq_list = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]

train_gens = [1, 3, 5, 7, 9]
test_gens = [2, 4, 6, 8, 10, 11]

fitted_plant = {}

for freq in freq_list:
    omega = 2.0 * math.pi * freq
    t_gain, t_phase = TARGETS[freq]
    
    def freq_loss(p):
        Re_Z, Im_Z, M_drive = p
        Z_plant = complex(Re_Z, Im_Z)
        
        loss = 0.0
        for g_data in ledger["generations"]:
            if g_data["generation"] not in train_gens:
                continue
            params = g_data["params"]
            b_true = g_data["bode"][str(freq)]
            
            Z_reflex, T_phas = get_reflex_terms(params, freq)
            T_drive = - (omega**2) * (M_drive - T_phas)
            
            Theta_over_X = T_drive / (Z_plant + Z_reflex)
            gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
            
            phase_rad = cmath.phase(Theta_over_X)
            phase_deg = math.degrees(phase_rad)
            while phase_deg > 90.0: phase_deg -= 360.0
            while phase_deg < -270.0: phase_deg += 360.0
            
            gain_db_diff = 20.0 * math.log10(max(1e-6, gain_deg_m) / t_gain)
            phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
            
            loss += (gain_db_diff - b_true["gain_db"])**2 + (abs(phase_diff) - b_true["phase_err"])**2
        return loss

    res = opt.minimize(freq_loss, [10.0, 1.0, 0.5], method='Nelder-Mead')
    fitted_plant[freq] = res.x

print("Frequency-Specific Plant Impedances Fitted.")

def score_candidate_spline(params):
    total_score = 0.0
    for freq in freq_list:
        omega = 2.0 * math.pi * freq
        t_gain, t_phase = TARGETS[freq]
        Re_Z, Im_Z, M_drive = fitted_plant[freq]
        Z_plant = complex(Re_Z, Im_Z)
        
        Z_reflex, T_phas = get_reflex_terms(params, freq)
        T_drive = - (omega**2) * (M_drive - T_phas)
        
        Theta_over_X = T_drive / (Z_plant + Z_reflex)
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0
        
        gain_db_diff = 20.0 * math.log10(max(1e-6, gain_deg_m) / t_gain)
        gain_loss = (gain_db_diff / 1.0)**2 * 100.0
        
        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = (phase_diff / 10.0)**2 * 100.0
        
        total_score += (gain_loss + phase_loss)
    return total_score

print("\n" + "="*85)
print(f"{'Gen':<5} | {'Candidate':<12} | {'OpenSim Score':<14} | {'Surrogate Score':<16} | {'Abs Error':<10} | {'Rel Err %':<10}")
print("="*85)

total_rel_err = 0.0
for g_data in ledger["generations"]:
    g = g_data["generation"]
    cand = g_data["candidate"]
    true_score = g_data["score"]
    surr_score = score_candidate_spline(g_data["params"])
    
    abs_err = abs(surr_score - true_score)
    rel_err = (abs_err / true_score) * 100.0
    total_rel_err += rel_err
    
    tag = " [TEST]" if g in test_gens else " [TRAIN]"
    print(f"G{g:<3} | {cand:<12} | {true_score:>13.2f} | {surr_score:>15.2f} | {abs_err:>9.2f} | {rel_err:>8.1f}%{tag}")

mean_rel_err = total_rel_err / len(ledger["generations"])
print("="*85)
print(f"Mean Relative Prediction Error across all 11 Generations: {mean_rel_err:.2f}%")

