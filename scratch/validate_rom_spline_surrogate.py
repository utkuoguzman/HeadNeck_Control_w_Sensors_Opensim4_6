import json
import math
import numpy as np
from rom_spline_surrogate import evaluate_rom_surrogate, TARGETS

# Load Master Ledger
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

print("="*95)
print(f"{'Gen':<5} | {'Candidate':<12} | {'OpenSim Score':<14} | {'ROM Spline Score':<18} | {'Abs Error':<10} | {'Rel Err %':<10}")
print("="*95)

total_rel_err = 0.0
for g_data in ledger["generations"]:
    g = g_data["generation"]
    cand = g_data["candidate"]
    true_score = g_data["score"]
    params = g_data["params"]
    
    surr_score, surr_bode = evaluate_rom_surrogate(params)
    
    abs_err = abs(surr_score - true_score)
    rel_err = (abs_err / true_score) * 100.0
    total_rel_err += rel_err
    
    print(f"G{g:<3} | {cand:<12} | {true_score:>13.2f} | {surr_score:>17.2f} | {abs_err:>9.2f} | {rel_err:>8.1f}%")

mean_rel_err = total_rel_err / len(ledger["generations"])
print("="*95)
print(f"Mean Relative Error across all {len(ledger['generations'])} Generations: {mean_rel_err:.2f}%")

# Detailed check on Generation 10 All-Time Best
g10 = [g for g in ledger["generations"] if g["generation"] == 10][0]
score_10, bode_10 = evaluate_rom_surrogate(g10["params"])

print("\n" + "="*95)
print(f"Frequency-by-Frequency Comparison on Generation 10 ({g10['candidate']}):")
print(f"{'Freq (Hz)':<10} | {'OpenSim Gain (dB)':<18} | {'ROM Spline Gain':<18} | {'OpenSim Phase Err':<18} | {'ROM Spline Phase Err':<18}")
print("="*95)

for freq, (t_gain, t_phase) in TARGETS.items():
    b_true = g10["bode"][str(freq)]
    b_surr = bode_10[freq]
    
    g_true = b_true["gain_db"]
    p_true = b_true["phase_err"]
    g_surr = b_surr["gain_db"]
    p_surr = b_surr["phase_err"]
    
    print(f"{freq:<10.3f} | {g_true:>16.1f} dB | {g_surr:>16.1f} dB | {p_true:>16.1f}° | {p_surr:>16.1f}°")
print("="*95)

