import json
import math
import cmath

# Load test script and check detailed frequency comparison for Gen 10
from test_offline_surrogate import compute_surrogate_bode, opt_plant, TARGETS

with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

g10 = ledger["generations"][9]
params10 = g10["params"]

print("="*90)
print(f"Detailed Frequency Comparison for Generation 10 ({g10['candidate']}):")
print(f"{'Freq (Hz)':<10} | {'OpenSim Gain (dB)':<18} | {'Surr Gain (dB)':<16} | {'OpenSim Phase Err':<18} | {'Surr Phase Err':<16}")
print("="*90)

for freq, (t_gain, t_phase) in TARGETS.items():
    sim_gain, sim_phase = compute_surrogate_bode(params10, opt_plant, freq)
    
    gain_db_surr = 20.0 * math.log10(max(1e-6, sim_gain) / t_gain)
    phase_diff_surr = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
    
    b_true = g10["bode"][str(freq)]
    gain_db_true = b_true["gain_db"]
    phase_err_true = b_true["phase_err"]
    
    print(f"{freq:<10.3f} | {gain_db_true:>16.1f} dB | {gain_db_surr:>14.1f} dB | {phase_err_true:>16.1f}° | {abs(phase_diff_surr):>14.1f}°")
print("="*90)

