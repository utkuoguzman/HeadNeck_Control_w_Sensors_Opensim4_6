"""
surrogacy/validate_in_opensim.py
Step 3 of the Differentiable Neuromotor Surrogate Pipeline.

Takes the optimal parameter solution discovered by the surrogate
(surrogacy/exact_plant_surrogate_solution.json) and validates it inside the
full non-linear HYOID OpenSim model across all 6 canonical frequencies.

Outputs:
    Prints full comparison table (Gain, Phase, Diff, Score per frequency).
"""

import json
import math
import subprocess
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
import pandas as pd

# Load Happee experimental target data
data_file = "extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv"
df = pd.read_csv(data_file)
targets = {round(r['Frequency_Hz'], 3): (r['Exp_Gain_deg_per_m'], r['Exp_Phase_deg']) for _, r in df.iterrows()}

sol_file = os.path.join(os.path.dirname(__file__), "exact_plant_surrogate_solution.json")
with open(sol_file) as f:
    sol = json.load(f)

p = sol["params"]
K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr = p
r_ton = sol["r_ton"]
r_lead_sc = sol["r_lead_sc"]
r_lead_ccr = sol["r_lead_ccr"]
k_dyn = sol["k_dyn"]
k_stat = sol["k_stat"]

freqs = [0.783, 1.099, 2.446, 5.583, 7.927, 0.324]

def evaluate_freq(freq):
    tg, tp = targets[freq]
    lbl = f"SURR_VAL_{freq:.2f}Hz"
    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        str(r_ton), str(p_ton), str(K_sc), str(r_lead_sc), str(p_sc),
        str(K_phas), str(p_phas), str(kp), str(r_lead_ccr), str(p_ccr),
        str(k_dyn), str(k_stat),
        str(freq), str(tg), str(tp),
        lbl
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=350)
        res_file = f"logs/candidates/{lbl}_{freq:.2f}Hz.json"
        if os.path.exists(res_file):
            with open(res_file) as fp:
                d = json.load(fp)
            return freq, d
        else:
            print(f"[FAIL] {freq:.3f}Hz: {res.stderr}")
            return freq, None
    except Exception as e:
        print(f"[ERROR] {freq:.3f}Hz: {e}")
        return freq, None

def main():
    print("="*115)
    print("VALIDATING EXACT PLANT SURROGATE SOLUTION IN FULL OPENSIM 4.6 MODEL")
    print("="*115)
    results = []
    with ThreadPoolExecutor(max_workers=6) as executor:
        futs = [executor.submit(evaluate_freq, f) for f in freqs]
        for fut in as_completed(futs):
            res = fut.result()
            results.append(res)
            
    results.sort(key=lambda x: x[0])
    print("\n" + "="*115)
    print(f"{'Freq [Hz]':<10} | {'Sim Gain':<10} | {'Tgt Gain':<10} | {'Gain Diff':<10} | {'Sim Phase':<10} | {'Tgt Phase':<10} | {'Phase Diff':<11} | {'Score':<8}")
    print("-" * 115)
    tot = 0.0
    for freq, d in results:
        if d:
            sg = d['sim_gain']
            tg = d['target_gain']
            sp = d['sim_phase']
            tp = d['target_phase']
            g_diff = d['gain_db_diff']
            p_diff = d['phase_err_deg']
            score = d['score']
            tot += score
            print(f"{freq:<10.3f} | {sg:<10.1f} | {tg:<10.1f} | {g_diff:+10.2f}dB | {sp:<10.1f}° | {tp:<10.1f}° | {p_diff:11.1f}° | {score:<8.1f}")
        else:
            print(f"{freq:<10.3f} | FAILED")
    print("-" * 115)
    print(f"TOTAL OPENSIM SCORE: {tot:.1f} pts")

if __name__ == '__main__':
    main()

