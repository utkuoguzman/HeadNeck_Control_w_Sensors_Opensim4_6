"""
surrogacy/validate_in_opensim.py
Step 3 of the Upgraded 15-Parameter, 6-DOF, 72-Muscle MIMO Differentiable Neuromotor Surrogate Pipeline.

Takes the optimal 15D parameter solution discovered by the 6-DOF, 72-muscle surrogate
(surrogacy/exact_plant_surrogate_solution.json) and validates it inside the full non-linear
HYOID OpenSim 4.6 model across all 6 canonical Happee (2017) frequencies in parallel.

Outputs:
    Prints full 6-DOF comparison table (Gain, Phase, Diff, Score, p1/p2 ranges, max|roll|, max|yaw|).
"""

import json
import math
import subprocess
import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
import pandas as pd

data_file = "extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv"
df = pd.read_csv(data_file)
targets = {round(r["Frequency_Hz"], 3): (r["Exp_Gain_deg_per_m"], r["Exp_Phase_deg"]) for _, r in df.iterrows()}

sol_file = os.path.join(os.path.dirname(__file__), "exact_plant_surrogate_solution.json")
with open(sol_file, "r") as f:
    sol = json.load(f)

p15 = sol["params_15d"]
K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat, Kp_task, Kd_task, Ki_task = p15

freqs = [0.783, 1.099, 2.446, 5.583, 7.927, 0.324]

def evaluate_freq(freq):
    tg, tp = targets[freq]
    lbl = "SURR6D_VAL"
    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        "1.8", "1.319", str(Ki_task),
        str(K_ton), str(p_ton), str(K_sc), str(r_lead_sc), str(p_sc),
        str(K_phas), str(p_phas), str(kp), str(r_lead_ccr), str(p_ccr),
        str(k_dyn), str(k_stat), str(Kp_task), str(Kd_task), str(Ki_task),
        str(freq), str(tg), str(tp),
        lbl
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        res_file = f"logs/candidates/{lbl}_{freq:.2f}Hz.json"
        if os.path.exists(res_file):
            with open(res_file, "r") as fp:
                d = json.load(fp)
            return freq, d
        else:
            print(f"[FAIL] {freq:.3f}Hz: {res.stderr}")
            return freq, None
    except Exception as e:
        print(f"[ERROR] {freq:.3f}Hz: {e}")
        return freq, None

def main():
    print("=" * 135)
    print("VALIDATING 15-PARAMETER, 6-DOF, 72-MUSCLE MIMO SURROGATE SOLUTION IN FULL OPENSIM 4.6 MODEL")
    print("=" * 135)
    results = []
    with ThreadPoolExecutor(max_workers=6) as executor:
        futs = [executor.submit(evaluate_freq, f) for f in freqs]
        for fut in as_completed(futs):
            results.append(fut.result())

    results.sort(key=lambda x: x[0])
    print("\n" + "=" * 135)
    print(f"{'Freq [Hz]':<10} | {'Sim Gain':<9} | {'Tgt Gain':<9} | {'Gain Diff':<9} | {'Sim Phase':<9} | {'Tgt Phase':<9} | {'Phase Err':<9} | {'Score':<8} | {'p1 [deg]':<12} | {'p2 [deg]':<13} | {'|r|/|y|':<11}")
    print("-" * 135)
    tot = 0.0
    for freq, d in results:
        if d:
            sg = d["sim_gain"]
            tg = d["target_gain"]
            sp = d["sim_phase"]
            tp = d["target_phase"]
            g_diff = d["gain_db_diff"]
            p_diff = d["phase_err_deg"]
            score = d["score"]
            tel = d.get("telemetry", {})
            p1_rng = f"[{tel.get('p1_min_deg',0):+.1f},{tel.get('p1_max_deg',0):+.1f}]"
            p2_rng = f"[{tel.get('p2_min_deg',0):+.1f},{tel.get('p2_max_deg',0):+.1f}]"
            ry_str = f"{tel.get('max_abs_roll_deg',0):.2f}/{tel.get('max_abs_yaw_deg',0):.2f}°"
            tot += score
            print(f"{freq:<10.3f} | {sg:<9.1f} | {tg:<9.1f} | {g_diff:+7.2f}dB | {sp:<8.1f}° | {tp:<8.1f}° | {p_diff:8.1f}° | {score:<8.1f} | {p1_rng:<12} | {p2_rng:<13} | {ry_str:<11}")
        else:
            print(f"{freq:<10.3f} | FAILED")
    print("-" * 135)
    print(f"TOTAL 6-FREQUENCY OPENSIM SCORE: {tot:.1f} pts")

if __name__ == "__main__":
    main()
