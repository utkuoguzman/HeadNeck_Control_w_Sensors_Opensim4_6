import subprocess
import sys
import json
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed

# Param order: r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat

test_candidates = {
    "C1_Current_G12": [1.062, 11.2, 45.4, 0.064, 72.9, 8.4, 31.0, 1.41, 0.538, 59.6, 0.93, 0.96],
    "C2_Ksc25_kp08":  [0.800, 15.0, 25.0, 0.250, 55.0, 6.0, 40.0, 0.80, 0.350, 45.0, 0.60, 0.60],
    "C3_Ksc18_kp05":  [0.800, 15.0, 18.0, 0.250, 50.0, 5.0, 40.0, 0.50, 0.350, 40.0, 0.50, 0.50],
    "C4_Ksc12_kp04":  [0.800, 15.0, 12.0, 0.250, 45.0, 4.0, 40.0, 0.40, 0.350, 35.0, 0.40, 0.40],
    "C5_Ksc08_kp03":  [0.700, 15.0,  8.0, 0.250, 40.0, 3.0, 35.0, 0.30, 0.350, 30.0, 0.30, 0.30],
    "C6_Ksc05_kp025": [0.600, 15.0,  5.0, 0.250, 35.0, 2.0, 30.0, 0.25, 0.350, 25.0, 0.25, 0.25],
    "C7_Ksc15_kp06":  [0.900, 15.0, 15.0, 0.300, 50.0, 5.0, 40.0, 0.60, 0.400, 45.0, 0.50, 0.50]
}

freq = 0.783
tg = 236.93
tp = -96.96

def run_worker(cand_name, params):
    cmd = [
        r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        *[str(v) for v in params],
        str(freq), str(tg), str(tp),
        f"TEST_{cand_name}"
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    res_file = f"logs/candidates/TEST_{cand_name}_{freq:.2f}Hz.json"
    try:
        with open(res_file) as fp:
            d = json.load(fp)
        return cand_name, d
    except Exception as e:
        return cand_name, {"error": str(e), "stdout": res.stdout, "stderr": res.stderr}

print(f"Launching {len(test_candidates)} candidates concurrently on {freq} Hz (Target Gain: {tg:.1f} deg/m, Phase: {tp:.1f}°)...")

results = {}
with ThreadPoolExecutor(max_workers=7) as executor:
    future_to_cand = {executor.submit(run_worker, name, p): name for name, p in test_candidates.items()}
    for future in as_completed(future_to_cand):
        name, res = future.result()
        results[name] = res
        if "error" in res:
            print(f"[FAIL] {name}: {res['error']}")
        else:
            print(f"[DONE] {name}: Sim Gain = {res['sim_gain']:6.1f} deg/m ({res['gain_db_diff']:+5.2f} dB) | Phase = {res['sim_phase']:6.1f}° (Err: {res['phase_err_deg']:4.1f}°) | Score = {res['score']:8.1f}")

