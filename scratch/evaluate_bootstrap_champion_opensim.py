import os
import sys
import json
import math
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

# 1. Load the Surrogate Bootstrap Champion
champion_path = "logs/cmaes_surrogate_bootstrap_champion.json"
with open(champion_path, "r", encoding="utf-8") as f:
    champ_data = json.load(f)

p = champ_data["optimized_params"]
print("="*85)
print("Evaluating Surrogate Bootstrap Champion in Full High-Fidelity OpenSim:")
print(f"  Surrogate Predicted Score: {champ_data['optimized_surrogate_score']:.2f} pts (Target: < 2803.88)")
print("="*85)

TARGETS = [
    (0.324, 86.55,   -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

FIXED_W_N = 22.0
FIXED_ZETA = 0.707
FIXED_KI = 40.2786

candidate_vec = [
    p["r_ton"],
    p["p_ton"],
    p["K_sc"],
    p["r_lead_sc"],
    p["p_sc"],
    p["K_phas"],
    p["p_phas"],
    p["kp"],
    p["r_lead_ccr"],
    p["p_ccr"],
    p["k_dyn"],
    p["k_stat"]
]

def eval_single_freq(freq_idx):
    freq, t_gain, t_phase = TARGETS[freq_idx]
    worker_label = f"BOOTSTRAP_CHAMP_{freq:.2f}Hz"
    
    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
        *[str(v) for v in candidate_vec],
        str(freq), str(t_gain), str(t_phase),
        worker_label
    ]
    
    t0 = time.perf_counter()
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    dt = time.perf_counter() - t0
    
    if res.returncode == 0:
        lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
        score = float(lines[-1])
        
        # Read the detailed result json from logs/candidates/
        cand_dir = os.path.join("logs", "candidates")
        cand_file = os.path.join(cand_dir, f"{worker_label}_{freq:.2f}Hz.json")
        bode_detail = {}
        if os.path.exists(cand_file):
            with open(cand_file, "r", encoding="utf-8") as f_det:
                bode_detail = json.load(f_det)
                
        return freq, score, dt, bode_detail
    else:
        print(f"Error on {freq:.3f} Hz: {res.stderr}")
        return freq, 1000000.0, dt, {}

print("Launching 6 OpenSim frequencies in parallel on 6 CPU cores...")
t_all_start = time.perf_counter()

results = {}
with ThreadPoolExecutor(max_workers=6) as executor:
    futures = [executor.submit(eval_single_freq, i) for i in range(len(TARGETS))]
    for future in as_completed(futures):
        freq, score, dt, detail = future.result()
        results[freq] = (score, dt, detail)
        print(f"  [OpenSim {freq:5.3f} Hz] Finished in {dt:4.1f}s | Score: {score:8.2f} pts")

t_all_elapsed = time.perf_counter() - t_all_start
total_opensim_score = sum(r[0] for r in results.values())

print("\n" + "="*85)
print(f"OpenSim Parallel Evaluation Completed in {t_all_elapsed:.1f} seconds!")
print(f"Total True OpenSim Score: {total_opensim_score:.2f} pts (Target: < 2803.88)")
print("="*85)

print("\n--- Detailed True OpenSim Bode Matrix vs. Surrogate Prediction ---")
print(f"{'Freq (Hz)':<10} | {'OpenSim Gain':<14} | {'Surr Gain':<14} | {'OpenSim Phase':<16} | {'Surr Phase':<14} | {'Freq Score':<12}")
print("-"*85)

for freq, t_gain, t_phase in TARGETS:
    score, dt, detail = results[freq]
    surr_b = champ_data["bode_prediction"][str(freq)]
    
    sim_g = detail.get("sim_gain", 0.0)
    g_db = detail.get("gain_db_diff", 0.0)
    sim_p = detail.get("sim_phase", 0.0)
    p_err = detail.get("phase_err_deg", 0.0)
    
    print(f"{freq:<10.3f} | {g_db:>10.1f} dB  | {surr_b['gain_db']:>10.1f} dB | {p_err:>12.1f}°   | {surr_b['phase_err']:>10.1f}° | {score:>9.1f} pts")
print("="*85)

