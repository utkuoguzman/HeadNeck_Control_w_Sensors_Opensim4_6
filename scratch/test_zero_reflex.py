import subprocess
import json

cmd = [
    r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
    "22.0", "0.707", "40.2786",
    # r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat
    "0.0", "15.0", "0.0", "0.25", "50.0", "0.0", "40.0", "0.0", "0.35", "40.0", "0.0", "0.0",
    "0.783", "236.93", "-96.96",
    "TEST_ZERO_REFLEX"
]

print("Running Zero Reflex Test...")
res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
try:
    with open("logs/candidates/TEST_ZERO_REFLEX_0.78Hz.json") as fp:
        d = json.load(fp)
    print("\n--- RESULTS FOR ZERO REFLEX ---")
    print(f"Sim Gain: {d['sim_gain']:.2f} deg/m | Sim Phase: {d['sim_phase']:.2f}°")
except Exception as e:
    print("Error reading json:", e, res.stdout, res.stderr)

