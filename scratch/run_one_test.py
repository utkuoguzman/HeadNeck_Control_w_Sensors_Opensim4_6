import subprocess
import json

cmd = [
    r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
    "22.0", "0.707", "40.2786",
    # r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat
    "0.8", "15.0", "18.0", "0.25", "50.0", "5.0", "40.0", "0.5", "0.35", "40.0", "0.5", "0.5",
    "0.783", "236.93", "-96.96",
    "TEST_C3_Ksc18_kp05"
]

print("Running C3 (K_sc=18, kp=0.50)...")
res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
print("Returncode:", res.returncode)
print("STDOUT:", res.stdout)
print("STDERR:", res.stderr)

try:
    with open("logs/candidates/TEST_C3_Ksc18_kp05_0.78Hz.json") as fp:
        d = json.load(fp)
    print("\n--- RESULTS FOR C3 (K_sc=18, kp=0.50) ---")
    print(f"Sim Gain: {d['sim_gain']:.2f} deg/m (Target: {d['target_gain']:.2f}) | Gain Error: {d['gain_db_diff']:+.2f} dB")
    print(f"Sim Phase: {d['sim_phase']:.2f}° (Target: {d['target_phase']:.2f}°) | Phase Error: {d['phase_err_deg']:.2f}°")
    print(f"Total Score: {d['score']:.2f}")
except Exception as e:
    print("Error reading json:", e)

