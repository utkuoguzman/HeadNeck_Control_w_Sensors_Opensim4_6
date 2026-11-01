import subprocess
import json

cmd = [
    r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
    "22.0", "0.707", "40.2786",
    # r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat
    "1.5", "15.0", "2.0", "0.20", "45.0", "1.0", "40.0", "0.65", "0.30", "45.0", "0.30", "0.30",
    "0.783", "236.93", "-96.96",
    "TEST_HAPPEE_SCALE"
]

print("Running Happee Scale Test on 0.783 Hz (K_sc=2.0, K_ton=3.0, kp=0.65)...")
res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
print("Returncode:", res.returncode)
print("STDOUT:", res.stdout)
print("STDERR:", res.stderr)

try:
    with open("logs/candidates/TEST_HAPPEE_SCALE_0.78Hz.json") as fp:
        d = json.load(fp)
    print("\n--- RESULTS FOR HAPPEE SCALE CANDIDATE ---")
    print(f"Sim Gain: {d['sim_gain']:.2f} deg/m (Target: {d['target_gain']:.2f}) | Gain Error: {d['gain_db_diff']:+.2f} dB")
    print(f"Sim Phase: {d['sim_phase']:.2f}° (Target: {d['target_phase']:.2f}°) | Phase Error: {d['phase_err_deg']:.2f}°")
    print(f"Total Score: {d['score']:.2f} (Gain Loss: {d['gain_loss']:.1f}, Phase Loss: {d['phase_loss']:.1f})")
except Exception as e:
    print("Error reading json:", e)

