import sys
import subprocess
import json

cmd = [
    r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
    "10.1", "1.98", "40.2786",
    # 12 params: r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat
    "1.178", "9.888", "48.658", "0.181", "85.910", "6.388", "43.473", "1.787", "0.513", "74.927", "0.519", "0.877",
    # freq, target_gain, target_phase, worker_label
    "0.783", "236.93", "-97.0", "TEST_WORKER_078"
]

print("Running test worker...")
res = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
print("Return code:", res.returncode)
print("STDOUT:", res.stdout)
print("STDERR:", res.stderr)

