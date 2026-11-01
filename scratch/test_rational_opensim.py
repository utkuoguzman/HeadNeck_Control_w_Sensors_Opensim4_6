import subprocess
import sys
import json

# Rational candidate
# [r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat]
cand_rational = [
    0.55, 25.0, 0.55, 0.20, 75.0, 0.95, 4.0, 1.25, 0.08, 35.0, 0.72, 1.15
]

freq = 0.783
t_gain = 236.93
t_phase = -96.96

cmd = [
    sys.executable, 'cmaes_worker_p22.py',
    '22.0', '0.707', '40.2786',
    *[str(v) for v in cand_rational],
    str(freq), str(t_gain), str(t_phase),
    'TEST_RATIONAL'
]

print("Running Rational Candidate in OpenSim at 0.783 Hz...")
res = subprocess.run(cmd, capture_output=True, text=True)
print("STDOUT:", res.stdout.strip())
if res.stderr:
    print("STDERR:", res.stderr.strip()[:300])

try:
    with open("logs/candidates/TEST_RATIONAL_0.78Hz.json", "r") as f:
        print("Result JSON:", f.read())
except Exception as e:
    print("Could not read result JSON:", e)

