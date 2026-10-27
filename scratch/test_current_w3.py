import subprocess
import sys
import json

w3_params = [0.5769, 76.4989, 0.5302, 0.2458, 96.0501, 4.2943, 17.5282, 1.2839, 0.0687, 177.0660, 0.7189, 1.1773]
freq = 0.783
t_gain = 236.93
t_phase = -96.96

cmd = [
    sys.executable, 'cmaes_worker_p22.py',
    '22.0', '0.707', '40.2786',
    *[str(v) for v in w3_params],
    str(freq), str(t_gain), str(t_phase),
    'TEST_W3_CURRENT'
]
print("Running W3 on current model...")
res = subprocess.run(cmd, capture_output=True, text=True)
print("STDOUT:", res.stdout.strip())
if res.stderr:
    print("STDERR:", res.stderr.strip()[:400])

try:
    with open("logs/candidates/TEST_W3_CURRENT_0.78Hz.json", "r") as f:
        print("Result JSON:", f.read())
except Exception as e:
    print("Could not read result JSON:", e)

