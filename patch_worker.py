import os
import re

file_path = 'cmaes_worker.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Replace the argument parsing block
old_args = r"""    kp = float\(sys.argv\[1\]\)
    ki = float\(sys.argv\[2\]\)
    kd = float\(sys.argv\[3\]\)
    g_ton = float\(sys.argv\[4\]\)
    g_sc = float\(sys.argv\[5\]\)
    g_phas = float\(sys.argv\[6\]\)
    kp_prop = float\(sys.argv\[7\]\)
    k_gamma_dyn = float\(sys.argv\[8\]\)
    k_gamma_stat = float\(sys.argv\[9\]\)
    frequency = float\(sys.argv\[10\]\)
    target_gain = float\(sys.argv\[11\]\)
    target_phase = float\(sys.argv\[12\]\)
    worker_id = sys.argv\[13\].*"""

new_args = """    omega_n = float(sys.argv[1])
    ki = float(sys.argv[2])
    g_ton = float(sys.argv[3])
    g_sc = float(sys.argv[4])
    g_phas = float(sys.argv[5])
    kp_prop = float(sys.argv[6])
    k_gamma_dyn = float(sys.argv[7])
    k_gamma_stat = float(sys.argv[8])
    frequency = float(sys.argv[9])
    target_gain = float(sys.argv[10])
    target_phase = float(sys.argv[11])
    worker_id = sys.argv[12] if len(sys.argv) > 12 else "W0"

    # Bandwidth mapping
    zeta = 1.0 # Critically damped
    I_eff = 1.0 # Base normalized inertia
    kp = I_eff * (omega_n ** 2)
    kd = 2.0 * zeta * I_eff * omega_n"""

# Also fix the len check
content = re.sub(r"if len\(sys\.argv\) < 13:", "if len(sys.argv) < 12:", content)
content = re.sub(old_args, new_args, content)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Updated cmaes_worker.py for bandwidth arguments.")
