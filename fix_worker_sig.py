import os
with open("cmaes_worker_tf.py", "r", encoding="utf-8") as f:
    code = f.read()

# The old signature was: def run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, frequency, worker_name="W-?"):
old_sig = 'def run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, frequency, worker_name="W-?"):\\n'
new_sig = 'def run_simulation(kp, ki, kd, K_ton, z_ton, p_ton, K_sc, z_sc, p_sc, K_phas, z_phas, p_phas, kp_prop, k_gamma_dyn, k_gamma_stat, frequency, worker_name="W-?"):\\n'

import re
code = re.sub(r'def run_simulation\(.*?\):', 'def run_simulation(kp, ki, kd, K_ton, z_ton, p_ton, K_sc, z_sc, p_sc, K_phas, z_phas, p_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name="W-?"):', code)

# Note: The bottom call uses 'freq' instead of 'frequency', but my regex replaced the param to 'freq' so it should match!
with open("cmaes_worker_tf.py", "w", encoding="utf-8") as f:
    f.write(code)
print("Fixed signature")

