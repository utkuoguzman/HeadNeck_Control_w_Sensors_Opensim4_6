import sys

with open("cmaes_worker_tf.py", "r", encoding="utf-8", errors="ignore") as f:
    code = f.read()

# Replace parameter reading
old_params = '''    omega_n = float(sys.argv[1])
    zeta = float(sys.argv[2])
    ki = float(sys.argv[3])
    g_ton = float(sys.argv[4])
    g_sc = float(sys.argv[5])
    g_phas = float(sys.argv[6])
    kp_prop = float(sys.argv[7])
    k_gamma_dyn = float(sys.argv[8])
    k_gamma_stat = float(sys.argv[9])
    
    freq = float(sys.argv[10])
    target_gain = float(sys.argv[11])
    target_phase = float(sys.argv[12])'''

new_params = '''    omega_n = float(sys.argv[1])
    zeta = float(sys.argv[2])
    ki = float(sys.argv[3])
    
    # TF Parameters (12 total)
    K_ton = float(sys.argv[4])
    z_ton = float(sys.argv[5])
    p_ton = float(sys.argv[6])
    
    K_sc = float(sys.argv[7])
    z_sc = float(sys.argv[8])
    p_sc = float(sys.argv[9])
    
    K_phas = float(sys.argv[10])
    z_phas = float(sys.argv[11])
    p_phas = float(sys.argv[12])
    
    kp_prop = float(sys.argv[13])
    k_gamma_dyn = float(sys.argv[14])
    k_gamma_stat = float(sys.argv[15])
    
    freq = float(sys.argv[16])
    target_gain = float(sys.argv[17])
    target_phase = float(sys.argv[18])'''

code = code.replace(old_params, new_params)

# Replace function signature and invocation
old_sig = "def run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name):"
new_sig = "def run_simulation(kp, ki, kd, K_ton, z_ton, p_ton, K_sc, z_sc, p_sc, K_phas, z_phas, p_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name):"
code = code.replace(old_sig, new_sig)

old_call = "sim_gain, sim_phase = run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name)"
new_call = "sim_gain, sim_phase = run_simulation(kp, ki, kd, K_ton, z_ton, p_ton, K_sc, z_sc, p_sc, K_phas, z_phas, p_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name)"
code = code.replace(old_call, new_call)

# Replace property setting
old_props = '''        controller.set_G_ton(g_ton)
        controller.set_G_sc(g_sc)
        controller.set_G_phas(g_phas)'''
new_props = '''        controller.setPropertyValue("K_ton", str(K_ton))
        controller.setPropertyValue("z_ton", str(z_ton))
        controller.setPropertyValue("p_ton", str(p_ton))
        controller.setPropertyValue("K_sc", str(K_sc))
        controller.setPropertyValue("z_sc", str(z_sc))
        controller.setPropertyValue("p_sc", str(p_sc))
        controller.setPropertyValue("K_phas", str(K_phas))
        controller.setPropertyValue("z_phas", str(z_phas))
        controller.setPropertyValue("p_phas", str(p_phas))'''
code = code.replace(old_props, new_props)

old_w = "worker_name = sys.argv[13] if len(sys.argv) >= 14 else"
new_w = "worker_name = sys.argv[19] if len(sys.argv) >= 20 else"
code = code.replace(old_w, new_w)
code = code.replace("if len(sys.argv) < 13:", "if len(sys.argv) < 19:")

with open("cmaes_worker_tf.py", "w", encoding="utf-8") as f:
    f.write(code)

print("Worker TF script generated")
