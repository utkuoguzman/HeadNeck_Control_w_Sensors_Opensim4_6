import os

file_path = 'cmaes_worker.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

old_block = """    kp = float(sys.argv[1])
    ki = float(sys.argv[2])
    kd = float(sys.argv[3])
    g_ton = float(sys.argv[4])
    g_sc = float(sys.argv[5])
    g_phas = float(sys.argv[6])
    kp_prop = float(sys.argv[7])
    k_gamma_dyn = float(sys.argv[8])
    k_gamma_stat = float(sys.argv[9])
    
    freq = float(sys.argv[10])
    target_gain = float(sys.argv[11])
    target_phase = float(sys.argv[12])
    
    try:
        worker_name = sys.argv[13] if len(sys.argv) >= 14 else f"W-{os.getpid()}" """

new_block = """    omega_n = float(sys.argv[1])
    ki = float(sys.argv[2])
    g_ton = float(sys.argv[3])
    g_sc = float(sys.argv[4])
    g_phas = float(sys.argv[5])
    kp_prop = float(sys.argv[6])
    k_gamma_dyn = float(sys.argv[7])
    k_gamma_stat = float(sys.argv[8])
    
    freq = float(sys.argv[9])
    target_gain = float(sys.argv[10])
    target_phase = float(sys.argv[11])
    
    zeta = 1.0
    I_eff = 1.0
    kp = I_eff * (omega_n ** 2)
    kd = 2.0 * zeta * I_eff * omega_n
    
    try:
        worker_name = sys.argv[12] if len(sys.argv) >= 13 else f"W-{os.getpid()}" """

content = content.replace(old_block, new_block)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("Actually updated cmaes_worker.py!")
