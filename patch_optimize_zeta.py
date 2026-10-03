import os

# --- 1. Update cmaes_worker.py ---
with open('cmaes_worker.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace the argument parsing block to include zeta
old_args = """    omega_n = float(sys.argv[1])
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
    
    zeta = 0.425
    I_eff = 1.0
    kp = I_eff * (omega_n ** 2)
    kd = 2.0 * zeta * I_eff * omega_n
    
    try:
        worker_name = sys.argv[12]"""

new_args = """    omega_n = float(sys.argv[1])
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
    target_phase = float(sys.argv[12])
    
    I_eff = 1.0
    kp = I_eff * (omega_n ** 2)
    kd = 2.0 * zeta * I_eff * omega_n
    
    try:
        worker_name = sys.argv[13]"""

# The number of arguments shifted
content = content.replace("len(sys.argv) < 12", "len(sys.argv) < 13")
content = content.replace("sys.argv[12] if len(sys.argv) >= 13", "sys.argv[13] if len(sys.argv) >= 14")
content = content.replace(old_args, new_args)
# Failsafe if zeta=0.8 or zeta=1.0 was in there
content = content.replace("zeta = 0.8", "")
content = content.replace("zeta = 1.0", "")

with open('cmaes_worker.py', 'w', encoding='utf-8') as f:
    f.write(content)


# --- 2. Update optimize_PID_cmaes_parallel.py ---
with open('optimize_PID_cmaes_parallel.py', 'r', encoding='utf-8') as f:
    opt_content = f.read()

# Replace param names
opt_content = opt_content.replace('["omega_n", "Ki", "G_ton"', '["omega_n", "zeta", "Ki", "G_ton"')
opt_content = opt_content.replace('omega_n, ki, g_ton', 'omega_n, zeta, ki, g_ton')

# Replace the subprocess call arguments
old_subprocess = """                    str(omega_n), str(ki),
                    str(g_ton), str(g_sc), str(g_phas),"""
new_subprocess = """                    str(omega_n), str(zeta), str(ki),
                    str(g_ton), str(g_sc), str(g_phas),"""
opt_content = opt_content.replace(old_subprocess, new_subprocess)

# Replace Initial Guess
opt_content = opt_content.replace("x0 = [9.0, 5.0, 11.2, 4.3, 2.6, 0.2, 58.7, 94.8]", "x0 = [9.0, 0.425, 5.0, 11.2, 4.3, 2.6, 0.2, 58.7, 94.8]")

# Replace bounds
old_lower = "[ 1.0,  0.0,   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ]"
new_lower = "[ 1.0,  0.1,  0.0,   5.0,  0.5,   2.0,    0.01,        20.0,         50.0  ]"
old_upper = "[20.0, 300.0,  25.0,  5.0,  10.0,    2.50,        80.0,        150.0  ]"
new_upper = "[20.0, 0.8, 300.0,  25.0,  5.0,  10.0,    2.50,        80.0,        150.0  ]"
opt_content = opt_content.replace(old_lower, new_lower)
opt_content = opt_content.replace(old_upper, new_upper)

# Replace popsize and CMA_stds
opt_content = opt_content.replace("'popsize': 14,", "'popsize': 15,")
opt_content = opt_content.replace("CMA_stds': [2.0, 10.0, 3.8", "CMA_stds': [2.0, 0.2, 10.0, 3.8")

# Replace output print
old_print = 'w_n={w_n:.2f} (Kp={kp:.1f},Kd={kd:.1f}), Ki={best_sol[1]:.1f}, G_ton={best_sol[2]:.3f}, G_sc={best_sol[3]:.3f}, G_phas={best_sol[4]:.3f}\\n"'
new_print = 'w_n={w_n:.2f}, zeta={best_sol[1]:.3f} (Kp={kp:.1f},Kd={kd:.1f}), Ki={best_sol[2]:.1f}, G_ton={best_sol[3]:.3f}, G_sc={best_sol[4]:.3f}, G_phas={best_sol[5]:.3f}\\n"'
opt_content = opt_content.replace(old_print, new_print)

# Replace file writes
old_file_write = """f.write(f"omega_n={best_sol[0]:.4f}, Ki={best_sol[1]:.4f}\\n")
                f.write(f"G_ton={best_sol[2]:.4f}, G_sc={best_sol[3]:.4f}, G_phas={best_sol[4]:.4f}\\n")
                f.write(f"kp_prop={best_sol[5]:.4f}, k_gamma_dyn={best_sol[6]:.4f}, k_gamma_stat={best_sol[7]:.4f}\\n")"""

new_file_write = """f.write(f"omega_n={best_sol[0]:.4f}, zeta={best_sol[1]:.4f}, Ki={best_sol[2]:.4f}\\n")
                f.write(f"G_ton={best_sol[3]:.4f}, G_sc={best_sol[4]:.4f}, G_phas={best_sol[5]:.4f}\\n")
                f.write(f"kp_prop={best_sol[6]:.4f}, k_gamma_dyn={best_sol[7]:.4f}, k_gamma_stat={best_sol[8]:.4f}\\n")"""
opt_content = opt_content.replace(old_file_write, new_file_write)

# Also fix the Kd calculation in the script summary
opt_content = opt_content.replace("kd = 2.0 * w_n", "kd = 2.0 * best_sol[1] * w_n")


with open('optimize_PID_cmaes_parallel.py', 'w', encoding='utf-8') as f:
    f.write(opt_content)

print("Upgraded to a 9-Dimensional Search (added zeta)!")
