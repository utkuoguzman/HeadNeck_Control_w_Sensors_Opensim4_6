import os

file_path = 'cmaes_worker.py'
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

prefix = content.split('if __name__ == "__main__":')[0]

suffix = """if __name__ == "__main__":
    import sys
    import os
    if len(sys.argv) < 12:
        sys.exit(1)
        
    omega_n = float(sys.argv[1])
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
        worker_name = sys.argv[12] if len(sys.argv) >= 13 else f"W-{os.getpid()}"
        sim_gain, sim_phase = run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name)
        error = (sim_gain - target_gain)**2 + 0.5*(sim_phase - target_phase)**2
        
        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10
        
        final_score = error + penalty
        print(f"{final_score}")
        sys.exit(0)
    except Exception as e:
        import traceback
        traceback.print_exc(file=sys.stderr)
        print("100000.0")
        sys.exit(1)
"""

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(prefix + suffix)

print("Split and append successful!")
