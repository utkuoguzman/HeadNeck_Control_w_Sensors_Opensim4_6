from pathlib import Path

p = Path('optimize_6_independent.py')
content = p.read_text(encoding='utf-8')

old_block = """    try:
        result = subprocess.run(
            [
                sys.executable, "cmaes_worker.py",
                str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
                str(g_ton), str(g_sc), str(g_phas),
                str(kp_prop), str(k_gamma_dyn), str(k_gamma_stat),
                str(freq), str(t_gain), str(t_phase),
                f"F{freq_idx+1}-W{w_idx+1}"
            ],
            stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=3600
        )"""

replacement = """    target_sim_time = 0.5 + 2.0 / freq
    dynamic_timeout = int(target_sim_time * 240 * 1.5) # 240x real-time + 50% buffer

    try:
        result = subprocess.run(
            [
                sys.executable, "cmaes_worker.py",
                str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
                str(g_ton), str(g_sc), str(g_phas),
                str(kp_prop), str(k_gamma_dyn), str(k_gamma_stat),
                str(freq), str(t_gain), str(t_phase),
                f"F{freq_idx+1}-W{w_idx+1}"
            ],
            stdout=subprocess.PIPE, stderr=sys.stderr, text=True, timeout=dynamic_timeout
        )"""

content = content.replace(old_block, replacement)

# If it had 600 instead of 3600:
old_block_600 = old_block.replace("timeout=3600", "timeout=600")
content = content.replace(old_block_600, replacement)

p.write_text(content, encoding='utf-8')
print("Dynamic timeouts injected!")
