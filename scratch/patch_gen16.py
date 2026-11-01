import re

with open('scripts/run_phase2_3_campaign.py', 'r', encoding='utf-8') as f:
    code = f.read()

# 1. Add gen16 bounds definition
target1 = """    # Phase 2.3 Generation 14+ Expanded Headroom Bounds (p_ccr extended up to 200.0 rad/s)
    gen14_lower_bounds = gen12_lower_bounds.copy()
    gen14_upper_bounds = gen12_upper_bounds.copy()
    gen14_upper_bounds[9] = 200.0  # 9: p_ccr expanded headroom up to 200.0 rad/s"""

repl1 = """    # Phase 2.3 Generation 14+ Expanded Headroom Bounds (p_ccr extended up to 200.0 rad/s)
    gen14_lower_bounds = gen12_lower_bounds.copy()
    gen14_upper_bounds = gen12_upper_bounds.copy()
    gen14_upper_bounds[9] = 200.0  # 9: p_ccr expanded headroom up to 200.0 rad/s

    # Phase 2.3 Generation 16+ Expanded Headroom Bounds (p_ton up to 120.0 rad/s, p_sc up to 150.0 rad/s)
    gen16_lower_bounds = gen14_lower_bounds.copy()
    gen16_upper_bounds = gen14_upper_bounds.copy()
    gen16_upper_bounds[1] = 120.0  # 1: p_ton extended up from 80.0 to 120.0 rad/s
    gen16_upper_bounds[4] = 150.0  # 4: p_sc extended up from 100.0 to 150.0 rad/s"""

code = code.replace(target1.replace('\r\n', '\n'), repl1.replace('\r\n', '\n'))
code = code.replace(target1.replace('\n', '\r\n'), repl1.replace('\n', '\r\n'))

# 2. Update replay loop bounds
target2 = """            elif g >= 14:
                set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)"""

repl2 = """            elif 14 <= g <= 15:
                set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
            elif g >= 16:
                set_cma_bounds(es, gen16_lower_bounds, gen16_upper_bounds)"""

code = code.replace(target2.replace('\r\n', '\n'), repl2.replace('\r\n', '\n'))
code = code.replace(target2.replace('\n', '\r\n'), repl2.replace('\n', '\r\n'))

# 3. Update main loop bounds
target3 = """        elif iteration >= 14:
            set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ccr in [5.0, 200.0] (expanded headroom), k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], leads in [0.001, ...]", flush=True)"""

repl3 = """        elif 14 <= iteration <= 15:
            set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ccr in [5.0, 200.0] (expanded headroom), k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], leads in [0.001, ...]", flush=True)
        elif iteration >= 16:
            set_cma_bounds(es, gen16_lower_bounds, gen16_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ton in [2.0, 120.0] (expanded headroom), p_sc in [15.0, 150.0] (expanded headroom), p_ccr in [5.0, 200.0], k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], kp in [0.15, 5.00]", flush=True)"""

code = code.replace(target3.replace('\r\n', '\n'), repl3.replace('\r\n', '\n'))
code = code.replace(target3.replace('\n', '\r\n'), repl3.replace('\n', '\r\n'))

# 4. Update active_lower, active_upper, and local perturbations
target4 = """            active_lower = gen14_lower_bounds
            active_upper = gen14_upper_bounds
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            
            # Local perturbations around both champions: W15_P23G10 (all-time) and W18_P23G14 (Gen 14)
            stds = active_upper - active_lower
            champ_local = np.random.normal(global_best_sol, stds * 0.04, size=(4000, 12))
            pccr_local = np.tile(global_best_sol, (4000, 1))
            pccr_local[:, 9] = np.random.uniform(20.0, 180.0, size=4000)
            extra_pool = [np.clip(c, active_lower, active_upper).tolist() for c in np.vstack([champ_local, pccr_local])]"""

repl4 = """            active_lower = gen16_lower_bounds if iteration >= 16 else gen14_lower_bounds
            active_upper = gen16_upper_bounds if iteration >= 16 else gen14_upper_bounds
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            
            # Local perturbations around all-time champions (W3_P23G15, W18_P23G15) + expanded headroom exploration
            stds = active_upper - active_lower
            champ_local = np.random.normal(global_best_sol, stds * 0.04, size=(4000, 12))
            headroom_local = np.tile(global_best_sol, (4000, 1))
            headroom_local[:, 1] = np.random.uniform(70.0, 120.0, size=4000)
            headroom_local[:, 4] = np.random.uniform(90.0, 150.0, size=4000)
            headroom_local[:, 9] = np.random.uniform(100.0, 195.0, size=4000)
            headroom_local += np.random.normal(0, stds * 0.03, size=(4000, 12))
            extra_pool = [np.clip(c, active_lower, active_upper).tolist() for c in np.vstack([champ_local, headroom_local])]"""

code = code.replace(target4.replace('\r\n', '\n'), repl4.replace('\r\n', '\n'))
code = code.replace(target4.replace('\n', '\r\n'), repl4.replace('\n', '\r\n'))

with open('scripts/run_phase2_3_campaign.py', 'w', encoding='utf-8') as f:
    f.write(code)

print("Applied Gen 16 bounds and configuration successfully!")

