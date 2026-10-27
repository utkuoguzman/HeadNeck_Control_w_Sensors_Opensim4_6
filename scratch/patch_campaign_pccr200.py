with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    text = f.read()

# Normalize line endings
text = text.replace("\r\n", "\n")

old_bounds_def = """    # Phase 2.3 Generation 12+ Expanded Headroom Bounds (k_dyn extended up to 1.50)
    gen12_lower_bounds = gen4_lower_bounds.copy()
    gen12_upper_bounds = gen4_upper_bounds.copy()
    gen12_upper_bounds[10] = 1.50  # 10: k_dyn expanded headroom up to 1.50"""

new_bounds_def = """    # Phase 2.3 Generation 12+ Expanded Headroom Bounds (k_dyn extended up to 1.50)
    gen12_lower_bounds = gen4_lower_bounds.copy()
    gen12_upper_bounds = gen4_upper_bounds.copy()
    gen12_upper_bounds[10] = 1.50  # 10: k_dyn expanded headroom up to 1.50

    # Phase 2.3 Generation 14+ Expanded Headroom Bounds (p_ccr extended up to 200.0 rad/s)
    gen14_lower_bounds = gen12_lower_bounds.copy()
    gen14_upper_bounds = gen12_upper_bounds.copy()
    gen14_upper_bounds[9] = 200.0  # 9: p_ccr expanded headroom up to 200.0 rad/s"""

assert old_bounds_def in text, "old_bounds_def not found"
text = text.replace(old_bounds_def, new_bounds_def)

old_bounds_branch = """        elif iteration >= 12:
            es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
            es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)"""

new_bounds_branch = """        elif 12 <= iteration <= 13:
            es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
            es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif iteration >= 14:
            es.boundary_handler.bounds = [gen14_lower_bounds, gen14_upper_bounds]
            es.opts['bounds'] = [gen14_lower_bounds, gen14_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ccr in [5.0, 200.0] (expanded headroom), k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], leads in [0.001, ...]", flush=True)"""

assert old_bounds_branch in text, "old_bounds_branch not found"
text = text.replace(old_bounds_branch, new_bounds_branch)

# Update active_lower / active_upper in prescreening
old_active_bounds = """            active_lower = gen12_lower_bounds if iteration >= 12 else (gen4_lower_bounds if iteration >= 4 else gen3_lower_bounds)
            active_upper = gen12_upper_bounds if iteration >= 12 else (gen4_upper_bounds if iteration >= 4 else gen3_upper_bounds)"""

new_active_bounds = """            active_lower = gen14_lower_bounds if iteration >= 14 else (gen12_lower_bounds if iteration >= 12 else (gen4_lower_bounds if iteration >= 4 else gen3_lower_bounds))
            active_upper = gen14_upper_bounds if iteration >= 14 else (gen12_upper_bounds if iteration >= 12 else (gen4_upper_bounds if iteration >= 4 else gen3_upper_bounds))"""

assert old_active_bounds in text, "old_active_bounds not found"
text = text.replace(old_active_bounds, new_active_bounds)

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(text)

print("scripts/run_phase2_3_campaign.py successfully patched with p_ccr <= 200 rad/s!")

