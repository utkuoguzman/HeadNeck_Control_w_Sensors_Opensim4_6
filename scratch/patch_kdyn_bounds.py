with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    text = f.read()

target1 = """        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    for g in range(1, max_total_generations + 1):"""

replacement1 = """        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    # Phase 2.3 Generation 10+ Expanded Headroom Bounds (k_dyn extended up to 1.50)
    gen10_lower_bounds = gen4_lower_bounds.copy()
    gen10_upper_bounds = gen4_upper_bounds.copy()
    gen10_upper_bounds[10] = 1.50  # 10: k_dyn expanded headroom up to 1.50

    for g in range(1, max_total_generations + 1):"""

assert target1 in text, "target1 not found"
text = text.replace(target1, replacement1, 1)

target2 = """            elif g >= 4:
                es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
                es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]"""

replacement2 = """            elif 4 <= g <= 9:
                es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
                es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            elif g >= 10:
                es.boundary_handler.bounds = [gen10_lower_bounds, gen10_upper_bounds]
                es.opts['bounds'] = [gen10_lower_bounds, gen10_upper_bounds]"""

assert target2 in text, "target2 not found"
text = text.replace(target2, replacement2, 1)

target3 = """        elif iteration >= 4:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)"""

replacement3 = """        elif 4 <= iteration <= 9:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif iteration >= 10:
            es.boundary_handler.bounds = [gen10_lower_bounds, gen10_upper_bounds]
            es.opts['bounds'] = [gen10_lower_bounds, gen10_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)"""

assert target3 in text, "target3 not found"
text = text.replace(target3, replacement3, 1)

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Successfully patched run_phase2_3_campaign.py with k_dyn <= 1.50 headroom for Gen 10+!")

