with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    text = f.read()

target = """    # Phase 2.3 Generation 10+ Expanded Headroom Bounds (k_dyn extended up to 1.50)
    gen10_lower_bounds = gen4_lower_bounds.copy()
    gen10_upper_bounds = gen4_upper_bounds.copy()
    gen10_upper_bounds[10] = 1.50  # 10: k_dyn expanded headroom up to 1.50

    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_3_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            if g == 3:
                es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
                es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            elif 4 <= g <= 9:
                es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
                es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            elif g >= 10:
                es.boundary_handler.bounds = [gen10_lower_bounds, gen10_upper_bounds]
                es.opts['bounds'] = [gen10_lower_bounds, gen10_upper_bounds]"""

replacement = """    # Phase 2.3 Generation 12+ Expanded Headroom Bounds (k_dyn extended up to 1.50)
    gen12_lower_bounds = gen4_lower_bounds.copy()
    gen12_upper_bounds = gen4_upper_bounds.copy()
    gen12_upper_bounds[10] = 1.50  # 10: k_dyn expanded headroom up to 1.50

    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_3_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            if g == 3:
                es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
                es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            elif 4 <= g <= 11:
                es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
                es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            elif g >= 12:
                es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
                es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]"""

assert target in text, "target not found"
text = text.replace(target, replacement, 1)

target_loop = """        elif 4 <= iteration <= 9:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif iteration >= 10:
            es.boundary_handler.bounds = [gen10_lower_bounds, gen10_upper_bounds]
            es.opts['bounds'] = [gen10_lower_bounds, gen10_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)"""

replacement_loop = """        elif 4 <= iteration <= 11:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif iteration >= 12:
            es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
            es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)"""

assert target_loop in text, "target_loop not found"
text = text.replace(target_loop, replacement_loop, 1)

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Successfully updated scripts/run_phase2_3_campaign.py for Gen 12+ switch!")

