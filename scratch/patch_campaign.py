import re

with open('scripts/run_phase2_3_campaign.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Replace replay
old_replay = """            if g == 3:
                es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
                es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            elif 4 <= g <= 11:
                es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
                es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            elif 12 <= g <= 13:
                es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
                es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            elif g >= 14:
                es.boundary_handler.bounds = [gen14_lower_bounds, gen14_upper_bounds]
                es.opts['bounds'] = [gen14_lower_bounds, gen14_upper_bounds]"""

new_replay = """            if g == 3:
                set_cma_bounds(es, gen3_lower_bounds, gen3_upper_bounds)
            elif 4 <= g <= 11:
                set_cma_bounds(es, gen4_lower_bounds, gen4_upper_bounds)
            elif 12 <= g <= 13:
                set_cma_bounds(es, gen12_lower_bounds, gen12_upper_bounds)
            elif g >= 14:
                set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)"""

content = content.replace(old_replay.replace('\r\n', '\n'), new_replay.replace('\r\n', '\n'))
content = content.replace(old_replay.replace('\n', '\r\n'), new_replay.replace('\n', '\r\n'))

# Replace main loop
old_main = """        if iteration == 3:
            es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
            es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation 3: r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 4 <= iteration <= 11:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 12 <= iteration <= 13:
            es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
            es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif iteration >= 14:
            es.boundary_handler.bounds = [gen14_lower_bounds, gen14_upper_bounds]
            es.opts['bounds'] = [gen14_lower_bounds, gen14_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ccr in [5.0, 200.0] (expanded headroom), k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], leads in [0.001, ...]", flush=True)"""

new_main = """        if iteration == 3:
            set_cma_bounds(es, gen3_lower_bounds, gen3_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation 3: r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 4 <= iteration <= 11:
            set_cma_bounds(es, gen4_lower_bounds, gen4_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 12 <= iteration <= 13:
            set_cma_bounds(es, gen12_lower_bounds, gen12_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif iteration >= 14:
            set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ccr in [5.0, 200.0] (expanded headroom), k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], leads in [0.001, ...]", flush=True)"""

content = content.replace(old_main.replace('\r\n', '\n'), new_main.replace('\r\n', '\n'))
content = content.replace(old_main.replace('\n', '\r\n'), new_main.replace('\n', '\r\n'))

# Also replace tell clamping
content = content.replace('clamped_replay = [np.clip(c, b_low + 1e-6, b_upp - 1e-6).tolist() for c in cands]', 'clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]')
content = content.replace('clamped_candidates = [np.clip(c, active_lower + eps, active_upper - eps).tolist() for c in candidates]', 'b_low, b_upp = es.boundary_handler.bounds\n        clamped_candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in candidates]')

with open('scripts/run_phase2_3_campaign.py', 'w', encoding='utf-8') as f:
    f.write(content)

print('Updated scripts/run_phase2_3_campaign.py successfully!')

