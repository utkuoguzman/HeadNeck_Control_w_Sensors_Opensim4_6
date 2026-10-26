with open('scripts/run_phase2_3_campaign.py', 'r', encoding='utf-8') as f:
    content = f.read()

target = '''    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_3_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            cands = es.ask()
            es.tell(cands, sdata["all_scores"])
            completed_gens = g
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = cands[sdata["best_candidate_idx"] - 1]
            print(f"[REPLAY] Phase 2.3 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break

    # Phase 2.3 Generation 3+ Expanded Headroom Bounds (User Approved)
    gen3_lower_bounds = np.array([
        0.05,   # 0: r_ton (extended down from 0.20)
        2.0,    # 1: p_ton (rad/s)
        0.20,   # 2: K_sc (Nm/(rad/s))
        0.001,  # 3: r_lead_sc (extended down from 0.01)
        15.0,   # 4: p_sc (rad/s)
        0.20,   # 5: K_phas (Nm/(m/s^2))
        15.0,   # 6: p_phas (rad/s)
        0.15,   # 7: kp (spindle stiffness gain)
        0.001,  # 8: r_lead_ccr (extended down from 0.01)
        5.0,    # 9: p_ccr (rad/s, extended down from 10.0)
        0.10,   # 10: k_dyn
        0.15    # 11: k_stat
    ])
    
    gen3_upper_bounds = np.array([
        2.50,   # 0: r_ton
        80.0,   # 1: p_ton (rad/s, extended up from 45.0)
        30.0,   # 2: K_sc
        0.60,   # 3: r_lead_sc
       100.0,   # 4: p_sc
        15.0,   # 5: K_phas
        80.0,   # 6: p_phas
        5.00,   # 7: kp (extended up from 3.50)
        0.65,   # 8: r_lead_ccr
        90.0,   # 9: p_ccr
        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])
            
    print(f"\\nPhase 2.3 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Target Benchmark to Beat: Happee (2017) Model Score = {HAPPEE_BENCHMARK:.2f}")
    print(f"Initial Candidate x0:\\n{x0.round(2)}")
    print(f"Sigma0:\\n{sigma0.round(2)}")
    
    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        
        # Apply expanded bounds from Generation 3 onward
        if iteration >= 3:
            es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
            es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]\", flush=True)
            
        print(f"\\n{'='*35} STARTING PHASE 2.3 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)'''

replacement = '''    # Phase 2.3 Generation 3 Expanded Headroom Bounds
    gen3_lower_bounds = np.array([
        0.05,   # 0: r_ton (extended down from 0.20)
        2.0,    # 1: p_ton (rad/s)
        0.20,   # 2: K_sc (Nm/(rad/s))
        0.001,  # 3: r_lead_sc (extended down from 0.01)
        15.0,   # 4: p_sc (rad/s)
        0.20,   # 5: K_phas (Nm/(m/s^2))
        15.0,   # 6: p_phas (rad/s)
        0.15,   # 7: kp (spindle stiffness gain)
        0.001,  # 8: r_lead_ccr (extended down from 0.01)
        5.0,    # 9: p_ccr (rad/s, extended down from 10.0)
        0.10,   # 10: k_dyn
        0.15    # 11: k_stat
    ])
    
    gen3_upper_bounds = np.array([
        2.50,   # 0: r_ton
        80.0,   # 1: p_ton (rad/s, extended up from 45.0)
        30.0,   # 2: K_sc
        0.60,   # 3: r_lead_sc
       100.0,   # 4: p_sc
        15.0,   # 5: K_phas
        80.0,   # 6: p_phas
        5.00,   # 7: kp (extended up from 3.50)
        0.65,   # 8: r_lead_ccr
        90.0,   # 9: p_ccr
        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    # Phase 2.3 Generation 4+ Expanded Headroom Bounds (k_stat extended down to 0.01)
    gen4_lower_bounds = np.array([
        0.05,   # 0: r_ton (extended down from 0.20)
        2.0,    # 1: p_ton (rad/s)
        0.20,   # 2: K_sc (Nm/(rad/s))
        0.001,  # 3: r_lead_sc (extended down from 0.01)
        15.0,   # 4: p_sc (rad/s)
        0.20,   # 5: K_phas (Nm/(m/s^2))
        15.0,   # 6: p_phas (rad/s)
        0.15,   # 7: kp (spindle stiffness gain)
        0.001,  # 8: r_lead_ccr (extended down from 0.01)
        5.0,    # 9: p_ccr (rad/s, extended down from 10.0)
        0.10,   # 10: k_dyn
        0.01    # 11: k_stat (extended down from 0.15 to 0.01 for full static gamma headroom)
    ])
    
    gen4_upper_bounds = np.array([
        2.50,   # 0: r_ton
        80.0,   # 1: p_ton (rad/s, extended up from 45.0)
        30.0,   # 2: K_sc
        0.60,   # 3: r_lead_sc
       100.0,   # 4: p_sc
        15.0,   # 5: K_phas
        80.0,   # 6: p_phas
        5.00,   # 7: kp (extended up from 3.50)
        0.65,   # 8: r_lead_ccr
        90.0,   # 9: p_ccr
        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_3_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            if g == 3:
                es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
                es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            elif g >= 4:
                es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
                es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            cands = es.ask()
            es.tell(cands, sdata["all_scores"])
            completed_gens = g
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = cands[sdata["best_candidate_idx"] - 1]
            print(f"[REPLAY] Phase 2.3 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    print(f"\\nPhase 2.3 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Target Benchmark to Beat: Happee (2017) Model Score = {HAPPEE_BENCHMARK:.2f}")
    print(f"Initial Candidate x0:\\n{x0.round(2)}")
    print(f"Sigma0:\\n{sigma0.round(2)}")
    
    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        
        # Apply expanded bounds for Generation 3 and Generation 4+
        if iteration == 3:
            es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
            es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation 3: r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]\", flush=True)
        elif iteration >= 4:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]\", flush=True)
            
        print(f"\\n{'='*35} STARTING PHASE 2.3 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)'''

assert target in content, 'Target string not found!'
new_content = content.replace(target, replacement, 1)
with open('scripts/run_phase2_3_campaign.py', 'w', encoding='utf-8') as f:
    f.write(new_content)
print('Successfully updated scripts/run_phase2_3_campaign.py with k_stat headroom!')

