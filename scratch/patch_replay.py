path = "scripts/run_phase2_4_campaign.py"
with open(path, "r", encoding="utf-8") as f:
    content = f.read()

# Replace replay block
old_block = """    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_4_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            gen_scores = sdata["all_scores"]
            pop_len = len(gen_scores)
            if es.popsize != pop_len:
                es.opts['popsize'] = pop_len
                es.sp.set(es.opts, pop_len)
            cands = es.ask()
            
            # Load exact saved candidate parameters if available
            saved_cands = []
            for k in range(pop_len):
                cf = f"logs/candidates/W{k+1}_P24G{g}.json"
                if os.path.exists(cf):
                    try:
                        with open(cf, "r") as fp:
                            saved_cands.append(json.load(fp)["params"])
                    except Exception:
                        pass
            if len(saved_cands) == pop_len:
                clamped_replay = saved_cands
            else:
                b_low, b_upp = es.boundary_handler.bounds
                clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]

            es.tell(clamped_replay, gen_scores)
            completed_gens = g
            
            # Populate archive
            for c_arr, s_val in zip(clamped_replay, gen_scores):
                archive_X.append(c_arr)
                archive_y.append(s_val)
                
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = clamped_replay[sdata["best_candidate_idx"] - 1]
                consecutive_non_improving = 0
            else:
                consecutive_non_improving += 1
            print(f"[REPLAY] Phase 2.4 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    # Switch to 14 candidates for Generation 4 onwards (1:1 core concurrency)
    NEW_POPSIZE = 14
    if es.popsize != NEW_POPSIZE:
        es.opts['popsize'] = NEW_POPSIZE
        es.sp.set(es.opts, NEW_POPSIZE)
    print(f"\\n[CMA-ES RECONFIGURATION] Confirmed population size at {NEW_POPSIZE} (mu={es.sp.mu}) with 2 TPA + 12 Orthogonal samples (100% 14-core synchronization)!")
    print(f"[ARCHIVE] Initialized with {len(archive_X)} real OpenSim historical evaluations.")"""

new_block = """    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_4_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            gen_scores = sdata["all_scores"]
            pop_len = len(gen_scores)
            if es.popsize != pop_len:
                es.opts['popsize'] = pop_len
                es.sp.set(es.opts, pop_len)
                
            if pop_len == 14:
                cands = sample_candidates_tpa_orthogonal(es, g, archive_X, archive_y)
            else:
                cands = es.ask()
                
            b_low, b_upp = es.boundary_handler.bounds
            clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]

            es.tell(clamped_replay, gen_scores)
            completed_gens = g
            
            # Populate archive
            for c_arr, s_val in zip(clamped_replay, gen_scores):
                archive_X.append(c_arr)
                archive_y.append(s_val)
                
            best_cand_rep = clamped_replay[sdata["best_candidate_idx"] - 1]
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = best_cand_rep.copy()
                consecutive_non_improving = 0
            else:
                consecutive_non_improving += 1
                
            # Replay dynamic boundary expansion if triggered
            new_low, new_upp, expansions = check_and_expand_bounds(best_cand_rep, b_low, b_upp)
            if expansions:
                set_cma_bounds(es, new_low, new_upp)
                
            print(f"[REPLAY] Phase 2.4 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    # Switch to 14 candidates for Generation 4 onwards (1:1 core concurrency)
    NEW_POPSIZE = 14
    if es.popsize != NEW_POPSIZE:
        es.opts['popsize'] = NEW_POPSIZE
        es.sp.set(es.opts, NEW_POPSIZE)
    print(f"\\n[CMA-ES RECONFIGURATION] Confirmed population size at {NEW_POPSIZE} (mu={es.sp.mu}) with 2 TPA + 12 Orthogonal samples (100% 14-core synchronization)!")
    print(f"[ARCHIVE] Initialized with {len(archive_X)} real OpenSim historical evaluations.")"""

old_norm = old_block.replace("\r\n", "\n")
content_norm = content.replace("\r\n", "\n")

if old_norm in content_norm:
    content_norm = content_norm.replace(old_norm, new_block.replace("\r\n", "\n"))
    with open(path, "w", encoding="utf-8") as f:
        f.write(content_norm)
    print("SUCCESS: Updated replay loop in scripts/run_phase2_4_campaign.py!")
else:
    print("ERROR: old_norm not found in content!")
