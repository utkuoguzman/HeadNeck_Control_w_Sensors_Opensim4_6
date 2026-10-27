with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    content = f.read()

old_block = """        if iteration >= 13:
            # Global Sobol (65,536) + Local CMA-ES (16,000) Quasi-Monte Carlo Pre-Screening Engine
            pool_t0 = time.perf_counter()
            local_pool = es.ask(number=16000)
            
            sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42 + iteration * 100)
            unit_sobol = sobol_sampler.random(65536)
            active_lower = gen12_lower_bounds if iteration >= 12 else (gen4_lower_bounds if iteration >= 4 else gen3_lower_bounds)
            active_upper = gen12_upper_bounds if iteration >= 12 else (gen4_upper_bounds if iteration >= 4 else gen3_upper_bounds)
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            
            combined_pool = local_pool + global_pool
            surr_scores = [surrogate_score(c) for c in combined_pool]
            ranked_indices = np.argsort(surr_scores)
            candidates = [combined_pool[i] for i in ranked_indices[:18]]
            
            pool_dt = time.perf_counter() - pool_t0
            print(f"[GLOBAL SOBOL PRE-SCREENING] Evaluated 81,536 candidates (65,536 Sobol + 16,000 CMA-ES) in {pool_dt:.2f} s ({len(combined_pool)/pool_dt:,.0f} evals/s). Best predicted score: {surr_scores[ranked_indices[0]]:.1f} pts (Worst discarded: {surr_scores[ranked_indices[-1]]:.1f} pts). Passing top 18 elite candidates to OpenSim.", flush=True)
        else:
            candidates = es.ask()"""

new_block = """        if iteration >= 14:
            # =========================================================================
            # HIERARCHICAL MULTI-FIDELITY PRE-SCREENING FUNNEL (TIER 1 + TIER 2)
            # =========================================================================
            pool_t0 = time.perf_counter()
            local_pool = es.ask(number=16000)
            
            sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42 + iteration * 100)
            unit_sobol = sobol_sampler.random(65536)
            active_lower = gen12_lower_bounds if iteration >= 12 else (gen4_lower_bounds if iteration >= 4 else gen3_lower_bounds)
            active_upper = gen12_upper_bounds if iteration >= 12 else (gen4_upper_bounds if iteration >= 4 else gen3_upper_bounds)
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            
            combined_pool = local_pool + global_pool
            
            # --- TIER 1: Fast Algebraic ROM Spline Pre-Screening (81,536 candidates) ---
            t1_t0 = time.perf_counter()
            t1_scores = [surrogate_score(c) for c in combined_pool]
            t1_ranked = np.argsort(t1_scores)
            top8k_cands = [combined_pool[i] for i in t1_ranked[:8000]]
            t1_dt = time.perf_counter() - t1_t0
            print(f"[TIER 1 PRE-SCREEN] Screened {len(combined_pool):,} candidates in {t1_dt:.2f} s ({len(combined_pool)/t1_dt:,.0f} evals/s)! Best algebraic score: {t1_scores[t1_ranked[0]]:.1f} pts. Selected top {len(top8k_cands):,} candidates for Tier 2.", flush=True)
            
            # --- TIER 2: Option A Non-Linear 2-DOF Musculoskeletal Time-Domain ODE Simulator ---
            t2_t0 = time.perf_counter()
            print(f"[TIER 2 PRE-SCREEN] Screening top {len(top8k_cands):,} candidates through Non-Linear Muscle ODE Simulator across 14 cores...", flush=True)
            with ProcessPoolExecutor(max_workers=14) as t2_executor:
                t2_scores = list(t2_executor.map(fast_eval_candidate_with_early_stopping, top8k_cands, chunksize=50))
            t2_scores = np.array(t2_scores)
            t2_ranked = np.argsort(t2_scores)
            raw_candidates = [top8k_cands[i] for i in t2_ranked[:18]]
            t2_dt = time.perf_counter() - t2_t0
            
            # Strictly clamp candidate phenotype to avoid CMA-ES boundary transformation errors
            candidates = [np.clip(c, active_lower, active_upper).tolist() for c in raw_candidates]
            
            pool_dt = time.perf_counter() - pool_t0
            print(f"[TIER 2 PRE-SCREEN] Screened 8,000 candidates in {t2_dt:.2f} s ({8000/t2_dt:,.0f} evals/s)! Best Tier 2 candidate score: {t2_scores[t2_ranked[0]]:.1f} pts. Total funnel time: {pool_dt:.2f} s. Passing top 18 elites to OpenSim.", flush=True)
        elif iteration == 13:
            pool_t0 = time.perf_counter()
            local_pool = es.ask(number=16000)
            sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42 + iteration * 100)
            unit_sobol = sobol_sampler.random(65536)
            active_lower = gen12_lower_bounds if iteration >= 12 else (gen4_lower_bounds if iteration >= 4 else gen3_lower_bounds)
            active_upper = gen12_upper_bounds if iteration >= 12 else (gen4_upper_bounds if iteration >= 4 else gen3_upper_bounds)
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            combined_pool = local_pool + global_pool
            surr_scores = [surrogate_score(c) for c in combined_pool]
            ranked_indices = np.argsort(surr_scores)
            candidates = [np.clip(combined_pool[i], active_lower, active_upper).tolist() for i in ranked_indices[:18]]
        else:
            candidates = es.ask()"""

# Normalize line endings
content_norm = content.replace("\r\n", "\n")
old_block_norm = old_block.replace("\r\n", "\n")
new_block_norm = new_block.replace("\r\n", "\n")

assert old_block_norm in content_norm, "Old block not found!"
content_norm = content_norm.replace(old_block_norm, new_block_norm)

# Also fix es.tell(candidates, all_scores) to es.tell(clamped_candidates, all_scores)
old_tell = "        es.tell(candidates, all_scores)"
new_tell = """        clamped_candidates = [np.clip(c, active_lower, active_upper).tolist() for c in candidates]
        es.tell(clamped_candidates, all_scores)"""
assert old_tell in content_norm, "Old tell not found!"
content_norm = content_norm.replace(old_tell, new_tell)

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(content_norm)

print("scripts/run_phase2_3_campaign.py patched successfully with Hierarchical Multi-Fidelity Pre-Screening Funnel!")

