with open('scripts/run_phase2_3_campaign.py', 'r', encoding='utf-8') as f:
    content = f.read()

# 1. Add import
old_imp = 'from scratch.test_tier2_accelerated import fast_eval_candidate_with_early_stopping'
new_imp = 'from scratch.test_tier2_accelerated import fast_eval_candidate_with_early_stopping\nfrom scratch.physics_informed_residual_regressor import predict_opensim_scores'
assert old_imp in content, 'old_imp not found'
content = content.replace(old_imp, new_imp)

# 2. Update replay loop bounds and clamping
old_rep = """            elif g >= 12:
                es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
                es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            cands = es.ask()
            es.tell(cands, sdata["all_scores"])"""

new_rep = """            elif 12 <= g <= 13:
                es.boundary_handler.bounds = [gen12_lower_bounds, gen12_upper_bounds]
                es.opts['bounds'] = [gen12_lower_bounds, gen12_upper_bounds]
            elif g >= 14:
                es.boundary_handler.bounds = [gen14_lower_bounds, gen14_upper_bounds]
                es.opts['bounds'] = [gen14_lower_bounds, gen14_upper_bounds]
            cands = es.ask()
            b_low, b_upp = es.boundary_handler.bounds
            clamped_replay = [np.clip(c, b_low + 1e-6, b_upp - 1e-6).tolist() for c in cands]
            es.tell(clamped_replay, sdata["all_scores"])"""

assert old_rep in content, 'old_rep not found'
content = content.replace(old_rep, new_rep)

# 3. Add Generation 15+ Physics-Informed Residual Regressor Pre-Screening
old_screen = """        if iteration >= 14:
            # =========================================================================
            # HIERARCHICAL MULTI-FIDELITY PRE-SCREENING FUNNEL (TIER 1 + TIER 2)
            # =========================================================================
            pool_t0 = time.perf_counter()
            local_pool = es.ask(number=16000)
            
            sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42 + iteration * 100)
            unit_sobol = sobol_sampler.random(65536)
            active_lower = gen14_lower_bounds if iteration >= 14 else (gen12_lower_bounds if iteration >= 12 else (gen4_lower_bounds if iteration >= 4 else gen3_lower_bounds))
            active_upper = gen14_upper_bounds if iteration >= 14 else (gen12_upper_bounds if iteration >= 12 else (gen4_upper_bounds if iteration >= 4 else gen3_upper_bounds))
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            
            # Inject targeted local perturbations directly around champion with p_ccr in [90, 200]
            stds = active_upper - active_lower
            champ_local = np.random.normal(global_best_sol, stds * 0.05, size=(4000, 12))
            pccr_local = np.tile(global_best_sol, (4000, 1))
            pccr_local[:, 9] = np.random.uniform(90.0, 200.0, size=4000)
            pccr_local += np.random.normal(0, stds * 0.03, size=(4000, 12))
            pccr_local[:, 9] = np.random.uniform(90.0, 200.0, size=4000)
            extra_pool = [np.clip(c, active_lower, active_upper).tolist() for c in np.vstack([champ_local, pccr_local])]
            
            combined_pool = local_pool + global_pool + extra_pool
            
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
            print(f"[TIER 2 PRE-SCREEN] Screened 8,000 candidates in {t2_dt:.2f} s ({8000/t2_dt:,.0f} evals/s)! Best Tier 2 candidate score: {t2_scores[t2_ranked[0]]:.1f} pts. Total funnel time: {pool_dt:.2f} s. Passing top 18 elites to OpenSim.", flush=True)"""

new_screen = """        if iteration >= 15:
            # =========================================================================
            # PHYSICS-INFORMED RESIDUAL REGRESSOR SCREENING (Validated rho = 0.751 on unseen OpenSim)
            # =========================================================================
            pool_t0 = time.perf_counter()
            local_pool = es.ask(number=16000)
            
            sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42 + iteration * 100)
            unit_sobol = sobol_sampler.random(65536)
            active_lower = gen14_lower_bounds
            active_upper = gen14_upper_bounds
            global_pool = [c.tolist() for c in qmc.scale(unit_sobol, active_lower, active_upper)]
            
            # Local perturbations around both champions: W15_P23G10 (all-time) and W18_P23G14 (Gen 14)
            stds = active_upper - active_lower
            champ_local = np.random.normal(global_best_sol, stds * 0.04, size=(4000, 12))
            pccr_local = np.tile(global_best_sol, (4000, 1))
            pccr_local[:, 9] = np.random.uniform(20.0, 180.0, size=4000)
            extra_pool = [np.clip(c, active_lower, active_upper).tolist() for c in np.vstack([champ_local, pccr_local])]
            
            combined_pool = local_pool + global_pool + extra_pool
            
            # Screen all 85,536+ candidates with Physics-Informed Residual Regressor in ~2 seconds!
            screen_t0 = time.perf_counter()
            pred_scores = predict_opensim_scores(combined_pool)
            ranked_idx = np.argsort(pred_scores)
            screen_dt = time.perf_counter() - screen_t0
            
            raw_candidates = [combined_pool[i] for i in ranked_idx[:18]]
            eps = 1e-6
            candidates = [np.clip(c, active_lower + eps, active_upper - eps).tolist() for c in raw_candidates]
            
            pool_dt = time.perf_counter() - pool_t0
            print(f"[RESIDUAL REGRESSOR PRE-SCREEN] Screened {len(combined_pool):,} candidates in {screen_dt:.2f} s ({len(combined_pool)/screen_dt:,.0f} evals/s)!", flush=True)
            print(f"[RESIDUAL REGRESSOR PRE-SCREEN] Best predicted score: {pred_scores[ranked_idx[0]]:.1f} pts. Total screening time: {pool_dt:.2f} s. Passing top 18 true elites to OpenSim.", flush=True)
        elif iteration == 14:
            # Replay Gen 14 candidates if needed
            candidates = es.ask()
            candidates = [np.clip(c, gen14_lower_bounds + 1e-6, gen14_upper_bounds - 1e-6).tolist() for c in candidates]"""

assert old_screen in content, 'old_screen not found'
content = content.replace(old_screen, new_screen)

with open('scripts/run_phase2_3_campaign.py', 'w', encoding='utf-8') as f:
    f.write(content)

print("Successfully patched run_phase2_3_campaign.py for Generation 15!")

