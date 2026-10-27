with open('scripts/run_phase2_3_campaign.py', 'r', encoding='utf-8') as f:
    code = f.read()

# Replace lines around iteration screening with pure CMA-ES
target = """        if iteration >= 15:
            # =========================================================================
            # PHYSICS-INFORMED RESIDUAL REGRESSOR SCREENING (Validated rho = 0.751 on unseen OpenSim)
            # =========================================================================
            pool_t0 = time.perf_counter()
            local_pool = es.ask(number=16000)
            
            sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42 + iteration * 100)
            unit_sobol = sobol_sampler.random(65536)
            active_lower = gen16_lower_bounds if iteration >= 16 else gen14_lower_bounds
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
            extra_pool = [np.clip(c, active_lower, active_upper).tolist() for c in np.vstack([champ_local, headroom_local])]
            
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
            candidates = [np.clip(c, gen14_lower_bounds + 1e-6, gen14_upper_bounds - 1e-6).tolist() for c in candidates]
        elif iteration == 13:
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
            surr_scores = [surrogate_score(c) for c in combined_pool]
            ranked_indices = np.argsort(surr_scores)
            candidates = [np.clip(combined_pool[i], active_lower, active_upper).tolist() for i in ranked_indices[:18]]
        else:
            candidates = es.ask()"""

replacement = """        # Pure Covariance Matrix Adaptation (CMA-ES)
        print(f"[CMA-ES SAMPLING] Sampling population of 18 directly from multivariate normal distribution N(mu, C)...", flush=True)
        raw_cands = es.ask()
        b_low, b_upp = es.boundary_handler.bounds
        candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in raw_cands]"""

code = code.replace(target.replace('\r\n', '\n'), replacement.replace('\r\n', '\n'))
code = code.replace(target.replace('\n', '\r\n'), replacement.replace('\n', '\r\n'))

with open('scripts/run_phase2_3_campaign.py', 'w', encoding='utf-8') as f:
    f.write(code)

print("Configured pure CMA-ES successfully!")

