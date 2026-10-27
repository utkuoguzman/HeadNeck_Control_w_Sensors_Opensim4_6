with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    content = f.read()

# 1. Add qmc import
if "from scipy.stats import qmc" not in content:
    content = content.replace("import cma\n", "import cma\nfrom scipy.stats import qmc\n")
    if "from scipy.stats import qmc" not in content:
        content = content.replace("import cma\r\n", "import cma\r\nfrom scipy.stats import qmc\r\n")

# 2. Replace sampling block
old_block_part = "        if iteration >= 13:"
end_block_part = "        all_scores = [0.0] * len(candidates)"

new_block = """        if iteration >= 13:
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
            candidates = es.ask()
        all_scores = [0.0] * len(candidates)"""

# Locate old block
idx_start = content.find(old_block_part)
idx_end = content.find(end_block_part, idx_start) + len(end_block_part)

if idx_start != -1 and idx_end != -1:
    content = content[:idx_start] + new_block + content[idx_end:]
    print("Replaced sampling block with 81,536 Sobol+CMA-ES engine!")
else:
    print("Could not find block boundaries!")

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(content)

print("Patch applied to scripts/run_phase2_3_campaign.py successfully!")

