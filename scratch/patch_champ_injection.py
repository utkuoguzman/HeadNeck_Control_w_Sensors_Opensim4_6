with open("scripts/run_phase2_3_campaign.py", "r", encoding="utf-8") as f:
    text = f.read().replace("\r\n", "\n")

old_pool = """            combined_pool = local_pool + global_pool"""

new_pool = """            # Inject targeted local perturbations directly around champion with p_ccr in [90, 200]
            stds = active_upper - active_lower
            champ_local = np.random.normal(global_best_sol, stds * 0.05, size=(4000, 12))
            pccr_local = np.tile(global_best_sol, (4000, 1))
            pccr_local[:, 9] = np.random.uniform(90.0, 200.0, size=4000)
            pccr_local += np.random.normal(0, stds * 0.03, size=(4000, 12))
            pccr_local[:, 9] = np.random.uniform(90.0, 200.0, size=4000)
            extra_pool = [np.clip(c, active_lower, active_upper).tolist() for c in np.vstack([champ_local, pccr_local])]
            
            combined_pool = local_pool + global_pool + extra_pool"""

assert old_pool in text, "old_pool not found"
text = text.replace(old_pool, new_pool)

with open("scripts/run_phase2_3_campaign.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Injected champion perturbations with p_ccr in [90, 200] into campaign pool successfully!")

