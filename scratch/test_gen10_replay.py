import cma
import numpy as np
import json

def test_gen10_replay():
    lower_bounds = np.array([0.20, 2.0, 0.20, 0.01, 15.0, 0.20, 15.0, 0.15, 0.01, 10.0, 0.10, 0.15])
    upper_bounds = np.array([2.50, 45.0, 30.0, 0.60, 100.0, 15.0, 80.0, 3.50, 0.65, 90.0, 1.20, 1.50])
    
    gen3_lower_bounds = np.array([0.05, 2.0, 0.20, 0.001, 15.0, 0.20, 15.0, 0.15, 0.001, 5.0, 0.10, 0.15])
    gen3_upper_bounds = np.array([2.50, 80.0, 30.0, 0.60, 100.0, 15.0, 80.0, 5.00, 0.65, 90.0, 1.20, 1.50])
    
    gen4_lower_bounds = np.array([0.05, 2.0, 0.20, 0.001, 15.0, 0.20, 15.0, 0.15, 0.001, 5.0, 0.10, 0.01])
    gen4_upper_bounds = np.array([2.50, 80.0, 30.0, 0.60, 100.0, 15.0, 80.0, 5.00, 0.65, 90.0, 1.20, 1.50])

    x0 = np.array([0.327, 40.0, 10.0, 0.092, 55.723, 3.806, 60.0, 3.0, 0.033, 15.0, 0.25, 0.40])
    sigma0 = np.array([0.15, 5.00, 2.50, 0.05, 8.00, 1.20, 6.00, 0.35, 0.04, 6.00, 0.12, 0.15])

    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': 30,
        'seed': 2026,
        'verb_log': 0,
        'verb_disp': 0
    })

    for g in range(1, 11):
        summary_file = f"logs/phase2_3_gen_{g}_summary.json"
        with open(summary_file, 'r', encoding='utf-8') as f:
            sdata = json.load(f)

        if g == 3:
            es.boundary_handler.bounds = [gen3_lower_bounds, gen3_upper_bounds]
            es.opts['bounds'] = [gen3_lower_bounds, gen3_upper_bounds]
        elif g >= 4:
            es.boundary_handler.bounds = [gen4_lower_bounds, gen4_upper_bounds]
            es.opts['bounds'] = [gen4_lower_bounds, gen4_upper_bounds]

        cands = es.ask()
        best_cand = cands[sdata["best_candidate_idx"] - 1]
        reported_best_r_ton = sdata["best_params"]["r_ton"]
        diff = abs(best_cand[0] - reported_best_r_ton)
        print(f"Gen {g} replay diff for r_ton: {diff:.2e}")
        es.tell(cands, sdata["all_scores"])

    print("Replayed up to Gen 10 with gen4 bounds (k_dyn <= 1.20) successfully!")

if __name__ == "__main__":
    test_gen10_replay()

