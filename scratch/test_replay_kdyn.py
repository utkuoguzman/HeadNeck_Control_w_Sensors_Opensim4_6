import cma
import numpy as np
import json

def test_replay():
    lower_bounds = np.array([
        0.20,   # 0: r_ton (K_ton / K_sc)
         2.0,   # 1: p_ton (rad/s)
         0.20,  # 2: K_sc (Nm/(rad/s))
        0.01,   # 3: r_lead_sc (z_sc / p_sc)
        15.0,   # 4: p_sc (rad/s)
         0.20,  # 5: K_phas (Nm/(m/s^2))
        15.0,   # 6: p_phas (rad/s)
        0.15,   # 7: kp (spindle stiffness gain)
        0.01,   # 8: r_lead_ccr (z_ccr / p_ccr)
        10.0,   # 9: p_ccr (rad/s)
        0.10,   # 10: k_dyn
        0.15    # 11: k_stat
    ])

    upper_bounds = np.array([
        2.50,   # 0: r_ton
        45.0,   # 1: p_ton
        30.0,   # 2: K_sc
        0.60,   # 3: r_lead_sc
       100.0,   # 4: p_sc
        15.0,   # 5: K_phas
        80.0,   # 6: p_phas
        3.50,   # 7: kp
        0.65,   # 8: r_lead_ccr
        90.0,   # 9: p_ccr
        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    gen3_lower_bounds = np.array([
        0.05, 2.0, 0.20, 0.001, 15.0, 0.20, 15.0, 0.15, 0.001, 5.0, 0.10, 0.15
    ])
    gen3_upper_bounds = np.array([
        2.50, 80.0, 30.0, 0.60, 100.0, 15.0, 80.0, 5.00, 0.65, 90.0, 1.20, 1.50
    ])

    gen4_lower_bounds = np.array([
        0.05, 2.0, 0.20, 0.001, 15.0, 0.20, 15.0, 0.15, 0.001, 5.0, 0.10, 0.01
    ])
    gen4_upper_bounds = np.array([
        2.50, 80.0, 30.0, 0.60, 100.0, 15.0, 80.0, 5.00, 0.65, 90.0, 1.20, 1.50
    ])

    gen10_lower_bounds = gen4_lower_bounds.copy()
    gen10_upper_bounds = gen4_upper_bounds.copy()
    gen10_upper_bounds[10] = 1.50 # k_dyn headroom up to 1.50

    x0 = np.array([
        0.327, 40.0, 10.0, 0.092, 55.723, 3.806, 60.0, 3.0, 0.033, 15.0, 0.25, 0.40
    ])

    sigma0 = np.array([
        0.15, 5.00, 2.50, 0.05, 8.00, 1.20, 6.00, 0.35, 0.04, 6.00, 0.12, 0.15
    ])

    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': 30,
        'seed': 2026,
        'verb_log': 0,
        'verb_disp': 0
    })

    for g in range(1, 9):
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
        assert diff < 1e-6, f"Gen {g} replay mismatch: {diff}"

        es.tell(cands, sdata["all_scores"])

    print("Replay verified for Gen 1..8 with 0.00 discrepancy!")

if __name__ == "__main__":
    test_replay()

