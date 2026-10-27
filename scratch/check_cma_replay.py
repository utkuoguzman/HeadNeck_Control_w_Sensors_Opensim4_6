import json, numpy as np, cma
import sys
sys.path.append('scripts')
from run_phase2_3_campaign import set_cma_bounds, lower_bounds, upper_bounds

gen3_lower_bounds = lower_bounds.copy()
gen3_lower_bounds[0] = 0.05
gen3_lower_bounds[3] = 0.001
gen3_lower_bounds[8] = 0.001
gen3_lower_bounds[9] = 5.0
gen3_upper_bounds = upper_bounds.copy()
gen3_upper_bounds[1] = 80.0
gen3_upper_bounds[7] = 5.00

gen4_lower_bounds = gen3_lower_bounds.copy()
gen4_lower_bounds[11] = 0.01
gen4_upper_bounds = gen3_upper_bounds.copy()

gen12_lower_bounds = gen4_lower_bounds.copy()
gen12_upper_bounds = gen4_upper_bounds.copy()
gen12_upper_bounds[10] = 1.50

gen14_lower_bounds = gen12_lower_bounds.copy()
gen14_upper_bounds = gen12_upper_bounds.copy()
gen14_upper_bounds[9] = 200.0

gen16_lower_bounds = gen14_lower_bounds.copy()
gen16_upper_bounds = gen14_upper_bounds.copy()
gen16_upper_bounds[1] = 120.0
gen16_upper_bounds[4] = 150.0

x0 = np.array([0.33, 40.0, 10.0, 0.09, 55.72, 3.81, 60.0, 3.0, 0.033, 15.0, 0.25, 0.40])
sigma0 = np.array([0.15, 5.00, 2.50, 0.05, 8.00, 1.20, 6.00, 0.35, 0.04, 6.00, 0.12, 0.15])

es = cma.CMAEvolutionStrategy(x0, 1.0, {
    'CMA_stds': sigma0,
    'bounds': [lower_bounds, upper_bounds],
    'popsize': 18,
    'maxiter': 30,
    'seed': 2026
})

for g in range(1, 17):
    summary_file = f'logs/phase2_3_gen_{g}_summary.json'
    with open(summary_file, 'r', encoding='utf-8') as f:
        sdata = json.load(f)
    if g == 3: set_cma_bounds(es, gen3_lower_bounds, gen3_upper_bounds)
    elif 4 <= g <= 11: set_cma_bounds(es, gen4_lower_bounds, gen4_upper_bounds)
    elif 12 <= g <= 13: set_cma_bounds(es, gen12_lower_bounds, gen12_upper_bounds)
    elif 14 <= g <= 15: set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
    elif g >= 16: set_cma_bounds(es, gen16_lower_bounds, gen16_upper_bounds)
    
    cands = es.ask()
    b_low, b_upp = es.boundary_handler.bounds
    clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]
    
    # Check best candidate in this generation
    b_idx = sdata['best_candidate_idx']
    cand_file = f'logs/candidates/W{b_idx}_P23G{g}.json'
    with open(cand_file) as fp:
        w_file = json.load(fp)
    
    # w_file params are [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, k_dyn, k_stat]
    r_ton_disk = w_file['params'][0] / w_file['params'][2]
    r_lead_sc_disk = w_file['params'][3] / w_file['params'][4]
    r_lead_ccr_disk = w_file['params'][8] / w_file['params'][9]
    cand_disk = [
        r_ton_disk, w_file['params'][1], w_file['params'][2],
        r_lead_sc_disk, w_file['params'][4], w_file['params'][5],
        w_file['params'][6], w_file['params'][7], r_lead_ccr_disk,
        w_file['params'][9], w_file['params'][10], w_file['params'][11]
    ]
    
    diff = np.max(np.abs(np.array(cand_disk) - np.array(clamped_replay[b_idx - 1])))
    print(f'Gen {g:2d}: Best Cand W{b_idx:2d} -> Max diff vs es.ask(): {diff:.4f}')
    
    es.tell(clamped_replay, sdata['all_scores'])

