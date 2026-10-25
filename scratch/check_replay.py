import json
import numpy as np
import cma

lower_bounds = np.array([
    0.45,   # 0: r_ton
     8.0,   # 1: p_ton
    30.0,   # 2: K_sc
    0.01,   # 3: r_lead_sc
    25.0,   # 4: p_sc
     1.0,   # 5: K_phas
    25.0,   # 6: p_phas
    0.25,   # 7: kp
    0.10,   # 8: r_lead_ccr
    15.0,   # 9: p_ccr
    0.15,   # 10: k_dyn
    0.15    # 11: k_stat
])

upper_bounds = np.array([
    2.00,   # 0: r_ton
    40.0,   # 1: p_ton
    95.0,   # 2: K_sc
    0.55,   # 3: r_lead_sc
    90.0,   # 4: p_sc
    14.0,   # 5: K_phas
    85.0,   # 6: p_phas
    1.80,   # 7: kp
    0.55,   # 8: r_lead_ccr
    75.0,   # 9: p_ccr
    1.20,   # 10: k_dyn
    1.20    # 11: k_stat
])

x0 = np.array([
    0.910986, 17.414272, 38.176383, 0.265675, 69.691364, 5.036390,
    56.844082, 1.194967, 0.430164, 51.277029, 0.623851, 0.773309
])

sigma0 = np.array([
    0.07, 3.0, 6.0, 0.04, 7.0, 1.0, 6.0, 0.10, 0.04, 6.0, 0.08, 0.08
])

es = cma.CMAEvolutionStrategy(x0, 1.0, {
    'CMA_stds': sigma0,
    'bounds': [lower_bounds, upper_bounds],
    'popsize': 18,
    'maxiter': 30,
    'seed': 2026
})

for g in range(1, 12):
    if g == 10:
        lb_active = lower_bounds.copy()
        ub_active = upper_bounds.copy()
        lb_active[1] = 5.0
        ub_active[11] = 2.00
        es.boundary_handler = cma.boundary_handler.BoundTransform([lb_active, ub_active])
        es.opts['bounds'] = [lb_active, ub_active]
        print(f"--> Bound changed before asking Gen {g}")
        
    cands = es.ask()
    with open(f"logs/phase2_2_gen_{g}_summary.json") as f:
        sdata = json.load(f)
    es.tell(cands, sdata["all_scores"])
    
    # Check candidate 0 against disk
    cand0 = cands[0]
    # compute derived
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp_prop, r_lead_ccr, p_ccr, k_dyn, k_stat = cand0
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    sim_params = [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_dyn, k_stat]
    
    with open(f"logs/candidates/W1_P22G{g}.json") as f:
        disk_cand = json.load(f)["params"]
    
    max_err = max(abs(a - b) for a, b in zip(sim_params, disk_cand))
    print(f"Gen {g} candidate 0 match error: {max_err:.2e}")

