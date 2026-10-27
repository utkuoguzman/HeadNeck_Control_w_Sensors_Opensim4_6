import numpy as np

PHYSICAL_LOWER_BOUNDS = np.array([
    0.01,   # 0: r_ton (must be > 0)
    0.50,   # 1: p_ton (rad/s, must be > 0)
    0.05,   # 2: K_sc (Nm/(rad/s), must be > 0)
    0.0005, # 3: r_lead_sc (ratio, must be > 0)
    2.00,   # 4: p_sc (rad/s, must be > 0)
    0.05,   # 5: K_phas (Nm/(m/s^2), must be > 0)
    2.00,   # 6: p_phas (rad/s, must be > 0)
    0.05,   # 7: kp (must be > 0)
    0.0005, # 8: r_lead_ccr (ratio, must be > 0)
    1.00,   # 9: p_ccr (rad/s, must be > 0)
    0.01,   # 10: k_dyn (must be > 0)
    0.001   # 11: k_stat (must be > 0)
])

PHYSICAL_UPPER_BOUNDS = np.array([
    10.0,   # 0: r_ton
    300.0,  # 1: p_ton
    100.0,  # 2: K_sc
    0.99,   # 3: r_lead_sc (lead ratio must be < 1.0 for biological lead)
    300.0,  # 4: p_sc
    50.0,   # 5: K_phas
    200.0,  # 6: p_phas
    20.0,   # 7: kp
    0.99,   # 8: r_lead_ccr (lead ratio must be < 1.0)
    500.0,  # 9: p_ccr
    5.0,    # 10: k_dyn
    5.0     # 11: k_stat
])

PARAM_NAMES = [
    "r_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",
    "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat"
]

def check_and_expand_bounds(current_lower, current_upper, champ_params):
    new_lower = current_lower.copy().astype(float)
    new_upper = current_upper.copy().astype(float)
    expanded_details = []
    
    for i in range(len(champ_params)):
        val = float(champ_params[i])
        L = float(current_lower[i])
        U = float(current_upper[i])
        rng = U - L
        
        # Upper boundary check: difference < 5% of range
        diff_u = U - val
        if diff_u < 0.05 * rng:
            new_u = min(PHYSICAL_UPPER_BOUNDS[i], U + 0.10 * rng)
            if new_u > U + 1e-6:
                pct = (diff_u / rng) * 100.0
                expanded_details.append(f"{PARAM_NAMES[i]} close to UPPER ({diff_u:.2f} diff, {pct:.1f}% of range) -> extended from {U:.2f} to {new_u:.2f} (+10% old range {rng:.2f})")
                new_upper[i] = new_u
                
        # Lower boundary check: difference < 5% of range
        diff_l = val - L
        if diff_l < 0.05 * rng:
            new_l = max(PHYSICAL_LOWER_BOUNDS[i], L - 0.10 * rng)
            if new_l < L - 1e-6:
                pct = (diff_l / rng) * 100.0
                expanded_details.append(f"{PARAM_NAMES[i]} close to LOWER ({diff_l:.2f} diff, {pct:.1f}% of range) -> extended from {L:.2f} to {new_l:.2f} (-10% old range {rng:.2f})")
                new_lower[i] = new_l
                
    return new_lower, new_upper, expanded_details

# Test with W3_P23G15 params on Gen 14 bounds
import json
with open("logs/phase2_3_gen_15_summary.json") as f:
    s = json.load(f)
p = s["best_params"]
champ_vec = [
    p["r_ton"], p["p_ton"], p["K_sc"], p["r_lead_sc"], p["p_sc"], p["K_phas"],
    p["p_phas"], p["kp"], p["r_lead_ccr"], p["p_ccr"], p["k_dyn"], p["k_stat"]
]

# Gen 14 bounds:
gen14_lower = np.array([0.05, 2.0, 0.20, 0.001, 15.0, 0.20, 15.0, 0.15, 0.001, 5.0, 0.10, 0.01])
gen14_upper = np.array([2.50, 80.0, 30.0, 0.60, 100.0, 15.0, 80.0, 5.00, 0.65, 200.0, 1.50, 1.50])

nl, nu, details = check_and_expand_bounds(gen14_lower, gen14_upper, champ_vec)
print(f"Detected {len(details)} boundary extensions:")
for d in details:
    print("  *", d)

