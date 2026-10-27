import sys
sys.path.append("scratch")
from cervical_chadwick_blana_simulator import evaluate_chadwick_blana_candidate

# 1. 64k Sobol raw mathematical winner
cand_sobol_raw = [
    0.0983, 17.2357, 14.0616, 0.1691, 37.8375, 11.5286,
    8.2132, 1.9376, 0.2185, 189.9153, 1.2317, 1.0208
]

# 2. Rational First-Principles Biomechanical Candidate
# Derived from:
# - K_phas = 0.95 (cancels head inertial toppling m*h = 0.78 Nm/(m/s^2))
# - K_sc   = 0.55 (natural reflex damping ~0.5 Nms/rad, ~35% of critical damping)
# - p_ccr  = 35.0 (physiological motor unit firing bandwidth ~5 Hz, not 30 Hz)
# - p_phas = 4.0  (otolith translation high-pass cutoff ~0.6 Hz)
cand_rational = [
    0.55,    # r_ton
    25.0,    # p_ton
    0.55,    # K_sc
    0.20,    # r_lead_sc
    75.0,    # p_sc
    0.95,    # K_phas (first-principles inertial match!)
    4.0,     # p_phas
    1.25,    # kp
    0.08,    # r_lead_ccr
    35.0,    # p_ccr (true biological reflex cutoff!)
    0.72,    # k_dyn
    1.15     # k_stat
]

# 3. All-time Champion W3 (Which scored 1258 pts at 0.78 Hz in OpenSim)
cand_w3 = [
    0.5769, 76.4989, 0.5302, 0.2458, 96.0501, 4.2943,
    17.5282, 1.2839, 0.0687, 177.0660, 0.7189, 1.1773
]

print("Evaluating 64k Raw Sobol...")
score_sobol, det_sobol = evaluate_chadwick_blana_candidate(cand_sobol_raw)

print("Evaluating Rational First-Principles Candidate...")
score_rat, det_rat = evaluate_chadwick_blana_candidate(cand_rational)

print("Evaluating Champion W3...")
score_w3, det_w3 = evaluate_chadwick_blana_candidate(cand_w3)

print("\n" + "="*85)
print(f"{'Candidate':<28} | {'Total Score':<12} | {'0.78 Hz Gain':<13} | {'0.78 Hz Phase':<13} | {'1.10 Hz Gain':<13}")
print("-" * 85)
print(f"{'64k Raw Sobol (Over-stiff)':<28} | {score_sobol:<12.1f} | {det_sobol[0.783]['sim_gain']:<5.1f} ({det_sobol[0.783]['gain_db_diff']:>+4.1f} dB) | {det_sobol[0.783]['sim_phase']:>5.1f}° (Err:{det_sobol[0.783]['phase_err_deg']:>4.1f}°) | {det_sobol[1.099]['sim_gain']:<5.1f} ({det_sobol[1.099]['gain_db_diff']:>+4.1f} dB)")
print(f"{'Rational First-Principles':<28} | {score_rat:<12.1f} | {det_rat[0.783]['sim_gain']:<5.1f} ({det_rat[0.783]['gain_db_diff']:>+4.1f} dB) | {det_rat[0.783]['sim_phase']:>5.1f}° (Err:{det_rat[0.783]['phase_err_deg']:>4.1f}°) | {det_rat[1.099]['sim_gain']:<5.1f} ({det_rat[1.099]['gain_db_diff']:>+4.1f} dB)")
print(f"{'Champion W3':<28} | {score_w3:<12.1f} | {det_w3[0.783]['sim_gain']:<5.1f} ({det_w3[0.783]['gain_db_diff']:>+4.1f} dB) | {det_w3[0.783]['sim_phase']:>5.1f}° (Err:{det_w3[0.783]['phase_err_deg']:>4.1f}°) | {det_w3[1.099]['sim_gain']:<5.1f} ({det_w3[1.099]['gain_db_diff']:>+4.1f} dB)")
print("="*85)

