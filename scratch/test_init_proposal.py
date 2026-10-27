import numpy as np

# W3_P23G15 parameters
x0 = np.array([
    0.5769298,   # 0: r_ton
    76.4989490,  # 1: p_ton
    0.5301718,   # 2: K_sc
    0.2457791,   # 3: r_lead_sc
    96.0500752,  # 4: p_sc
    4.2943242,   # 5: K_phas
    17.5282154,  # 6: p_phas
    1.2838733,   # 7: kp
    0.0687314,   # 8: r_lead_ccr
    177.0659882, # 9: p_ccr
    0.7189289,   # 10: k_dyn
    1.1772806    # 11: k_stat
])

param_names = [
    'r_ton', 'p_ton', 'K_sc', 'r_lead_sc', 'p_sc', 'K_phas',
    'p_phas', 'kp', 'r_lead_ccr', 'p_ccr', 'k_dyn', 'k_stat'
]

lower = np.array([
    0.05,    # r_ton
    2.0,     # p_ton
    0.05,    # K_sc (extended down from 0.20 for compliance headroom)
    0.001,   # r_lead_sc
    15.0,    # p_sc
    0.20,    # K_phas
    8.0,     # p_phas (extended down from 15.0 for phase lag headroom)
    0.15,    # kp
    0.001,   # r_lead_ccr
    5.0,     # p_ccr
    0.10,    # k_dyn
    0.01     # k_stat
])

upper = np.array([
    2.50,    # r_ton
    120.0,   # p_ton
    15.0,    # K_sc
    0.60,    # r_lead_sc
    150.0,   # p_sc
    15.0,    # K_phas
    80.0,    # p_phas
    5.00,    # kp
    0.65,    # r_lead_ccr
    220.0,   # p_ccr (extended to 220 to allow headroom above 177.0)
    1.50,    # k_dyn
    1.50     # k_stat
])

# Healthy sigmas
sigmas = np.array([
    0.08,    # r_ton
    6.0,     # p_ton
    0.12,    # K_sc
    0.03,    # r_lead_sc
    8.0,     # p_sc
    0.60,    # K_phas
    2.50,    # p_phas
    0.20,    # kp
    0.015,   # r_lead_ccr
    12.0,    # p_ccr
    0.08,    # k_dyn
    0.10     # k_stat
])

print('%-12s | %-10s | %-16s | %-8s | %-24s' % ('Parameter', 'x0 (W3)', 'Domain', 'Sigma0', '+/- 2*sigma interval'))
print('-'*76)
for i, name in enumerate(param_names):
    dom_str = '[%.3f, %.1f]' % (lower[i], upper[i])
    int_str = '[%.3f, %.3f]' % (x0[i]-2*sigmas[i], x0[i]+2*sigmas[i])
    print('%-12s | %10.4f | %-16s | %8.3f | %-24s' % (name, x0[i], dom_str, sigmas[i], int_str))

