import os
import glob
import json
import shutil

# 1. Clean previous logs
print("Cleaning previous logs...")
cand_files = glob.glob("logs/candidates/*.json")
for f in cand_files:
    try:
        os.remove(f)
    except Exception as e:
        print(f"Error removing {f}: {e}")

summary_files = glob.glob("logs/phase2_4_*")
for f in summary_files:
    try:
        os.remove(f)
    except Exception as e:
        print(f"Error removing {f}: {e}")

print(f"Removed {len(cand_files)} candidate logs and {len(summary_files)} Phase 2.4 summary/ledger files.")

# 2. Write new rational initialization JSON
rational_init = {
    "x0": [
        0.5800,   # 0: r_ton (ratio K_ton / K_sc)
        78.0000,  # 1: p_ton (rad/s)
        0.4500,   # 2: K_sc (Nm/(rad/s))
        0.2400,   # 3: r_lead_sc (ratio z_sc / p_sc)
        90.0000,  # 4: p_sc (rad/s)
        4.5000,   # 5: K_phas (Nm/(m/s^2))
        15.0000,  # 6: p_phas (rad/s)
        1.4000,   # 7: kp (spindle gain)
        0.0500,   # 8: r_lead_ccr (ratio z_ccr / p_ccr)
        170.0000, # 9: p_ccr (rad/s)
        0.7000,   # 10: k_dyn
        1.1500    # 11: k_stat
    ],
    "sigma0": [
        0.12,     # r_ton
        10.0,     # p_ton
        0.12,     # K_sc
        0.05,     # r_lead_sc
        12.0,     # p_sc
        0.60,     # K_phas
        3.50,     # p_phas
        0.25,     # kp
        0.015,    # r_lead_ccr
        18.0,     # p_ccr
        0.12,     # k_dyn
        0.12      # k_stat
    ],
    "lower_bounds": [
        0.001,    # r_ton
        5.0,      # p_ton
        0.001,    # K_sc
        0.001,    # r_lead_sc
        15.0,     # p_sc
        0.20,     # K_phas
        1.0,      # p_phas
        0.15,     # kp
        0.0001,   # r_lead_ccr
        20.0,     # p_ccr
        0.01,     # k_dyn
        0.01      # k_stat
    ],
    "upper_bounds": [
        2.50,     # r_ton
        150.0,    # p_ton
        2.50,     # K_sc
        0.60,     # r_lead_sc
        160.0,    # p_sc
        8.00,     # K_phas
        45.0,     # p_phas
        4.00,     # kp
        0.50,     # r_lead_ccr
        240.0,    # p_ccr
        1.50,     # k_dyn
        1.50      # k_stat
    ],
    "description": "Anatomically Split Delayed Model with Smith-Augmented Mahony Rational Initialization"
}

with open("data/prescreen_rational_champion.json", "w", encoding="utf-8") as f:
    json.dump(rational_init, f, indent=2)

print("Saved data/prescreen_rational_champion.json successfully!")

