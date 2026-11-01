import json
import numpy as np
import sys
import glob
import os

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

BOUNDS_P21 = {
    "r_ton":        (0.45, 2.00),
    "p_ton":        (8.0, 40.0),
    "K_sc":         (30.0, 95.0),
    "r_lead_sc":    (0.01, 0.55),
    "p_sc":         (25.0, 90.0),
    "K_phas":       (1.0, 14.0),
    "p_phas":       (25.0, 85.0),
    "kp":           (0.25, 1.30),
    "r_lead_ccr":   (0.10, 0.55),
    "p_ccr":        (15.0, 75.0),
    "k_dyn":        (0.15, 1.20),
    "k_stat":       (0.15, 1.20)
}

BOUNDS_P22_G1 = {
    "r_ton":        (0.45, 2.00),
    "p_ton":        (8.0, 40.0),
    "K_sc":         (30.0, 95.0),
    "r_lead_sc":    (0.01, 0.55),
    "p_sc":         (25.0, 90.0),
    "K_phas":       (1.0, 14.0),
    "p_phas":       (25.0, 85.0),
    "kp":           (0.25, 1.80),
    "r_lead_ccr":   (0.10, 0.55),
    "p_ccr":        (15.0, 75.0),
    "k_dyn":        (0.15, 1.20),
    "k_stat":       (0.15, 1.20)
}

BOUNDS_P22_G10 = {
    "r_ton":        (0.45, 2.00),
    "p_ton":        (5.0, 40.0),    # broadened lower bound to 5.0
    "K_sc":         (30.0, 95.0),
    "r_lead_sc":    (0.01, 0.55),
    "p_sc":         (25.0, 90.0),
    "K_phas":       (1.0, 14.0),
    "p_phas":       (25.0, 85.0),
    "kp":           (0.25, 1.80),
    "r_lead_ccr":   (0.10, 0.55),
    "p_ccr":        (15.0, 75.0),
    "k_dyn":        (0.15, 1.20),
    "k_stat":       (0.15, 2.00)    # broadened upper bound to 2.00
}

BOUNDS_P22_G12 = {
    "r_ton":        (0.45, 2.00),
    "p_ton":        (5.0, 40.0),    # broadened lower bound to 5.0
    "K_sc":         (30.0, 95.0),
    "r_lead_sc":    (0.01, 0.70),   # broadened ceiling to 0.70
    "p_sc":         (25.0, 110.0),  # broadened ceiling to 110.0
    "K_phas":       (1.0, 25.0),    # broadened ceiling to 25.0
    "p_phas":       (25.0, 85.0),
    "kp":           (0.25, 2.50),   # broadened ceiling to 2.50
    "r_lead_ccr":   (0.10, 0.70),   # broadened ceiling to 0.70
    "p_ccr":        (15.0, 100.0),  # broadened ceiling to 100.0
    "k_dyn":        (0.15, 1.20),
    "k_stat":       (0.15, 2.00)    # broadened upper bound to 2.00
}

BOUNDS_P23 = {
    "r_ton":        (0.30, 2.50),
    "p_ton":        (3.0, 35.0),
    "K_sc":         (0.20, 30.0),
    "r_lead_sc":    (0.01, 0.60),
    "p_sc":         (20.0, 100.0),
    "K_phas":       (0.20, 15.0),
    "p_phas":       (15.0, 80.0),
    "kp":           (0.15, 1.50),
    "r_lead_ccr":   (0.05, 0.65),
    "p_ccr":        (15.0, 90.0),
    "k_dyn":        (0.10, 1.20),
    "k_stat":       (0.15, 1.50)
}

PARAM_DESCRIPTIONS = {
    "r_ton": "Tonic Gain Ratio (K_ton / K_sc)",
    "p_ton": "Tonic Pole (rad/s)",
    "K_sc": "Semicircular Canal Gain (Nm/(rad/s))",
    "r_lead_sc": "Canal Active Lead Ratio (z_sc / p_sc)",
    "p_sc": "Canal Main Filter Pole (rad/s)",
    "K_phas": "Phasic High-Frequency Gain (Nm/(m/s^2))",
    "p_phas": "Phasic High-Frequency Pole (rad/s)",
    "kp": "Proprioceptive Spindle Gain",
    "r_lead_ccr": "Cervical Lead Ratio (z_ccr / p_ccr)",
    "p_ccr": "Cervical Filter Pole (rad/s)",
    "k_dyn": "Dynamic Spindle Gamma Drive",
    "k_stat": "Static Spindle Gamma Drive"
}

def analyze_gen(gen_num, phase="auto"):
    path = None
    bounds = BOUNDS_P23
    if phase == "2.3" or (phase == "auto" and os.path.exists(f"logs/phase2_3_gen_{gen_num}_summary.json")):
        path = f"logs/phase2_3_gen_{gen_num}_summary.json"
        bounds = BOUNDS_P23
    elif phase == "2.2" or (phase == "auto" and os.path.exists(f"logs/phase2_2_gen_{gen_num}_summary.json")):
        path = f"logs/phase2_2_gen_{gen_num}_summary.json"
        if gen_num >= 12:
            bounds = BOUNDS_P22_G12
        elif gen_num >= 10:
            bounds = BOUNDS_P22_G10
        else:
            bounds = BOUNDS_P22_G1
    elif os.path.exists(f"logs/phase2_1_gen_{gen_num}_summary.json"):
        path = f"logs/phase2_1_gen_{gen_num}_summary.json"
        bounds = BOUNDS_P21
    else:
        print(f"Summary file for Gen {gen_num} not found.")
        return
        
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
        
    params = data.get("best_params", {})
    cand_idx = data.get("best_candidate_idx", "?")
    
    print("| Parameter | Value | Range [Min, Max] | Proximity / Status |")
    print("| :--- | :---: | :---: | :--- |")
    
    for key, (low, high) in bounds.items():
        val = params.get(key)
        if val is None:
            continue
        
        # Calculate percentage position in range [0%, 100%]
        pos_pct = (val - low) / (high - low) * 100.0
        
        if pos_pct < 5.0:
            status = f"🔴 **Lower Bound!** ({pos_pct:.1f}%)"
        elif pos_pct < 10.0:
            status = f"🟡 Near Min ({pos_pct:.1f}%)"
        elif pos_pct > 95.0:
            status = f"🔴 **Upper Bound!** ({pos_pct:.1f}%)"
        elif pos_pct > 90.0:
            status = f"🟡 Near Max ({pos_pct:.1f}%)"
        else:
            status = f"🟢 Open Interior ({pos_pct:.1f}%)"
            
        print(f"| `{key}` ({PARAM_DESCRIPTIONS.get(key, key)}) | `{val:.3f}` | `[{low:.2f}, {high:.2f}]` | {status} |")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        gen = int(sys.argv[1])
        phase = sys.argv[2] if len(sys.argv) > 2 else "auto"
        analyze_gen(gen, phase)
    else:
        # Auto detect latest generation
        p23_gens = [int(f.split("gen_")[1].split("_")[0]) for f in glob.glob("logs/phase2_3_gen_*_summary.json")]
        if p23_gens:
            analyze_gen(max(p23_gens), "2.3")
        else:
            p22_gens = [int(f.split("gen_")[1].split("_")[0]) for f in glob.glob("logs/phase2_2_gen_*_summary.json")]
            if p22_gens:
                analyze_gen(max(p22_gens), "2.2")
            else:
                p21_gens = [int(f.split("gen_")[1].split("_")[0]) for f in glob.glob("logs/phase2_1_gen_*_summary.json")]
                if p21_gens:
                    analyze_gen(max(p21_gens), "2.1")
                else:
                    print("No generation summaries found.")
