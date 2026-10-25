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

BOUNDS_P22 = {
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
    bounds = BOUNDS_P22
    if phase == "2.2" or (phase == "auto" and os.path.exists(f"logs/phase2_2_gen_{gen_num}_summary.json")):
        path = f"logs/phase2_2_gen_{gen_num}_summary.json"
        bounds = BOUNDS_P22
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
    
    warnings = []
    
    for p_name, (low, high) in bounds.items():
        val = params.get(p_name)
        if val is None:
            continue
            
        span = high - low
        pos_pct = (val - low) / span * 100.0
        
        status = "Normal"
        if pos_pct <= 10.0:
            status = f"[ALERT: Near Lower Bound] ({pos_pct:.1f}% of range)"
            warnings.append((p_name, val, "LOWER", pos_pct))
        elif pos_pct >= 90.0:
            status = f"[ALERT: Near Upper Bound] ({pos_pct:.1f}% of range)"
            warnings.append((p_name, val, "UPPER", pos_pct))
        else:
            status = f"Normal ({pos_pct:.1f}% of range)"
            
        desc = PARAM_DESCRIPTIONS.get(p_name, p_name)
        print(f"| **{p_name}** (`{desc}`) | {val:.3f} | [{low:.2f}, {high:.2f}] | {status} |")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python check_parameter_boundaries.py <gen_number> [phase]")
        sys.exit(1)
        
    gen = int(sys.argv[1])
    phase = sys.argv[2] if len(sys.argv) > 2 else "auto"
    analyze_gen(gen, phase)
