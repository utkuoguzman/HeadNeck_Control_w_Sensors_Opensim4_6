import json
import glob
import os
import pandas as pd
import numpy as np

def build_ledger():
    df = pd.read_csv('extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv')
    
    # Happee baseline
    hap_bode = {}
    hap_total = 0.0
    for _, r in df.iterrows():
        f = f"{r['Frequency_Hz']:.3f}"
        g_db = 20.0 * np.log10(r['HappeeModel_Gain_deg_per_m'] / r['Exp_Gain_deg_per_m'])
        p_diff = (r['HappeeModel_Phase_deg'] - r['Exp_Phase_deg'] + 180.0) % 360.0 - 180.0
        hap_total += ((g_db/1.0)**2)*100.0 + ((p_diff/10.0)**2)*100.0
        hap_bode[f] = {"gain_db": round(g_db, 1), "phase_err": round(abs(p_diff), 1)}
    
    ledger = {
        "benchmark": {
            "name": "Happee (2017) Benchmark",
            "score": round(hap_total, 2),
            "bode": hap_bode
        },
        "generations": []
    }
    
    # Process gen 1 to 6
    cand_map = {
        1: "W10_P23G1",
        2: "W17_P23G2",
        3: "W17_P23G3",
        4: "W16_P23G4",
        5: "W9_P23G5",
        6: "W4_P23G6"
    }
    
    for g, cand in cand_map.items():
        summary_path = f"logs/phase2_3_gen_{g}_summary.json"
        if not os.path.exists(summary_path):
            continue
        with open(summary_path, 'r', encoding='utf-8') as fp:
            sdata = json.load(fp)
            
        bode_files = sorted(glob.glob(f"logs/candidates/{cand}_*Hz.json"))
        bode = {}
        for bf in bode_files:
            with open(bf, 'r', encoding='utf-8') as bfp:
                d = json.load(bfp)
                f_str = f"{d['freq']:.3f}"
                bode[f_str] = {
                    "gain_db": round(d['gain_db_diff'], 1),
                    "phase_err": round(d['phase_err_deg'], 1),
                    "score": round(d['score'], 1)
                }
                
        ledger["generations"].append({
            "generation": g,
            "candidate": cand,
            "score": round(sdata["best_score"], 2),
            "duration_min": round(sdata.get("duration_min", 0.0), 1),
            "params": {k: round(v, 4) for k, v in sdata.get("best_params", {}).items()},
            "bode": bode
        })
        
    with open("logs/phase2_3_master_ledger.json", "w", encoding="utf-8") as out:
        json.dump(ledger, out, indent=2)
    print("Master ledger created successfully with Gen 1..6!")

if __name__ == "__main__":
    build_ledger()

