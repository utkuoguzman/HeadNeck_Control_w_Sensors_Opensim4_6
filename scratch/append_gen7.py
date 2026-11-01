import json
import glob
import sys
import os

cand = "W12_P23G7"
with open("logs/phase2_3_gen_7_summary.json", "r", encoding="utf-8") as fp:
    sdata = json.load(fp)

bode_files = sorted(glob.glob(f"logs/candidates/{cand}_*Hz.json"))
bode = {}
for bf in bode_files:
    with open(bf, "r", encoding="utf-8") as bfp:
        d = json.load(bfp)
        f_str = f"{d['freq']:.3f}"
        bode[f_str] = {
            "gain_db": round(d['gain_db_diff'], 1),
            "phase_err": round(d['phase_err_deg'], 1),
            "score": round(d['score'], 1)
        }

with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as fp:
    ledger = json.load(fp)

# Check if gen 7 already in ledger
if not any(g["generation"] == 7 for g in ledger["generations"]):
    ledger["generations"].append({
        "generation": 7,
        "candidate": cand,
        "score": round(sdata["best_score"], 2),
        "duration_min": round(sdata.get("duration_min", 0.0), 1),
        "params": {k: round(v, 4) for k, v in sdata.get("best_params", {}).items()},
        "bode": bode
    })

    with open("logs/phase2_3_master_ledger.json", "w", encoding="utf-8") as out:
        json.dump(ledger, out, indent=2)

sys.path.append("scratch")
import export_progress_matrix
export_progress_matrix.generate_matrix_md()
print("Gen 7 appended and matrix updated successfully!")

