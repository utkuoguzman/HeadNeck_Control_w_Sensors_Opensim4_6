import json
import glob
import sys
import os

def update_ledger(gen_num=None):
    if gen_num is None:
        # Find latest completed summary
        summaries = sorted(glob.glob("logs/phase2_3_gen_*_summary.json"), key=lambda x: int(x.split('_gen_')[1].split('_')[0]))
        if not summaries:
            print("No summaries found.")
            return
        latest_file = summaries[-1]
        gen_num = int(latest_file.split('_gen_')[1].split('_')[0])
    else:
        latest_file = f"logs/phase2_3_gen_{gen_num}_summary.json"

    with open(latest_file, "r", encoding="utf-8") as fp:
        sdata = json.load(fp)

    cand_idx = sdata["best_candidate_idx"]
    cand = f"W{cand_idx}_P23G{gen_num}"

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

    # Replace or append
    existing = [i for i, g in enumerate(ledger["generations"]) if g["generation"] == gen_num]
    record = {
        "generation": gen_num,
        "candidate": cand,
        "score": round(sdata["best_score"], 2),
        "duration_min": round(sdata.get("duration_min", 0.0), 1),
        "params": {k: round(v, 4) for k, v in sdata.get("best_params", {}).items()},
        "bode": bode
    }
    if existing:
        ledger["generations"][existing[0]] = record
    else:
        ledger["generations"].append(record)

    with open("logs/phase2_3_master_ledger.json", "w", encoding="utf-8") as out:
        json.dump(ledger, out, indent=2)

    sys.path.append("scratch")
    import export_progress_matrix
    export_progress_matrix.generate_matrix_md()
    print(f"Gen {gen_num} ({cand}) successfully updated in master ledger and matrix!")

if __name__ == "__main__":
    g = int(sys.argv[1]) if len(sys.argv) > 1 else None
    update_ledger(g)

