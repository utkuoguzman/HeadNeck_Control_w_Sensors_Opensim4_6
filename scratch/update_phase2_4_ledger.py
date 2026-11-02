import json
import glob
import sys
import os

def update_phase2_4_ledger(gen_num=None):
    if gen_num is None:
        summaries = sorted(glob.glob("logs/phase2_4_gen_*_summary.json"), key=lambda x: int(x.split('_gen_')[1].split('_')[0]))
        if not summaries:
            print("No Phase 2.4 summaries found.")
            return
        latest_file = summaries[-1]
        gen_num = int(latest_file.split('_gen_')[1].split('_')[0])
    else:
        latest_file = f"logs/phase2_4_gen_{gen_num}_summary.json"

    with open(latest_file, "r", encoding="utf-8") as fp:
        sdata = json.load(fp)

    cand_idx = sdata["best_candidate_idx"]
    cand = f"W{cand_idx}_P24G{gen_num}"

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

    ledger_path = "logs/phase2_4_master_ledger.json"
    if not os.path.exists(ledger_path):
        prescreen_path = "data/prescreen_rational_champion.json"
        ps_params = {}
        if os.path.exists(prescreen_path):
            with open(prescreen_path, "r", encoding="utf-8") as pfp:
                ps_data = json.load(pfp)
                param_names = [
                    "K_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",
                    "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat",
                    "Kp_task_pitch", "Kd_task_pitch", "Ki_task_pitch"
                ]
                ps_params = {name: round(val, 4) for name, val in zip(param_names, ps_data.get("x0", []))}
        ledger = {
            "phase": 2.4,
            "description": "15D CMA-ES + Local GP Campaign from First-Principles Rational Initialization",
            "benchmark_score": 2803.88,
            "generations": []
        }
    else:
        with open(ledger_path, "r", encoding="utf-8") as fp:
            ledger = json.load(fp)

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

    with open(ledger_path, "w", encoding="utf-8") as out:
        json.dump(ledger, out, indent=2)

    export_matrix()
    print(f"Gen {gen_num} ({cand}) successfully updated in Phase 2.4 ledger and matrix!")

def export_matrix():
    ledger_path = "logs/phase2_4_master_ledger.json"
    if not os.path.exists(ledger_path):
        return
    with open(ledger_path, "r", encoding="utf-8") as fp:
        ledger = json.load(fp)

    md = []
    md.append("| Campaign Milestone | 0.324 Hz | 0.783 Hz | 1.099 Hz | 2.446 Hz | 5.583 Hz | 7.927 Hz | Total Score |")
    md.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |")
    md.append("| **Happee (2017) Benchmark** | -2.5 dB<br>/ 39.1° | +0.0 dB<br>/ 3.0° | +0.7 dB<br>/ 9.1° | -0.9 dB<br>/ 0.1° | -0.7 dB<br>/ 6.7° | +1.0 dB<br>/ 14.8° | **`2803.88`** |")

    freq_keys = ["0.324", "0.783", "1.099", "2.446", "5.583", "7.927"]
    for g in ledger.get("generations", []):
        g_num = g["generation"]
        cand = g["candidate"]
        score_str = f"**`{g['score']:.2f}`**"
        label = f"**Phase 2.4 Baseline** (`{cand}`)" if g_num == 0 else f"**Phase 2.4 Gen {g_num}** (`{cand}`)"
        row = [label]
        for f in freq_keys:
            if f in g.get("bode", {}):
                db = g["bode"][f]["gain_db"]
                db_str = f"+{db:.1f} dB" if db >= 0 else f"{db:.1f} dB"
                ph = g["bode"][f]["phase_err"]
                row.append(f"{db_str}<br>/ {ph:.1f}°")
            else:
                row.append("—")
        row.append(score_str)
        md.append("| " + " | ".join(row) + " |")

    with open("logs/phase2_4_bode_progress_matrix.md", "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print("Exported logs/phase2_4_bode_progress_matrix.md successfully!")

if __name__ == "__main__":
    g = int(sys.argv[1]) if len(sys.argv) > 1 else None
    update_phase2_4_ledger(g)

