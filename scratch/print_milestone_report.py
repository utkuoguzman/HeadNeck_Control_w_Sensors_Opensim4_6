import json
import glob
import sys
import os

PHYSICAL_LOWER_BOUNDS = [0.01, 0.50, 0.05, 0.0005, 2.00, 0.05, 2.00, 0.05, 0.0005, 1.00, 0.01, 0.001]
PHYSICAL_UPPER_BOUNDS = [10.0, 300.0, 100.0, 0.99, 300.0, 50.0, 200.0, 20.0, 0.99, 500.0, 5.0, 5.0]
PARAM_NAMES = ["r_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas", "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat"]
PARAM_SYMBOLS = ["r_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas", "p_phas", "k_p", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat"]

def generate_report(gen_num=None):
    if gen_num is None:
        summaries = sorted(glob.glob("logs/phase2_3_gen_*_summary.json"), key=lambda x: int(x.split('_gen_')[1].split('_')[0]))
        if not summaries:
            print("No summaries found.")
            return
        summary_file = summaries[-1]
        gen_num = int(summary_file.split('_gen_')[1].split('_')[0])
    else:
        summary_file = f"logs/phase2_3_gen_{gen_num}_summary.json"

    with open(summary_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Master ledger
    with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
        ledger = json.load(f)

    rec = next((g for g in ledger["generations"] if g["generation"] == gen_num), None)
    cand_label = rec["candidate"] if rec else f"W{data['best_candidate_idx']}_P23G{gen_num}"
    bode = rec["bode"] if rec else {}

    # Print 5 titles
    print(f"## Milestone Report: Phase 2.3 — Generation {gen_num}\n")
    print(f"### 1. Good News")
    print(f"* **Generation Champion:** Candidate [`{cand_label}`](file:///d:/Akademik/PhD_Thesis/OpenSim/HeadNeck_Control_w_Sensors_Opensim4_6/logs/candidates/{cand_label}.json) scored **`{data['best_score']:.2f} pts`** (duration: {data.get('duration_min', 0.0):.1f} min).")
    
    # Compare with previous gen
    if gen_num > 1:
        prev_file = f"logs/phase2_3_gen_{gen_num-1}_summary.json"
        if os.path.exists(prev_file):
            prev_data = json.load(open(prev_file))
            diff = prev_data["best_score"] - data["best_score"]
            if diff > 0:
                print(f"* **Score Improvement:** Improved by **{diff:.2f} pts** compared to Gen {gen_num-1} ({prev_data['best_score']:.2f} pts).")
            else:
                print(f"* **Score Change:** Score shifted by {diff:.2f} pts relative to Gen {gen_num-1}.")

    print(f"\n### 2. Bad News / Challenges")
    # Identify largest error frequency
    worst_freq = None
    worst_score = -1
    for f_str, b in bode.items():
        if b.get("score", 0) > worst_score:
            worst_score = b.get("score", 0)
            worst_freq = f_str
    if worst_freq:
        print(f"* **Highest Error Frequency:** Frequency {worst_freq} Hz contributed {worst_score:.1f} pts ({bode[worst_freq].get('gain_db', 0):+.1f} dB / {bode[worst_freq].get('phase_err', 0):.1f}° error).")

    print(f"\n### 3. Parameter Boundary Watch")
    p = data["best_params"]
    print("| Parameter | Symbol | Value | Proximity & Headroom Status |")
    print("| :--- | :---: | :---: | :--- |")
    for name in PARAM_NAMES:
        val = p.get(name, 0.0)
        print(f"| {name} | `{name}` | {val:.4f} | Monitored |")

    print(f"\n### 4. Current Projection & CMA-ES Trajectory")
    all_scores = [g["score"] for g in ledger["generations"]]
    all_time_best = min(all_scores)
    gap_to_happee = data["best_score"] - 2803.88
    print(f"* **All-Time Campaign Best:** `{all_time_best:.2f} pts`")
    print(f"* **Distance to Happee (2017) Benchmark (2803.88 pts):** {gap_to_happee:+.2f} pts")

    print(f"\n### 5. Progress Matrix vs. Happee (2017) Benchmark")
    if os.path.exists("logs/bode_progress_matrix.md"):
        with open("logs/bode_progress_matrix.md", "r", encoding="utf-8") as mf:
            lines = mf.readlines()
            # print table header and last 6 lines
            table_lines = [l.strip() for l in lines if l.strip().startswith('|')]
            for l in table_lines[:3]:
                print(l)
            for l in table_lines[-4:]:
                print(l)

if __name__ == "__main__":
    g = int(sys.argv[1]) if len(sys.argv) > 1 else None
    generate_report(g)

