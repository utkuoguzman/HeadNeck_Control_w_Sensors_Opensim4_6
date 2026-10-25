import os
import glob
import json
import math
import sys

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

# Exact digitized Happee (2017) Figure 5 experimental targets
HAPPEE_TARGETS = {
    0.31: {"gain": 0.893, "phase": 4.5},
    0.78: {"gain": 0.840, "phase": -98.9},
    1.11: {"gain": 0.814, "phase": -112.6},
    2.48: {"gain": 0.655, "phase": -163.7},
    5.52: {"gain": 0.509, "phase": -194.6},
    7.95: {"gain": 0.443, "phase": -247.7}
}

FREQ_ORDER = [0.31, 0.78, 1.11, 2.48, 5.52, 7.95]

def get_generation_best(gen_num, phase_str="P21G"):
    files = glob.glob(f"logs/candidates/W*_{phase_str}{gen_num}_*.json")
    if not files:
        return None, None, None
        
    worker_data = {}
    for f in files:
        try:
            with open(f, "r", encoding="utf-8") as fp:
                d = json.load(fp)
            w = d["worker"]
            freq = d["freq"]
            if w not in worker_data:
                worker_data[w] = {}
            worker_data[w][freq] = d
        except Exception:
            continue
            
    summary_file = f"logs/phase{phase_str[1]}_{phase_str[2]}_gen_{gen_num}_summary.json" if len(phase_str) >= 4 else None
    best_worker = None
    if summary_file and os.path.exists(summary_file):
        try:
            with open(summary_file, "r", encoding="utf-8") as fp:
                sdata = json.load(fp)
                b_idx = sdata.get("best_candidate_idx")
                if b_idx:
                    best_worker = f"W{b_idx}_{phase_str}{gen_num}"
        except Exception:
            pass
            
    if not best_worker:
        best_score = float('inf')
        for w, fdict in worker_data.items():
            if all(f in fdict for f in FREQ_ORDER):
                tot_score = sum(fdict[f]["score"] for f in FREQ_ORDER)
                if tot_score < best_score:
                    best_score = tot_score
                    best_worker = w

    if not best_worker or best_worker not in worker_data:
        return None, None, None
        
    metrics = {}
    total_score = 0.0
    for freq in FREQ_ORDER:
        d = worker_data[best_worker].get(freq)
        if d:
            metrics[freq] = d
            total_score += d.get("score", 0.0)
        else:
            metrics[freq] = None
            
    return best_worker, metrics, total_score

def main():
    phase = "2.2" if (len(sys.argv) > 1 and sys.argv[1] == "2.2") or glob.glob("logs/phase2_2_gen_*_summary.json") else "2.1"
    
    headers = ["Generation / Model"] + [f"{freq:.2f} Hz" for freq in FREQ_ORDER] + ["Score"]
    print("| " + " | ".join(headers) + " |")
    print("|" + "|".join(["---" for _ in headers]) + "|")
    
    if phase == "2.2":
        happee_row = {
            0.31: "+2.48 dB / 45.0 deg",
            0.78: "+0.42 dB / 6.0 deg",
            1.11: "+0.42 dB / 6.0 deg",
            2.48: "+0.42 dB / 6.0 deg",
            5.52: "+0.83 dB / 13.0 deg",
            7.95: "+0.83 dB / 13.0 deg"
        }
        happee_cells = ["**Happee (2017) Ref (Log)**"] + [happee_row[freq] for freq in FREQ_ORDER] + ["**3,275.52**"]
        print("| " + " | ".join(happee_cells) + " |")
        
        # P2.1 Champion as Baseline
        p21_champ_row = {
            0.31: "+2.24 dB / 62.1 deg",
            0.78: "+1.16 dB / 17.5 deg",
            1.11: "+0.51 dB / 32.7 deg",
            2.48: "-2.28 dB / 48.5 deg",
            5.52: "-1.57 dB / 25.4 deg",
            7.95: "-3.90 dB / 30.3 deg"
        }
        p21_cells = ["*P2.1 Champion (`W12_P21G13`)*"] + [p21_champ_row[freq] for freq in FREQ_ORDER] + ["*12,099.95*"]
        print("| " + " | ".join(p21_cells) + " |")
        
        # Collect P2.2 results
        p22_results = []
        for gen in range(1, 35):
            worker, metrics, score = get_generation_best(gen, phase_str="P22G")
            if not worker or not metrics or score is None:
                continue
            is_complete = all(metrics.get(freq) is not None for freq in FREQ_ORDER)
            if is_complete:
                p22_results.append((gen, worker, metrics, score))
                
        if p22_results:
            champ_score = min(item[3] for item in p22_results)
            total_completed = len(p22_results)
            display_items = []
            for idx, item in enumerate(p22_results):
                gen, worker, metrics, score = item
                is_recent_5 = (idx >= total_completed - 5)
                is_champion = (score == champ_score)
                if is_recent_5 or is_champion:
                    display_items.append((item, is_champion))
                    
            for (gen, worker, metrics, score), is_champ in display_items:
                cells = []
                for freq in FREQ_ORDER:
                    m = metrics.get(freq)
                    db_val = m.get('gain_db_diff', 20.0 * math.log10(m['sim_gain'] / HAPPEE_TARGETS[freq]['gain']))
                    cell = f"{db_val:+.2f} dB / {m['phase_err_deg']:.1f} deg"
                    cells.append(cell)
                label = f"P2.2 Gen {gen} (`{worker}`)"
                if is_champ:
                    label += " **[BEST]**"
                    score_str = f"**{score:,.2f}**"
                else:
                    score_str = f"{score:,.2f}"
                print(f"| {label} | " + " | ".join(cells) + f" | {score_str} |")
    else:
        happee_row = {
            0.31: "33.0% / 45.0 deg",
            0.78: "5.0% / 6.0 deg",
            1.11: "5.0% / 6.0 deg",
            2.48: "5.0% / 6.0 deg",
            5.52: "10.0% / 13.0 deg",
            7.95: "10.0% / 13.0 deg"
        }
        happee_cells = ["**Happee (2017) Ref**"] + [happee_row[freq] for freq in FREQ_ORDER] + ["**3,835.00**"]
        print("| " + " | ".join(happee_cells) + " |")
        
        p21_results = []
        for gen in range(1, 35):
            worker, metrics, score = get_generation_best(gen, phase_str="P21G")
            if not worker or not metrics or score is None:
                continue
            is_complete = all(metrics.get(freq) is not None for freq in FREQ_ORDER)
            if is_complete:
                p21_results.append((gen, worker, metrics, score))
                
        if p21_results:
            champ_score = min(item[3] for item in p21_results)
            total_completed = len(p21_results)
            display_items = []
            for idx, item in enumerate(p21_results):
                gen, worker, metrics, score = item
                is_recent_5 = (idx >= total_completed - 5)
                is_champion = (score == champ_score)
                if is_recent_5 or is_champion:
                    display_items.append((item, is_champion))
                    
            for (gen, worker, metrics, score), is_champ in display_items:
                cells = []
                for freq in FREQ_ORDER:
                    m = metrics.get(freq)
                    cell = f"{m['gain_err_pct']:.1f}% / {m['phase_err_deg']:.1f} deg"
                    cells.append(cell)
                label = f"P2.1 Gen {gen} (`{worker}`)"
                if is_champ:
                    label += " **[BEST]**"
                    score_str = f"**{score:,.2f}**"
                else:
                    score_str = f"{score:,.2f}"
                print(f"| {label} | " + " | ".join(cells) + f" | {score_str} |")

if __name__ == "__main__":
    main()
