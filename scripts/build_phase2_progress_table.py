import os
import glob
import json
import math
import sys

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')

# Canonical Aligned Happee (2017) Angular Velocity Experimental Targets
HAPPEE_TARGETS_CANONICAL = {
    0.32: {"freq_hz": 0.324, "gain": 86.55, "phase": -7.03, "happee_gain": 64.80, "happee_phase": -46.14},
    0.78: {"freq_hz": 0.783, "gain": 236.93, "phase": -96.96, "happee_gain": 236.93, "happee_phase": -93.98},
    1.10: {"freq_hz": 1.099, "gain": 319.74, "phase": -109.23, "happee_gain": 346.08, "happee_phase": -118.32},
    2.45: {"freq_hz": 2.446, "gain": 423.34, "phase": -161.34, "happee_gain": 380.60, "happee_phase": -161.26},
    5.58: {"freq_hz": 5.583, "gain": 503.30, "phase": -201.38, "happee_gain": 464.66, "happee_phase": -194.71},
    7.93: {"freq_hz": 7.927, "gain": 572.69, "phase": -247.62, "happee_gain": 644.85, "happee_phase": -232.83}
}

FREQ_ORDER = [0.32, 0.78, 1.10, 2.45, 5.58, 7.93]

def find_best_freq_data(fdict, target_f):
    # Match candidate freq by proximity
    for f_val in fdict:
        if abs(f_val - target_f) < 0.08:
            return fdict[f_val]
    return None

def get_generation_best(gen_num, phase_str="P23G"):
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
    summary_score = None
    if summary_file and os.path.exists(summary_file):
        try:
            with open(summary_file, "r", encoding="utf-8") as fp:
                sdata = json.load(fp)
                b_idx = sdata.get("best_candidate_idx")
                summary_score = sdata.get("best_score")
                if b_idx:
                    best_worker = f"W{b_idx}_{phase_str}{gen_num}"
        except Exception:
            pass
            
    if not best_worker:
        best_score = float('inf')
        for w, fdict in worker_data.items():
            matched = [find_best_freq_data(fdict, f) for f in FREQ_ORDER]
            if all(m is not None for m in matched):
                tot_score = sum(m["score"] for m in matched)
                if tot_score < best_score:
                    best_score = tot_score
                    best_worker = w

    if not best_worker or best_worker not in worker_data:
        return None, None, None
        
    metrics = {}
    total_score = 0.0
    for freq in FREQ_ORDER:
        m = find_best_freq_data(worker_data[best_worker], freq)
        if m:
            metrics[freq] = m
            total_score += m.get("score", 0.0)
        else:
            metrics[freq] = None
            
    final_score = summary_score if summary_score is not None else total_score
    return best_worker, metrics, final_score

def main():
    headers = ["Generation / Model"] + [f"{freq:.2f} Hz" for freq in FREQ_ORDER] + ["Score"]
    print("| " + " | ".join(headers) + " |")
    print("|" + "|".join(["---" for _ in headers]) + "|")
    
    # Happee 2017 Model Benchmark on Canonical Aligned Data
    happee_row = {
        0.32: "-2.51 dB / 39.1 deg",
        0.78: "+0.00 dB / 3.0 deg",
        1.10: "+0.69 dB / 9.1 deg",
        2.45: "-0.92 dB / 0.1 deg",
        5.58: "-0.69 dB / 6.7 deg",
        7.93: "+1.03 dB / 14.8 deg"
    }
    happee_cells = ["**Happee (2017) Model (Benchmark)**"] + [happee_row[freq] for freq in FREQ_ORDER] + ["**2,803.88**"]
    print("| " + " | ".join(happee_cells) + " |")
    
    # Collect P2.3 results
    p23_results = []
    for gen in range(1, 40):
        worker, metrics, score = get_generation_best(gen, phase_str="P23G")
        if not worker or not metrics or score is None:
            continue
        is_complete = all(metrics.get(freq) is not None for freq in FREQ_ORDER)
        if is_complete:
            p23_results.append((gen, worker, metrics, score))
            
    if p23_results:
        champ_score_23 = min([item[3] for item in p23_results])
        for gen, worker, metrics, score in p23_results:
            cells = []
            for freq in FREQ_ORDER:
                m = metrics.get(freq)
                if m is None:
                    cells.append("N/A")
                    continue
                db_val = m.get('gain_db_diff', 0.0)
                phase_err = m.get('phase_err_deg', 0.0)
                cells.append(f"{db_val:+.2f} dB / {phase_err:.1f} deg")
            is_champ = (score == champ_score_23)
            label = f"P2.3 Gen {gen} (`{worker}`)"
            if is_champ:
                label += " **[BEST]**"
                score_str = f"**{score:,.2f}**"
            else:
                score_str = f"{score:,.2f}"
            print(f"| {label} | " + " | ".join(cells) + f" | {score_str} |")
    else:
        # Show recent P2.2 for reference if P2.3 has not started yet
        p22_results = []
        for gen in [10, 11, 12]:
            worker, metrics, score = get_generation_best(gen, phase_str="P22G")
            if worker and metrics and score is not None:
                p22_results.append((gen, worker, metrics, score))
        for gen, worker, metrics, score in p22_results:
            cells = []
            for freq in FREQ_ORDER:
                m = metrics.get(freq)
                if m is None:
                    cells.append("N/A")
                    continue
                db_val = m.get('gain_db_diff', 0.0)
                phase_err = m.get('phase_err_deg', 0.0)
                cells.append(f"{db_val:+.2f} dB / {phase_err:.1f} deg")
            label = f"P2.2 Gen {gen} (`{worker}`)"
            print(f"| {label} | " + " | ".join(cells) + f" | {score:,.2f} |")

if __name__ == "__main__":
    main()
