import os
import glob
import json
import re
import sys

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

def compute_real_score(freq, sim_gain, sim_phase):
    t_gain = HAPPEE_TARGETS[freq]["gain"]
    t_phase = HAPPEE_TARGETS[freq]["phase"]
    
    gain_err_pct = abs(sim_gain - t_gain) / t_gain * 100.0
    phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
    phase_err_deg = abs(phase_diff)
    
    score = (gain_err_pct)**2 + (phase_err_deg)**2
    return gain_err_pct, phase_err_deg, score

def get_generation_best(gen_num, is_p21=False):
    prefix = "P21G" if is_p21 else "P2G"
    files = glob.glob(f"logs/candidates/W*_{prefix}{gen_num}_*.json")
    if not files:
        return None, None, None
        
    worker_data = {}
    for f in files:
        m = re.search(rf"(W\d+_{prefix}{gen_num})_([\d\.]+)Hz\.json", f)
        if m:
            w_name = m.group(1)
            freq = float(m.group(2))
            try:
                with open(f, "r", encoding="utf-8") as fp:
                    d = json.load(fp)
                    gain_err, phase_err, score = compute_real_score(freq, d["sim_gain"], d["sim_phase"])
                    worker_data.setdefault(w_name, {})[freq] = {
                        "gain_err_pct": gain_err,
                        "phase_err_deg": phase_err,
                        "score": score
                    }
            except Exception:
                pass

    if not worker_data:
        return None, None, None

    # Check official summary file first
    summary_path = f"logs/phase2_1_gen_{gen_num}_summary.json" if is_p21 else f"logs/phase2_gen_{gen_num}_summary.json"
    if os.path.exists(summary_path):
        try:
            with open(summary_path, "r", encoding="utf-8") as sf:
                sdata = json.load(sf)
                official_idx = sdata.get("best_candidate_idx")
                official_name = f"W{official_idx}_{prefix}{gen_num}"
                best_score = sdata.get("best_score", None)
                if official_name in worker_data:
                    metrics = {}
                    for freq in FREQ_ORDER:
                        metrics[freq] = worker_data[official_name].get(freq, None)
                    return official_name, metrics, best_score
        except Exception:
            pass

    full_workers = {w: sum(vals[freq]["score"] for freq in FREQ_ORDER)
                    for w, vals in worker_data.items() if all(freq in vals for freq in FREQ_ORDER)}
    
    if full_workers:
        best_worker = min(full_workers, key=full_workers.get)
        best_score = full_workers[best_worker]
    else:
        max_freqs = max(len(vals) for vals in worker_data.values())
        cand_workers = {w: sum(d["score"] for d in vals.values())
                        for w, vals in worker_data.items() if len(vals) == max_freqs}
        best_worker = min(cand_workers, key=cand_workers.get)
        best_score = None

    metrics = {}
    for freq in FREQ_ORDER:
        metrics[freq] = worker_data[best_worker].get(freq, None)
        
    return best_worker, metrics, best_score

def main():
    print(f"{'Generation / Model':<22} | " + " | ".join(f"{freq:^14.2f} Hz" for freq in FREQ_ORDER) + " | Score")
    print("-" * 135)
    
    happee_row = {
        0.31: "33.0% / 45.0°",
        0.78: " 5.0% /  6.0°",
        1.11: " 5.0% /  6.0°",
        2.48: " 5.0% /  6.0°",
        5.52: "10.0% / 13.0°",
        7.95: "10.0% / 13.0°"
    }
    print(f"{'Happee (2017) Ref':<22} | " + " | ".join(f"{happee_row[freq]:^17}" for freq in FREQ_ORDER) + " | 3,835.00")
    print("-" * 135)
    
    # Phase 2.1 (Consolidated)
    for gen in range(1, 26):
        worker, metrics, score = get_generation_best(gen, is_p21=True)
        if not worker or not metrics:
            continue
        cells = []
        for freq in FREQ_ORDER:
            m = metrics.get(freq)
            if m:
                cell = f"{m['gain_err_pct']:.1f}% / {m['phase_err_deg']:.1f}°"
            else:
                cell = "[Evaluating...]"
            cells.append(f"{cell:^17}")
        is_complete = all(metrics.get(freq) is not None for freq in FREQ_ORDER)
        label = f"P2.1 Gen {gen} ({worker})" if is_complete else f"P2.1 Gen {gen}* ({worker})"
        score_str = f"{score:10.2f}" if score is not None else "[Running]"
        print(f"{label:<22} | " + " | ".join(cells) + f" | {score_str}")

if __name__ == "__main__":
    main()
