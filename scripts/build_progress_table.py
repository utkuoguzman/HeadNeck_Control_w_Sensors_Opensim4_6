import os
import glob
import json
import re

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
    
    # Equal-Corridor Balanced Loss: Score = (Gain % Error)^2 + (Phase Error in deg)^2
    score = (gain_err_pct)**2 + (phase_err_deg)**2
    return gain_err_pct, phase_err_deg, score

def get_generation_best(gen_num):
    files = glob.glob(f"logs/candidates/W*_G{gen_num}_*.json")
    if not files:
        return None, None
        
    worker_data = {}
    for f in files:
        m = re.search(rf"(W\d+_G{gen_num})_([\d\.]+)Hz\.json", f)
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
        return None, None

    # Check if official summary exists first
    summary_path = f"logs/gen_{gen_num}_summary.json"
    if os.path.exists(summary_path):
        try:
            with open(summary_path, "r", encoding="utf-8") as sf:
                sdata = json.load(sf)
                official_idx = sdata.get("best_candidate_idx")
                official_name = f"W{official_idx}_G{gen_num}"
                if official_name in worker_data:
                    metrics = {}
                    for freq in FREQ_ORDER:
                        metrics[freq] = worker_data[official_name].get(freq, None)
                    return official_name, metrics
        except Exception:
            pass

    # Find worker that completed all 6 frequencies with lowest score
    full_workers = {w: sum(vals[freq]["score"] for freq in FREQ_ORDER)
                    for w, vals in worker_data.items() if all(freq in vals for freq in FREQ_ORDER)}
    
    if full_workers:
        best_worker = min(full_workers, key=full_workers.get)
    else:
        # If in progress, find worker with most completed frequencies and lowest sum score
        max_freqs = max(len(vals) for vals in worker_data.values())
        cand_workers = {w: sum(d["score"] for d in vals.values())
                        for w, vals in worker_data.items() if len(vals) == max_freqs}
        best_worker = min(cand_workers, key=cand_workers.get)

    metrics = {}
    for freq in FREQ_ORDER:
        metrics[freq] = worker_data[best_worker].get(freq, None)
        
    return best_worker, metrics

def main():
    import sys
    start_gen = 26
    if len(sys.argv) > 1:
        try:
            start_gen = int(sys.argv[1])
        except ValueError:
            pass

    print(f"{'Generation / Model':<20} | " + " | ".join(f"{freq:^14.2f} Hz" for freq in FREQ_ORDER))
    print("-" * 118)
    
    # Happee Published Benchmark reference row
    happee_row = {
        0.31: "33.0% / 45.0°",
        0.78: " 5.0% /  6.0°",
        1.11: " 5.0% /  6.0°",
        2.48: " 5.0% /  6.0°",
        5.52: "10.0% / 13.0°",
        7.95: "10.0% / 13.0°"
    }
    print(f"{'Happee (2017) Ref':<20} | " + " | ".join(f"{happee_row[freq]:^17}" for freq in FREQ_ORDER))
    print("-" * 118)
    
    for gen in range(start_gen, 61):
        worker, metrics = get_generation_best(gen)
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
        label = f"Gen {gen} ({worker})" if is_complete else f"Gen {gen}* ({worker})"
        print(f"{label:<20} | " + " | ".join(cells))

if __name__ == "__main__":
    main()

