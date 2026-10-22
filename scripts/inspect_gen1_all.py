import glob
import json

FREQ_ORDER = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]

with open("logs/phase2_1_gen_1_summary.json", "r") as fp:
    sdata = json.load(fp)

print(f"Official best idx in Gen 1: {sdata['best_candidate_idx']} with score {sdata['best_score']:.2f}")

candidates = {}
for w in range(1, 19):
    worker_label = f"W{w}_P21G1"
    candidates[worker_label] = {}
    for freq in FREQ_ORDER:
        fpath = f"logs/candidates/{worker_label}_{freq:.2f}Hz.json"
        if glob.glob(fpath):
            with open(fpath) as f:
                candidates[worker_label][freq] = json.load(f)

for w_label, freqs in sorted(candidates.items()):
    completed = len(freqs)
    total_score = sum(d["score"] for d in freqs.values())
    if completed == 6:
        print(f"\n{w_label} (PASSED ALL 6 FREQS): Total Score = {total_score:.1f}")
        for freq in [0.31, 0.78, 1.11, 2.48, 5.52, 7.95]:
            d = freqs[freq]
            gain_err = abs(d['sim_gain'] - d['target_gain']) / d['target_gain'] * 100
            phase_diff = (d['sim_phase'] - d['target_phase'] + 180.0) % 360.0 - 180.0
            print(f"  {freq} Hz: Gain={d['sim_gain']:.3f} (err {gain_err:.1f}%), Phase={d['sim_phase']:.1f}° (target {d['target_phase']}°, diff {abs(phase_diff):.1f}°), Score={d['score']:.1f}")
    elif completed > 0:
        print(f"{w_label}: Completed {completed}/6 freqs, Total Score so far = {total_score:.1f}")

