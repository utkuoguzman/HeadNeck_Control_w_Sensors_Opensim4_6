import json
import re
import glob

# Read log file for Gen 15 lines
with open("logs/phase2_3_campaign.log", "r") as f:
    lines = f.readlines()

gen15_lines = []
in_gen15 = False
for line in lines:
    if "STARTING PHASE 2.3 GENERATION 15" in line:
        in_gen15 = True
        gen15_lines = []
        continue
    if in_gen15:
        gen15_lines.append(line)

# Extract scores per worker
worker_scores = {}
for i in range(1, 19):
    worker_scores[f"W{i}_P23G15"] = []

pass_pattern = re.compile(r"\[PASS\] \[(W\d+_P23G15)\] passed ([\d\.]+)Hz \((\d+)/6\) \(Score: ([\d\.]+) \| Cum: ([\d\.]+)\)")

for line in gen15_lines:
    m = pass_pattern.search(line)
    if m:
        w_id = m.group(1)
        freq = float(m.group(2))
        step = int(m.group(3))
        sc = float(m.group(4))
        cum = float(m.group(5))
        worker_scores[w_id].append((freq, sc, cum))

print(f"{'Worker':<12} {'Total Score':<12} {'Freq Scores (0.324, 0.783, 1.099, 2.446, 5.583, 7.927)'}")
print("-" * 80)
sorted_workers = []
for w_id, recs in worker_scores.items():
    if len(recs) == 6:
        tot = recs[-1][2]
        sorted_workers.append((w_id, tot, [r[1] for r in recs]))

sorted_workers.sort(key=lambda x: x[1])
for w_id, tot, scs in sorted_workers:
    scs_str = ", ".join([f"{s:.1f}" for s in scs])
    print(f"{w_id:<12} {tot:<12.2f} [{scs_str}]")

print(f"\nChampion: {sorted_workers[0][0]} with total score {sorted_workers[0][1]:.2f}")

