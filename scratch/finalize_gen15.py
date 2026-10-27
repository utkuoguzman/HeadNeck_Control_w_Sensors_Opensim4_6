import json
import re
import datetime
import os

# 1. Parse campaign log for Gen 15
with open("logs/phase2_3_campaign.log", "r") as f:
    lines = f.readlines()

gen15_lines = []
in_gen15 = False
start_time_str = "2026-10-26T19:10:16"
for line in lines:
    if "STARTING PHASE 2.3 GENERATION 15" in line:
        in_gen15 = True
        m = re.search(r"at (\d\d:\d\d:\d\d)", line)
        if m:
            start_time_str = f"2026-10-26T{m.group(1)}"
        continue
    if in_gen15:
        gen15_lines.append(line)

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

all_scores = []
for i in range(1, 19):
    w_id = f"W{i}_P23G15"
    recs = worker_scores[w_id]
    tot = recs[-1][2]
    all_scores.append(tot)

best_idx = 2 # W3 is index 2 (1-based: 3)
best_score = all_scores[best_idx]
print(f"Best Candidate: W{best_idx+1}_P23G15, Score: {best_score}")

# Load W3_P23G15 candidate params
with open("logs/candidates/W3_P23G15.json", "r") as f:
    cand_data = json.load(f)

# Candidate saved params: [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat]
params = cand_data["params"]
K_ton = params[0]
p_ton = params[1]
K_sc = params[2]
z_sc = params[3]
p_sc = params[4]
K_phas = params[5]
p_phas = params[6]
kp_prop = params[7]
z_ccr = params[8]
p_ccr = params[9]
k_dyn = params[10]
k_stat = params[11]

r_ton = K_ton / K_sc
r_lead_sc = z_sc / p_sc
r_lead_ccr = z_ccr / p_ccr

start_dt = datetime.datetime.fromisoformat(start_time_str)
end_dt = datetime.datetime.now()
duration_min = (end_dt - start_dt).total_seconds() / 60.0

summary = {
    "phase": 2.3,
    "generation": 15,
    "start_time": start_dt.isoformat(),
    "end_time": end_dt.isoformat(),
    "duration_min": round(duration_min, 1),
    "best_candidate_idx": best_idx + 1,
    "best_score": float(best_score),
    "r_ton": float(r_ton),
    "best_params": {
        "r_ton": float(r_ton),
        "K_ton": float(K_ton),
        "p_ton": float(p_ton),
        "K_sc": float(K_sc),
        "r_lead_sc": float(r_lead_sc),
        "z_sc": float(z_sc),
        "p_sc": float(p_sc),
        "K_phas": float(K_phas),
        "p_phas": float(p_phas),
        "kp": float(kp_prop),
        "r_lead_ccr": float(r_lead_ccr),
        "z_ccr": float(z_ccr),
        "p_ccr": float(p_ccr),
        "k_dyn": float(k_dyn),
        "k_stat": float(k_stat)
    },
    "all_scores": all_scores
}

summary_path = "logs/phase2_3_gen_15_summary.json"
with open(summary_path, "w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2)

print(f"Saved {summary_path} successfully!")

