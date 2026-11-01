import json
import sys
sys.path.append('scratch')
from cervical_chadwick_blana_simulator import evaluate_chadwick_blana_candidate

with open('data/prescreen_64k_best_candidate.json', 'r') as f:
    data = json.load(f)

score, details = evaluate_chadwick_blana_candidate(data['x0'])
print(f"Total Score across all 6 frequencies: {score:.2f} pts\n")
print(f"{'Freq':<8} | {'Sim Gain':<10} {'Gain Diff':<10} | {'Sim Phase':<10} {'Phase Err':<10} | {'Score':<10}")
print("-" * 65)
for freq in sorted(details.keys()):
    d = details[freq]
    print(f"{freq:<8.3f} | {d['sim_gain']:<10.1f} {d['gain_db_diff']:>+6.1f} dB  | {d['sim_phase']:>6.1f}°     {d['phase_err_deg']:>6.1f}°     | {d['score']:>8.1f} pts")

