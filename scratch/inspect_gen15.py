import json
import glob
import os

files = glob.glob('logs/candidates/*P23G15.json')
candidates = []
for f in files:
    with open(f, 'r') as fp:
        data = json.load(fp)
        candidates.append((os.path.basename(f), data['total_score'], data['candidate_vector'], data['scores']))

candidates.sort(key=lambda x: x[1])

print(f"{'Candidate':<15} {'Total Score':<12} {'Scores by Freq'}")
print("-" * 75)
for name, score, vec, scs in candidates:
    scs_str = ", ".join([f"{s:.1f}" for s in scs])
    print(f"{name:<15} {score:<12.2f} [{scs_str}]")

print(f"\nWinner: {candidates[0][0]} with score {candidates[0][1]:.2f}")

