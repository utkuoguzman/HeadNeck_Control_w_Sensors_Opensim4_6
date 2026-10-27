import json

worker_scores = [
    655056.36, 69067.69, 31802.31, 68812.68, 32596.09, 56459.26,
    26285.00, 31392.65, 121001.16, 82863.76, 95935.12, 110527.14,
    34073.08, 655056.36, 42129.46, 55353.93, 37599.88, 87194.97
]

with open("logs/phase2_3_gen_13_summary.json", "r", encoding="utf-8") as f:
    s = json.load(f)

s["all_scores"] = worker_scores

with open("logs/phase2_3_gen_13_summary.json", "w", encoding="utf-8") as f:
    json.dump(s, f, indent=2)

print("Added all_scores to Gen 13 summary JSON successfully!")

