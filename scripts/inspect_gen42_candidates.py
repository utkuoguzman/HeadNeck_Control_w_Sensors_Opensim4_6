import glob
import json

print("Candidate | Total Score |   0.31 Hz   |   0.78 Hz   |   1.11 Hz   |   2.48 Hz   |   5.52 Hz   |   7.95 Hz   ")
print("-" * 105)

for w in range(1, 19):
    w_name = f"W{w}_G42"
    cells = []
    tot_score = 0
    for freq in [0.31, 0.78, 1.11, 2.48, 5.52, 7.95]:
        fn = f"logs/candidates/{w_name}_{freq}Hz.json"
        try:
            with open(fn, "r", encoding="utf-8") as f:
                d = json.load(f)
                cells.append(f"{d['gain_err_pct']:4.1f}% / {d['phase_err_deg']:4.1f}°")
                tot_score += d.get("score", 0.0)
        except Exception:
            cells.append("     N/A     ")
    print(f"{w_name:<9} | {tot_score:11.1f} | " + " | ".join(f"{c:^11}" for c in cells))

