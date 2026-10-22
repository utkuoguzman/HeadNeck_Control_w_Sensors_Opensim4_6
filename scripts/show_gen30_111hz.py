import glob
import json

files = sorted(glob.glob("logs/candidates/W*_G30_1.11Hz.json"))
print(f"--- Gen 30 Candidates at 1.11 Hz (N={len(files)}) ---")
data = [json.load(open(f, encoding="utf-8")) for f in files]
data.sort(key=lambda x: x["score"])

for d in data:
    beat_gain = "[BEATS HAPPEE]" if d["gain_err_pct"] < 5.0 else ""
    beat_phase = "[BEATS HAPPEE]" if d["phase_err_deg"] < 6.0 else ""
    print(f"{d['worker']:<10} | Score: {d['score']:>6.2f} | Gain Err: {d['gain_err_pct']:>5.2f}% {beat_gain:<15} | Phase Err: {d['phase_err_deg']:>5.2f} deg {beat_phase}")

