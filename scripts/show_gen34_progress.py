import glob
import json

for freq in [7.95, 5.52]:
    files = sorted(glob.glob(f"logs/candidates/W*_G34_{freq:.2f}Hz.json"))
    data = [json.load(open(f, encoding="utf-8")) for f in files]
    data.sort(key=lambda x: x["score"])
    print(f"\n--- Gen 34 at {freq:.2f} Hz (N={len(files)}) Top Candidates ---")
    t_gain = 10.0
    t_phase = 13.0
    for d in data[:5]:
        bg = "[BEATS HAPPEE]" if d["gain_err_pct"] < t_gain else ""
        bp = "[BEATS HAPPEE]" if d["phase_err_deg"] < t_phase else ""
        print(f"{d['worker']:<10} | Score: {d['score']:>6.2f} | Gain Err: {d['gain_err_pct']:>5.2f}% {bg:<15} | Phase Err: {d['phase_err_deg']:>5.2f} deg {bp}")

