import sys
import glob
import json

gen = sys.argv[1] if len(sys.argv) > 1 else "43"
freq = sys.argv[2] if len(sys.argv) > 2 else "2.48"

files = glob.glob(f"logs/candidates/*_G{gen}_{freq}Hz.json")
records = []
for f in files:
    try:
        with open(f, "r", encoding="utf-8") as fp:
            records.append(json.load(fp))
    except Exception:
        pass

records.sort(key=lambda r: r.get("score", 999999.0))
print(f"--- Top Candidates for {freq} Hz (Gen {gen}) [{len(records)}/18] ---")
for r in records[:8]:
    print(f"{r['worker']:<8} | GainErr: {r['gain_err_pct']:5.2f}% | PhaseErr: {r['phase_err_deg']:5.2f} deg | Score: {r['score']:7.1f} | SimG: {r['sim_gain']:.3f} | SimPh: {r['sim_phase']:.1f}")

