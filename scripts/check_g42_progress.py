import glob
import json

frequencies = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]

print("="*80)
print("GEN 42 FREQUENCY EVALUATION STATUS")
print("="*80)

for freq in frequencies:
    files = glob.glob(f"logs/candidates/*_G42_{freq}Hz.json")
    if not files:
        print(f"Freq {freq:5.2f} Hz: 0/18 completed (IN PROGRESS or PENDING)")
        continue
    records = []
    for f in files:
        with open(f, "r", encoding="utf-8") as fp:
            try:
                records.append(json.load(fp))
            except Exception:
                pass
    records.sort(key=lambda r: r.get("score", 999999.0))
    best = records[0]
    print(f"Freq {freq:5.2f} Hz: ({len(records)}/18 passed) | Best: {best['worker']:<7} | Gain Err: {best['gain_err_pct']:5.2f}% | Phase Err: {best['phase_err_deg']:5.2f} deg | Score: {best['score']:7.2f} (SimG={best['sim_gain']:.3f}, SimPh={best['sim_phase']:.1f})")

print("="*80)

