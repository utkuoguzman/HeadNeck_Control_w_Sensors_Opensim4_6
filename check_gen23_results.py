import glob
import json
import os
import datetime

cutoff = datetime.datetime(2026, 10, 18, 21, 17).timestamp()
frequencies = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]

print("="*85)
print("GENERATION 23 PROGRESS MONITOR")
print("="*85)

for freq in frequencies:
    pattern = f"logs/candidates/*_G23_{freq:.2f}Hz.json"
    files = glob.glob(pattern)
    recent_files = [f for f in files if os.path.getmtime(f) > cutoff]
    
    cand_data = []
    for f in recent_files:
        try:
            with open(f, "r") as fp:
                cand_data.append(json.load(fp))
        except Exception:
            pass
            
    cand_data.sort(key=lambda d: d.get("score", 999999))
    print(f"\n=== Frequency: {freq:.2f} Hz (Evaluated: {len(cand_data)}/18) ===")
    for d in cand_data[:3]:
        w = d.get("worker")
        sg = d.get("sim_gain")
        tg = d.get("target_gain")
        sp = d.get("sim_phase")
        tp = d.get("target_phase")
        pe = d.get("phase_err_deg")
        ge = d.get("gain_err_pct")
        sc = d.get("score")
        print(f"[{w}] Gain: {sg:.4f} (tgt {tg:.4f}, err {ge:.1f}%), Phase: {sp:+.2f} deg (tgt {tp:+.2f} deg, err: {pe:.2f} deg), Score: {sc:.1f}")

