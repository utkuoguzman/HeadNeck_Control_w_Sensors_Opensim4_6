import glob
import json
import os

frequencies = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]
for freq in frequencies:
    pattern = f"logs/candidates/*_G21_{freq:.2f}Hz.json"
    files = glob.glob(pattern)
    import datetime
    cutoff = datetime.datetime(2026, 10, 18, 17, 30).timestamp()
    recent_files = [f for f in files if os.path.getmtime(f) > cutoff]
    
    cand_data = []
    for f in recent_files:
        try:
            with open(f, "r") as fp:
                cand_data.append(json.load(fp))
        except Exception:
            pass
            
    cand_data.sort(key=lambda d: d.get("score", 999999))
    print(f"\n=== Frequency: {freq} Hz (Total Evaluated: {len(cand_data)}) ===")
    for d in cand_data[:4]:
        w = d.get("worker")
        sg = d.get("sim_gain")
        tg = d.get("target_gain")
        sp = d.get("sim_phase")
        tp = d.get("target_phase")
        pe = d.get("phase_err_deg")
        ge = d.get("gain_err_pct")
        sc = d.get("score")
        print(f"[{w}] Gain: {sg:.4f} (tgt {tg:.4f}, err {ge:.1f}%), Phase: {sp:+.2f} deg (tgt {tp:+.2f} deg, err: {pe:.2f} deg), Score: {sc:.1f}")
