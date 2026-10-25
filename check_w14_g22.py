import json

freqs = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]
print("="*85)
print("GENERATION 22 CHAMPION (Candidate W14) DETAILED METRICS ACROSS ALL 6 FREQUENCIES")
print("="*85)
total_score = 0.0
for freq in freqs:
    fname = f"logs/candidates/W14_G22_{freq:.2f}Hz.json"
    try:
        with open(fname, "r") as fp:
            d = json.load(fp)
            sg = d["sim_gain"]
            tg = d["target_gain"]
            ge = d["gain_err_pct"]
            sp = d["sim_phase"]
            tp = d["target_phase"]
            pe = d["phase_err_deg"]
            sc = d["score"]
            total_score += sc
            print(f"{freq:5.2f} Hz | Gain: {sg:.4f} (tgt {tg:.4f}, err {ge:5.1f}%) | Phase: {sp:+6.1f} deg (tgt {tp:+6.1f} deg, err: {pe:5.1f} deg) | Score: {sc:7.1f}")
    except Exception as e:
        print(f"{freq:5.2f} Hz | Error loading {fname}: {e}")
print("-"*85)
print(f"Total Composite Score: {total_score:.2f}")
print("="*85)

