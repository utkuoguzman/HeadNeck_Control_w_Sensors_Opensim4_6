import glob
import json

files = glob.glob("logs/candidates/W*_P21G1_1.11Hz.json")
print(f"Found {len(files)} candidates evaluated on 1.11 Hz in Gen 1:")
for f in sorted(files):
    with open(f, "r", encoding="utf-8") as fp:
        d = json.load(fp)
        print(f"{d['worker']}: SimGain={d['sim_gain']:.3f} | SimPhase={d['sim_phase']:.1f}° (target -112.6°) | PhaseErr={d['phase_err_deg']:.1f}° | Score={d['score']:.1f}")

