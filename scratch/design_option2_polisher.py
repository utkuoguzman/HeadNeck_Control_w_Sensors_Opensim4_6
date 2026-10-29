import os
import glob
import json
import math
import numpy as np

FREQS = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]
TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]
PARAM_NAMES = [
    "K_ton", "p_ton", "K_sc", "z_sc", "p_sc",
    "K_phas", "p_phas", "kp_prop", "z_ccr", "p_ccr",
    "K_gamma_dyn", "K_gamma_stat"
]

files = glob.glob("logs/candidates/W*_P24G*.json") + glob.glob("logs/candidates/W_ADAPT_LYAP*.json") + glob.glob("logs/candidates/W_NEW_KP0*.json")
by_worker = {}
for f in files:
    base = os.path.basename(f)
    if "Hz.json" in base:
        w = base.rsplit("_", 1)[0]
        d = json.load(open(f, encoding="utf-8"))
        by_worker.setdefault(w, {}).setdefault("freqs", {})[d["freq"]] = d
        if "params" in d:
            by_worker[w]["params"] = d["params"]
    else:
        w = base[:-5]
        d = json.load(open(f, encoding="utf-8"))
        if "params" in d:
            by_worker.setdefault(w, {})["params"] = d["params"]

clean = []
aborted = []
for w, v in by_worker.items():
    if "params" in v and len(v.get("freqs", {})) == 6:
        tot = sum(x["score"] for x in v["freqs"].values())
        max_g = max(x["sim_gain"] for x in v["freqs"].values())
        if tot < 100000 and max_g < 40000:
            clean.append((tot, w))
        else:
            aborted.append((tot, w))

clean.sort()
print(f"Clean: {len(clean)}, Aborted: {len(aborted)}")
p_clean = np.array([by_worker[w]["params"] for _, w in clean])
p_abort = np.array([by_worker[w]["params"] for _, w in aborted])

print("\nParameter comparison (Clean Top 15 vs Aborted):")
p_top15 = np.array([by_worker[w]["params"] for _, w in clean[:15]])
for j, name in enumerate(PARAM_NAMES):
    print(f"  {name:14s}: Top15 mean={np.mean(p_top15[:,j]):7.3f} [{np.min(p_top15[:,j]):7.3f}, {np.max(p_top15[:,j]):7.3f}] | Abort mean={np.mean(p_abort[:,j]):7.3f} [{np.min(p_abort[:,j]):7.3f}, {np.max(p_abort[:,j]):7.3f}]")

print("\nDetailed look at Top 5 Clean Candidates:")
for tot, w in clean[:5]:
    p = by_worker[w]["params"]
    r_sc = p[3] / p[4]
    r_ccr = p[8] / p[9]
    print(f"  {w:10s} ({tot:8.2f} pts): K_ton={p[0]:.4f}, p_ton={p[1]:.1f}, K_sc={p[2]:.4f}, z_sc={p[3]:.2f} (r={r_sc:.3f}), p_sc={p[4]:.1f}, K_phas={p[5]:.3f}, p_phas={p[6]:.2f}, kp={p[7]:.4f}, z_ccr={p[8]:.2f} (r={r_ccr:.3f}), p_ccr={p[9]:.1f}, gdyn={p[10]:.4f}, gstat={p[11]:.4f}")
    for f, tg, tp in TARGETS:
        d = by_worker[w]["freqs"][f]
        pdiff = (d["sim_phase"] - tp + 180.0) % 360.0 - 180.0
        print(f"      {f:.3f} Hz: gain={d['sim_gain']:6.2f} (tgt {tg:6.2f}, {d['gain_db_diff']:+5.2f} dB, {d['gain_loss']:6.1f} pts) | phase={d['sim_phase']:7.2f}° (tgt {tp:7.2f}°, {pdiff:+6.2f}°, {d['phase_loss']:6.1f} pts) | tot={d['score']:6.1f}")

