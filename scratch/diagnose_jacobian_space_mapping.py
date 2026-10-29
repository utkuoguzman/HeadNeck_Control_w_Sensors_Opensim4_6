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

valid = [w for w, v in by_worker.items() if "params" in v and len(v.get("freqs", {})) == 6]
print(f"Total valid 6-freq OpenSim evaluations: {len(valid)}")

scored = []
for w in valid:
    tot = sum(v["score"] for v in by_worker[w]["freqs"].values())
    max_g = max(v["sim_gain"] for v in by_worker[w]["freqs"].values())
    if tot < 200000 and max_g < 40000:
        scored.append((tot, w))
scored.sort()
print(f"Clean non-aborted evaluations: {len(scored)}")
print("\nTop 15 OpenSim Evaluations:")
for tot, w in scored[:15]:
    p = by_worker[w]["params"]
    g_str = " ".join(f"{by_worker[w]['freqs'][f]['gain_db_diff']:+5.1f}dB/{by_worker[w]['freqs'][f]['sim_phase']-t_ph:+5.1f}°" for f, _, t_ph in TARGETS)
    print(f"  {w:12s}: {tot:9.2f} pts | K_ton={p[0]:.3f} K_sc={p[2]:.4f} z_sc={p[3]:.1f} p_sc={p[4]:.1f} K_phas={p[5]:.2f} kp={p[7]:.3f} gdyn={p[10]:.3f} gstat={p[11]:.3f}")
    print(f"                Bode errs: {g_str}")
