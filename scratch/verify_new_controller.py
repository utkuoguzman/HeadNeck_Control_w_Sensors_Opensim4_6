import json, math, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

PY = sys.executable
ps = json.load(open("data/prescreen_rational_champion.json"))
x0_raw = list(ps["x0"])

# Rescaled x0 preserves exact physical passband torque of K_sc and kp after center-frequency normalization
x0_rescaled = list(x0_raw)
x0_rescaled[2] = round(x0_raw[2] * 0.489214 * math.sqrt(x0_raw[3]), 6)  # K_sc
x0_rescaled[7] = round(x0_raw[7] * math.sqrt(x0_raw[8]), 6)             # kp

TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
]

def run_one(tag, kp_val, kd_val, cand, freq, tg, tp):
    label = f"VER_{tag}_{freq}"
    env = dict(os.environ, KP_TASK_PITCH=str(kp_val), KD_TASK_PITCH=str(kd_val))
    cmd = [
        PY, "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        *[str(v) for v in cand],
        str(freq), str(tg), str(tp), label
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=3600)
    out_file = f"logs/candidates/{label}_{freq:.2f}Hz.json"
    if os.path.exists(out_file):
        d = json.load(open(out_file))
        try:
            os.remove(out_file)
        except OSError:
            pass
        try:
            os.remove(f"logs/candidates/{label}.json")
        except OSError:
            pass
        return tag, freq, d["sim_gain"], tg, d["gain_db_diff"], d["sim_phase"], tp, d["phase_err_deg"], d["score"], ""
    else:
        return tag, freq, -1, tg, -99, 0, tp, 180, 1000000.0, r.stderr[-200:]

jobs = []
for tag, kp_v, kd_v, cand in [
    ("KP0_Rescaled", 0.0, 0.0, x0_rescaled),
    ("KP10_2ndOrd_Rescaled", 10.24, 1.31, x0_rescaled),
]:
    for freq, tg, tp in TARGETS:
        jobs.append((tag, kp_v, kd_v, cand, freq, tg, tp))

with ThreadPoolExecutor(max_workers=12) as ex:
    res = list(ex.map(lambda a: run_one(*a), jobs))

for tag in ["KP0_Rescaled", "KP10_2ndOrd_Rescaled"]:
    print(f"\n=== {tag} ===")
    print(f"{'freq':>6} | {'sim_gain':>8} | {'tgt_gain':>8} | {'dB_err':>7} | {'sim_ph':>7} | {'tgt_ph':>7} | {'ph_err':>7} | {'score':>9}")
    print("-" * 80)
    tot = 0.0
    for t, f, sg, tg, gdb, sp, tp, pe, sc, err in sorted([r for r in res if r[0] == tag], key=lambda x: x[1]):
        tot += sc
        print(f"{f:6.3f} | {sg:8.2f} | {tg:8.2f} | {gdb:+7.2f} | {sp:7.1f} | {tp:7.1f} | {pe:7.1f} | {sc:9.1f} {err}")
    print(f"TOTAL SCORE ({tag}): {tot:.2f} pts")
