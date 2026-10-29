import json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

PY = sys.executable
best = json.load(open("logs/archive_run2_ratio_ton/W6_P24G1.json"))["params"]
K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, kd, ks = best
# Note: cmaes_worker_p22.py now takes K_ton directly as cand[0]
cand = [K_ton, p_ton, K_sc, z_sc / p_sc, p_sc, K_phas, p_phas, kp, z_ccr / p_ccr, p_ccr, kd, ks]
TARGET = {0.324: (86.55, -7.03), 5.583: (503.30, -201.38), 7.927: (572.69, -247.62)}

BASE_KP = 10.24
BASE_KD = 1.31
BASE_KI = 40.2786

def run(tag, kp_scale, ki_scale, freq):
    tg, tp = TARGET[freq]
    label = f"OUT_{tag}_{freq}"
    env = dict(
        os.environ,
        KP_TASK_PITCH=f"{BASE_KP * kp_scale:.6f}",
        KD_TASK_PITCH=f"{BASE_KD * kp_scale:.6f}",
    )
    ki_val = BASE_KI * ki_scale
    cmd = [
        PY, "cmaes_worker_p22.py",
        "22.0", "0.707", f"{ki_val:.6f}",
        *[str(v) for v in cand],
        str(freq), str(tg), str(tp), label
    ]
    subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=3600)
    out_file = f"logs/candidates/{label}_{freq:.2f}Hz.json"
    d = json.load(open(out_file))
    try:
        os.remove(out_file)
    except OSError:
        pass
    return tag, kp_scale, ki_scale, freq, d["gain_db_diff"], d["sim_gain"], d["sim_phase"], tp, d["score"]

jobs = [
    ("0.0x_KpKd", 0.0, 1.0, 0.324),
    ("0.5x_KpKd", 0.5, 1.0, 0.324),
    ("1.0x_Nom",  1.0, 1.0, 0.324),
    ("1.5x_KpKd", 1.5, 1.0, 0.324),
    ("0.0x_All",  0.0, 0.0, 0.324),

    ("0.0x_KpKd", 0.0, 1.0, 5.583),
    ("0.5x_KpKd", 0.5, 1.0, 5.583),
    ("1.0x_Nom",  1.0, 1.0, 5.583),
    ("1.5x_KpKd", 1.5, 1.0, 5.583),
    ("0.0x_All",  0.0, 0.0, 5.583),

    ("0.0x_KpKd", 0.0, 1.0, 7.927),
    ("0.5x_KpKd", 0.5, 1.0, 7.927),
    ("1.0x_Nom",  1.0, 1.0, 7.927),
    ("1.5x_KpKd", 1.5, 1.0, 7.927),
]

with ThreadPoolExecutor(max_workers=14) as ex:
    res = list(ex.map(lambda a: run(*a), jobs))

print(f"{'freq':>6} | {'Config':>10} | {'Kp':>6} | {'Kd':>5} | {'Ki':>6} | {'sim_gain':>8} | {'gain_dB_err':>11} | {'sim_ph':>7} | {'tgt_ph':>7} | {'ph_err':>7} | {'score':>9}")
print("-" * 108)
for tag, kps, kis, f, gdb, sg, sp, tp, sc in sorted(res, key=lambda r: (r[3], r[1], r[2])):
    pe = (sp - tp + 180) % 360 - 180
    print(f"{f:6.3f} | {tag:>10} | {BASE_KP*kps:6.2f} | {BASE_KD*kps:5.2f} | {BASE_KI*kis:6.2f} | {sg:8.3f} | {gdb:+11.2f} | {sp:7.1f} | {tp:7.1f} | {pe:+7.1f} | {sc:9.1f}")
