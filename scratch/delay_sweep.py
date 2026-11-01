import json, os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

PY = sys.executable
src = open("cmaes_worker_p22.py", encoding="utf-8", newline="").read()
# Clone the worker with a DELAY_SCALE env knob (original worker + running campaign untouched)
src = src.replace("import opensim as osim", 'import os as _os\r\nDS = float(_os.environ.get("DELAY_SCALE", "1.0"))\r\nimport opensim as osim', 1)
for name, val in [("delay_vcr", "0.010"), ("delay_ccr", "0.015"), ("delay_smith_proprio", "0.007"), ("delay_afferent_proprio", "0.007")]:
    old = f'setValueDouble({val}, controller.updPropertyByName("{name}"))'
    assert old in src, name
    src = src.replace(old, f'setValueDouble({val} * DS, controller.updPropertyByName("{name}"))')
open("scratch/cmaes_worker_p22_delay.py", "w", encoding="utf-8", newline="").write(src)

best = json.load(open("logs/candidates/W6_P24G1.json"))["params"]
K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, kd, ks = best
cand = [K_ton / K_sc, p_ton, K_sc, z_sc / p_sc, p_sc, K_phas, p_phas, kp, z_ccr / p_ccr, p_ccr, kd, ks]
TARGET = {5.583: (503.30, -201.38), 7.927: (572.69, -247.62), 0.324: (86.55, -7.03)}

def run(scale, freq):
    tg, tp = TARGET[freq]
    label = f"DLY_{scale}_{freq}"
    env = dict(os.environ, DELAY_SCALE=str(scale))
    cmd = [PY, "scratch/cmaes_worker_p22_delay.py", "22.0", "0.707", "40.2786", *[str(v) for v in cand], str(freq), str(tg), str(tp), label]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=3600)
    d = json.load(open(f"logs/candidates/{label}_{freq:.2f}Hz.json"))
    return scale, freq, d["gain_db_diff"], d["sim_phase"], tp, d["score"]

jobs = [(s, f) for f in (5.583, 7.927) for s in (0.0, 0.5, 1.0, 1.5)]
with ThreadPoolExecutor(max_workers=8) as ex:
    res = list(ex.map(lambda a: run(*a), jobs))
print(f"{'freq':>6} {'delay x':>8} {'gain err dB':>12} {'sim phase':>10} {'tgt phase':>10} {'phase err':>10} {'score':>9}")
for s, f, g, sp, tp, sc in sorted(res, key=lambda r: (r[1], r[0])):
    pe = (sp - tp + 180) % 360 - 180
    print(f"{f:6.3f} {s:8.1f} {g:12.2f} {sp:10.1f} {tp:10.1f} {pe:10.1f} {sc:9.0f}")

