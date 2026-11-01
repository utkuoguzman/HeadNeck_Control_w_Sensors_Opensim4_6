import os
import sys
import time
import json
import math
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

TARGETS = [
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
    (1.099, 319.74, -109.23),
    (0.783, 236.93,  -96.96),
    (0.324,  86.55,   -7.03)
]

# Candidate A: W12_P24G1 with High-Freq Asymptote Preserved (K_phas*p_phas = 84.47: K_phas=4.2235, p_phas=20.0, r_lead_phas=0.25 -> z_phas=5.0 rad/s)
CAND_A = [
    0.5170257283876958,   # K_ton
    76.87482720459431,    # p_ton
    0.1592989238219975,   # K_sc
    0.16555515026733947,  # r_lead_sc
    81.34613726696269,    # p_sc
    4.2235,               # K_phas
    20.0,                 # p_phas
    0.2973811747784371,   # kp
    0.052390568504977794, # r_lead_ccr
    196.2636628121668,    # p_ccr
    0.7058020211457222,   # k_dyn
    0.9150562742348893,   # k_stat
    0.25,                 # r_lead_phas (z_phas = 5.0 rad/s)
    3.7248722911664895,   # Kp_task_pitch
    0.10975307350470226,  # Kd_task_pitch
    37.72059916102091     # Ki_task_pitch
]

# Candidate B: W4_P24G3 seed with High-Freq Asymptote Preserved (K_phas*p_phas = 87.89: K_phas=3.662, p_phas=24.0, r_lead_phas=0.20 -> z_phas=4.8 rad/s)
CAND_B = [
    0.5487639837300085,   # K_ton
    77.69846349041005,    # p_ton
    0.14095118813479413,  # K_sc
    0.1867947165475933,   # r_lead_sc
    79.33871912024726,    # p_sc
    3.662,                # K_phas
    24.0,                 # p_phas
    0.3123696401245463,   # kp
    0.05559467980337742,  # r_lead_ccr
    187.08409601266197,   # p_ccr
    0.6616603740787023,   # k_dyn
    0.9660773095711337,   # k_stat
    0.20,                 # r_lead_phas (z_phas = 4.8 rad/s)
    1.5,                  # Kp_task_pitch
    0.4,                  # Kd_task_pitch
    36.0                  # Ki_task_pitch
]

def run_one_freq(label, cand, freq, t_gain, t_phase):
    t0 = time.time()
    cmd = [
        sys.executable, "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        *[str(v) for v in cand],
        str(freq), str(t_gain), str(t_phase),
        label
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    dt = time.time() - t0
    lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
    score = float(lines[-1]) if (res.returncode == 0 and lines) else 1000000.0
    res_file = os.path.join("logs", "candidates", f"{label}_{freq:.2f}Hz.json")
    details = {}
    if os.path.exists(res_file):
        with open(res_file, "r", encoding="utf-8") as fp:
            details = json.load(fp)
    print(f"[{label}] {freq:.3f} Hz finished in {dt:.1f}s -> Score: {score:.1f} (dB: {details.get('gain_db_diff', 0):+.2f}, Phase: {details.get('sim_phase', 0):.1f} vs {t_phase:.1f}, err: {details.get('phase_err_deg', 0):.1f} deg)", flush=True)
    return label, freq, score, dt, details

if __name__ == "__main__":
    t_start = time.time()
    tasks = []
    with ThreadPoolExecutor(max_workers=12) as ex:
        for freq, tg, tp in TARGETS:
            tasks.append(ex.submit(run_one_freq, "TEST_LEAD_A", CAND_A, freq, tg, tp))
            tasks.append(ex.submit(run_one_freq, "TEST_LEAD_B", CAND_B, freq, tg, tp))
        results = [f.result() for f in as_completed(tasks)]

    for lbl in ["TEST_LEAD_A", "TEST_LEAD_B"]:
        sub = sorted([r for r in results if r[0] == lbl], key=lambda x: x[1])
        tot = sum(r[2] for r in sub)
        sum_dt = sum(r[3] for r in sub)
        print(f"\n=== {lbl} TOTAL SCORE: {tot:.2f} pts (Sum of 6 freq CPU time: {sum_dt/60.0:.2f} min) ===")
        for _, freq, sc, dt, det in sub:
            print(f"  {freq:5.3f} Hz | Gain: {det.get('sim_gain',0):6.1f} vs {det.get('target_gain',0):6.1f} ({det.get('gain_db_diff',0):+5.2f} dB) | Phase: {det.get('sim_phase',0):6.1f} vs {det.get('target_phase',0):6.1f} (err {det.get('phase_err_deg',0):5.1f} deg) | Score: {sc:7.1f} | {dt:5.1f}s")
