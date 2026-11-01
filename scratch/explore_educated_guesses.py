import subprocess
import json
import os
import sys
import math
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed

TARGETS = {
    1.099: (319.74, -109.23),
    2.446: (423.34, -161.34),
    5.583: (503.30, -201.38),
    7.927: (572.69, -247.62),
}

# 3 focused educated guesses:
CANDIDATES = {
    "Cand_Happee_Table1": [
        3.636,  # r_ton (K_ton = 2.0 / 0.55 = 3.636 -> K_ton = 2.0)
        12.0,   # p_ton
        0.55,   # K_sc (Happee Table 1 exact)
        0.20,   # r_lead_sc
        50.0,   # p_sc
        0.87,   # K_phas (Happee Table 1 exact)
        35.0,   # p_phas
        0.81,   # kp (Happee Table 1 exact)
        0.25,   # r_lead_ccr
        45.0,   # p_ccr
        0.30,   # k_dyn
        0.30    # k_stat
    ],
    "Cand_W16_Gen1Best": [
        1.312,  # r_ton (K_ton = 4.58)
        6.815,  # p_ton
        3.493,  # K_sc
        0.187,  # r_lead_sc
        62.187, # p_sc
        1.550,  # K_phas
        33.001, # p_phas
        0.706,  # kp
        0.317,  # r_lead_ccr
        51.624, # p_ccr
        0.264,  # k_dyn
        0.582   # k_stat
    ],
    "Cand_HighLead_Tuned": [
        2.00,   # r_ton (K_ton = 2.4)
        10.0,   # p_ton
        1.20,   # K_sc
        0.15,   # r_lead_sc (z_sc = 9 -> high phase lead)
        60.0,   # p_sc
        1.20,   # K_phas
        35.0,   # p_phas
        0.65,   # kp
        0.18,   # r_lead_ccr (z_ccr = 10.8 -> high phase lead)
        60.0,   # p_ccr
        0.25,   # k_dyn
        0.35    # k_stat
    ]
}

def evaluate_single(cand_name, params, freq):
    t_gain, t_phase = TARGETS[freq]
    worker_label = f"BS2_{cand_name}_{freq:.2f}Hz"
    
    cmd = [
        r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        *[str(v) for v in params],
        str(freq), str(t_gain), str(t_phase),
        worker_label
    ]
    
    try:
        # Generous timeout of 350s
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=350)
        if res.returncode == 0:
            lines = [l.strip() for l in res.stdout.strip().split("\n") if l.strip()]
            score = float(lines[-1])
            res_file = os.path.join("logs", "candidates", f"{worker_label}_{freq:.2f}Hz.json")
            if os.path.exists(res_file):
                with open(res_file, "r") as f:
                    d = json.load(f)
                return cand_name, freq, d
            return cand_name, freq, {"score": score}
        else:
            print(f"[FAIL] {cand_name} on {freq}Hz: {res.stderr}", file=sys.stderr)
            return cand_name, freq, None
    except Exception as e:
        print(f"[ERROR] {cand_name} on {freq}Hz: {e}", file=sys.stderr)
        return cand_name, freq, None

def main():
    # Test on 2.446 Hz and 5.583 Hz (6 total evaluations, well within 14 cores!)
    test_freqs = [2.446, 5.583]
    print(f"Launching 6 concurrent evaluations (3 candidates x 2 freqs) across 14 cores...")
    sys.stdout.flush()
    
    tasks = []
    with ThreadPoolExecutor(max_workers=6) as executor:
        for cname, params in CANDIDATES.items():
            for freq in test_freqs:
                tasks.append(executor.submit(evaluate_single, cname, params, freq))
        
        results = []
        for future in as_completed(tasks):
            cname, freq, res = future.result()
            results.append((cname, freq, res))
            
    print("\n" + "=" * 115)
    print(f"{'Candidate':<22} | {'Freq [Hz]':<10} | {'Sim Gain':<10} | {'Tgt Gain':<10} | {'Gain Diff':<10} | {'Sim Phase':<10} | {'Tgt Phase':<10} | {'Phase Diff':<11} | {'Score':<8}")
    print("-" * 115)
    
    results.sort(key=lambda x: (x[0], x[1]))
    totals = {}
    for cname, freq, d in results:
        if d:
            g_diff = d.get('gain_db_diff', 0.0)
            p_diff = d.get('phase_err_deg', 0.0)
            sg = d.get('sim_gain', 0.0)
            tg = d.get('target_gain', 0.0)
            sp = d.get('sim_phase', 0.0)
            tp = d.get('target_phase', 0.0)
            score = d.get('score', 0.0)
            totals[cname] = totals.get(cname, 0.0) + score
            print(f"{cname:<22} | {freq:<10.3f} | {sg:<10.1f} | {tg:<10.1f} | {g_diff:+10.2f}dB | {sp:<10.1f}° | {tp:<10.1f}° | {p_diff:11.1f}° | {score:<8.1f}")
        else:
            print(f"{cname:<22} | {freq:<10.3f} | FAILED")
            
    print("-" * 115)
    print("TOTAL SCORE ON TEST FREQUENCIES (2.45 Hz & 5.58 Hz):")
    for cname, tot in sorted(totals.items(), key=lambda x: x[1]):
        print(f"  {cname:<22}: {tot:.1f} pts")
    sys.stdout.flush()

if __name__ == "__main__":
    main()

