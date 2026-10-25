import subprocess
import sys
import json
import numpy as np

# Test candidate with more compliant, physiological parameters:
# r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat

candidates = {
    "W1_P22G12 (Stiff/Current)": [
        1.062, 11.227, 45.400, 0.064, 72.931, 8.434, 31.045, 1.413, 0.538, 59.572, 0.929, 0.957
    ],
    "Compliant_Test_1 (Low Gains)": [
        0.80, 15.0, 20.0, 0.25, 50.0, 5.0, 40.0, 0.60, 0.35, 45.0, 0.50, 0.50
    ],
    "Compliant_Test_2 (Medium Gains)": [
        0.90, 18.0, 28.0, 0.30, 60.0, 6.0, 45.0, 0.80, 0.40, 50.0, 0.60, 0.70
    ]
}

freqs = [
    (0.324, 86.55, -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

for name, params in candidates.items():
    print(f"\n{'='*60}\nTesting Candidate: {name}\n{'='*60}")
    # Let's test on 0.783 Hz first
    f, tg, tp = freqs[1]
    cmd = [
        r"C:\Users\User\miniforge3\envs\opensim_env\python.exe", "cmaes_worker_p22.py",
        "22.0", "0.707", "40.2786",
        *[str(v) for v in params],
        str(f), str(tg), str(tp),
        f"TEST_{name[:10]}"
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    # Read saved candidate file
    res_file = f"logs/candidates/TEST_{name[:10]}_{f:.2f}Hz.json"
    try:
        with open(res_file) as fp:
            d = json.load(fp)
        print(f"Freq: {f} Hz | Target Gain: {tg:.1f} deg/m | Sim Gain: {d['sim_gain']:.1f} deg/m (dB diff: {d['gain_db_diff']:+.2f} dB)")
        print(f"Target Phase: {tp:.1f}° | Sim Phase: {d['sim_phase']:.1f}° (diff: {d['phase_err_deg']:.1f}°)")
        print(f"Score: {d['score']:.1f} (Gain Loss: {d['gain_loss']:.1f}, Phase Loss: {d['phase_loss']:.1f})")
    except Exception as e:
        print("Error reading result:", e, res.stdout, res.stderr)

