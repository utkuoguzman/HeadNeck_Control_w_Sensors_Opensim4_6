#!/usr/bin/env python3
"""Option 1 Step 2 (Wave 6): 16-Probe Gen 35-Anchored Clean Stiffness Wave."""

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from scripts.generate_standard_plot import run_axis_worker

G35_JSON = os.path.join(_ROOT, "logs", "cmaes_3axis_18p_roll_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "local_wave6_g35_stiff_clean.json")


def _eval_probe(args):
    """Top-level worker calling unmodified run_axis_worker on LatEC (all 28 booleans TRUE)."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    assert not res["false_booleans"], f"Disabled booleans in {name}: {res['false_booleans']}"
    t_arr = np.asarray(res["t"], dtype=float)
    y_arr = np.asarray(res["main_angle"], dtype=float)
    m_post = t_arr >= 5.0
    if np.sum(m_post) >= 500:
        r_smooth = np.convolve(y_arr[m_post], np.ones(500, dtype=float) / 500.0, mode="valid")
        lowfreq_ptp = float(np.ptp(r_smooth))
    elif np.sum(m_post) >= 2:
        lowfreq_ptp = float(np.ptp(y_arr[m_post]))
    else:
        lowfreq_ptp = 0.0
    mean_post = float(np.mean(y_arr[m_post])) if np.any(m_post) else 0.0

    rec = {
        "name": name,
        "bool_banner": res["bool_banner"],
        "aborted": bool(res["aborted"]),
        "abort_time_s": float(res["abort_time_s"]),
        "bode_latec": float(res["classic_bode_score"]),
        "gain_loss_pts": float(res["gain_loss_pts"]),
        "phase_loss_pts": float(res["phase_loss_pts"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "corr_vel": float(res["corr_vel"]),
        "xtalk_latec": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "lowfreq_ptp_deg": round(lowfreq_ptp, 4),
        "mean_post5s_deg": round(mean_post, 4),
        "freq_results": res.get("freq_results", {}),
        "params": pdict,
    }
    return idx, rec


def build_wave6_probes(g35):
    """Build 16 Gen 35-anchored clean stiffness probes incorporating Step 1 findings."""
    specs = [
        ("W6_01_psc35_zsc12_Ksc38", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.012, "K_sc_roll": 3.8,
        }),
        ("W6_02_psc30_zsc10_Ksc35", {
            "p_sc_roll": 30.0, "z_sc_roll": 0.010, "K_sc_roll": 3.5,
        }),
        ("W6_03_psc25_zsc08_Ksc32", {
            "p_sc_roll": 25.0, "z_sc_roll": 0.008, "K_sc_roll": 3.2,
        }),
        ("W6_04_psc45_zsc12_Ksc34", {
            "p_sc_roll": 45.0, "z_sc_roll": 0.012, "K_sc_roll": 3.4,
        }),
        ("W6_05_psc55_zsc06_Ksc42", {
            "p_sc_roll": 55.0, "z_sc_roll": 0.006, "K_sc_roll": 4.2,
        }),
        ("W6_06_psc45_zsc04_Ksc45_zph0", {
            "p_sc_roll": 45.0, "z_sc_roll": 0.004, "K_sc_roll": 4.5, "z_phas_roll": 0.0,
        }),
        ("W6_07_psc35_zsc10_Ksc36_pton9", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.010, "K_sc_roll": 3.6, "p_ton_roll": 9.0, "K_ton_roll": 1.6,
        }),
        ("W6_08_psc35_zsc10_Ksc36_Kph4", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.010, "K_sc_roll": 3.6, "K_phas_roll": 4.0, "z_phas_roll": 0.0,
        }),
        ("W6_09_psc35_zsc10_Ksc36_Kph0", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.010, "K_sc_roll": 3.6,
            "K_phas_roll": 0.5, "z_phas_roll": 0.0, "p_ton_roll": 9.0, "K_ton_roll": 2.0,
        }),
        ("W6_10_psc35_zsc10_Ksc36_stiffTask", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.010, "K_sc_roll": 3.6,
            "Kp_task_roll": 3.2, "Kd_task_roll": 2.3, "Ki_task_roll": 0.10, "p_ton_roll": 9.0,
        }),
        ("W6_11_psc35_zsc10_Ksc36_buckle25", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.010, "K_sc_roll": 3.6,
            "Kp_buckle_roll": 2.6, "Kd_buckle_roll": 0.40,
        }),
        ("W6_12_psc35_zsc10_Ksc36_tau30", {
            "p_sc_roll": 35.0, "z_sc_roll": 0.010, "K_sc_roll": 3.6,
            "tau_lead_pole_roll": 0.030, "Kp_proprioception_roll": 0.010,
        }),
        # Step 1 (D08 Ki0 + D09 HighBuckle + D11 SoftLead + D12 StiffLowFreq) combined off Gen 35
        ("W6_13_STEP1_COMBO_CLEAN", {
            "p_sc_roll": 38.0, "z_sc_roll": 0.010, "K_sc_roll": 3.8,
            "tau_lead_pole_roll": 0.028,
            "Ki_task_roll": 0.0, "Kp_task_roll": 2.8, "Kd_task_roll": 2.2,
            "p_ton_roll": 9.0, "K_ton_roll": 2.0,
            "Kp_buckle_roll": 2.6, "Kd_buckle_roll": 0.38,
            "Kp_proprioception_roll": 0.008, "z_phas_roll": 0.0,
        }),
        # +1e-9 reproducibility twin of W6_13_STEP1_COMBO_CLEAN
        ("W6_14_STEP1_COMBO_EPS1E9", {
            "p_sc_roll": 38.0, "z_sc_roll": 0.010, "K_sc_roll": 3.8 * (1.0 + 1e-9),
            "tau_lead_pole_roll": 0.028,
            "Ki_task_roll": 0.0, "Kp_task_roll": 2.8, "Kd_task_roll": 2.2,
            "p_ton_roll": 9.0, "K_ton_roll": 2.0,
            "Kp_buckle_roll": 2.6, "Kd_buckle_roll": 0.38,
            "Kp_proprioception_roll": 0.008, "z_phas_roll": 0.0,
        }),
        # Near-Gen 35 canal gain damped via soft tau_lead_pole + lower z_sc + high buckle + zero Ki
        ("W6_15_G35_SOFTLEAD_KSC46", {
            "p_sc_roll": 75.0, "z_sc_roll": 0.022, "K_sc_roll": 4.6,
            "tau_lead_pole_roll": 0.032,
            "Ki_task_roll": 0.02,
            "Kp_buckle_roll": 2.6, "Kd_buckle_roll": 0.40,
            "z_phas_roll": 0.0,
        }),
        # High canal + reduced phasic gain (K_phas_roll=8.0) + Step 1 stabilization
        ("W6_16_KSC44_KPH8_STEP1", {
            "p_sc_roll": 55.0, "z_sc_roll": 0.014, "K_sc_roll": 4.4,
            "K_phas_roll": 8.0, "z_phas_roll": 0.0,
            "tau_lead_pole_roll": 0.028,
            "Ki_task_roll": 0.0, "Kp_task_roll": 2.6, "Kd_task_roll": 2.0,
            "p_ton_roll": 9.0, "K_ton_roll": 1.8,
            "Kp_buckle_roll": 2.6, "Kd_buckle_roll": 0.38,
        }),
    ]
    probes = []
    for name, mods in specs:
        p = dict(g35)
        p.update(mods)
        probes.append((name, p))
    return probes


def main():
    parser = argparse.ArgumentParser(description="Option 1 Step 2: 16-Probe Gen 35-Anchored Clean Stiffness Wave")
    parser.add_argument("--workers", type=int, default=16, help="Local worker processes")
    args = parser.parse_args()

    t0 = time.time()
    g35 = {k: float(v) for k, v in json.load(open(G35_JSON, "r", encoding="utf-8"))["params"].items()}
    probes = build_wave6_probes(g35)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]

    print(f"=== Running Wave 6 ({len(tasks)} Gen35-anchored clean probes on {args.workers} workers) ===", flush=True)
    records = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_eval_probe, t) for t in tasks]
        for fut in as_completed(futs):
            idx, rec = fut.result()
            records[idx] = rec
            print(
                f"  [done] {rec['name']:30s} RollHF={rec['hf_noise_rms']:6.3f} Bode={rec['bode_latec']:9.1f} "
                f"LowPtP={rec['lowfreq_ptp_deg']:6.3f} Mean5s={rec['mean_post5s_deg']:+6.3f} Corr={rec['corr_vel']:+6.3f}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    print(f"\n=== Wave 6 Summary ({dt:.1f}s) ===", flush=True)
    hdr = f"{'Name':30s} | {'RollHF':>6s} | {'LatEC Bode':>10s} | {'GainLoss':>8s} | {'PhsLoss':>8s} | {'LowPtP':>6s} | {'Mean5s':>7s} | {'CorrVel':>7s} | {'XTalk':>6s}"
    print(hdr)
    print("-" * len(hdr))
    for r in records:
        print(
            f"{r['name']:30s} | {r['hf_noise_rms']:6.3f} | {r['bode_latec']:10.2f} | {r['gain_loss_pts']:8.1f} | "
            f"{r['phase_loss_pts']:8.1f} | {r['lowfreq_ptp_deg']:6.3f} | {r['mean_post5s_deg']:+7.3f} | "
            f"{r['corr_vel']:+7.3f} | {r['xtalk_latec']:6.1f}"
        )


if __name__ == "__main__":
    main()
