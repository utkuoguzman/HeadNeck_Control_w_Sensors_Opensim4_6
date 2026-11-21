#!/usr/bin/env python3
"""Local Wave 4: 4-Worker Diagnostic (Vestibular/Spindle Deadbands) + Hybrid Clean Probe."""

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

G2_CLEAN_JSON = os.path.join(_ROOT, "logs", "cmaes_latec_only_18p_clean_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "local_wave4_diag.json")


def _eval_probe(args):
    """Top-level worker for Windows spawn multiprocessing."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.10g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    t_arr = np.asarray(res["t"], dtype=float)
    y_arr = np.asarray(res["main_angle"], dtype=float)
    m_post = t_arr >= 5.0
    if np.sum(m_post) >= 10:
        roll_mean_post5s = float(np.mean(y_arr[m_post]))
        roll_slope = float(np.polyfit(t_arr[m_post], y_arr[m_post], 1)[0])
    else:
        roll_mean_post5s = 0.0
        roll_slope = 0.0
    res_summary = {
        "name": name,
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "bode_latec": float(res["classic_bode_score"]),
        "gain_loss_pts": float(res["gain_loss_pts"]),
        "phase_loss_pts": float(res["phase_loss_pts"]),
        "corr_vel": float(res["corr_vel"]),
        "roll_mean_post5s": round(roll_mean_post5s, 4),
        "roll_slope_deg_per_s": round(roll_slope, 4),
        "xtalk_latec": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "aborted": bool(res["aborted"]),
        "freq_results": res.get("freq_results", {}),
        "params": pdict,
    }
    return idx, res_summary


def build_wave4_probes(g2_clean):
    """Build 4 targeted diagnostic/hybrid probes off pton_9.00."""
    pton9 = dict(g2_clean)
    pton9["p_ton_roll"] = 9.00

    specs = [
        (
            "DIAG_A_NO_VEST_DB",
            {
                "thresh_vest_tilt": 0.0,
                "thresh_vest_scc": 0.0,
                "thresh_vest_oto": 0.0,
            },
        ),
        (
            "DIAG_B_NO_SPINDLE_DB",
            {
                "thresh_spindle_ia": 0.0,
                "thresh_spindle_ii": 0.0,
            },
        ),
        (
            "DIAG_C_NO_ALL_DB",
            {
                "thresh_vest_tilt": 0.0,
                "thresh_vest_scc": 0.0,
                "thresh_vest_oto": 0.0,
                "thresh_spindle_ia": 0.0,
                "thresh_spindle_ii": 0.0,
            },
        ),
        (
            "DIAG_D_HYBRID_2",
            {
                "K_sc_roll": 3.35,
                "z_sc_roll": 0.016,
                "p_sc_roll": 65.0,
                "K_phas_roll": 13.2,
                "p_phas_roll": 3.3,
                "Kp_prop_roll": 1.95,
                "Kd_task_roll": 0.90,
                "tau_lead_pole_roll": 0.017,
                "Kp_proprioception_roll": 0.036,
            },
        ),
    ]
    probes = []
    for name, mods in specs:
        p = dict(pton9)
        p.update(mods)
        probes.append((name, p))
    return probes


def _print_freq_breakdown(rec):
    print(
        f"\n=== {rec['name']} (Bode={rec['bode_latec']:.2f}, RollHF={rec['hf_noise_rms']:.3f} deg/s, "
        f"CorrVel={rec['corr_vel']:+.3f}, MeanPost5s={rec['roll_mean_post5s']:+.3f}, Slope={rec['roll_slope_deg_per_s']:+.4f}) ===",
        flush=True,
    )
    print(
        "  Freq(Hz) |  SimGain |  TgtGain | Gain_dB_Diff | GainLoss | SimPhase | TgtPhase | PhaseErr | PhaseLoss"
    )
    freqs = rec.get("freq_results", {})
    for k in sorted(freqs.keys(), key=lambda s: float(s)):
        f = freqs[k]
        print(
            f"  {float(f['freq_hz']):8.4f} | {float(f['sim_gain']):8.2f} | {float(f['target_gain']):8.2f} | "
            f"{float(f['gain_db_diff']):+12.3f} | {float(f['gain_loss_pts']):8.1f} | "
            f"{float(f['sim_phase']):8.2f} | {float(f['target_phase']):8.2f} | "
            f"{float(f['phase_err']):+8.2f} | {float(f['phase_loss_pts']):9.1f}",
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser(description="Local Wave 4: 4-Worker Deadband Diagnostic & Hybrid")
    parser.add_argument("--workers", type=int, default=4, help="Local worker count")
    args = parser.parse_args()

    t0 = time.time()
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}
    probes = build_wave4_probes(g2_clean)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]

    print(f"=== Running Wave 4 ({len(tasks)} diag/hybrid LatEC probes locally on {args.workers} workers) ===", flush=True)
    records = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_eval_probe, t) for t in tasks]
        for fut in as_completed(futs):
            idx, rec = fut.result()
            records[idx] = rec
            print(
                f"  [done] {rec['name']:22s} RollHF={rec['hf_noise_rms']:6.3f} Bode={rec['bode_latec']:8.1f} "
                f"Mean5s={rec['roll_mean_post5s']:+.3f} Slope={rec['roll_slope_deg_per_s']:+.4f}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    print(f"\n=== Wave 4 Summary ({dt:.1f}s) ===", flush=True)
    print(
        f"{'Name':22s} | {'RollHF':>6s} | {'LatEC Bode':>10s} | {'GainLoss':>8s} | {'PhaseLoss':>9s} | "
        f"{'CorrVel':>7s} | {'Mean5s':>7s} | {'Slope':>7s} | {'XTalk':>6s}"
    )
    print("-" * 102)
    for r in records:
        print(
            f"{r['name']:22s} | {r['hf_noise_rms']:6.3f} | {r['bode_latec']:10.2f} | {r['gain_loss_pts']:8.1f} | "
            f"{r['phase_loss_pts']:9.1f} | {r['corr_vel']:+7.3f} | {r['roll_mean_post5s']:+7.3f} | "
            f"{r['roll_slope_deg_per_s']:+7.4f} | {r['xtalk_latec']:6.1f}"
        )

    for r in records:
        _print_freq_breakdown(r)


if __name__ == "__main__":
    main()
