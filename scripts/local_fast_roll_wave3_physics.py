#!/usr/bin/env python3
"""Local Wave 3: 12-Core Physics-Guided Multi-Band Roll Polish Around pton_9.00."""

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

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from scripts.generate_standard_plot import run_axis_worker

G2_CLEAN_JSON = os.path.join(_ROOT, "logs", "cmaes_latec_only_18p_clean_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "local_fast_roll_wave3_physics.json")


def _eval_probe(args):
    """Top-level worker for Windows spawn multiprocessing."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.10g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    return idx, name, pdict, res


def build_wave3_probes(g2_clean):
    """Build 12 physics-guided multi-band Roll probes around pton_9.00."""
    specs = [
        ("P1_pton15", {"p_ton_roll": 15.0}),
        ("P2_pton22", {"p_ton_roll": 22.0}),
        ("P3_pton15_Kton1.5", {"p_ton_roll": 15.0, "K_ton_roll": 1.50}),
        ("P4_pton18_Kton2.2", {"p_ton_roll": 18.0, "K_ton_roll": 2.20}),
        ("P5_pton15_KpProp1.5", {"p_ton_roll": 15.0, "Kp_prop_roll": 1.50}),
        ("P6_pton15_KdTask0.6", {"p_ton_roll": 15.0, "Kd_task_roll": 0.60}),
        ("P7_pton15_Ksc3.0_tau16", {"p_ton_roll": 15.0, "K_sc_roll": 3.00, "tau_lead_pole_roll": 0.016}),
        ("P8_pton15_Kph10_pph4.5", {"p_ton_roll": 15.0, "K_phas_roll": 10.5, "p_phas_roll": 4.5}),
        (
            "P9_COMBO_A",
            {
                "p_ton_roll": 16.0, "K_ton_roll": 1.50,
                "Kp_prop_roll": 1.70, "Kd_task_roll": 0.75,
                "K_sc_roll": 3.10, "tau_lead_pole_roll": 0.016,
            },
        ),
        (
            "P10_COMBO_B",
            {
                "p_ton_roll": 18.0, "K_ton_roll": 1.80,
                "Kp_prop_roll": 1.30, "Kd_task_roll": 0.55,
                "K_sc_roll": 2.80, "z_sc_roll": 0.014, "tau_lead_pole_roll": 0.018,
            },
        ),
        (
            "P11_COMBO_C",
            {
                "p_ton_roll": 16.0, "K_ton_roll": 1.60,
                "Kp_prop_roll": 1.50, "Kd_task_roll": 0.70,
                "K_sc_roll": 3.00, "z_sc_roll": 0.015,
                "K_phas_roll": 11.0, "p_phas_roll": 4.5,
                "tau_lead_pole_roll": 0.017, "Kp_proprioception_roll": 0.036,
            },
        ),
        (
            "P12_COMBO_D",
            {
                "p_ton_roll": 20.0, "K_ton_roll": 1.90,
                "Kp_prop_roll": 1.40, "Kd_task_roll": 0.65,
                "K_sc_roll": 2.90, "z_sc_roll": 0.013, "p_sc_roll": 75.0,
                "K_phas_roll": 11.5, "p_phas_roll": 4.5,
                "Kp_task_roll": 1.50, "Ki_task_roll": 0.25,
                "tau_lead_pole_roll": 0.018, "Kp_proprioception_roll": 0.036,
            },
        ),
    ]
    probes = []
    for name, mods in specs:
        p = dict(g2_clean)
        p.update(mods)
        probes.append((name, p))
    return probes


def _print_freq_breakdown(title, rec):
    print(
        f"\n=== {title}: {rec['name']} (Bode={rec['bode_latec']:.2f}, "
        f"RollHF={rec['hf_noise_rms']:.3f} deg/s, CorrVel={rec['corr_vel']:+.3f}) ===",
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
    parser = argparse.ArgumentParser(description="Local Wave 3: Multi-Band Roll Polish Around pton_9.00")
    parser.add_argument("--workers", type=int, default=12, help="Local worker count")
    args = parser.parse_args()

    t0 = time.time()
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}

    probes = build_wave3_probes(g2_clean)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]

    print(f"=== Running Wave 3 ({len(tasks)} multi-band LatEC probes locally on {args.workers} workers) ===", flush=True)
    records = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_eval_probe, t) for t in tasks]
        for fut in as_completed(futs):
            idx, name, pdict, res = fut.result()
            records[idx] = {
                "name": name,
                "hf_noise_rms": float(res["hf_noise_rms"]),
                "bode_latec": float(res["classic_bode_score"]),
                "gain_loss_pts": float(res["gain_loss_pts"]),
                "phase_loss_pts": float(res["phase_loss_pts"]),
                "corr_vel": float(res["corr_vel"]),
                "xtalk_latec": float(res["pitch_crosstalk_penalty"]),
                "hf_penalty": float(res["hf_penalty"]),
                "aborted": bool(res["aborted"]),
                "freq_results": res.get("freq_results", {}),
                "params": pdict,
            }
            r = records[idx]
            print(
                f"  [done] {name:24s} RollHF={r['hf_noise_rms']:6.3f} Bode={r['bode_latec']:8.1f} Aborted={r['aborted']}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    sorted_recs = sorted(records, key=lambda r: (1 if r["aborted"] else 0, r["bode_latec"]))
    print(f"\n=== Wave 3 Summary ({dt:.1f}s) ===", flush=True)
    print(
        f"{'Name':24s} | {'RollHF':>7s} | {'LatEC Bode':>10s} | {'GainLoss':>9s} | "
        f"{'PhaseLoss':>9s} | {'CorrVel':>7s} | {'XTalk':>6s} | Aborted"
    )
    print("-" * 96)
    for r in sorted_recs:
        print(
            f"{r['name']:24s} | {r['hf_noise_rms']:7.3f} | {r['bode_latec']:10.2f} | "
            f"{r['gain_loss_pts']:9.1f} | {r['phase_loss_pts']:9.1f} | {r['corr_vel']:+7.3f} | "
            f"{r['xtalk_latec']:6.1f} | {r['aborted']}"
        )

    clean_top3 = [r for r in sorted_recs if (not r["aborted"]) and r["hf_noise_rms"] <= 3.5][:3]
    for idx, r in enumerate(clean_top3, 1):
        _print_freq_breakdown(f"TOP {idx} CLEAN PROBE", r)


if __name__ == "__main__":
    main()
