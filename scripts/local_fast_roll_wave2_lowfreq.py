#!/usr/bin/env python3
"""Local Wave 2: 16-Core Surgical Low-Frequency Sweep Inside the Clean ~1 deg/s Basin."""

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
OUT_JSON = os.path.join(_ROOT, "logs", "local_fast_roll_wave2_lowfreq.json")


def _eval_probe(args):
    """Top-level worker for Windows spawn multiprocessing."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.10g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    return idx, name, pdict, res


def build_wave2_probes(g2_clean):
    """Build 16 surgical low-frequency probes (K_ton_roll, p_ton_roll, Ki_task_roll, Kp_task_roll)."""
    specs = [
        ("BASE_G2", {}),
        ("Kton_0.20", {"K_ton_roll": 0.20}),
        ("Kton_0.40", {"K_ton_roll": 0.40}),
        ("Kton_1.40", {"K_ton_roll": 1.40}),
        ("Kton_2.20", {"K_ton_roll": 2.20}),
        ("Kton_3.20", {"K_ton_roll": 3.20}),
        ("pton_0.60", {"p_ton_roll": 0.60}),
        ("pton_1.20", {"p_ton_roll": 1.20}),
        ("pton_5.00", {"p_ton_roll": 5.00}),
        ("pton_9.00", {"p_ton_roll": 9.00}),
        ("Ki_0.05", {"Ki_task_roll": 0.05}),
        ("Ki_0.15", {"Ki_task_roll": 0.15}),
        ("Ki_1.20", {"Ki_task_roll": 1.20}),
        ("Ki_2.20", {"Ki_task_roll": 2.20}),
        ("KpTask_0.40", {"Kp_task_roll": 0.40}),
        ("KpTask_2.50", {"Kp_task_roll": 2.50}),
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
    parser = argparse.ArgumentParser(description="Local Wave 2: Low-Frequency Sweep Inside Clean Basin")
    parser.add_argument("--workers", type=int, default=16, help="Local worker count (<=16 on 20-core machine)")
    args = parser.parse_args()

    t0 = time.time()
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}

    probes = build_wave2_probes(g2_clean)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]

    print(f"=== Running Wave 2 ({len(tasks)} low-freq LatEC probes locally on {args.workers} workers) ===", flush=True)
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
                f"  [done] {name:16s} RollHF={r['hf_noise_rms']:6.3f} Bode={r['bode_latec']:8.1f} Aborted={r['aborted']}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    sorted_recs = sorted(records, key=lambda r: (1 if r["aborted"] else 0, r["bode_latec"]))
    print(f"\n=== Wave 2 Summary ({dt:.1f}s) ===", flush=True)
    print(
        f"{'Name':16s} | {'RollHF':>7s} | {'LatEC Bode':>10s} | {'GainLoss':>9s} | "
        f"{'PhaseLoss':>9s} | {'CorrVel':>7s} | {'XTalk':>6s} | Aborted"
    )
    print("-" * 88)
    for r in sorted_recs:
        print(
            f"{r['name']:16s} | {r['hf_noise_rms']:7.3f} | {r['bode_latec']:10.2f} | "
            f"{r['gain_loss_pts']:9.1f} | {r['phase_loss_pts']:9.1f} | {r['corr_vel']:+7.3f} | "
            f"{r['xtalk_latec']:6.1f} | {r['aborted']}"
        )

    base_rec = next(r for r in records if r["name"] == "BASE_G2")
    _print_freq_breakdown("BASE_G2", base_rec)
    top2 = [r for r in sorted_recs if r["name"] != "BASE_G2" and not r["aborted"]][:2]
    for idx, r in enumerate(top2, 1):
        _print_freq_breakdown(f"TOP {idx} LOW-BODE PROBE", r)


if __name__ == "__main__":
    main()
