#!/usr/bin/env python3
"""Fast Local 16-Core Surgical Roll-Jitter Boundary Search (Wave 1: 16 parallel LatEC sims)."""

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

G35_JSON = os.path.join(_ROOT, "logs", "cmaes_3axis_18p_roll_champion.json")
G2_CLEAN_JSON = os.path.join(_ROOT, "logs", "cmaes_latec_only_18p_clean_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "local_fast_roll_wave1.json")

ROLL18_KEYS = [
    "K_sc_roll", "z_sc_roll", "p_sc_roll",
    "K_phas_roll", "z_phas_roll", "p_phas_roll",
    "K_ton_roll", "p_ton_roll",
    "Kp_prop_roll", "z_ccr_roll", "p_ccr_roll", "Kp_proprioception_roll",
    "Kp_task_roll", "Kd_task_roll", "Ki_task_roll",
    "tau_lead_pole_roll", "Kp_buckle_roll", "Kd_buckle_roll",
]


def _eval_probe(args):
    """Top-level worker for Windows spawn multiprocessing."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.10g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    return idx, name, pdict, res


def build_wave1_probes(g35, g2_clean):
    """Build 8 line-scan interpolations + 8 surgical Hopf-boundary probes."""
    probes = []
    for alpha in (0.00, 0.25, 0.45, 0.60, 0.72, 0.82, 0.90, 0.95):
        p = dict(g35)
        for k in ROLL18_KEYS:
            p[k] = (1.0 - alpha) * float(g2_clean[k]) + alpha * float(g35[k])
        probes.append((f"ALPHA_{alpha:.2f}", p))

    surgical = [
        ("G35_Ksc85", {"K_sc_roll": float(g35["K_sc_roll"]) * 0.85}),
        ("G35_Ksc75", {"K_sc_roll": float(g35["K_sc_roll"]) * 0.75}),
        ("G35_zsc75", {"z_sc_roll": float(g35["z_sc_roll"]) * 0.75}),
        ("G35_zsc60", {"z_sc_roll": float(g35["z_sc_roll"]) * 0.60}),
        ("G35_Ksc85_zsc75", {"K_sc_roll": float(g35["K_sc_roll"]) * 0.85, "z_sc_roll": float(g35["z_sc_roll"]) * 0.75}),
        ("G35_Ksc80_zsc70_zph0", {"K_sc_roll": float(g35["K_sc_roll"]) * 0.80, "z_sc_roll": float(g35["z_sc_roll"]) * 0.70, "z_phas_roll": 0.0}),
        ("G35_Ksc80_zsc70_tau25", {"K_sc_roll": float(g35["K_sc_roll"]) * 0.80, "z_sc_roll": float(g35["z_sc_roll"]) * 0.70, "tau_lead_pole_roll": 0.025}),
        ("G35_Ksc75_zsc65_psc70_tau25", {"K_sc_roll": float(g35["K_sc_roll"]) * 0.75, "z_sc_roll": float(g35["z_sc_roll"]) * 0.65, "p_sc_roll": 70.0, "tau_lead_pole_roll": 0.025}),
    ]
    for name, mods in surgical:
        p = dict(g35)
        p.update(mods)
        probes.append((name, p))
    return probes


def _print_freq_breakdown(title, rec):
    print(f"\n=== {title}: {rec['name']} (Bode={rec['bode_latec']:.2f}, RollHF={rec['hf_noise_rms']:.3f} deg/s, CorrVel={rec['corr_vel']:+.3f}) ===", flush=True)
    print("  Freq(Hz) | Gain_dB_Diff | GainLoss | PhaseErr(deg) | PhaseLoss")
    freqs = rec.get("frequencies", {})
    for k in sorted(freqs.keys(), key=lambda s: float(s)):
        f = freqs[k]
        print(
            f"  {float(f['freq_hz']):8.4f} | {float(f['gain_db_diff']):+12.3f} | "
            f"{float(f['gain_loss_pts']):8.1f} | {float(f['phase_err']):+13.2f} | {float(f['phase_loss_pts']):9.1f}",
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser(description="Fast Local 16-Core Surgical Roll-Jitter Boundary Search")
    parser.add_argument("--workers", type=int, default=16, help="Local worker count (<=16 on 20-core machine)")
    args = parser.parse_args()

    t0 = time.time()
    g35 = {k: float(v) for k, v in json.load(open(G35_JSON, "r", encoding="utf-8"))["params"].items()}
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}

    probes = build_wave1_probes(g35, g2_clean)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]

    print(f"=== Running {len(tasks)} parallel LatEC probes locally on {args.workers} workers ===", flush=True)
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
                "frequencies": res.get("frequencies", {}),
                "params": pdict,
            }
            r = records[idx]
            print(
                f"  [done] {name:30s} RollHF={r['hf_noise_rms']:6.3f} Bode={r['bode_latec']:8.1f} Aborted={r['aborted']}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    sorted_recs = sorted(records, key=lambda r: (1 if r["aborted"] else 0, r["bode_latec"]))
    print(f"\n=== Wave 1 Summary ({dt:.1f}s) ===", flush=True)
    print(f"{'Name':30s} | {'RollHF':>7s} | {'LatEC Bode':>10s} | {'GainLoss':>9s} | {'PhaseLoss':>9s} | {'CorrVel':>7s} | {'XTalk':>6s} | Aborted")
    print("-" * 102)
    for r in sorted_recs:
        print(
            f"{r['name']:30s} | {r['hf_noise_rms']:7.3f} | {r['bode_latec']:10.2f} | "
            f"{r['gain_loss_pts']:9.1f} | {r['phase_loss_pts']:9.1f} | {r['corr_vel']:+7.3f} | "
            f"{r['xtalk_latec']:6.1f} | {r['aborted']}"
        )

    clean_recs = [r for r in sorted_recs if (not r["aborted"]) and r["hf_noise_rms"] <= 3.5]
    mid_recs = [r for r in sorted_recs if (not r["aborted"]) and 3.5 < r["hf_noise_rms"] <= 6.0]
    if clean_recs:
        _print_freq_breakdown("BEST CLEAN (RollHF <= 3.5)", clean_recs[0])
    if mid_recs:
        _print_freq_breakdown("BEST MID (3.5 < RollHF <= 6.0)", mid_recs[0])


if __name__ == "__main__":
    main()
