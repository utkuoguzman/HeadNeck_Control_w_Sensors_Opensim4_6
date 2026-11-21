#!/usr/bin/env python3
"""Local Wave 5: 12-Core Fast Surgical Merge of P12_COMBO_D (high-freq) + pton_9.00 (low-freq)."""

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
OUT_JSON = os.path.join(_ROOT, "logs", "local_fast_roll_wave5_p12_pton9.json")


def _eval_probe(args):
    """Top-level worker for Windows spawn multiprocessing."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.10g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    return idx, name, pdict, res


def build_wave5_probes(g2_clean):
    """Merge P12_COMBO_D (0.87-3.99 Hz) with pton_9.00 (0.12-0.32 Hz) and tune 1.4-2.1 Hz."""
    p12_base = {
        "p_ton_roll": 9.0,
        "K_ton_roll": 0.801,
        "Kp_prop_roll": 1.40,
        "Kd_task_roll": 0.65,
        "K_sc_roll": 2.90,
        "z_sc_roll": 0.013,
        "p_sc_roll": 75.0,
        "K_phas_roll": 11.5,
        "p_phas_roll": 4.5,
        "Kp_task_roll": 1.50,
        "Ki_task_roll": 0.55,
        "tau_lead_pole_roll": 0.018,
        "Kp_proprioception_roll": 0.036,
    }
    specs = [
        ("W5_01_MERGE_BASE", {}),
        ("W5_02_Kph9.5", {"K_phas_roll": 9.5}),
        ("W5_03_Kph8.5_pph3.5", {"K_phas_roll": 8.5, "p_phas_roll": 3.5}),
        ("W5_04_Kph9.5_KpTask1.2", {"K_phas_roll": 9.5, "Kp_task_roll": 1.20}),
        ("W5_05_pton10_Kton1.1", {"p_ton_roll": 10.0, "K_ton_roll": 1.10, "K_phas_roll": 9.5}),
        ("W5_06_pton8_Kton0.6", {"p_ton_roll": 8.0, "K_ton_roll": 0.60, "K_phas_roll": 9.5}),
        ("W5_07_Ksc3.2_Kph9.0", {"K_sc_roll": 3.20, "K_phas_roll": 9.0, "p_phas_roll": 3.5}),
        ("W5_08_Ksc3.4_zsc015_Kph9", {"K_sc_roll": 3.40, "z_sc_roll": 0.015, "K_phas_roll": 9.0}),
        ("W5_09_KpProp1.7_Kph9.5", {"Kp_prop_roll": 1.70, "K_phas_roll": 9.5}),
        ("W5_10_Ki0.35_Kph9.5", {"Ki_task_roll": 0.35, "K_phas_roll": 9.5}),
        ("W5_11_Ki0.75_Kph9.5", {"Ki_task_roll": 0.75, "K_phas_roll": 9.5}),
        ("W5_12_Kd0.85_Kph9.5", {"Kd_task_roll": 0.85, "K_phas_roll": 9.5}),
    ]
    probes = []
    for name, mods in specs:
        p = dict(g2_clean)
        p.update(p12_base)
        p.update(mods)
        probes.append((name, p))
    return probes


def main():
    parser = argparse.ArgumentParser(description="Local Wave 5: Merge P12_COMBO_D + pton_9.00")
    parser.add_argument("--workers", type=int, default=12, help="Local worker count")
    args = parser.parse_args()

    t0 = time.time()
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}
    probes = build_wave5_probes(g2_clean)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]

    print(f"=== Running Wave 5 ({len(tasks)} LatEC probes locally on {args.workers} workers) ===", flush=True)
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
                "aborted": bool(res["aborted"]),
                "freq_results": res.get("freq_results", {}),
                "params": pdict,
            }
            r = records[idx]
            print(f"  [done] {name:24s} RollHF={r['hf_noise_rms']:6.3f} Bode={r['bode_latec']:8.1f}", flush=True)

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    sorted_recs = sorted(records, key=lambda r: (1 if r["aborted"] else 0, r["bode_latec"]))
    print(f"\n=== Wave 5 Summary ({dt:.1f}s) ===", flush=True)
    for r in sorted_recs:
        print(
            f"{r['name']:24s} | RollHF={r['hf_noise_rms']:6.3f} | Bode={r['bode_latec']:8.2f} | "
            f"G={r['gain_loss_pts']:7.1f} P={r['phase_loss_pts']:7.1f} | corr={r['corr_vel']:+.3f}"
        )


if __name__ == "__main__":
    main()
