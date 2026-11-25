#!/usr/bin/env python3
"""Local Wave 1: 16-Probe Sensitivity Sweep for LatEC 0.1209 Hz Phase (+95.6 deg -> < 15 deg)."""

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
from scripts.run_roll_0p12_phase_cmaes import ROLL_NAMES, compute_roll_0p12_rank

CHAMP_JSON = os.path.join(_ROOT, "logs", "coop_3axis_champion.json")
G35_JSON = os.path.join(_ROOT, "logs", "cmaes_3axis_18p_roll_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "local_probe_latec_0p12_phase.json")


def _eval_probe(args):
    """Top-level worker calling unmodified run_axis_worker on LatEC (all 28 booleans TRUE)."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    assert not res["false_booleans"], f"Disabled booleans in {name}: {res['false_booleans']}"
    return idx, name, pdict, res


def build_probes(champ53, g35_53):
    specs = [
        ("P00_CHAMP_BASE", {}),
        ("P01_Ki_0.00", {"Ki_task_roll": 0.0}),
        ("P02_Ki_2.50", {"Ki_task_roll": 2.50}),
        ("P03_KpTask_1.80", {"Kp_task_roll": 1.80}),
        ("P04_KpTask_4.50", {"Kp_task_roll": 4.50}),
        ("P05_KdTask_0.60", {"Kd_task_roll": 0.60}),
        ("P06_KdTask_4.50", {"Kd_task_roll": 4.50}),
        ("P07_Vol_HiKp_LoKi", {"Kp_task_roll": 2.2, "Kd_task_roll": 1.6, "Ki_task_roll": 0.05}),
        ("P08_fcVol_0.80", {"fc_voluntary": 0.80}),
        ("P09_Kton_1.2_pton_1.5", {"K_ton_roll": 1.20, "p_ton_roll": 1.50}),
        ("P10_Kton_3.0_pton_0.7", {"K_ton_roll": 3.00, "p_ton_roll": 0.70}),
        ("P11_Kphas_12_pphas_7", {"K_phas_roll": 12.0, "p_phas_roll": 7.0}),
        ("P12_G35_OtoVCR", {"K_phas_roll": 14.44, "p_phas_roll": 7.16, "K_ton_roll": 0.99, "p_ton_roll": 1.50}),
        ("P13_Kphas_10_pphas_1.0", {"K_phas_roll": 10.0, "p_phas_roll": 1.0}),
        ("P14_Kphas_1.0_pphas_0.4", {"K_phas_roll": 1.0, "p_phas_roll": 0.4}),
        ("P15_G35_ROLL18_SMOOTH", {k: g35_53[k] for k in ROLL_NAMES}),
    ]
    probes = []
    for name, mods in specs:
        p = dict(champ53)
        p.update({k: float(v) for k, v in mods.items()})
        probes.append((name, p))
    return probes


def main():
    parser = argparse.ArgumentParser(description="Local 16-Probe LatEC 0.1209 Hz Phase Sweep")
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args()

    t0 = time.time()
    champ53 = {k: float(v) for k, v in json.load(open(CHAMP_JSON, "r", encoding="utf-8"))["params"].items()}
    g35_53 = {k: float(v) for k, v in json.load(open(G35_JSON, "r", encoding="utf-8"))["params"].items()}

    probes = build_probes(champ53, g35_53)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]
    print(f"=== Running Local Wave 1 ({len(tasks)} LatEC probes on {args.workers} workers) ===", flush=True)

    records = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_eval_probe, t) for t in tasks]
        for fut in as_completed(futs):
            idx, name, pdict, res = fut.result()
            bode_5f = float(res["classic_bode_score"])
            xtalk_l = float(res["pitch_crosstalk_penalty"])
            hf_l = float(res["hf_penalty"])
            hf_rms_l = float(res["hf_noise_rms"])
            corr_v = float(res["corr_vel"])
            aborted = bool(res["aborted"])
            f0 = res["freq_results"]["0.1209"]
            ph_0p12 = float(f0["sim_phase"])
            ph_err_0p12 = float(f0["phase_err"])
            ph_loss_0p12 = float(f0["phase_loss_pts_raw"])
            g_0p12 = float(f0["sim_gain"])
            db_0p12 = float(f0["gain_db_diff"])
            bode_all6 = bode_5f + ph_loss_0p12
            rank_score = compute_roll_0p12_rank(bode_5f, ph_loss_0p12, xtalk_l, hf_l, hf_rms_l, corr_v, aborted)

            rec = {
                "name": name,
                "bool_banner": res["bool_banner"],
                "aborted": aborted,
                "ph_0p12": round(ph_0p12, 2),
                "ph_err_0p12": round(ph_err_0p12, 2),
                "ph_loss_0p12": round(ph_loss_0p12, 2),
                "g_0p12": round(g_0p12, 3),
                "db_0p12": round(db_0p12, 3),
                "bode_5f": round(bode_5f, 2),
                "bode_all6": round(bode_all6, 2),
                "gain_loss_pts": round(float(res["gain_loss_pts"]), 2),
                "phase_loss_pts": round(float(res["phase_loss_pts"]), 2),
                "hf_noise_rms": round(hf_rms_l, 3),
                "corr_vel": round(corr_v, 4),
                "xtalk_latec": round(xtalk_l, 2),
                "hf_penalty": round(hf_l, 2),
                "rank_score": round(rank_score, 2),
                "freq_results": res["freq_results"],
                "params": pdict,
            }
            records[idx] = rec
            print(
                f"  [done] {name:24s} ph={rec['ph_0p12']:7.1f}({rec['ph_err_0p12']:+6.1f}°) "
                f"db={rec['db_0p12']:+5.2f} | 5f={rec['bode_5f']:7.1f} All6={rec['bode_all6']:8.1f} "
                f"HF={rec['hf_noise_rms']:5.2f} r={rec['corr_vel']:+.3f} rank={rec['rank_score']:9.1f}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    sorted_recs = sorted(records, key=lambda r: (1 if r["aborted"] else 0, r["rank_score"]))
    print(f"\n=== Wave 1 Sorted Summary ({dt:.1f}s) -> {OUT_JSON} ===", flush=True)
    print(
        f"{'Name':24s} | {'ph0.12':>7s} | {'err0.12':>7s} | {'db0.12':>6s} | "
        f"{'bode_5f':>8s} | {'bode_all6':>9s} | {'HF':>6s} | {'corr':>6s} | {'XTalk':>6s} | {'rank_score':>10s}"
    )
    print("-" * 112)
    for r in sorted_recs:
        print(
            f"{r['name']:24s} | {r['ph_0p12']:7.1f} | {r['ph_err_0p12']:+7.1f} | {r['db_0p12']:+6.2f} | "
            f"{r['bode_5f']:8.1f} | {r['bode_all6']:9.1f} | {r['hf_noise_rms']:6.2f} | {r['corr_vel']:+6.3f} | "
            f"{r['xtalk_latec']:6.1f} | {r['rank_score']:10.1f}"
        )


if __name__ == "__main__":
    main()
