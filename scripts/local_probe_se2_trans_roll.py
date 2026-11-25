#!/usr/bin/env python3
"""Local Wave 2: 16-Probe SE2(3) Voluntary Lateral Translation Sweep for LatEC 0.1209 Hz Phase."""

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
from scripts.run_roll_0p12_phase_cmaes import compute_roll_0p12_rank

CHAMP_JSON = os.path.join(_ROOT, "logs", "coop_3axis_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "local_probe_se2_trans_roll.json")


def _eval_probe(args):
    """Top-level worker calling unmodified run_axis_worker on LatEC (all 28 booleans TRUE)."""
    idx, name, pdict = args
    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()]
    res = run_axis_worker(("LatEC", overrides))
    assert not res["false_booleans"], f"Disabled booleans in {name}: {res['false_booleans']}"
    return idx, name, pdict, res


def build_probes(champ53):
    specs = [
        ("S00_BASE_ZERO",      {"Kp_trans_roll":  0.0, "Kv_trans_roll":  0.0, "tau_vel_leak": 15.0, "tau_pos_leak": 5.0}),
        ("S01_Kp_pos2",        {"Kp_trans_roll":  2.0, "Kv_trans_roll":  0.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S02_Kp_pos5",        {"Kp_trans_roll":  5.0, "Kv_trans_roll":  0.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S03_Kp_pos10",       {"Kp_trans_roll": 10.0, "Kv_trans_roll":  0.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S04_Kp_neg5",        {"Kp_trans_roll": -5.0, "Kv_trans_roll":  0.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S05_Kp_neg10",       {"Kp_trans_roll":-10.0, "Kv_trans_roll":  0.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S06_Kv_pos3",        {"Kp_trans_roll":  0.0, "Kv_trans_roll":  3.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S07_Kv_neg3",        {"Kp_trans_roll":  0.0, "Kv_trans_roll": -3.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S08_PosCombo_3_2",   {"Kp_trans_roll":  6.0, "Kv_trans_roll":  2.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S09_NegCombo_3_2",   {"Kp_trans_roll": -6.0, "Kv_trans_roll": -2.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S10_MixPosKp_NegKv", {"Kp_trans_roll":  8.0, "Kv_trans_roll": -2.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S11_MixNegKp_PosKv", {"Kp_trans_roll": -8.0, "Kv_trans_roll":  2.0, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("S12_DefLeak_Pos",    {"Kp_trans_roll":  4.0, "Kv_trans_roll":  3.0, "tau_vel_leak": 15.0, "tau_pos_leak": 5.0}),
        ("S13_DefLeak_Neg",    {"Kp_trans_roll": -4.0, "Kv_trans_roll": -3.0, "tau_vel_leak": 15.0, "tau_pos_leak": 5.0}),
        ("S14_MidLeak_Pos",    {"Kp_trans_roll":  6.0, "Kv_trans_roll":  1.5, "tau_vel_leak":  5.0, "tau_pos_leak": 3.0}),
        ("S15_MidLeak_Neg",    {"Kp_trans_roll": -6.0, "Kv_trans_roll": -1.5, "tau_vel_leak":  5.0, "tau_pos_leak": 3.0}),
    ]
    probes = []
    for name, mods in specs:
        p = dict(champ53)
        p.update({k: float(v) for k, v in mods.items()})
        probes.append((name, p))
    return probes


def main():
    parser = argparse.ArgumentParser(description="Local 16-Probe SE2(3) Roll Stabilization Sweep")
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args()

    t0 = time.time()
    champ53 = {k: float(v) for k, v in json.load(open(CHAMP_JSON, "r", encoding="utf-8"))["params"].items()}
    probes = build_probes(champ53)
    tasks = [(i, name, pdict) for i, (name, pdict) in enumerate(probes)]
    print(f"=== Running Local Wave 2 SE2(3) ({len(tasks)} LatEC probes on {args.workers} workers) ===", flush=True)

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
            quiet_m = float(res["quiet_main_max_deg"])
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
                "quiet_main_max_deg": round(quiet_m, 4),
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
                f"  [done] {name:20s} ph={rec['ph_0p12']:7.1f}({rec['ph_err_0p12']:+6.1f}°) "
                f"db={rec['db_0p12']:+5.2f} | 5f={rec['bode_5f']:7.1f} All6={rec['bode_all6']:8.1f} "
                f"HF={rec['hf_noise_rms']:5.2f} Q={rec['quiet_main_max_deg']:.3f}° r={rec['corr_vel']:+.3f} rank={rec['rank_score']:9.1f}",
                flush=True,
            )

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    sorted_recs = sorted(records, key=lambda r: (1 if r["aborted"] else 0, r["rank_score"]))
    print(f"\n=== Wave 2 SE2(3) Sorted Summary ({dt:.1f}s) -> {OUT_JSON} ===", flush=True)
    print(
        f"{'Name':20s} | {'ph0.12':>7s} | {'err0.12':>7s} | {'db0.12':>6s} | "
        f"{'bode_5f':>8s} | {'bode_all6':>9s} | {'HF':>6s} | {'Quiet':>6s} | {'corr':>6s} | {'XTalk':>6s} | {'rank_score':>10s}"
    )
    print("-" * 116)
    for r in sorted_recs:
        print(
            f"{r['name']:20s} | {r['ph_0p12']:7.1f} | {r['ph_err_0p12']:+7.1f} | {r['db_0p12']:+6.2f} | "
            f"{r['bode_5f']:8.1f} | {r['bode_all6']:9.1f} | {r['hf_noise_rms']:6.2f} | "
            f"{r['quiet_main_max_deg']:6.3f} | {r['corr_vel']:+6.3f} | {r['xtalk_latec']:6.1f} | {r['rank_score']:10.1f}"
        )


if __name__ == "__main__":
    main()
