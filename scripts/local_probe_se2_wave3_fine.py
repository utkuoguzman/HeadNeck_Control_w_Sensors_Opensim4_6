#!/usr/bin/env python3
"""Local Wave 3: Precision SE2(3) Sweep & 3-Axis Verification Around Exact Empirical Optimum."""

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
OUT_JSON = os.path.join(_ROOT, "logs", "local_probe_se2_wave3_fine.json")

TRUBA_G1_S7_ROLL22 = {
    "K_sc_roll": 1.972608376002428,
    "z_sc_roll": 0.016826629698346616,
    "p_sc_roll": 33.39552492030596,
    "K_phas_roll": 4.343990830614617,
    "z_phas_roll": 0.009823951512213443,
    "p_phas_roll": 10.178901816663148,
    "K_ton_roll": 1.4327916855890446,
    "p_ton_roll": 12.856519299772554,
    "Kp_prop_roll": 1.6806097345802073,
    "z_ccr_roll": 0.006965333681894562,
    "p_ccr_roll": 93.76500845054794,
    "Kp_proprioception_roll": 0.03870912351352191,
    "Kp_task_roll": 0.08130058811980642,
    "Kd_task_roll": 2.4375993890101864,
    "Ki_task_roll": 0.5056185834394976,
    "tau_lead_pole_roll": 0.019655291133694915,
    "Kp_buckle_roll": 1.3613998259997198,
    "Kd_buckle_roll": 0.32532286875132094,
    "Kp_trans_roll": -1.9424340018662547,
    "Kv_trans_roll": -0.35089948667672277,
    "tau_vel_leak": 5.0811413739169335,
    "tau_pos_leak": 2.4198534542113506,
}


def _eval_probe(args):
    """Top-level worker calling unmodified run_axis_worker on mixed axes (all 28 booleans TRUE)."""
    idx, mode, name, pdict = args
    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()]
    res = run_axis_worker((mode, overrides))
    assert not res["false_booleans"], f"Disabled booleans in {name}: {res['false_booleans']}"
    return idx, mode, name, pdict, res


def build_probes(champ53):
    emp_mods = {"Kp_trans_roll": -2.26, "Kv_trans_roll": -0.02, "tau_vel_leak": 3.0, "tau_pos_leak": 2.0}
    specs = [
        ("LatEC", "W3_01_EMP_OPT",               emp_mods),
        ("APEO",  "W3_02_EMP_APEO",              emp_mods),
        ("YawMA", "W3_03_EMP_YAWMA",             emp_mods),
        ("LatEC", "W3_04_Kp_m1p8",               {"Kp_trans_roll": -1.80, "Kv_trans_roll":  0.0,  "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_05_Kp_m2p0",               {"Kp_trans_roll": -2.00, "Kv_trans_roll":  0.0,  "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_06_Kp_m2p26_Kv0",          {"Kp_trans_roll": -2.26, "Kv_trans_roll":  0.0,  "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_07_Kp_m2p5",               {"Kp_trans_roll": -2.50, "Kv_trans_roll":  0.0,  "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_08_Kp_m2p8",               {"Kp_trans_roll": -2.80, "Kv_trans_roll":  0.0,  "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_09_Kp_m2p1_Kvm0p3",        {"Kp_trans_roll": -2.10, "Kv_trans_roll": -0.30, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_10_Kp_m2p4_Kvp0p3",        {"Kp_trans_roll": -2.40, "Kv_trans_roll":  0.30, "tau_vel_leak":  3.0, "tau_pos_leak": 2.0}),
        ("LatEC", "W3_11_Leak_5_3_Kpm1p8",       {"Kp_trans_roll": -1.80, "Kv_trans_roll": -0.20, "tau_vel_leak":  5.0, "tau_pos_leak": 3.0}),
        ("LatEC", "W3_12_Leak_4_2p5_Kpm2p0",     {"Kp_trans_roll": -2.00, "Kv_trans_roll": -0.20, "tau_vel_leak":  4.0, "tau_pos_leak": 2.5}),
        ("LatEC", "W3_13_Leak_2p5_1p5_Kpm2p6",   {"Kp_trans_roll": -2.60, "Kv_trans_roll":  0.20, "tau_vel_leak":  2.5, "tau_pos_leak": 1.5}),
        ("LatEC", "W3_14_Leak_15_5_Kpm1p4_Kvm0p7", {"Kp_trans_roll": -1.40, "Kv_trans_roll": -0.70, "tau_vel_leak": 15.0, "tau_pos_leak": 5.0}),
        ("LatEC", "W3_15_TRUBA_G1_S7",           TRUBA_G1_S7_ROLL22),
        ("LatEC", "W3_16_EMP_KPHAS4p45",         {"Kp_trans_roll": -2.26, "Kv_trans_roll":  0.0,  "tau_vel_leak":  3.0, "tau_pos_leak": 2.0, "K_phas_roll": 4.45}),
    ]
    probes = []
    for mode, name, mods in specs:
        p = dict(champ53)
        p.update({k: float(v) for k, v in mods.items()})
        probes.append((mode, name, p))
    return probes


def main():
    parser = argparse.ArgumentParser(description="Local Wave 3 Precision SE2(3) & 3-Axis Verification")
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args()

    t0 = time.time()
    champ53 = {k: float(v) for k, v in json.load(open(CHAMP_JSON, "r", encoding="utf-8"))["params"].items()}
    probes = build_probes(champ53)
    tasks = [(i, mode, name, pdict) for i, (mode, name, pdict) in enumerate(probes)]
    print(f"=== Running Local Wave 3 SE2(3) Fine Sweep ({len(tasks)} probes on {args.workers} workers) ===", flush=True)

    records = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_eval_probe, t) for t in tasks]
        for fut in as_completed(futs):
            idx, mode, name, pdict, res = fut.result()
            fr = res["freq_results"]
            bode_score = float(res["classic_bode_score"])
            xtalk = float(res["pitch_crosstalk_penalty"])
            hf_pen = float(res["hf_penalty"])
            hf_rms = float(res["hf_noise_rms"])
            corr_v = float(res["corr_vel"])
            quiet_m = float(res["quiet_main_max_deg"])
            aborted = bool(res["aborted"])

            rec = {
                "mode": mode,
                "name": name,
                "bool_banner": res["bool_banner"],
                "aborted": aborted,
                "quiet_main_max_deg": round(quiet_m, 4),
                "classic_bode_score": round(bode_score, 2),
                "gain_loss_pts": round(float(res["gain_loss_pts"]), 2),
                "phase_loss_pts": round(float(res["phase_loss_pts"]), 2),
                "hf_noise_rms": round(hf_rms, 3),
                "corr_vel": round(corr_v, 4),
                "xtalk_penalty": round(xtalk, 2),
                "hf_penalty": round(hf_pen, 2),
                "total_score": round(float(res["total_score"]), 2),
                "freq_results": fr,
                "params": pdict,
            }
            if mode == "LatEC":
                f0 = fr["0.1209"]
                ph_loss_0p12 = float(f0["phase_loss_pts_raw"])
                rk = compute_roll_0p12_rank(bode_score, ph_loss_0p12, xtalk, hf_pen, hf_rms, corr_v, aborted)
                rec.update({
                    "ph_0p12": round(float(f0["sim_phase"]), 2),
                    "ph_err_0p12": round(float(f0["phase_err"]), 2),
                    "ph_loss_0p12": round(ph_loss_0p12, 2),
                    "g_0p12": round(float(f0["sim_gain"]), 3),
                    "db_0p12": round(float(f0["gain_db_diff"]), 3),
                    "bode_all6": round(bode_score + ph_loss_0p12, 2),
                    "rank_score": round(rk, 2),
                })
                print(
                    f"  [done] {name:28s} ph={rec['ph_0p12']:7.1f}({rec['ph_err_0p12']:+6.1f}°) "
                    f"db={rec['db_0p12']:+5.2f} | 5f={rec['classic_bode_score']:7.1f} All6={rec['bode_all6']:8.1f} "
                    f"HF={rec['hf_noise_rms']:5.2f} Q={rec['quiet_main_max_deg']:.3f}° r={rec['corr_vel']:+.3f} rank={rec['rank_score']:8.1f}",
                    flush=True,
                )
            elif mode == "APEO":
                db5 = float(fr["5.583"]["gain_db_diff"]) if "5.583" in fr else None
                rec["db_5p583"] = round(db5, 3) if db5 is not None else None
                print(
                    f"  [done] {name:28s} [APEO ] Bode={rec['classic_bode_score']:7.2f} db5p5={rec['db_5p583']:+5.2f}dB "
                    f"HF={rec['hf_noise_rms']:5.2f} Q={rec['quiet_main_max_deg']:.3f}° r={rec['corr_vel']:+.3f}",
                    flush=True,
                )
            else:
                print(
                    f"  [done] {name:28s} [YawMA] Bode={rec['classic_bode_score']:7.2f} "
                    f"HF={rec['hf_noise_rms']:5.2f} Q={rec['quiet_main_max_deg']:.3f}° r={rec['corr_vel']:+.3f}",
                    flush=True,
                )
            records[idx] = rec

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records}, f, indent=2)

    print(f"\n=== 3-Axis Verification Check (W3_01 / W3_02 / W3_03 EMP_OPT) ({dt:.1f}s) ===", flush=True)
    for r in records[:3]:
        if r["mode"] == "LatEC":
            print(
                f"  [LatEC] 5f={r['classic_bode_score']:.2f}, All6={r['bode_all6']:.2f}, "
                f"ph0.12={r['ph_0p12']:.1f}° ({r['ph_err_0p12']:+.1f}°), db0.12={r['db_0p12']:+.2f}dB, "
                f"HF={r['hf_noise_rms']:.3f}, corr={r['corr_vel']:+.4f}, Q={r['quiet_main_max_deg']:.3f}°"
            )
        elif r["mode"] == "APEO":
            print(
                f"  [APEO ] Bode={r['classic_bode_score']:.2f}, db5p5={r['db_5p583']:+.3f}dB, "
                f"HF={r['hf_noise_rms']:.3f}, corr={r['corr_vel']:+.4f}, Q={r['quiet_main_max_deg']:.3f}°"
            )
        else:
            print(
                f"  [YawMA] Bode={r['classic_bode_score']:.2f}, "
                f"HF={r['hf_noise_rms']:.3f}, corr={r['corr_vel']:+.4f}, Q={r['quiet_main_max_deg']:.3f}°"
            )

    lat_recs = sorted([r for r in records if r["mode"] == "LatEC"], key=lambda r: (1 if r["aborted"] else 0, r["rank_score"]))
    print(f"\n=== Wave 3 Sorted LatEC Summary ({len(lat_recs)} probes) -> {OUT_JSON} ===", flush=True)
    print(
        f"{'Name':28s} | {'ph0.12':>7s} | {'err0.12':>7s} | {'db0.12':>6s} | "
        f"{'bode_5f':>8s} | {'bode_all6':>9s} | {'HF':>6s} | {'Quiet':>6s} | {'corr':>6s} | {'XTalk':>6s} | {'rank_score':>10s}"
    )
    print("-" * 124)
    for r in lat_recs:
        print(
            f"{r['name']:28s} | {r['ph_0p12']:7.1f} | {r['ph_err_0p12']:+7.1f} | {r['db_0p12']:+6.2f} | "
            f"{r['classic_bode_score']:8.1f} | {r['bode_all6']:9.1f} | {r['hf_noise_rms']:6.2f} | "
            f"{r['quiet_main_max_deg']:6.3f} | {r['corr_vel']:+6.3f} | {r['xtalk_penalty']:6.1f} | {r['rank_score']:10.1f}"
        )


if __name__ == "__main__":
    main()
