#!/usr/bin/env python3
"""Focused Wave 3 hybrid co-optimization on the unified ungated 3-axis controller.

Combines Wave 2 LF-phase mechanisms (W2_cp_04_K_ton_roll / LF7 / B2) with Wave 2
low-noise HF mechanisms (W2_rnd_06 / W2_rnd_10 / B1) while keeping YawMA-tuned
Yaw parameters (Kp_prop_yaw=8.5, z_ccr_yaw=0.06), then runs parallel YawMA +
APEO Pitch checks and Pitch co-trim probes.
"""

import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _env_k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_env_k, "1")

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import evaluate_and_plot_tuned_3axis as e
import optimize_latec_roll_targeted as o
from cmaes_worker_multisine import run_multisine_simulation


def build_wave3_candidates():
    """12 hybrid Roll candidates built around W2_cp_04 / LF7 LF-phase + W2_rnd_06/10 HF smoothness."""
    base = dict(o.BASE_R4)
    base.update({
        "z_sc_roll": 0.0,
        "z_phas_roll": 0.0,
        "p_ap_roll": -1.0,
        "c_ap_roll": 0.0,
        "deriv_comp_roll": 0.0,
        "Kp_prop_yaw": 8.5,
        "z_ccr_yaw": 0.06,
    })

    defs = [
        ("W3_01_W2rnd06_YawKeep", {
            "Kp_roll": 1.076179, "Kd_roll": 0.984002, "Ki_roll": 1.862658,
            "K_ton_roll": 0.835141, "p_ton_roll": 2.987426,
            "K_sc_roll": 1.346138, "p_sc_roll": 50.589082,
            "K_phas_roll": 14.699731, "p_phas_roll": 5.823704,
            "Kp_prop_roll": 5.527639, "z_ccr_roll": 0.105556,
        }),
        ("W3_02_W2cp04_LowNoise", {
            "Kp_roll": 1.40, "Kd_roll": 0.98, "Ki_roll": 1.50,
            "K_ton_roll": -0.85, "p_ton_roll": 3.0,
            "K_sc_roll": 1.40, "p_sc_roll": 58.0,
            "K_phas_roll": 15.0, "p_phas_roll": 5.2,
            "Kp_prop_roll": 4.20, "z_ccr_roll": 0.10,
        }),
        ("W3_03_LF7_LowNoise", {
            "Kp_roll": 1.30, "Kd_roll": 1.00, "Ki_roll": 1.45,
            "K_ton_roll": -0.60, "p_ton_roll": 3.0,
            "K_sc_roll": 1.42, "p_sc_roll": 62.0,
            "K_phas_roll": 15.2, "p_phas_roll": 5.0,
            "Kp_prop_roll": 4.40, "z_ccr_roll": 0.10,
        }),
        ("W3_04_MidNegTon_pph45", {
            "Kp_roll": 1.25, "Kd_roll": 0.98, "Ki_roll": 1.45,
            "K_ton_roll": -0.45, "p_ton_roll": 2.5,
            "K_sc_roll": 1.45, "p_sc_roll": 62.0,
            "K_phas_roll": 15.5, "p_phas_roll": 4.5,
            "Kp_prop_roll": 4.80, "z_ccr_roll": 0.11,
        }),
        ("W3_05_ZeroTon_HighKi", {
            "Kp_roll": 1.15, "Kd_roll": 1.02, "Ki_roll": 1.80,
            "K_ton_roll": 0.0, "p_ton_roll": 2.5,
            "K_sc_roll": 1.40, "p_sc_roll": 62.0,
            "K_phas_roll": 15.2, "p_phas_roll": 4.8,
            "Kp_prop_roll": 5.00, "z_ccr_roll": 0.11,
        }),
        ("W3_06_MildPosTon_pph44", {
            "Kp_roll": 1.08, "Kd_roll": 0.98, "Ki_roll": 1.90,
            "K_ton_roll": 0.35, "p_ton_roll": 2.4,
            "K_sc_roll": 1.38, "p_sc_roll": 62.0,
            "K_phas_roll": 15.2, "p_phas_roll": 4.4,
            "Kp_prop_roll": 5.40, "z_ccr_roll": 0.105,
        }),
        ("W3_07_NegTon65_pTon20", {
            "Kp_roll": 1.35, "Kd_roll": 1.02, "Ki_roll": 1.30,
            "K_ton_roll": -0.65, "p_ton_roll": 2.0,
            "K_sc_roll": 1.45, "p_sc_roll": 64.0,
            "K_phas_roll": 15.0, "p_phas_roll": 4.8,
            "Kp_prop_roll": 4.50, "z_ccr_roll": 0.105,
        }),
        ("W3_08_NegTon75_Kd110", {
            "Kp_roll": 1.30, "Kd_roll": 1.10, "Ki_roll": 1.25,
            "K_ton_roll": -0.75, "p_ton_roll": 2.4,
            "K_sc_roll": 1.50, "p_sc_roll": 65.0,
            "K_phas_roll": 15.8, "p_phas_roll": 4.8,
            "Kp_prop_roll": 4.30, "z_ccr_roll": 0.105,
        }),
        ("W3_09_NegTon50_dc010", {
            "Kp_roll": 1.25, "Kd_roll": 1.00, "Ki_roll": 1.55,
            "K_ton_roll": -0.50, "p_ton_roll": 2.8,
            "K_sc_roll": 1.48, "p_sc_roll": 62.0,
            "K_phas_roll": 15.5, "p_phas_roll": 4.7,
            "Kp_prop_roll": 4.80, "z_ccr_roll": 0.11,
            "deriv_comp_roll": 0.10,
        }),
        ("W3_10_DeepNegTon95_Ki135", {
            "Kp_roll": 1.45, "Kd_roll": 1.02, "Ki_roll": 1.35,
            "K_ton_roll": -0.95, "p_ton_roll": 2.5,
            "K_sc_roll": 1.40, "p_sc_roll": 62.0,
            "K_phas_roll": 14.8, "p_phas_roll": 4.8,
            "Kp_prop_roll": 4.20, "z_ccr_roll": 0.10,
        }),
        ("W3_11_B2Hybrid_LowKsc", {
            "Kp_roll": 0.95, "Kd_roll": 1.00, "Ki_roll": 1.75,
            "K_ton_roll": -0.30, "p_ton_roll": 2.6,
            "K_sc_roll": 1.45, "p_sc_roll": 60.0,
            "K_phas_roll": 16.5, "p_phas_roll": 4.3,
            "Kp_prop_roll": 5.20, "z_ccr_roll": 0.105,
        }),
        ("W3_12_W2rnd10_NegTon", {
            "Kp_roll": 1.22, "Kd_roll": 1.12, "Ki_roll": 1.60,
            "K_ton_roll": -0.45, "p_ton_roll": 2.8,
            "K_sc_roll": 1.50, "p_sc_roll": 66.0,
            "K_phas_roll": 16.0, "p_phas_roll": 5.0,
            "Kp_prop_roll": 4.50, "z_ccr_roll": 0.108,
        }),
    ]
    out = []
    for name, ov in defs:
        merged = dict(base)
        merged.update(ov)
        out.append((name, merged))
    return out


def _eval_w3_lat_worker(item):
    name, roll_ov = item
    params = dict(o.BASE_YAW_TUNED)
    params.update(roll_ov)
    try:
        sim_res = o.run_full_axis_sim(("LatEC", f"w3_{name}", params, 2.0))
        if sim_res["aborted"]:
            return {
                "name": name,
                "roll_ov": roll_ov,
                "ok": False,
                "error": "aborted",
                "metrics": {"score": 1e6, "latec_cost": 1e6},
            }
        metrics = o.compute_targeted_roll_score(sim_res)
        metrics["latec_cost"] = round(float(e.compute_latec_cost(sim_res)), 3)
        return {"name": name, "roll_ov": roll_ov, "ok": True, "metrics": metrics}
    except Exception as ex:
        return {
            "name": name,
            "roll_ov": roll_ov,
            "ok": False,
            "error": str(ex),
            "metrics": {"score": 1e6, "latec_cost": 1e6},
        }


def _eval_verify_worker(job):
    kind = job["kind"]
    name = job["name"]
    params = dict(o.BASE_YAW_TUNED)
    params.update(job["ov"])
    try:
        if kind == "YawMA":
            sim_yaw = o.run_full_axis_sim(("YawMA", f"w3_yaw_{name}", params, 2.0))
            if sim_yaw["aborted"]:
                return {"kind": kind, "name": name, "ok": False, "error": "aborted"}
            y_fr = sim_yaw["freq_results"]
            y_cost = float(e.compute_latec_cost(sim_yaw))
            return {
                "kind": kind,
                "name": name,
                "ok": True,
                "yaw_cost": round(y_cost, 3),
                "hf_noise_rms": round(float(sim_yaw["hf_noise_rms"]), 3),
                "corr_vel": round(float(sim_yaw["corr_vel"]), 4),
                "max_db": round(float(max(abs(y_fr[k]["gain_db_diff"]) for k in y_fr)), 2),
                "max_ph": round(float(max(abs(y_fr[k]["phase_err"]) for k in y_fr)), 1),
                "ov": job["ov"],
            }
        elif kind == "APEO":
            os.environ["SAVE_TS"] = "0"
            os.environ["MULTISINE_EXP"] = "1.0"
            os.environ["MULTISINE_MIN_AMP_M"] = "0.0"
            os.environ["NOISE_FLOOR_7927_DEG"] = "0.0"
            os.environ["MULTISINE_CYCLES"] = "6.0"
            total_score, freq_results, telemetry = run_multisine_simulation(
                params["Kp_pitch"], params["Ki_pitch"], params["Kd_pitch"],
                params["K_ton"], params["p_ton"], params["K_sc"], params["z_sc"], params["p_sc"],
                params["K_phas"], params["p_phas"], params["kp_prop"], params["z_ccr"], params["p_ccr"],
                params["k_gamma_dyn"], params["k_gamma_stat"],
                worker_name=f"w3_apeo_{name}",
                p_phas2=-1.0,
                z_phas=params["z_phas"],
                extra_props=params,
            )
            max_db = max(abs(freq_results[k]["gain_db_diff"]) for k in freq_results) if freq_results else 999.0
            max_ph = max(abs(freq_results[k]["phase_diff"]) for k in freq_results) if freq_results else 999.0
            return {
                "kind": kind,
                "name": name,
                "ok": True,
                "apeo_score": round(float(total_score), 2),
                "bode_without_hf": telemetry.get("bode_score_without_hf"),
                "hf_penalty": telemetry.get("hf_penalty"),
                "hf_rms_pitch_deg": telemetry.get("hf_rms_pitch_deg"),
                "max_abs_roll_deg": telemetry.get("max_abs_roll_deg"),
                "max_abs_yaw_deg": telemetry.get("max_abs_yaw_deg"),
                "max_db": round(float(max_db), 2),
                "max_ph": round(float(max_ph), 1),
                "ov": job["ov"],
            }
    except Exception as ex:
        return {"kind": kind, "name": name, "ok": False, "error": str(ex)}


def select_top4(w3_results):
    valid = [r for r in w3_results if r["ok"]]
    smooth = [r for r in valid if r["metrics"]["noise"] <= 3.3 and r["metrics"]["corr"] >= 0.82]
    chosen = []
    chosen_names = set()
    for r in smooth[:3]:
        chosen.append(r)
        chosen_names.add(r["name"])
    if valid and valid[0]["name"] not in chosen_names:
        chosen.append(valid[0])
        chosen_names.add(valid[0]["name"])
    for r in valid:
        if len(chosen) >= 4:
            break
        if r["name"] not in chosen_names:
            chosen.append(r)
            chosen_names.add(r["name"])
    return chosen


def main():
    os.makedirs(os.path.join(_ROOT, "logs", "roll_yaw_baseline"), exist_ok=True)
    cands = build_wave3_candidates()
    print(f"Running Wave 3 Hybrid LatEC Search ({len(cands)} candidates, 12 workers)...", flush=True)

    t0 = time.time()
    w3_results = []
    with ProcessPoolExecutor(max_workers=12) as ex:
        futs = [ex.submit(_eval_w3_lat_worker, c) for c in cands]
        for idx, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            w3_results.append(r)
            m = r["metrics"]
            if r["ok"]:
                print(
                    f"  [W3] {idx:2d}/{len(cands)} ({time.time()-t0:5.0f}s) | {r['name']:24s} "
                    f"J={m['score']:6.1f} cost={m['latec_cost']:5.2f} | LFph={m['ph_errs'][:3]} | "
                    f"HFdB={m['db_errs'][5:]} | noise={m['noise']:.2f} r={m['corr']:.3f}",
                    flush=True,
                )
            else:
                print(f"  [W3] {idx:2d}/{len(cands)} FAILED {r['name']}: {r.get('error')}", flush=True)

    w3_results.sort(key=lambda x: x["metrics"]["score"])
    print(f"\n=== WAVE 3 RANKED RESULTS ({time.time()-t0:.1f}s) ===", flush=True)
    for r in w3_results:
        if not r["ok"]:
            continue
        m = r["metrics"]
        print(
            f"  {r['name']:24s} | J={m['score']:6.2f} cost={m['latec_cost']:5.2f} | "
            f"LFmax={m['lf_ph_max']:4.1f} | HFdBmax={m['hf_db_max']:4.2f} | "
            f"midHFph={m['midhf_ph_max']:4.1f} | noise={m['noise']:4.2f} | r={m['corr']:.3f} | "
            f"ph={m['ph_errs']} | dB={m['db_errs']}",
            flush=True,
        )

    top4 = select_top4(w3_results)
    print(f"\nSelected Top 4 for parallel 3-axis verification: {[r['name'] for r in top4]}", flush=True)

    best_lat = top4[0]
    best_ov = best_lat["roll_ov"]

    verify_jobs = []
    for r in top4:
        verify_jobs.append({"kind": "YawMA", "name": r["name"], "ov": r["roll_ov"]})
        verify_jobs.append({"kind": "APEO", "name": r["name"], "ov": r["roll_ov"]})

    # 4 APEO Pitch co-trim probes on #1 selected Roll candidate
    cotrims = [
        ("PitchCotrim_A_Kp32_Ki32_Kd052", {"Kp_pitch": 3.20, "Ki_pitch": 3.20, "Kd_pitch": 0.52}),
        ("PitchCotrim_B_Kp35_Ki34_Kt48", {"Kp_pitch": 3.50, "Ki_pitch": 3.40, "K_ton": 4.80}),
        ("PitchCotrim_C_Kp34_Kd058_kp055", {"Kp_pitch": 3.40, "Kd_pitch": 0.58, "kp_prop": 0.55}),
        ("PitchCotrim_D_Ki42_Kt45_kp038", {"Ki_pitch": 4.20, "K_ton": 4.50, "kp_prop": 0.38}),
    ]
    for pname, pov in cotrims:
        merged_ov = dict(best_ov)
        merged_ov.update(pov)
        verify_jobs.append({"kind": "APEO", "name": pname, "ov": merged_ov})

    print(f"\nRunning 3-Axis Verification + Pitch Co-Trim Wave ({len(verify_jobs)} jobs, 12 workers)...", flush=True)
    t1 = time.time()
    verify_results = []
    with ProcessPoolExecutor(max_workers=12) as ex:
        futs = [ex.submit(_eval_verify_worker, j) for j in verify_jobs]
        for idx, fut in enumerate(as_completed(futs), 1):
            vr = fut.result()
            verify_results.append(vr)
            if not vr["ok"]:
                print(f"  [V] {idx:2d}/{len(verify_jobs)} FAILED {vr['kind']} {vr['name']}: {vr.get('error')}", flush=True)
            elif vr["kind"] == "YawMA":
                print(
                    f"  [V] {idx:2d}/{len(verify_jobs)} ({time.time()-t1:5.0f}s) YawMA {vr['name']:24s} | "
                    f"yaw_cost={vr['yaw_cost']:5.2f} noise={vr['hf_noise_rms']:.2f} r={vr['corr_vel']:.3f} "
                    f"max_dB={vr['max_db']:.2f} max_ph={vr['max_ph']:.1f}",
                    flush=True,
                )
            else:
                print(
                    f"  [V] {idx:2d}/{len(verify_jobs)} ({time.time()-t1:5.0f}s) APEO  {vr['name']:30s} | "
                    f"score={vr['apeo_score']:7.2f} (bode={vr['bode_without_hf']}, hf_pen={vr['hf_penalty']}) "
                    f"max_dB={vr['max_db']:.2f} max_ph={vr['max_ph']:.1f}",
                    flush=True,
                )

    out_path = os.path.join(_ROOT, "logs", "roll_yaw_baseline", "ungated_wave3_coopt_results.json")
    payload = {
        "wave3_lat_results": w3_results,
        "top4_names": [r["name"] for r in top4],
        "verification_and_cotrim": verify_results,
    }
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nSaved Wave 3 + verification results to {out_path}", flush=True)


if __name__ == "__main__":
    main()
