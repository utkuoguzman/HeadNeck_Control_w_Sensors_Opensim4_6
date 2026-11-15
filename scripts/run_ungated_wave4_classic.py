#!/usr/bin/env python3
"""Wave 4 single 12-worker bridge & APEO classic Bode score batch.

Computes Classic Bode Error Score across all axes:
    classic_bode_score = sum_k (100.0 * gain_db_diff_k^2 + 1.0 * phase_err_deg_k^2)
    gain_loss_pts      = sum_k (100.0 * gain_db_diff_k^2)
    phase_loss_pts     = sum_k (1.0 * phase_err_deg_k^2)

Jobs (12 parallel workers):
  - 8 LatEC Roll bridge candidates between W2_cp_04_K_ton_roll (8,148.50 pts)
    and W3_02_W2cp04_LowNoise (2.19 deg/s noise)
  - 4 APEO Pitch checks using fixed freq_results[k]["phase_err_deg"] key
"""

import json
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
import run_ungated_wave3_coopt as w3
from cmaes_worker_multisine import run_multisine_simulation


W2_CP_04_PARAMS = {
    "Kp_roll": 1.4,
    "Kd_roll": 1.14,
    "Ki_roll": 1.5,
    "K_ton_roll": -0.85,
    "p_ton_roll": 3.0,
    "K_sc_roll": 2.029,
    "p_sc_roll": 60.0,
    "z_sc_roll": 0.0,
    "K_phas_roll": 16.653,
    "p_phas_roll": 5.747,
    "z_phas_roll": 0.0,
    "Kp_prop_roll": 4.2,
    "z_ccr_roll": 0.10,
    "deriv_comp_roll": 0.0,
    "p_ap_roll": -1.0,
    "c_ap_roll": 0.0,
    "Kp_prop_yaw": 8.5,
    "z_ccr_yaw": 0.06,
}


def compute_classic_bode_score(freq_results, is_apeo=False):
    gain_pts = 0.0
    phase_pts = 0.0
    for k, v in freq_results.items():
        db = float(v["gain_db_diff"])
        pe = float(v["phase_err_deg"] if is_apeo else v["phase_err"])
        gain_pts += 100.0 * (db ** 2)
        phase_pts += 1.0 * (pe ** 2)
    return {
        "classic_bode_score": round(gain_pts + phase_pts, 2),
        "gain_loss_pts": round(gain_pts, 2),
        "phase_loss_pts": round(phase_pts, 2),
    }


def build_wave4_jobs():
    w3_map = dict(w3.build_wave3_candidates())

    bridge_defs = [
        ("W4_01_cp04_Ksc165", {"K_sc_roll": 1.65}),
        ("W4_02_cp04_Ksc178", {"K_sc_roll": 1.78}),
        ("W4_03_cp04_Ksc170_Kph170", {"K_sc_roll": 1.70, "K_phas_roll": 17.0}),
        ("W4_04_cp04_Ksc175_Ki130", {"K_sc_roll": 1.75, "Ki_roll": 1.30}),
        ("W4_05_cp04_Ksc170_dc008", {"K_sc_roll": 1.70, "deriv_comp_roll": 0.08, "z_ccr_roll": 0.115}),
        ("W4_06_cp04_MidBridge", {
            "Kp_roll": 1.40, "Kd_roll": 1.08, "Ki_roll": 1.50,
            "K_ton_roll": -0.85, "p_ton_roll": 3.0,
            "K_sc_roll": 1.68, "p_sc_roll": 60.0,
            "K_phas_roll": 16.4, "p_phas_roll": 5.6,
            "Kp_prop_roll": 4.2, "z_ccr_roll": 0.11,
        }),
        ("W4_07_cp04_Kton90_Ksc165", {
            "Kp_roll": 1.42, "Kd_roll": 1.10, "Ki_roll": 1.45,
            "K_ton_roll": -0.90, "p_ton_roll": 2.8,
            "K_sc_roll": 1.65, "p_sc_roll": 60.0,
            "K_phas_roll": 16.653, "p_phas_roll": 5.65,
            "Kp_prop_roll": 4.2, "z_ccr_roll": 0.105,
        }),
        ("W4_08_B1_cp04_Hybrid", {
            "Kp_roll": 1.30, "Kd_roll": 1.06, "Ki_roll": 1.50,
            "K_ton_roll": -0.75, "p_ton_roll": 3.0,
            "K_sc_roll": 1.72, "p_sc_roll": 58.0,
            "K_phas_roll": 17.2, "p_phas_roll": 5.2,
            "Kp_prop_roll": 4.5, "z_ccr_roll": 0.11,
        }),
    ]

    jobs = []
    for name, delta in bridge_defs:
        ov = dict(W2_CP_04_PARAMS)
        ov.update(delta)
        jobs.append({"kind": "LatEC", "name": name, "ov": ov})

    apeo_1 = dict(W2_CP_04_PARAMS)
    apeo_2 = dict(w3_map["W3_10_DeepNegTon95_Ki135"])
    apeo_3 = dict(w3_map["W3_11_B2Hybrid_LowKsc"])
    apeo_4 = dict(W2_CP_04_PARAMS)
    apeo_4.update({"Kp_pitch": 3.45, "Ki_pitch": 3.45, "K_ton": 4.95})

    jobs.extend([
        {"kind": "APEO", "name": "APEO_W2_cp_04_K_ton_roll", "ov": apeo_1},
        {"kind": "APEO", "name": "APEO_W3_10_DeepNegTon95", "ov": apeo_2},
        {"kind": "APEO", "name": "APEO_W3_11_B2Hybrid", "ov": apeo_3},
        {"kind": "APEO", "name": "APEO_Cotrim_Kp34_Ki34_Kt49", "ov": apeo_4},
    ])
    return jobs


def _worker(job):
    kind = job["kind"]
    name = job["name"]
    ov = job["ov"]
    params = dict(o.BASE_YAW_TUNED)
    params.update(ov)
    try:
        if kind == "LatEC":
            sim_res = o.run_full_axis_sim(("LatEC", f"w4_{name}", params, 2.0))
            if sim_res["aborted"]:
                return {"kind": kind, "name": name, "ok": False, "error": "aborted", "ov": ov}
            cls = compute_classic_bode_score(sim_res["freq_results"], is_apeo=False)
            tgt = o.compute_targeted_roll_score(sim_res)
            tgt["classic_bode_score"] = cls["classic_bode_score"]
            tgt["gain_loss_pts"] = cls["gain_loss_pts"]
            tgt["phase_loss_pts"] = cls["phase_loss_pts"]
            tgt["latec_cost"] = round(float(e.compute_latec_cost(sim_res)), 3)
            return {"kind": kind, "name": name, "ok": True, "metrics": tgt, "ov": ov}
        else:
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
                worker_name=f"w4_{name}",
                p_phas2=-1.0,
                z_phas=params["z_phas"],
                extra_props=params,
            )
            cls = compute_classic_bode_score(freq_results, is_apeo=True)
            max_db = max(abs(freq_results[k]["gain_db_diff"]) for k in freq_results) if freq_results else 999.0
            max_ph = max(abs(freq_results[k]["phase_err_deg"]) for k in freq_results) if freq_results else 999.0
            return {
                "kind": kind,
                "name": name,
                "ok": True,
                "apeo_score": round(float(total_score), 2),
                "classic_bode_score": cls["classic_bode_score"],
                "gain_loss_pts": cls["gain_loss_pts"],
                "phase_loss_pts": cls["phase_loss_pts"],
                "hf_penalty": telemetry.get("hf_penalty"),
                "hf_rms_pitch_deg": telemetry.get("hf_rms_pitch_deg"),
                "max_abs_roll_deg": telemetry.get("max_abs_roll_deg"),
                "max_abs_yaw_deg": telemetry.get("max_abs_yaw_deg"),
                "max_db": round(float(max_db), 2),
                "max_ph": round(float(max_ph), 1),
                "freq_results": freq_results,
                "ov": ov,
            }
    except Exception as ex:
        return {"kind": kind, "name": name, "ok": False, "error": str(ex), "ov": ov}


def main():
    os.makedirs(os.path.join(_ROOT, "logs", "roll_yaw_baseline"), exist_ok=True)
    jobs = build_wave4_jobs()
    print(f"Running Wave 4 Bridge + APEO Verification Batch ({len(jobs)} jobs, 12 workers)...", flush=True)
    t0 = time.time()
    lat_results = []
    apeo_results = []

    with ProcessPoolExecutor(max_workers=12) as ex:
        futs = [ex.submit(_worker, j) for j in jobs]
        for idx, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            if not r["ok"]:
                print(f"  [W4] {idx:2d}/{len(jobs)} ({time.time()-t0:5.0f}s) FAILED {r['kind']} {r['name']}: {r.get('error')}", flush=True)
                continue
            if r["kind"] == "LatEC":
                lat_results.append(r)
                m = r["metrics"]
                print(
                    f"  [W4] {idx:2d}/{len(jobs)} ({time.time()-t0:5.0f}s) LatEC {r['name']:26s} | "
                    f"Classic={m['classic_bode_score']:8.2f} pts (G={m['gain_loss_pts']:7.1f}, P={m['phase_loss_pts']:7.1f}) | "
                    f"LFph={m['ph_errs'][:3]} | HFdB={m['db_errs'][5:]} | noise={m['noise']:.2f} r={m['corr']:.3f}",
                    flush=True,
                )
            else:
                apeo_results.append(r)
                print(
                    f"  [W4] {idx:2d}/{len(jobs)} ({time.time()-t0:5.0f}s) APEO  {r['name']:26s} | "
                    f"Classic={r['classic_bode_score']:8.2f} pts (G={r['gain_loss_pts']:6.1f}, P={r['phase_loss_pts']:6.1f}) | "
                    f"hf_pen={r['hf_penalty']} | max_dB={r['max_db']:.2f} max_ph={r['max_ph']:.1f}",
                    flush=True,
                )

    lat_results.sort(key=lambda x: x["metrics"]["classic_bode_score"])
    apeo_results.sort(key=lambda x: x["classic_bode_score"])

    print(f"\n=== WAVE 4 LatEC ROLL RANKED BY CLASSIC BODE SCORE ({time.time()-t0:.1f}s) ===", flush=True)
    for r in lat_results:
        m = r["metrics"]
        print(
            f"  {r['name']:26s} | Classic={m['classic_bode_score']:8.2f} pts (G={m['gain_loss_pts']:7.1f}, P={m['phase_loss_pts']:7.1f}) | "
            f"noise={m['noise']:4.2f} r={m['corr']:.3f} | ph={m['ph_errs']} | dB={m['db_errs']}",
            flush=True,
        )

    print(f"\n=== WAVE 4 APEO PITCH CLASSIC BODE SCORES ===", flush=True)
    for r in apeo_results:
        print(
            f"  {r['name']:26s} | Classic={r['classic_bode_score']:8.2f} pts (G={r['gain_loss_pts']:6.1f}, P={r['phase_loss_pts']:6.1f}) | "
            f"hf_pen={r['hf_penalty']} | max_dB={r['max_db']:.2f} max_ph={r['max_ph']:.1f}",
            flush=True,
        )

    out_path = os.path.join(_ROOT, "logs", "roll_yaw_baseline", "ungated_wave4_classic_results.json")
    with open(out_path, "w") as f:
        json.dump({"lat_results": lat_results, "apeo_results": apeo_results}, f, indent=2)
    print(f"\nSaved Wave 4 Classic Bode results to {out_path}", flush=True)


if __name__ == "__main__":
    main()
