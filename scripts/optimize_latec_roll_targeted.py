import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
import sys
import math
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from run_roll_yaw_baseline import P15_RUN4
from evaluate_and_plot_tuned_3axis import run_full_axis_sim, run_apeo_pitch_check

# Fixed Yaw (Y4_hybrid) and Pitch (P15_RUN4) base
BASE_YAW_TUNED = {
    **P15_RUN4,
    "Kp_yaw": 0.24,
    "Kd_yaw": 0.03,
    "Ki_yaw": 0.0,
    "K_ton_yaw": P15_RUN4["K_ton"],
    "p_ton_yaw": P15_RUN4["p_ton"],
    "K_sc_yaw": 1.2,
    "p_sc_yaw": 3.5,
    "Kp_prop_yaw": 8.5,
    "z_ccr_yaw": 0.06,
    "deriv_comp_yaw": 0.25,
}

BASE_R4 = {
    "Kp_roll": 3.8,
    "Kd_roll": 1.4,
    "Ki_roll": 0.0,
    "K_ton_roll": 4.2,
    "p_ton_roll": P15_RUN4["p_ton"],
    "K_sc_roll": 3.2,
    "p_sc_roll": P15_RUN4["p_sc"],
    "z_sc_roll": 0.0,
    "K_phas_roll": 16.5,
    "p_phas_roll": 6.0,
    "z_phas_roll": 0.0,
    "Kp_prop_roll": -1000.0,
    "z_ccr_roll": 0.0,
    "p_ap_roll": -1.0,
    "c_ap_roll": 0.0,
    "deriv_comp_roll": 0.0,
}

ROLL_KEYS = list(BASE_R4.keys()) + ["Kp_prop_yaw", "z_ccr_yaw"]


def compute_targeted_roll_score(sim_res):
    fr = sim_res["freq_results"]
    freqs = sim_res["freqs"]
    db_errs = [fr[f"{f:.4f}"]["gain_db_diff"] for f in freqs]
    ph_errs = [fr[f"{f:.4f}"]["phase_err"] for f in freqs]
    noise = float(sim_res["hf_noise_rms"])
    corr = float(sim_res["corr_vel"])

    # Discrepancy 1: Low-freq phase (0.1209, 0.1986, 0.3214 Hz)
    lf_ph_loss = sum((ph_errs[i] / 12.0) ** 2 for i in [0, 1, 2])
    lf_ph_cap = sum(max(0.0, abs(ph_errs[i]) - 22.0) * 0.25 for i in [0, 1, 2])

    # Discrepancy 2: High-freq gain (1.4040, 2.0936, 3.9915 Hz)
    hf_db_loss = 2.2 * sum((db_errs[i] / 1.2) ** 2 for i in [5, 6, 7])
    hf_db_cap = sum(max(0.0, abs(db_errs[i]) - 1.8) * 1.5 for i in [5, 6, 7])

    # Preserve good mid/HF phase (0.5588..3.9915 Hz)
    mid_hf_ph_loss = 1.4 * sum((ph_errs[i] / 10.0) ** 2 for i in [3, 4, 5, 6, 7])
    mid_hf_ph_cap = sum(max(0.0, abs(ph_errs[i]) - 15.0) * 0.6 for i in [3, 4, 5, 6, 7])

    # Preserve LF/Mid gain (0.1209..0.8722 Hz)
    lf_mid_db_loss = 0.8 * sum((db_errs[i] / 2.0) ** 2 for i in [0, 1, 2, 3, 4])

    # Preserve smooth velocity & correlation
    noise_pen = max(0.0, noise - 2.65) * 4.0 + 0.15 * (noise ** 2)
    corr_pen = max(0.0, 0.86 - corr) * 35.0

    # Stability sanity check on steady-state roll angle
    t = sim_res["t"]
    roll = sim_res["roll"]
    mask_ss = t >= 3.0
    max_roll = float(np.max(np.abs(roll[mask_ss]))) if np.any(mask_ss) else 999.0
    stab_pen = max(0.0, max_roll - 12.0) * 10.0

    total = (
        lf_ph_loss + lf_ph_cap
        + hf_db_loss + hf_db_cap
        + mid_hf_ph_loss + mid_hf_ph_cap
        + lf_mid_db_loss
        + noise_pen + corr_pen + stab_pen
    )
    return {
        "score": float(total),
        "lf_ph_max": float(max(abs(ph_errs[i]) for i in [0, 1, 2])),
        "hf_db_max": float(max(abs(db_errs[i]) for i in [5, 6, 7])),
        "midhf_ph_max": float(max(abs(ph_errs[i]) for i in [3, 4, 5, 6, 7])),
        "db_errs": [round(x, 2) for x in db_errs],
        "ph_errs": [round(x, 1) for x in ph_errs],
        "gains": [round(fr[f"{f:.4f}"]["sim_gain"], 1) for f in freqs],
        "phases": [round(fr[f"{f:.4f}"]["sim_phase"], 1) for f in freqs],
        "noise": round(noise, 3),
        "corr": round(corr, 4),
        "max_roll": round(max_roll, 2),
    }


def _eval_worker(args):
    name, roll_ov = args
    p = dict(BASE_YAW_TUNED)
    p.update(BASE_R4)
    p.update(roll_ov)
    try:
        res = run_full_axis_sim(("LatEC", name, p, 2.0))
        m = compute_targeted_roll_score(res)
        return {
            "name": name,
            "ok": True,
            "roll_ov": {k: p[k] for k in ROLL_KEYS},
            "metrics": m,
        }
    except Exception as exc:
        return {
            "name": name,
            "ok": False,
            "error": str(exc),
            "roll_ov": {k: p[k] for k in ROLL_KEYS},
            "metrics": {"score": 1e9},
        }


def build_wave1_candidates():
    # 24 targeted candidates around W2_rnd_12 and R4 (ungated simultaneous 3-axis controller)
    w2_base = {
        "Kp_roll": 1.637, "Kd_roll": 1.140, "Ki_roll": 0.375,
        "K_ton_roll": 1.423, "p_ton_roll": 5.208,
        "K_sc_roll": 2.029, "p_sc_roll": 60.0, "z_sc_roll": 0.0,
        "K_phas_roll": 16.653, "p_phas_roll": 5.747, "z_phas_roll": 0.0,
        "Kp_prop_roll": 3.533, "z_ccr_roll": 0.125, "deriv_comp_roll": 0.829,
    }
    w2_dc0 = {**w2_base, "deriv_comp_roll": 0.0}

    cands = [
        # 1-4: Clean baselines & cross-axis Yaw CCR sensitivity checks
        ("W1_00_R4_dc0", {}),
        ("W1_01_W2rnd12", dict(w2_base)),
        ("W1_02_W2rnd12_dc0", dict(w2_dc0)),
        ("W1_03_W2dc0_zy0", {**w2_dc0, "Kp_prop_yaw": 6.0, "z_ccr_yaw": 0.0}),
        # 5-12: LF phase pull-down on top of W2rnd12_dc0 (stronger Ki_roll / Kp_prop_roll, lighter Kp/K_ton)
        ("LF1_Ki10_prop45", {**w2_dc0, "Kp_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Ki_roll": 1.0, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.10}),
        ("LF2_Ki14_prop50", {**w2_dc0, "Kp_roll": 1.0, "K_ton_roll": 0.6, "p_ton_roll": 3.0, "Ki_roll": 1.4, "Kp_prop_roll": 5.0, "z_ccr_roll": 0.10}),
        ("LF3_Ki18_prop55", {**w2_dc0, "Kp_roll": 0.8, "K_ton_roll": 0.4, "p_ton_roll": 2.8, "Ki_roll": 1.8, "Kp_prop_roll": 5.5, "z_ccr_roll": 0.09}),
        ("LF4_Ki22_kt02",   {**w2_dc0, "Kp_roll": 0.7, "K_ton_roll": 0.2, "p_ton_roll": 2.5, "Ki_roll": 2.2, "Kp_prop_roll": 5.0, "z_ccr_roll": 0.08}),
        ("LF5_prop60_z08",  {**w2_dc0, "Kp_roll": 0.8, "K_ton_roll": 0.5, "p_ton_roll": 3.5, "Ki_roll": 1.2, "Kp_prop_roll": 6.0, "z_ccr_roll": 0.08}),
        ("LF6_Ki16_zy0",    {**w2_dc0, "Kp_roll": 1.0, "K_ton_roll": 0.6, "p_ton_roll": 3.0, "Ki_roll": 1.6, "Kp_prop_roll": 5.0, "z_ccr_roll": 0.09, "Kp_prop_yaw": 6.0, "z_ccr_yaw": 0.0}),
        ("LF7_negTon_Ki15", {**w2_dc0, "Kp_roll": 1.4, "K_ton_roll": -0.6, "p_ton_roll": 3.0, "Ki_roll": 1.5, "Kp_prop_roll": 4.2, "z_ccr_roll": 0.10}),
        ("LF8_R4_Ki15_prop40", {"Kp_roll": 1.4, "Kd_roll": 1.25, "Ki_roll": 1.5, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "K_sc_roll": 2.4, "p_sc_roll": 50.0, "K_phas_roll": 16.5, "p_phas_roll": 5.8, "Kp_prop_roll": 4.0, "z_ccr_roll": 0.10}),
        # 13-20: HF gain flattening (cut 1.40-2.09 Hz peak & lift 3.99 Hz toward 495 deg/m)
        ("HF1_psc35_Kd10",   {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Kd_roll": 1.0, "K_sc_roll": 2.4, "p_sc_roll": 35.0, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.10}),
        ("HF2_pph42_Kph18",  {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Kd_roll": 0.95, "K_phas_roll": 18.5, "p_phas_roll": 4.2, "K_sc_roll": 1.8, "p_sc_roll": 55.0, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.12}),
        ("HF3_zph010_Kph14", {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Kd_roll": 1.20, "K_sc_roll": 2.8, "p_sc_roll": 48.0, "K_phas_roll": 14.0, "p_phas_roll": 5.0, "z_phas_roll": 0.010, "Kp_prop_roll": 4.2, "z_ccr_roll": 0.10}),
        ("HF4_zsc008_pph45", {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Kd_roll": 1.05, "K_sc_roll": 2.6, "p_sc_roll": 45.0, "z_sc_roll": 0.008, "K_phas_roll": 15.5, "p_phas_roll": 4.5, "Kp_prop_roll": 4.2, "z_ccr_roll": 0.11}),
        ("HF5_Kd08_z015",    {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Kd_roll": 0.80, "K_sc_roll": 2.2, "p_sc_roll": 60.0, "K_phas_roll": 19.5, "p_phas_roll": 4.5, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.15}),
        ("HF6_dc03_Ksc18",   {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.3, "K_ton_roll": 0.7, "p_ton_roll": 3.5, "Kd_roll": 1.00, "K_sc_roll": 1.8, "p_sc_roll": 60.0, "K_phas_roll": 17.5, "p_phas_roll": 5.2, "Kp_prop_roll": 4.6, "z_ccr_roll": 0.11, "deriv_comp_roll": 0.30}),
        ("HF7_Kph21_pph36",  {**w2_dc0, "Kp_roll": 1.1, "Ki_roll": 1.3, "K_ton_roll": 0.7, "p_ton_roll": 3.5, "Kd_roll": 0.90, "K_sc_roll": 2.2, "p_sc_roll": 58.0, "K_phas_roll": 21.0, "p_phas_roll": 3.6, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.12}),
        ("HF8_R4_noYawVel",  {**w2_dc0, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 0.8, "p_ton_roll": 3.5, "Kd_roll": 1.00, "K_sc_roll": 2.4, "p_sc_roll": 50.0, "K_phas_roll": 18.0, "p_phas_roll": 4.5, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.11, "Kp_prop_yaw": 5.0, "z_ccr_yaw": 0.02}),
        # 21-24: Balanced joint candidates
        ("B1_joint_a",     {**w2_dc0, "Kp_roll": 1.1, "Ki_roll": 1.5, "K_ton_roll": 0.6, "p_ton_roll": 3.2, "Kd_roll": 0.95, "K_sc_roll": 2.3, "p_sc_roll": 45.0, "K_phas_roll": 18.0, "p_phas_roll": 4.5, "Kp_prop_roll": 4.8, "z_ccr_roll": 0.11}),
        ("B2_joint_b",     {**w2_dc0, "Kp_roll": 0.9, "Ki_roll": 1.7, "K_ton_roll": 0.5, "p_ton_roll": 3.0, "Kd_roll": 1.00, "K_sc_roll": 2.1, "p_sc_roll": 52.0, "K_phas_roll": 19.0, "p_phas_roll": 4.2, "Kp_prop_roll": 5.2, "z_ccr_roll": 0.10}),
        ("B3_joint_c",     {**w2_dc0, "Kp_roll": 1.3, "Ki_roll": 1.2, "K_ton_roll": 0.9, "p_ton_roll": 3.8, "Kd_roll": 1.10, "K_sc_roll": 2.5, "p_sc_roll": 42.0, "z_phas_roll": 0.008, "K_phas_roll": 16.5, "p_phas_roll": 4.8, "Kp_prop_roll": 4.2, "z_ccr_roll": 0.12}),
        ("B4_joint_d_zy0", {**w2_dc0, "Kp_roll": 1.0, "Ki_roll": 1.5, "K_ton_roll": 0.6, "p_ton_roll": 3.2, "Kd_roll": 0.95, "K_sc_roll": 2.3, "p_sc_roll": 48.0, "K_phas_roll": 18.5, "p_phas_roll": 4.3, "Kp_prop_roll": 5.0, "z_ccr_roll": 0.11, "Kp_prop_yaw": 6.5, "z_ccr_yaw": 0.0}),
    ]
    return cands


def clamp_roll_params(p):
    p = dict(p)
    p["Kp_roll"] = float(np.clip(p["Kp_roll"], 0.1, 6.0))
    p["Kd_roll"] = float(np.clip(p["Kd_roll"], 0.4, 3.0))
    p["Ki_roll"] = float(np.clip(p["Ki_roll"], 0.0, 4.0))
    p["K_ton_roll"] = float(np.clip(p["K_ton_roll"], -2.5, 6.0))
    p["p_ton_roll"] = float(np.clip(p["p_ton_roll"], 0.6, 25.0))
    p["K_sc_roll"] = float(np.clip(p["K_sc_roll"], 1.2, 7.0))
    p["p_sc_roll"] = float(np.clip(p["p_sc_roll"], 15.0, 65.0))
    p["z_sc_roll"] = float(np.clip(p["z_sc_roll"], 0.0, 0.025))
    p["K_phas_roll"] = float(np.clip(p["K_phas_roll"], 5.0, 25.0))
    p["p_phas_roll"] = float(np.clip(p["p_phas_roll"], 2.0, 12.0))
    p["z_phas_roll"] = float(np.clip(p["z_phas_roll"], 0.0, 0.025))
    if p["Kp_prop_roll"] > -500.0:
        p["Kp_prop_roll"] = float(np.clip(p["Kp_prop_roll"], -3.0, 8.0))
    p["z_ccr_roll"] = float(np.clip(p["z_ccr_roll"], 0.0, 0.25))
    p["p_ap_roll"] = -1.0
    p["c_ap_roll"] = 0.0
    p["deriv_comp_roll"] = float(np.clip(p["deriv_comp_roll"], 0.0, 1.5))
    p["Kp_prop_yaw"] = float(np.clip(p.get("Kp_prop_yaw", BASE_YAW_TUNED["Kp_prop_yaw"]), 2.0, 10.0))
    p["z_ccr_yaw"] = float(np.clip(p.get("z_ccr_yaw", BASE_YAW_TUNED["z_ccr_yaw"]), 0.0, 0.08))
    return p


def build_wave_from_seeds(seeds, wave_idx, rng, n_cands=12):
    cands = []
    scales = {
        "Kp_roll": 0.25,
        "Kd_roll": 0.15,
        "Ki_roll": 0.35,
        "K_ton_roll": 0.25,
        "p_ton_roll": 0.25,
        "K_sc_roll": 0.35,
        "p_sc_roll": 0.20,
        "z_sc_roll": 0.005,
        "K_phas_roll": 1.5,
        "p_phas_roll": 0.6,
        "z_phas_roll": 0.006,
        "Kp_prop_roll": 0.60,
        "z_ccr_roll": 0.02,
        "deriv_comp_roll": 0.15,
    }
    top0 = dict(seeds[0]["roll_ov"])
    coord_probes = [
        ("Ki_roll", +0.40), ("Ki_roll", -0.40),
        ("Kp_prop_roll", +0.60), ("Kp_roll", -0.25),
        ("K_ton_roll", -0.25), ("K_phas_roll", +1.8),
        ("p_phas_roll", -0.7), ("Kd_roll", -0.15),
    ]
    for i, (k, d) in enumerate(coord_probes):
        if len(cands) >= 6:
            break
        p = dict(top0)
        p[k] = p[k] + d
        cands.append((f"W{wave_idx}_cp_{i:02d}_{k}", clamp_roll_params(p)))

    while len(cands) < n_cands:
        idx = len(cands)
        seed = seeds[idx % min(len(seeds), 4)]
        p = dict(seed["roll_ov"])
        for k in ["Kp_roll", "Kd_roll", "Ki_roll", "K_ton_roll", "K_sc_roll", "K_phas_roll", "p_phas_roll", "deriv_comp_roll"]:
            p[k] += float(rng.normal(0.0, scales[k]))
        p["p_ton_roll"] *= float(math.exp(rng.normal(0.0, scales["p_ton_roll"])))
        p["p_sc_roll"] *= float(math.exp(rng.normal(0.0, scales["p_sc_roll"])))
        if p["Kp_prop_roll"] > -500.0:
            p["Kp_prop_roll"] += float(rng.normal(0.0, scales["Kp_prop_roll"]))
            p["z_ccr_roll"] += float(rng.normal(0.0, scales["z_ccr_roll"]))
        cands.append((f"W{wave_idx}_rnd_{idx:02d}", clamp_roll_params(p)))

    return cands[:n_cands]


def _yaw_check_worker(r):
    p = dict(BASE_YAW_TUNED)
    p.update(r["roll_ov"])
    sim_yaw = run_full_axis_sim(("YawMA", f"check_yaw_{r['name']}", p, 2.0))
    yaw_fr = sim_yaw["freq_results"]
    yaw_max_db = max(abs(yaw_fr[k]["gain_db_diff"]) for k in yaw_fr)
    yaw_max_ph = max(abs(yaw_fr[k]["phase_err"]) for k in yaw_fr)
    return {
        "name": r["name"],
        "roll_ov": r["roll_ov"],
        "lat_metrics": r["metrics"],
        "yaw_summary": {
            "hf_noise_rms": round(float(sim_yaw["hf_noise_rms"]), 3),
            "corr_vel": round(float(sim_yaw["corr_vel"]), 4),
            "max_db": round(float(yaw_max_db), 2),
            "max_ph": round(float(yaw_max_ph), 1),
        },
    }


def run_wave(wave_name, cands, max_workers=12):
    t0 = time.time()
    results = []
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        futs = [ex.submit(_eval_worker, c) for c in cands]
        for idx, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            results.append(r)
            m = r["metrics"]
            if r["ok"]:
                print(
                    f"  [{wave_name}] {idx}/{len(cands)} ({time.time()-t0:.0f}s) | {r['name']:22s} J={m['score']:6.1f} | "
                    f"LFph={m['ph_errs'][:3]} | HFdB={m['db_errs'][5:]} | noise={m['noise']:.2f} r={m['corr']:.3f}",
                    flush=True,
                )
            else:
                print(f"  [{wave_name}] {idx}/{len(cands)} FAILED {r['name']}: {r.get('error')}", flush=True)
    results.sort(key=lambda x: x["metrics"]["score"])
    dt = time.time() - t0
    print(f"\n=== {wave_name}: {len(results)} evaluated in {dt:.1f}s (TOP 8) ===", flush=True)
    for r in results[:8]:
        m = r["metrics"]
        if not r["ok"]:
            continue
        print(
            f"  {r['name']:22s} | J={m['score']:6.2f} | LF_ph_max={m['lf_ph_max']:4.1f}deg | HF_dB_max={m['hf_db_max']:4.2f} | "
            f"midHF_ph={m['midhf_ph_max']:4.1f}deg | noise={m['noise']:4.2f} | r={m['corr']:.3f} | "
            f"ph={m['ph_errs']} | dB={m['db_errs']}",
            flush=True,
        )
    return results


def main():
    rng = np.random.default_rng(42)
    n_workers = min(12, max(8, (os.cpu_count() or 8) - 4))
    print(f"[INFO] Starting Unified Ungated Roll Optimization on {n_workers} workers (CPU count={os.cpu_count()})", flush=True)
    all_waves = {}

    w1_cands = build_wave1_candidates()
    w1_res = run_wave("WAVE 1 (24 Surgical Exploration)", w1_cands, max_workers=n_workers)
    all_waves["wave1"] = w1_res[:16]

    valid_w1 = [r for r in w1_res if r["ok"]]
    w2_cands = build_wave_from_seeds(valid_w1[:5], 2, rng, n_cands=12)
    w2_res = run_wave("WAVE 2 (12 Local Refinement)", w2_cands, max_workers=n_workers)
    all_waves["wave2"] = w2_res[:12]

    best_all = sorted([r for r in (w1_res + w2_res) if r["ok"]], key=lambda x: x["metrics"]["score"])
    top3 = best_all[:3]
    print("\n=== PARALLEL CROSS-AXIS CHECK OF TOP 3 ON YawMA & SEQUENTIAL APEO CHECK ===", flush=True)
    with ProcessPoolExecutor(max_workers=3) as ex:
        verified = list(ex.map(_yaw_check_worker, top3))
    for v in verified:
        p = dict(BASE_YAW_TUNED)
        p.update(v["roll_ov"])
        apeo_res = run_apeo_pitch_check(p)
        p_fr = apeo_res["freq_results"]
        v["pitch_summary"] = {
            "apeo_score": round(float(apeo_res["total_score"]), 2),
            "max_db": round(float(max(abs(p_fr[k]["gain_db_diff"]) for k in p_fr)), 2),
            "max_ph": round(float(max(abs(p_fr[k]["phase_diff"]) for k in p_fr)), 1),
        }
        ys = v["yaw_summary"]
        ps = v["pitch_summary"]
        print(
            f"  {v['name']:22s} | LatEC J={v['lat_metrics']['score']:.2f} | "
            f"YawMA noise={ys['hf_noise_rms']:.2f}, corr={ys['corr_vel']:.3f}, maxdB={ys['max_db']:.2f}, maxPh={ys['max_ph']:.1f}deg | "
            f"APEO score={ps['apeo_score']:.2f}, maxdB={ps['max_db']:.2f}, maxPh={ps['max_ph']:.1f}deg",
            flush=True,
        )

    out_path = os.path.join(_ROOT, "logs/roll_yaw_baseline/roll_targeted_opt_results.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"top_verified": verified, "waves": all_waves}, f, indent=2)
    print(f"[OK] Saved optimization summary to {out_path}", flush=True)


if __name__ == "__main__":
    main()
