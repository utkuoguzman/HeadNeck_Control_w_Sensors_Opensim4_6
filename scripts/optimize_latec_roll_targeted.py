import os
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
from evaluate_and_plot_tuned_3axis import run_full_axis_sim

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
    "deriv_comp_roll": 1.0,
}

ROLL_KEYS = list(BASE_R4.keys())


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
    # 32 physics-guided candidates targeting LF phase (0.12-0.32 Hz) & HF gain (1.40-3.99 Hz)
    cands = [("W1_00_R4_ref", {})]

    hf_a = {"K_sc_roll": 5.0, "p_sc_roll": 14.0, "K_phas_roll": 11.5, "p_phas_roll": 4.5, "z_phas_roll": 0.032, "Kd_roll": 1.65}
    hf_b = {"K_sc_roll": 5.8, "p_sc_roll": 11.5, "K_phas_roll": 10.0, "p_phas_roll": 4.2, "z_phas_roll": 0.045, "Kd_roll": 1.75}
    hf_c = {"K_sc_roll": 4.6, "p_sc_roll": 16.0, "K_phas_roll": 9.0,  "p_phas_roll": 4.8, "z_phas_roll": 0.055, "Kd_roll": 1.85, "z_sc_roll": 0.006}

    # Group 1 (12): Swap GIA/task stiffness (Kp_roll, K_ton_roll low or negative) for pure neck Kp_prop_roll (+2.0..+6.5)
    g1 = [
        ("G1_prop25_kt10", {**hf_a, "Kp_roll": 1.5, "K_ton_roll": 1.0,  "p_ton_roll": 4.0, "Kp_prop_roll": 2.5, "z_ccr_roll": 0.06}),
        ("G1_prop40_kt05", {**hf_a, "Kp_roll": 1.2, "K_ton_roll": 0.5,  "p_ton_roll": 3.5, "Kp_prop_roll": 4.0, "z_ccr_roll": 0.08}),
        ("G1_prop55_kt00", {**hf_a, "Kp_roll": 1.0, "K_ton_roll": 0.0,  "p_ton_roll": 4.0, "Kp_prop_roll": 5.5, "z_ccr_roll": 0.08}),
        ("G1_prop65_ktm10", {**hf_a, "Kp_roll": 0.8, "K_ton_roll": -1.0, "p_ton_roll": 5.0, "Kp_prop_roll": 6.5, "z_ccr_roll": 0.10}),
        ("G1_prop35_ktm15", {**hf_b, "Kp_roll": 0.6, "K_ton_roll": -1.5, "p_ton_roll": 4.0, "Kp_prop_roll": 3.5, "z_ccr_roll": 0.12}),
        ("G1_prop45_kt18", {**hf_b, "Kp_roll": 1.6, "K_ton_roll": 1.8,  "p_ton_roll": 2.5, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.05}),
        ("G1_prop60_kt12", {**hf_b, "Kp_roll": 2.0, "K_ton_roll": 1.2,  "p_ton_roll": 6.0, "Kp_prop_roll": 6.0, "z_ccr_roll": 0.04}),
        ("G1_prop30_kt22", {**hf_c, "Kp_roll": 2.2, "K_ton_roll": 2.2,  "p_ton_roll": 3.0, "Kp_prop_roll": 3.0, "z_ccr_roll": 0.08}),
        ("G1_prop50_ktm05", {**hf_c, "Kp_roll": 1.0, "K_ton_roll": -0.5, "p_ton_roll": 4.5, "Kp_prop_roll": 5.0, "z_ccr_roll": 0.10}),
        ("G1_prop20_R4hf", {"Kp_roll": 1.5, "K_ton_roll": 1.0, "p_ton_roll": 4.0, "Kp_prop_roll": 3.5, "z_ccr_roll": 0.08}),
        ("G1_prop65_R4hf", {"Kp_roll": 0.8, "K_ton_roll": -1.0, "p_ton_roll": 4.0, "Kp_prop_roll": 6.0, "z_ccr_roll": 0.08}),
        ("G1_zeroKpKt_prop45", {**hf_b, "Kp_roll": 0.4, "K_ton_roll": 0.2, "p_ton_roll": 2.0, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.14}),
    ]
    cands.extend(g1)

    # Group 2 (8): Integral phase lag Ki_roll (0.8..2.5) + lowered Kp/K_ton + HF shaping
    g2 = [
        ("G2_Ki08_Kp18", {**hf_a, "Kp_roll": 1.8, "Ki_roll": 0.8, "K_ton_roll": 2.4, "p_ton_roll": 3.5}),
        ("G2_Ki14_Kp18", {**hf_a, "Kp_roll": 1.8, "Ki_roll": 1.4, "K_ton_roll": 2.0, "p_ton_roll": 3.5}),
        ("G2_Ki20_Kp22", {**hf_b, "Kp_roll": 2.2, "Ki_roll": 2.0, "K_ton_roll": 2.2, "p_ton_roll": 4.5}),
        ("G2_Ki25_Kp26", {**hf_b, "Kp_roll": 2.6, "Ki_roll": 2.5, "K_ton_roll": 2.6, "p_ton_roll": 5.0}),
        ("G2_Ki12_prop30", {**hf_a, "Kp_roll": 1.2, "Ki_roll": 1.2, "K_ton_roll": 1.0, "p_ton_roll": 3.0, "Kp_prop_roll": 3.0, "z_ccr_roll": 0.06}),
        ("G2_Ki18_prop45", {**hf_b, "Kp_roll": 1.0, "Ki_roll": 1.8, "K_ton_roll": 0.5, "p_ton_roll": 4.0, "Kp_prop_roll": 4.5, "z_ccr_roll": 0.08}),
        ("G2_Ki12_pt14",   {**hf_c, "Kp_roll": 1.6, "Ki_roll": 1.2, "K_ton_roll": 2.0, "p_ton_roll": 1.4}),
        ("G2_Ki15_R4hf",   {"Kp_roll": 2.0, "Ki_roll": 1.5, "K_ton_roll": 2.2, "p_ton_roll": 3.5}),
    ]
    cands.extend(g2)

    # Group 3 (6): Negative Kp_prop_roll & very low p_ton_roll (1.2..2.5) + HF shaping
    g3 = [
        ("G3_negProp12_a", {**hf_a, "Kp_roll": 2.4, "K_ton_roll": 2.6, "p_ton_roll": 3.0, "Kp_prop_roll": -1.2, "z_ccr_roll": 0.04}),
        ("G3_negProp22_b", {**hf_b, "Kp_roll": 1.6, "K_ton_roll": 1.8, "p_ton_roll": 2.2, "Kp_prop_roll": -2.2, "z_ccr_roll": 0.02}),
        ("G3_negProp18_c", {**hf_c, "Kp_roll": 2.0, "K_ton_roll": 2.0, "p_ton_roll": 2.0, "Kp_prop_roll": -1.8, "z_ccr_roll": 0.05}),
        ("G3_lowKpKt_pt15", {**hf_a, "Kp_roll": 1.2, "K_ton_roll": 1.4, "p_ton_roll": 1.5}),
        ("G3_veryLow_pt12", {**hf_b, "Kp_roll": 0.6, "K_ton_roll": 0.8, "p_ton_roll": 1.2}),
        ("G3_hfC_only",     {**hf_c, "Kp_roll": 2.8, "K_ton_roll": 3.2, "p_ton_roll": 4.5}),
    ]
    cands.extend(g3)

    # Group 4 (5): GIA all-pass branch (p_ap_roll / c_ap_roll) + HF shaping
    g4 = [
        ("G4_ap3_cap04",    {**hf_a, "Kp_roll": 1.8, "K_ton_roll": 0.3,  "p_ap_roll": 3.0, "c_ap_roll": 0.4}),
        ("G4_ap5_capm05",   {**hf_a, "Kp_roll": 1.6, "K_ton_roll": 0.2,  "p_ap_roll": 5.0, "c_ap_roll": -0.5}),
        ("G4_ap25_cap08",   {**hf_b, "Kp_roll": 1.8, "K_ton_roll": -0.3, "p_ap_roll": 2.5, "c_ap_roll": 0.8}),
        ("G4_ap4_prop35",   {**hf_b, "Kp_roll": 1.2, "K_ton_roll": 0.2,  "p_ap_roll": 4.0, "c_ap_roll": 0.5, "Kp_prop_roll": 3.5, "z_ccr_roll": 0.08}),
        ("G4_ap3_Ki12",     {**hf_c, "Kp_roll": 1.5, "Ki_roll": 1.2, "K_ton_roll": 0.2, "p_ap_roll": 3.0, "c_ap_roll": 0.3}),
    ]
    cands.extend(g4)
    return cands


def clamp_roll_params(p):
    p["Kp_roll"] = float(np.clip(p["Kp_roll"], 0.1, 6.0))
    p["Kd_roll"] = float(np.clip(p["Kd_roll"], 0.5, 3.0))
    p["Ki_roll"] = float(np.clip(p["Ki_roll"], 0.0, 5.0))
    p["K_ton_roll"] = float(np.clip(p["K_ton_roll"], -2.5, 6.0))
    p["p_ton_roll"] = float(np.clip(p["p_ton_roll"], 0.6, 25.0))
    p["K_sc_roll"] = float(np.clip(p["K_sc_roll"], 2.0, 9.0))
    p["p_sc_roll"] = float(np.clip(p["p_sc_roll"], 6.0, 60.0))
    p["z_sc_roll"] = float(np.clip(p["z_sc_roll"], 0.0, 0.04))
    p["K_phas_roll"] = float(np.clip(p["K_phas_roll"], 5.0, 20.0))
    p["p_phas_roll"] = float(np.clip(p["p_phas_roll"], 2.0, 12.0))
    p["z_phas_roll"] = float(np.clip(p["z_phas_roll"], 0.0, 0.09))
    if p["Kp_prop_roll"] > -500.0:
        p["Kp_prop_roll"] = float(np.clip(p["Kp_prop_roll"], -4.0, 8.0))
    p["z_ccr_roll"] = float(np.clip(p["z_ccr_roll"], 0.0, 0.30))
    if p["p_ap_roll"] > 0.0:
        p["p_ap_roll"] = float(np.clip(p["p_ap_roll"], 0.8, 20.0))
        p["c_ap_roll"] = float(np.clip(p["c_ap_roll"], -3.0, 3.0))
    p["deriv_comp_roll"] = float(np.clip(p["deriv_comp_roll"], 0.2, 1.5))
    return p


def build_wave_from_seeds(seeds, wave_idx, rng, n_cands=56):
    cands = []
    # Coordinate probes around top 2 seeds + random perturbations around top 5 seeds
    scales = {
        "Kp_roll": 0.45 if wave_idx == 2 else 0.20,
        "Kd_roll": 0.25 if wave_idx == 2 else 0.12,
        "Ki_roll": 0.50 if wave_idx == 2 else 0.25,
        "K_ton_roll": 0.45 if wave_idx == 2 else 0.20,
        "p_ton_roll": 0.30,  # multiplicative fractional
        "K_sc_roll": 0.60 if wave_idx == 2 else 0.30,
        "p_sc_roll": 0.22,   # multiplicative fractional
        "z_sc_roll": 0.005 if wave_idx == 2 else 0.003,
        "K_phas_roll": 1.2 if wave_idx == 2 else 0.6,
        "p_phas_roll": 0.8 if wave_idx == 2 else 0.4,
        "z_phas_roll": 0.008 if wave_idx == 2 else 0.004,
        "Kp_prop_roll": 0.50 if wave_idx == 2 else 0.25,
        "z_ccr_roll": 0.04 if wave_idx == 2 else 0.02,
        "p_ap_roll": 0.25,   # multiplicative
        "c_ap_roll": 0.30 if wave_idx == 2 else 0.15,
        "deriv_comp_roll": 0.20 if wave_idx == 2 else 0.10,
    }
    # Deterministic coordinate perturbations around top seed
    top0 = dict(seeds[0]["roll_ov"])
    coord_probes = [
        ("Kp_roll", -scales["Kp_roll"]), ("Kp_roll", +scales["Kp_roll"]),
        ("Ki_roll", +scales["Ki_roll"]), ("Ki_roll", -scales["Ki_roll"]),
        ("K_ton_roll", -scales["K_ton_roll"]), ("K_ton_roll", +scales["K_ton_roll"]),
        ("p_ton_roll_mul", 0.65), ("p_ton_roll_mul", 1.5),
        ("K_sc_roll", +scales["K_sc_roll"]), ("K_sc_roll", -scales["K_sc_roll"]),
        ("p_sc_roll_mul", 0.78), ("p_sc_roll_mul", 1.28),
        ("z_phas_roll", +scales["z_phas_roll"]), ("z_phas_roll", -scales["z_phas_roll"]),
        ("K_phas_roll", -scales["K_phas_roll"]), ("K_phas_roll", +scales["K_phas_roll"]),
        ("p_phas_roll", -scales["p_phas_roll"]), ("p_phas_roll", +scales["p_phas_roll"]),
        ("Kd_roll", +scales["Kd_roll"]), ("Kd_roll", -scales["Kd_roll"]),
        ("z_sc_roll", +scales["z_sc_roll"]),
        ("deriv_comp_roll", -scales["deriv_comp_roll"]), ("deriv_comp_roll", +scales["deriv_comp_roll"]),
    ]
    for i, (k, d) in enumerate(coord_probes):
        if len(cands) >= (12 if wave_idx == 2 else 6):
            break
        p = dict(top0)
        if k == "p_ton_roll_mul":
            p["p_ton_roll"] *= d
        elif k == "p_sc_roll_mul":
            p["p_sc_roll"] *= d
        else:
            p[k] = p[k] + d
        cands.append((f"W{wave_idx}_cp_{i:02d}_{k}", clamp_roll_params(p)))

    while len(cands) < n_cands:
        idx = len(cands)
        seed = seeds[idx % min(len(seeds), 5)]
        p = dict(seed["roll_ov"])
        for k in ["Kp_roll", "Kd_roll", "Ki_roll", "K_ton_roll", "K_sc_roll", "z_sc_roll", "K_phas_roll", "p_phas_roll", "z_phas_roll", "deriv_comp_roll"]:
            p[k] += float(rng.normal(0.0, scales[k]))
        p["p_ton_roll"] *= float(math.exp(rng.normal(0.0, scales["p_ton_roll"])))
        p["p_sc_roll"] *= float(math.exp(rng.normal(0.0, scales["p_sc_roll"])))
        if p["Kp_prop_roll"] > -500.0:
            p["Kp_prop_roll"] += float(rng.normal(0.0, scales["Kp_prop_roll"]))
            p["z_ccr_roll"] += float(rng.normal(0.0, scales["z_ccr_roll"]))
        elif rng.random() < 0.25:
            p["Kp_prop_roll"] = float(rng.uniform(-2.0, 5.0))
            p["z_ccr_roll"] = float(rng.uniform(0.0, 0.12))
        if p["p_ap_roll"] > 0.0:
            p["p_ap_roll"] *= float(math.exp(rng.normal(0.0, scales["p_ap_roll"])))
            p["c_ap_roll"] += float(rng.normal(0.0, scales["c_ap_roll"]))
        cands.append((f"W{wave_idx}_rnd_{idx:02d}", clamp_roll_params(p)))

    return cands[:n_cands]


def _yaw_check_worker(r):
    p = dict(BASE_YAW_TUNED)
    p.update(r["roll_ov"])
    sim_yaw = run_full_axis_sim(("YawMA", f"check_{r['name']}", p, 2.0))
    yaw_fr = sim_yaw["freq_results"]
    yaw_max_db = max(abs(yaw_fr[k]["gain_db_diff"]) for k in yaw_fr)
    yaw_max_ph = max(abs(yaw_fr[k]["phase_err"]) for k in yaw_fr)
    return {
        "name": r["name"],
        "roll_ov": r["roll_ov"],
        "lat_metrics": r["metrics"],
        "yaw_summary": {
            "hf_noise_rms": sim_yaw["hf_noise_rms"],
            "corr_vel": sim_yaw["corr_vel"],
            "max_db": yaw_max_db,
            "max_ph": yaw_max_ph,
        },
    }


def run_wave(wave_name, cands, max_workers=8):
    t0 = time.time()
    results = []
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        futs = [ex.submit(_eval_worker, c) for c in cands]
        for idx, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            results.append(r)
            if idx % 8 == 0 or idx == len(cands):
                m = r["metrics"]
                print(f"  [{wave_name}] {idx}/{len(cands)} done ({time.time()-t0:.0f}s) | last={r['name']} J={m.get('score',1e9):.1f}", flush=True)
    results.sort(key=lambda x: x["metrics"]["score"])
    dt = time.time() - t0
    print(f"\n=== {wave_name}: {len(results)} evaluated in {dt:.1f}s (TOP 8) ===", flush=True)
    for r in results[:8]:
        m = r["metrics"]
        if not r["ok"]:
            print(f"  {r['name']:28s} | FAILED: {r.get('error')}", flush=True)
            continue
        print(
            f"  {r['name']:28s} | J={m['score']:6.2f} | LF_ph_max={m['lf_ph_max']:4.1f}° | HF_dB_max={m['hf_db_max']:4.2f} | "
            f"midHF_ph={m['midhf_ph_max']:4.1f}° | noise={m['noise']:4.2f} | r={m['corr']:.3f} | "
            f"ph={m['ph_errs']} | dB={m['db_errs']}",
            flush=True,
        )
    return results


def main():
    rng = np.random.default_rng(42)
    n_workers = min(16, max(8, (os.cpu_count() or 8) - 2))
    print(f"[INFO] Starting Roll Optimization on {n_workers} workers (CPU count={os.cpu_count()})", flush=True)
    all_waves = {}

    w1_cands = build_wave1_candidates()
    w1_res = run_wave("WAVE 1 (32 Broad Exploration)", w1_cands, max_workers=n_workers)
    all_waves["wave1"] = w1_res[:15]

    valid_w1 = [r for r in w1_res if r["ok"]]
    w2_cands = build_wave_from_seeds(valid_w1[:6], 2, rng, n_cands=24)
    w2_res = run_wave("WAVE 2 (24 Focused Refinement)", w2_cands, max_workers=n_workers)
    all_waves["wave2"] = w2_res[:15]

    merged = sorted([r for r in (w1_res + w2_res) if r["ok"]], key=lambda x: x["metrics"]["score"])
    w3_cands = build_wave_from_seeds(merged[:5], 3, rng, n_cands=16)
    w3_res = run_wave("WAVE 3 (16 Local Polish)", w3_cands, max_workers=n_workers)
    all_waves["wave3"] = w3_res[:15]

    best_all = sorted([r for r in (w1_res + w2_res + w3_res) if r["ok"]], key=lambda x: x["metrics"]["score"])
    top3 = best_all[:3]
    print("\n=== PARALLEL CROSS-AXIS CHECK OF TOP 3 ON YawMA ===", flush=True)
    with ProcessPoolExecutor(max_workers=3) as ex:
        verified = list(ex.map(_yaw_check_worker, top3))
    for v in verified:
        ys = v["yaw_summary"]
        print(
            f"  {v['name']:28s} | LatEC J={v['lat_metrics']['score']:.2f} | "
            f"YawMA noise={ys['hf_noise_rms']:.2f}, corr={ys['corr_vel']:.3f}, maxdB={ys['max_db']:.2f}, maxPh={ys['max_ph']:.1f}°",
            flush=True,
        )

    out_path = os.path.join(_ROOT, "logs/roll_yaw_baseline/roll_targeted_opt_results.json")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({"top_verified": verified, "waves": all_waves}, f, indent=2)
    print(f"[OK] Saved optimization summary to {out_path}", flush=True)


if __name__ == "__main__":
    main()
