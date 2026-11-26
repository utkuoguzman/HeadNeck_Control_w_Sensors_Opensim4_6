"""Batch 5 (8-worker Local Full-Model Unified Upright Push to < 5,000, ~8 min total!).
Uses the verified gravity-stable unified Pitch baseline (K_ton=3.8, Kp_prop_pitch=0.50, Ki_ton=1.8)
so LatEC and YawMA pitch drift stays < 0.3 deg (wiping out +23.6k / +9.9k crosstalk) with zero Yaw chatter!"""
import json, os, sys, time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

sys.path.insert(0, os.getcwd())
from sensitivity_tests.bode_xray import print_xray, run_bode_probed
from sensitivity_tests.run_sensitivity import plot_torque_budget

OUT_DIR = os.path.join("sensitivity_tests", "results")
os.makedirs(OUT_DIR, exist_ok=True)


def _run(item):
    label, mode, ov = item
    res = run_bode_probed(mode, ov, exact_15s=False)
    return label, res


if __name__ == "__main__":
    cases = [
        # --- YAWMA (2 shots): Upright Pitch (xt~0!) + Y_C phase anchor + damped 4.05Hz peak -> aiming < 4,000! ---
        ("Y5_1_YC_damp0.06",   "YawMA", {"K_sc_yaw": 0.36, "z_sc_yaw": 0.012, "p_sc_yaw": 6.0, "K_ton_yaw": 1.8, "p_ton_yaw": 12.0, "Kp_prop_yaw": 1.5, "Kp_proprioception_yaw": 0.008, "Kp_buckle_yaw": 0.60, "Kd_buckle_yaw": 0.06}),
        ("Y5_2_YC_Ksc0.40_d8", "YawMA", {"K_sc_yaw": 0.40, "z_sc_yaw": 0.010, "p_sc_yaw": 7.0, "K_ton_yaw": 2.0, "p_ton_yaw": 14.0, "Kp_prop_yaw": 1.7, "Kp_proprioception_yaw": 0.009, "Kp_buckle_yaw": 0.65, "Kd_buckle_yaw": 0.08}),
        # --- LATEC (3 shots): Upright Pitch (xt~0!) + clean Yaw (zero co-contraction!) + low p_ton_roll -> aiming < 5,000! ---
        ("R5_1_upright_pT2.5", "LatEC", {"K_sc_roll": 0.06, "z_sc_roll": 0.012, "p_sc_roll": 5.0, "K_phas_roll": 0.02, "K_ton_roll": 0.45, "p_ton_roll": 2.5, "Ki_ton_roll": 0.10, "Kp_prop_roll": 0.03, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.10, "Kd_buckle_roll": 0.025}),
        ("R5_2_upright_soft",  "LatEC", {"K_sc_roll": 0.03, "z_sc_roll": 0.012, "p_sc_roll": 4.0, "K_phas_roll": 0.01, "K_ton_roll": 0.25, "p_ton_roll": 2.0, "Ki_ton_roll": 0.05, "Kp_prop_roll": 0.02, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.06, "Kd_buckle_roll": 0.015}),
        ("R5_3_upright_Kph15", "LatEC", {"K_sc_roll": 0.10, "z_sc_roll": 0.015, "p_sc_roll": 6.0, "K_phas_roll": 0.15, "K_ton_roll": 0.60, "p_ton_roll": 4.0, "Ki_ton_roll": 0.20, "Kp_prop_roll": 0.05, "Kp_proprioception_roll": 0.002, "Kp_buckle_roll": 0.15, "Kd_buckle_roll": 0.030}),
        # --- APEO (3 shots): First-ever upright Pitch + clean Yaw (zero 3.4x co-contraction!) + K_phas sweep! ---
        ("P5_1_up3.6_Kph0.05", "APEO",  {"K_sc": 0.15, "z_sc": 0.010, "p_sc": 20.0, "K_phas": 0.05, "z_phas": 0.010, "p_phas": 4.0, "K_ton": 3.6, "p_ton": 15.0, "Ki_ton": 1.8, "Kp_prop_pitch": 0.45, "Kp_proprioception": 0.004, "Kp_buckle": 0.90, "Kd_buckle": 0.08, "vcr_upper_share_pitch": 0.44}),
        ("P5_2_up3.8_Kph1.5",  "APEO",  {"K_sc": 0.40, "z_sc": 0.012, "p_sc": 30.0, "K_phas": 1.50, "z_phas": 0.015, "p_phas": 3.5, "K_ton": 3.8, "p_ton": 15.0, "Ki_ton": 1.8, "Kp_prop_pitch": 0.45, "Kp_proprioception": 0.005, "Kp_buckle": 0.90, "Kd_buckle": 0.12, "vcr_upper_share_pitch": 0.44}),
        ("P5_3_up4.2_Kph4.5",  "APEO",  {"K_sc": 1.20, "z_sc": 0.015, "p_sc": 45.0, "K_phas": 4.50, "z_phas": 0.020, "p_phas": 3.5, "K_ton": 4.2, "p_ton": 15.0, "Ki_ton": 2.2, "Kp_prop_pitch": 0.55, "Kp_proprioception": 0.008, "Kp_buckle": 0.95, "Kd_buckle": 0.16, "vcr_upper_share_pitch": 0.50}),
    ]

    t0 = time.time()
    all_res = {}
    with ProcessPoolExecutor(max_workers=8) as pool:
        futs = {pool.submit(_run, c): c[0] for c in cases}
        for fut in as_completed(futs):
            label, res = fut.result()
            all_res[label] = res
            print_xray(label, res)
            sys.stdout.flush()

    for ax in ("APEO", "LatEC", "YawMA"):
        cands = [(k, v) for k, v in all_res.items() if v.get("mode") == ax and not v.get("aborted")]
        if cands:
            best_k, best_v = min(cands, key=lambda kv: kv[1]["total_score"])
            plot_torque_budget(ax, best_v, os.path.join(OUT_DIR, f"torque_budget_{ax}_batch5.png"))
    with open(os.path.join(OUT_DIR, "batch5_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 5 done in {time.time()-t0:.1f}s")
