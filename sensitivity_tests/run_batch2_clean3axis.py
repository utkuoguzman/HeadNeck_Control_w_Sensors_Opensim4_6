"""Batch 2 (8-worker Local Full-Model Probed Push toward < 5,000 on ALL 3 AXES)
NOW running on the CLEAN 3-axis baseline (zero stale Yaw/Pitch co-contraction chatter!)."""
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
        # --- YAWMA: Targeting < 3,500 (interpolating Y_C phase + stronger 4.05Hz damping!) ---
        ("Y2_1_Ksc0.42_psc12", "YawMA", {"srs_k_srs": 0.30, "K_sc_yaw": 0.42, "z_sc_yaw": 0.008, "p_sc_yaw": 12.0, "K_ton_yaw": 2.4, "p_ton_yaw": 14.0, "Kp_prop_yaw": 1.6, "Kp_proprioception_yaw": 0.008, "Kp_buckle_yaw": 0.65, "Kd_buckle_yaw": 0.04}),
        ("Y2_2_Ksc0.48_psc15", "YawMA", {"srs_k_srs": 0.30, "K_sc_yaw": 0.48, "z_sc_yaw": 0.004, "p_sc_yaw": 15.0, "K_ton_yaw": 2.8, "p_ton_yaw": 16.0, "Kp_prop_yaw": 1.9, "Kp_proprioception_yaw": 0.010, "Kp_buckle_yaw": 0.70, "Kd_buckle_yaw": 0.045}),
        # --- LATEC: Clean 3-axis (mean_act=0.033!) + Pitch Ki_ton=1.0 to eliminate pitch sag cross-talk! ---
        ("R2_1_clean_pTon1.5", "LatEC", {"srs_k_srs": 0.25, "K_ton": 0.50, "Ki_ton": 1.0, "K_sc_roll": 0.04, "z_sc_roll": 0.012, "p_sc_roll": 6.0, "K_phas_roll": 0.01, "K_ton_roll": 0.15, "p_ton_roll": 1.5, "Ki_ton_roll": 0.03, "Kp_prop_roll": 0.02, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.08, "Kd_buckle_roll": 0.02}),
        ("R2_2_clean_pTon1.0", "LatEC", {"srs_k_srs": 0.15, "K_ton": 0.50, "Ki_ton": 1.0, "K_sc_roll": 0.02, "z_sc_roll": 0.010, "p_sc_roll": 4.0, "K_phas_roll": 0.005,"K_ton_roll": 0.25, "p_ton_roll": 1.0, "Ki_ton_roll": 0.05, "Kp_prop_roll": 0.01, "Kp_proprioception_roll": 0.0005,"Kp_buckle_roll": 0.04, "Kd_buckle_roll": 0.01}),
        ("R2_3_clean_Kton0.35","LatEC", {"srs_k_srs": 0.20, "K_ton": 0.60, "Ki_ton": 1.2, "K_sc_roll": 0.08, "z_sc_roll": 0.020, "p_sc_roll": 3.0, "K_phas_roll": 0.01, "K_ton_roll": 0.35, "p_ton_roll": 1.2, "Ki_ton_roll": 0.08, "Kp_prop_roll": 0.03, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.06, "Kd_buckle_roll": 0.015}),
        # --- APEO: Clean 3-axis (zero xt!) + lower Kp_buckle/p_sc so 7.93Hz q1(-234 deg) isn't fought! ---
        ("P2_1_bkl0.25_psc8",  "APEO",  {"srs_k_srs": 0.25, "K_sc": 0.06, "z_sc": 0.008, "p_sc": 8.0,  "K_phas": 0.02, "p_phas": 3.0, "K_ton": 0.12, "p_ton": 2.0, "Ki_ton": 0.15, "Kp_prop_pitch": 0.03, "Kp_proprioception": 0.001, "Kp_buckle": 0.25, "Kd_buckle": 0.02, "vcr_upper_share_pitch": 0.44}),
        ("P2_2_bkl0.12_psc6",  "APEO",  {"srs_k_srs": 0.15, "K_sc": 0.04, "z_sc": 0.006, "p_sc": 6.0,  "K_phas": 0.01, "p_phas": 3.0, "K_ton": 0.08, "p_ton": 1.5, "Ki_ton": 0.20, "Kp_prop_pitch": 0.02, "Kp_proprioception": 0.001, "Kp_buckle": 0.12, "Kd_buckle": 0.015,"vcr_upper_share_pitch": 0.44}),
        ("P2_3_bkl0.45_ush25", "APEO",  {"srs_k_srs": 0.20, "K_sc": 0.06, "z_sc": 0.008, "p_sc": 10.0, "K_phas": 0.01, "p_phas": 3.0, "K_ton": 0.10, "p_ton": 2.0, "Ki_ton": 0.10, "Kp_prop_pitch": 0.02, "Kp_proprioception": 0.001, "Kp_buckle": 0.45, "Kd_buckle": 0.02, "vcr_upper_share_pitch": 0.25}),
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
            plot_torque_budget(ax, best_v, os.path.join(OUT_DIR, f"torque_budget_{ax}_batch2.png"))
    with open(os.path.join(OUT_DIR, "batch2_clean3axis_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 2 done in {time.time()-t0:.1f}s")
