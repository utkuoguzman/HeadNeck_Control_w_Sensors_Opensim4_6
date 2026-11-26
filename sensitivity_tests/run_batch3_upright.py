"""Batch 3 (8-worker Local Full-Model Probed Push to < 5,000 on ALL 3 AXES, ~8 min total!).
Fixes 0Hz Pitch gravity sag (K_ton=2.2, p_ton=1.5, Ki_ton=2.2 -> zero xt!) while keeping 0.32-7.93 Hz free!"""
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
        # --- YAWMA: Upright Pitch (xt~0!) + balanced Ksc/Kpr targeting < 4,000 ---
        ("Y3_1_Ksc0.34_psc8",  "YawMA", {"K_sc_yaw": 0.34, "z_sc_yaw": 0.008, "p_sc_yaw": 8.0,  "K_ton_yaw": 2.4, "p_ton_yaw": 14.0, "Kp_prop_yaw": 1.8, "Kp_proprioception_yaw": 0.008, "Kp_buckle_yaw": 0.65, "Kd_buckle_yaw": 0.04}),
        ("Y3_2_Ksc0.28_psc10", "YawMA", {"K_sc_yaw": 0.28, "z_sc_yaw": 0.005, "p_sc_yaw": 10.0, "K_ton_yaw": 2.6, "p_ton_yaw": 16.0, "Kp_prop_yaw": 2.0, "Kp_proprioception_yaw": 0.010, "Kp_buckle_yaw": 0.70, "Kd_buckle_yaw": 0.05}),
        # --- LATEC: Upright Pitch (xt~0!) + low-pass Kton/Ki_ton for 0.12 Hz while preserving 0.56-3.99 Hz (~5,300)! ---
        ("R3_1_Kton0.5_Ki0.4", "LatEC", {"K_sc_roll": 0.06, "z_sc_roll": 0.015, "p_sc_roll": 4.0, "K_phas_roll": 0.01, "K_ton_roll": 0.50, "p_ton_roll": 1.2, "Ki_ton_roll": 0.40, "Kp_prop_roll": 0.03, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.10, "Kd_buckle_roll": 0.03}),
        ("R3_2_Kton0.8_Ki0.8", "LatEC", {"K_sc_roll": 0.08, "z_sc_roll": 0.020, "p_sc_roll": 3.0, "K_phas_roll": 0.01, "K_ton_roll": 0.80, "p_ton_roll": 1.0, "Ki_ton_roll": 0.80, "Kp_prop_roll": 0.04, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.12, "Kd_buckle_roll": 0.04}),
        ("R3_3_Kton0.35_Ki0.2","LatEC", {"K_sc_roll": 0.05, "z_sc_roll": 0.015, "p_sc_roll": 5.0, "K_phas_roll": 0.01, "K_ton_roll": 0.35, "p_ton_roll": 1.5, "Ki_ton_roll": 0.20, "Kp_prop_roll": 0.02, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.08, "Kd_buckle_roll": 0.03}),
        # --- APEO: Low-pass K_ton=2.2/p_ton=1.0..1.8 (zero sag + soft 0.32Hz!) + Kd_buckle=0.06 (clean hf!) ---
        ("P3_1_lpTon1.0_bk0.7","APEO",  {"K_sc": 0.08, "z_sc": 0.010, "p_sc": 18.0, "K_phas": 0.02, "z_phas": 0.010, "p_phas": 4.0,  "K_ton": 2.2, "p_ton": 1.0, "Ki_ton": 1.8, "Kp_prop_pitch": 0.08, "Kp_proprioception": 0.002, "Kp_buckle": 0.70, "Kd_buckle": 0.06, "vcr_upper_share_pitch": 0.44}),
        ("P3_2_lpTon1.5_bk0.9","APEO",  {"K_sc": 0.10, "z_sc": 0.012, "p_sc": 25.0, "K_phas": 0.05, "z_phas": 0.015, "p_phas": 6.0,  "K_ton": 2.0, "p_ton": 1.5, "Ki_ton": 1.5, "Kp_prop_pitch": 0.10, "Kp_proprioception": 0.002, "Kp_buckle": 0.90, "Kd_buckle": 0.07, "vcr_upper_share_pitch": 0.44}),
        ("P3_3_Kph0.25_bk0.6", "APEO",  {"K_sc": 0.06, "z_sc": 0.010, "p_sc": 12.0, "K_phas": 0.25, "z_phas": 0.025, "p_phas": 10.0, "K_ton": 2.2, "p_ton": 1.2, "Ki_ton": 1.8, "Kp_prop_pitch": 0.06, "Kp_proprioception": 0.002, "Kp_buckle": 0.60, "Kd_buckle": 0.06, "vcr_upper_share_pitch": 0.44}),
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
            plot_torque_budget(ax, best_v, os.path.join(OUT_DIR, f"torque_budget_{ax}_batch3.png"))
    with open(os.path.join(OUT_DIR, "batch3_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 3 done in {time.time()-t0:.1f}s")
