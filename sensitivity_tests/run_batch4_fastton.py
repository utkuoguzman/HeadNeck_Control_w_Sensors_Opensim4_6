"""Batch 4 (8-worker Local Full-Model Probed Push to < 5,000, ~7.5 min total!).
Fixes p_ton=15.0 (Pitch) & p_ton_roll=12.0 (Roll) so tonic feedback is zero-lag and non-oscillatory!"""
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
        # --- YAWMA: Fast upright Pitch (xt~0!) + Y_C-style Kpr=1.0..1.35 & Ksc=0.38..0.44 targeting < 3,500! ---
        ("Y4_1_Ksc0.38_Kpr1.2", "YawMA", {"K_sc_yaw": 0.38, "z_sc_yaw": 0.010, "p_sc_yaw": 7.0, "K_ton_yaw": 1.8, "p_ton_yaw": 14.0, "Kp_prop_yaw": 1.2, "Kp_proprioception_yaw": 0.007, "Kp_buckle_yaw": 0.60, "Kd_buckle_yaw": 0.04}),
        ("Y4_2_Ksc0.44_Kpr0.9", "YawMA", {"K_sc_yaw": 0.44, "z_sc_yaw": 0.006, "p_sc_yaw": 6.0, "K_ton_yaw": 1.5, "p_ton_yaw": 14.0, "Kp_prop_yaw": 0.9, "Kp_proprioception_yaw": 0.006, "Kp_buckle_yaw": 0.50, "Kd_buckle_yaw": 0.035}),
        # --- LATEC: Fast p_ton_roll=12.0 + fast Pitch p_ton=15.0 (zero xt!) targeting < 5,000! ---
        ("R4_1_fastTon0.45",    "LatEC", {"K_sc_roll": 0.08, "z_sc_roll": 0.015, "p_sc_roll": 5.0, "K_phas_roll": 0.01, "K_ton_roll": 0.45, "p_ton_roll": 12.0, "Ki_ton_roll": 0.15, "Kp_prop_roll": 0.03, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.10, "Kd_buckle_roll": 0.03}),
        ("R4_2_fastTon0.25",    "LatEC", {"K_sc_roll": 0.05, "z_sc_roll": 0.015, "p_sc_roll": 5.0, "K_phas_roll": 0.01, "K_ton_roll": 0.25, "p_ton_roll": 12.0, "Ki_ton_roll": 0.05, "Kp_prop_roll": 0.02, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.08, "Kd_buckle_roll": 0.025}),
        ("R4_3_fastTon0.75",    "LatEC", {"K_sc_roll": 0.12, "z_sc_roll": 0.015, "p_sc_roll": 4.0, "K_phas_roll": 0.01, "K_ton_roll": 0.75, "p_ton_roll": 15.0, "Ki_ton_roll": 0.30, "Kp_prop_roll": 0.04, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.12, "Kd_buckle_roll": 0.03}),
        # --- APEO: Fast p_ton=15.0 (zero oscillation!) + clean Roll/Yaw (zero xt!) ---
        ("P4_1_Kton1.2_bk0.7",  "APEO",  {"K_sc": 0.08, "z_sc": 0.010, "p_sc": 20.0, "K_phas": 0.02, "z_phas": 0.010, "p_phas": 4.0, "K_ton": 1.2, "p_ton": 15.0, "Ki_ton": 0.4, "Kp_prop_pitch": 0.06, "Kp_proprioception": 0.002, "Kp_buckle": 0.70, "Kd_buckle": 0.05}),
        ("P4_2_Kton2.0_bk0.8",  "APEO",  {"K_sc": 0.10, "z_sc": 0.010, "p_sc": 25.0, "K_phas": 0.03, "z_phas": 0.010, "p_phas": 4.0, "K_ton": 2.0, "p_ton": 15.0, "Ki_ton": 1.0, "Kp_prop_pitch": 0.10, "Kp_proprioception": 0.002, "Kp_buckle": 0.80, "Kd_buckle": 0.06}),
        ("P4_3_Kton0.8_bk0.5",  "APEO",  {"K_sc": 0.06, "z_sc": 0.008, "p_sc": 15.0, "K_phas": 0.02, "z_phas": 0.010, "p_phas": 4.0, "K_ton": 0.8, "p_ton": 15.0, "Ki_ton": 0.2, "Kp_prop_pitch": 0.04, "Kp_proprioception": 0.002, "Kp_buckle": 0.50, "Kd_buckle": 0.05}),
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
            plot_torque_budget(ax, best_v, os.path.join(OUT_DIR, f"torque_budget_{ax}_batch4.png"))
    with open(os.path.join(OUT_DIR, "batch4_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 4 done in {time.time()-t0:.1f}s")
