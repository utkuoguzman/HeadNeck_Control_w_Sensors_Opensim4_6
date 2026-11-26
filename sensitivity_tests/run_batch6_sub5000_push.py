"""Batch 6 (8-worker Local Full-Model < 5,000 Push, ~8 min total!).
- APEO (3 shots around P5_1 = 7,252!): slash 7.93Hz bklG=107.5Nm clamp (Kd_buckle 0.08->0.015-0.02, Kp_buckle 0.90->0.30-0.45)!
- LatEC (3 shots): apply the EXACT APEO gravity-floor breakthrough to Roll (K_ton_roll 0.45->2.0-3.8, Ki_ton_roll 1.0-1.8) to un-flip 0.12Hz +180->+55 deg!
- YawMA (2 shots): lower Kd_buckle_yaw (0.06->0.015-0.025) & Kp_prop_yaw (1.5->0.55-0.85) to drop resonance onto 2.8Hz (-65 deg / -100 deg @ 4.05Hz)!"""
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
        # --- APEO (3 shots around P5_1 = 7,252, G=2,237, xt=0.0!): free 7.93Hz from 107.5Nm bklG clamp -> target < 4,000! ---
        ("P6_1_lowBkl",         "APEO",  {"K_sc": 0.15, "z_sc": 0.010, "p_sc": 12.0, "K_phas": 0.05, "p_phas": 4.0, "K_ton": 3.6, "p_ton": 15.0, "Ki_ton": 1.8, "Kp_prop_pitch": 0.45, "Kp_proprioception": 0.001, "Kp_buckle": 0.45, "Kd_buckle": 0.020, "vcr_upper_share_pitch": 0.44}),
        ("P6_2_softBkl_ush35",  "APEO",  {"K_sc": 0.10, "z_sc": 0.008, "p_sc": 10.0, "K_phas": 0.03, "p_phas": 4.0, "K_ton": 3.6, "p_ton": 15.0, "Ki_ton": 1.8, "Kp_prop_pitch": 0.45, "Kp_proprioception": 0.001, "Kp_buckle": 0.30, "Kd_buckle": 0.015, "vcr_upper_share_pitch": 0.35}),
        ("P6_3_tightLow_softHi","APEO",  {"K_sc": 0.12, "z_sc": 0.010, "p_sc": 14.0, "K_phas": 0.04, "p_phas": 4.0, "K_ton": 3.2, "p_ton": 15.0, "Ki_ton": 1.2, "Kp_prop_pitch": 0.55, "Kp_proprioception": 0.001, "Kp_buckle": 0.40, "Kd_buckle": 0.020, "vcr_upper_share_pitch": 0.44, "srs_k_srs": 0.25}),
        # --- LATEC (3 shots): raise K_ton_roll/Ki_ton_roll/Kp_prop_roll above Roll gravity floor to un-flip 0.12Hz +180->+55 deg! ---
        ("R6_1_KtonR2.5_Ki1.2", "LatEC", {"K_sc_roll": 0.08, "z_sc_roll": 0.012, "p_sc_roll": 6.0, "K_phas_roll": 0.02, "K_ton_roll": 2.5, "p_ton_roll": 12.0, "Ki_ton_roll": 1.2, "Kp_prop_roll": 0.25, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.15, "Kd_buckle_roll": 0.020}),
        ("R6_2_KtonR3.8_Ki1.8", "LatEC", {"K_sc_roll": 0.12, "z_sc_roll": 0.012, "p_sc_roll": 8.0, "K_phas_roll": 0.02, "K_ton_roll": 3.8, "p_ton_roll": 15.0, "Ki_ton_roll": 1.8, "Kp_prop_roll": 0.35, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.20, "Kd_buckle_roll": 0.020}),
        ("R6_3_KtonR2.0_KpR0.8","LatEC", {"K_sc_roll": 0.06, "z_sc_roll": 0.012, "p_sc_roll": 5.0, "K_phas_roll": 0.01, "K_ton_roll": 2.0, "p_ton_roll": 10.0, "Ki_ton_roll": 0.8, "Kp_prop_roll": 0.80, "Kp_proprioception_roll": 0.002, "Kp_buckle_roll": 0.12, "Kd_buckle_roll": 0.015}),
        # --- YAWMA (2 shots): slash Kd_buckle_yaw & lower Kp_prop_yaw to pull resonance onto 2.8Hz (-65 deg / -100 deg @ 4.05Hz)! ---
        ("Y6_1_lowKd_Kpr0.75",  "YawMA", {"K_sc_yaw": 0.34, "z_sc_yaw": 0.006, "p_sc_yaw": 5.0, "K_ton_yaw": 1.4, "p_ton_yaw": 12.0, "Kp_prop_yaw": 0.75, "Kp_proprioception_yaw": 0.004, "Kp_buckle_yaw": 0.45, "Kd_buckle_yaw": 0.020}),
        ("Y6_2_lowKd_Kpr0.45",  "YawMA", {"K_sc_yaw": 0.38, "z_sc_yaw": 0.004, "p_sc_yaw": 4.0, "K_ton_yaw": 1.0, "p_ton_yaw": 10.0, "Kp_prop_yaw": 0.45, "Kp_proprioception_yaw": 0.003, "Kp_buckle_yaw": 0.35, "Kd_buckle_yaw": 0.012}),
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
            plot_torque_budget(ax, best_v, os.path.join(OUT_DIR, f"torque_budget_{ax}_batch6.png"))
    with open(os.path.join(OUT_DIR, "batch6_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 6 done in {time.time()-t0:.1f}s")
