"""Batch 7 (8-worker Local Full-Model < 5,000 Push, ~9 min total!).
- YawMA (2 shots from Y6_2 = 5,545, rv=+0.9148!): step Kp_prop_yaw 0.45->0.15-0.25 & Kd_bkl 0.012->0.005 -> SMASH < 3,500!
- APEO  (3 shots from P5_1 = 7,252, G=2,237, xt=0.0!): keep upright K_ton=4.0-4.2, slash 7.93Hz ccrG=28Nm clamp (Kp_prop_pitch 0.45->0.06-0.12, z_ccr 0.02->0.003, p_ton 15->5-8) -> SMASH < 5,000!
- LatEC (3 shots on EXACT 15.8s window = 1.31 cycles of 0.121Hz!): test Roll gravity-hold K_ton_roll=1.2-3.8 with low Ki_ton_roll (0.02-0.15) so 0.12Hz doesn't get integrator +90 deg flip!"""
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
    res = run_bode_probed(mode, ov, exact_15s=(mode == "LatEC"))
    return label, res


if __name__ == "__main__":
    cases = [
        # --- YAWMA (2 shots from Y6_2 = 5,545!): push straight into < 3,500! ---
        ("Y7_1_Kpr0.22_Kton0.65", "YawMA", {"K_sc_yaw": 0.40, "z_sc_yaw": 0.003, "p_sc_yaw": 3.5, "K_ton_yaw": 0.65, "p_ton_yaw": 8.0, "Kp_prop_yaw": 0.22, "Kp_proprioception_yaw": 0.002, "Kp_buckle_yaw": 0.20, "Kd_buckle_yaw": 0.006, "z_ccr_yaw": 0.008}),
        ("Y7_2_Kpr0.12_Kton0.40", "YawMA", {"K_sc_yaw": 0.44, "z_sc_yaw": 0.002, "p_sc_yaw": 3.0, "K_ton_yaw": 0.40, "p_ton_yaw": 6.0, "Kp_prop_yaw": 0.12, "Kp_proprioception_yaw": 0.001, "Kp_buckle_yaw": 0.12, "Kd_buckle_yaw": 0.004, "z_ccr_yaw": 0.005}),
        # --- APEO (3 shots from P5_1 = 7,252!): keep K_ton=4.0 upright, cut ccrG@7.93Hz (28Nm->4Nm) & roll off p_ton=5-8! ---
        ("P7_1_Kton4.0_Kp0.10_pT8", "APEO", {"K_sc": 0.15, "z_sc": 0.008, "p_sc": 18.0, "K_phas": 0.04, "p_phas": 4.0, "K_ton": 4.0, "p_ton": 8.0,  "Ki_ton": 1.8, "Kp_prop_pitch": 0.10, "z_ccr": 0.004, "Kp_proprioception": 0.001, "Kp_buckle": 0.70, "Kd_buckle": 0.04, "vcr_upper_share_pitch": 0.44}),
        ("P7_2_Kton4.2_Kp0.06_pT5", "APEO", {"K_sc": 0.12, "z_sc": 0.006, "p_sc": 15.0, "K_phas": 0.03, "p_phas": 4.0, "K_ton": 4.2, "p_ton": 5.0,  "Ki_ton": 1.6, "Kp_prop_pitch": 0.06, "z_ccr": 0.003, "Kp_proprioception": 0.001, "Kp_buckle": 0.65, "Kd_buckle": 0.03, "vcr_upper_share_pitch": 0.44}),
        ("P7_3_Kton3.8_Kp0.08_pT12","APEO", {"K_sc": 0.22, "z_sc": 0.010, "p_sc": 25.0, "K_phas": 0.05, "p_phas": 4.0, "K_ton": 3.8, "p_ton": 12.0, "Ki_ton": 1.8, "Kp_prop_pitch": 0.08, "z_ccr": 0.003, "Kp_proprioception": 0.001, "Kp_buckle": 0.85, "Kd_buckle": 0.06, "vcr_upper_share_pitch": 0.40}),
        # --- LATEC (3 shots on EXACT 15.8s window!): upright Roll K_ton_roll with low Ki_ton_roll to cure 0.12Hz phase! ---
        ("R7_1_ex15_Kt2.5_Ki0.05",  "LatEC", {"K_sc_roll": 0.08, "z_sc_roll": 0.012, "p_sc_roll": 6.0, "K_phas_roll": 0.02, "K_ton_roll": 2.5, "p_ton_roll": 4.0, "Ki_ton_roll": 0.05, "Kp_prop_roll": 0.25, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.15, "Kd_buckle_roll": 0.020}),
        ("R7_2_ex15_Kt3.8_Ki0.15",  "LatEC", {"K_sc_roll": 0.10, "z_sc_roll": 0.012, "p_sc_roll": 6.0, "K_phas_roll": 0.05, "K_ton_roll": 3.8, "p_ton_roll": 6.0, "Ki_ton_roll": 0.15, "Kp_prop_roll": 0.35, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.18, "Kd_buckle_roll": 0.020}),
        ("R7_3_ex15_Kt1.2_Kph0.35", "LatEC", {"K_sc_roll": 0.06, "z_sc_roll": 0.012, "p_sc_roll": 5.0, "K_phas_roll": 0.35, "z_phas_roll": 0.020, "p_phas_roll": 3.0, "K_ton_roll": 1.2, "p_ton_roll": 2.0, "Ki_ton_roll": 0.02, "Kp_prop_roll": 0.15, "Kp_proprioception_roll": 0.001, "Kp_buckle_roll": 0.10, "Kd_buckle_roll": 0.015}),
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
            plot_torque_budget(ax, best_v, os.path.join(OUT_DIR, f"torque_budget_{ax}_batch7.png"))
    with open(os.path.join(OUT_DIR, "batch7_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 7 done in {time.time()-t0:.1f}s")
