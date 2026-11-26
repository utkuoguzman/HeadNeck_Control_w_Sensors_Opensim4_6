"""8-worker Full-Model Probed Sensitivity & <5000 Push across YawMA, LatEC, and APEO.
Uses sensitivity_tests/bode_xray.py (100% FULL MODEL: all subsystems ON) to expose internal
VCR/CCR/Buckle/QP torque budgets, q1/q2 S-bend splits, and disc bushing stiffness sensitivity."""
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
    label, mode, ov, bk, bc = item
    res = run_bode_probed(mode, ov, exact_15s=False, bushing_k_scale=bk, bushing_c_scale=bc)
    return label, res


if __name__ == "__main__":
    p1_base = {
        "srs_k_srs": 0.50, "srs_thix_min": 0.12,
        "K_sc": 0.08, "z_sc": 0.010, "p_sc": 25.0,
        "K_phas": 0.03, "z_phas": 0.010, "p_phas": 4.0,
        "K_ton": 0.25, "p_ton": 3.0, "Ki_ton": 0.05,
        "Kp_prop_pitch": 0.05, "Kp_proprioception": 0.002,
        "Kp_buckle": 0.80, "Kd_buckle": 0.05, "vcr_upper_share_pitch": 0.44,
    }
    r1_base = {
        "srs_k_srs": 0.50, "srs_thix_min": 0.12,
        "K_sc_roll": 0.06, "z_sc_roll": 0.012, "p_sc_roll": 6.0,
        "K_phas_roll": 0.02, "z_phas_roll": 0.010, "p_phas_roll": 4.0,
        "K_ton_roll": 0.25, "p_ton_roll": 2.5, "Ki_ton_roll": 0.02,
        "Kp_prop_roll": 0.03, "Kp_proprioception_roll": 0.002,
        "Kp_buckle_roll": 0.15, "Kd_buckle_roll": 0.03,
    }
    cases = [
        # --- YAWMA: Push < 3,000 using exact physics (low Ksc / low psc + Kton/Kprop balance for 1.5-2.9Hz peak!) ---
        ("Y_A_lowKsc_Kpr2.5", "YawMA", {"srs_k_srs": 0.5, "K_sc_yaw": 0.22, "z_sc_yaw": 0.015, "p_sc_yaw": 3.0, "K_ton_yaw": 2.8, "p_ton_yaw": 15.0, "Kp_prop_yaw": 2.5, "Kp_proprioception_yaw": 0.012, "Kp_buckle_yaw": 0.80, "Kd_buckle_yaw": 0.04}, 1.0, 1.0),
        ("Y_B_lowKsc_Kpr4.5", "YawMA", {"srs_k_srs": 0.5, "K_sc_yaw": 0.15, "z_sc_yaw": 0.018, "p_sc_yaw": 2.0, "K_ton_yaw": 4.2, "p_ton_yaw": 18.0, "Kp_prop_yaw": 4.5, "Kp_proprioception_yaw": 0.018, "Kp_buckle_yaw": 1.00, "Kd_buckle_yaw": 0.05}, 1.0, 1.0),
        ("Y_C_medKsc_Kpr1.5", "YawMA", {"srs_k_srs": 0.5, "K_sc_yaw": 0.30, "z_sc_yaw": 0.012, "p_sc_yaw": 6.0, "K_ton_yaw": 1.8, "p_ton_yaw": 12.0, "Kp_prop_yaw": 1.5, "Kp_proprioception_yaw": 0.008, "Kp_buckle_yaw": 0.60, "Kd_buckle_yaw": 0.03}, 1.0, 1.0),
        # --- LATEC: Full internal probe on R1 + ultra-soft reflex vs softer disc bushings (0.35x) ---
        ("R_A_R1_probed",     "LatEC", dict(r1_base), 1.0, 1.0),
        ("R_B_ultrasoft",     "LatEC", {**r1_base, "srs_k_srs": 0.10, "K_sc_roll": 0.015, "K_phas_roll": 0.005, "K_ton_roll": 0.15, "p_ton_roll": 1.2, "Kp_prop_roll": 0.01, "Kp_proprioception_roll": 0.0005, "Kp_buckle_roll": 0.04, "Kd_buckle_roll": 0.01}, 1.0, 1.0),
        ("R_C_softDisc0.35",  "LatEC", dict(r1_base), 0.35, 0.35),
        # --- APEO: Full internal probe on P1 (13,877!) + softer disc bushings (0.35x) ---
        ("P_A_P1_probed",     "APEO",  dict(p1_base), 1.0, 1.0),
        ("P_B_softDisc0.35",  "APEO",  dict(p1_base), 0.35, 0.35),
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

    for key, ax in (("P_A_P1_probed", "APEO"), ("R_A_R1_probed", "LatEC"), ("Y_A_lowKsc_Kpr2.5", "YawMA")):
        if key in all_res and not all_res[key].get("aborted"):
            plot_torque_budget(ax, all_res[key], os.path.join(OUT_DIR, f"torque_budget_{ax}_batch1.png"))
    with open(os.path.join(OUT_DIR, "batch1_probed_results.json"), "w", encoding="utf-8") as f:
        json.dump(all_res, f, indent=2)
    print(f"\nBatch 1 done in {time.time()-t0:.1f}s")
