import os
import sys
import time
import math
import cmath
import json
import numpy as np
from scipy.stats import qmc
import multiprocessing as mp

sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "scratch"))

from cervical_chadwick_blana_simulator import fast_eval_candidate

TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (5.583, 503.30, -201.38),
    (2.446, 423.34, -161.34),
    (7.927, 572.69, -247.62)
]

PARAM_NAMES = [
    "r_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",
    "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat"
]

lower_bounds = np.array([
    0.05,    # r_ton
    2.0,     # p_ton
    0.05,    # K_sc
    0.001,   # r_lead_sc
    15.0,    # p_sc
    0.20,    # K_phas
    8.0,     # p_phas
    0.15,    # kp
    0.001,   # r_lead_ccr
    5.0,     # p_ccr
    0.10,    # k_dyn
    0.01     # k_stat
])

upper_bounds = np.array([
    2.50,    # r_ton
    120.0,   # p_ton
    15.0,    # K_sc
    0.60,    # r_lead_sc
    150.0,   # p_sc
    15.0,    # K_phas
    80.0,    # p_phas
    5.00,    # kp
    0.65,    # r_lead_ccr
    220.0,   # p_ccr
    1.50,    # k_dyn
    1.50     # k_stat
])

def fast_rom_score(cand):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = cand
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr

    J_eff = 0.1633
    B_pass = 1.46
    K_pass = 5.20
    M_drive = 0.778
    tau_act = 0.012
    k_geom = 1.0

    delay_vcr = 0.010
    delay_ccr = 0.015

    total_score = 0.0
    for freq, t_gain, t_phase in TARGETS:
        omega = 2.0 * math.pi * freq
        s = 1j * omega

        H_act = 1.0 / (tau_act * s + 1.0)
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
        H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)

        Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
        Z_plant = - (omega**2) * J_eff + s * B_pass + K_pass
        T_drive = - (omega**2) * (M_drive - K_phas * H_phas * H_act)

        Theta_over_X = (T_drive / (Z_plant + Z_reflex)) * k_geom
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0

        sim_gain_safe = max(1e-6, gain_deg_m)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0

        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0

        total_score += (gain_loss + phase_loss)
    return total_score

def run_prescreen():
    print("=" * 75)
    print("64K SOBOL PRE-SCREENING VIA 8DOF CHADWICK-BLANA SURROGATE PIPELINE")
    print("=" * 75)

    n_samples = 65536
    print(f"Generating {n_samples} scrambled Sobol quasi-random samples across 12D space...")
    sampler = qmc.Sobol(d=12, scramble=True, seed=2026)
    raw_samples = sampler.random(n_samples)
    samples = qmc.scale(raw_samples, lower_bounds, upper_bounds)

    print("Phase 1: Evaluating all 65,536 samples via calibrated Analytical ROM...")
    t0 = time.perf_counter()
    rom_scores = np.array([fast_rom_score(s) for s in samples])
    t_rom = time.perf_counter() - t0
    print(f"Phase 1 completed in {t_rom:.2f} s ({n_samples/t_rom:.0f} evals/s)!")

    # Select top 1,024 candidates for 8DOF Chadwick-Blana forward dynamics evaluation
    top_k = 1024
    top_indices = np.argsort(rom_scores)[:top_k]
    top_candidates = [samples[idx].tolist() for idx in top_indices]
    print(f"Phase 2: Evaluating top {top_k} candidates on 8DOF Chadwick-Blana Musculoskeletal ODE across 14 cores...")

    t1 = time.perf_counter()
    with mp.Pool(14) as pool:
        cb_scores = pool.map(fast_eval_candidate, top_candidates, chunksize=16)
    t_cb = time.perf_counter() - t1
    print(f"Phase 2 completed in {t_cb:.2f} s ({top_k/t_cb:.1f} evals/s)!")

    cb_scores = np.array(cb_scores)
    best_cb_idx = np.argmin(cb_scores)
    best_candidate = top_candidates[best_cb_idx]
    best_score = cb_scores[best_cb_idx]

    print("=" * 75)
    print(f"CHOSEN GLOBAL CENTROID x0 FROM 64K 8DOF SCREEN:")
    print(f"8DOF Surrogate Predicted Score: {best_score:.2f} pts")
    print("-" * 75)
    print(f"{'Parameter':<15} | {'Value':<12} | {'Active Domain':<18} | {'Proposed Sigma0':<15}")
    print("-" * 75)

    sigma0 = []
    for i, name in enumerate(PARAM_NAMES):
        val = best_candidate[i]
        L = lower_bounds[i]
        U = upper_bounds[i]
        R = U - L
        # Healthy sigma ~ 8-12% of parameter range
        sig = 0.08 * R
        # Format-specific adjustments for clean bounds
        if name in ["p_ton", "p_sc", "p_ccr"]:
            sig = max(6.0, sig)
        elif name in ["r_ton", "r_lead_sc", "r_lead_ccr"]:
            sig = max(0.02, sig)
        sigma0.append(sig)
        print(f"{name:<15} | {val:<12.4f} | [{L:7.3f}, {U:7.1f}] | {sig:<15.4f}")

    print("=" * 75)

    result_data = {
        "x0": best_candidate,
        "sigma0": sigma0,
        "lower_bounds": lower_bounds.tolist(),
        "upper_bounds": upper_bounds.tolist(),
        "predicted_score": best_score,
        "n_samples": n_samples,
        "top_k_evaluated_on_8dof": top_k
    }

    with open("data/prescreen_64k_best_candidate.json", "w", encoding="utf-8") as f:
        json.dump(result_data, f, indent=2)
    print("Saved result to data/prescreen_64k_best_candidate.json")
    return result_data

if __name__ == '__main__':
    run_prescreen()
