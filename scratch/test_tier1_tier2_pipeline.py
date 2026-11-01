import os
import sys
import json
import math
import cmath
import time
import numpy as np
from scipy.stats import qmc
from concurrent.futures import ProcessPoolExecutor

# Make sure we can import from scratch and current dir
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "scratch"))

from scratch.test_tier2_accelerated import fast_eval_candidate_with_early_stopping

SURR_DELAY_VCR = 0.010
SURR_DELAY_CCR = 0.015
SURR_OPT_PLANT = (0.0185, 0.45, 12.5, 0.145, 0.012, 1.0)

def surrogate_score(cand):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = cand
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    J_eff, B_pass, K_pass, M_drive, tau_act, k_geom = SURR_OPT_PLANT
    
    total_score = 0.0
    for freq, (t_gain, t_phase) in [(0.324, (86.55, -7.03)), (0.783, (236.93, -96.96)), (1.099, (319.74, -109.23)), (2.446, (423.34, -161.34)), (5.583, (503.30, -201.38)), (7.927, (572.69, -247.62))]:
        omega = 2.0 * math.pi * freq
        s = 1j * omega
        
        H_act = 1.0 / (tau_act * s + 1.0)
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * SURR_DELAY_VCR)
        H_ccr = ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * SURR_DELAY_CCR)
        
        Z_reflex = (K_sc * H_sc + K_ton * H_ton + kp * (1.0 + k_dyn * 0.5) * H_ccr) * H_act * k_geom
        Z_plant = J_eff * (s**2) + B_pass * s + K_pass
        
        T_drive = (omega**2) * (M_drive - 0.015 * K_phas * H_phas * H_act)
        Theta_over_X = T_drive / (Z_plant + Z_reflex)
        
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0
        
        gain_db_diff = 20.0 * math.log10(max(1e-6, gain_deg_m) / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        total_score += (gain_loss + phase_loss)
    return total_score

def main():
    print("="*70)
    print("TESTING HIERARCHICAL MULTI-FIDELITY SURROGATE PRE-SCREENING PIPELINE")
    print("="*70)
    
    lower_bounds = np.array([0.05, 2.0, 0.05, 0.001, 1.0, 0.0, 0.5, 0.15, 0.001, 5.0, 0.10, 0.01])
    upper_bounds = np.array([2.50, 80.0, 30.0, 1.000, 50.0, 50.0, 30.0, 5.00, 1.000, 90.0, 1.50, 1.50])
    
    # 1. Generate 65,536 Sobol + 16,000 Local CMA-ES candidates
    t0 = time.perf_counter()
    sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=42)
    unit_sobol = sobol_sampler.random(65536)
    sobol_samples = qmc.scale(unit_sobol, lower_bounds, upper_bounds).tolist()
    
    with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
        ledger = json.load(f)
    g10_p = ledger["generations"][9]["params"]
    champ_x = np.array([
        g10_p["r_ton"], g10_p["p_ton"], g10_p["K_sc"], g10_p["r_lead_sc"], g10_p["p_sc"],
        g10_p["K_phas"], g10_p["p_phas"], g10_p["kp"], g10_p["r_lead_ccr"], g10_p["p_ccr"],
        g10_p["k_dyn"], g10_p["k_stat"]
    ])
    
    np.random.seed(42)
    jitter_samples = np.random.normal(champ_x, (upper_bounds - lower_bounds) * 0.05, size=(16000, 12))
    jitter_samples = np.clip(jitter_samples, lower_bounds, upper_bounds).tolist()
    
    combined_pool = sobol_samples + jitter_samples
    print(f"Generated pool of {len(combined_pool):,} candidates in {time.perf_counter()-t0:.2f} s")
    
    # 2. Tier 1: Fast Algebraic ROM Spline Surrogate (81,536 candidates)
    t1_start = time.perf_counter()
    t1_scores = [surrogate_score(cand) for cand in combined_pool]
    t1_scores = np.array(t1_scores)
    
    top8k_idx = np.argsort(t1_scores)[:8000]
    top8k_cands = [combined_pool[i] for i in top8k_idx]
    t1_time = time.perf_counter() - t1_start
    print(f"Tier 1 (Algebraic Surrogate): 81,536 evals in {t1_time:.2f} s ({len(combined_pool)/t1_time:,.0f} evals/s)!")
    print(f"Tier 1 Best Predicted Score: {t1_scores[top8k_idx[0]]:.2f} pts | Worst in Top 8k: {t1_scores[top8k_idx[-1]]:.2f} pts")
    
    # 3. Tier 2: Option A Non-Linear 2-DOF Muscle ODE Simulator (Top 8,000 across 14 cores)
    print(f"\nDispatching top {len(top8k_cands):,} candidates to Tier 2 (Non-Linear ODE) across 14 cores...")
    t2_start = time.perf_counter()
    with ProcessPoolExecutor(max_workers=14) as executor:
        t2_scores = list(executor.map(fast_eval_candidate_with_early_stopping, top8k_cands, chunksize=50))
    t2_time = time.perf_counter() - t2_start
    t2_scores = np.array(t2_scores)
    
    top18_idx = np.argsort(t2_scores)[:18]
    elite18_cands = [top8k_cands[i] for i in top18_idx]
    
    print(f"Tier 2 (Non-Linear ODE): 8,000 evals in {t2_time:.2f} s ({8000/t2_time:,.0f} evals/s)!")
    print(f"\nTop 5 Elites from Tier 2 Funnel:")
    for rank, idx in enumerate(top18_idx[:5]):
        cand = top8k_cands[idx]
        t2_sc = t2_scores[idx]
        t1_sc = t1_scores[top8k_idx[idx]]
        print(f"  Rank {rank+1}: Tier 2 Score = {t2_sc:>9.1f} pts (Tier 1 was {t1_sc:>9.1f} pts) | K_sc={cand[2]:.2f}, kp={cand[7]:.3f}, k_dyn={cand[10]:.2f}, k_stat={cand[11]:.2f}")
    
    total_time = t1_time + t2_time
    print(f"\nTotal Pre-Screening Funnel Time: {total_time:.2f} seconds!")
    print("="*70)

if __name__ == "__main__":
    main()

