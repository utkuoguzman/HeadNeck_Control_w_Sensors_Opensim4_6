import os
import sys
import json
import math
import cmath
import time
import numpy as np
from scipy.stats import qmc
from concurrent.futures import ProcessPoolExecutor

sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "scratch"))

from scratch.test_tier2_accelerated import fast_eval_candidate_with_early_stopping

SURR_DELAY_VCR = 0.010
SURR_DELAY_CCR = 0.015
SURR_OPT_PLANT = (0.0185, 0.45, 12.5, 0.145, 0.012, 1.0)
TARGETS = [(0.324, (86.55, -7.03)), (0.783, (236.93, -96.96)), (1.099, (319.74, -109.23)), (2.446, (423.34, -161.34)), (5.583, (503.30, -201.38)), (7.927, (572.69, -247.62))]

def surrogate_score(cand):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = cand
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    J_eff, B_pass, K_pass, M_drive, tau_act, k_geom = SURR_OPT_PLANT
    
    total_score = 0.0
    for freq, (t_gain, t_phase) in TARGETS:
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
    print("="*75, flush=True)
    print("PRE-SCREENING AROUND CHAMPION (W7_P23G13) WITH p_ccr <= 200 rad/s", flush=True)
    print("="*75, flush=True)
    
    # Expanded bounds for Gen 14+: p_ccr up to 200.0
    lower_bounds = np.array([0.05, 2.0, 0.05, 0.001, 1.0, 0.0, 0.5, 0.15, 0.001, 5.0, 0.10, 0.01])
    upper_bounds = np.array([2.50, 80.0, 30.0, 1.000, 50.0, 50.0, 30.0, 5.00, 1.000, 200.0, 1.50, 1.50])
    
    # 1. Load Champion W7_P23G13
    with open("logs/candidates/W7_P23G13.json", "r", encoding="utf-8") as f:
        cand_data = json.load(f)
    p = cand_data["params"]
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr, k_dyn, k_stat = p
    r_ton = K_ton / K_sc
    r_lead_sc = z_sc / p_sc
    r_lead_ccr = z_ccr / p_ccr
    champ_x = np.array([r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat])
    
    print(f"Champion W7_P23G13 Parameters (baseline OpenSim score: 26,285.00 pts):", flush=True)
    print(f"  r_ton={r_ton:.3f}, p_ton={p_ton:.1f}, K_sc={K_sc:.2f}, r_lead_sc={r_lead_sc:.4f}, p_sc={p_sc:.1f}", flush=True)
    print(f"  K_phas={K_phas:.2f}, p_phas={p_phas:.1f}, kp={kp:.3f}, r_lead_ccr={r_lead_ccr:.4f}, p_ccr={p_ccr:.1f} (old ceiling: 90.0)", flush=True)
    print(f"  k_dyn={k_dyn:.2f}, k_stat={k_stat:.2f}\n", flush=True)
    
    # 2. Generate Candidate Pool: 24,000 Local Champion Perturbations + 65,536 Global Sobol Samples
    t0 = time.perf_counter()
    np.random.seed(2026)
    
    # Local Gaussian sampling around champion with varying perturbation scales (tight, medium, broad)
    stds = (upper_bounds - lower_bounds)
    local_samples = []
    
    # 8,000 tight samples (sigma = 3%)
    local_samples.append(np.random.normal(champ_x, stds * 0.03, size=(8000, 12)))
    # 8,000 medium samples (sigma = 8%)
    local_samples.append(np.random.normal(champ_x, stds * 0.08, size=(8000, 12)))
    # 8,000 targeted p_ccr sweep samples (champion with p_ccr drawn uniformly from [80, 200])
    pccr_sweep = np.tile(champ_x, (8000, 1))
    pccr_sweep[:, 9] = np.random.uniform(80.0, 200.0, size=8000)
    pccr_sweep += np.random.normal(0, stds * 0.04, size=(8000, 12))
    pccr_sweep[:, 9] = np.random.uniform(80.0, 200.0, size=8000) # enforce high p_ccr
    local_samples.append(pccr_sweep)
    
    local_pool = np.vstack(local_samples)
    local_pool = np.clip(local_pool, lower_bounds, upper_bounds).tolist()
    
    # 65,536 Global Sobol Sequence across full expanded bounds
    sobol_sampler = qmc.Sobol(d=12, scramble=True, seed=1400)
    unit_sobol = sobol_sampler.random(65536)
    global_pool = [c.tolist() for c in qmc.scale(unit_sobol, lower_bounds, upper_bounds)]
    
    combined_pool = local_pool + global_pool
    print(f"Generated pool of {len(combined_pool):,} candidates (24,000 Local Champion + 65,536 Global Sobol) in {time.perf_counter()-t0:.2f} s", flush=True)
    
    # 3. Tier 1: Fast Algebraic ROM Spline Pre-Screening (89,536 candidates)
    t1_t0 = time.perf_counter()
    t1_scores = [surrogate_score(c) for c in combined_pool]
    t1_scores = np.array(t1_scores)
    t1_ranked = np.argsort(t1_scores)
    top8k_cands = [combined_pool[i] for i in t1_ranked[:8000]]
    t1_dt = time.perf_counter() - t1_t0
    
    print(f"\n[TIER 1 COMPLETE] Screened {len(combined_pool):,} candidates in {t1_dt:.2f} s ({len(combined_pool)/t1_dt:,.0f} evals/s)!", flush=True)
    print(f"  Best Tier 1 Algebraic Score: {t1_scores[t1_ranked[0]]:.1f} pts (Worst in top 8k: {t1_scores[t1_ranked[7999]]:.1f} pts)", flush=True)
    print(f"  Top candidate p_ccr in Tier 1: {top8k_cands[0][9]:.1f} rad/s", flush=True)
    
    # 4. Tier 2: Option A Non-Linear 2-DOF Musculoskeletal Time-Domain ODE Simulator
    print(f"\n[TIER 2 DISPATCH] Screening top {len(top8k_cands):,} candidates through Non-Linear Muscle ODE Simulator across 14 cores...", flush=True)
    t2_t0 = time.perf_counter()
    with ProcessPoolExecutor(max_workers=14) as t2_executor:
        t2_scores = list(t2_executor.map(fast_eval_candidate_with_early_stopping, top8k_cands, chunksize=50))
    t2_scores = np.array(t2_scores)
    t2_ranked = np.argsort(t2_scores)
    top18_cands = [top8k_cands[i] for i in t2_ranked[:18]]
    t2_dt = time.perf_counter() - t2_t0
    
    print(f"[TIER 2 COMPLETE] Screened 8,000 non-linear candidates in {t2_dt:.2f} s ({8000/t2_dt:,.0f} evals/s)!", flush=True)
    print(f"\n{'='*75}", flush=True)
    print("TOP 10 ELITE CANDIDATES FROM PRE-SCREENING FUNNEL (WITH p_ccr UP TO 200):", flush=True)
    print(f"{'Rank':<5} | {'Tier 2 Score':<12} | {'p_ccr (rad/s)':<13} | {'K_sc':<6} | {'kp':<6} | {'k_dyn':<6} | {'r_lead_ccr':<10}", flush=True)
    print("-"*75, flush=True)
    for rank, idx in enumerate(t2_ranked[:10]):
        c = top8k_cands[idx]
        print(f"#{rank+1:<4} | {t2_scores[idx]:>12.1f} | {c[9]:>13.1f} | {c[2]:>6.2f} | {c[7]:>6.3f} | {c[10]:>6.2f} | {c[8]:>10.4f}", flush=True)
    print("="*75, flush=True)
    
    # Save top 18 candidates for OpenSim injection
    out_file = "logs/prescreened_elites_gen14.json"
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump({
            "total_candidates_screened": len(combined_pool),
            "tier1_duration_s": t1_dt,
            "tier2_duration_s": t2_dt,
            "top18_candidates": top18_cands,
            "top18_scores": [float(t2_scores[i]) for i in t2_ranked[:18]]
        }, f, indent=2)
    print(f"\nSaved top 18 elite candidates to {out_file}!", flush=True)

if __name__ == "__main__":
    main()

