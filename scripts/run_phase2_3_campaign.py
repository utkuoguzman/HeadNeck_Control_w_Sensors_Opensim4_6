import os
import sys
import json
import math
import cmath
import time
import random
import datetime
import subprocess
import numpy as np
import cma
from scipy.stats import qmc
from concurrent.futures import ThreadPoolExecutor, as_completed, ProcessPoolExecutor
sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "scratch"))
from scratch.test_tier2_accelerated import fast_eval_candidate_with_early_stopping
from scratch.physics_informed_residual_regressor import predict_opensim_scores

class Tee(object):
    def __init__(self, name, mode):
        os.makedirs(os.path.dirname(name), exist_ok=True)
        self.file = open(name, mode, encoding="utf-8")
        self.stdout = sys.stdout
        sys.stdout = self
    def __del__(self):
        sys.stdout = self.stdout
        self.file.close()
    def write(self, data):
        self.file.write(data)
        try:
            self.stdout.write(data)
        except UnicodeEncodeError:
            self.stdout.write(data.encode('ascii', errors='replace').decode('ascii'))
        self.file.flush()
        self.stdout.flush()
    def flush(self):
        self.file.flush()
        self.stdout.flush()

Tee("logs/phase2_3_campaign.log", "a")

# Phase 2.3 Optimization Campaign: Canonical Angular Velocity Bode Optimization
# Isotropic Equal-Corridor Bode Loss: 1.0 dB Gain Error == 10.0 deg Phase Error == 100 pts
# Benchmark to Beat: Happee (2017) Model Score on Canonical Aligned Data = 2,803.88
# Upgrades:
#  - 10ms VCR transport delay
#  - 4-tier anatomical co-contraction floors (2.5% to 7.0%)
#  - Recalibrated physiological compliance parameters (x0, widened bounds, adaptive sigmas)
#  - Robust early-stopping threshold (250,000 pts) allowing full 18-worker exploration

FIXED_W_N = 22.0
FIXED_ZETA = 0.707
FIXED_KI = 40.2786

# 6 Happee (2017) Canonical Aligned Frequencies (LPT / early-pruning order)
TARGETS = [
    (0.324, 86.55,   -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (5.583, 503.30, -201.38),
    (2.446, 423.34, -161.34),
    (7.927, 572.69, -247.62)
]

# Recalibrated Physiological Bounds for Real Human Compliance
lower_bounds = np.array([
    0.20,   # 0: r_ton (K_ton / K_sc)
     2.0,   # 1: p_ton (rad/s)
     0.20,  # 2: K_sc (Nm/(rad/s))
    0.01,   # 3: r_lead_sc (z_sc / p_sc)
    15.0,   # 4: p_sc (rad/s)
     0.20,  # 5: K_phas (Nm/(m/s^2))
    15.0,   # 6: p_phas (rad/s)
    0.15,   # 7: kp (spindle stiffness gain)
    0.01,   # 8: r_lead_ccr (z_ccr / p_ccr)
    10.0,   # 9: p_ccr (rad/s)
    0.10,   # 10: k_dyn
    0.15    # 11: k_stat
])

upper_bounds = np.array([
    2.50,   # 0: r_ton
    45.0,   # 1: p_ton
    30.0,   # 2: K_sc
    0.60,   # 3: r_lead_sc
   100.0,   # 4: p_sc
    15.0,   # 5: K_phas
    80.0,   # 6: p_phas
    3.50,   # 7: kp
    0.65,   # 8: r_lead_ccr
    90.0,   # 9: p_ccr
    1.20,   # 10: k_dyn
    1.50    # 11: k_stat
])


# Phase 2.3 Section 3 Surrogate Pre-Screening Engine
SURR_OPT_PLANT = [0.6000, 0.3636, 2.8558, 0.0050, 0.0085, 1.348]
SURR_DELAY_VCR = 0.010
SURR_DELAY_CCR = 0.015

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
        H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * SURR_DELAY_CCR)
        
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

def evaluate_freq(candidate, target_idx, w_idx, gen_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = int(target_sim_time * 240 * 2.5)
    worker_label = f"W{w_idx+1}_P23G{gen_idx}"
    
    try:
        cmd = [
            sys.executable, "cmaes_worker_p22.py",
            str(FIXED_W_N), str(FIXED_ZETA), str(FIXED_KI),
            *[str(v) for v in candidate],
            str(freq), str(t_gain), str(t_phase),
            worker_label
        ]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=dynamic_timeout
        )
        if result.returncode == 0:
            lines = [l.strip() for l in result.stdout.strip().split("\n") if l.strip()]
            score = float(lines[-1])
            return score
        else:
            print(f"[FAIL] on {freq:.3f}Hz [{worker_label}]: {result.stderr}", file=sys.stderr)
            return 1000000.0 + random.uniform(0, 100.0)
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] on {freq:.3f}Hz [{worker_label}]", file=sys.stderr)
        return 1000000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"[ERROR] on {freq:.3f}Hz [{worker_label}]: {e}", file=sys.stderr)
        return 1000000.0 + random.uniform(0, 100.0)

def set_cma_bounds(es, lower, upper):
    new_bh = cma.boundary_handler.BoundTransform([lower, upper])
    es.boundary_handler = new_bh
    es.gp.bounds = new_bh
    es.opts['bounds'] = [lower, upper]

def main():
    # Calibrated Initial Seed Vector x0 (Bootstrapped directly from Exact-Plant Differentiable Surrogate)
    # OpenSim 5.58 Hz score = 162.3 pts (+0.27 dB), 0.78 Hz phase error = 15.2 deg
    x0 = np.array([
        0.327,  # 0: r_ton (K_ton = 3.27 Nm/rad, K_sc = 10.0 Nm/(rad/s))
       40.000,  # 1: p_ton (rad/s)
       10.000,  # 2: K_sc (Nm/(rad/s))
        0.092,  # 3: r_lead_sc (z_sc = 5.13 rad/s)
       55.723,  # 4: p_sc (rad/s)
        3.806,  # 5: K_phas (Nm/(m/s^2))
       60.000,  # 6: p_phas (rad/s)
        3.000,  # 7: kp (spindle stiffness gain)
        0.033,  # 8: r_lead_ccr (z_ccr = 0.50 rad/s)
       15.000,  # 9: p_ccr (rad/s)
        0.250,  # 10: k_dyn
        0.400   # 11: k_stat
    ])
    
    # Standard Healthy Initial Search Sigmas (~15-20% of parameter domain)
    sigma0 = np.array([
        0.15,   # r_ton (domain [0.20, 2.50])
        5.00,   # p_ton (domain [2.0, 45.0])
        2.50,   # K_sc (domain [0.20, 30.0])
        0.05,   # r_lead_sc (domain [0.01, 0.60])
        8.00,   # p_sc (domain [15.0, 100.0])
        1.20,   # K_phas (domain [0.20, 15.0])
        6.00,   # p_phas (domain [15.0, 80.0])
        0.35,   # kp (domain [0.15, 3.50])
        0.04,   # r_lead_ccr (domain [0.01, 0.65])
        6.00,   # p_ccr (domain [10.0, 90.0])
        0.12,   # k_dyn (domain [0.10, 1.20])
        0.15    # k_stat (domain [0.15, 1.50])
    ])
    
    max_total_generations = 30
    target_stop_generation = int(os.environ.get("PHASE2_3_STOP_GEN", "30"))
    
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': max_total_generations,
        'seed': 2026
    })
    
    os.makedirs("logs", exist_ok=True)
    os.makedirs("logs/candidates", exist_ok=True)
    
    # Replay completed Phase 2.3 generations if resuming
    completed_gens = 0
    global_best_score = float('inf')
    global_best_sol = x0.copy()
    HAPPEE_BENCHMARK = 2803.88
    
    # Phase 2.3 Generation 3 Expanded Headroom Bounds
    gen3_lower_bounds = np.array([
        0.05,   # 0: r_ton (extended down from 0.20)
        2.0,    # 1: p_ton (rad/s)
        0.20,   # 2: K_sc (Nm/(rad/s))
        0.001,  # 3: r_lead_sc (extended down from 0.01)
        15.0,   # 4: p_sc (rad/s)
        0.20,   # 5: K_phas (Nm/(m/s^2))
        15.0,   # 6: p_phas (rad/s)
        0.15,   # 7: kp (spindle stiffness gain)
        0.001,  # 8: r_lead_ccr (extended down from 0.01)
        5.0,    # 9: p_ccr (rad/s, extended down from 10.0)
        0.10,   # 10: k_dyn
        0.15    # 11: k_stat
    ])
    
    gen3_upper_bounds = np.array([
        2.50,   # 0: r_ton
        80.0,   # 1: p_ton (rad/s, extended up from 45.0)
        30.0,   # 2: K_sc
        0.60,   # 3: r_lead_sc
       100.0,   # 4: p_sc
        15.0,   # 5: K_phas
        80.0,   # 6: p_phas
        5.00,   # 7: kp (extended up from 3.50)
        0.65,   # 8: r_lead_ccr
        90.0,   # 9: p_ccr
        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    # Phase 2.3 Generation 4+ Expanded Headroom Bounds (k_stat extended down to 0.01)
    gen4_lower_bounds = np.array([
        0.05,   # 0: r_ton (extended down from 0.20)
        2.0,    # 1: p_ton (rad/s)
        0.20,   # 2: K_sc (Nm/(rad/s))
        0.001,  # 3: r_lead_sc (extended down from 0.01)
        15.0,   # 4: p_sc (rad/s)
        0.20,   # 5: K_phas (Nm/(m/s^2))
        15.0,   # 6: p_phas (rad/s)
        0.15,   # 7: kp (spindle stiffness gain)
        0.001,  # 8: r_lead_ccr (extended down from 0.01)
        5.0,    # 9: p_ccr (rad/s, extended down from 10.0)
        0.10,   # 10: k_dyn
        0.01    # 11: k_stat (extended down from 0.15 to 0.01 for full static gamma headroom)
    ])
    
    gen4_upper_bounds = np.array([
        2.50,   # 0: r_ton
        80.0,   # 1: p_ton (rad/s, extended up from 45.0)
        30.0,   # 2: K_sc
        0.60,   # 3: r_lead_sc
       100.0,   # 4: p_sc
        15.0,   # 5: K_phas
        80.0,   # 6: p_phas
        5.00,   # 7: kp (extended up from 3.50)
        0.65,   # 8: r_lead_ccr
        90.0,   # 9: p_ccr
        1.20,   # 10: k_dyn
        1.50    # 11: k_stat
    ])

    # Phase 2.3 Generation 12+ Expanded Headroom Bounds (k_dyn extended up to 1.50)
    gen12_lower_bounds = gen4_lower_bounds.copy()
    gen12_upper_bounds = gen4_upper_bounds.copy()
    gen12_upper_bounds[10] = 1.50  # 10: k_dyn expanded headroom up to 1.50

    # Phase 2.3 Generation 14+ Expanded Headroom Bounds (p_ccr extended up to 200.0 rad/s)
    gen14_lower_bounds = gen12_lower_bounds.copy()
    gen14_upper_bounds = gen12_upper_bounds.copy()
    gen14_upper_bounds[9] = 200.0  # 9: p_ccr expanded headroom up to 200.0 rad/s

    # Phase 2.3 Generation 16+ Expanded Headroom Bounds (p_ton up to 120.0 rad/s, p_sc up to 150.0 rad/s)
    gen16_lower_bounds = gen14_lower_bounds.copy()
    gen16_upper_bounds = gen14_upper_bounds.copy()
    gen16_upper_bounds[1] = 120.0  # 1: p_ton extended up from 80.0 to 120.0 rad/s
    gen16_upper_bounds[4] = 150.0  # 4: p_sc extended up from 100.0 to 150.0 rad/s

    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_3_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            if g == 3:
                set_cma_bounds(es, gen3_lower_bounds, gen3_upper_bounds)
            elif 4 <= g <= 11:
                set_cma_bounds(es, gen4_lower_bounds, gen4_upper_bounds)
            elif 12 <= g <= 13:
                set_cma_bounds(es, gen12_lower_bounds, gen12_upper_bounds)
            elif 14 <= g <= 15:
                set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
            elif g >= 16:
                set_cma_bounds(es, gen16_lower_bounds, gen16_upper_bounds)
            cands = es.ask()
            b_low, b_upp = es.boundary_handler.bounds
            clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]
            es.tell(clamped_replay, sdata["all_scores"])
            completed_gens = g
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = cands[sdata["best_candidate_idx"] - 1]
            print(f"[REPLAY] Phase 2.3 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    print(f"\nPhase 2.3 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Target Benchmark to Beat: Happee (2017) Model Score = {HAPPEE_BENCHMARK:.2f}")
    print(f"Initial Candidate x0:\n{x0.round(2)}")
    print(f"Sigma0:\n{sigma0.round(2)}")
    
    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        
        # Apply expanded bounds for Generation 3 and Generation 4+
        if iteration == 3:
            set_cma_bounds(es, gen3_lower_bounds, gen3_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation 3: r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 4 <= iteration <= 11:
            set_cma_bounds(es, gen4_lower_bounds, gen4_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 12 <= iteration <= 13:
            set_cma_bounds(es, gen12_lower_bounds, gen12_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: k_dyn in [0.10, 1.50] (expanded headroom), k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], p_ccr in [5.0, 90.0], leads in [0.001, ...]", flush=True)
        elif 14 <= iteration <= 15:
            set_cma_bounds(es, gen14_lower_bounds, gen14_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ccr in [5.0, 200.0] (expanded headroom), k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], p_ton in [2.0, 80.0], kp in [0.15, 5.00], leads in [0.001, ...]", flush=True)
        elif iteration >= 16:
            set_cma_bounds(es, gen16_lower_bounds, gen16_upper_bounds)
            print(f"[BOUNDS EXPANSION] Active bounds for Generation {iteration}: p_ton in [2.0, 120.0] (expanded headroom), p_sc in [15.0, 150.0] (expanded headroom), p_ccr in [5.0, 200.0], k_dyn in [0.10, 1.50], k_stat in [0.01, 1.50], r_ton in [0.05, 2.50], kp in [0.15, 5.00]", flush=True)
            
        print(f"\n{'='*35} STARTING PHASE 2.3 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)
        
        # Pure Covariance Matrix Adaptation (CMA-ES)
        print(f"[CMA-ES SAMPLING] Sampling population of 18 directly from multivariate normal distribution N(mu, C)...", flush=True)
        raw_cands = es.ask()
        b_low, b_upp = es.boundary_handler.bounds
        candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in raw_cands]
        all_scores = [0.0] * len(candidates)
        
        # Max concurrency: 14 physical cores
        with ThreadPoolExecutor(max_workers=14) as executor:
            future_to_info = {}
            for c_idx in range(len(candidates)):
                fut = executor.submit(evaluate_freq, candidates[c_idx], 0, c_idx, iteration)
                future_to_info[fut] = (c_idx, 0)
                
            active_count = len(future_to_info)
            while active_count > 0:
                for fut in as_completed(list(future_to_info.keys())):
                    c_idx, t_idx = future_to_info.pop(fut)
                    score = fut.result()
                    freq, _, _ = TARGETS[t_idx]
                    
                    # Robust early-stopping threshold (250,000 pts)
                    if score < 250000.0:
                        all_scores[c_idx] += score
                        print(f"[PASS] [W{c_idx+1}_P23G{iteration}] passed {freq:.3f}Hz ({t_idx+1}/{len(TARGETS)}) (Score: {score:.1f} | Cum: {all_scores[c_idx]:.1f})", flush=True)
                        if t_idx + 1 < len(TARGETS):
                            next_fut = executor.submit(evaluate_freq, candidates[c_idx], t_idx + 1, c_idx, iteration)
                            future_to_info[next_fut] = (c_idx, t_idx + 1)
                        else:
                            print(f"[COMPLETE] [W{c_idx+1}_P23G{iteration}] finished all 6 frequencies! Total Score: {all_scores[c_idx]:.2f}", flush=True)
                    else:
                        penalty = (len(TARGETS) - t_idx) * 200000.0
                        all_scores[c_idx] += (score + penalty)
                        print(f"[EARLY STOPPING] [W{c_idx+1}_P23G{iteration}] aborted on {freq:.3f}Hz (Score: {score:.1f}). Penalty added.", flush=True)
                        
                    active_count = len(future_to_info)
                    break
                    
        eps = 1e-6
        b_low, b_upp = es.boundary_handler.bounds
        clamped_candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in candidates]
        es.tell(clamped_candidates, all_scores)
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds() / 60.0
        
        best_idx = int(np.argmin(all_scores))
        best_score = float(all_scores[best_idx])
        best_cand = candidates[best_idx]
        
        is_new_record = best_score < global_best_score
        if is_new_record:
            global_best_score = best_score
            global_best_sol = best_cand.copy()
            beat_happee_str = " *** BEATS HAPPEE (2017) MODEL! ***" if best_score < HAPPEE_BENCHMARK else ""
            print(f"\n>>> NEW PHASE 2.3 RECORD in Gen {iteration}: {best_score:.2f} (Candidate W{best_idx+1}_P23G{iteration}){beat_happee_str} <<<\n", flush=True)
            
        r_ton = best_cand[0]
        p_ton = best_cand[1]
        K_sc = best_cand[2]
        r_lead_sc = best_cand[3]
        p_sc = best_cand[4]
        K_phas = best_cand[5]
        p_phas = best_cand[6]
        kp_prop = best_cand[7]
        r_lead_ccr = best_cand[8]
        p_ccr = best_cand[9]
        k_dyn = best_cand[10]
        k_stat = best_cand[11]
        
        K_ton = r_ton * K_sc
        z_sc = r_lead_sc * p_sc
        z_ccr = r_lead_ccr * p_ccr
        
        summary = {
            "phase": 2.3,
            "generation": iteration,
            "start_time": gen_start_time.isoformat(),
            "end_time": datetime.datetime.now().isoformat(),
            "duration_min": gen_duration,
            "best_candidate_idx": best_idx + 1,
            "best_score": best_score,
            "r_ton": float(r_ton),
            "best_params": {
                "r_ton": float(r_ton),
                "K_ton": float(K_ton),
                "p_ton": float(p_ton),
                "K_sc": float(K_sc),
                "r_lead_sc": float(r_lead_sc),
                "z_sc": float(z_sc),
                "p_sc": float(p_sc),
                "K_phas": float(K_phas),
                "p_phas": float(p_phas),
                "kp": float(kp_prop),
                "r_lead_ccr": float(r_lead_ccr),
                "z_ccr": float(z_ccr),
                "p_ccr": float(p_ccr),
                "k_dyn": float(k_dyn),
                "k_stat": float(k_stat)
            },
            "all_scores": [float(s) for s in all_scores]
        }
        
        summary_path = f"logs/phase2_3_gen_{iteration}_summary.json"
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        try:
            sys.path.append('scratch')
            import update_ledger
            update_ledger.update_ledger(iteration)
        except Exception as e_led:
            print(f'Warning: failed auto-ledger update: {e_led}', flush=True)
            
        print(f"Phase 2.3 Generation {iteration} completed in {gen_duration:.1f} mins.", flush=True)
        print(f"Best Candidate: W{best_idx+1}_P23G{iteration} | Score: {best_score:.2f} (Target: < {HAPPEE_BENCHMARK:.2f})", flush=True)
        print(f"r_ton: {r_ton:.3f} | K_ton: {K_ton:.2f} | K_sc: {K_sc:.2f} | z_sc: {z_sc:.2f} | z_ccr: {z_ccr:.2f}\n", flush=True)

    print(f"PHASE 2.3 TARGET GENERATIONS (1..{target_stop_generation}) COMPLETE!", flush=True)

if __name__ == "__main__":
    main()
