import os
import sys
import json
import math
import time
import random
import datetime
import subprocess
import numpy as np
import cma
from concurrent.futures import ThreadPoolExecutor, as_completed

# Phase 2.2 Optimization Campaign: Isotropic Equal-Corridor Bode Loss
# Standard 1.0 dB Gain Error == 10.0 deg Phase Error == 100 pts
# Target: Beat Happee (2017) Model Score on Canonical Aligned Dataset (2,803.88)
# Upgrades for Gen 12+: 10ms VCR transport delay, 4-tier anatomical co-contraction,
# broadened ceilings on K_phas, lead ratios, p_ccr, kp, and p_sc.

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

# Baseline Physiological Parameter Bounds
lower_bounds = np.array([
    0.45,   # 0: r_ton (K_ton / K_sc)
     8.0,   # 1: p_ton (rad/s)
    30.0,   # 2: K_sc (Nm/(rad/s))
    0.01,   # 3: r_lead_sc (z_sc / p_sc)
    25.0,   # 4: p_sc (rad/s)
     1.0,   # 5: K_phas (Nm/(m/s^2))
    25.0,   # 6: p_phas (rad/s)
    0.25,   # 7: kp (spindle stiffness gain)
    0.10,   # 8: r_lead_ccr (z_ccr / p_ccr)
    15.0,   # 9: p_ccr (rad/s)
    0.15,   # 10: k_gamma_dyn
    0.15    # 11: k_gamma_stat
])

upper_bounds = np.array([
    2.00,   # 0: r_ton
    40.0,   # 1: p_ton
    95.0,   # 2: K_sc
    0.55,   # 3: r_lead_sc
    90.0,   # 4: p_sc
    14.0,   # 5: K_phas
    85.0,   # 6: p_phas
    1.80,   # 7: kp
    0.55,   # 8: r_lead_ccr
    75.0,   # 9: p_ccr
    1.20,   # 10: k_gamma_dyn
    1.20    # 11: k_gamma_stat
])

def evaluate_freq(candidate, target_idx, w_idx, gen_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = int(target_sim_time * 240 * 2.5)
    worker_label = f"W{w_idx+1}_P22G{gen_idx}"
    
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
            print(f"[FAIL] on {freq}Hz [{worker_label}]: {result.stderr}", file=sys.stderr)
            return 100000.0 + random.uniform(0, 100.0)
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] on {freq}Hz [{worker_label}]", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)
    except Exception as e:
        print(f"[ERROR] on {freq}Hz [{worker_label}]: {e}", file=sys.stderr)
        return 100000.0 + random.uniform(0, 100.0)

def main():
    # Initial Vector x0: Phase 2.1 All-Time Champion (W12_P21G13)
    x0 = np.array([
        0.910986,  # 0: r_ton (K_ton = 34.78)
        17.414272, # 1: p_ton
        38.176383, # 2: K_sc
        0.265675,  # 3: r_lead_sc (z_sc = 18.52 rad/s)
        69.691364, # 4: p_sc
        5.036390,  # 5: K_phas
        56.844082, # 6: p_phas
        1.194967,  # 7: kp
        0.430164,  # 8: r_lead_ccr (z_ccr = 22.06 rad/s)
        51.277029, # 9: p_ccr
        0.623851,  # 10: k_dyn
        0.773309   # 11: k_stat
    ])
    
    # Recalibrated Search Sigmas (~10-12% of parameter range)
    sigma0 = np.array([
        0.07,   # r_ton
        3.0,    # p_ton
        6.0,    # K_sc
        0.04,   # r_lead_sc
        7.0,    # p_sc
        1.0,    # K_phas
        6.0,    # p_phas
        0.10,   # kp
        0.04,   # r_lead_ccr
        6.0,    # p_ccr
        0.08,   # k_dyn
        0.08    # k_stat
    ])
    
    max_total_generations = 30
    target_stop_generation = int(os.environ.get("PHASE2_2_STOP_GEN", "30"))
    
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': max_total_generations,
        'seed': 2026
    })
    
    os.makedirs("logs", exist_ok=True)
    os.makedirs("logs/candidates", exist_ok=True)
    
    # Replay completed Phase 2.2 generations with exact historical bounds
    completed_gens = 0
    global_best_score = float('inf')
    global_best_sol = x0.copy()
    
    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_2_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            if g == 10:
                lb_g10 = lower_bounds.copy()
                ub_g10 = upper_bounds.copy()
                lb_g10[1] = 5.0    # p_ton broadened
                ub_g10[11] = 2.00  # k_stat broadened
                es.boundary_handler = cma.boundary_handler.BoundTransform([lb_g10, ub_g10])
                es.opts['bounds'] = [lb_g10, ub_g10]
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            cands = es.ask()
            es.tell(cands, sdata["all_scores"])
            completed_gens = g
            print(f"[REPLAY] Phase 2.2 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    # Apply fully expanded bounds for Generation 12+ (Levers 4 & 5 + bound expansions)
    lb_active = lower_bounds.copy()
    ub_active = upper_bounds.copy()
    lb_active[1] = 5.0    # p_ton lower bound broadened to 5.0
    ub_active[11] = 2.00  # k_stat upper bound broadened to 2.00
    ub_active[3] = 0.70   # r_lead_sc ceiling widened to 0.70
    ub_active[4] = 110.0  # p_sc ceiling widened to 110.0
    ub_active[5] = 25.0   # K_phas ceiling widened to 25.0
    ub_active[7] = 2.50   # kp ceiling widened to 2.50
    ub_active[8] = 0.70   # r_lead_ccr ceiling widened to 0.70
    ub_active[9] = 100.0  # p_ccr ceiling widened to 100.0
    
    es.boundary_handler = cma.boundary_handler.BoundTransform([lb_active, ub_active])
    es.opts['bounds'] = [lb_active, ub_active]
    print(f"\n[BOUNDS UPDATED for Gen {completed_gens + 1}+] Full 5-Lever physiological bounds active.")
    print(f"  p_ton: [{lb_active[1]:.1f}, {ub_active[1]:.1f}] | k_stat: [{lb_active[11]:.2f}, {ub_active[11]:.2f}]")
    print(f"  K_phas: [{lb_active[5]:.1f}, {ub_active[5]:.1f}] | kp: [{lb_active[7]:.2f}, {ub_active[7]:.2f}]")
    print(f"  r_lead_sc: [{lb_active[3]:.2f}, {ub_active[3]:.2f}] | r_lead_ccr: [{lb_active[8]:.2f}, {ub_active[8]:.2f}]")
    print(f"  p_sc: [{lb_active[4]:.1f}, {ub_active[4]:.1f}] | p_ccr: [{lb_active[9]:.1f}, {ub_active[9]:.1f}]")
    
    # Reset global best for new canonical metric tracking
    global_best_score = float('inf')
    HAPPEE_BENCHMARK = 2803.88
            
    print(f"\nPhase 2.2 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Target Benchmark to Beat: Happee (2017) Model Score = {HAPPEE_BENCHMARK:.2f}")
    
    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        iteration += 1
        gen_start_time = datetime.datetime.now()
        print(f"\n{'='*35} STARTING PHASE 2.2 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)
        
        candidates = es.ask()
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
                    
                    if score < 50000.0:
                        all_scores[c_idx] += score
                        print(f"[PASS] [W{c_idx+1}_P22G{iteration}] passed {freq:.3f}Hz ({t_idx+1}/{len(TARGETS)}) (Score: {score:.1f} | Cum: {all_scores[c_idx]:.1f})", flush=True)
                        if t_idx + 1 < len(TARGETS):
                            next_fut = executor.submit(evaluate_freq, candidates[c_idx], t_idx + 1, c_idx, iteration)
                            future_to_info[next_fut] = (c_idx, t_idx + 1)
                        else:
                            print(f"[COMPLETE] [W{c_idx+1}_P22G{iteration}] finished all 6 frequencies! Total Score: {all_scores[c_idx]:.2f}", flush=True)
                    else:
                        penalty = (len(TARGETS) - t_idx) * 20000.0
                        all_scores[c_idx] += (score + penalty)
                        print(f"[EARLY STOPPING] [W{c_idx+1}_P22G{iteration}] failed on {freq:.3f}Hz (Score: {score:.1f}). Penalty added.", flush=True)
                        
                    active_count = len(future_to_info)
                    break
                    
        es.tell(candidates, all_scores)
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds() / 60.0
        
        best_idx = int(np.argmin(all_scores))
        best_score = float(all_scores[best_idx])
        best_cand = candidates[best_idx]
        
        is_new_record = best_score < global_best_score
        if is_new_record:
            global_best_score = best_score
            global_best_sol = best_cand.copy()
            beat_happee_str = " *** BEATS HAPPEE (2017) MODEL! ***" if best_score < HAPPEE_BENCHMARK else ""
            print(f"\n>>> NEW PHASE 2.2 RECORD in Gen {iteration}: {best_score:.2f} (Candidate W{best_idx+1}_P22G{iteration}){beat_happee_str} <<<\n", flush=True)
            
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
            "phase": 2.2,
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
        
        summary_path = f"logs/phase2_2_gen_{iteration}_summary.json"
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
            
        print(f"Phase 2.2 Generation {iteration} completed in {gen_duration:.1f} mins.", flush=True)
        print(f"Best Candidate: W{best_idx+1}_P22G{iteration} | Score: {best_score:.2f} (Target: < {HAPPEE_BENCHMARK:.2f})", flush=True)
        print(f"r_ton: {r_ton:.3f} | K_ton: {K_ton:.2f} | K_sc: {K_sc:.2f} | z_sc: {z_sc:.2f} | z_ccr: {z_ccr:.2f}\n", flush=True)

    print(f"PHASE 2.2 TARGET GENERATIONS (1..{target_stop_generation}) COMPLETE!", flush=True)

if __name__ == "__main__":
    main()
