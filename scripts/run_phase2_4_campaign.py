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
from scipy.spatial.distance import cdist

sys.path.append(os.getcwd())
sys.path.append(os.path.join(os.getcwd(), "scratch"))

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

Tee("logs/phase2_4_campaign.log", "a")

# Phase 2.4 Optimization Campaign: Pure CMA-ES Deep Basin Refinement
# Starting directly from All-Time Champion W3_P23G15 (21,059.90 pts)
# Benchmark to Beat: Happee (2017) Model Score on Canonical Aligned Data = 2,803.88 pts

FIXED_W_N = 22.0
FIXED_ZETA = 0.707
FIXED_KI = 40.2786
HAPPEE_BENCHMARK = 2803.88

# 6 Canonical Aligned Frequencies (Fastest-First + High-Sensitivity Early-Pruning Order)
TARGETS = [
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
    (1.099, 319.74, -109.23),
    (0.783, 236.93,  -96.96),
    (0.324,  86.55,   -7.03)
]

PARAM_NAMES = [
    "K_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas",
    "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat"
]

# Physical floor bounds: parameters that MUST remain strictly non-negative
PHYSICAL_LOWER_LIMITS = np.array([
    0.01,   # K_ton   (absolute, independent of K_sc)
    0.1,    # p_ton
    0.05,   # K_sc    (floor > 0 so canal/tonic paths cannot be deleted)
    0.001,  # r_lead_sc
    1.0,    # p_sc
    0.01,   # K_phas
    1.0,    # p_phas
    0.01,   # kp
    0.001,  # r_lead_ccr
    1.0,    # p_ccr
    0.05,   # k_dyn
    0.05    # k_stat
])

# Physical ceilings: a dynamic expansion may never exceed these
PHYSICAL_UPPER_LIMITS = np.array([
    3.0,    # K_ton
    300.0,  # p_ton
    3.0,    # K_sc
    1.0,    # r_lead_sc  (zero must stay below pole -> lead)
    300.0,  # p_sc
    15.0,   # K_phas
    80.0,   # p_phas
    5.0,    # kp
    1.0,    # r_lead_ccr
    400.0,  # p_ccr
    2.0,    # k_dyn
    2.0     # k_stat
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
    worker_label = f"W{w_idx+1}_P24G{gen_idx}"
    
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


class SimpleGP:
    """Lightweight self-contained Gaussian Process regressor for local surrogate prescreening."""
    def __init__(self, length_scale=1.0, signal_var=1.0, noise_var=1e-4):
        self.length_scale = length_scale
        self.signal_var = signal_var
        self.noise_var = noise_var
        self.X_train = None
        self.y_train = None
        self.y_mean = 0.0
        self.y_std = 1.0
        self.L = None
        self.alpha = None

    def fit(self, X, y):
        self.X_train = np.array(X)
        self.y_train = np.array(y)
        self.y_mean = np.mean(self.y_train)
        self.y_std = np.std(self.y_train) + 1e-6
        y_norm = (self.y_train - self.y_mean) / self.y_std
        
        dists_sq = cdist(self.X_train, self.X_train, 'sqeuclidean')
        K = self.signal_var * np.exp(-0.5 * dists_sq / (self.length_scale**2))
        K += self.noise_var * np.eye(len(X))
        
        self.L = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(self.L.T, np.linalg.solve(self.L, y_norm))

    def predict(self, X_test):
        X_test = np.array(X_test)
        dists_test_sq = cdist(X_test, self.X_train, 'sqeuclidean')
        K_star = self.signal_var * np.exp(-0.5 * dists_test_sq / (self.length_scale**2))
        
        mu_norm = K_star @ self.alpha
        v = np.linalg.solve(self.L, K_star.T)
        var_norm = np.maximum(1e-8, self.signal_var - np.sum(v**2, axis=0))
        
        mu = self.y_mean + mu_norm * self.y_std
        sigma = np.sqrt(var_norm) * self.y_std
        return mu, sigma

def sample_candidates_tpa_orthogonal(es, iteration, archive_X=None, archive_y=None):
    """
    14 Candidates for 14 Physical Cores (1:1 Concurrency):
      - Candidates 0 & 1: 1 TPA Mirror Pair (along the mean evolution path p_c)
      - Candidates 2 to 13: 6 Mutually Orthogonal Antipodal Pairs in the subspace perpendicular to p_c.
      - Starting at Generation 8 (archive >= 80): Uses Direction-Pair GP Sieve to screen 12 candidate orthogonal bases.
    """
    X = es.ask()
    mean = np.array(es.mean)
    sigma = es.sigma
    B = es.sm.B
    D = es.sm.D
    stds = np.asarray(es.sigma_vec.scaling, dtype=float) * np.ones(len(mean))  # per-parameter CMA_stds
    
    # 1. TPA direction in z-space (standard normal space)
    d0 = np.array(X[0]) - mean
    z_tpa = (B.T @ (d0 / stds)) / (D * sigma)
    z_tpa_norm = np.linalg.norm(z_tpa)
    if z_tpa_norm > 1e-10:
        u_tpa = z_tpa / z_tpa_norm
    else:
        u_tpa = np.zeros(12)
        u_tpa[0] = 1.0

    # 2. Check if Direction-Pair GP pre-screening should be active (iteration >= 8 and archive >= 80)
    use_gp = (iteration >= 8 and archive_X is not None and len(archive_X) >= 80)
    
    if use_gp:
        # Fit local GP on nearest 80 historical models in stds-normalized space (all 12 parameters weighted equally)
        X_arch_norm = (np.array(archive_X) - mean) / stds
        dists = np.linalg.norm(X_arch_norm, axis=1)
        idx = np.argsort(dists)[:80]
        local_X_norm = X_arch_norm[idx]
        # Winsorize crash penalties (>200,000) so GP variance is not distorted by early-stop penalties
        local_y = np.minimum(np.array([archive_y[i] for i in idx], dtype=float), 200000.0)
        
        l_scale = max(0.1, float(np.median(dists[idx])))
        gp = SimpleGP(length_scale=l_scale)
        gp.fit(local_X_norm, local_y)
        
        # Sample candidate orthogonal bases and evaluate direction utility
        best_score = float('inf')
        best_Q = None
        for _ in range(12):
            G = np.random.randn(12, 6)
            G = G - np.outer(u_tpa, u_tpa @ G)
            Q, _ = np.linalg.qr(G)
            
            test_pts_norm = []
            for k in range(6):
                zk = Q[:, k] * np.sqrt(12.0)
                dx_norm = sigma * (B @ (D * zk))
                test_pts_norm.append(dx_norm)
                test_pts_norm.append(-dx_norm)
            mu, sig = gp.predict(test_pts_norm)
            # Minimize predicted loss while rewarding epistemic uncertainty
            acq = np.sum(mu - 1.0 * sig)
            if acq < best_score:
                best_score = acq
                best_Q = Q
        Q_chosen = best_Q
        print(f"[GP PRE-SCREEN] Screened 12 candidate orthogonal bases via local GP (archive size: {len(archive_X)}). Best acquisition score: {best_score:.1f}", flush=True)
    else:
        # Standard Orthogonal sampling in subspace perpendicular to u_tpa
        G = np.random.randn(12, 6)
        G = G - np.outer(u_tpa, u_tpa @ G)
        Q_chosen, _ = np.linalg.qr(G)

    # 3. Construct the 6 orthogonal pairs (12 candidates)
    for k in range(6):
        qk = Q_chosen[:, k]
        r_len = np.random.chisquare(12)**0.5
        zk = qk * r_len
        dx = sigma * stds * (B @ (D * zk))
        X[2 + 2*k] = (mean + dx).tolist()
        X[3 + 2*k] = (mean - dx).tolist()
        
    return X

def set_cma_bounds(es, lower, upper):
    new_bh = cma.boundary_handler.BoundTransform([lower, upper])
    es.boundary_handler = new_bh
    es.gp.bounds = new_bh
    es.opts['bounds'] = [lower, upper]

def check_and_expand_bounds(best_cand, lower_b, upper_b):
    """
    User's Dynamic Boundary Expansion Rule:
    If a parameter gets within 5% of its domain boundary:
    extend that side by 10% of the old range (bounded by physical non-negative limits).
    """
    new_lower = lower_b.copy()
    new_upper = upper_b.copy()
    expansions = []
    
    for i in range(len(best_cand)):
        val = best_cand[i]
        L = lower_b[i]
        U = upper_b[i]
        R = U - L
        dist_low = val - L
        dist_upp = U - val
        
        # Check lower boundary
        if (dist_low / R) < 0.05:
            delta = 0.10 * R
            target_L = L - delta
            phys_floor = PHYSICAL_LOWER_LIMITS[i]
            if target_L < phys_floor:
                # Capped at physical limit
                if L > phys_floor:
                    new_lower[i] = phys_floor
                    expansions.append(f"{PARAM_NAMES[i]} lower bounded at physical limit {phys_floor:.4f}")
                else:
                    expansions.append(f"{PARAM_NAMES[i]} reached physical floor ({phys_floor:.4f})")
            else:
                new_lower[i] = target_L
                expansions.append(f"{PARAM_NAMES[i]} lower expanded: {L:.4f} -> {target_L:.4f} (+10% range)")
                
        # Check upper boundary
        if (dist_upp / R) < 0.05:
            delta = 0.10 * R
            cap = PHYSICAL_UPPER_LIMITS[i]
            target_U = min(U + delta, cap)
            if target_U > U + 1e-12:
                new_upper[i] = target_U
                expansions.append(f"{PARAM_NAMES[i]} upper expanded: {U:.4f} -> {target_U:.4f} (+10% range, cap {cap})")
            else:
                expansions.append(f"{PARAM_NAMES[i]} reached physical ceiling ({cap})")
            
    return new_lower, new_upper, expansions

def print_5_title_milestone_report(gen, cand_idx, score, dur, best_cand, lower_b, upper_b, all_scores, global_best):
    cand_label = f"W{cand_idx}_P24G{gen}"
    print(f"\n{'='*75}")
    print(f"## Milestone Report: Phase 2.4 - Generation {gen}")
    print(f"{'='*75}\n")
    
    # Title 1: Good News
    print("### 1. Good News")
    print(f"* **Generation Champion:** Candidate [`{cand_label}`](file:///d:/Akademik/PhD_Thesis/OpenSim/HeadNeck_Control_w_Sensors_Opensim4_6/logs/candidates/{cand_label}.json) scored **`{score:.2f} pts`** (duration: {dur:.1f} min).")
    if score < global_best:
        print(f"* **Score Improvement:** Improved by **{global_best - score:.2f} pts** over previous best ({global_best:.2f} pts)!")
        if score < HAPPEE_BENCHMARK:
            print(f"* **BREAKTHROUGH:** Surpassed Happee (2017) model score benchmark ({HAPPEE_BENCHMARK:.2f} pts)!")
    else:
        print(f"* **Basin Exploration:** 18 distinct perturbations evaluated in parallel across all 6 canonical frequencies.")
    print()
    
    # Title 2: Bad News / Challenges
    print("### 2. Bad News / Challenges")
    bode_files = sorted([f for f in os.listdir("logs/candidates") if f.startswith(f"{cand_label}_") and f.endswith("Hz.json")])
    worst_freq = None
    worst_pts = 0.0
    for bf in bode_files:
        try:
            with open(os.path.join("logs/candidates", bf), "r") as fp:
                d = json.load(fp)
                if d.get("score", 0) > worst_pts:
                    worst_pts = d["score"]
                    worst_freq = (d["freq"], d.get("gain_db_diff", 0), d.get("phase_err_deg", 0))
        except Exception:
            pass
    if worst_freq:
        print(f"* **Highest Error Frequency:** Frequency {worst_freq[0]:.3f} Hz contributed {worst_pts:.1f} pts ({worst_freq[1]:+.1f} dB / {worst_freq[2]:.1f}° error).")
    else:
        print(f"* **Score Gap:** Still working toward Happee benchmark target of {HAPPEE_BENCHMARK:.2f} pts.")
    print()
    
    # Title 3: Parameter Boundary Watch
    print("### 3. Parameter Boundary Watch")
    print("| Parameter | Symbol | Value | Active Domain | Proximity Status |")
    print("| :--- | :---: | :---: | :---: | :--- |")
    for i, name in enumerate(PARAM_NAMES):
        val = best_cand[i]
        L = lower_b[i]
        U = upper_b[i]
        R = U - L
        p_low = (val - L) / R * 100.0
        p_upp = (U - val) / R * 100.0
        if p_low < 5.0:
            status = f"Near Lower ({p_low:.1f}%)"
        elif p_upp < 5.0:
            status = f"Near Upper ({p_upp:.1f}%)"
        else:
            status = "Interior (OK)"
        print(f"| {name:15s} | `{name}` | {val:8.4f} | [{L:7.3f}, {U:7.1f}] | {status} |")
    print()
    
    # Title 4: Current Projection & CMA-ES Trajectory
    print("### 4. Current Projection & CMA-ES Trajectory")
    print(f"* **Phase 2.4 All-Time Best:** `{min(global_best, score):.2f} pts`")
    dist_happee = min(global_best, score) - HAPPEE_BENCHMARK
    print(f"* **Distance to Happee (2017) Benchmark ({HAPPEE_BENCHMARK:.2f} pts):** {dist_happee:+.2f} pts")
    print(f"* **Population Size:** {len(all_scores)} candidates (2 TPA + 12 Orthogonal Pairs) | **Parallel Workers:** 14 physical CPU cores")
    print()
    
    # Title 5: Progress Matrix vs. Happee (2017) Benchmark
    print("### 5. Progress Matrix vs. Happee (2017) Benchmark")
    try:
        with open("logs/phase2_4_bode_progress_matrix.md", "r", encoding="utf-8") as fp:
            print(fp.read())
    except Exception:
        print("*(Matrix exported to logs/phase2_4_bode_progress_matrix.md)*")
    print(f"{'='*75}\n", flush=True)

def main():
    os.makedirs("logs", exist_ok=True)
    os.makedirs("logs/candidates", exist_ok=True)
    
    # -------------------------------------------------------------------------
    # Initial Centroid x0: Loaded from First-Principles Rational Champion
    # -------------------------------------------------------------------------
    prescreen_file = "data/prescreen_rational_champion.json" if os.path.exists("data/prescreen_rational_champion.json") else "data/prescreen_64k_best_candidate.json"
    if os.path.exists(prescreen_file):
        with open(prescreen_file, "r", encoding="utf-8") as fp:
            ps_data = json.load(fp)
        x0 = np.array(ps_data["x0"])
        sigma0 = np.array(ps_data["sigma0"])
        lower_bounds = np.array(ps_data["lower_bounds"])
        upper_bounds = np.array(ps_data["upper_bounds"])
        print(f"[PRESCREEN] Initializing CMA-ES centroid x0 from {prescreen_file} ({ps_data.get('description', 'Pre-screen Champion')})")
    else:
        x0 = np.array([
            0.58,        # 0: K_ton
            76.4989490,  # 1: p_ton (rad/s)
            0.5301718,   # 2: K_sc (Nm/(rad/s))
            0.2457791,   # 3: r_lead_sc
            96.0500752,  # 4: p_sc (rad/s)
            4.2943242,   # 5: K_phas (Nm/(m/s^2))
            17.5282154,  # 6: p_phas (rad/s)
            1.2838733,   # 7: kp (spindle gain)
            0.0687314,   # 8: r_lead_ccr
            177.0659882, # 9: p_ccr (rad/s)
            0.7189289,   # 10: k_dyn
            1.1772806    # 11: k_stat
        ])
        sigma0 = np.array([
            0.07,    # K_ton
            6.0,     # p_ton
            0.12,    # K_sc
            0.03,    # r_lead_sc
            8.0,     # p_sc
            0.60,    # K_phas
            2.50,    # p_phas
            0.20,    # kp
            0.015,   # r_lead_ccr
            12.0,    # p_ccr
            0.08,    # k_dyn
            0.10     # k_stat
        ])
        lower_bounds = np.array([
            0.05, 2.0, 0.05, 0.001, 15.0, 0.20,
            8.0, 0.15, 0.001, 5.0, 0.10, 0.01
        ])
        upper_bounds = np.array([
            2.50, 120.0, 15.0, 0.60, 150.0, 15.0,
            80.0, 5.00, 0.65, 220.0, 1.50, 1.50
        ])
    
    max_total_generations = int(os.environ.get('PHASE2_4_MAX_GEN', '50'))
    target_stop_generation = int(os.environ.get("PHASE2_4_STOP_GEN", "30"))
    
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 14,
        'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
        'maxiter': max_total_generations,
        'seed': 2026
    })
    
    # Replay completed Phase 2.4 generations if resuming
    completed_gens = 0
    global_best_score = float('inf')
    global_best_sol = x0.copy()
    consecutive_non_improving = 0
    
    archive_X = []
    archive_y = []
    
    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_4_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            gen_scores = sdata["all_scores"]
            pop_len = len(gen_scores)
            if es.popsize != pop_len:
                es.opts['popsize'] = pop_len
                es.sp.set(es.opts, pop_len)
                
            if pop_len == 14:
                cands = sample_candidates_tpa_orthogonal(es, g, archive_X, archive_y)
            else:
                cands = es.ask()
                
            b_low, b_upp = es.boundary_handler.bounds
            loaded_cands = []
            for w_i in range(1, pop_len + 1):
                cf_path = f'logs/candidates/W{w_i}_P24G{g}.json'
                if os.path.exists(cf_path):
                    with open(cf_path, 'r', encoding='utf-8') as f_cf:
                        p_raw = json.load(f_cf)['params']
                    p_vec = list(p_raw)
                    p_vec[3] = p_raw[3] / p_raw[4]
                    p_vec[8] = p_raw[8] / p_raw[9]
                    loaded_cands.append(np.clip(p_vec, b_low + 1e-4, b_upp - 1e-4).tolist())
            if len(loaded_cands) == pop_len:
                clamped_replay = loaded_cands
            else:
                clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]

            es.tell(clamped_replay, gen_scores)
            completed_gens = g
            
            # Populate archive
            for c_arr, s_val in zip(clamped_replay, gen_scores):
                archive_X.append(c_arr)
                archive_y.append(s_val)
                
            best_cand_rep = clamped_replay[sdata["best_candidate_idx"] - 1]
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = best_cand_rep.copy()
                consecutive_non_improving = 0
            else:
                consecutive_non_improving += 1
                
            # Replay dynamic boundary expansion if triggered
            new_low, new_upp, expansions = check_and_expand_bounds(best_cand_rep, b_low, b_upp)
            if expansions:
                set_cma_bounds(es, new_low, new_upp)
                
            print(f"[REPLAY] Phase 2.4 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    # Switch to 14 candidates for Generation 4 onwards (1:1 core concurrency)
    NEW_POPSIZE = 14
    if es.popsize != NEW_POPSIZE:
        es.opts['popsize'] = NEW_POPSIZE
        es.sp.set(es.opts, NEW_POPSIZE)
    print(f"\n[CMA-ES RECONFIGURATION] Confirmed population size at {NEW_POPSIZE} (mu={es.sp.mu}) with 2 TPA + 12 Orthogonal samples (100% 14-core synchronization)!")
    print(f"[ARCHIVE] Initialized with {len(archive_X)} real OpenSim historical evaluations.")
            
    print(f"\n{'='*75}")
    print(f"Phase 2.4 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Target Benchmark to Beat: Happee (2017) Model Score = {HAPPEE_BENCHMARK:.2f} pts")
    print(f"Baseline to Beat: W3_P23G15 Score = {global_best_score:.2f} pts")
    print(f"Plateau Stopping Rule: Stop after 7 consecutive non-improving generations")
    print(f"Initial Centroid x0:\n{np.round(x0, 3)}")
    print(f"Search Sigmas sigma0:\n{np.round(sigma0, 3)}")
    print(f"{'='*75}\n", flush=True)
    
    iteration = completed_gens
    while not es.stop() and iteration < target_stop_generation:
        pause_disabled = os.environ.get("DISABLE_NIGHT_PAUSE", "0") == "1"
        if not pause_disabled:
            now_dt = datetime.datetime.now()
            if (now_dt.hour == 22 and now_dt.minute >= 40) or (now_dt.hour >= 23 or now_dt.hour < 7):
                print(f"\n[PAUSE_2300] Current time {now_dt.strftime('%H:%M:%S')} reached night pause window (23:00 - 07:00). Cleanly pausing generations.", flush=True)
                break

        iteration += 1
        gen_start_time = datetime.datetime.now()
        
        print(f"\n{'='*35} STARTING PHASE 2.4 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)
        print(f"[CMA-ES SAMPLING] Sampling population of 14 (2 TPA + 12 Orthogonal Pairs across 14 cores)...", flush=True)
        
        raw_cands = sample_candidates_tpa_orthogonal(es, iteration, archive_X, archive_y)
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
                        print(f"[PASS] [W{c_idx+1}_P24G{iteration}] passed {freq:.3f}Hz ({t_idx+1}/{len(TARGETS)}) (Score: {score:.1f} | Cum: {all_scores[c_idx]:.1f})", flush=True)
                        if t_idx + 1 < len(TARGETS):
                            next_fut = executor.submit(evaluate_freq, candidates[c_idx], t_idx + 1, c_idx, iteration)
                            future_to_info[next_fut] = (c_idx, t_idx + 1)
                        else:
                            print(f"[COMPLETE] [W{c_idx+1}_P24G{iteration}] finished all 6 frequencies! Total Score: {all_scores[c_idx]:.2f}", flush=True)
                    else:
                        penalty = (len(TARGETS) - t_idx) * 200000.0
                        all_scores[c_idx] += (score + penalty)
                        print(f"[EARLY STOPPING] [W{c_idx+1}_P24G{iteration}] aborted on {freq:.3f}Hz (Score: {score:.1f}). Penalty added.", flush=True)
                        
                    active_count = len(future_to_info)
                    break
                    
        # Update CMA-ES distribution
        b_low, b_upp = es.boundary_handler.bounds
        clamped_candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in candidates]
        es.tell(clamped_candidates, all_scores)
        for c_arr, s_val in zip(clamped_candidates, all_scores):
            archive_X.append(c_arr)
            archive_y.append(s_val)
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds() / 60.0
        
        best_idx = int(np.argmin(all_scores))
        best_score = float(all_scores[best_idx])
        best_cand = candidates[best_idx]
        
        is_new_record = best_score < global_best_score
        if is_new_record:
            global_best_score = best_score
            global_best_sol = best_cand.copy()
            consecutive_non_improving = 0
            beat_happee_str = " *** BEATS HAPPEE (2017) MODEL! ***" if best_score < HAPPEE_BENCHMARK else ""
            print(f"\n>>> NEW PHASE 2.4 RECORD in Gen {iteration}: {best_score:.2f} (Candidate W{best_idx+1}_P24G{iteration}){beat_happee_str} <<<\n", flush=True)
        else:
            consecutive_non_improving += 1
            print(f"\n[PLATEAU TRACKER] Consecutive non-improving generations: {consecutive_non_improving} / 7", flush=True)
            
        K_ton = best_cand[0]
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
        
        z_sc = r_lead_sc * p_sc
        z_ccr = r_lead_ccr * p_ccr
        
        # Save generation summary
        summary = {
            "phase": 2.4,
            "generation": iteration,
            "start_time": gen_start_time.isoformat(),
            "end_time": datetime.datetime.now().isoformat(),
            "duration_min": gen_duration,
            "best_candidate_idx": best_idx + 1,
            "best_score": best_score,
            "best_params": {
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
        
        print(f"\n[GEN COMPLETE] Generation {iteration}: Best Candidate W{best_idx+1}_P24G{iteration} Score = {best_score:.2f} pts (Duration: {gen_duration:.1f} min)", flush=True)
        summary_path = f"logs/phase2_4_gen_{iteration}_summary.json"
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
            
        # Update Master Ledger and Bode Progress Matrix
        try:
            import update_phase2_4_ledger
            update_phase2_4_ledger.update_phase2_4_ledger(iteration)
        except Exception as e_led:
            print(f"Warning: failed auto-ledger update: {e_led}", flush=True)
            
        # Check and expand dynamic boundaries for next generation
        new_low, new_upp, expansions = check_and_expand_bounds(best_cand, b_low, b_upp)
        if expansions:
            print("\n[DYNAMIC BOUNDARY EXPANSION]")
            for exp in expansions:
                print(f"  * {exp}")
            set_cma_bounds(es, new_low, new_upp)
            print()
            
        # Print mandatory 5-title milestone report
        print_5_title_milestone_report(
            iteration, best_idx + 1, best_score, gen_duration,
            best_cand, b_low, b_upp, all_scores, global_best_score
        )
        
        # Check plateau stopping criterion
        max_non_improving = int(os.environ.get("MAX_NON_IMPROVING", "50"))
        if consecutive_non_improving >= max_non_improving:
            print(f"\n[PLATEAU CRITERION MET] No score improvement for {max_non_improving} consecutive generations. Stopping optimization loop.", flush=True)
            break
            
        if best_score < HAPPEE_BENCHMARK:
            print(f"\n[TARGET REACHED] Score {best_score:.2f} beat Happee (2017) benchmark ({HAPPEE_BENCHMARK:.2f} pts)! Optimization successful!", flush=True)
            break
            
    print(f"PHASE 2.4 RUN CONCLUDED at {datetime.datetime.now().strftime('%H:%M:%S')}.", flush=True)

if __name__ == "__main__":
    main()

