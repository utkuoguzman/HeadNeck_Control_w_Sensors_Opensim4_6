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

# Phase 2.4 Optimization Campaign: 16-Parameter CMA-ES Deep Basin Refinement
# Starting from W4_P24G3 Champion (12,415.56 pts) + Second Phasic Otolith Pole (p_phas2) + Pitch Task PID (Kp, Kd, Ki)
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
    "p_phas", "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat",
    "Kp_task_pitch", "Kd_task_pitch", "Ki_task_pitch"
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
    0.05,   # k_stat
    0.0,    # Kp_task_pitch
    0.0,    # Kd_task_pitch
    0.1     # Ki_task_pitch
])

# Physical ceilings: a dynamic expansion may never exceed these
PHYSICAL_UPPER_LIMITS = np.array([
    3.0,    # K_ton
    300.0,  # p_ton
    3.0,    # K_sc
    1.0,    # r_lead_sc  (zero must stay below pole -> lead)
    300.0,  # p_sc
    35.0,   # K_phas
    120.0,  # p_phas
    5.0,    # kp
    1.0,    # r_lead_ccr
    400.0,  # p_ccr
    2.0,    # k_dyn
    2.0,    # k_stat
    100.0,  # Kp_task_pitch
    50.0,   # Kd_task_pitch
    150.0   # Ki_task_pitch
])

def evaluate_freq(candidate, target_idx, w_idx, gen_idx):
    freq, t_gain, t_phase = TARGETS[target_idx]
    worker_label = f"W{w_idx+1}_P24G{gen_idx}"
    
    # Check if this frequency evaluation is already cached on disk for this exact candidate
    res_file = os.path.join("logs", "candidates", f"{worker_label}_{freq:.2f}Hz.json")
    if os.path.exists(res_file):
        try:
            with open(res_file, "r", encoding="utf-8") as f_res:
                d_res = json.load(f_res)
            if "score" in d_res and "params" in d_res and len(d_res["params"]) == len(candidate):
                # Compare K_ton, p_ton, p_sc, Kp_task_pitch (indices 0, 1, 4, 12) to verify same candidate
                if (abs(d_res["params"][0] - candidate[0]) < 1e-6 and
                    abs(d_res["params"][1] - candidate[1]) < 1e-5 and
                    abs(d_res["params"][4] - candidate[4]) < 1e-5 and
                    abs(d_res["params"][12] - candidate[12]) < 1e-5):
                    return float(d_res["score"])
        except Exception:
            pass

    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_sim_time = delay + ramp_duration + (n_eval * period)
    dynamic_timeout = max(1200, int(target_sim_time * 240 * 3.5))
    
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


def evaluate_candidate(candidate, w_idx, gen_idx):
    """
    Evaluates all 6 canonical frequencies for one candidate sequentially, spawning a fresh
    subprocess per frequency so OS reclaims 100% of OpenSim C++ memory after each frequency
    (preventing the 1.8 GB/worker RAM accumulation of single-process ALL mode) while keeping
    all 14 physical cores 100% utilized without inter-frequency barrier stalls.
    """
    worker_label = f"W{w_idx+1}_P24G{gen_idx}"
    total_score = 0.0
    for t_idx, (f_val, _, _) in enumerate(TARGETS):
        f_score = evaluate_freq(candidate, t_idx, w_idx, gen_idx)
        if f_score < 150000.0:
            total_score += f_score
            print(f"[PASS] [{worker_label}] passed {f_val:.3f}Hz ({t_idx+1}/{len(TARGETS)}) (Score: {f_score:.1f} | Cum: {total_score:.1f})", flush=True)
        else:
            early_stop_penalty = (len(TARGETS) - t_idx) * 200000.0
            total_score += (f_score + early_stop_penalty)
            print(f"[EARLY STOPPING] [{worker_label}] aborted on {f_val:.3f}Hz (Score: {f_score:.1f}). Penalty added.", flush=True)
            break
    else:
        print(f"[COMPLETE] [{worker_label}] finished all 6 frequencies! Total Score: {total_score:.2f}", flush=True)
    return total_score


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

_ORTH_CYCLE_Q = None
_ORTH_CYCLE_PTR = 0

def sample_candidates_tpa_orthogonal(es, iteration, archive_X=None, archive_y=None):
    """
    28 Candidates for 14 Physical Cores (2 Full Waves of 14 Cores):
      - Candidates 0 & 1: 1 TPA Mirror Pair (along the mean evolution path p_c, preserved as pycma Solution objects)
      - Candidates 2 to 27: 13 Mutually Orthogonal Antipodal Pairs in the subspace perpendicular to p_c,
        covering 14/16 (88%) of the search space every generation and cycling through the full (N-1)-dimensional
        orthogonal basis, registered in es.sent_solutions so BoundTransform genotype-phenotype mapping is exact.
      - Starting when archive >= 80: Uses Direction-Pair GP Sieve to screen 12 candidate orthogonal bases.
    """
    global _ORTH_CYCLE_Q, _ORTH_CYCLE_PTR
    X = es.ask()
    n_pairs = (len(X) - 2) // 2
    mean = np.array(es.mean)
    n_dim = len(mean)
    sigma = es.sigma
    B = es.sm.B
    D = es.sm.D
    stds = np.asarray(es.sigma_vec.scaling, dtype=float) * np.ones(n_dim)  # per-parameter CMA_stds
    
    # 1. Exact un-repaired TPA genotype direction in z-space (standard normal space)
    d0 = (np.array(es.ary[0]) - mean) if (hasattr(es, 'ary') and es.ary is not None and len(es.ary) > 0) else (np.array(X[0]) - mean)
    z_tpa = (B.T @ (d0 / stds)) / (D * sigma)
    z_tpa_norm = np.linalg.norm(z_tpa)
    if z_tpa_norm > 1e-10:
        u_tpa = z_tpa / z_tpa_norm
    else:
        u_tpa = np.zeros(n_dim)
        u_tpa[0] = 1.0

    # 2. Check if Direction-Pair GP pre-screening should be active (archive >= 80)
    use_gp = (archive_X is not None and len(archive_X) >= 80)
    
    if use_gp:
        # Fit local GP on nearest 80 historical models in stds-normalized space
        X_arch_norm = (np.array(archive_X) - mean) / stds
        dists = np.linalg.norm(X_arch_norm, axis=1)
        idx = np.argsort(dists)[:80]
        local_X_norm = X_arch_norm[idx]
        # Winsorize crash penalties (>200,000) so GP variance is not distorted by early-stop penalties
        local_y = np.minimum(np.array([archive_y[i] for i in idx], dtype=float), 200000.0)
        
        l_scale = max(0.1, float(np.median(dists[idx])))
        gp = SimpleGP(length_scale=l_scale)
        gp.fit(local_X_norm, local_y)
        
        # Sample candidate orthogonal bases and evaluate direction utility via Pairwise-Minimum LCB
        best_score = float('inf')
        best_Q = None
        for _ in range(12):
            G = np.random.randn(n_dim, n_pairs)
            G = G - np.outer(u_tpa, u_tpa @ G)
            Q, _ = np.linalg.qr(G)
            
            test_pts_norm = []
            for k in range(n_pairs):
                zk = Q[:, k] * np.sqrt(float(n_dim))
                dx_norm = sigma * (B @ (D * zk))
                test_pts_norm.append(dx_norm)
                test_pts_norm.append(-dx_norm)
            mu, sig = gp.predict(test_pts_norm)
            lcb = mu - 0.5 * sig
            pair_mins = [min(lcb[2 * k], lcb[2 * k + 1]) for k in range(n_pairs)]
            acq = float(np.sum(pair_mins))
            if acq < best_score:
                best_score = acq
                best_Q = Q
        Q_chosen = best_Q
        print(f"[GP PRE-SCREEN] Screened 12 candidate orthogonal bases via Pairwise-Min GP (archive size: {len(archive_X)}). Best {n_pairs}-pair min LCB sum: {best_score:.1f}", flush=True)
    else:
        # Full (N-1)-dimensional block-orthogonal cycling in subspace perpendicular to u_tpa
        if _ORTH_CYCLE_Q is None or _ORTH_CYCLE_Q.shape[0] != n_dim or _ORTH_CYCLE_PTR >= n_dim - 1:
            G_full = np.random.randn(n_dim, n_dim - 1)
            G_full = G_full - np.outer(u_tpa, u_tpa @ G_full)
            _ORTH_CYCLE_Q, _ = np.linalg.qr(G_full)
            _ORTH_CYCLE_PTR = 0
        cols = [(_ORTH_CYCLE_PTR + i) % (n_dim - 1) for i in range(n_pairs)]
        _ORTH_CYCLE_PTR += n_pairs
        G = _ORTH_CYCLE_Q[:, cols]
        G = G - np.outer(u_tpa, u_tpa @ G)
        Q_chosen, _ = np.linalg.qr(G)

    # 3. Construct the n_pairs orthogonal pairs and register their exact genotypes in es.sent_solutions
    for k in range(n_pairs):
        qk = Q_chosen[:, k]
        r_len = np.random.chisquare(n_dim)**0.5
        zk = qk * r_len
        dx = sigma * stds * (B @ (D * zk))
        for idx, geno_vec in [(2 + 2*k, mean + dx), (3 + 2*k, mean - dx)]:
            pheno_vec = es.gp.pheno(geno_vec, copy=True, into_bounds=es.boundary_handler.repair)
            es.sent_solutions.insert(pheno_vec, geno=geno_vec, iteration=es.countiter)
            X[idx] = pheno_vec
        
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
        print(f"* **Basin Exploration:** 14 distinct perturbations evaluated in parallel across all 6 canonical frequencies.")
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
    for i, name in enumerate(PARAM_NAMES[:len(best_cand)]):
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
    # Initial Centroid x0: Loaded from 16-Parameter Rational Champion
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
            0.55, 78.0, 0.15, 0.18, 80.0, 5.0,
            20.0, 0.30, 0.055, 185.0, 0.70, 0.95,
            2.50, 0.25, 37.0
        ])
        sigma0 = np.array([
            0.12, 10.0, 0.04, 0.04, 12.0, 0.75,
            4.0, 0.06, 0.015, 18.0, 0.12, 0.12,
            1.20, 0.20, 5.0
        ])
        lower_bounds = np.array([
            0.01, 5.0, 0.01, 0.001, 15.0, 0.20,
            1.0, 0.03, 0.001, 20.0, 0.05, 0.05,
            0.0, 0.0, 0.5
        ])
        upper_bounds = np.array([
            3.0, 150.0, 1.5, 0.60, 160.0, 10.0,
            65.0, 2.0, 0.50, 240.0, 1.50, 1.50,
            30.0, 12.0, 65.0
        ])
    
    max_total_generations = int(os.environ.get('PHASE2_4_MAX_GEN', '50'))
    target_stop_generation = int(os.environ.get("PHASE2_4_STOP_GEN", "30"))
    
    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 28,
        'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
        'maxiter': max_total_generations,
        'seed': 2026
    })
    
    # Replay completed Phase 2.4 generations if resuming with matching dimension
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
            
            b_low, b_upp = es.boundary_handler.bounds
            if "all_candidates" in sdata and len(sdata["all_candidates"]) == pop_len and len(sdata["all_candidates"][0]) == len(x0):
                loaded_cands = [list(c) for c in sdata["all_candidates"]]
            else:
                loaded_cands = []
                for w_i in range(1, pop_len + 1):
                    cf_path = f'logs/candidates/W{w_i}_P24G{g}.json'
                    if os.path.exists(cf_path):
                        with open(cf_path, 'r', encoding='utf-8') as f_cf:
                            p_raw = json.load(f_cf)['params']
                        if len(p_raw) != len(x0):
                            continue
                        p_vec = list(p_raw)
                        p_vec[3] = p_raw[3] / p_raw[4]
                        p_vec[8] = p_raw[8] / p_raw[9]
                        loaded_cands.append(np.clip(p_vec, b_low + 1e-4, b_upp - 1e-4).tolist())
            if len(loaded_cands) != pop_len:
                break

            if es.popsize != pop_len:
                es.opts['popsize'] = pop_len
                es.sp.set(es.opts, pop_len)
                
            if pop_len in (14, 28):
                raw_replay = sample_candidates_tpa_orthogonal(es, g, archive_X, archive_y)
                for idx in range(2, pop_len):
                    pheno_vec = np.asarray(loaded_cands[idx], dtype=float)
                    geno_vec = es.gp.geno(pheno_vec, from_bounds=es.boundary_handler.inverse, copy=True)
                    es.sent_solutions.insert(pheno_vec, geno=geno_vec, iteration=es.countiter)
                    raw_replay[idx] = pheno_vec
            else:
                raw_replay = es.ask()
                
            clamped_replay = loaded_cands
            es.tell(raw_replay, gen_scores)
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
            
    NEW_POPSIZE = 28
    if es.popsize != NEW_POPSIZE:
        es.opts['popsize'] = NEW_POPSIZE
        es.sp.set(es.opts, NEW_POPSIZE)
    print(f"\n[CMA-ES RECONFIGURATION] Confirmed population size at {NEW_POPSIZE} (mu={es.sp.mu}) with 2 TPA + 26 Orthogonal samples across {len(x0)} parameters (2 x 14-core waves)!")
    print(f"[ARCHIVE] Initialized with {len(archive_X)} real OpenSim historical evaluations.")
            
    print(f"\n{'='*75}")
    print(f"Phase 2.4 Initialized. Starting from Generation {completed_gens + 1} up to Generation {target_stop_generation}")
    print(f"Target Benchmark to Beat: Happee (2017) Model Score = {HAPPEE_BENCHMARK:.2f} pts")
    print(f"Baseline to Beat: W4_P24G3 Score = {global_best_score:.2f} pts")
    print(f"Initial Centroid x0 ({len(x0)}D):\n{np.round(x0, 3)}")
    print(f"Search Sigmas sigma0 ({len(sigma0)}D):\n{np.round(sigma0, 3)}")
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
        print(f"[CMA-ES SAMPLING] Sampling population of {es.popsize} (2 TPA + {es.popsize-2} Orthogonal Samples across 14 cores)...", flush=True)
        
        raw_cands = sample_candidates_tpa_orthogonal(es, iteration, archive_X, archive_y)
        b_low, b_upp = es.boundary_handler.bounds
        
        # If resuming an interrupted generation whose 16D candidate files already exist on disk, reuse them
        existing_cands = []
        for w_i in range(1, es.popsize + 1):
            cf_path = f"logs/candidates/W{w_i}_P24G{iteration}.json"
            if not os.path.exists(cf_path):
                cf_path = f"logs/candidates/W{w_i}_P24G{iteration}_2.45Hz.json"
            if os.path.exists(cf_path):
                try:
                    with open(cf_path, "r", encoding="utf-8") as f_cf:
                        p_raw = json.load(f_cf)["params"]
                    if len(p_raw) == len(x0):
                        p_vec = list(p_raw)
                        p_vec[3] = p_raw[3] / p_raw[4]
                        p_vec[8] = p_raw[8] / p_raw[9]
                        existing_cands.append(p_vec)
                except Exception:
                    pass
        if len(existing_cands) == es.popsize:
            print(f"[RESUME] Found all {es.popsize} existing 16D candidate files for Generation {iteration}. Resuming from cached frequency evaluations...", flush=True)
            for idx in range(2, es.popsize):
                pheno_vec = np.asarray(existing_cands[idx], dtype=float)
                geno_vec = es.gp.geno(pheno_vec, from_bounds=es.boundary_handler.inverse, copy=True)
                es.sent_solutions.insert(pheno_vec, geno=geno_vec, iteration=es.countiter)
                raw_cands[idx] = pheno_vec

        candidates = [np.asarray(c, dtype=float).tolist() for c in raw_cands]
        all_scores = [0.0] * len(candidates)
        
        # Max concurrency: 14 physical cores (each frequency runs in a clean subprocess so OS reclaims C++ memory)
        with ThreadPoolExecutor(max_workers=14) as executor:
            future_to_cidx = {
                executor.submit(evaluate_candidate, candidates[c_idx], c_idx, iteration): c_idx
                for c_idx in range(len(candidates))
            }
            for fut in as_completed(future_to_cidx):
                c_idx = future_to_cidx[fut]
                all_scores[c_idx] = fut.result()
                    
        # Update CMA-ES distribution using raw_cands (preserving pycma Solution objects & exact genotypes)
        es.tell(raw_cands, all_scores)
        clamped_candidates = candidates
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
        if len(best_cand) == 15:
            r_lead_phas = -1.0
            kp_task_pitch = best_cand[12]
            kd_task_pitch = best_cand[13]
            ki_task_pitch = best_cand[14]
        else:
            r_lead_phas = best_cand[12] if len(best_cand) > 12 else -1.0
            kp_task_pitch = best_cand[13] if len(best_cand) > 13 else 0.0
            kd_task_pitch = best_cand[14] if len(best_cand) > 14 else 0.0
            ki_task_pitch = best_cand[15] if len(best_cand) > 15 else FIXED_KI
        
        z_sc = r_lead_sc * p_sc
        z_ccr = r_lead_ccr * p_ccr
        z_phas = r_lead_phas * p_phas if 0.0 < r_lead_phas <= 1.0 else -1.0
        
        # Save generation summary
        pass_scores = [float(s) for s in all_scores if s < 150000.0]
        try:
            cond_num = float(np.max(es.D) / max(1e-12, np.min(es.D))) if hasattr(es, "D") and es.D is not None else 1.0
        except Exception:
            cond_num = 1.0
        cmaes_state = {
            "sigma_global": round(float(es.sigma), 5),
            "condition_number": round(cond_num, 3),
            "n_passed_all_6": len(pass_scores),
            "popsize": len(all_scores),
            "pop_median_score": round(float(np.median(all_scores)), 2),
            "pop_mean_pass_score": round(float(np.mean(pass_scores)), 2) if pass_scores else None,
            "mean_vector": {name: round(float(val), 4) for name, val in zip(PARAM_NAMES, es.mean)},
            "effective_stds": {name: round(float(es.sigma * std_val), 4) for name, std_val in zip(PARAM_NAMES, es.stds)}
        }
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
                "k_stat": float(k_stat),
                "Kp_task_pitch": float(kp_task_pitch),
                "Kd_task_pitch": float(kd_task_pitch),
                "Ki_task_pitch": float(ki_task_pitch)
            },
            "cmaes_state": cmaes_state,
            "all_scores": [float(s) for s in all_scores],
            "all_candidates": clamped_candidates
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
