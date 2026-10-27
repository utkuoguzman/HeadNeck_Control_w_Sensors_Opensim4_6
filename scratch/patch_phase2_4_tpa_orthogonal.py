import os
import re

file_path = "scripts/run_phase2_4_campaign.py"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Add cdist import if not present
if "from scipy.spatial.distance import cdist" not in content:
    content = content.replace("from concurrent.futures import ThreadPoolExecutor, as_completed",
                              "from concurrent.futures import ThreadPoolExecutor, as_completed\nfrom scipy.spatial.distance import cdist")

# 2. Add SimpleGP and sample_candidates_tpa_orthogonal classes/functions before check_and_expand_bounds
helper_code = '''
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
    
    # 1. TPA direction in z-space (standard normal space)
    d0 = np.array(X[0]) - mean
    z_tpa = (B.T @ d0) / (D * sigma)
    z_tpa_norm = np.linalg.norm(z_tpa)
    if z_tpa_norm > 1e-10:
        u_tpa = z_tpa / z_tpa_norm
    else:
        u_tpa = np.zeros(12)
        u_tpa[0] = 1.0

    # 2. Check if Direction-Pair GP pre-screening should be active (iteration >= 8 and archive >= 80)
    use_gp = (iteration >= 8 and archive_X is not None and len(archive_X) >= 80)
    
    if use_gp:
        # Fit local GP on nearest 80 historical models
        dists = np.linalg.norm(np.array(archive_X) - mean, axis=1)
        idx = np.argsort(dists)[:80]
        local_X = [archive_X[i] for i in idx]
        local_y = [archive_y[i] for i in idx]
        
        l_scale = max(0.1, float(np.median(dists[idx])))
        gp = SimpleGP(length_scale=l_scale)
        gp.fit(local_X, local_y)
        
        # Sample candidate orthogonal bases and evaluate direction utility
        best_score = float('inf')
        best_Q = None
        for _ in range(12):
            G = np.random.randn(12, 6)
            G = G - np.outer(u_tpa, u_tpa @ G)
            Q, _ = np.linalg.qr(G)
            
            test_pts = []
            for k in range(6):
                zk = Q[:, k] * np.sqrt(12.0)
                dx = sigma * (B @ (D * zk))
                test_pts.append(mean + dx)
                test_pts.append(mean - dx)
            mu, sig = gp.predict(test_pts)
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
        dx = sigma * (B @ (D * zk))
        X[2 + 2*k] = (mean + dx).tolist()
        X[3 + 2*k] = (mean - dx).tolist()
        
    return X
'''

if "class SimpleGP" not in content:
    content = content.replace("def set_cma_bounds(es, lower, upper):", helper_code + "\ndef set_cma_bounds(es, lower, upper):")

# 3. Update es initialization to use CMAAdaptSigmaTPA
old_es_init = """    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'maxiter': max_total_generations,
        'seed': 2026
    })"""

new_es_init = """    es = cma.CMAEvolutionStrategy(x0, 1.0, {
        'CMA_stds': sigma0,
        'bounds': [lower_bounds, upper_bounds],
        'popsize': 18,
        'AdaptSigma': cma.sigma_adaptation.CMAAdaptSigmaTPA,
        'maxiter': max_total_generations,
        'seed': 2026
    })"""

content = content.replace(old_es_init, new_es_init)

# 4. Update replay loop to populate archive and switch to popsize 14
old_replay_block = """    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_4_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            cands = es.ask()
            b_low, b_upp = es.boundary_handler.bounds
            clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]
            es.tell(clamped_replay, sdata["all_scores"])
            completed_gens = g
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = cands[sdata["best_candidate_idx"] - 1]
                consecutive_non_improving = 0
            else:
                consecutive_non_improving += 1
            print(f"[REPLAY] Phase 2.4 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break"""

new_replay_block = """    archive_X = []
    archive_y = []
    
    for g in range(1, max_total_generations + 1):
        summary_file = f"logs/phase2_4_gen_{g}_summary.json"
        if os.path.exists(summary_file):
            with open(summary_file, "r", encoding="utf-8") as f:
                sdata = json.load(f)
            cands = es.ask()
            b_low, b_upp = es.boundary_handler.bounds
            clamped_replay = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in cands]
            es.tell(clamped_replay, sdata["all_scores"])
            completed_gens = g
            
            # Populate archive
            for c_arr, s_val in zip(clamped_replay, sdata["all_scores"]):
                archive_X.append(c_arr)
                archive_y.append(s_val)
                
            if sdata["best_score"] < global_best_score:
                global_best_score = sdata["best_score"]
                global_best_sol = cands[sdata["best_candidate_idx"] - 1]
                consecutive_non_improving = 0
            else:
                consecutive_non_improving += 1
            print(f"[REPLAY] Phase 2.4 Generation {g} replayed. Best score: {sdata['best_score']:.2f}")
        else:
            break
            
    # Switch to 14 candidates for Generation 3 onwards (1:1 core concurrency)
    NEW_POPSIZE = 14
    es.opts['popsize'] = NEW_POPSIZE
    es.sp.set(es.opts, NEW_POPSIZE)
    print(f"\\n[CMA-ES RECONFIGURATION] Switched population size to {NEW_POPSIZE} (mu={es.sp.mu}) with 2 TPA + 12 Orthogonal samples (100% 14-core synchronization)!")
    print(f"[ARCHIVE] Initialized with {len(archive_X)} real OpenSim historical evaluations.")"""

content = content.replace(old_replay_block, new_replay_block)

# 5. Update generation sampling loop to use sample_candidates_tpa_orthogonal
old_sample_block = """        print(f"\\n{'='*35} STARTING PHASE 2.4 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)
        print(f"[CMA-ES SAMPLING] Sampling population of 18 directly from N(mu, C) around 21k basin...", flush=True)
        
        raw_cands = es.ask()
        b_low, b_upp = es.boundary_handler.bounds
        candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in raw_cands]
        all_scores = [0.0] * len(candidates)"""

new_sample_block = """        print(f"\\n{'='*35} STARTING PHASE 2.4 GENERATION {iteration} of {target_stop_generation} at {gen_start_time.strftime('%H:%M:%S')} {'='*35}", flush=True)
        print(f"[CMA-ES SAMPLING] Sampling population of 14 (2 TPA + 12 Orthogonal Pairs across 14 cores)...", flush=True)
        
        raw_cands = sample_candidates_tpa_orthogonal(es, iteration, archive_X, archive_y)
        b_low, b_upp = es.boundary_handler.bounds
        candidates = [np.clip(c, b_low + 1e-4, b_upp - 1e-4).tolist() for c in raw_cands]
        all_scores = [0.0] * len(candidates)"""

content = content.replace(old_sample_block, new_sample_block)

# 6. Append to archive in generation loop after es.tell
old_tell_block = """        es.tell(clamped_candidates, all_scores)
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds() / 60.0"""

new_tell_block = """        es.tell(clamped_candidates, all_scores)
        for c_arr, s_val in zip(clamped_candidates, all_scores):
            archive_X.append(c_arr)
            archive_y.append(s_val)
        gen_duration = (datetime.datetime.now() - gen_start_time).total_seconds() / 60.0"""

content = content.replace(old_tell_block, new_tell_block)

# 7. Update report printout
content = content.replace(
    'print(f"* **Population Size:** 18 candidates | **Parallel Workers:** 14 physical CPU cores")',
    'print(f"* **Population Size:** {len(all_scores)} candidates (2 TPA + 12 Orthogonal Pairs) | **Parallel Workers:** 14 physical CPU cores")'
)

with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)

print("Patch applied successfully to scripts/run_phase2_4_campaign.py!")

