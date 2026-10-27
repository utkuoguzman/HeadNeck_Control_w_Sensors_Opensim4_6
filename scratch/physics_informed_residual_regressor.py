import glob
import json
import os
import sys
import math
import cmath
import numpy as np

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

class PhysicsInformedResidualRegressor:
    def __init__(self, alpha=20.0):
        self.alpha = alpha
        self.mu_X = None
        self.std_X = None
        self.w = None
        self.is_trained = False
        
    def fit_from_logs(self, candidate_dir="logs/candidates"):
        files = glob.glob(os.path.join(candidate_dir, "*_*.json"))
        cand_names = sorted(list(set(os.path.basename(f).split('_')[0] + '_' + os.path.basename(f).split('_')[1] for f in files if '_' in os.path.basename(f))))
        
        dataset = []
        for c in cand_names:
            if 'P22' not in c and 'P23' not in c:
                continue
            p_file = os.path.join(candidate_dir, f"{c}.json")
            if not os.path.exists(p_file): continue
            try:
                p_data = json.load(open(p_file, 'r', encoding='utf-8'))
                params = p_data.get('params')
                if not params or len(params) != 12: continue
            except: continue
            
            freq_files = glob.glob(os.path.join(candidate_dir, f"{c}_*Hz.json"))
            if len(freq_files) == 6:
                try:
                    scores = [json.load(open(ff, 'r', encoding='utf-8'))['score'] for ff in freq_files]
                    total_score = sum(scores)
                    if total_score < 250000.0:
                        dataset.append((params, total_score))
                except: continue
                
        print(f"[RESIDUAL REGRESSOR] Loaded {len(dataset)} verified canonical OpenSim simulations.", flush=True)
        X_raw = np.array([d[0] for d in dataset])
        y_osim = np.array([d[1] for d in dataset])
        
        y_prior = np.array([surrogate_score(x) for x in X_raw])
        log_y_osim = np.log(y_osim)
        log_y_prior = np.log(y_prior)
        delta = log_y_osim - log_y_prior
        
        self.mu_X = np.mean(X_raw, axis=0)
        self.std_X = np.std(X_raw, axis=0) + 1e-8
        X_norm = (X_raw - self.mu_X) / self.std_X
        
        X_design = np.column_stack([np.ones(len(X_norm)), X_norm])
        D = X_design.shape[1]
        
        reg_matrix = self.alpha * np.eye(D)
        reg_matrix[0, 0] = 0.0
        self.w = np.linalg.solve(X_design.T @ X_design + reg_matrix, X_design.T @ delta)
        self.is_trained = True
        
        pred_delta = X_design @ self.w
        pred_y = np.exp(log_y_prior + pred_delta)
        mae = np.mean(np.abs(y_osim - pred_y))
        print(f"[RESIDUAL REGRESSOR] Trained successfully. L2 Ridge Alpha={self.alpha}. In-sample MAE={mae:.1f} pts.", flush=True)

    def predict_batch(self, candidates):
        if not self.is_trained:
            raise RuntimeError("Regressor is not trained yet!")
            
        cands_arr = np.asarray(candidates)
        N = len(cands_arr)
        
        priors = np.array([surrogate_score(c) for c in cands_arr])
        log_priors = np.log(np.maximum(1.0, priors))
        
        X_norm = (cands_arr - self.mu_X) / self.std_X
        X_design = np.column_stack([np.ones(N), X_norm])
        
        pred_delta = X_design @ self.w
        pred_log_score = log_priors + pred_delta
        return np.exp(pred_log_score)

_regressor_instance = None

def get_regressor():
    global _regressor_instance
    if _regressor_instance is None:
        _regressor_instance = PhysicsInformedResidualRegressor(alpha=20.0)
        _regressor_instance.fit_from_logs()
    return _regressor_instance

def predict_opensim_scores(candidates):
    reg = get_regressor()
    return reg.predict_batch(candidates)

