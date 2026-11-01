import glob
import json
import os
import sys
import numpy as np
from scipy.stats import spearmanr, pearsonr
from scipy.spatial.distance import cdist

sys.path.append(os.getcwd())

# Load all Phase 2.2 and Phase 2.3 completed candidates
files = glob.glob('logs/candidates/*_*.json')
cand_names = sorted(list(set(os.path.basename(f).split('_')[0] + '_' + os.path.basename(f).split('_')[1] for f in files if '_' in os.path.basename(f))))

dataset = []
for c in cand_names:
    if 'P22' not in c and 'P23' not in c:
        continue
    p_file = f'logs/candidates/{c}.json'
    if not os.path.exists(p_file): continue
    try:
        p_data = json.load(open(p_file, 'r', encoding='utf-8'))
        params = p_data.get('params')
        if not params or len(params) != 12: continue
    except: continue
    
    freq_files = glob.glob(f'logs/candidates/{c}_*Hz.json')
    if len(freq_files) == 6:
        try:
            scores = [json.load(open(ff, 'r', encoding='utf-8'))['score'] for ff in freq_files]
            total_score = sum(scores)
            if total_score < 250000.0:
                dataset.append((c, params, total_score))
        except: continue

X_raw = np.array([d[1] for d in dataset])
y_osim = np.array([d[2] for d in dataset])
log_y = np.log(y_osim)

# Log-transform positive parameters spanning multiple orders of magnitude
# [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat]
log_X = np.log(np.maximum(1e-4, X_raw))
X_combined = np.hstack([X_raw, log_X])
mu_X = np.mean(X_combined, axis=0)
std_X = np.std(X_combined, axis=0) + 1e-8
X_norm = (X_combined - mu_X) / std_X

N = len(dataset)
np.random.seed(42)
indices = np.random.permutation(N)
k_folds = 5
fold_size = N // k_folds

def evaluate_log_model(name, predict_fn):
    oof_preds = np.zeros(N)
    for fold in range(k_folds):
        val_idx = indices[fold * fold_size : (fold + 1) * fold_size] if fold < k_folds - 1 else indices[fold * fold_size :]
        train_idx = np.setdiff1d(indices, val_idx)
        
        X_tr, y_tr = X_norm[train_idx], log_y[train_idx]
        X_va = X_norm[val_idx]
        
        pred_log_val = predict_fn(X_tr, y_tr, X_va)
        oof_preds[val_idx] = np.exp(pred_log_val)
        
    test_mae = np.mean(np.abs(y_osim - oof_preds))
    test_rho, _ = spearmanr(y_osim, oof_preds)
    test_r, _ = pearsonr(y_osim, oof_preds)
    
    full_pred = np.exp(predict_fn(X_norm, log_y, X_norm))
    train_rho, _ = spearmanr(y_osim, full_pred)
    train_mae = np.mean(np.abs(y_osim - full_pred))
    
    print(f"\nModel: {name}")
    print(f"  Train: Spearman rho = {train_rho:.4f} | MAE = {train_mae:.1f} pts")
    print(f"  TEST (Out-of-Sample CV): Spearman rho = {test_rho:.4f} | Pearson r = {test_r:.4f} | MAE = {test_mae:.1f} pts")
    print(f"  Generalization Gap (Train vs Test rho): {abs(train_rho - test_rho):.4f}")

def ridge_predict(X_tr, y_tr, X_va, alpha=15.0):
    D = X_tr.shape[1]
    w = np.linalg.solve(X_tr.T @ X_tr + alpha * np.eye(D), X_tr.T @ y_tr)
    return X_va @ w

evaluate_log_model("Direct Log-Ridge (alpha=15.0)", ridge_predict)

def rbf_krr_predict(X_tr, y_tr, X_va, gamma=0.04, alpha=1.0):
    K_tr = np.exp(- gamma * cdist(X_tr, X_tr, 'sqeuclidean'))
    K_va = np.exp(- gamma * cdist(X_va, X_tr, 'sqeuclidean'))
    N_tr = X_tr.shape[0]
    weights = np.linalg.solve(K_tr + alpha * np.eye(N_tr), y_tr)
    return K_va @ weights

evaluate_log_model("Direct Log-Gaussian Process / Kernel Ridge (gamma=0.04, alpha=1.0)", rbf_krr_predict)

