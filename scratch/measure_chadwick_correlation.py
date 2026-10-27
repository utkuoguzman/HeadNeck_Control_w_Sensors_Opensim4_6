import json
import sys
import os
sys.path.append(os.getcwd())
from scipy.stats import spearmanr, pearsonr
from scratch.cervical_chadwick_blana_simulator import evaluate_chadwick_blana_candidate

with open('logs/phase2_3_master_ledger.json', 'r', encoding='utf-8') as f:
    ledger = json.load(f)

gens = ledger['generations']
print(f"Evaluating all {len(gens)} Generation Winners through Chadwick-Blana...\n")

sim_scores = []
osim_scores = []

for g in gens:
    p = g['params']
    cand = [
        p['r_ton'], p['p_ton'], p['K_sc'], p['r_lead_sc'], p['p_sc'],
        p['K_phas'], p['p_phas'], p['kp'], p['r_lead_ccr'], p['p_ccr'],
        p['k_dyn'], p['k_stat']
    ]
    score, _ = evaluate_chadwick_blana_candidate(cand)
    sim_scores.append(score)
    osim_scores.append(g['score'])
    print(f"Gen {g['generation']:>2} ({g['candidate']:<10}): OpenSim = {g['score']:>9.2f} pts | Chadwick-Blana = {score:>9.2f} pts")

spearman_corr, sp_p = spearmanr(osim_scores, sim_scores)
pearson_corr, pe_p = pearsonr(osim_scores, sim_scores)

print("\n" + "="*70)
print(f"Sample Size N = {len(gens)} Generation Champions")
print(f"Spearman Rank Correlation (rho): {spearman_corr:.4f} (p-value: {sp_p:.4e})")
print(f"Pearson Linear Correlation (r)  : {pearson_corr:.4f} (p-value: {pe_p:.4e})")
print("="*70)
