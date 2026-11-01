import json, glob

with open('logs/phase2_3_gen_15_summary.json', 'r', encoding='utf-8') as fp:
    s15 = json.load(fp)

bode15 = {}
for bf in sorted(glob.glob('logs/candidates/W3_P23G15_*Hz.json')):
    with open(bf, 'r', encoding='utf-8') as bfp:
        d = json.load(bfp)
        bode15[f"{d['freq']:.3f}"] = {
            'gain_db': round(d['gain_db_diff'], 1),
            'phase_err': round(d['phase_err_deg'], 1),
            'score': round(d['score'], 1)
        }

ledger = {
    'phase': 2.4,
    'description': 'Targeted Basin Optimization Centered on 21k Champion W3_P23G15',
    'benchmark_score': 2803.88,
    'generations': [
        {
            'generation': 0,
            'candidate': 'W3_P23G15',
            'score': round(s15['best_score'], 2),
            'duration_min': round(s15.get('duration_min', 91.8), 1),
            'params': {k: round(v, 4) for k, v in s15.get('best_params', {}).items()},
            'bode': bode15
        }
    ]
}

with open('logs/phase2_4_master_ledger.json', 'w', encoding='utf-8') as out:
    json.dump(ledger, out, indent=2)

import sys
sys.path.append('scratch')
from update_phase2_4_ledger import export_matrix
export_matrix()
print('Initialized Phase 2.4 master ledger and matrix with Gen 0 (W3_P23G15) baseline!')

