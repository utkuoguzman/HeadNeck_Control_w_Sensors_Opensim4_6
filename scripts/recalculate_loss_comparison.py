import math
import glob
import json

targets = {
    0.31: {'gain': 0.893, 'phase': 4.5},
    0.78: {'gain': 0.840, 'phase': -98.9},
    1.11: {'gain': 0.814, 'phase': -112.6},
    2.48: {'gain': 0.655, 'phase': -163.7},
    5.52: {'gain': 0.509, 'phase': -194.6},
    7.95: {'gain': 0.443, 'phase': -247.7}
}

happee_model = {
    0.31: {'sim_gain': 0.893 * 1.33, 'sim_phase': 4.5 - 45.0},
    0.78: {'sim_gain': 0.840 * 1.05, 'sim_phase': -98.9 - 6.0},
    1.11: {'sim_gain': 0.814 * 1.05, 'sim_phase': -112.6 - 6.0},
    2.48: {'sim_gain': 0.655 * 1.05, 'sim_phase': -163.7 - 6.0},
    5.52: {'sim_gain': 0.509 * 1.10, 'sim_phase': -194.6 - 13.0},
    7.95: {'sim_gain': 0.443 * 1.10, 'sim_phase': -247.7 - 13.0}
}

def calc_scores(data):
    old_tot, log_tot, db_tot = 0.0, 0.0, 0.0
    per_freq = {}
    for f in [0.31, 0.78, 1.11, 2.48, 5.52, 7.95]:
        sg = data[f]['sim_gain']
        sp = data[f]['sim_phase']
        tg = targets[f]['gain']
        tp = targets[f]['phase']
        
        # Phase error
        p_diff = abs((sp - tp + 180.0) % 360.0 - 180.0)
        p_loss = p_diff ** 2
        
        # Old linear gain loss
        g_err_pct = abs(sg - tg) / tg * 100.0
        old_g_loss = g_err_pct ** 2
        old_s = old_g_loss + p_loss
        
        # Log-natural loss (w=10000)
        log_diff = math.log(sg / tg)
        log_g_loss = (log_diff ** 2) * 10000.0
        log_s = log_g_loss + p_loss
        
        # dB loss (1 dB = 100 pts)
        db_diff = 20.0 * math.log10(sg / tg)
        db_g_loss = (db_diff ** 2) * 100.0
        db_s = db_g_loss + p_loss
        
        old_tot += old_s
        log_tot += log_s
        db_tot += db_s
        per_freq[f] = {
            'gain_err_pct': g_err_pct,
            'db_diff': db_diff,
            'phase_diff': p_diff,
            'old': old_s,
            'log': log_s,
            'db': db_s
        }
    return old_tot, log_tot, db_tot, per_freq

def load_worker(worker_name):
    data = {}
    for f in [0.31, 0.78, 1.11, 2.48, 5.52, 7.95]:
        files = glob.glob(f"logs/candidates/{worker_name}_{f:.2f}Hz.json")
        if not files:
            return None
        with open(files[0], "r", encoding="utf-8") as fp:
            d = json.load(fp)
            data[f] = {'sim_gain': d['sim_gain'], 'sim_phase': d['sim_phase']}
    return data

print("=" * 60)
print("HAPPEE (2017) BENCHMARK RECALCULATION")
print("=" * 60)
old_h, log_h, db_h, pf_h = calc_scores(happee_model)
print(f"Old Linear Loss Score : {old_h:9.2f}")
print(f"Natural Log Loss Score: {log_h:9.2f}")
print(f"Decibel (dB) Loss Score: {db_h:9.2f}")
print("\nPer-frequency breakdown for Happee:")
for f, v in pf_h.items():
    print(f"  {f:4.2f} Hz | Gain Err: {v['gain_err_pct']:4.1f}% ({v['db_diff']:+5.2f} dB) | Phase Err: {v['phase_diff']:4.1f} deg | Old: {v['old']:7.1f} | Log: {v['log']:7.1f} | dB: {v['db']:7.1f}")

# Check champions
champions = [
    ("W16_P21G3", "Phase 2.1 Gen 3 (Initial Lead Champion)"),
    ("W17_P21G11", "Phase 2.1 Gen 11 (15ms Delay Breakthrough)"),
    ("W11_P21G12", "Phase 2.1 Gen 12 (Mid-Frequency Tightening)"),
    ("W12_P21G13", "Phase 2.1 Gen 13 (All-Time Champion)")
]

for w, desc in champions:
    data = load_worker(w)
    if data:
        old, l, db, pf = calc_scores(data)
        print("\n" + "=" * 60)
        print(f"CHAMPION: {w} - {desc}")
        print("=" * 60)
        print(f"Old Linear Loss Score : {old:9.2f}")
        print(f"Natural Log Loss Score: {l:9.2f}")
        print(f"Decibel (dB) Loss Score: {db:9.2f}")
        print("Per-frequency breakdown:")
        for f, v in pf.items():
            print(f"  {f:4.2f} Hz | Gain Err: {v['gain_err_pct']:4.1f}% ({v['db_diff']:+5.2f} dB) | Phase Err: {v['phase_diff']:4.1f} deg | Old: {v['old']:7.1f} | Log: {v['log']:7.1f} | dB: {v['db']:7.1f}")

