import json
import glob
import pandas as pd
import numpy as np

def inspect_candidate(pattern):
    files = sorted(glob.glob(f"logs/candidates/{pattern}_*Hz.json"))
    data = []
    for f in files:
        with open(f, 'r', encoding='utf-8') as fp:
            data.append(json.load(fp))
    return sorted(data, key=lambda x: x['freq'])

df = pd.read_csv('extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv')
hap_elems, hap_score = [], 0.0
for _, r in df.iterrows():
    e_g, h_g = r['Exp_Gain_deg_per_m'], r['HappeeModel_Gain_deg_per_m']
    e_p, h_p = r['Exp_Phase_deg'], r['HappeeModel_Phase_deg']
    g_db = 20.0 * np.log10(h_g / e_g)
    g_loss = ((g_db / 1.0) ** 2) * 100.0
    p_diff = (h_p - e_p + 180.0) % 360.0 - 180.0
    p_loss = ((p_diff / 10.0) ** 2) * 100.0
    hap_score += (g_loss + p_loss)
    hap_elems.append(f"{g_db:+.1f} dB<br>/ {abs(p_diff):.1f}°")

def get_row(cand_data):
    elems, tot = [], 0.0
    for d in cand_data:
        tot += d['score']
        elems.append(f"{d['gain_db_diff']:+.1f} dB<br>/ {d['phase_err_deg']:.1f}°")
    return elems, tot

freqs = [f"{r['Frequency_Hz']:.3f} Hz" for _, r in df.iterrows()]
header = "| Campaign Milestone | " + " | ".join(freqs) + " | Total Score |"
sep = "| :--- | " + " | ".join([":---:"] * len(freqs)) + " | :---: |"

print(header)
print(sep)
print(f"| **Happee (2017) Benchmark** | " + " | ".join(hap_elems) + f" | **`{hap_score:.2f}`** |")
for name, tag in [
    ('W10_P23G1', 'Gen 1 (`W10`)'),
    ('W17_P23G2', 'Gen 2 (`W17`)'),
    ('W17_P23G3', 'Gen 3 (`W17`)'),
    ('W16_P23G4', 'Gen 4 (`W16`)'),
    ('W9_P23G5', 'Gen 5 (`W9`)'),
    ('W4_P23G6', 'Gen 6 (`W4`)')
]:
    elems, tot = get_row(inspect_candidate(name))
    print(f"| **Phase 2.3 {tag}** | " + " | ".join(elems) + f" | **`{tot:.2f}`** |")

print("\n--- Detailed W4_P23G6 Breakdown ---")
for d in inspect_candidate('W4_P23G6'):
    print(f"Freq: {d['freq']:.3f} Hz | Gain: {d['sim_gain']:.2f} (tgt: {d['target_gain']:.2f}, {d['gain_db_diff']:+.2f} dB) | Phase: {d['sim_phase']:.2f} (tgt: {d['target_phase']:.2f}, err: {d['phase_err_deg']:.2f} deg) | Score: {d['score']:.1f}")
