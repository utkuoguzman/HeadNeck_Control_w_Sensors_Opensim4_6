import json
import glob
import pandas as pd
import numpy as np

def inspect_candidate(pattern, label):
    files = sorted(glob.glob(f"logs/candidates/{pattern}_*Hz.json"))
    data = []
    for f in files:
        with open(f, 'r', encoding='utf-8') as fp:
            d = json.load(fp)
            data.append(d)
    data = sorted(data, key=lambda x: x['freq'])
    return data

def print_comparison():
    df = pd.read_csv('extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv')
    g1_data = inspect_candidate("W10_P23G1", "Gen 1")
    g2_data = inspect_candidate("W17_P23G2", "Gen 2")
    g3_data = inspect_candidate("W17_P23G3", "Gen 3")
    
    print("\n======================================================= COMPREHENSIVE BODE GAIN & PHASE ERROR COMPARISON =======================================================")
    print(f"{'Freq (Hz)':<10} | {'Happee Gain Err':<16} | {'Gen 1 Gain Err':<15} | {'Gen 2 Gain Err':<15} | {'Gen 3 Gain Err':<15} || {'Happee Ph Err':<14} | {'Gen 1 Ph Err':<13} | {'Gen 2 Ph Err':<13} | {'Gen 3 Ph Err':<13}")
    print("-" * 145)
    
    for i, r in df.iterrows():
        freq = r['Frequency_Hz']
        e_g = r['Exp_Gain_deg_per_m']
        h_g = r['HappeeModel_Gain_deg_per_m']
        e_p = r['Exp_Phase_deg']
        h_p = r['HappeeModel_Phase_deg']
        
        hap_g_db = 20.0 * np.log10(h_g / e_g)
        hap_p_diff = abs((h_p - e_p + 180.0) % 360.0 - 180.0)
        
        g1_g_db = g1_data[i]['gain_db_diff'] if i < len(g1_data) else 0.0
        g1_p_diff = g1_data[i]['phase_err_deg'] if i < len(g1_data) else 0.0
        
        g2_g_db = g2_data[i]['gain_db_diff'] if i < len(g2_data) else 0.0
        g2_p_diff = g2_data[i]['phase_err_deg'] if i < len(g2_data) else 0.0
        
        g3_g_db = g3_data[i]['gain_db_diff'] if i < len(g3_data) else 0.0
        g3_p_diff = g3_data[i]['phase_err_deg'] if i < len(g3_data) else 0.0
        
        print(f"{freq:<10.3f} | {hap_g_db:<+16.2f} | {g1_g_db:<+15.2f} | {g2_g_db:<+15.2f} | {g3_g_db:<+15.2f} || {hap_p_diff:<14.2f} | {g1_p_diff:<13.2f} | {g2_p_diff:<13.2f} | {g3_p_diff:<13.2f}")
    print("-" * 145)

print_comparison()

