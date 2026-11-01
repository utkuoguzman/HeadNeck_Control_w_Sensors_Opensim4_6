import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

data_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\extracted_data\harpee_2017\B8EC_BodePlots_of_1Measurement_with_HappeesModel"

files = {
    'Exp_Gain': 'angular velocity_GainPlot_experiment.csv',
    'Model_Gain': 'angular velocity_GainPlot_HappesModel.csv',
    'Exp_Phase': 'angular velocity_PhasePlot_experiment.csv',
    'Model_Phase': 'angular velocity_PhasePlot_HappesModel.csv'
}

raw_data = {}
log_x_list = []
for key, fname in files.items():
    path = os.path.join(data_dir, fname)
    df = pd.read_csv(path, sep=',', engine='python')
    df.columns = [col.strip() for col in df.columns]
    df = df.sort_values(by='x')
    raw_data[key] = df
    log_x_list.append(np.log10(df['x'].values))

log_x_matrix = np.array(log_x_list)
common_log_x = np.mean(log_x_matrix, axis=0)
common_x = 10**common_log_x

print(f"Calculated Common Frequencies [Hz]:\n{common_x}")

out_df = pd.DataFrame({
    'Frequency_Hz': common_x
})

aligned_data = {}

for key, df in raw_data.items():
    x = df['x'].values
    y = df['y'].values
    log_x = np.log10(x)
    
    if 'Gain' in key:
        log_y = np.log10(y)
        cs = CubicSpline(log_x, log_y, bc_type='not-a-knot')
        interp_log_y = cs(common_log_x)
        interp_y = 10**interp_log_y
    else:
        cs = CubicSpline(log_x, y, bc_type='not-a-knot')
        interp_y = cs(common_log_x)
        
    out_df[key] = interp_y
    aligned_data[key] = interp_y

# Rename columns clearly
out_df = out_df.rename(columns={
    'Exp_Gain': 'Exp_Gain_deg_per_m',
    'Exp_Phase': 'Exp_Phase_deg',
    'Model_Gain': 'HappeeModel_Gain_deg_per_m',
    'Model_Phase': 'HappeeModel_Phase_deg'
})

# Reorder columns logically
cols = [
    'Frequency_Hz',
    'Exp_Gain_deg_per_m',
    'Exp_Phase_deg',
    'HappeeModel_Gain_deg_per_m',
    'HappeeModel_Phase_deg'
]
out_df = out_df[cols]

out_csv_path = os.path.join(data_dir, 'aligned_angular_velocity_data.csv')
out_df.to_csv(out_csv_path, index=False)
print(f"\nSaved aligned CSV to: {out_csv_path}")
print("\n--- Aligned Data Table ---")
print(out_df.to_string(index=False))

# Calculate Happee Model Score in Decibel loss vs Experiment on this new aligned dataset!
# Isotropic Equal-Corridor Decibel Loss:
# (20 * log10(G_model / G_exp) / 1.0 dB)^2 * 100 + (delta_phi / 10 deg)^2 * 100
scores = []
print("\n--- Happee Model vs Experiment Errors on Aligned Data ---")
for i, row in out_df.iterrows():
    f = row['Frequency_Hz']
    g_exp = row['Exp_Gain_deg_per_m']
    g_mod = row['HappeeModel_Gain_deg_per_m']
    p_exp = row['Exp_Phase_deg']
    p_mod = row['HappeeModel_Phase_deg']
    
    gain_db_diff = 20.0 * np.log10(g_mod / g_exp)
    gain_loss = ((gain_db_diff / 1.0)**2) * 100.0
    
    phase_diff = (p_mod - p_exp + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0)**2) * 100.0
    
    total = gain_loss + phase_loss
    scores.append(total)
    print(f"{f:5.2f} Hz | Gain: {gain_db_diff:+6.2f} dB (Loss: {gain_loss:7.1f}) | Phase: {phase_diff:+6.1f} deg (Loss: {phase_loss:7.1f}) | Total: {total:7.1f}")

total_happee_score = sum(scores)
print(f"\n>>> Total Happee (2017) Model Score on New Aligned Data: {total_happee_score:.2f} <<<")

# Generate High-Resolution Validation Plot
fig, axes = plt.subplots(1, 2, figsize=(14, 6))
fig.suptitle('Happee (2017) Angular Head Velocity: Experiment vs. Model (Aligned Data)', fontsize=14, fontweight='bold')

# Gain Plot
ax_g = axes[0]
ax_g.set_title('Bode Magnitude (Gain)')
ax_g.set_xscale('log')
ax_g.set_yscale('log')
ax_g.set_xlabel('Frequency [Hz]')
ax_g.set_ylabel('Gain [°/m]')
ax_g.grid(True, which="both", ls="--", alpha=0.5)

# Raw points
ax_g.plot(raw_data['Exp_Gain']['x'], raw_data['Exp_Gain']['y'], 'bo', label='Exp (Picked Points)')
ax_g.plot(raw_data['Model_Gain']['x'], raw_data['Model_Gain']['y'], 'ro', label='Model (Picked Points)')

# Aligned points & Spline
dense_log_x = np.linspace(np.min(log_x_matrix), np.max(log_x_matrix), 100)
dense_x = 10**dense_log_x

cs_exp_g = CubicSpline(np.log10(raw_data['Exp_Gain']['x']), np.log10(raw_data['Exp_Gain']['y']))
cs_mod_g = CubicSpline(np.log10(raw_data['Model_Gain']['x']), np.log10(raw_data['Model_Gain']['y']))
ax_g.plot(dense_x, 10**cs_exp_g(dense_log_x), 'b-', alpha=0.6, label='Exp Spline')
ax_g.plot(dense_x, 10**cs_mod_g(dense_log_x), 'r-', alpha=0.6, label='Model Spline')

ax_g.plot(out_df['Frequency_Hz'], out_df['Exp_Gain_deg_per_m'], 'bX', markersize=10, label='Aligned Exp Target')
ax_g.plot(out_df['Frequency_Hz'], out_df['HappeeModel_Gain_deg_per_m'], 'rX', markersize=10, label='Aligned Model Target')
ax_g.legend()

# Phase Plot
ax_p = axes[1]
ax_p.set_title('Bode Phase')
ax_p.set_xscale('log')
ax_p.set_xlabel('Frequency [Hz]')
ax_p.set_ylabel('Phase [°]')
ax_p.grid(True, which="both", ls="--", alpha=0.5)

# Raw points
ax_p.plot(raw_data['Exp_Phase']['x'], raw_data['Exp_Phase']['y'], 'bo', label='Exp (Picked Points)')
ax_p.plot(raw_data['Model_Phase']['x'], raw_data['Model_Phase']['y'], 'ro', label='Model (Picked Points)')

cs_exp_p = CubicSpline(np.log10(raw_data['Exp_Phase']['x']), raw_data['Exp_Phase']['y'])
cs_mod_p = CubicSpline(np.log10(raw_data['Model_Phase']['x']), raw_data['Model_Phase']['y'])
ax_p.plot(dense_x, cs_exp_p(dense_log_x), 'b-', alpha=0.6, label='Exp Spline')
ax_p.plot(dense_x, cs_mod_p(dense_log_x), 'r-', alpha=0.6, label='Model Spline')

ax_p.plot(out_df['Frequency_Hz'], out_df['Exp_Phase_deg'], 'bX', markersize=10, label='Aligned Exp Target')
ax_p.plot(out_df['Frequency_Hz'], out_df['HappeeModel_Phase_deg'], 'rX', markersize=10, label='Aligned Model Target')
ax_p.legend()

plt.tight_layout()
out_plot_path = os.path.join(data_dir, 'aligned_angular_velocity_plots.png')
plt.savefig(out_plot_path, dpi=300)
print(f"Saved plot to: {out_plot_path}")

# Also copy to gemini brain directory
import shutil
gemini_plot_path = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\aligned_angular_velocity_plots.png"
shutil.copy(out_plot_path, gemini_plot_path)
print(f"Copied plot to artifacts directory: {gemini_plot_path}")

