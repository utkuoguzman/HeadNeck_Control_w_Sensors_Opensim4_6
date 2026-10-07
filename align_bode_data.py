import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Paths
data_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\extracted_data\harpee_2017\B8EC_BodePlots_of_3Measurements"

files = {
    'GlobalTrans_Gain': 'GlobalTranslationalHeadVelocity_BodeGain.csv',
    'GlobalTrans_Phase': 'GlobalTranslationalHeadVelocity_BodePhase.csv',
    'Angular_Gain': 'AngularHeadVelocity_BodeGain.csv',
    'Angular_Phase': 'AngularHeadVelocity_BodePhase.csv',
    'RelTrans_Gain': 'RelativeTranslationalHeadVelocity_BodeGain.csv',
    'RelTrans_Phase': 'RelativeTranslationalHeadVelocity_BodePhase.csv'
}

# Load data
raw_data = {}
for key, fname in files.items():
    path = os.path.join(data_dir, fname)
    df = pd.read_csv(path, sep=',', engine='python')
    df.columns = [col.strip() for col in df.columns]
    raw_data[key] = df

# Find common frequencies using log10 average
log_x_list = []
for key, df in raw_data.items():
    # Ensure sorted by x just in case
    df = df.sort_values(by='x')
    raw_data[key] = df
    log_x_list.append(np.log10(df['x'].values))

log_x_matrix = np.array(log_x_list)
common_log_x = np.mean(log_x_matrix, axis=0)
common_x = 10**common_log_x

print(f"Common Target Frequencies: {common_x}")

# Output DataFrame
out_df = pd.DataFrame({'Frequency_Hz': common_x})

aligned_data = {}

# Interpolate each dataset
for key, df in raw_data.items():
    x = df['x'].values
    y = df['y'].values
    
    log_x = np.log10(x)
    
    if 'Gain' in key:
        log_y = np.log10(y)
        # Use cubic spline in log-log space for gains
        cs = CubicSpline(log_x, log_y, bc_type='not-a-knot')
        interp_log_y = cs(common_log_x)
        interp_y = 10**interp_log_y
    else:
        # Use cubic spline in semi-log space for phase
        cs = CubicSpline(log_x, y, bc_type='not-a-knot')
        interp_y = cs(common_log_x)
        
    out_df[key] = interp_y
    aligned_data[key] = interp_y

out_csv_path = os.path.join(data_dir, 'aligned_B8EC_data.csv')
out_df.to_csv(out_csv_path, index=False)
print(f"Saved aligned data to {out_csv_path}")

# Plotting
fig, axes = plt.subplots(2, 3, figsize=(16, 10))
fig.suptitle('Bode Plots: Original Data vs. Aligned Cubic Interpolation', fontsize=16)

plot_mapping = [
    ('GlobalTrans_Gain', axes[0,0], 'Global Translational Vel Gain', True),
    ('Angular_Gain', axes[0,1], 'Angular Head Vel Gain', True),
    ('RelTrans_Gain', axes[0,2], 'Relative Translational Vel Gain', True),
    ('GlobalTrans_Phase', axes[1,0], 'Global Translational Vel Phase', False),
    ('Angular_Phase', axes[1,1], 'Angular Head Vel Phase', False),
    ('RelTrans_Phase', axes[1,2], 'Relative Translational Vel Phase', False)
]

for key, ax, title, is_gain in plot_mapping:
    df = raw_data[key]
    
    # Continuous line for interpolation to show the spline fit
    x_dense_log = np.linspace(np.min(log_x_matrix), np.max(log_x_matrix), 100)
    
    if is_gain:
        cs = CubicSpline(np.log10(df['x']), np.log10(df['y']), bc_type='not-a-knot')
        y_dense = 10**cs(x_dense_log)
    else:
        cs = CubicSpline(np.log10(df['x']), df['y'], bc_type='not-a-knot')
        y_dense = cs(x_dense_log)

    # Plot continuous spline curve
    ax.plot(10**x_dense_log, y_dense, 'r-', alpha=0.5, label='Cubic Spline Curve')
    
    # Plot original points
    ax.plot(df['x'], df['y'], 'bo', markersize=8, label='Original Picked Points')
    
    # Plot aligned points
    ax.plot(common_x, aligned_data[key], 'rX', markersize=12, label='Aligned Target Points')
    
    ax.set_title(title)
    ax.set_xscale('log')
    ax.set_xlabel('Frequency [Hz]')
    
    if is_gain:
        ax.set_yscale('log')
        ax.set_ylabel('Gain')
    else:
        ax.set_ylabel('Phase [°]')
        
    ax.legend()
    ax.grid(True, which="both", ls="--", alpha=0.5)

plt.tight_layout()
out_plot_path = os.path.join(data_dir, 'aligned_B8EC_plots.png')
plt.savefig(out_plot_path, dpi=300)
print(f"Saved plots to {out_plot_path}")

# Copy to gemini brain directory for viewing
import shutil
shutil.copy(out_plot_path, r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\aligned_B8EC_plots.png")
