import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

gain_df = pd.read_csv(r"extracted_data\harpee_2017\B1EC_gain_and_phase\frequency_vs_gain.csv")
phase_df = pd.read_csv(r"extracted_data\harpee_2017\B1EC_gain_and_phase\frequency_vs_phase.csv")

# Clean column names
gain_df.columns = [c.strip() for c in gain_df.columns]
phase_df.columns = [c.strip() for c in phase_df.columns]

# Sort by frequency
gain_df = gain_df.sort_values("frequency")
phase_df = phase_df.sort_values("frequency")

# Create interpolation functions
gain_interp = interp1d(gain_df["frequency"], gain_df["gain"], kind="linear")
phase_interp = interp1d(phase_df["frequency"], phase_df["phase"], kind="linear")

# Define 5 log-spaced target frequencies between min and max bounds
# Using log spacing because Bode plots are log-spaced
min_freq = max(gain_df["frequency"].min(), phase_df["frequency"].min())
max_freq = min(gain_df["frequency"].max(), phase_df["frequency"].max())

target_freqs = np.geomspace(min_freq, max_freq, 5)

# Generate interpolated data
target_gains = gain_interp(target_freqs)
target_phases = phase_interp(target_freqs)

# Create combined dataframe
combined_df = pd.DataFrame({
    "frequency": target_freqs,
    "gain": target_gains,
    "phase": target_phases
})

combined_df.to_csv(r"extracted_data\harpee_2017\B1EC_gain_and_phase\frequency_vs_gain_phase.csv", index=False)
print("Interpolated Targets:")
print(combined_df.to_string(index=False))

