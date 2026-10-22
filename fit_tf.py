import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Frequencies in Hz and rad/s
freqs = np.array([0.31, 0.78, 1.11, 2.48, 5.52, 7.95])
w = 2 * np.pi * freqs

# Best parameters from good_parameters.txt
G_ton = np.array([5.08, 26.39, 23.39, 82.34, 48.26, 31.61])
G_sc = np.array([4.12, 23.96, 5.26, 27.94, 22.55, 6.57])
G_phas = np.array([6.96, 2.13, 7.06, 5.68, 8.15, 7.54])

def tf_magnitude(w, K, z, p):
    # |H(jw)| = K * sqrt(w^2 + z^2) / sqrt(w^2 + p^2)
    return K * np.sqrt(w**2 + z**2) / np.sqrt(w**2 + p**2)

def fit_pathway(name, w, mags):
    # Bounds: K > 0, z > 0, p > 0 to ensure minimum phase and stability
    popt, _ = curve_fit(tf_magnitude, w, mags, p0=[np.mean(mags), 1.0, 1.0], bounds=([0.01, 0.01, 0.01], [1000, 100, 100]))
    K, z, p = popt
    
    # Calculate R-squared
    residuals = mags - tf_magnitude(w, K, z, p)
    ss_res = np.sum(residuals**2)
    ss_tot = np.sum((mags - np.mean(mags))**2)
    r_squared = 1 - (ss_res / ss_tot)
    
    print(f"--- {name} ---")
    print(f"K = {K:.4f}, z = {z:.4f}, p = {p:.4f}")
    print(f"R^2 = {r_squared:.4f}")
    
    return K, z, p

K_ton, z_ton, p_ton = fit_pathway("G_ton", w, G_ton)
K_sc, z_sc, p_sc = fit_pathway("G_sc", w, G_sc)
K_phas, z_phas, p_phas = fit_pathway("G_phas", w, G_phas)

# We also need initial scalar values for proprioception
# Average of best:
kp_p_avg = np.mean([1.52, 0.41, 0.23, 1.28, 3.32, 1.83])
k_dyn_avg = np.mean([41.30, 40.10, 42.90, 16.60, 62.60, 25.30])
k_stat_avg = np.mean([126.70, 113.80, 90.10, 83.50, 133.10, 93.30])

print("\n--- Proprioception (Averages) ---")
print(f"kp_prop = {kp_p_avg:.4f}")
print(f"k_gamma_dyn = {k_dyn_avg:.4f}")
print(f"k_gamma_stat = {k_stat_avg:.4f}")

os.makedirs("logs", exist_ok=True)
with open("logs/tf_initialization.txt", "w") as f:
    f.write(f"K_ton={K_ton:.4f}, z_ton={z_ton:.4f}, p_ton={p_ton:.4f}\n")
    f.write(f"K_sc={K_sc:.4f}, z_sc={z_sc:.4f}, p_sc={p_sc:.4f}\n")
    f.write(f"K_phas={K_phas:.4f}, z_phas={z_phas:.4f}, p_phas={p_phas:.4f}\n")
    f.write(f"kp_prop={kp_p_avg:.4f}, k_gamma_dyn={k_dyn_avg:.4f}, k_gamma_stat={k_stat_avg:.4f}\n")
