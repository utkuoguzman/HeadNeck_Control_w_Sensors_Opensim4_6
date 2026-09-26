import subprocess
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Best Parameters from user: Kp=66.4, Ki=37.2, Kd=22.8, Gton=13.01, Gsc=2.84, Gphas=5.08, Kp_prop=0.458, K_gamma_dyn=37.1, K_gamma_stat=89.5 (Error: 9025.6)
kp = 66.4
ki = 37.2
kd = 22.8
g_ton = 13.01
g_sc = 2.84
g_phas = 5.08
kp_prop = 0.458
k_gamma_dyn = 37.1
k_gamma_stat = 89.5

targets = [
    (1.124195, 149.430277, -81.089109),
    (0.823652, 106.268348, -85.537188),
    (0.603456, 81.029187, -86.129080),
    (0.442127, 63.821646, -82.694072),
    (0.323929, 50.868070, -77.524752)
]

freqs = []
exp_gains = []
exp_phases = []
sim_gains = []
sim_phases = []

print("Running Bode response simulations using best parameters...")

import sys
import os
sys.path.append(os.getcwd())
import cmaes_worker

for freq, target_gain, target_phase in targets:
    freqs.append(freq)
    exp_gains.append(target_gain)
    exp_phases.append(target_phase)
    
    print(f"Simulating frequency {freq:.2f} Hz...")
    
    # We call run_simulation directly to bypass subprocess overhead and easily extract gain/phase!
    # run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, frequency, worker_name="W-?")
    try:
        gain, phase = cmaes_worker.run_simulation(
            kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat,
            freq, worker_name="Plot-W"
        )
    except Exception as e:
        print("Failed to run simulation for frequency", freq)
        print("Error:", e)
        sys.exit(1)
        
    sim_gains.append(gain)
    sim_phases.append(phase)
    print(f"  Target: Gain={target_gain:.1f}, Phase={target_phase:.1f}")
    print(f"  Sim:    Gain={gain:.1f}, Phase={phase:.1f}")

# Sort by frequency for plotting
sorted_indices = np.argsort(freqs)
freqs = np.array(freqs)[sorted_indices]
exp_gains = np.array(exp_gains)[sorted_indices]
exp_phases = np.array(exp_phases)[sorted_indices]
sim_gains = np.array(sim_gains)[sorted_indices]
sim_phases = np.array(sim_phases)[sorted_indices]

# Ensure phase differences wrap around nicely for visual comparison if they differ by 360 (unlikely here but safe)
for i in range(len(sim_phases)):
    while sim_phases[i] - exp_phases[i] > 180:
        sim_phases[i] -= 360
    while sim_phases[i] - exp_phases[i] < -180:
        sim_phases[i] += 360

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Gain plot
ax1.plot(freqs, exp_gains, 'k-o', linewidth=2, markersize=8, label='Happee 2017 (Experimental)')
ax1.plot(freqs, sim_gains, 'r--s', linewidth=2, markersize=8, label='Optimized Model (CMA-ES)')
ax1.set_ylabel('Gain (Head/Sled Accel Ratio)', fontsize=12)
ax1.set_title('Bode Response of Optimized Head-Neck Model', fontsize=14)
ax1.set_xscale('log')
ax1.grid(True, which="both", ls="--", alpha=0.5)
ax1.legend()

# Phase plot
ax2.plot(freqs, exp_phases, 'k-o', linewidth=2, markersize=8, label='Happee 2017 (Experimental)')
ax2.plot(freqs, sim_phases, 'r--s', linewidth=2, markersize=8, label='Optimized Model (CMA-ES)')
ax2.set_xlabel('Frequency (Hz)', fontsize=12)
ax2.set_ylabel('Phase (deg)', fontsize=12)
ax2.set_xscale('log')
ax2.grid(True, which="both", ls="--", alpha=0.5)

plt.tight_layout()
os.makedirs("logs", exist_ok=True)
plt.savefig("logs/optimized_bode_plot.png", dpi=300)
print("\nSaved Bode plot to logs/optimized_bode_plot.png")
