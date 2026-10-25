import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sig
import os

print("Computing Theoretical Vestibular SCC Bode Plot (Happee 2017 + 13ms delay)...")

freqs_hz = np.logspace(np.log10(0.01), np.log10(20.0), 500)
s = 2j * np.pi * freqs_hz

# C++ parameters from HeadNeckNeuralController.cpp
k_reg = 2.83; T1_reg = 0.0175; T2_reg = 0.0027; Tc_reg = 5.7
k_irr = 27.09; T1_irr = 0.03; T2_irr = 0.0006; Tc_irr = 5.7

# Transfer functions
H_reg = k_reg * s * (s + 1.0/T1_reg) / ((s + 1.0/Tc_reg) * (s + 1.0/T2_reg))
H_irr = k_irr * s * (s + 1.0/T1_irr) / ((s + 1.0/Tc_irr) * (s + 1.0/T2_irr))
H_scc = (H_reg + H_irr) / 2.0

# Add 13ms neural delay (VCR)
delay = 0.013
H_delay = np.exp(-s * delay)
H_reg_total = H_reg * H_delay
H_irr_total = H_irr * H_delay
H_total = H_scc * H_delay

def get_mag_phase(H):
    mag = np.abs(H)
    phase = np.angle(H) * 180.0 / np.pi
    phase = np.unwrap(phase * np.pi / 180.0) * 180.0 / np.pi
    return mag, phase

mag_reg, phase_reg = get_mag_phase(H_reg_total)
mag_irr, phase_irr = get_mag_phase(H_irr_total)
mag_total, phase_total = get_mag_phase(H_total)

fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax_mag.loglog(freqs_hz, mag_reg, 'b--', linewidth=2, label='Regular Afferents + Delay')
ax_mag.loglog(freqs_hz, mag_irr, 'g--', linewidth=2, label='Irregular Afferents + Delay')
ax_mag.loglog(freqs_hz, mag_total, 'r-', linewidth=2, label='Averaged Response + Delay')
ax_mag.axhline(1.0, color='k', linestyle=':', alpha=0.5)
ax_mag.set_ylabel('Gain (deg/s per deg/s)')
ax_mag.grid(True, which='both', ls='--')
ax_mag.legend()
ax_mag.set_title('Vestibular Semicircular Canal Dynamics (Bode Plot)')

ax_phase.semilogx(freqs_hz, phase_reg, 'b--', linewidth=2)
ax_phase.semilogx(freqs_hz, phase_irr, 'g--', linewidth=2)
ax_phase.semilogx(freqs_hz, phase_total, 'r-', linewidth=2)
ax_phase.axhline(0.0, color='k', linestyle=':', alpha=0.5)
ax_phase.set_ylabel('Phase (deg)')
ax_phase.set_xlabel('Frequency (Hz)')
ax_phase.grid(True, which='both', ls='--')

plt.tight_layout()
os.makedirs("logs", exist_ok=True)
plt.savefig("logs/bode_plot_analytical.png")
print("Saved Bode plot to logs/bode_plot_analytical.png")
