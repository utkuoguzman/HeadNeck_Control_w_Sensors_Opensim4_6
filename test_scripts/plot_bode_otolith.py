import numpy as np
import matplotlib.pyplot as plt
import os

print("Computing Theoretical Vestibular Otolith Bode Plot...")

freqs_hz = np.logspace(np.log10(0.05), np.log10(20.0), 500)
s = 2j * np.pi * freqs_hz

# Typical Otolith Afferent Transfer Function parameters (e.g. Fernandez & Goldberg 1976, Jamali)
# Regular afferents encode mostly static head tilt and low-frequency translation
# Irregular afferents show significant high-frequency gain enhancement and phase lead

# Simplified linear approximation of Otolith dynamics
tau_m = 0.016  # Macular mechanics low-pass (~10-16 ms)
tau_a = 0.1    # Irregular afferent lead time constant

H_reg = 1.0 / (1.0 + tau_m * s)
# Irregular afferents act as fractional differentiators; approximated here by a lead-lag
H_irr = 10.0 * (1.0 + tau_a * s) / (1.0 + tau_m * s)

# VCR neural delay (approx 13ms)
delay = 0.013
H_delay = np.exp(-s * delay)

H_reg_total = H_reg * H_delay
H_irr_total = H_irr * H_delay

def get_mag_phase(H):
    mag = np.abs(H)
    phase = np.angle(H) * 180.0 / np.pi
    phase = np.unwrap(phase * np.pi / 180.0) * 180.0 / np.pi
    return mag, phase

mag_reg, phase_reg = get_mag_phase(H_reg_total)
mag_irr, phase_irr = get_mag_phase(H_irr_total)

fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax_mag.loglog(freqs_hz, mag_reg, 'b--', linewidth=2, label='Regular Otolith Afferents + Delay')
ax_mag.loglog(freqs_hz, mag_irr, 'g--', linewidth=2, label='Irregular Otolith Afferents + Delay')
ax_mag.axhline(1.0, color='k', linestyle=':', alpha=0.5)
ax_mag.set_ylabel('Gain (normalized)')
ax_mag.grid(True, which='both', ls='--')
ax_mag.legend()
ax_mag.set_title('Vestibular Otolith Dynamics (Bode Plot)')

ax_phase.semilogx(freqs_hz, phase_reg, 'b--', linewidth=2)
ax_phase.semilogx(freqs_hz, phase_irr, 'g--', linewidth=2)
ax_phase.axhline(0.0, color='k', linestyle=':', alpha=0.5)
ax_phase.set_ylabel('Phase (deg)')
ax_phase.set_xlabel('Frequency (Hz)')
ax_phase.grid(True, which='both', ls='--')

plt.tight_layout()
os.makedirs("logs", exist_ok=True)
plt.savefig("logs/bode_plot_otolith.png")
print("Saved Bode plot to logs/bode_plot_otolith.png")
