import json
import math
import numpy as np

# Load multisine trajectory
with open("logs/adaptive_continuous_trajectory.json", "r") as f:
    data = json.load(f)

t_arr = np.array(data["time"])
pitch_arr = np.array(data["head_pitch"]) # in degrees

# Discard initial transient (t < 10.0s)
mask = t_arr >= 10.0
t_fit = t_arr[mask]
y_fit = pitch_arr[mask]

TARGETS = [
    (0.324,  86.55,   -7.03),
    (0.783, 236.93,  -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

K = len(TARGETS)
freqs = np.array([t[0] for t in TARGETS])
phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])
base_amps = np.array([0.0215 * (0.835 / f)**2 for f in freqs])
scale_factor = 0.35 / math.sqrt(K)
amps = base_amps * scale_factor # input displacement amplitude in meters

# Linear harmonic regression: y(t) = a + b*t + sum_k [ S_k * sin(omega_k * (t-1.5) + phi_k) + C_k * cos(omega_k * (t-1.5) + phi_k) ]
# So that input is simply A_k * sin(...)
N_samples = len(t_fit)
cols = [np.ones(N_samples), t_fit - 10.0]

for k in range(K):
    f = freqs[k]
    phi = phases[k]
    t_rel = t_fit - 1.5
    omega = 2.0 * math.pi * f
    cols.append(np.sin(omega * t_rel + phi))
    cols.append(np.cos(omega * t_rel + phi))

A_mat = np.column_stack(cols)
# Solve least squares
coeffs, residuals, rank, s = np.linalg.lstsq(A_mat, y_fit, rcond=None)

print("=" * 85)
print("MULTISINE BODE EXTRACTION & PHASE 2.4 ISOTROPIC EQUAL-CORRIDOR SCORING")
print("=" * 85)
print(f"{'Freq(Hz)':>8s} | {'SimGain(deg/m)':>14s} | {'TgtGain':>8s} | {'GainErr(dB)':>11s} | {'SimPhase':>8s} | {'TgtPhase':>8s} | {'PhaseErr':>8s} | {'Loss(pts)':>9s}")
print("-" * 85)

total_loss = 0.0

for k in range(K):
    f, tgt_gain, tgt_phase = TARGETS[k]
    S_k = coeffs[2 + 2*k]
    C_k = coeffs[2 + 2*k + 1]
    
    # Output pitch amplitude in degrees
    amp_out_deg = math.sqrt(S_k**2 + C_k**2)
    # Gain in deg / m
    sim_gain = amp_out_deg / amps[k]
    # Phase in degrees
    sim_phase_rad = math.atan2(C_k, S_k)
    sim_phase = math.degrees(sim_phase_rad)
    # Wrap to target phase range
    phase_diff = (sim_phase - tgt_phase + 180.0) % 360.0 - 180.0
    sim_phase_unwrapped = tgt_phase + phase_diff

    # Phase 2.2 / Phase 2.4 Isotropic Equal-Corridor Loss
    gain_db_diff = 20.0 * math.log10(sim_gain / tgt_gain)
    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
    freq_loss = gain_loss + phase_loss
    total_loss += freq_loss

    print(f"{f:8.3f} | {sim_gain:14.2f} | {tgt_gain:8.2f} | {gain_db_diff:+10.2f}dB | {sim_phase_unwrapped:8.2f}° | {tgt_phase:8.2f}° | {phase_diff:+7.2f}° | {freq_loss:9.1f}")

print("-" * 85)
print(f"TOTAL PHASE 2.4 ISOTROPIC EQUAL-CORRIDOR SCORE: {total_loss:,.2f} pts")
print("=" * 85)

