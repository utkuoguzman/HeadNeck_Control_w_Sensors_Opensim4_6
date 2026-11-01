import numpy as np
import scipy.optimize as opt
import pandas as pd
import cmath
import math

# Load the canonical Happee aligned data
df = pd.read_csv("extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv")
freqs = df['Frequency_Hz'].values
exp_gain = df['Exp_Gain_deg_per_m'].values
exp_phase = df['Exp_Phase_deg'].values
happee_gain = df['HappeeModel_Gain_deg_per_m'].values
happee_phase = df['HappeeModel_Phase_deg'].values

print("Loaded 6 Frequencies:")
for f, g, p in zip(freqs, exp_gain, exp_phase):
    print(f"  {f:.3f} Hz -> Target Gain: {g:.1f} deg/m, Phase: {p:.1f} deg")

# Biomechanical Plant Parameters
# Head mass m = 4.5 kg, COM height h = 0.15 m, Moment of Inertia I = 0.025 kg m^2 + m h^2 = 0.126 kg m^2
I_eff = 0.126
m = 4.5
h = 0.15
mgh = m * 9.81 * h # 6.62 Nm/rad (destabilizing gravity)
K_passive = 12.0 # Nm/rad (passive cervical stiffness)
B_passive = 0.80 # Nms/rad (passive damping)

tau_act = 0.030 # 30 ms muscle activation dynamics
delay_vcr = 0.010 # 10 ms
delay_ccr = 0.015 # 15 ms

def transfer_function(params, freq):
    # params: [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr]
    K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp, z_ccr, p_ccr = params
    
    omega = 2.0 * math.pi * freq
    s = 1j * omega
    
    # Muscle activation
    H_act = 1.0 / (tau_act * s + 1.0)
    
    # Sensor dynamics
    # Canal (VCR angular velocity): H_sc(s) = (s + z_sc) / (s + p_sc) * exp(-s * delay_vcr)
    H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
    
    # Tonic otolith (VCR pitch angle): H_ton(s) = p_ton / (s + p_ton) * exp(-s * delay_vcr)
    H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
    
    # Phasic otolith (acceleration feedforward): H_phas(s) = s / (s + p_phas) * exp(-s * delay_vcr)
    H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
    
    # CCR (proprioceptive muscle length): H_ccr(s) = (s + z_ccr) / (s + p_ccr) * exp(-s * delay_ccr)
    H_ccr = ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
    
    # Total reflexive feedback torque on head per unit pitch angle theta:
    # tau_fb = - [ K_sc * H_sc * s + K_ton * H_ton + kp * H_ccr ] * H_act * theta
    Z_fb = (K_sc * H_sc * s + K_ton * H_ton + kp * H_ccr) * H_act
    
    # Passive and gravitational impedance:
    Z_plant = I_eff * (s**2) + B_passive * s + (K_passive - mgh)
    
    # Driving torque from sled translation X_sled:
    # Inertial torque from base acceleration: tau_inertial = m * h * (omega^2) * X_sled
    # Plus otolith phasic feedback: tau_phas = K_phas * H_phas * (omega^2) * X_sled * H_act
    T_drive = (m * h * (omega**2) - K_phas * H_phas * (omega**2) * H_act)
    
    # Closed loop response: Theta / X_sled [rad / m]
    Theta_over_X = T_drive / (Z_plant + Z_fb)
    
    # Convert Theta / X_sled to Angular Velocity Gain:
    # Head angular velocity = s * Theta [rad/s]
    # Sled velocity = s * X_sled [m/s]
    # Ratio = (s * Theta) / (s * X_sled) = Theta / X_sled [rad/m]
    # In deg/m:
    Gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
    
    # Phase in degrees (relative to sled displacement, with velocity lead -90 deg):
    phase_rad = cmath.phase(Theta_over_X)
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    
    return Gain_deg_m, phase_deg

def objective(params):
    total_loss = 0.0
    for f, t_gain, t_phase in zip(freqs, exp_gain, exp_phase):
        sim_gain, sim_phase = transfer_function(params, f)
        
        sim_gain_safe = max(1e-6, sim_gain)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
        gain_loss = (gain_db_diff ** 2) * 100.0
        
        phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        total_loss += (gain_loss + phase_loss)
    return total_loss

# Initial guess (Happee scale)
x0 = np.array([2.0, 10.0, 0.55, 10.0, 50.0, 0.87, 35.0, 0.81, 12.0, 45.0])
bounds = [
    (0.1, 15.0),  # K_ton
    (2.0, 40.0),  # p_ton
    (0.1, 10.0),  # K_sc
    (1.0, 30.0),  # z_sc
    (20.0, 100.0),# p_sc
    (0.1, 10.0),  # K_phas
    (10.0, 60.0), # p_phas
    (0.1, 3.0),   # kp
    (1.0, 30.0),  # z_ccr
    (15.0, 90.0)  # p_ccr
]

print(f"\nInitial Guess Loss: {objective(x0):.2f}")

# Run Gradient Descent (L-BFGS-B)
print("Running L-BFGS-B gradient descent on analytical surrogate...")
res = opt.minimize(objective, x0, method='L-BFGS-B', bounds=bounds, options={'maxiter': 500, 'disp': False})

print(f"Optimization Status: {res.message}")
print(f"Optimal Surrogate Score: {res.fun:.2f}")
print("\nOptimal Parameter Set:")
param_names = ['K_ton', 'p_ton', 'K_sc', 'z_sc', 'p_sc', 'K_phas', 'p_phas', 'kp', 'z_ccr', 'p_ccr']
for name, val in zip(param_names, res.x):
    print(f"  {name:<10}: {val:.3f}")

print("\n--- Frequency Response of Optimal Surrogate ---")
print(f"{'Freq [Hz]':<10} | {'Sim Gain':<12} | {'Tgt Gain':<12} | {'Gain Diff':<12} | {'Sim Phase':<12} | {'Tgt Phase':<12} | {'Phase Diff':<12}")
print("-" * 90)
for f, t_gain, t_phase in zip(freqs, exp_gain, exp_phase):
    sg, sp = transfer_function(res.x, f)
    g_diff = 20.0 * math.log10(sg / t_gain)
    p_diff = (sp - t_phase + 180.0) % 360.0 - 180.0
    print(f"{f:<10.3f} | {sg:<12.1f} | {t_gain:<12.1f} | {g_diff:+10.2f} dB | {sp:<10.1f}° | {t_phase:<10.1f}° | {p_diff:10.1f}°")

