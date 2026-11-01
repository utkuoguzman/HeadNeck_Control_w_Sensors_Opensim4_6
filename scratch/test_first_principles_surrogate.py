import math
import cmath

# Physical plant parameters of the CURRENT uncoupled model
J_eff = 0.1633      # kg*m^2 (skull pitch inertia about C7/T1)
B_pass = 1.46       # Nms/rad (Kd_task=1.31 + joint damping)
K_pass = 5.20       # Nm/rad (Kp_task=10.24 - mgh=7.63 + ligament passive stiffness)
M_drive = 0.778     # kg*m (m*h)
tau_act = 0.012     # s (muscle activation delay)
k_geom = 1.0        # geometric scale factor

# W3 parameters
r_ton = 0.5769
p_ton = 76.4989
K_sc = 0.5302
r_lead_sc = 0.2458
p_sc = 96.0501
K_phas = 4.2943
p_phas = 17.5282
kp = 1.2839
r_lead_ccr = 0.0687
p_ccr = 177.0660
k_dyn = 0.7189
k_stat = 1.1773

K_ton = r_ton * K_sc
z_sc = r_lead_sc * p_sc
z_ccr = r_lead_ccr * p_ccr

delay_vcr = 0.010
delay_ccr = 0.015

freq = 0.783
omega = 2.0 * math.pi * freq
s = 1j * omega

H_act = 1.0 / (tau_act * s + 1.0)
H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)

Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
Z_plant = - (omega**2) * J_eff + s * B_pass + K_pass
T_drive = - (omega**2) * (M_drive - K_phas * H_phas * H_act)

Theta_over_X = (T_drive / (Z_plant + Z_reflex)) * k_geom
gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
phase_rad = cmath.phase(Theta_over_X)
phase_deg = math.degrees(phase_rad)
while phase_deg > 90.0: phase_deg -= 360.0
while phase_deg < -270.0: phase_deg += 360.0

print(f"Surrogate with first-principles physical plant at {freq} Hz:")
print(f"  Gain:  {gain_deg_m:.2f} deg/m  (OpenSim actual: 160.24 deg/m, Target: 236.93 deg/m)")
print(f"  Phase: {phase_deg:.2f} deg    (OpenSim actual: -86.73 deg,  Target: -96.96 deg)")

