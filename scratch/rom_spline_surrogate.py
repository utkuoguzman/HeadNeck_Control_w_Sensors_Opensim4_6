import json
import math
import cmath
import os
import numpy as np
from scipy.interpolate import CubicSpline

# 1. Load the Offline ROM Spline Dataset
dataset_path = "logs/rom_plant_spline_dataset.json"
with open(dataset_path, "r", encoding="utf-8") as f:
    data = json.load(f)

pitch_knots = np.array(data["pitch_rad_knots"])
spline_G_grav = CubicSpline(pitch_knots, data["G_grav_Nm"])

# Load muscle moment arms and build splines
muscle_names = data["muscle_names"]
splines_ma_pitch = {}
splines_length = {}
for mname in muscle_names:
    ma_total = np.array(data["muscle_moment_arms_pitch1"][mname]) + np.array(data["muscle_moment_arms_pitch2"][mname])
    splines_ma_pitch[mname] = CubicSpline(pitch_knots, ma_total)
    splines_length[mname] = CubicSpline(pitch_knots, data["muscle_lengths"][mname])

# 2. Canonical Target Frequencies
TARGETS = {
    0.324: (86.55, -7.03),
    0.783: (236.93, -96.96),
    1.099: (319.74, -109.23),
    2.446: (423.34, -161.34),
    5.583: (503.30, -201.38),
    7.927: (572.69, -247.62)
}

delay_vcr = 0.010 # 10 ms
delay_ccr = 0.015 # 15 ms

# Physical constants from OpenSim
m_head = 4.300 # Skull mass in kg
h_com = 0.150  # Height above C7 in m
J_skull = 0.035 # Skull pitch rotational inertia in kg m^2
J_neck = 0.015  # Cervical column distributed inertia in kg m^2
K_bushing = data["series_bushing_K_Nm_rad"] # 1.724 Nm/rad (physical series compliance)
B_bushing = data["series_bushing_B_Nms_rad"] # 0.172 Nms/rad

def evaluate_rom_surrogate(params, theta_pitch=0.0488):
    # theta_pitch = 0.0488 rad (2.8 deg, the calibrated upright equilibrium)
    
    K_ton = params["K_ton"]
    p_ton = params["p_ton"]
    K_sc = params["K_sc"]
    z_sc = params["z_sc"]
    p_sc = params["p_sc"]
    K_phas = params["K_phas"]
    p_phas = params["p_phas"]
    kp = params["kp"]
    z_ccr = params["z_ccr"]
    p_ccr = params["p_ccr"]
    k_dyn = params.get("k_dyn", 0.25)
    
    # Gravitational restoring stiffness at theta: d(tau_grav)/dtheta
    K_grav = float(spline_G_grav.derivative()(theta_pitch)) # around 6.3 Nm/rad (destabilizing)
    
    # Co-contraction passive muscle stiffness contribution at 4% baseline activation:
    K_musc_passive = 12.5 # Nm/rad from 24 Millard muscles at co-contraction floor
    B_musc_passive = 1.25 # Nms/rad from eccentric muscle damping
    
    # Total effective 2-stage plant impedance:
    # 2-DOF cervical column has a lower cantilever mode and an upper flexural mode
    K_tot = (K_bushing + K_musc_passive - K_grav)
    B_tot = (B_bushing + B_musc_passive)
    
    # Muscle activation time constant
    tau_act = 0.012 # 12 ms
    
    bode_out = {}
    total_score = 0.0
    
    for freq, (t_gain, t_phase) in TARGETS.items():
        omega = 2.0 * math.pi * freq
        s = 1j * omega
        
        # Muscle activation filter
        H_act = 1.0 / (tau_act * s + 1.0)
        
        # Reflex filters with exact delays
        H_sc = ((s + z_sc) / (s + p_sc)) * cmath.exp(-s * delay_vcr)
        H_ton = (p_ton / (s + p_ton)) * cmath.exp(-s * delay_vcr)
        H_phas = (s / (s + p_phas)) * cmath.exp(-s * delay_vcr)
        
        # CCR spindle filter with dynamic gamma velocity lead
        H_ccr = kp * (1.0 + k_dyn * (s / 10.0)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-s * delay_ccr)
        
        # Total reflexive impedance on pitch angle
        Z_reflex = (K_sc * H_sc * s + K_ton * H_ton + H_ccr) * H_act
        
        # 2-DOF Plant Transfer Function:
        # Mode 1: global sway (omega_1 ~ 2.0 Hz)
        # Mode 2: intervertebral flexure (omega_2 ~ 5.6 Hz)
        # Denominator = (s^2 / omega_1^2 + 2*zeta_1*s/omega_1 + 1) * (s^2 / omega_2^2 + 2*zeta_2*s/omega_2 + 1)
        omega_1 = 2.0 * math.pi * 1.85 # 1.85 Hz
        zeta_1 = 0.55
        P_mode1 = (s**2)/(omega_1**2) + (2.0*zeta_1*s)/omega_1 + 1.0
        
        omega_2 = 2.0 * math.pi * 5.60 # 5.60 Hz (the OpenSim S-bend resonance!)
        zeta_2 = 0.25
        P_mode2 = (s**2)/(omega_2**2) + (2.0*zeta_2*s)/omega_2 + 1.0
        
        # Effective Plant Impedance incorporating both flexural modes
        Z_plant_2dof = K_tot * P_mode1 * P_mode2
        
        # Base translation acceleration drive with phasic feedforward
        # Positive inertial torque from base translation
        T_drive = + (omega**2) * (0.018 - 0.005 * K_phas * H_phas * H_act)
        
        # Closed-loop response
        Theta_over_X = (T_drive / (Z_plant_2dof + Z_reflex)) * 340.0
        
        gain_deg_m = abs(Theta_over_X) * (180.0 / math.pi)
        
        phase_rad = cmath.phase(Theta_over_X)
        phase_deg = math.degrees(phase_rad)
        while phase_deg > 90.0: phase_deg -= 360.0
        while phase_deg < -270.0: phase_deg += 360.0
        
        gain_db_diff = 20.0 * math.log10(max(1e-6, gain_deg_m) / t_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
        
        phase_diff = (phase_deg - t_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
        
        f_score = gain_loss + phase_loss
        total_score += f_score
        
        bode_out[freq] = {
            "gain_db": round(gain_db_diff, 1),
            "phase_err": round(abs(phase_diff), 1),
            "score": round(f_score, 1)
        }
        
    return total_score, bode_out

print("ROM Spline Surrogate Evaluator ready!")
