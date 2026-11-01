import json
import math
import cmath
import os
import time
import numpy as np
from scipy.interpolate import CubicSpline

# 1. Load the Offline ROM Spline Dataset
dataset_path = "logs/rom_plant_spline_dataset.json"
with open(dataset_path, "r", encoding="utf-8") as f:
    spline_data = json.load(f)

pitch_knots = np.array(spline_data["pitch_rad_knots"])
spline_G_grav = CubicSpline(pitch_knots, spline_data["G_grav_Nm"])

muscle_names = spline_data["muscle_names"]
# Select the 24 primary cervical muscle pairs (12 bilateral pairs)
PRIMARY_NECK_MUSCLES = [
    "stern_mast", "stern_mast_L",
    "cleid_mast", "cleid_mast_L",
    "scalenus_ant", "scalenus_ant_L",
    "scalenus_med", "scalenus_med_L",
    "long_cap_sklc4", "long_cap_sklc4_L",
    "long_col_c1thx", "long_col_c1thx_L",
    "splen_cap_sklc6", "splen_cap_sklc6_L",
    "splen_cap_sklthx", "splen_cap_sklthx_L",
    "semi_cap_sklc5", "semi_cap_sklc5_L",
    "semi_cap_sklthx", "semi_cap_sklthx_L",
    "trap_cl", "trap_cl_L",
    "rectcap_post_maj", "rectcap_post_maj_L"
]

# Build splines for primary neck muscles
splines_ma_p1 = {}
splines_ma_p2 = {}
splines_len = {}
F0_dict = {}

for mname in PRIMARY_NECK_MUSCLES:
    if mname in spline_data["muscle_moment_arms_pitch1"]:
        splines_ma_p1[mname] = CubicSpline(pitch_knots, spline_data["muscle_moment_arms_pitch1"][mname])
        splines_ma_p2[mname] = CubicSpline(pitch_knots, spline_data["muscle_moment_arms_pitch2"][mname])
        splines_len[mname]   = CubicSpline(pitch_knots, spline_data["muscle_lengths"][mname])
        # Approximate F0 (N)
        if "stern" in mname or "cleid" in mname: F0_dict[mname] = 110.0
        elif "splen" in mname or "semi" in mname: F0_dict[mname] = 90.0
        elif "trap" in mname: F0_dict[mname] = 120.0
        elif "scalen" in mname: F0_dict[mname] = 60.0
        else: F0_dict[mname] = 50.0

TARGETS = [
    (0.324, 86.55,   -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

# Physical 2-DOF parameters
J1 = 0.022 # lower neck inertia (kg m^2)
J2 = 0.035 # upper neck + skull inertia (kg m^2)
K1_bush = 5.12 # lower intervertebral series bushing stiffness (Nm/rad)
B1_bush = 0.51 # lower damping (Nms/rad)
K2_bush = 2.60 # upper intervertebral series bushing stiffness (Nm/rad)
B2_bush = 0.26 # upper damping (Nms/rad)

def simulate_nonlinear_muscle_freq(params, freq, target_gain, target_phase):
    # params: 12-vector
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = params
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    omega = 2.0 * math.pi * freq
    period = 1.0 / freq
    amplitude = 0.0215 * (0.835 / freq)**2
    
    # Simulate 3 steady cycles
    dt = min(0.002, period / 50.0)
    n_cycles = 3
    t_end = n_cycles * period
    t_steps = int(t_end / dt)
    
    delay_steps_vest = max(1, int(0.010 / dt)) # 10 ms delay
    delay_steps_prop = max(1, int(0.015 / dt)) # 15 ms delay
    
    # State: [theta1, dtheta1, theta2, dtheta2]
    # Initial posture: small equilibrium trim
    theta1 = 0.016
    dtheta1 = 0.0
    theta2 = 0.032
    dtheta2 = 0.0
    
    # Muscle activations (initialized to co-contraction floor 0.04)
    act = {mname: 0.04 for mname in PRIMARY_NECK_MUSCLES}
    
    # Ring buffers for delayed sensory signals
    buf_size = max(delay_steps_vest, delay_steps_prop) + 5
    buf_omega_head = np.zeros(buf_size)
    buf_theta_head = np.zeros(buf_size)
    
    # Lead-lag filter states
    sc_lead_state = 0.0
    ton_state = 0.0
    phas_state = 0.0
    ccr_state = 0.0
    
    t_history = []
    pitch_history = []
    
    buf_ptr = 0
    
    for step in range(t_steps):
        t = step * dt
        
        # Sled base motion
        x_sled = amplitude * math.sin(omega * t)
        ax_sled = - (omega**2) * x_sled
        
        theta_head = theta1 + theta2
        omega_head = dtheta1 + dtheta2
        
        # Update ring buffer
        buf_ptr = (buf_ptr + 1) % buf_size
        buf_omega_head[buf_ptr] = omega_head
        buf_theta_head[buf_ptr] = theta_head
        
        # Delayed sensor readout
        ptr_vest = (buf_ptr - delay_steps_vest + buf_size) % buf_size
        ptr_prop = (buf_ptr - delay_steps_prop + buf_size) % buf_size
        
        omega_meas = buf_omega_head[ptr_vest]
        theta_meas = buf_theta_head[ptr_vest]
        
        # Semicircular canal lead filter: (s + z_sc) / (s + p_sc)
        sc_lead_state += (omega_meas - p_sc * sc_lead_state) * dt
        y_sc = omega_meas + (z_sc - p_sc) * sc_lead_state
        
        # Tonic otolith lag filter: p_ton / (s + p_ton)
        ton_state += (p_ton * (theta_meas - ton_state)) * dt
        
        # Phasic otolith high-pass: s / (s + p_phas) on sled acceleration
        phas_state += (ax_sled - p_phas * phas_state) * dt
        y_phas = ax_sled - p_phas * phas_state
        
        # CCR spindle filter: (s + z_ccr) / (s + p_ccr)
        ccr_state += (theta_meas - p_ccr * ccr_state) * dt
        y_ccr = theta_meas + (z_ccr - p_ccr) * ccr_state
        
        # Total neural reflex drive on pitch
        u_reflex_pitch = (K_sc * y_sc + K_ton * ton_state + kp * (1.0 + k_dyn * 0.5) * y_ccr)
        
        # Feedforward phasic otolith drive
        u_phasic = K_phas * y_phas
        
        # Muscle forces & moments
        tau_musc_1 = 0.0
        tau_musc_2 = 0.0
        
        tau_act = 0.012
        for mname in PRIMARY_NECK_MUSCLES:
            r1 = float(splines_ma_p1[mname](theta_head))
            r2 = float(splines_ma_p2[mname](theta_head))
            r_tot = r1 + r2
            
            # Reciprocal muscle activation drive
            u_i = 0.04 + max(0.0, - r_tot * u_reflex_pitch * 0.05)
            u_i = min(1.0, max(0.04, u_i))
            
            # Activation dynamics
            act[mname] += ((u_i - act[mname]) / tau_act) * dt
            
            # Hill force: unilateral pulling constraint F >= 0
            F0 = F0_dict[mname]
            vm = - (r1 * dtheta1 + r2 * dtheta2) # lengthening rate
            
            # Normalized velocity factor
            if vm >= 0:
                fv = 1.0 + 0.8 * (vm / (vm + 0.15)) # eccentric plateau
            else:
                fv = max(0.05, 1.0 + 1.2 * vm) # concentric drop-off
                
            F_musc = act[mname] * fv * F0
            tau_musc_1 += r1 * F_musc
            tau_musc_2 += r2 * F_musc
            
        # Gravitational torque
        tau_grav = float(spline_G_grav(theta_head))
        
        # Passive intervertebral bushings
        tau_bush_1 = - K1_bush * theta1 - B1_bush * dtheta1
        tau_bush_2 = - K2_bush * theta2 - B2_bush * dtheta2
        
        # Sled inertial base acceleration torque
        tau_inertial_1 = - 0.012 * ax_sled
        tau_inertial_2 = - 0.025 * ax_sled + u_phasic * 0.01
        
        # 2-DOF Equations of motion (Euler integration)
        ddtheta1 = (tau_bush_1 + tau_musc_1 + tau_inertial_1) / J1
        ddtheta2 = (tau_bush_2 + tau_musc_2 + tau_inertial_2 + tau_grav) / J2
        
        dtheta1 += ddtheta1 * dt
        dtheta2 += ddtheta2 * dt
        theta1 += dtheta1 * dt
        theta2 += dtheta2 * dt
        
        # Save last cycle for Fourier extraction
        if t >= (n_cycles - 1) * period:
            t_history.append(t)
            pitch_history.append(math.degrees(theta1 + theta2))
            
    # Fourier extraction of gain and phase
    t_arr = np.array(t_history)
    p_arr = np.array(pitch_history)
    
    # Detrend
    coeffs = np.polyfit(t_arr, p_arr, 1)
    p_detrend = p_arr - (coeffs[0] * t_arr + coeffs[1])
    
    Is = 2.0 * np.mean(p_detrend * np.sin(omega * t_arr))
    Ic = 2.0 * np.mean(p_detrend * np.cos(omega * t_arr))
    amp_deg = math.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    
    sim_gain = amp_deg / amplitude
    sim_phase = math.degrees(phase_rad)
    while sim_phase > 90.0: sim_phase -= 360.0
    while sim_phase < -270.0: sim_phase += 360.0
    
    # Isotropic Loss
    sim_gain_safe = max(1e-6, sim_gain)
    gain_db_diff = 20.0 * math.log10(sim_gain_safe / target_gain)
    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
    
    phase_diff = (sim_phase - target_phase + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
    
    return gain_loss + phase_loss, gain_db_diff, abs(phase_diff)

def evaluate_nonlinear_candidate(params):
    total_loss = 0.0
    details = {}
    for freq, t_gain, t_phase in TARGETS:
        score_f, g_db, p_err = simulate_nonlinear_muscle_freq(params, freq, t_gain, t_phase)
        total_loss += score_f
        details[freq] = (g_db, p_err, score_f)
    return total_loss, details

print("Non-Linear 2-DOF Musculoskeletal Time-Domain Simulator Ready!")

if __name__ == "__main__":
    with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
        ledger = json.load(f)
    g10 = [g for g in ledger["generations"] if g["generation"] == 10][0]
    p = g10["params"]
    
    cand_vec = [
        p["r_ton"], p["p_ton"], p["K_sc"], p["r_lead_sc"], p["p_sc"],
        p["K_phas"], p["p_phas"], p["kp"], p["r_lead_ccr"], p["p_ccr"],
        p["k_dyn"], p["k_stat"]
    ]
    
    print("\nBenchmarking Non-Linear ODE Simulator on W15_P23G10...")
    t0 = time.perf_counter()
    score, details = evaluate_nonlinear_candidate(cand_vec)
    dt = time.perf_counter() - t0
    
    print(f"Evaluated all 6 frequencies in {dt*1000:.1f} ms ({dt/6*1000:.2f} ms/freq)!")
    print(f"Total Non-Linear Score: {score:.2f} pts (OpenSim ground truth was {g10['score']:.2f} pts)")
    print("\nFrequency Breakdown:")
    for freq in details:
        g_db, p_err, sc = details[freq]
        print(f"  {freq:5.3f} Hz -> Gain Err: {g_db:>6.1f} dB | Phase Err: {p_err:>5.1f}° | Score: {sc:>7.1f} pts")

