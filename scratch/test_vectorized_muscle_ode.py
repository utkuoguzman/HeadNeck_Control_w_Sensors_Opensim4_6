import json
import math
import time
import numpy as np

# Load spline dataset
dataset_path = "logs/rom_plant_spline_dataset.json"
with open(dataset_path, "r", encoding="utf-8") as f:
    spline_data = json.load(f)

# Precompute muscle arrays at upright equilibrium
PRIMARY_NECK_MUSCLES = [
    "stern_mast", "stern_mast_L", "cleid_mast", "cleid_mast_L",
    "scalenus_ant", "scalenus_ant_L", "scalenus_med", "scalenus_med_L",
    "long_cap_sklc4", "long_cap_sklc4_L", "long_col_c1thx", "long_col_c1thx_L",
    "splen_cap_sklc6", "splen_cap_sklc6_L", "splen_cap_sklthx", "splen_cap_sklthx_L",
    "semi_cap_sklc5", "semi_cap_sklc5_L", "semi_cap_sklthx", "semi_cap_sklthx_L",
    "trap_cl", "trap_cl_L", "rectcap_post_maj", "rectcap_post_maj_L"
]

n_musc = len(PRIMARY_NECK_MUSCLES)
F0_arr = np.zeros(n_musc)
r1_arr = np.zeros(n_musc)
r2_arr = np.zeros(n_musc)

for idx, mname in enumerate(PRIMARY_NECK_MUSCLES):
    # Nominal moment arms at upright equilibrium (theta ~ 0)
    mid_idx = 45 # index for ~0 deg in 91 knots
    r1_arr[idx] = spline_data["muscle_moment_arms_pitch1"][mname][mid_idx]
    r2_arr[idx] = spline_data["muscle_moment_arms_pitch2"][mname][mid_idx]
    
    if "stern" in mname or "cleid" in mname: F0_arr[idx] = 110.0
    elif "splen" in mname or "semi" in mname: F0_arr[idx] = 90.0
    elif "trap" in mname: F0_arr[idx] = 120.0
    elif "scalen" in mname: F0_arr[idx] = 60.0
    else: F0_arr[idx] = 50.0

r_tot_arr = r1_arr + r2_arr

TARGETS = [
    (0.324, 86.55,   -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

J1 = 0.022
J2 = 0.035
K1_bush = 5.12
B1_bush = 0.51
K2_bush = 2.60
B2_bush = 0.26

def simulate_fast_vectorized_freq(params, freq, t_gain, t_phase):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = params
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    omega = 2.0 * math.pi * freq
    period = 1.0 / freq
    amplitude = 0.0215 * (0.835 / freq)**2
    
    dt = min(0.002, period / 40.0)
    n_cycles = 2
    t_end = n_cycles * period
    t_steps = int(t_end / dt)
    
    theta1, dtheta1 = 0.016, 0.0
    theta2, dtheta2 = 0.032, 0.0
    act = np.full(n_musc, 0.04)
    
    delay_steps_vest = max(1, int(0.010 / dt))
    buf_size = delay_steps_vest + 5
    buf_omega = np.zeros(buf_size)
    buf_theta = np.zeros(buf_size)
    
    sc_state = 0.0
    ton_state = 0.0
    phas_state = 0.0
    ccr_state = 0.0
    
    t_save = []
    pitch_save = []
    
    tau_act = 0.012
    buf_ptr = 0
    
    for step in range(t_steps):
        t = step * dt
        x_sled = amplitude * math.sin(omega * t)
        ax_sled = - (omega**2) * x_sled
        
        theta_head = theta1 + theta2
        omega_head = dtheta1 + dtheta2
        
        buf_ptr = (buf_ptr + 1) % buf_size
        buf_omega[buf_ptr] = omega_head
        buf_theta[buf_ptr] = theta_head
        
        ptr_v = (buf_ptr - delay_steps_vest + buf_size) % buf_size
        omega_m = buf_omega[ptr_v]
        theta_m = buf_theta[ptr_v]
        
        sc_state += (omega_m - p_sc * sc_state) * dt
        y_sc = omega_m + (z_sc - p_sc) * sc_state
        
        ton_state += (p_ton * (theta_m - ton_state)) * dt
        phas_state += (ax_sled - p_phas * phas_state) * dt
        y_phas = ax_sled - p_phas * phas_state
        
        ccr_state += (theta_m - p_ccr * ccr_state) * dt
        y_ccr = theta_m + (z_ccr - p_ccr) * ccr_state
        
        u_reflex = K_sc * y_sc + K_ton * ton_state + kp * (1.0 + k_dyn * 0.5) * y_ccr
        u_phasic = K_phas * y_phas
        
        # Vectorized muscle activation & forces
        u_arr = np.clip(0.04 + np.maximum(0.0, - r_tot_arr * u_reflex * 0.05), 0.04, 1.0)
        act += ((u_arr - act) / tau_act) * dt
        
        vm_arr = - (r1_arr * dtheta1 + r2_arr * dtheta2)
        # Vectorized force-velocity
        fv_arr = np.where(vm_arr >= 0, 1.0 + 0.8 * (vm_arr / (vm_arr + 0.15)), np.maximum(0.05, 1.0 + 1.2 * vm_arr))
        F_arr = act * fv_arr * F0_arr
        
        tau_musc_1 = np.dot(r1_arr, F_arr)
        tau_musc_2 = np.dot(r2_arr, F_arr)
        
        tau_grav = 6.3 * theta_head
        tau_bush_1 = - K1_bush * theta1 - B1_bush * dtheta1
        tau_bush_2 = - K2_bush * theta2 - B2_bush * dtheta2
        
        tau_inertial_1 = - 0.012 * ax_sled
        tau_inertial_2 = - 0.025 * ax_sled + u_phasic * 0.01
        
        ddtheta1 = (tau_bush_1 + tau_musc_1 + tau_inertial_1) / J1
        ddtheta2 = (tau_bush_2 + tau_musc_2 + tau_inertial_2 + tau_grav) / J2
        
        dtheta1 += ddtheta1 * dt
        dtheta2 += ddtheta2 * dt
        theta1 += dtheta1 * dt
        theta2 += dtheta2 * dt
        
        if t >= (n_cycles - 1) * period:
            t_save.append(t)
            pitch_save.append(math.degrees(theta1 + theta2))
            
    # Fourier
    t_a = np.array(t_save)
    p_a = np.array(pitch_save)
    coeffs = np.polyfit(t_a, p_a, 1)
    p_det = p_a - (coeffs[0] * t_a + coeffs[1])
    
    Is = 2.0 * np.mean(p_det * np.sin(omega * t_a))
    Ic = 2.0 * np.mean(p_det * np.cos(omega * t_a))
    amp_deg = math.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    
    sim_gain = amp_deg / amplitude
    sim_phase = math.degrees(phase_rad)
    while sim_phase > 90.0: sim_phase -= 360.0
    while sim_phase < -270.0: sim_phase += 360.0
    
    gain_db_diff = 20.0 * math.log10(max(1e-6, sim_gain) / t_gain)
    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
    phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
    
    return gain_loss + phase_loss

def fast_eval_candidate(params):
    return sum(simulate_fast_vectorized_freq(params, f, tg, tp) for f, tg, tp in TARGETS)

# Benchmark speed across 100 candidates
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)
cand_vec = [
    ledger["generations"][9]["params"]["r_ton"], 39.68, 1.63, 0.0311, 33.97,
    7.43, 15.99, 1.097, 0.1438, 66.78, 1.20, 0.3516
]

print("Benchmarking Vectorized Non-Linear Muscle Simulator...")
t0 = time.perf_counter()
n_trials = 100
for _ in range(n_trials):
    _ = fast_eval_candidate(cand_vec)
dt = time.perf_counter() - t0

print(f"Evaluated {n_trials} full 6-frequency candidates in {dt:.3f} s!")
print(f"Time per candidate: {dt/n_trials*1000.0:.2f} ms ({n_trials/dt:,.0f} evals/sec)!")
print(f"Time to evaluate 8,000 candidates across 14 cores: {(dt/n_trials*8000)/14:.2f} seconds!")

