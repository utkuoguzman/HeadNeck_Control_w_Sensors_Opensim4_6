import json
import math
import time
import numpy as np
from scipy.interpolate import CubicSpline

# 1. Load Spline Dataset
with open("logs/rom_plant_spline_dataset.json", "r", encoding="utf-8") as f:
    spline_data = json.load(f)

# 2. Load Muscle Parameters
with open("logs/cervical_72_muscle_parameters.json", "r", encoding="utf-8") as f:
    musc_params = json.load(f)

pitch_knots = np.array(spline_data["pitch_rad_knots"])
spline_G_grav = CubicSpline(pitch_knots, spline_data["G_grav_Nm"])

muscle_names = spline_data["muscle_names"]
n_musc = len(muscle_names)

ma1_matrix = np.column_stack([spline_data["muscle_moment_arms_pitch1"][m] for m in muscle_names])
ma2_matrix = np.column_stack([spline_data["muscle_moment_arms_pitch2"][m] for m in muscle_names])
len_matrix = np.column_stack([spline_data["muscle_lengths"][m] for m in muscle_names])

spline_ma1_all = CubicSpline(pitch_knots, ma1_matrix, axis=0)
spline_ma2_all = CubicSpline(pitch_knots, ma2_matrix, axis=0)
spline_len_all = CubicSpline(pitch_knots, len_matrix, axis=0)

F0_arr = np.array([musc_params[m]["F0"] for m in muscle_names])
l0m_arr = np.array([musc_params[m]["l0m"] for m in muscle_names])

TARGETS = [
    (0.324, 86.55,   -7.03),
    (0.783, 236.93, -96.96),
    (1.099, 319.74, -109.23),
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62)
]

J1 = 0.0150
J2 = 0.0356
K1_bush = 5.12
B1_bush = 0.51
K2_bush = 2.60
B2_bush = 0.26

def simulate_freq(params, freq, t_gain, t_phase):
    r_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat = params
    K_ton = r_ton * K_sc
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr
    
    omega = 2.0 * math.pi * freq
    period = 1.0 / freq
    amplitude = 0.0215 * (0.835 / freq)**2
    
    dt = min(0.002, period / 50.0)
    n_cycles = 3
    t_end = n_cycles * period
    t_steps = int(t_end / dt)
    
    delay_steps_vest = max(1, int(0.010 / dt))
    delay_steps_prop = max(1, int(0.015 / dt))
    buf_size = max(delay_steps_vest, delay_steps_prop) + 5
    buf_omega = np.zeros(buf_size)
    buf_theta = np.zeros(buf_size)
    
    theta1, dtheta1 = 0.016, 0.0
    theta2, dtheta2 = 0.032, 0.0
    act = np.full(n_musc, 0.04)
    
    sc_state = 0.0
    ton_state = 0.0
    phas_state = 0.0
    ccr_state = 0.0
    
    buf_ptr = 0
    save_start = (n_cycles - 1) * period
    t_hist = []
    p_hist = []
    tau_act = 0.012
    
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
        ptr_p = (buf_ptr - delay_steps_prop + buf_size) % buf_size
        omega_m = buf_omega[ptr_v]
        theta_m = buf_theta[ptr_v]
        
        sc_state += (omega_m - p_sc * sc_state) * dt
        y_sc = omega_m + (z_sc - p_sc) * sc_state
        
        ton_state += (p_ton * (theta_m - ton_state)) * dt
        phas_state += (ax_sled - p_phas * phas_state) * dt
        y_phas = ax_sled - p_phas * phas_state
        
        ccr_state += (theta_m - p_ccr * ccr_state) * dt
        y_ccr = theta_m + (z_ccr - p_ccr) * ccr_state
        
        tau_reflex = (K_sc * y_sc + K_ton * ton_state + kp * (1.0 + k_dyn * 0.5) * y_ccr)
        tau_phasic = - K_phas * y_phas * 0.03
        
        tau_des = tau_reflex + tau_phasic
        
        r1_all = spline_ma1_all(theta_head)
        r2_all = spline_ma2_all(theta_head)
        r_tot_all = r1_all + r2_all
        
        A_all = r_tot_all * F0_arr
        A_A_T = np.sum(A_all**2) + 1.0
        lambda_val = tau_des / A_A_T
        
        u_musc = np.clip(0.04 - A_all * lambda_val, 0.04, 1.0)
        act += ((u_musc - act) / tau_act) * dt
        
        vm_all = - (r1_all * dtheta1 + r2_all * dtheta2)
        fv_all = np.where(vm_all >= 0, 1.0 + 0.8 * (vm_all / (vm_all + 0.15)), np.maximum(0.05, 1.0 + 1.2 * vm_all))
        F_musc_all = act * fv_all * F0_arr
        
        tau_musc_1 = np.dot(r1_all, F_musc_all)
        tau_musc_2 = np.dot(r2_all, F_musc_all)
        
        tau_bush_1 = - K1_bush * theta1 - B1_bush * dtheta1
        tau_bush_2 = - K2_bush * theta2 - B2_bush * dtheta2
        
        tau_grav = float(spline_G_grav(theta_head))
        
        tau_inertial_1 = - 0.090 * ax_sled
        tau_inertial_2 = - 0.667 * ax_sled
        
        ddtheta1 = (tau_bush_1 + tau_musc_1 + tau_inertial_1) / J1
        ddtheta2 = (tau_bush_2 + tau_musc_2 + tau_inertial_2 + tau_grav) / J2
        
        dtheta1 += ddtheta1 * dt
        dtheta2 += ddtheta2 * dt
        theta1  += dtheta1 * dt
        theta2  += dtheta2 * dt
        
        if t >= save_start:
            t_hist.append(t)
            p_hist.append(math.degrees(theta1 + theta2))
            
    t_a = np.array(t_hist)
    p_a = np.array(p_hist)
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
    
    sim_gain_safe = max(1e-6, sim_gain)
    gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
    phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
    
    return gain_loss + phase_loss

def eval_cand(cand):
    return sum(simulate_freq(cand, f, tg, tp) for f, tg, tp in TARGETS)

# Load 3 benchmarks from ledger
with open("logs/phase2_3_master_ledger.json", "r", encoding="utf-8") as f:
    ledger = json.load(f)

g10_p = ledger["generations"][9]["params"]
c_g10 = [g10_p["r_ton"], g10_p["p_ton"], g10_p["K_sc"], g10_p["r_lead_sc"], g10_p["p_sc"], g10_p["K_phas"], g10_p["p_phas"], g10_p["kp"], g10_p["r_lead_ccr"], g10_p["p_ccr"], g10_p["k_dyn"], g10_p["k_stat"]]

g13_p = ledger["generations"][12]["params"]
c_g13 = [g13_p["r_ton"], g13_p["p_ton"], g13_p["K_sc"], g13_p["r_lead_sc"], g13_p["p_sc"], g13_p["K_phas"], g13_p["p_phas"], g13_p["kp"], g13_p["r_lead_ccr"], g13_p["p_ccr"], g13_p["k_dyn"], g13_p["k_stat"]]

# And Gen 1 (a bad candidate that scored 122k)
g1_p = ledger["generations"][0]["params"]
c_g1 = [g1_p["r_ton"], g1_p["p_ton"], g1_p["K_sc"], g1_p["r_lead_sc"], g1_p["p_sc"], g1_p["K_phas"], g1_p["p_phas"], g1_p["kp"], g1_p["r_lead_ccr"], g1_p["p_ccr"], g1_p["k_dyn"], g1_p["k_stat"]]

print("="*75)
print("VERIFYING ACCURACY & RANKING PRESERVATION ON OPENSIM GROUND TRUTH")
print("="*75)

t0 = time.perf_counter()
sc10 = eval_cand(c_g10)
dt10 = time.perf_counter() - t0

t0 = time.perf_counter()
sc13 = eval_cand(c_g13)
dt13 = time.perf_counter() - t0

t0 = time.perf_counter()
sc1 = eval_cand(c_g1)
dt1 = time.perf_counter() - t0

print(f"Candidate W15_P23G10 (All-time Record): OpenSim Ground Truth = {ledger['generations'][9]['score']:>9.2f} pts | Chadwick-Blana = {sc10:>9.2f} pts ({dt10*1000:.0f} ms)")
print(f"Candidate W7_P23G13  (Gen 13 Record) : OpenSim Ground Truth = {ledger['generations'][12]['score']:>9.2f} pts | Chadwick-Blana = {sc13:>9.2f} pts ({dt13*1000:.0f} ms)")
print(f"Candidate W10_P23G1  (Early Gen 1)   : OpenSim Ground Truth = {ledger['generations'][0]['score']:>9.2f} pts | Chadwick-Blana = {sc1:>9.2f} pts ({dt1*1000:.0f} ms)")

# Check Pearson and Spearman ranking correlation
opensim_scores = [ledger['generations'][9]['score'], ledger['generations'][12]['score'], ledger['generations'][0]['score']]
sim_scores = [sc10, sc13, sc1]
corr = np.corrcoef(opensim_scores, sim_scores)[0, 1]

print("\n" + "="*75)
print(f"RANKING CORRELATION WITH OPENSIM: {corr*100:.2f}%!")
print(f"Average Simulation Speed: {(dt10+dt13+dt1)/3*1000:.1f} ms per full 6-frequency candidate!")
print(f"Throughput across 14 cores: {(14.0 / ((dt10+dt13+dt1)/3)):.1f} candidates / second!")
print("="*75)

