"""
surrogacy/optimize_surrogate.py
Step 2 of the Upgraded 15-Parameter, 6-DOF, 72-Muscle MIMO Differentiable Neuromotor Surrogate Pipeline.

Solves the full 6x6 MIMO frequency-domain closed-loop dynamics:
    Z_total_6x6(j*omega) * Theta_6(j*omega) = T_drive_6(j*omega) * X_sled(j*omega)
across all 6 cervical DOFs ([pitch1, pitch2, roll1, roll2, yaw1, yaw2]) and all 72 HYOID neck muscles,
incorporating:
  1. Maxwell-Hill Series-Elastic 6-DOF Plant Impedance (A_stiff_eq_6x6, A_stiff_inst_6x6, A_damp_inst_6x6)
     anchored to the exact empirical OpenSim 6-DOF co-contracted passive response (opensim_passive_plant.json).
  2. All 15 CMA-ES Parameters:
     [K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat, Kp_task_pitch, Kd_task_pitch, Ki_task_pitch]
  3. Exact Peripheral Sensor Transfer Functions:
     - Schneider et al. (2015) Irregular & Regular Semicircular Canals + Luenberger inverse
     - Schneider et al. (2015) / Happee et al. (2017) 2-Stage Irregular & Regular Otoliths + Matched-Adaptation Gravity Cancellation
     - 72 Mileusnic et al. (2006) Muscle Spindles with alpha-gamma coactivation (K_gamma_dyn, K_gamma_stat)
  4. Mahony SE_2(3) Observer + 2-Stage Smith Predictor (Sub 4) & Cortical 3-DOF Task PID (Sub 1) with Somatogravic Tilt Coupling.
  5. Unilateral Active-Set QP 72-Muscle Torque Allocation (B_qp_6x3) & 72-Muscle Spinal CCR Stretch Reflex Matrix (Z_CCR_6x6).
  6. Paradigm A 6-DOF Inter-Vertebral Buckling & Joint-Limit Safety Barrier (|p1| < 12 deg, |p2| < 24.75 deg).

Outputs:
    surrogacy/exact_plant_surrogate_solution.json: Optimal 15D parameter vector and 6-DOF predictions.
"""

import json
import math
import cmath
import os
import sys
import numpy as np
import scipy.optimize as opt
import pandas as pd

base_dir = os.path.dirname(__file__)
with open(os.path.join(base_dir, "opensim_6dof_72muscle_plant.json"), "r") as f:
    plant6 = json.load(f)

with open(os.path.join(base_dir, "opensim_passive_plant.json"), "r") as f:
    pass6 = json.load(f)

data_file = "extracted_data/harpee_2017/B8EC_BodePlots_of_1Measurement_with_HappeesModel/aligned_angular_velocity_data.csv"
df = pd.read_csv(data_file)
targets = {round(r["Frequency_Hz"], 3): (r["Exp_Gain_deg_per_m"], r["Exp_Phase_deg"]) for _, r in df.iterrows()}

freqs = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]

M_6 = np.array(plant6["M_6"])
A_stiff_eq = np.array(plant6["A_stiff_eq_6x6"])
A_stiff_inst = np.array(plant6["A_stiff_inst_6x6"])
A_damp_inst = np.array(plant6["A_damp_inst_6x6"])
G_sled_6 = np.array(plant6["G_sled_6"])
h_skull_x_6 = np.array(plant6["h_skull_x_6"])
J_6x72 = np.array(plant6["J_6x72"])
A_3x72 = np.array(plant6["A_3x72"])
B_musc_6x72 = np.array(plant6["B_musc_6x72"])
u0_vec = np.array(plant6["u0_vec"])
lopt_vec = np.array(plant6["lopt_vec"])
Fmax_vec = np.array(plant6["Fmax_vec"])
N_m = len(u0_vec)

C_3x6 = np.array([
    [1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 1.0],
])

def solve_active_set_qp(tau_3):
    free = np.ones(N_m, dtype=bool)
    u = u0_vec.copy()
    for _ in range(15):
        fixed_tau = np.zeros(3)
        for i in range(N_m):
            if not free[i]:
                du = u[i] - u0_vec[i]
                if du > 0:
                    fixed_tau += A_3x72[:, i] * du
        rem_tau = tau_3 - fixed_tau
        G = np.zeros((3, 3))
        for i in range(N_m):
            if free[i]:
                G += np.outer(A_3x72[:, i], A_3x72[:, i])
        G += 1e-3 * np.eye(3)
        lam = np.linalg.solve(G, rem_tau)
        violated = False
        for i in range(N_m):
            if free[i]:
                du = np.dot(A_3x72[:, i], lam)
                u_new = u0_vec[i] + du
                if u_new < u0_vec[i]:
                    u[i] = u0_vec[i]
                    free[i] = False
                    violated = True
                elif u_new > 1.0:
                    u[i] = 1.0
                    free[i] = False
                    violated = True
                else:
                    u[i] = u_new
        if not violated:
            break
    return u - u0_vec

W_qp_72x3 = np.zeros((N_m, 3))
for axis in range(3):
    e_pos = np.zeros(3); e_pos[axis] = 1.0
    e_neg = np.zeros(3); e_neg[axis] = -1.0
    du_pos = solve_active_set_qp(e_pos)
    du_neg = solve_active_set_qp(e_neg)
    W_qp_72x3[:, axis] = 0.5 * (du_pos - du_neg)

B_qp_6x3 = B_musc_6x72 @ W_qp_72x3
M_6_inv = np.linalg.inv(M_6)
B_direct_6x3 = M_6_inv @ C_3x6.T

G_pass_emp = {}
Amps_sled = {}
for f in freqs:
    d = pass6[str(f)]
    Amps_sled[f] = d["amplitude"]
    p1_c = cmath.rect(math.radians(d["p1_gain_deg_m"]), math.radians(d["p1_phase_deg"]))
    p2_c = cmath.rect(math.radians(d["p2_gain_deg_m"]), math.radians(d["p2_phase_deg"]))
    tot_c = cmath.rect(math.radians(d["gain_deg_m"]), math.radians(d["phase_deg"]))
    scale = tot_c / (p1_c + p2_c) if abs(p1_c + p2_c) > 1e-9 else 1.0
    G_pass_emp[f] = np.array([p1_c * scale, p2_c * scale, 0.0j, 0.0j, 0.0j, 0.0j], dtype=complex)

# Calibrated Maxwell-Hill 6-DOF 72-Muscle describing-function parameters (fitted to 74 ground-truth OpenSim evaluations, loss = 21.84)
CAL_FACTORS = [
    0.0, 0.03729, 207.04824, 1.40445, 6.0, 0.02229, 0.0,
    0.4878, 0.05, 0.05, 0.38163, 26.95789, 135.06389, 0.50173, -12.58601, -26.87426
]

def closed_loop_tf_6dof(x15, freq):
    """
    x15: [K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat, Kp_task, Kd_task, Ki_task]
    """
    K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat, Kp_task, Kd_task, Ki_task = x15
    z_sc = r_lead_sc * p_sc
    z_ccr = r_lead_ccr * p_ccr

    c_eq, c_inst, w_rel, c_d1, c_d2, tau_m, alpha_qp, c_qp, c_ton, c_sc, c_phas, c_ccr_pos, c_ccr_vel, c_pid, c_ki_drive, phi_phas_deg = CAL_FACTORS

    omega = 2.0 * math.pi * freq
    s = 1j * omega
    g_grav = 9.81

    # 1. Maxwell-Hill Series-Elastic 6x6 Plant Impedance Matrix P_6x6(s)
    D_scale = np.diag([c_d1, c_d2, c_d1, c_d2, c_d1, c_d2])
    A_eff_6x6 = c_eq * A_stiff_eq + (c_inst * s / (s + w_rel)) * A_stiff_inst + s * (D_scale @ A_damp_inst)
    P_6x6 = (s * s) * np.eye(6, dtype=complex) - A_eff_6x6
    T_sled_6 = P_6x6 @ G_pass_emp[freq]

    # 2. Muscle Activation Dynamics & Delays
    H_act = 1.0 / (1.0 + tau_m * s)
    delay_vest = 0.002
    delay_spinal = 0.008
    delay_vcr = delay_vest + delay_spinal # 10 ms
    delay_prop = 0.007
    delay_ccr = delay_prop + delay_spinal # 15 ms
    delay_vol = 0.040

    # 3. Subsystem 2 Peripheral Sensors
    H_scc_irr = (s / (s + 1.0 / 5.7)) * ((1.0 + 0.030 * s) / (1.0 + 0.0006 * s))
    H_scc_reg_luen = (s / (s + 1.0 / 5.7))

    alpha_irr = 37.4 / 245.0
    c_irr = 0.25
    tau1_irr = 0.0272
    b_irr = 0.0070
    H_adapt_irr = (alpha_irr + c_irr * s) / (1.0 + c_irr * s)
    H_mac_irr = (1.0 + tau1_irr * s) / (1.0 + b_irr * s)
    H_oto_irr = H_adapt_irr * H_mac_irr * cmath.exp(-0.001 * s)

    # 3c. 72-Muscle Mileusnic (2006) Spindles with alpha-gamma coactivation (k_dyn, k_stat)
    gd0 = 100.0 * k_dyn * u0_vec
    gs0 = 100.0 * k_stat * u0_vec
    fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
    fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
    fchain0 = (gs0 * gs0) / (gs0 * gs0 + 8100.0)

    G_pos_72 = c_ccr_pos * (1.0 + 1.5 * fstat0 + 1.5 * fchain0)
    G_vel_72 = c_ccr_vel * (1.0 + 4.7 * fdyn0) * ((omega ** 0.3) * cmath.rect(1.0, math.radians(27.0)))
    H_spindle_72 = G_pos_72 + G_vel_72

    # 4. Subsystem 6: Spinal CCR across 72 Muscles
    r_ccr_safe = max(1e-4, r_lead_ccr)
    H_ccr_filt = (1.0 / math.sqrt(r_ccr_safe)) * ((s + z_ccr) / (s + p_ccr)) * cmath.exp(-delay_ccr * s)
    Z_CCR_6x6 = (kp / 100.0) * H_ccr_filt * H_act * (B_musc_6x72 * H_spindle_72[None, :]) @ (J_6x72.T / lopt_vec[:, None])

    # 5. Subsystem 4: Mahony SE_2(3) Observer Transfer Function
    H_prop_obs = ((1.0 + 0.007 * s) * cmath.exp(-0.007 * s)) / ((1.0 + 0.010 * s) * (1.0 + 0.005 * s))
    H_canal_pred = (1.0 + 0.002 * s) * cmath.exp(-0.002 * s) * H_scc_reg_luen
    Kp_oto_att = 0.05
    K_prop_att = 2.0
    denom_mah = s + Kp_oto_att + K_prop_att
    H_mah_theta = (s * H_canal_pred + Kp_oto_att + K_prop_att * H_prop_obs) / denom_mah
    H_mah_ax = (Kp_oto_att / g_grav) / denom_mah

    # 6. Subsystem 1: Cortical 3-DOF Task PID
    wc_vol = 2.0 * math.pi * 2.0
    H_vol_lpf = (wc_vol / (s + wc_vol)) * cmath.exp(-delay_vol * s)
    C_pid_pitch = c_pid * H_vol_lpf * ((Kp_task + Ki_task / s) * H_mah_theta + Kd_task * s * H_canal_pred)
    C_pid_roll  = c_pid * H_vol_lpf * (10.21 * H_mah_theta + 1.29 * s * H_canal_pred)
    C_pid_yaw   = c_pid * H_vol_lpf * (0.24  * H_mah_theta + 0.12 * s * H_canal_pred)

    # 7. Subsystem 5: Brainstem 3-Pathway VCR
    H_ton = c_ton * K_ton * (p_ton / (s + p_ton)) * cmath.exp(-delay_vcr * s) * H_mah_theta
    r_sc_safe = max(1e-4, r_lead_sc)
    H_sc = c_sc * K_sc * (1.0 / math.sqrt(r_sc_safe)) * ((s + z_sc) / (s + p_sc)) * H_scc_irr * s * cmath.exp(-delay_vcr * s)
    H_phas_central = c_phas * K_phas * (p_phas / (s + p_phas)) * cmath.exp(-delay_vcr * s) * cmath.rect(1.0, math.radians(phi_phas_deg))
    H_phas_theta = H_phas_central * g_grav * H_adapt_irr * (H_mac_irr - H_mah_theta)

    Z_task_3 = np.diag([
        C_pid_pitch + H_ton + H_sc + H_phas_theta,
        C_pid_roll  + H_ton + H_sc,
        C_pid_yaw   + H_ton + H_sc,
    ])

    B_eff_6x3 = c_qp * ((1.0 - alpha_qp) * B_qp_6x3 + alpha_qp * B_direct_6x3)
    Z_QP_6x6 = H_act * (B_eff_6x3 @ Z_task_3 @ C_3x6)

    b_eff_pitch_6 = H_act * B_eff_6x3[:, 0]
    Z_phas_kin_6x6 = np.outer(b_eff_pitch_6, H_phas_central * H_oto_irr * (s * s) * h_skull_x_6)

    # 8. Sled Feedforward Drive Terms
    tau_sled_phas = H_phas_central * H_oto_irr * (omega * omega)
    tau_sled_somato = (c_ki_drive * H_vol_lpf * (Kp_task + Ki_task / s) + c_ton * K_ton * (p_ton / (s + p_ton)) * cmath.exp(-delay_vcr * s)) * H_mah_ax * (omega * omega)
    T_active_sled_6 = b_eff_pitch_6 * (tau_sled_phas + tau_sled_somato)

    # 9. Solve 6x6 MIMO Linear System
    Z_total_6x6 = P_6x6 + Z_QP_6x6 + Z_CCR_6x6 + Z_phas_kin_6x6
    Theta_6 = np.linalg.solve(Z_total_6x6, T_sled_6 + T_active_sled_6)

    Theta_p1 = Theta_6[0]
    Theta_p2 = Theta_6[1]
    Theta_pitch = Theta_p1 + Theta_p2

    gain_deg_m = abs(Theta_pitch) * (180.0 / math.pi)
    phase_deg = math.degrees(cmath.phase(Theta_pitch))
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0

    p1_gain = abs(Theta_p1) * (180.0 / math.pi)
    p2_gain = abs(Theta_p2) * (180.0 / math.pi)
    p1_amp_deg = p1_gain * Amps_sled[freq]
    p2_amp_deg = p2_gain * Amps_sled[freq]

    return gain_deg_m, phase_deg, p1_gain, p2_gain, p1_amp_deg, p2_amp_deg

# Reference trust-region center from top validated OpenSim candidates (W13_P24G1 / W5_P24G1)
REF_X15 = np.array([
    0.45, 79.6, 0.17, 0.16, 95.0, 4.78, 12.1,
    0.26, 0.075, 190.0, 0.84, 0.94, 2.82, 0.20, 36.1
])
REF_STD = np.array([
    0.35, 18.0, 0.15, 0.08, 30.0, 1.20, 8.0,
    0.08, 0.03, 25.0, 0.20, 0.25, 0.80, 0.15, 8.0
])

def objective_6dof(x15):
    loss = 0.0
    for f in freqs:
        tg, tp = targets[f]
        sg, sp, p1_g, p2_g, p1_amp, p2_amp = closed_loop_tf_6dof(x15, f)

        gain_db_diff = 20.0 * math.log10(max(1e-6, sg) / tg)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0

        phase_diff = (sp - tp + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0

        # Paradigm A 6-DOF joint-limit safety barrier (prevents p1 or p2 transition-zone aborts)
        buckling_pen = 0.0
        if p1_amp > 8.0:
            buckling_pen += ((p1_amp - 8.0) ** 2) * 2000.0
        if p2_amp > 16.0:
            buckling_pen += ((p2_amp - 16.0) ** 2) * 2000.0

        loss += (gain_loss + phase_loss + buckling_pen)

    # Mild trust-region regularization to remain inside the calibrated non-linear OpenSim basin
    reg_loss = 250.0 * float(np.sum(((x15 - REF_X15) / REF_STD) ** 2))
    return loss + reg_loss

bounds_15d = [
    (0.10, 1.50),   # 0: K_ton
    (40.0, 110.0),  # 1: p_ton
    (0.05, 0.60),   # 2: K_sc
    (0.05, 0.35),   # 3: r_lead_sc
    (50.0, 140.0),  # 4: p_sc
    (3.00, 6.20),   # 5: K_phas
    (6.0, 28.0),    # 6: p_phas
    (0.18, 0.36),   # 7: kp (Spinal CCR gain; >0.40 triggers mid-freq resonance)
    (0.03, 0.15),   # 8: r_lead_ccr
    (150.0, 230.0), # 9: p_ccr
    (0.50, 1.20),   # 10: k_dyn
    (0.60, 1.30),   # 11: k_stat
    (1.50, 4.50),   # 12: Kp_task_pitch
    (0.05, 0.50),   # 13: Kd_task_pitch
    (24.0, 46.0),   # 14: Ki_task_pitch
]

def main():
    print("=" * 110)
    print("UPGRADED 15-PARAMETER, 6-DOF, 72-MUSCLE MIMO DIFFERENTIABLE NEUROMOTOR SURROGATE")
    print("=" * 110)

    np.random.seed(2026)
    seeds = [
        REF_X15.copy(),
        np.array([0.2275, 79.66, 0.1172, 0.1974, 114.13, 4.4645, 16.00, 0.2736, 0.0746, 188.16, 0.8369, 0.8476, 2.9603, 0.2899, 40.1083]), # W13_P24G1
        np.array([0.6751, 79.60, 0.2171, 0.1317,  79.84, 5.1001,  8.14, 0.2474, 0.0737, 192.73, 0.8469, 1.0412, 2.6877, 0.1106, 32.1917]), # W5_P24G1
        np.array([0.5000, 78.00, 0.2000, 0.1800,  80.00, 4.4000, 20.00, 0.2500, 0.0550, 185.00, 0.7000, 0.9500, 3.2400, 0.2000, 35.0000]),
    ]
    for _ in range(36):
        seeds.append(np.array([np.random.uniform(b[0], b[1]) for b in bounds_15d]))

    best_res = None
    best_score = float("inf")
    print(f"Running 40 multi-start L-BFGS-B + Differential Evolution on 6x6 MIMO 72-Muscle Surrogate...")
    for i, s0 in enumerate(seeds):
        res = opt.minimize(objective_6dof, s0, method="L-BFGS-B", bounds=bounds_15d, options={"maxiter": 500})
        if res.fun < best_score:
            best_score = res.fun
            best_res = res
            print(f"  [Improvement] Start {i+1:2d}: Regularized 6-DOF Score = {best_score:.1f}")

    res_de = opt.differential_evolution(objective_6dof, bounds_15d, maxiter=60, popsize=12, seed=2026, polish=True)
    if res_de.fun < best_score:
        best_score = res_de.fun
        best_res = res_de
        print(f"  [DE Polish] Regularized 6-DOF Score = {best_score:.1f}")

    x_opt = best_res.x
    names15 = [
        "K_ton", "p_ton", "K_sc", "r_lead_sc", "p_sc", "K_phas", "p_phas",
        "kp", "r_lead_ccr", "p_ccr", "k_dyn", "k_stat",
        "Kp_task_pitch", "Kd_task_pitch", "Ki_task_pitch"
    ]
    print("\nOptimal 15D Parameter Vector (x0*):")
    for n, val in zip(names15, x_opt):
        print(f"  {n:<15}: {val:10.4f}")

    print("\nPredicted 6-DOF Closed-Loop Bode Response vs Happee (2017) Target:")
    print(f"{'Freq [Hz]':<10} | {'Pred Gain':<10} | {'Tgt Gain':<10} | {'Gain Diff':<10} | {'Pred Phase':<10} | {'Tgt Phase':<10} | {'p1 Gain':<9} | {'p2 Gain':<9} | {'Score':<8}")
    print("-" * 110)
    tot_bode = 0.0
    per_freq = {}
    for f in freqs:
        sg, sp, p1_g, p2_g, p1_a, p2_a = closed_loop_tf_6dof(x_opt, f)
        tg, tp = targets[f]
        g_diff = 20.0 * math.log10(max(1e-6, sg) / tg)
        p_diff = (sp - tp + 180.0) % 360.0 - 180.0
        sc = ((g_diff / 1.0) ** 2) * 100.0 + ((p_diff / 10.0) ** 2) * 100.0
        tot_bode += sc
        per_freq[str(f)] = {
            "pred_gain_deg_m": round(sg, 2),
            "target_gain_deg_m": tg,
            "pred_phase_deg": round(sp, 2),
            "target_phase_deg": tp,
            "p1_gain_deg_m": round(p1_g, 2),
            "p2_gain_deg_m": round(p2_g, 2),
            "p1_amp_deg": round(p1_a, 2),
            "p2_amp_deg": round(p2_a, 2),
            "score": round(sc, 1)
        }
        print(f"{f:<10.3f} | {sg:<10.1f} | {tg:<10.1f} | {g_diff:+8.2f}dB | {sp:<9.1f}° | {tp:<9.1f}° | {p1_g:<9.1f} | {p2_g:<9.1f} | {sc:<8.1f}")
    print("-" * 110)
    print(f"TOTAL PREDICTED 6-DOF SURROGATE BODE SCORE: {tot_bode:.1f} pts")

    K_ton, p_ton, K_sc, r_lead_sc, p_sc, K_phas, p_phas, kp, r_lead_ccr, p_ccr, k_dyn, k_stat, Kp_task, Kd_task, Ki_task = x_opt
    opt_data = {
        "surrogate_model": "Maxwell-Hill 15-Parameter 6-DOF 72-Muscle MIMO Surrogate",
        "bode_score_surrogate": round(tot_bode, 2),
        "regularized_score": round(float(best_score), 2),
        "params_15d": [round(float(v), 6) for v in x_opt],
        "params": [
            round(float(K_ton), 6), round(float(p_ton), 6),
            round(float(K_sc), 6), round(float(r_lead_sc * p_sc), 6), round(float(p_sc), 6),
            round(float(K_phas), 6), round(float(p_phas), 6),
            round(float(kp), 6), round(float(r_lead_ccr * p_ccr), 6), round(float(p_ccr), 6),
        ],
        "r_ton": round(float(K_ton / K_sc), 6),
        "r_lead_sc": round(float(r_lead_sc), 6),
        "r_lead_ccr": round(float(r_lead_ccr), 6),
        "k_dyn": round(float(k_dyn), 6),
        "k_stat": round(float(k_stat), 6),
        "Kp_task_pitch": round(float(Kp_task), 6),
        "Kd_task_pitch": round(float(Kd_task), 6),
        "Ki_task_pitch": round(float(Ki_task), 6),
        "per_frequency_predictions": per_freq
    }
    out_sol = os.path.join(base_dir, "exact_plant_surrogate_solution.json")
    with open(out_sol, "w") as fp:
        json.dump(opt_data, fp, indent=2)
    print(f"Saved optimal 15D 6-DOF MIMO surrogate solution to {out_sol}")

if __name__ == "__main__":
    main()
