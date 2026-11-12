import os
import sys
import math
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0
from run_roll_yaw_baseline import (
    P15_RUN4,
    load_aligned_data,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    build_c2_multisine_spline,
)


def build_c2_zero_offset_spline_yaw(freqs, amps, phases, delay, ramp_duration, target_time, spline_dt=0.0025):
    """
    Same C^2 acceleration-windowed spline as build_c2_multisine_spline, plus
    zero-velocity-integral antisymmetric basis phi_x(tau) = tau^2*(1-tau)^2*(0.5 - tau)
    so that both velocity drift AND position offset x_offset == 0.000000!
    """
    ms_spline = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        ms_spline.addPoint(float(t), 0.0)

    omegas = 2.0 * np.pi * freqs
    sub = 10
    dt_fine = spline_dt / sub
    N_fine = int(round(ramp_duration / dt_fine))
    t_fine = np.linspace(0.0, ramp_duration, N_fine + 1)
    tau_fine = t_fine / ramp_duration
    w_fine = 10.0 * (tau_fine ** 3) - 15.0 * (tau_fine ** 4) + 6.0 * (tau_fine ** 5)

    phi_v = (tau_fine ** 2) * ((1.0 - tau_fine) ** 2)
    phi_v /= np.sum(0.5 * (phi_v[:-1] + phi_v[1:]) * dt_fine)

    psi_fine = np.outer(t_fine, omegas) + phases
    y_fine = np.sin(psi_fine) @ amps
    dy_fine = np.cos(psi_fine) @ (amps * omegas)
    ddy_fine = -np.sin(psi_fine) @ (amps * (omegas ** 2))

    ddx_raw = w_fine * ddy_fine
    v_raw_end = float(np.sum(0.5 * (ddx_raw[:-1] + ddx_raw[1:]) * dt_fine))
    dv_match = float(dy_fine[-1]) - v_raw_end
    ddx_v = ddx_raw + dv_match * phi_v

    dx_v = np.zeros_like(t_fine)
    dx_v[1:] = np.cumsum(0.5 * (ddx_v[:-1] + ddx_v[1:]) * dt_fine)
    x_v = np.zeros_like(t_fine)
    x_v[1:] = np.cumsum(0.5 * (dx_v[:-1] + dx_v[1:]) * dt_fine)
    x_err = float(y_fine[-1] - x_v[-1])

    # Antisymmetric basis phi_x has zero net velocity change and unit position change at end of ramp:
    phi_x = (tau_fine ** 2) * ((1.0 - tau_fine) ** 2) * (0.5 - tau_fine)
    v_phi_x = np.zeros_like(t_fine)
    v_phi_x[1:] = np.cumsum(0.5 * (phi_x[:-1] + phi_x[1:]) * dt_fine)
    x_phi_x = float(np.sum(0.5 * (v_phi_x[:-1] + v_phi_x[1:]) * dt_fine))
    phi_x /= x_phi_x

    ddx_fine = ddx_v + x_err * phi_x
    dx_fine = np.zeros_like(t_fine)
    dx_fine[1:] = np.cumsum(0.5 * (ddx_fine[:-1] + ddx_fine[1:]) * dt_fine)
    x_fine = np.zeros_like(t_fine)
    x_fine[1:] = np.cumsum(0.5 * (dx_fine[:-1] + dx_fine[1:]) * dt_fine)
    x_offset = float(x_fine[-1] - y_fine[-1])

    N_ramp_knots = int(round(ramp_duration / spline_dt))
    for i_k in range(N_ramp_knots):
        t = delay + i_k * spline_dt
        ms_spline.addPoint(float(t), float(x_fine[i_k * sub]))
    for t in np.arange(delay + ramp_duration, target_time + 0.02, spline_dt):
        dt_rel = t - delay
        y = float(np.sum(amps * np.sin(omegas * dt_rel + phases)))
        ms_spline.addPoint(float(t), float(y + x_offset))

    return ms_spline, x_offset


def evaluate_axis_candidate(args):
    mode, cand_id, cand_params, eval_cycles = args
    ensure_libraries_loaded()
    df_latec, df_yawma = load_aligned_data()
    df_target = df_latec if mode == "LatEC" else df_yawma

    params = dict(P15_RUN4)
    params.update(cand_params)

    model = osim.Model(os.path.join(_ROOT, "osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim"))
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    # Base Pitch parameters (Run 4 champion - untouched)
    osim.PropertyHelper.setValueDouble(params["K_ton"], controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(params["p_ton"], controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(params["K_sc"], controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(params["z_sc"], controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(params["p_sc"], controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(params["K_phas"], controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(params["z_phas"], controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(params["p_phas"], controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(-1.0, controller.updPropertyByName("p_phas2"))
    osim.PropertyHelper.setValueDouble(params["kp_prop"], controller.updPropertyByName("k_p"))

    osim.PropertyHelper.setValueDouble(params["Kp_pitch"], controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Kd_pitch"], controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Kp_roll"], controller.updPropertyByName("Kp_task_roll"))
    osim.PropertyHelper.setValueDouble(params["Kd_roll"], controller.updPropertyByName("Kd_task_roll"))
    osim.PropertyHelper.setValueDouble(params["Kp_yaw"], controller.updPropertyByName("Kp_task_yaw"))
    osim.PropertyHelper.setValueDouble(params["Kd_yaw"], controller.updPropertyByName("Kd_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    osim.PropertyHelper.setValueDouble(params["Ki_pitch"], controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(params["Ki_pitch"], controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(params.get("Ki_roll", 0.0), controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(params.get("Ki_yaw", 0.0), controller.updPropertyByName("Ki_task_yaw"))

    osim.PropertyHelper.setValueDouble(params["kp_prop"], controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(params["z_ccr"], controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(params["p_ccr"], controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(params["k_gamma_dyn"], controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(params["k_gamma_stat"], controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))

    # Per-axis Roll & Yaw VCR/CCR properties
    per_axis_keys = [
        "K_ton_roll", "K_ton_yaw", "p_ton_roll", "p_ton_yaw",
        "K_sc_roll", "K_sc_yaw", "z_sc_roll", "z_sc_yaw", "p_sc_roll", "p_sc_yaw",
        "K_phas_roll", "z_phas_roll", "p_phas_roll", "p_ap_roll", "c_ap_roll",
        "Kp_prop_roll", "Kp_prop_yaw", "z_ccr_roll", "z_ccr_yaw", "p_ccr_roll", "p_ccr_yaw",
        "deriv_comp_roll", "deriv_comp_yaw"
    ]
    for pk in per_axis_keys:
        if pk in params:
            osim.PropertyHelper.setValueDouble(float(params[pk]), controller.updPropertyByName(pk))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    freqs = df_target["Frequency_Hz"]
    K = len(freqs)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])

    delay = 1.5
    ramp_duration = 1.5
    eval_duration = eval_cycles / freqs[0]
    target_time = delay + ramp_duration + eval_duration

    if mode == "LatEC":
        amps_input_unit = get_latec_amplitudes(freqs)
        amps_sim_unit = amps_input_unit.copy()
        prescribed_coord_name = "spine_tz"
        tgt_gains = df_target["Exp_Roll_Gain_deg_per_m"]
        tgt_phases = df_target["Exp_Roll_Phase_deg"]
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
    else:
        amps_input_unit = get_yawma_amplitudes_deg(freqs)
        amps_sim_unit = np.radians(amps_input_unit)
        prescribed_coord_name = "gndyaw"
        tgt_gains = df_target["Exp_Yaw_Gain_deg_per_deg"]
        tgt_phases = df_target["Exp_Yaw_Phase_deg"]
        ms_spline, _ = build_c2_zero_offset_spline_yaw(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)

    coord_pert = model.updCoordinateSet().get(prescribed_coord_name)
    coord_pert.set_locked(False)
    coord_pert.setDefaultIsPrescribed(True)
    coord_pert.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q.get(idx))

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        k_gdyn_eff = min(max(params["k_gamma_dyn"], 0.0), 1.0)
        k_gstat_eff = min(max(params["k_gamma_stat"], 0.0), 1.0)
        gd0 = 100.0 * k_gdyn_eff * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gstat_eff * a0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{name}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{name}", fstat0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setWriteToStorage(False)
    manager.setPerformAnalyses(False)
    manager.setIntegratorMethod(5)
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.001)

    state.setTime(0.0)
    manager.initialize(state)

    step = 0.005
    current_time = 0.0

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")
    coord_gndyaw = model.getCoordinateSet().get("gndyaw")

    t_list, pitch_list, roll_list, yaw_rel_list, yaw_glob_list = [], [], [], [], []
    aborted = False

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        p_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        r_deg = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        y_rel_deg = math.degrees(coord_y1.getValue(state) + coord_y2.getValue(state))
        gnd_y_deg = math.degrees(coord_gndyaw.getValue(state))
        y_glob_deg = gnd_y_deg + y_rel_deg

        t_list.append(current_time)
        pitch_list.append(p_deg)
        roll_list.append(r_deg)
        yaw_rel_list.append(y_rel_deg)
        yaw_glob_list.append(y_glob_deg)

        if abs(p_deg) > 45.0 or abs(r_deg) > 35.0 or abs(y_rel_deg) > 48.0:
            aborted = True
            break

    if aborted:
        return {"cand_id": cand_id, "aborted": True, "cost": 1e6}

    t_arr = np.array(t_list)
    roll_arr = np.array(roll_list)
    yaw_glob_arr = np.array(yaw_glob_list)

    m_post = t_arr >= (delay + ramp_duration)
    t_fit = t_arr[m_post]
    t_rel = t_fit - delay

    # Note: for LatEC, ISO sign convention is -roll_arr
    y_fit_deg = -roll_arr[m_post] if mode == "LatEC" else yaw_glob_arr[m_post]

    cols = [np.ones_like(t_fit), t_fit - (delay + ramp_duration)]
    for k in range(K):
        omega_k = 2.0 * math.pi * freqs[k]
        psi_k = omega_k * t_rel + phases[k]
        cols.append(np.sin(psi_k))
        cols.append(np.cos(psi_k))

    A_mat = np.column_stack(cols)
    coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit_deg, rcond=None)

    ref_fit = np.zeros_like(t_fit)
    sim_harm_fit = np.zeros_like(t_fit)
    for k in range(K):
        psi = 2.0 * np.pi * freqs[k] * t_rel + phases[k]
        ref_fit += amps_input_unit[k] * tgt_gains[k] * np.sin(psi + np.radians(tgt_phases[k]))
        a_sin = float(coeffs[2 + 2 * k])
        b_cos = float(coeffs[2 + 2 * k + 1])
        sim_harm_fit += a_sin * np.sin(psi) + b_cos * np.cos(psi)

    vel_actual = np.gradient(y_fit_deg, t_fit)
    vel_harm = np.gradient(sim_harm_fit, t_fit)
    vel_ref = np.gradient(ref_fit, t_fit)
    hf_noise_rms = float(np.sqrt(np.mean((vel_actual - vel_harm) ** 2)))
    corr_vel = float(np.corrcoef(vel_actual, vel_ref)[0, 1]) if np.std(vel_actual) > 1e-6 else 0.0

    gains = []
    phases_sim = []
    db_errs = []
    ph_errs = []

    for k in range(K):
        a_sin = float(coeffs[2 + 2 * k])
        b_cos = float(coeffs[2 + 2 * k + 1])
        amp_out_deg = math.hypot(a_sin, b_cos)
        sim_gain = amp_out_deg / float(amps_input_unit[k])
        phase_deg = math.degrees(math.atan2(b_cos, a_sin))
        tp = float(tgt_phases[k])
        ph_err = ((phase_deg - tp + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(sim_gain, 1e-4) / float(tgt_gains[k]))

        gains.append(sim_gain)
        phases_sim.append(tp + ph_err)
        db_errs.append(db_err)
        ph_errs.append(ph_err)

    if mode == "LatEC":
        gain_weights = [1.5, 1.5, 1.5, 1.0, 1.0, 1.0, 1.0, 1.0]
        gain_cost = sum(gain_weights[k] * (db_errs[k] ** 2) for k in range(K)) / sum(gain_weights)
        ph_weights = [2.2, 2.2, 1.8, 1.2, 1.2, 1.0, 1.0, 0.8]
        phase_cost = sum(ph_weights[k] * ((ph_errs[k] / 12.0) ** 2) for k in range(K)) / sum(ph_weights)
        noise_cost = 0.30 * (hf_noise_rms ** 2)
        cost = gain_cost + phase_cost + noise_cost
    else:
        gain_cost = sum((db_errs[k] ** 2) for k in range(K)) / K
        phase_cost = sum(((ph_errs[k] / 12.0) ** 2) for k in range(K)) / K
        noise_cost = 0.25 * (hf_noise_rms ** 2)
        cost = gain_cost + phase_cost + noise_cost

    return {
        "cand_id": cand_id,
        "aborted": False,
        "cost": float(cost),
        "gain_cost": float(gain_cost),
        "phase_cost": float(phase_cost),
        "hf_noise_rms": hf_noise_rms,
        "corr_vel": corr_vel,
        "gains": [round(g, 2) for g in gains],
        "phases": [round(p, 1) for p in phases_sim],
        "db_errs": [round(d, 2) for d in db_errs],
        "ph_errs": [round(e, 1) for e in ph_errs],
        "params": cand_params,
    }


def run_roll_and_yaw_surgical_sweep():
    roll_candidates = [
        ("R0_baseline", {}),
        ("R1_lowPID_zphas", {"Kp_roll": 2.5, "Kd_roll": 0.40, "K_phas_roll": 17.0, "z_phas_roll": 0.045, "z_sc_roll": 0.012, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
        ("R2_ap050_c55", {"Kp_roll": 2.5, "Kd_roll": 0.40, "K_phas_roll": 17.0, "z_phas_roll": 0.045, "z_sc_roll": 0.012, "p_ap_roll": 0.50, "c_ap_roll": 55.0, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
        ("R3_ap055_c75", {"Kp_roll": 2.5, "Kd_roll": 0.40, "K_phas_roll": 17.5, "z_phas_roll": 0.050, "z_sc_roll": 0.012, "p_ap_roll": 0.55, "c_ap_roll": 75.0, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
        ("R4_ap065_c65", {"Kp_roll": 2.0, "Kd_roll": 0.35, "K_phas_roll": 17.5, "z_phas_roll": 0.050, "z_sc_roll": 0.012, "p_ap_roll": 0.65, "c_ap_roll": 65.0, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
        ("R5_ap045_c90", {"Kp_roll": 2.0, "Kd_roll": 0.35, "K_phas_roll": 18.0, "z_phas_roll": 0.055, "z_sc_roll": 0.014, "p_ap_roll": 0.45, "c_ap_roll": 90.0, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
        ("R6_ap060_c50_ksc3", {"Kp_roll": 1.5, "Kd_roll": 0.30, "K_sc_roll": 3.2, "K_phas_roll": 16.5, "z_phas_roll": 0.048, "z_sc_roll": 0.015, "p_ap_roll": 0.60, "c_ap_roll": 50.0, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
        ("R7_ap075_c55_kp4", {"Kp_roll": 4.0, "Kd_roll": 0.55, "K_phas_roll": 18.0, "z_phas_roll": 0.052, "z_sc_roll": 0.012, "p_ap_roll": 0.75, "c_ap_roll": 55.0, "K_ton_roll": 0.0, "deriv_comp_roll": 1.0}),
    ]

    yaw_candidates = [
        ("Y1_clean_ccr8", {"K_sc_yaw": 0.30, "K_ton_yaw": 0.15, "Kp_yaw": 0.0, "Kd_yaw": 0.0, "Kp_prop_yaw": 8.0, "z_ccr_yaw": 0.14, "deriv_comp_yaw": 0.6}),
        ("Y2_xover_ksc25_p35", {"K_sc_yaw": 2.5, "p_sc_yaw": 3.5, "K_ton_yaw": 1.5, "p_ton_yaw": 2.5, "Kp_yaw": 0.0, "Kd_yaw": 0.0, "Kp_prop_yaw": 8.5, "z_ccr_yaw": 0.15, "deriv_comp_yaw": 0.6}),
        ("Y3_xover_ksc35_p28", {"K_sc_yaw": 3.5, "p_sc_yaw": 2.8, "K_ton_yaw": 2.0, "p_ton_yaw": 2.0, "Kp_yaw": 0.0, "Kd_yaw": 0.0, "Kp_prop_yaw": 9.5, "z_ccr_yaw": 0.16, "deriv_comp_yaw": 0.6}),
        ("Y4_xover_ksc20_p45", {"K_sc_yaw": 2.0, "p_sc_yaw": 4.5, "K_ton_yaw": 1.2, "p_ton_yaw": 3.0, "Kp_yaw": 0.0, "Kd_yaw": 0.0, "Kp_prop_yaw": 10.5, "z_ccr_yaw": 0.16, "deriv_comp_yaw": 0.8}),
        ("Y5_xover_ksc30_ccr11", {"K_sc_yaw": 3.0, "p_sc_yaw": 3.0, "K_ton_yaw": 2.5, "p_ton_yaw": 2.0, "Kp_yaw": 0.0, "Kd_yaw": 0.0, "Kp_prop_yaw": 11.5, "z_ccr_yaw": 0.17, "deriv_comp_yaw": 0.8}),
        ("Y6_xover_ksc15_ccr9", {"K_sc_yaw": 1.5, "p_sc_yaw": 3.5, "K_ton_yaw": 1.0, "p_ton_yaw": 2.5, "Kp_yaw": 0.0, "Kd_yaw": 0.0, "Kp_prop_yaw": 9.0, "z_ccr_yaw": 0.15, "deriv_comp_yaw": 0.5}),
    ]

    tasks = [("LatEC", name, p, 2.0) for name, p in roll_candidates] + [("YawMA", name, p, 2.0) for name, p in yaw_candidates]
    roll_res, yaw_res = [], []
    with ProcessPoolExecutor(max_workers=14) as ex:
        futs = [ex.submit(evaluate_axis_candidate, t) for t in tasks]
        for f in as_completed(futs):
            res = f.result()
            if res["cand_id"].startswith("R"):
                roll_res.append(res)
            else:
                yaw_res.append(res)
            print(f"[{res['cand_id']}] cost={res['cost']:.2f} (G={res.get('gain_cost',0):.2f}, P={res.get('phase_cost',0):.2f}, noise={res.get('hf_noise_rms',0):.2f} deg/s, corr={res.get('corr_vel',0):.3f})")
            print(f"   Gains : {res.get('gains')}")
            print(f"   dBErrs: {res.get('db_errs')}")
            print(f"   PhErrs: {res.get('ph_errs')}")

    roll_res.sort(key=lambda x: x["cost"])
    yaw_res.sort(key=lambda x: x["cost"])
    print("\n=== TOP ROLL CANDIDATES ===")
    for r in roll_res:
        print(f"{r['cand_id']:20s} | cost={r['cost']:6.2f} | noise={r.get('hf_noise_rms',0):4.2f} | corr={r.get('corr_vel',0):.3f} | PhErrs={r['ph_errs']} | dBErrs={r['db_errs']}")
    print("\n=== TOP YAW CANDIDATES ===")
    for r in yaw_res:
        print(f"{r['cand_id']:20s} | cost={r['cost']:6.2f} | noise={r.get('hf_noise_rms',0):4.2f} | corr={r.get('corr_vel',0):.3f} | PhErrs={r['ph_errs']} | dBErrs={r['db_errs']}")


if __name__ == "__main__":
    run_roll_and_yaw_surgical_sweep()
