import os
import sys
import math
import json
import shutil
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0
from cmaes_worker_multisine import run_multisine_simulation
from evaluate_pure_converged_simulation import compute_ref_pitch_analytic
from run_roll_yaw_baseline import (
    ART_DIR,
    P15_RUN4,
    load_aligned_data,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    generate_plausibility_plots,
    build_c2_multisine_spline,
    render_baseline_4panel,
)
from tune_roll_and_yaw import build_c2_zero_offset_spline_yaw


def run_full_axis_sim(args):
    mode, label, params, eval_cycles = args
    ensure_libraries_loaded()
    df_lat, df_yaw = load_aligned_data()
    df_target = df_lat if mode == "LatEC" else df_yaw

    model = osim.Model(os.path.join(_ROOT, "osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim"))
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

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

    per_axis_keys = [
        "K_ton_roll", "K_ton_yaw", "p_ton_roll", "p_ton_yaw",
        "K_sc_roll", "K_sc_yaw", "z_sc_roll", "z_sc_yaw", "p_sc_roll", "p_sc_yaw",
        "K_phas_roll", "z_phas_roll", "p_phas_roll", "p_ap_roll", "c_ap_roll",
        "Kp_prop_roll", "Kp_prop_yaw", "z_ccr_roll", "z_ccr_yaw", "p_ccr_roll", "p_ccr_yaw",
        "deriv_comp_roll", "deriv_comp_yaw",
    ]
    for pk in per_axis_keys:
        if pk in params and params[pk] is not None:
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
    coord_tz = model.getCoordinateSet().get("spine_tz")

    t_list, pitch_list, roll_list, yaw_rel_list, yaw_glob_list, pert_list = [], [], [], [], [], []
    aborted = False

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        p_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        r_deg = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        y_rel_deg = math.degrees(coord_y1.getValue(state) + coord_y2.getValue(state))
        gnd_y_deg = math.degrees(coord_gndyaw.getValue(state))
        y_glob_deg = gnd_y_deg + y_rel_deg
        pert_val = coord_tz.getValue(state) if mode == "LatEC" else gnd_y_deg

        t_list.append(current_time)
        pitch_list.append(p_deg)
        roll_list.append(r_deg)
        yaw_rel_list.append(y_rel_deg)
        yaw_glob_list.append(y_glob_deg)
        pert_list.append(pert_val)

        if abs(p_deg) > 45.0 or abs(r_deg) > 35.0 or abs(y_rel_deg) > 48.0:
            aborted = True
            break

    t_arr = np.array(t_list)
    pitch_arr = np.array(pitch_list)
    roll_arr = np.array(roll_list)
    yaw_rel_arr = np.array(yaw_rel_list)
    yaw_glob_arr = np.array(yaw_glob_list)
    pert_arr = np.array(pert_list)

    m_post = t_arr >= (delay + ramp_duration)
    t_fit = t_arr[m_post]
    t_rel = t_fit - delay

    # ISO sign convention for LatEC is -roll_arr
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

    freq_results = {}
    for k in range(K):
        f_k = float(freqs[k])
        a_sin = float(coeffs[2 + 2 * k])
        b_cos = float(coeffs[2 + 2 * k + 1])
        amp_out_deg = math.hypot(a_sin, b_cos)
        sim_gain = amp_out_deg / float(amps_input_unit[k])
        phase_deg = math.degrees(math.atan2(b_cos, a_sin))
        tp = float(tgt_phases[k])
        ph_err = ((phase_deg - tp + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k]))

        # Also store raw-roll phase sign for baseline compatibility:
        phase_deg_raw = math.degrees(math.atan2(-b_cos, -a_sin)) if mode == "LatEC" else phase_deg
        err_raw = ((phase_deg_raw - tp + 180.0) % 360.0) - 180.0

        freq_results[f"{f_k:.4f}"] = {
            "freq_hz": f_k,
            "input_amp": float(amps_input_unit[k]),
            "output_amp_deg": amp_out_deg,
            "sim_gain": sim_gain,
            "target_gain": float(tgt_gains[k]),
            "gain_db_diff": db_err,
            "sim_phase": tp + ph_err,
            "target_phase": tp,
            "phase_err": ph_err,
            "sim_phase_raw": tp + err_raw,
            "sim_phase_inv": tp + ph_err,
            "phase_err_raw": err_raw,
            "phase_err_inv": ph_err,
        }

    return {
        "mode": mode,
        "label": label,
        "aborted": aborted,
        "t": t_arr,
        "pitch": pitch_arr,
        "roll": roll_arr,
        "yaw_rel": yaw_rel_arr,
        "yaw_glob": yaw_glob_arr,
        "pert": pert_arr,
        "freqs": freqs,
        "amps_input_unit": amps_input_unit,
        "phases": phases,
        "tgt_gains": tgt_gains,
        "tgt_phases": tgt_phases,
        "freq_results": freq_results,
        "hf_noise_rms": hf_noise_rms,
        "corr_vel": corr_vel,
    }


def run_apeo_pitch_check(params):
    os.environ["SAVE_TS"] = "1"
    os.environ["MULTISINE_EXP"] = "1.0"
    os.environ["MULTISINE_MIN_AMP_M"] = "0.0"
    os.environ["NOISE_FLOOR_7927_DEG"] = "0.0"
    os.environ["MULTISINE_CYCLES"] = "6.0"

    worker_label = "tuned_3axis_apeo_pitch"
    total_score, freq_results, telemetry = run_multisine_simulation(
        params["Kp_pitch"], params["Ki_pitch"], params["Kd_pitch"],
        params["K_ton"], params["p_ton"], params["K_sc"], params["z_sc"], params["p_sc"],
        params["K_phas"], params["p_phas"], params["kp_prop"], params["z_ccr"], params["p_ccr"],
        params["k_gamma_dyn"], params["k_gamma_stat"],
        worker_name=worker_label, p_phas2=-1.0, z_phas=params["z_phas"],
        extra_props=params,
    )
    return {
        "worker_label": worker_label,
        "total_score": total_score,
        "freq_results": freq_results,
        "telemetry": telemetry,
    }


def render_tuned_comparison_4panel(sim_zero, sim_tuned, out_filename, apeo_score):
    mode = sim_tuned["mode"]
    t = sim_tuned["t"]
    pitch = sim_tuned["pitch"]
    roll = sim_tuned["roll"]
    yaw_rel = sim_tuned["yaw_rel"]
    yaw_glob = sim_tuned["yaw_glob"]
    freqs = sim_tuned["freqs"]
    amps = sim_tuned["amps_input_unit"]
    phases = sim_tuned["phases"]
    tgt_gains = sim_tuned["tgt_gains"]
    tgt_phases = sim_tuned["tgt_phases"]
    freq_res_tuned = sim_tuned["freq_results"]
    freq_res_zero = sim_zero["freq_results"]

    delay, ramp = 1.5, 1.5
    ref_trace = np.zeros_like(t)
    for i, ti in enumerate(t):
        if ti < delay:
            continue
        t_rel = ti - delay
        w = 1.0
        if t_rel < ramp:
            tau = t_rel / ramp
            w = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        y_deg = 0.0
        for k in range(len(freqs)):
            psi = 2.0 * np.pi * freqs[k] * t_rel + phases[k] + np.radians(tgt_phases[k])
            y_deg += amps[k] * tgt_gains[k] * np.sin(psi)
        ref_trace[i] = w * y_deg

    main_angle_tuned = -roll if mode == "LatEC" else yaw_glob
    main_angle_zero = -sim_zero["roll"] if mode == "LatEC" else sim_zero["yaw_glob"]
    main_vel_tuned = np.gradient(main_angle_tuned, t)
    main_vel_zero = np.gradient(main_angle_zero, sim_zero["t"])
    ref_vel = np.gradient(ref_trace, t)

    mask_ss = t >= 3.0
    mean_ang_ss = float(np.mean(main_angle_tuned[mask_ss]))
    rms_vel_ss = float(np.sqrt(np.mean(main_vel_tuned[mask_ss] ** 2)))
    ref_rms_v = float(np.sqrt(np.mean(ref_vel[mask_ss] ** 2)))
    corr_vel_tuned = sim_tuned["corr_vel"]
    corr_vel_zero = sim_zero["corr_vel"]
    noise_tuned = sim_tuned["hf_noise_rms"]
    noise_zero = sim_zero["hf_noise_rms"]

    sim_gains_tuned = [freq_res_tuned[f"{f:.4f}"]["sim_gain"] for f in freqs]
    sim_phases_tuned = [freq_res_tuned[f"{f:.4f}"]["sim_phase"] for f in freqs]
    sim_gains_zero = [freq_res_zero[f"{f:.4f}"]["sim_gain"] for f in freqs]
    sim_phases_zero = [freq_res_zero[f"{f:.4f}"]["sim_phase"] for f in freqs]

    color_main = "#2563eb" if mode == "LatEC" else "#059669"
    axis_name = "Head Roll" if mode == "LatEC" else "Global Head Yaw"
    gain_unit = "deg/m" if mode == "LatEC" else "deg/deg"

    fig = plt.figure(figsize=(15, 14.5), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[1.1, 1.1, 0.85, 1.20], hspace=0.33)

    # Subplot 1: Angle vs Time
    pert_vel = np.gradient(sim_tuned["pert"], t)
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(sim_zero["t"], main_angle_zero, color="#94a3b8", lw=1.3, alpha=0.65, label=f"Zero-Shot Baseline {axis_name}")
    ax1.plot(t, ref_trace, "k--", lw=1.6, label=f"Target Happee (2023) Reference {axis_name} [Max = {np.max(np.abs(ref_trace[mask_ss])):.2f}°]")
    ax1.plot(t, main_angle_tuned, color=color_main, lw=2.1, label=f"Tuned 3-Axis Model {axis_name} [Mean = {mean_ang_ss:+.2f}°, Max = {np.max(np.abs(main_angle_tuned[mask_ss])):.2f}°]")
    if mode == "YawMA":
        ax1.plot(t, sim_tuned["pert"], color="#64748b", ls=":", lw=1.2, alpha=0.8, label=f"Applied Trunk Yaw Input gndyaw(t) [Max = {np.max(np.abs(sim_tuned['pert'][mask_ss])):.2f}°]")
    ax1.axvline(1.5, color="#64748b", ls=":", lw=1.3, label="Perturbation Ramp Start (t = 1.5s)")
    ax1.axvline(3.0, color="#dc2626", ls="--", lw=1.4, label="Steady-State Window (t >= 3.0s)")
    ax1.set_xlim(0, t[-1])
    ax1.set_ylabel(f"{axis_name} [deg]", fontweight="bold", fontsize=11)
    ax1.set_title(f"1. {axis_name} Angle vs. Time — Tuned 3-Axis Model vs. Zero-Shot ({mode}, Duration: {t[-1]:.2f}s)", fontweight="bold", fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8.6)

    # Subplot 2: Angular Velocity vs Time
    ax2 = fig.add_subplot(gs[1])
    if mode == "LatEC":
        ax2.axhspan(-10, 10, color="#f1f5f9", alpha=0.9, label=r"Happee Fig. 5 Roll Rate Envelope ($\pm 10^\circ$/s; Seat $\dot{Y}$ Peak = " + f"{np.max(np.abs(pert_vel[mask_ss])):.3f} m/s)")
    else:
        ax2.axhspan(-15, 15, color="#f1f5f9", alpha=0.9, label=r"Target Yaw Rate Envelope ($\pm 15^\circ$/s; Trunk $\dot{\psi}_{\mathrm{T1}}$ Peak = " + f"{np.max(np.abs(pert_vel[mask_ss])):.1f}°/s)")
    ax2.axhline(0, color="#94a3b8", lw=0.8)
    ax2.plot(sim_zero["t"], main_vel_zero, color="#94a3b8", lw=1.1, alpha=0.55, label=f"Zero-Shot Velocity [HF Noise = {noise_zero:.2f}°/s, corr r = {corr_vel_zero:+.3f}]")
    ax2.plot(t, ref_vel, "k--", lw=1.5, alpha=0.85, label=f"Target Happee Reference Velocity [RMS = {ref_rms_v:.2f}°/s]")
    ax2.plot(t, main_vel_tuned, color=color_main, lw=1.9, label=f"Tuned 3-Axis Velocity [RMS = {rms_vel_ss:.2f}°/s, HF Noise = {noise_tuned:.2f}°/s, corr r = {corr_vel_tuned:+.3f}]")
    ax2.axvline(1.5, color="#64748b", ls=":", lw=1.3)
    ax2.axvline(3.0, color="#dc2626", ls="--", lw=1.4)
    ax2.set_xlim(0, t[-1])
    ax2.set_ylabel(f"{axis_name} Vel [°/s]", fontweight="bold", fontsize=11)
    ax2.set_title(f"2. {axis_name} Angular Velocity vs. Time ({mode})", fontweight="bold", fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8.4)

    # Subplot 3: Cross-Axis Stability
    ax3 = fig.add_subplot(gs[2])
    ax3.axhline(0, color="#94a3b8", lw=0.8)
    if mode == "LatEC":
        ax3.plot(t, pitch - pitch[0], color="#059669", lw=1.4, label=f"Pitch Deviation (p1+p2 - p0) [Max = {np.max(np.abs(pitch - pitch[0])):.2f}°]")
        ax3.plot(t, yaw_rel, color="#d97706", lw=1.4, label=f"Coupled Neck Yaw (y1+y2) [Max = {np.max(np.abs(yaw_rel)):.2f}°]")
    else:
        ax3.plot(t, pitch - pitch[0], color="#059669", lw=1.4, label=f"Pitch Deviation (p1+p2 - p0) [Max = {np.max(np.abs(pitch - pitch[0])):.2f}°]")
        ax3.plot(t, roll, color="#dc2626", lw=1.4, label=f"Coupled Head Roll (r1+r2) [Max = {np.max(np.abs(roll)):.2f}°]")
        ax3.plot(t, yaw_rel, color="#64748b", ls="--", lw=1.2, label=f"Relative Neck-on-Trunk Yaw (y1+y2) [Max = {np.max(np.abs(yaw_rel)):.2f}°]")
    ax3.axvline(1.5, color="#64748b", ls=":", lw=1.3)
    ax3.axvline(3.0, color="#dc2626", ls="--", lw=1.4)
    ax3.set_xlim(0, t[-1])
    ax3.set_ylabel("Cross-Axes [deg]", fontweight="bold", fontsize=11)
    ax3.set_title(f"3. 6-DOF Cross-Axis Coupling & Stability During {mode}", fontweight="bold", fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", fontsize=8.8)

    # Subplot 4: Bode Plot
    ax4 = fig.add_subplot(gs[3])
    ax4_r = ax4.twinx()

    ax4.plot(freqs, tgt_gains, "ko--", lw=1.8, label=f"Target Exp Gain (Happee 2023) [{gain_unit}]")
    ax4.plot(freqs, sim_gains_zero, "o:", color="#64748b", lw=1.4, alpha=0.65, label=f"Zero-Shot Gain [{gain_unit}]")
    ax4.plot(freqs, sim_gains_tuned, "s-", color=color_main, lw=2.3, label=f"Tuned 3-Axis Gain [{gain_unit}]")

    ax4_r.plot(freqs, tgt_phases, "^--", color="#92400e", lw=1.8, label="Target Exp Phase (Happee 2023) [deg]")
    ax4_r.plot(freqs, sim_phases_zero, "d:", color="#f87171", lw=1.4, alpha=0.60, label="Zero-Shot Phase [deg]")
    ax4_r.plot(freqs, sim_phases_tuned, "D-", color="#b91c1c", lw=2.1, label="Tuned 3-Axis Phase [deg]")

    for f, tg, mg, tp, mp in zip(freqs, tgt_gains, sim_gains_tuned, tgt_phases, sim_phases_tuned):
        db = 20.0 * np.log10(max(mg, 1e-6) / tg)
        pe = mp - tp
        ax4.annotate(f"{db:+.1f} dB", (f, mg), textcoords="offset points", xytext=(0, 9),
                     ha="center", fontsize=8.2, fontweight="bold", color=color_main)
        ax4_r.annotate(f"{pe:+.1f}°", (f, mp), textcoords="offset points", xytext=(0, -14),
                       ha="center", fontsize=8.2, fontweight="bold", color="#b91c1c")

    ax4.set_xscale("log")
    ax4.xaxis.set_minor_locator(ticker.NullLocator())
    ax4.xaxis.set_minor_formatter(ticker.NullFormatter())
    ax4.minorticks_off()
    ax4.set_xticks(freqs)
    ax4.set_xticklabels([f"{f:.2f} Hz" for f in freqs], fontweight="bold", fontsize=9.0)

    if mode == "LatEC":
        ax4.set_ylim(0, max(650.0, float(np.max(sim_gains_tuned)) * 1.18))
        ax4_r.set_ylim(-420, -150)
    else:
        ax4.set_ylim(0, max(2.1, float(np.max(sim_gains_tuned)) * 1.18))
        ax4_r.set_ylim(-160, 60)

    ax4.set_ylabel(f"Bode Gain [{gain_unit}]", color=color_main, fontweight="bold", fontsize=11)
    ax4_r.set_ylabel("Bode Phase [deg]", color="#b91c1c", fontweight="bold", fontsize=11)
    ax4.set_xlabel("Multisine Excitation Frequency [Hz] (Log Scale)", fontweight="bold", fontsize=11)
    ax4.set_title(f"4. Tuned vs. Zero-Shot {len(freqs)}-Frequency Bode Response ({mode})", fontweight="bold", fontsize=12)
    ax4.grid(True, alpha=0.3)

    l1, lb1 = ax4.get_legend_handles_labels()
    l2, lb2 = ax4_r.get_legend_handles_labels()
    ax4.legend(l1 + l2, lb1 + lb2, loc="upper left", fontsize=8.0, ncol=2)

    fig.suptitle(f"Surgically Tuned 3-Axis {mode} Evaluation vs. Zero-Shot Baseline (APEO Pitch = {apeo_score:,.2f} pts)", fontweight="bold", fontsize=13, y=0.98)
    out_art = os.path.join(ART_DIR, out_filename)
    fig.savefig(out_art, bbox_inches="tight")
    shutil.copyfile(out_art, os.path.join(_ROOT, out_filename))
    plt.close(fig)
    print(f"[OK] Saved 4-panel tuned comparison plot: {out_art}", flush=True)


def render_apeo_pitch_4panel(apeo_res, out_filename="tuned_3axis_apeo_pitch_4panel.png"):
    worker_label = apeo_res["worker_label"]
    total_score = apeo_res["total_score"]
    freq_results = apeo_res["freq_results"]
    ts_file = os.path.join(_ROOT, "logs", "candidates_multisine", f"{worker_label}_ts.npz")
    ts = np.load(ts_file)
    t = ts["t"]
    pitch = ts["pitch"]
    roll = ts["roll"]
    yaw = ts["yaw"]

    ref_p = compute_ref_pitch_analytic(t)
    ref_v = np.gradient(ref_p, t)
    vel = np.gradient(pitch, t)

    mask_ss = t >= 3.0
    mean_pitch_ss = float(np.mean(pitch[mask_ss]))
    pitch_3s = float(pitch[np.argmin(np.abs(t - 3.0))])
    rms_vel_ss = float(np.sqrt(np.mean(vel[mask_ss] ** 2)))
    ref_rms_v = float(np.sqrt(np.mean(ref_v[mask_ss] ** 2)))
    corr_vel_ss = float(np.corrcoef(vel[mask_ss], ref_v[mask_ss])[0, 1])
    max_roll = float(np.max(np.abs(roll)))
    max_yaw = float(np.max(np.abs(yaw)))

    freqs = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]
    tgt_gains = [86.55, 236.93, 319.74, 423.34, 503.30, 572.69]
    tgt_phases = [-7.03, -96.96, -109.23, -161.34, -201.38, -247.62]
    sim_gains = [freq_results[f"{f:.3f}"]["sim_gain"] for f in freqs]
    sim_phases = []
    for f, tp in zip(freqs, tgt_phases):
        raw_p = freq_results[f"{f:.3f}"]["sim_phase"]
        unwrapped_p = ((raw_p - tp + 180.0) % 360.0) - 180.0 + tp
        sim_phases.append(unwrapped_p)

    color_main = "#059669"
    fig = plt.figure(figsize=(15, 14), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[1.1, 1.1, 0.85, 1.15], hspace=0.30)

    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, ref_p, "k--", lw=1.6, label="Ideal Happee Target Reference Pitch (0° mean)")
    ax1.plot(t, pitch, color=color_main, lw=2.0, label=f"Unified 3-Axis Model [Mean = {mean_pitch_ss:+.2f}°, Pitch(3.0s) = {pitch_3s:+.2f}°]")
    ax1.axvline(1.5, color="#64748b", ls=":", lw=1.3, label="Perturbation Ramp Start (t = 1.5s)")
    ax1.axvline(3.0, color="#dc2626", ls="--", lw=1.4, label="Steady-State Window (t >= 3.0s)")
    ax1.set_xlim(0, t[-1])
    ax1.set_ylabel("Pitch Angle [deg]", fontweight="bold", fontsize=11)
    ax1.set_title(f"1. Head Pitch Angle vs. Time — Unified 3-Axis Model (Duration: {t[-1]:.2f}s)", fontweight="bold", fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)

    ax2 = fig.add_subplot(gs[1])
    ax2.axhspan(-50, 50, color="#f1f5f9", alpha=0.9, label=r"Happee Characteristic Operating Envelope ($\pm 50^\circ$/s)")
    ax2.axhline(0, color="#94a3b8", lw=0.8)
    ax2.plot(t, ref_v, "k--", lw=1.5, alpha=0.85, label=f"Ideal Target Ref Velocity [RMS = {ref_rms_v:.1f}°/s]")
    ax2.plot(t, vel, color=color_main, lw=1.8, label=f"Unified 3-Axis Velocity [RMS = {rms_vel_ss:.1f}°/s, corr r = {corr_vel_ss:+.3f}]")
    ax2.axvline(1.5, color="#64748b", ls=":", lw=1.3)
    ax2.axvline(3.0, color="#dc2626", ls="--", lw=1.4)
    ax2.set_xlim(0, t[-1])
    ax2.set_ylim(-110, 115)
    ax2.set_ylabel(r"Pitch Velocity $\dot{\theta}$ [°/s]", fontweight="bold", fontsize=11)
    ax2.set_title(r"2. Head Angular Pitch Velocity vs. Time (APEO Verification)", fontweight="bold", fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8.8)

    ax3 = fig.add_subplot(gs[2])
    ax3.axhline(0, color="#94a3b8", lw=0.8)
    ax3.plot(t, roll, color="#dc2626", lw=1.4, label=f"Head Roll Error (r1+r2) [Max |Roll| = {max_roll:.2f}°]")
    ax3.plot(t, yaw, color="#15803d", lw=1.4, label=f"Head Yaw Error (y1+y2) [Max |Yaw| = {max_yaw:.2f}°]")
    ax3.axvline(1.5, color="#64748b", ls=":", lw=1.3)
    ax3.axvline(3.0, color="#dc2626", ls="--", lw=1.4)
    ax3.set_xlim(0, t[-1])
    ax3.set_ylim(-1.5, 1.5)
    ax3.set_ylabel("Roll / Yaw [deg]", fontweight="bold", fontsize=11)
    ax3.set_title("3. Head 6-DOF Cross-Axis Stability During Sagittal APEO", fontweight="bold", fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", fontsize=9)

    ax4 = fig.add_subplot(gs[3])
    ax4_r = ax4.twinx()
    ax4.plot(freqs, tgt_gains, "ko--", lw=1.8, label="Target Bode Gain (Happee 2017) [deg/m]")
    ax4.plot(freqs, sim_gains, "s-", color=color_main, lw=2.2, label="Unified 3-Axis Bode Gain [deg/m]")
    ax4_r.plot(freqs, tgt_phases, "^--", color="#92400e", lw=1.8, label="Target Bode Phase (Happee 2017) [deg]")
    ax4_r.plot(freqs, sim_phases, "D-", color="#b91c1c", lw=2.0, label="Unified 3-Axis Bode Phase [deg]")

    for f, tg, mg, tp, mp in zip(freqs, tgt_gains, sim_gains, tgt_phases, sim_phases):
        db = 20.0 * np.log10(mg / tg)
        pe = mp - tp
        ax4.annotate(f"{db:+.1f} dB", (f, mg), textcoords="offset points", xytext=(0, 9), ha="center", fontsize=8.5, fontweight="bold", color=color_main)
        ax4_r.annotate(f"{pe:+.1f}°", (f, mp), textcoords="offset points", xytext=(0, -14), ha="center", fontsize=8.5, fontweight="bold", color="#b91c1c")

    ax4.set_xscale("log")
    ax4.xaxis.set_minor_locator(ticker.NullLocator())
    ax4.xaxis.set_minor_formatter(ticker.NullFormatter())
    ax4.minorticks_off()
    ax4.set_xticks(freqs)
    ax4.set_xticklabels([f"{f:.3f} Hz" for f in freqs], fontweight="bold")
    ax4.set_ylim(0, 660)
    ax4_r.set_ylim(-315, 30)
    ax4.set_ylabel("Bode Gain [deg/m]", color=color_main, fontweight="bold", fontsize=11)
    ax4_r.set_ylabel("Bode Phase [deg]", color="#b91c1c", fontweight="bold", fontsize=11)
    ax4.set_xlabel("Multisine Excitation Frequency [Hz] (Log Scale)", fontweight="bold", fontsize=11)
    ax4.set_title(f"4. Unified 3-Axis 6-Frequency Pitch Bode Response — Total Score: {total_score:,.2f} pts", fontweight="bold", fontsize=12)
    ax4.grid(True, alpha=0.3)

    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4_r.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc="center left", fontsize=8.5)

    fig.suptitle(f"Unified 3-Axis Model — Sagittal APEO Pitch Preservation Check (Score: {total_score:,.2f} pts)", fontweight="bold", fontsize=13, y=0.98)
    out_art = os.path.join(ART_DIR, out_filename)
    fig.savefig(out_art, bbox_inches="tight")
    shutil.copyfile(out_art, os.path.join(_ROOT, out_filename))
    plt.close(fig)
    print(f"[OK] Saved APEO pitch check plot: {out_art} (score={total_score:.2f} pts)", flush=True)


def compute_latec_cost(sim_res):
    fr = sim_res["freq_results"]
    g_cost = 0.0
    p_cost = 0.0
    for fk, d in fr.items():
        f_val = d["freq_hz"]
        db = d["gain_db_diff"]
        pe = d["phase_err"]
        w_ph = 2.0 if f_val <= 0.35 else 1.0
        g_cost += (db / 2.5) ** 2
        p_cost += w_ph * ((pe / 20.0) ** 2)
    return g_cost + p_cost + 0.15 * sim_res["hf_noise_rms"]


def main():
    df_lat, df_yaw = load_aligned_data()
    generate_plausibility_plots(df_lat, df_yaw)

    params_path = os.path.join(_ROOT, "logs/roll_yaw_baseline/tuned_3axis_params.json")
    p_combined = dict(P15_RUN4)
    p_combined.update({
        # Roll (LatEC) calibrated R4 overrides
        "Kp_roll": 3.8,
        "Kd_roll": 1.4,
        "Ki_roll": 0.0,
        "K_ton_roll": 4.2,
        "K_sc_roll": 3.2,
        "z_sc_roll": 0.0,
        "K_phas_roll": 16.5,
        "p_phas_roll": 6.0,
        "z_phas_roll": 0.0,
        "deriv_comp_roll": 1.0,
        # Yaw (YawMA) calibrated Y4_hybrid overrides
        "Kp_yaw": 0.24,
        "Kd_yaw": 0.03,
        "Ki_yaw": 0.0,
        "K_ton_yaw": 5.2598427596916935,
        "p_ton_yaw": 16.227225,
        "K_sc_yaw": 1.2,
        "p_sc_yaw": 3.5,
        "Kp_prop_yaw": 8.5,
        "z_ccr_yaw": 0.06,
        "deriv_comp_yaw": 0.25,
    })

    with ProcessPoolExecutor(max_workers=4) as ex:
        fut_lat_zero = ex.submit(run_full_axis_sim, ("LatEC", "zero_shot", P15_RUN4, 2.0))
        fut_lat_tuned = ex.submit(run_full_axis_sim, ("LatEC", "tuned_3axis", p_combined, 2.0))
        fut_yaw_zero = ex.submit(run_full_axis_sim, ("YawMA", "zero_shot", P15_RUN4, 2.0))
        fut_yaw_tuned = ex.submit(run_full_axis_sim, ("YawMA", "tuned_3axis", p_combined, 2.0))

        sim_lat_zero = fut_lat_zero.result()
        sim_lat_tuned = fut_lat_tuned.result()
        sim_yaw_zero = fut_yaw_zero.result()
        sim_yaw_tuned = fut_yaw_tuned.result()

    lat_cost = compute_latec_cost(sim_lat_tuned)
    yaw_cost = compute_latec_cost(sim_yaw_tuned)
    best_name = "R4_Y4_hybrid_decoupled"
    print(f"[EVAL] {best_name}: LatEC cost={lat_cost:.2f}, noise={sim_lat_tuned['hf_noise_rms']:.2f} deg/s, corr={sim_lat_tuned['corr_vel']:.3f}", flush=True)
    print(f"[EVAL] {best_name}: YawMA cost={yaw_cost:.2f}, noise={sim_yaw_tuned['hf_noise_rms']:.2f} deg/s, corr={sim_yaw_tuned['corr_vel']:.3f}", flush=True)

    with open(params_path, "w", encoding="utf-8") as f:
        json.dump(p_combined, f, indent=2)

    apeo_res = run_apeo_pitch_check(p_combined)

    render_baseline_4panel(sim_lat_zero, "zero_shot_latec_roll_baseline_4panel.png")
    render_tuned_comparison_4panel(sim_lat_zero, sim_lat_tuned, "tuned_latec_roll_4panel.png", apeo_res["total_score"])
    render_tuned_comparison_4panel(sim_yaw_zero, sim_yaw_tuned, "tuned_yawma_yaw_4panel.png", apeo_res["total_score"])
    render_apeo_pitch_4panel(apeo_res, "tuned_3axis_apeo_pitch_4panel.png")

    summary = {
        "P_COMBINED_3AXIS": p_combined,
        "selected_joint_roll_candidate": best_name,
        "APEO_Pitch_Check": {
            "total_score": apeo_res["total_score"],
            "run4_baseline_score": 991.107,
            "telemetry": apeo_res["telemetry"],
            "frequencies": apeo_res["freq_results"],
        },
        "LatEC_Roll": {
            "tuned_cost": lat_cost,
            "zero_shot_hf_noise_rms": sim_lat_zero["hf_noise_rms"],
            "tuned_hf_noise_rms": sim_lat_tuned["hf_noise_rms"],
            "zero_shot_corr_vel": sim_lat_zero["corr_vel"],
            "tuned_corr_vel": sim_lat_tuned["corr_vel"],
            "zero_shot_frequencies": sim_lat_zero["freq_results"],
            "tuned_frequencies": sim_lat_tuned["freq_results"],
        },
        "YawMA_Yaw": {
            "tuned_cost": yaw_cost,
            "zero_shot_hf_noise_rms": sim_yaw_zero["hf_noise_rms"],
            "tuned_hf_noise_rms": sim_yaw_tuned["hf_noise_rms"],
            "zero_shot_corr_vel": sim_yaw_zero["corr_vel"],
            "tuned_corr_vel": sim_yaw_tuned["corr_vel"],
            "zero_shot_frequencies": sim_yaw_zero["freq_results"],
            "tuned_frequencies": sim_yaw_tuned["freq_results"],
        },
    }
    summary_path = os.path.join(_ROOT, "logs/roll_yaw_baseline/tuned_3axis_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"\n[SUMMARY] APEO Pitch Score = {apeo_res['total_score']:.2f} pts (Run 4 baseline = 991.11 pts)", flush=True)
    print(f"[SUMMARY] LatEC Roll: HF Noise {sim_lat_zero['hf_noise_rms']:.2f} -> {sim_lat_tuned['hf_noise_rms']:.2f} deg/s | corr {sim_lat_zero['corr_vel']:.3f} -> {sim_lat_tuned['corr_vel']:.3f}", flush=True)
    print(f"[SUMMARY] YawMA Yaw : HF Noise {sim_yaw_zero['hf_noise_rms']:.2f} -> {sim_yaw_tuned['hf_noise_rms']:.2f} deg/s | corr {sim_yaw_zero['corr_vel']:.3f} -> {sim_yaw_tuned['corr_vel']:.3f}", flush=True)
    print(f"[OK] Saved complete 3-axis summary to {summary_path}", flush=True)


if __name__ == "__main__":
    main()
