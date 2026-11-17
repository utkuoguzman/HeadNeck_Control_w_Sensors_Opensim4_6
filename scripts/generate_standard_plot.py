import argparse
import base64
import html
import os
import sys
import math
import json
import shutil
from concurrent.futures import ProcessPoolExecutor

for _env_k, _env_v in (
    ("OMP_NUM_THREADS", "1"),
    ("OPENBLAS_NUM_THREADS", "1"),
    ("MKL_NUM_THREADS", "1"),
    ("NUMEXPR_NUM_THREADS", "1"),
    ("SIMBODY_NUM_THREADS", "1"),
):
    os.environ.setdefault(_env_k, _env_v)

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0, CANONICAL_TARGETS
from run_roll_yaw_baseline import (
    ART_DIR,
    load_aligned_data,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    build_c2_multisine_spline,
)
from tune_roll_and_yaw import build_c2_zero_offset_spline_yaw

MODEL_PATH = os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")

EXPECTED_28_BOOLS = [
    "enable_axis_pitch", "enable_axis_roll", "enable_axis_yaw",
    "enable_ccr", "enable_ccr_spinal", "enable_ccr_task",
    "enable_ccr_pitch", "enable_ccr_roll", "enable_ccr_yaw",
    "enable_vcr", "enable_vcr_tonic", "enable_vcr_canal", "enable_vcr_phasic",
    "enable_vcr_pitch", "enable_vcr_roll", "enable_vcr_yaw",
    "enable_task_controller", "enable_task_integral",
    "enable_task_pitch", "enable_task_roll", "enable_task_yaw",
    "enable_decoder", "enable_mahony", "enable_efference_cancellation",
    "enable_fusimotor", "enable_reciprocal_inhibition",
    "enable_antibuckling", "enable_lead_compensators",
]


def apply_cli_overrides_and_inspect(controller, overrides):
    """Apply explicit KEY=VAL property overrides and inspect all 28 branch booleans."""
    for key, val in overrides:
        if not controller.hasProperty(key):
            raise KeyError(f"Unknown HeadNeckNeuralController property: {key}")
        prop = controller.updPropertyByName(key)
        low = val.strip().lower()
        if low in ("true", "false"):
            osim.PropertyHelper.setValueBool(low == "true", prop)
        else:
            osim.PropertyHelper.setValueDouble(float(val), prop)

    bool_state = {}
    false_booleans = []
    for prop_name in EXPECTED_28_BOOLS:
        is_on = controller.getPropertyByName(prop_name).toString().strip().lower() == "true"
        bool_state[prop_name] = is_on
        if not is_on:
            false_booleans.append(prop_name)

    eye_on = controller.getPropertyByName("enable_eye_afferents").toString().strip().lower() == "true"
    bool_state["enable_eye_afferents"] = eye_on

    if not false_booleans:
        banner = "Booleans: ALL 28 TRUE (Default Open Mode)"
    else:
        banner = "DISABLED BOOLEANS: " + ", ".join(f"{b}=false" for b in false_booleans)
    return bool_state, false_booleans, banner


def run_axis_worker(args):
    mode, overrides = args
    ensure_libraries_loaded()

    model = osim.Model(MODEL_PATH)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    if controller is None:
        raise RuntimeError("HeadNeckNeuralController not found in model")

    bool_state, false_booleans, bool_banner = apply_cli_overrides_and_inspect(controller, overrides)

    # Read actual controller fusimotor state & gains directly from the loaded/overridden .osim
    k_gamma_dyn_raw = osim.PropertyHelper.getValueDouble(controller.getPropertyByName("K_gamma_dyn"))
    k_gamma_stat_raw = osim.PropertyHelper.getValueDouble(controller.getPropertyByName("K_gamma_stat"))
    fusimotor_on = bool_state["enable_fusimotor"]
    k_gdyn_eff = min(max(k_gamma_dyn_raw, 0.0), 1.0) if fusimotor_on else 0.0
    k_gstat_eff = min(max(k_gamma_stat_raw, 0.0), 1.0) if fusimotor_on else 0.0

    # Lock eye coordinates cleanly to 0
    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    # Prescribe all ground DOFs to 0 initially, then attach active perturbation spline
    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    delay = 0.8
    ramp_duration = 1.5
    pert_duration = 15.0
    target_time = delay + pert_duration  # 0.8 + 15.0 = 15.8 s for all 3 modes

    if mode == "APEO":
        ordered_targets = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
        freqs = np.array([t[0] for t in ordered_targets], dtype=float)
        tgt_gains = np.array([t[1] for t in ordered_targets], dtype=float)
        tgt_phases = np.array([t[2] for t in ordered_targets], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_input_unit = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
        amps_sim_unit = amps_input_unit.copy()
        prescribed_coord_name = "spine_tx"
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
    elif mode == "LatEC":
        df_lat, _ = load_aligned_data()
        freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_input_unit = np.array(get_latec_amplitudes(freqs), dtype=float)
        amps_sim_unit = amps_input_unit.copy()
        prescribed_coord_name = "spine_tz"
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
    elif mode == "YawMA":
        _, df_yaw = load_aligned_data()
        freqs = np.array(df_yaw["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_yaw["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.array(df_yaw["Exp_Yaw_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_input_unit = np.array(get_yawma_amplitudes_deg(freqs), dtype=float)
        amps_sim_unit = np.radians(amps_input_unit)
        prescribed_coord_name = "gndyaw"
        ms_spline, _ = build_c2_zero_offset_spline_yaw(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
    else:
        raise ValueError(f"Unknown mode: {mode}")

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

    for sv in (
        "pitch_error_integral",
        "roll_error_integral",
        "yaw_error_integral",
        "vol_filt_pitch",
        "vol_filt_roll",
        "vol_filt_yaw",
    ):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
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
    manager.setIntegratorMethod(5)  # SemiExplicitEuler2
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
    coord_tx = model.getCoordinateSet().get("spine_tx")
    coord_tz = model.getCoordinateSet().get("spine_tz")
    coord_gndyaw = model.getCoordinateSet().get("gndyaw")

    t_list, p1_list, p2_list, r1_list, r2_list, y1_list, y2_list, pert_list = [], [], [], [], [], [], [], []
    yaw_glob_list = []
    aborted = False
    abort_time_s = float(target_time)
    abort_reason = ""

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        r1_deg = math.degrees(coord_r1.getValue(state))
        r2_deg = math.degrees(coord_r2.getValue(state))
        y1_deg = math.degrees(coord_y1.getValue(state))
        y2_deg = math.degrees(coord_y2.getValue(state))
        p_deg = p1_deg + p2_deg
        r_deg = r1_deg + r2_deg
        y_rel_deg = y1_deg + y2_deg
        gnd_y_deg = math.degrees(coord_gndyaw.getValue(state))
        y_glob_deg = gnd_y_deg + y_rel_deg

        if mode == "APEO":
            pert_val = coord_tx.getValue(state)
        elif mode == "LatEC":
            pert_val = coord_tz.getValue(state)
        else:
            pert_val = gnd_y_deg

        t_list.append(current_time)
        p1_list.append(p1_deg)
        p2_list.append(p2_deg)
        r1_list.append(r1_deg)
        r2_list.append(r2_deg)
        y1_list.append(y1_deg)
        y2_list.append(y2_deg)
        yaw_glob_list.append(y_glob_deg)
        pert_list.append(pert_val)

        if mode == "APEO":
            if (
                p1_deg < -12.0 or p1_deg > 18.0
                or p2_deg < -24.75 or p2_deg > 36.0
                or p_deg < -36.75 or p_deg > 54.0
                or abs(r1_deg) > 4.5 or abs(r2_deg) > 24.75 or abs(r_deg) > 29.25
                or abs(y1_deg) > 28.5 or abs(y2_deg) > 20.25 or abs(y_rel_deg) > 48.75
            ):
                aborted = True
                abort_time_s = float(current_time)
                abort_reason = f"t={current_time:.2f}s p={p_deg:+.2f} r={r_deg:+.2f} y={y_rel_deg:+.2f}"
                break
        else:
            if abs(p_deg) > 45.0 or abs(r_deg) > 35.0 or abs(y_rel_deg) > 48.0:
                aborted = True
                abort_time_s = float(current_time)
                abort_reason = f"t={current_time:.2f}s p={p_deg:+.2f} r={r_deg:+.2f} y={y_rel_deg:+.2f}"
                break

    t_arr = np.array(t_list, dtype=float)
    p_arr = np.array(p1_list, dtype=float) + np.array(p2_list, dtype=float)
    r_arr = np.array(r1_list, dtype=float) + np.array(r2_list, dtype=float)
    yaw_rel_arr = np.array(y1_list, dtype=float) + np.array(y2_list, dtype=float)
    yaw_glob_arr = np.array(yaw_glob_list, dtype=float)
    pert_arr = np.array(pert_list, dtype=float)

    if mode == "APEO":
        main_arr = p_arr
    elif mode == "LatEC":
        main_arr = -r_arr  # ISO sign convention
    else:
        main_arr = yaw_glob_arr

    m_quiet = t_arr <= delay
    m_post = t_arr >= (delay + ramp_duration)
    n_post = int(np.sum(m_post))

    quiet_main_max = float(np.max(np.abs(main_arr[m_quiet] - main_arr[0]))) if np.any(m_quiet) else 0.0
    quiet_pitch_dev_max = float(np.max(np.abs(p_arr[m_quiet] - p_arr[0]))) if np.any(m_quiet) else 0.0
    quiet_roll_max = float(np.max(np.abs(r_arr[m_quiet]))) if np.any(m_quiet) else 0.0
    quiet_yaw_rel_max = float(np.max(np.abs(yaw_rel_arr[m_quiet]))) if np.any(m_quiet) else 0.0

    full_pitch_dev_max = float(np.max(np.abs(p_arr - p_arr[0])))
    full_roll_max = float(np.max(np.abs(r_arr)))
    full_yaw_rel_max = float(np.max(np.abs(yaw_rel_arr)))

    freq_results = {}
    hf_noise_rms = 0.0
    corr_vel = 0.0
    hf_penalty = 0.0
    pitch_crosstalk_penalty = 0.0
    pitch_hf_10hz_rms = 0.0
    gain_loss_pts = 0.0
    phase_loss_pts = 0.0

    if aborted or n_post < 50:
        progress = max(0.0, min(1.0, abort_time_s / target_time))
        classic_score = 6.0 * (200000.0 + (1.0 - progress) * 50000.0)
        if mode in ("LatEC", "YawMA"):
            pitch_crosstalk_penalty = 100.0 * (full_pitch_dev_max ** 2)
        total_score = classic_score + pitch_crosstalk_penalty
    else:
        t_fit = t_arr[m_post]
        t_rel = t_fit - delay
        y_fit_deg = main_arr[m_post]

        cols = [np.ones_like(t_fit), t_fit - (delay + ramp_duration)]
        for k in range(K):
            omega_k = 2.0 * math.pi * float(freqs[k])
            psi_k = omega_k * t_rel + float(phases[k])
            cols.append(np.sin(psi_k))
            cols.append(np.cos(psi_k))
        A_mat = np.column_stack(cols)
        coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit_deg, rcond=None)

        ref_fit = np.zeros_like(t_fit)
        sim_harm_fit = np.zeros_like(t_fit)
        for k in range(K):
            f_k = float(freqs[k])
            psi = 2.0 * np.pi * f_k * t_rel + float(phases[k])
            ref_fit += float(amps_input_unit[k]) * float(tgt_gains[k]) * np.sin(psi + math.radians(float(tgt_phases[k])))
            a_sin = float(coeffs[2 + 2 * k])
            b_cos = float(coeffs[2 + 2 * k + 1])
            sim_harm_fit += a_sin * np.sin(psi) + b_cos * np.cos(psi)

            amp_out_deg = math.hypot(a_sin, b_cos)
            sim_gain = amp_out_deg / float(amps_input_unit[k])
            raw_phase = math.degrees(math.atan2(b_cos, a_sin))
            tp = float(tgt_phases[k])
            ph_err = ((raw_phase - tp + 180.0) % 360.0) - 180.0
            db_err = 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k]))

            g_pts = 100.0 * (db_err ** 2)
            p_pts = 1.0 * (ph_err ** 2)
            gain_loss_pts += g_pts
            phase_loss_pts += p_pts

            key_str = f"{f_k:.3f}" if mode == "APEO" else f"{f_k:.4f}"
            freq_results[key_str] = {
                "freq_hz": f_k,
                "input_amp": float(amps_input_unit[k]),
                "output_amp_deg": amp_out_deg,
                "sim_gain": sim_gain,
                "target_gain": float(tgt_gains[k]),
                "gain_db_diff": db_err,
                "gain_loss_pts": g_pts,
                "sim_phase": tp + ph_err,
                "target_phase": tp,
                "phase_err": ph_err,
                "phase_loss_pts": p_pts,
            }

        vel_actual = np.gradient(y_fit_deg, t_fit)
        vel_harm = np.gradient(sim_harm_fit, t_fit)
        vel_ref = np.gradient(ref_fit, t_fit)
        hf_noise_rms = float(np.sqrt(np.mean((vel_actual - vel_harm) ** 2)))
        corr_vel = float(np.corrcoef(vel_actual, vel_ref)[0, 1]) if np.std(vel_actual) > 1e-6 else 0.0

        classic_score = gain_loss_pts + phase_loss_pts
        if mode == "APEO":
            t_u = np.arange(t_fit[0], t_fit[-1], 0.005)
            s_u = np.interp(t_u, t_fit, y_fit_deg)
            win = np.hanning(len(s_u))
            s_u = (s_u - np.mean(s_u)) * win
            mag = np.abs(np.fft.rfft(s_u)) * 2.0 / np.sum(win)
            fr = np.fft.rfftfreq(len(s_u), d=0.005)
            pitch_hf_10hz_rms = float(np.sqrt(np.sum(mag[fr >= 10.0] ** 2) / 2.0))
        if mode in ("LatEC", "YawMA"):
            pitch_crosstalk_penalty = 100.0 * (full_pitch_dev_max ** 2)
        else:
            pitch_crosstalk_penalty = 0.0
        hf_penalty = 0.0
        total_score = classic_score + pitch_crosstalk_penalty

    return {
        "mode": mode,
        "aborted": aborted,
        "abort_time_s": abort_time_s,
        "abort_reason": abort_reason,
        "bool_state": bool_state,
        "false_booleans": false_booleans,
        "bool_banner": bool_banner,
        "target_time": float(target_time),
        "t": t_arr,
        "pitch": p_arr,
        "roll": r_arr,
        "yaw_rel": yaw_rel_arr,
        "yaw_glob": yaw_glob_arr,
        "main_angle": main_arr,
        "pert": pert_arr,
        "freqs": freqs,
        "amps_input_unit": amps_input_unit,
        "phases": phases,
        "tgt_gains": tgt_gains,
        "tgt_phases": tgt_phases,
        "freq_results": freq_results,
        "classic_bode_score": round(classic_score, 2),
        "pitch_crosstalk_penalty": round(pitch_crosstalk_penalty, 2),
        "total_score": round(total_score, 2),
        "gain_loss_pts": round(gain_loss_pts, 2),
        "phase_loss_pts": round(phase_loss_pts, 2),
        "hf_penalty": round(hf_penalty, 2),
        "pitch_hf_10hz_rms_deg": round(pitch_hf_10hz_rms, 4),
        "hf_noise_rms": round(hf_noise_rms, 3),
        "corr_vel": round(corr_vel, 4),
        "quiet_main_max_deg": round(quiet_main_max, 4),
        "quiet_pitch_dev_max_deg": round(quiet_pitch_dev_max, 4),
        "quiet_roll_max_deg": round(quiet_roll_max, 4),
        "quiet_yaw_rel_max_deg": round(quiet_yaw_rel_max, 4),
        "full_pitch_dev_max_deg": round(full_pitch_dev_max, 3),
        "full_roll_max_deg": round(full_roll_max, 3),
        "full_yaw_rel_max_deg": round(full_yaw_rel_max, 3),
    }


def compute_ref_trace(t, freqs, amps, phases, tgt_gains, tgt_phases, delay=0.8, ramp=1.5):
    ref_trace = np.zeros_like(t, dtype=float)
    for i, ti in enumerate(t):
        if ti < delay:
            continue
        t_rel = ti - delay
        w = 1.0
        if t_rel < ramp:
            tau = t_rel / ramp
            w = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        y_deg = 0.0
        for f, a, ph, tg, tp in zip(freqs, amps, phases, tgt_gains, tgt_phases):
            psi = 2.0 * math.pi * float(f) * t_rel + float(ph) + math.radians(float(tp))
            y_deg += float(a) * float(tg) * math.sin(psi)
        ref_trace[i] = w * y_deg
    return ref_trace


def render_standard_4panel(sim_res, out_filename):
    mode = sim_res["mode"]
    t = sim_res["t"]
    main_angle = sim_res["main_angle"]
    pitch = sim_res["pitch"]
    roll = sim_res["roll"]
    yaw_rel = sim_res["yaw_rel"]
    freqs = sim_res["freqs"]
    amps = sim_res["amps_input_unit"]
    phases = sim_res["phases"]
    tgt_gains = sim_res["tgt_gains"]
    tgt_phases = sim_res["tgt_phases"]
    freq_res = sim_res["freq_results"]
    bool_banner = sim_res["bool_banner"]

    ref_trace = compute_ref_trace(t, freqs, amps, phases, tgt_gains, tgt_phases, delay=0.8, ramp=1.5)
    main_vel = np.gradient(main_angle, t)
    ref_vel = np.gradient(ref_trace, t)
    pert_vel = np.gradient(sim_res["pert"], t)

    mask_ss = t >= 2.3
    if not np.any(mask_ss):
        mask_ss = np.ones_like(t, dtype=bool)
    mean_ang_ss = float(np.mean(main_angle[mask_ss]))
    max_ang_ss = float(np.max(np.abs(main_angle[mask_ss])))
    ref_max_ang_ss = float(np.max(np.abs(ref_trace[mask_ss])))
    rms_vel_ss = float(np.sqrt(np.mean(main_vel[mask_ss] ** 2)))
    ref_rms_v = float(np.sqrt(np.mean(ref_vel[mask_ss] ** 2)))

    if mode == "APEO":
        color_main = "#059669"
        axis_name = "Head Pitch"
        gain_unit = "deg/m"
        env_lim = 50.0
        env_label = r"Happee Operating Envelope ($\pm 50^\circ$/s)"
        ref_tag = "Happee (2017)"
        f_fmt = ".3f"
    elif mode == "LatEC":
        color_main = "#2563eb"
        axis_name = "Head Roll"
        gain_unit = "deg/m"
        env_lim = 10.0
        env_label = r"Happee Fig. 5 Roll Rate Envelope ($\pm 10^\circ$/s; Seat $\dot{Y}$ Peak = " + f"{np.max(np.abs(pert_vel[mask_ss])):.3f} m/s)"
        ref_tag = "Happee (2023)"
        f_fmt = ".2f"
    else:
        color_main = "#7c3aed"
        axis_name = "Global Head Yaw"
        gain_unit = "deg/deg"
        env_lim = 15.0
        env_label = r"Target Yaw Rate Envelope ($\pm 15^\circ$/s; Trunk $\dot{\psi}_{\mathrm{T1}}$ Peak = " + f"{np.max(np.abs(pert_vel[mask_ss])):.1f}°/s)"
        ref_tag = "Happee (2023)"
        f_fmt = ".2f"

    fig = plt.figure(figsize=(15, 14.5), dpi=150)
    gs = fig.add_gridspec(4, 1, height_ratios=[1.1, 1.1, 0.85, 1.22], hspace=0.36)

    # Panel 1: Angle vs Time
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, ref_trace, "k--", lw=1.6, label=f"Target {ref_tag} {axis_name} [Max = {ref_max_ang_ss:.2f}°]")
    ax1.plot(
        t,
        main_angle,
        color=color_main,
        lw=2.1,
        label=f"Current Model {axis_name} [Mean = {mean_ang_ss:+.2f}°, Max = {max_ang_ss:.2f}°, Quiet Max|Δ| = {sim_res['quiet_main_max_deg']:.3f}°]",
    )
    if mode == "YawMA":
        ax1.plot(
            t,
            sim_res["pert"],
            color="#64748b",
            ls=":",
            lw=1.3,
            alpha=0.85,
            label=f"Applied Trunk Yaw Input gndyaw(t) [Max = {np.max(np.abs(sim_res['pert'][mask_ss])):.2f}°]",
        )
    ax1.axvline(0.8, color="#64748b", ls=":", lw=1.3, label="Perturbation Ramp Start (t = 0.8s)")
    ax1.axvline(2.3, color="#dc2626", ls="--", lw=1.4, label="Steady-State Window (t >= 2.3s)")
    ax1.set_xlim(0, sim_res["target_time"])
    all_ang = [ref_trace, main_angle] + ([sim_res["pert"]] if mode == "YawMA" else [])
    y1_min = float(min(np.min(a) for a in all_ang))
    y1_max = float(max(np.max(a) for a in all_ang))
    span1 = max(1.0, y1_max - y1_min)
    ax1.set_ylim(y1_min - 0.14 * span1, y1_max + 0.24 * span1)
    ax1.set_ylabel(f"{axis_name} [deg]", fontweight="bold", fontsize=11)
    ax1.set_title(f"1. {axis_name} Angle vs. Time ({mode}, Duration: {t[-1]:.2f}s)", fontweight="bold", fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8.6)

    # Panel 2: Angular Velocity vs Time
    ax2 = fig.add_subplot(gs[1])
    ax2.axhspan(-env_lim, env_lim, color="#f1f5f9", alpha=0.9, label=env_label)
    ax2.axhline(0, color="#94a3b8", lw=0.8)
    ax2.plot(t, ref_vel, "k--", lw=1.5, alpha=0.85, label=f"Target {ref_tag} Ref Velocity [RMS = {ref_rms_v:.2f}°/s]")
    ax2.plot(
        t,
        main_vel,
        color=color_main,
        lw=1.9,
        label=f"Current Model Velocity [RMS = {rms_vel_ss:.2f}°/s, HF Noise = {sim_res['hf_noise_rms']:.2f}°/s, corr r = {sim_res['corr_vel']:+.3f}]",
    )
    ax2.axvline(0.8, color="#64748b", ls=":", lw=1.3)
    ax2.axvline(2.3, color="#dc2626", ls="--", lw=1.4)
    ax2.set_xlim(0, sim_res["target_time"])
    vel_abs = max(env_lim, float(np.max(np.abs(main_vel))), float(np.max(np.abs(ref_vel))))
    ax2.set_ylim(-1.22 * vel_abs, 1.28 * vel_abs)
    ax2.set_ylabel(f"{axis_name} Vel [°/s]", fontweight="bold", fontsize=11)
    ax2.set_title(f"2. {axis_name} Angular Velocity vs. Time ({mode})", fontweight="bold", fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8.5)

    # Panel 3: Cross-Axis Excitation
    ax3 = fig.add_subplot(gs[2])
    ax3.axhline(0, color="#94a3b8", lw=0.8)
    pitch_dev = pitch - pitch[0]
    cross_max_abs = 0.0
    if mode == "APEO":
        ax3.plot(
            t,
            roll,
            color="#dc2626",
            lw=1.5,
            label=f"Head Roll (r1+r2) [Quiet Max = {sim_res['quiet_roll_max_deg']:.3f}°, Full Max = {sim_res['full_roll_max_deg']:.2f}°]",
        )
        ax3.plot(
            t,
            yaw_rel,
            color="#059669",
            lw=1.5,
            label=f"Head Yaw (y1+y2) [Quiet Max = {sim_res['quiet_yaw_rel_max_deg']:.3f}°, Full Max = {sim_res['full_yaw_rel_max_deg']:.2f}°]",
        )
        cross_max_abs = max(sim_res["full_roll_max_deg"], sim_res["full_yaw_rel_max_deg"])
    elif mode == "LatEC":
        ax3.plot(
            t,
            pitch_dev,
            color="#059669",
            lw=1.5,
            label=f"Pitch Deviation (p-p0) [Quiet Max = {sim_res['quiet_pitch_dev_max_deg']:.3f}°, Full Max = {sim_res['full_pitch_dev_max_deg']:.2f}°]",
        )
        ax3.plot(
            t,
            yaw_rel,
            color="#d97706",
            lw=1.5,
            label=f"Coupled Neck Yaw (y1+y2) [Quiet Max = {sim_res['quiet_yaw_rel_max_deg']:.3f}°, Full Max = {sim_res['full_yaw_rel_max_deg']:.2f}°]",
        )
        cross_max_abs = max(sim_res["full_pitch_dev_max_deg"], sim_res["full_yaw_rel_max_deg"])
    else:
        ax3.plot(
            t,
            pitch_dev,
            color="#059669",
            lw=1.5,
            label=f"Pitch Deviation (p-p0) [Quiet Max = {sim_res['quiet_pitch_dev_max_deg']:.3f}°, Full Max = {sim_res['full_pitch_dev_max_deg']:.2f}°]",
        )
        ax3.plot(
            t,
            roll,
            color="#dc2626",
            lw=1.5,
            label=f"Coupled Head Roll (r1+r2) [Quiet Max = {sim_res['quiet_roll_max_deg']:.3f}°, Full Max = {sim_res['full_roll_max_deg']:.2f}°]",
        )
        ax3.plot(
            t,
            yaw_rel,
            color="#64748b",
            ls="--",
            lw=1.25,
            label=f"Relative Neck-on-Trunk Yaw (y1+y2) [Quiet Max = {sim_res['quiet_yaw_rel_max_deg']:.3f}°, Full Max = {sim_res['full_yaw_rel_max_deg']:.2f}°]",
        )
        cross_max_abs = max(sim_res["full_pitch_dev_max_deg"], sim_res["full_roll_max_deg"], sim_res["full_yaw_rel_max_deg"])
    ax3.axvline(0.8, color="#64748b", ls=":", lw=1.3)
    ax3.axvline(2.3, color="#dc2626", ls="--", lw=1.4)
    ax3.set_xlim(0, sim_res["target_time"])
    cross_lim = max(1.5, 1.24 * cross_max_abs)
    ax3.set_ylim(-cross_lim, cross_lim)
    ax3.set_ylabel("Cross-Axes [deg]", fontweight="bold", fontsize=11)
    ax3.set_title(f"3. 6-DOF Cross-Axis Excitation & Pre-Perturbation Quietness ({mode})", fontweight="bold", fontsize=12)
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", fontsize=8.5)

    # Panel 4: Bode Plot + Score
    ax4 = fig.add_subplot(gs[3])
    ax4_r = ax4.twinx()

    if sim_res["aborted"] or not freq_res:
        ax4.text(
            0.5,
            0.5,
            f"SIMULATION ABORTED AT t = {sim_res['abort_time_s']:.2f}s\n({sim_res['abort_reason']})\nScore = {sim_res['total_score']:,.2f} pts",
            transform=ax4.transAxes,
            ha="center",
            va="center",
            fontsize=12,
            fontweight="bold",
            color="#b91c1c",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#fef2f2", edgecolor="#dc2626"),
        )
        ax4.set_title(f"4. Bode Response ({mode}) — ABORTED ({sim_res['total_score']:,.2f} pts)", fontweight="bold", fontsize=11.5)
    else:
        sim_gains = []
        sim_phases = []
        for f in freqs:
            key = f"{float(f):.3f}" if mode == "APEO" else f"{float(f):.4f}"
            sim_gains.append(freq_res[key]["sim_gain"])
            sim_phases.append(freq_res[key]["sim_phase"])
        sim_gains = np.array(sim_gains, dtype=float)
        sim_phases = np.array(sim_phases, dtype=float)

        ax4.plot(freqs, tgt_gains, "ko--", lw=1.8, label=f"Target Exp Gain ({ref_tag}) [{gain_unit}]")
        ax4.plot(freqs, sim_gains, "s-", color=color_main, lw=2.3, label=f"Current Model Gain [{gain_unit}]")
        ax4_r.plot(freqs, tgt_phases, "^--", color="#92400e", lw=1.8, label=f"Target Exp Phase ({ref_tag}) [deg]")
        ax4_r.plot(freqs, sim_phases, "D-", color="#b91c1c", lw=2.1, label="Current Model Phase [deg]")

        for f, tg, mg, tp, mp in zip(freqs, tgt_gains, sim_gains, tgt_phases, sim_phases):
            db = 20.0 * math.log10(max(mg, 1e-6) / float(tg))
            pe = float(mp) - float(tp)
            ax4.annotate(
                f"{db:+.1f} dB",
                (f, mg),
                textcoords="offset points",
                xytext=(0, 9),
                ha="center",
                fontsize=8.2,
                fontweight="bold",
                color=color_main,
            )
            ax4_r.annotate(
                f"{pe:+.1f}°",
                (f, mp),
                textcoords="offset points",
                xytext=(0, -14),
                ha="center",
                fontsize=8.2,
                fontweight="bold",
                color="#b91c1c",
            )

        ax4.set_xscale("log")
        ax4.xaxis.set_minor_locator(ticker.NullLocator())
        ax4.xaxis.set_minor_formatter(ticker.NullFormatter())
        ax4.minorticks_off()
        ax4.set_xticks(freqs)
        ax4.set_xticklabels([f"{f:{f_fmt}} Hz" for f in freqs], fontweight="bold", fontsize=9.0)

        g_top = float(max(np.max(tgt_gains), np.max(sim_gains)))
        ax4.set_ylim(0.0, max(1.0, 1.22 * g_top))

        p_all = np.concatenate([tgt_phases, sim_phases])
        p_min, p_max = float(np.min(p_all)), float(np.max(p_all))
        p_span = max(60.0, p_max - p_min)
        ax4_r.set_ylim(p_min - 0.22 * p_span - 15.0, p_max + 0.22 * p_span + 15.0)

        ax4.set_ylabel(f"Bode Gain [{gain_unit}]", color=color_main, fontweight="bold", fontsize=11)
        ax4_r.set_ylabel("Bode Phase [deg]", color="#b91c1c", fontweight="bold", fontsize=11)
        ax4.set_xlabel("Multisine Excitation Frequency [Hz] (Log Scale)", fontweight="bold", fontsize=11)

        score_str = f"Classic Score: {sim_res['classic_bode_score']:,.2f} pts (G={sim_res['gain_loss_pts']:.1f}, P={sim_res['phase_loss_pts']:.1f})"
        if mode in ("LatEC", "YawMA"):
            score_str += f" | Pitch XTalk: {sim_res['full_pitch_dev_max_deg']:.2f}° (+{sim_res['pitch_crosstalk_penalty']:.1f}) | Total: {sim_res['total_score']:,.2f} pts"
        ax4.set_title(f"4. {len(freqs)}-Frequency Bode Response ({mode}) — {score_str}", fontweight="bold", fontsize=11.0)
        ax4.grid(True, alpha=0.3)

        l1, lb1 = ax4.get_legend_handles_labels()
        l2, lb2 = ax4_r.get_legend_handles_labels()
        ax4.legend(l1 + l2, lb1 + lb2, loc="upper left", fontsize=8.2, ncol=2)

    if mode in ("LatEC", "YawMA"):
        score_hdr = f"Total: {sim_res['total_score']:,.2f} pts (Classic Bode: {sim_res['classic_bode_score']:,.2f} + Pitch XTalk: {sim_res['pitch_crosstalk_penalty']:,.2f})"
    else:
        score_hdr = f"Total: {sim_res['total_score']:,.2f} pts (Classic Bode: {sim_res['classic_bode_score']:,.2f} pts)"
    fig.suptitle(
        f"Standard Plot ({mode} — {axis_name}) | {score_hdr}\n{bool_banner}",
        fontweight="bold",
        fontsize=12.4,
        y=0.985,
    )

    out_art = os.path.join(ART_DIR, out_filename)
    os.makedirs(ART_DIR, exist_ok=True)
    fig.savefig(out_art, bbox_inches="tight")
    shutil.copyfile(out_art, os.path.join(_ROOT, out_filename))
    plt.close(fig)
    print(f"[OK] Saved standard plot: {out_art}", flush=True)


def write_html_viewer(bool_banner, axes_summary, out_files, suffix=""):
    """Build self-contained 3-tab HTML viewer embedding the 3 PNG plots as base64 data URIs."""
    def _data_uri(fname):
        raw = open(os.path.join(ART_DIR, fname), "rb").read()
        return "data:image/png;base64," + base64.b64encode(raw).decode("ascii")

    apeo = axes_summary["APEO"]
    latec = axes_summary["LatEC"]
    yawma = axes_summary["YawMA"]

    apeo_score = f"{apeo['total_score']:,.0f} pts"
    latec_score = f"{latec['total_score']:,.0f} pts"
    yawma_score = f"{yawma['total_score']:,.0f} pts"
    combined_total = apeo['total_score'] + latec['total_score'] + yawma['total_score']
    score_summary = (
        f"APEO Pitch: {apeo['total_score']:,.2f} pts | "
        f"LatEC Roll: {latec['total_score']:,.2f} pts (XTalk +{latec['pitch_crosstalk_penalty']:.1f}) | "
        f"YawMA Yaw: {yawma['total_score']:,.2f} pts (XTalk +{yawma['pitch_crosstalk_penalty']:.1f}) | "
        f"Combined: {combined_total:,.2f} pts"
    )

    template = f'''<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Standard Plot — 3-Axis Evaluation</title>
  <script src="https://www.gstatic.com/antigravity/web/dev/tailwindcss.min.js"></script>
  <style>
    .tab-btn.active {{
      background: var(--primary, #059669);
      color: var(--primary-foreground, #ffffff);
      font-weight: 600;
    }}
  </style>
</head>
<body class="bg-[var(--background)] text-[var(--foreground)] antialiased p-4 m-0 font-sans">
  <div class="bg-[var(--card)] border border-[var(--border)] rounded-xl p-4 shadow-sm max-w-6xl mx-auto space-y-4">
    <div class="flex flex-wrap items-center justify-between gap-3 pb-3 border-b border-[var(--border)]">
      <div>
        <h1 class="text-base font-bold">Standard Plot — 3-Axis Closed-Loop Evaluation</h1>
        <p class="text-xs text-[var(--muted-foreground)] mt-0.5">
          {html.escape(bool_banner)} &bull; {html.escape(score_summary)}
        </p>
      </div>
      <div class="flex flex-wrap items-center gap-1.5 bg-[var(--background)] p-1 rounded-lg border border-[var(--border)]">
        <button id="btn-apeo" onclick="showTab('apeo')" class="tab-btn active px-3 py-1.5 rounded-md text-xs transition-colors">
          APEO Pitch ({html.escape(apeo_score)})
        </button>
        <button id="btn-latec" onclick="showTab('latec')" class="tab-btn px-3 py-1.5 rounded-md text-xs text-[var(--muted-foreground)] hover:text-[var(--foreground)] transition-colors">
          LatEC Roll ({html.escape(latec_score)})
        </button>
        <button id="btn-yawma" onclick="showTab('yawma')" class="tab-btn px-3 py-1.5 rounded-md text-xs text-[var(--muted-foreground)] hover:text-[var(--foreground)] transition-colors">
          YawMA Yaw ({html.escape(yawma_score)})
        </button>
        <button id="btn-all" onclick="showTab('all')" class="tab-btn px-3 py-1.5 rounded-md text-xs text-[var(--muted-foreground)] hover:text-[var(--foreground)] transition-colors">
          All 3 Stacked
        </button>
      </div>
    </div>

    <div id="pane-apeo" class="pane">
      <div class="bg-[var(--background)] rounded-lg p-2 border border-[var(--border)] flex justify-center">
        <img src="{_data_uri(out_files['APEO'])}" alt="Standard Plot APEO Pitch" class="max-w-full h-auto rounded" />
      </div>
    </div>

    <div id="pane-latec" class="pane hidden">
      <div class="bg-[var(--background)] rounded-lg p-2 border border-[var(--border)] flex justify-center">
        <img src="{_data_uri(out_files['LatEC'])}" alt="Standard Plot LatEC Roll" class="max-w-full h-auto rounded" />
      </div>
    </div>

    <div id="pane-yawma" class="pane hidden">
      <div class="bg-[var(--background)] rounded-lg p-2 border border-[var(--border)] flex justify-center">
        <img src="{_data_uri(out_files['YawMA'])}" alt="Standard Plot YawMA Yaw" class="max-w-full h-auto rounded" />
      </div>
    </div>
  </div>

  <script>
    function showTab(which) {{
      const panes = {{
        apeo: document.getElementById('pane-apeo'),
        latec: document.getElementById('pane-latec'),
        yawma: document.getElementById('pane-yawma'),
      }};
      const btns = {{
        apeo: document.getElementById('btn-apeo'),
        latec: document.getElementById('btn-latec'),
        yawma: document.getElementById('btn-yawma'),
        all: document.getElementById('btn-all'),
      }};
      Object.values(btns).forEach(b => b.classList.remove('active'));
      btns[which].classList.add('active');
      if (which === 'all') {{
        Object.values(panes).forEach(p => p.classList.remove('hidden'));
      }} else {{
        Object.entries(panes).forEach(([k, p]) => {{
          p.classList.toggle('hidden', k !== which);
        }});
      }}
    }}
  </script>
</body>
</html>
'''
    viewer_name = f"standard_plot_viewer{suffix}.html"
    art_viewer = os.path.join(ART_DIR, viewer_name)
    root_viewer = os.path.join(_ROOT, viewer_name)
    with open(art_viewer, "w", encoding="utf-8") as f:
        f.write(template)
    shutil.copyfile(art_viewer, root_viewer)
    print(f"[OK] Saved self-contained viewer: {art_viewer}", flush=True)


def parse_set_args(raw_list):
    overrides = []
    for item in raw_list:
        if "=" not in item:
            raise ValueError(f"Expected KEY=VAL for --set, got: {item}")
        k, v = item.split("=", 1)
        overrides.append((k.strip(), v.strip()))
    return overrides


def main():
    parser = argparse.ArgumentParser(description="Generate canonical 3-axis Standard Plots using exact current .osim parameters.")
    parser.add_argument("--set", action="append", default=[], dest="overrides", help="Explicit property override KEY=VAL (e.g. enable_ccr=false)")
    parser.add_argument("--tag", default="", help="Optional filename suffix tag (e.g. 'no_ccr')")
    args = parser.parse_args()

    overrides = parse_set_args(args.overrides)
    suffix = f"_{args.tag.strip()}" if args.tag.strip() else ""

    modes = ["APEO", "LatEC", "YawMA"]
    with ProcessPoolExecutor(max_workers=3) as ex:
        futs = [ex.submit(run_axis_worker, (m, overrides)) for m in modes]
        results = {res["mode"]: res for res in (f.result() for f in futs)}

    out_files = {
        "APEO": f"standard_plot_apeo_pitch{suffix}.png",
        "LatEC": f"standard_plot_latec_roll{suffix}.png",
        "YawMA": f"standard_plot_yawma_yaw{suffix}.png",
    }
    for m in modes:
        render_standard_4panel(results[m], out_files[m])

    summary = {
        "model_path": MODEL_PATH,
        "cli_overrides": [{"key": k, "val": v} for k, v in overrides],
        "false_booleans": results["APEO"]["false_booleans"],
        "bool_banner": results["APEO"]["bool_banner"],
        "bool_state": results["APEO"]["bool_state"],
        "axes": {},
    }
    for m in modes:
        r = results[m]
        summary["axes"][m] = {
            "output_png": out_files[m],
            "aborted": r["aborted"],
            "abort_time_s": r["abort_time_s"],
            "abort_reason": r["abort_reason"],
            "classic_bode_score": r["classic_bode_score"],
            "pitch_crosstalk_penalty": r["pitch_crosstalk_penalty"],
            "total_score": r["total_score"],
            "gain_loss_pts": r["gain_loss_pts"],
            "phase_loss_pts": r["phase_loss_pts"],
            "hf_penalty": r["hf_penalty"],
            "pitch_hf_10hz_rms_deg": r["pitch_hf_10hz_rms_deg"],
            "hf_noise_rms": r["hf_noise_rms"],
            "corr_vel": r["corr_vel"],
            "quiet_main_max_deg": r["quiet_main_max_deg"],
            "quiet_pitch_dev_max_deg": r["quiet_pitch_dev_max_deg"],
            "quiet_roll_max_deg": r["quiet_roll_max_deg"],
            "quiet_yaw_rel_max_deg": r["quiet_yaw_rel_max_deg"],
            "full_pitch_dev_max_deg": r["full_pitch_dev_max_deg"],
            "full_roll_max_deg": r["full_roll_max_deg"],
            "full_yaw_rel_max_deg": r["full_yaw_rel_max_deg"],
            "frequencies": r["freq_results"],
        }

    summary["combined_bode_score"] = round(sum(summary["axes"][m]["classic_bode_score"] for m in modes), 2)
    summary["combined_pitch_crosstalk_penalty"] = round(sum(summary["axes"][m]["pitch_crosstalk_penalty"] for m in modes), 2)
    summary["combined_total_score"] = round(sum(summary["axes"][m]["total_score"] for m in modes), 2)

    summary_name = f"standard_plot_summary{suffix}.json"
    summary_path = os.path.join(_ROOT, "logs", summary_name)
    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print(f"\n=== {summary['bool_banner']} ===", flush=True)
    for m in modes:
        a = summary["axes"][m]
        print(
            f"[{m:5s}] Classic={a['classic_bode_score']:,.2f} pts (G={a['gain_loss_pts']:.1f}, P={a['phase_loss_pts']:.1f}), "
            f"PitchXTalk={a['pitch_crosstalk_penalty']:.1f} (max={a['full_pitch_dev_max_deg']:.2f}°), "
            f"Total={a['total_score']:,.2f}, QuietMain={a['quiet_main_max_deg']:.3f}°, "
            f"corr={a['corr_vel']:+.3f}, HFNoise={a['hf_noise_rms']:.2f}°/s, aborted={a['aborted']}",
            flush=True,
        )
    print(
        f"[TOTAL] CombinedBode={summary['combined_bode_score']:,.2f} + PitchXTalk={summary['combined_pitch_crosstalk_penalty']:,.2f} "
        f"= CombinedTotal={summary['combined_total_score']:,.2f} pts",
        flush=True,
    )
    print(f"[OK] Saved summary JSON: {summary_path}", flush=True)
    write_html_viewer(summary["bool_banner"], summary["axes"], out_files, suffix=suffix)


if __name__ == "__main__":
    main()
