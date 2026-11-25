#!/usr/bin/env python3
"""Step 4: 6-Worker Validation of Corrected 3-Axis Mahony Spatial Estimator & Subsystem 3(C)."""

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0, CANONICAL_TARGETS
from run_roll_yaw_baseline import (
    load_aligned_data,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    build_c2_multisine_spline,
)
from tune_roll_and_yaw import build_c2_zero_offset_spline_yaw
from scripts.generate_standard_plot import MODEL_PATH, apply_cli_overrides_and_inspect

G35_JSON = os.path.join(_ROOT, "logs", "cmaes_3axis_18p_roll_champion.json")
G2_CLEAN_JSON = os.path.join(_ROOT, "logs", "cmaes_latec_only_18p_clean_champion.json")
W6_JSON = os.path.join(_ROOT, "logs", "local_wave6_g35_stiff_clean.json")
OUT_JSON = os.path.join(_ROOT, "logs", "validate_corrected_mahony_3axis.json")


def _axis_fit_stats(true_arr, est_arr):
    if len(true_arr) < 5:
        return {"true_ptp": 0.0, "est_ptp": 0.0, "slope": 0.0, "r": 0.0, "rmse_deg": 0.0, "mean_err_deg": 0.0}
    rmse = float(np.sqrt(np.mean((est_arr - true_arr) ** 2)))
    mean_err = float(np.mean(est_arr - true_arr))
    r = float(np.corrcoef(true_arr, est_arr)[0, 1]) if (np.std(true_arr) > 1e-4 and np.std(est_arr) > 1e-4) else 0.0
    slope = float(np.polyfit(true_arr, est_arr, 1)[0]) if np.std(true_arr) > 1e-3 else 0.0
    return {
        "true_ptp": round(float(np.ptp(true_arr)), 4),
        "est_ptp": round(float(np.ptp(est_arr)), 4),
        "slope": round(slope, 4),
        "r": round(r, 5),
        "rmse_deg": round(rmse, 4),
        "mean_err_deg": round(mean_err, 4),
    }


def _eval_validation_job(args):
    idx, name, mode, pdict = args
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
        raise RuntimeError("HeadNeckNeuralController not found")

    overrides = [(k, f"{float(v):.15g}") for k, v in pdict.items()]
    bool_state, false_booleans, bool_banner = apply_cli_overrides_and_inspect(controller, overrides)
    assert not false_booleans, f"Disabled booleans in {name}: {false_booleans}"

    k_gamma_dyn_raw = osim.PropertyHelper.getValueDouble(controller.getPropertyByName("K_gamma_dyn"))
    k_gamma_stat_raw = osim.PropertyHelper.getValueDouble(controller.getPropertyByName("K_gamma_stat"))
    fusimotor_on = bool_state["enable_fusimotor"]
    k_gdyn_eff = min(max(k_gamma_dyn_raw, 0.0), 1.0) if fusimotor_on else 0.0
    k_gstat_eff = min(max(k_gamma_stat_raw, 0.0), 1.0) if fusimotor_on else 0.0

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

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
    target_time = 10.0 if mode == "VOL3" else (delay + pert_duration)

    prescribed_coord_name = "spine_tx"
    ms_spline = zero_func
    freqs = np.array([])
    tgt_gains = np.array([])
    tgt_phases = np.array([])
    phases = np.array([])
    amps_input_unit = np.array([])
    K = 0

    if mode == "APEO":
        ordered_targets = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
        freqs = np.array([t[0] for t in ordered_targets], dtype=float)
        tgt_gains = np.array([t[1] for t in ordered_targets], dtype=float)
        tgt_phases = np.array([t[2] for t in ordered_targets], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_input_unit = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
        prescribed_coord_name = "spine_tx"
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_input_unit, phases, delay, ramp_duration, target_time)
    elif mode == "LatEC":
        df_lat, _ = load_aligned_data()
        freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_input_unit = np.array(get_latec_amplitudes(freqs), dtype=float)
        prescribed_coord_name = "spine_tz"
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_input_unit, phases, delay, ramp_duration, target_time)
    elif mode == "YawMA":
        _, df_yaw = load_aligned_data()
        freqs = np.array(df_yaw["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_yaw["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.array(df_yaw["Exp_Yaw_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_input_unit = np.array(get_yawma_amplitudes_deg(freqs), dtype=float)
        prescribed_coord_name = "gndyaw"
        ms_spline, _ = build_c2_zero_offset_spline_yaw(freqs, np.radians(amps_input_unit), phases, delay, ramp_duration, target_time)

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
    q_init = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    for q_idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{q_idx}", q_init.get(q_idx))

    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral", "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        m_name = m.getName()
        if m_name.startswith("r_") or m_name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(m_name, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gdyn_eff * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gstat_eff * a0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{m_name}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{m_name}", fstat0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setWriteToStorage(False)
    manager.setPerformAnalyses(False)
    manager.setIntegratorMethod(5)  # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(1e-3)
    state.setTime(0.0)
    manager.initialize(state)

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")
    coord_gndyaw = model.getCoordinateSet().get("gndyaw")

    outs = {k: osim.OutputDouble.safeDownCast(controller.getOutput(k)) for k in (
        "mahony_pitch", "mahony_roll", "mahony_yaw",
        "mahony_pitch_speed", "mahony_roll_speed", "mahony_yaw_speed",
        "cuneate_neck_pitch", "cuneate_neck_roll", "cuneate_neck_yaw",
        "est_neck_pitch", "est_neck_roll", "est_neck_yaw",
    )}

    step = 0.005
    current_time = 0.0
    rows = []
    aborted = False
    abort_time_s = float(target_time)
    abort_reason = ""

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)
        model.realizeVelocity(state)

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

        R_s = skull.getTransformInGround(state).R()
        q_s = osim.Rotation(R_s).convertRotationToQuaternion()
        q0, q1, q2, q3 = q_s.get(0), q_s.get(1), q_s.get(2), q_s.get(3)
        tp = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))
        tr = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))
        ty = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))))
        w_g = skull.getAngularVelocityInGround(state)
        wx_h = R_s.get(0, 0) * w_g.get(0) + R_s.get(1, 0) * w_g.get(1) + R_s.get(2, 0) * w_g.get(2)
        wy_h = R_s.get(0, 1) * w_g.get(0) + R_s.get(1, 1) * w_g.get(1) + R_s.get(2, 1) * w_g.get(2)
        wz_h = R_s.get(0, 2) * w_g.get(0) + R_s.get(1, 2) * w_g.get(1) + R_s.get(2, 2) * w_g.get(2)

        rows.append([
            current_time,
            p_deg, r_deg, y_rel_deg, y_glob_deg,
            tp, tr, ty,
            math.degrees(outs["mahony_pitch"].getValue(state)),
            math.degrees(outs["mahony_roll"].getValue(state)),
            math.degrees(outs["mahony_yaw"].getValue(state)),
            math.degrees(outs["cuneate_neck_pitch"].getValue(state)),
            math.degrees(outs["cuneate_neck_roll"].getValue(state)),
            math.degrees(outs["cuneate_neck_yaw"].getValue(state)),
            math.degrees(outs["est_neck_pitch"].getValue(state)),
            math.degrees(outs["est_neck_roll"].getValue(state)),
            math.degrees(outs["est_neck_yaw"].getValue(state)),
            math.degrees(wz_h), math.degrees(wx_h), math.degrees(wy_h),
            math.degrees(outs["mahony_pitch_speed"].getValue(state)),
            math.degrees(outs["mahony_roll_speed"].getValue(state)),
            math.degrees(outs["mahony_yaw_speed"].getValue(state)),
        ])

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

    arr = np.asarray(rows, dtype=float)
    t_arr = arr[:, 0]
    p_arr = arr[:, 1]
    r_arr = arr[:, 2]
    y_rel_arr = arr[:, 3]
    y_glob_arr = arr[:, 4]
    main_arr = p_arr if mode == "APEO" else (-r_arr if mode == "LatEC" else y_glob_arr)

    # Zero-reference skull angles and Mahony angles relative to t=0 pose
    true_p = arr[:, 5] - arr[0, 5]
    true_r = arr[:, 6] - arr[0, 6]
    true_y = arr[:, 7] - arr[0, 7]
    mah_p = arr[:, 8] - arr[0, 8]
    mah_r = arr[:, 9] - arr[0, 9]
    mah_y = arr[:, 10] - arr[0, 10]
    cun_p, cun_r, cun_y = arr[:, 11], arr[:, 12], arr[:, 13]
    est_p, est_r, est_y = arr[:, 14], arr[:, 15], arr[:, 16]
    true_wp, true_wr, true_wy = arr[:, 17], arr[:, 18], arr[:, 19]
    mah_wp, mah_wr, mah_wy = arr[:, 20], arr[:, 21], arr[:, 22]

    rel_p = p_arr - p_arr[0]
    rel_r = r_arr - r_arr[0]
    rel_y = y_rel_arr - y_rel_arr[0]

    eval_start = 5.0
    m_post = t_arr >= eval_start
    m_stat = m_post if np.sum(m_post) >= 10 else np.ones_like(t_arr, dtype=bool)
    m_full = np.ones_like(t_arr, dtype=bool)

    bode_score = 0.0
    gain_loss_pts = 0.0
    phase_loss_pts = 0.0
    hf_noise_rms = 0.0
    corr_vel = 0.0
    xtalk_penalty = 0.0
    lowfreq_ptp_deg = 0.0
    freq_results = {}

    if mode in ("APEO", "LatEC", "YawMA"):
        full_p_dev = float(np.max(np.abs(rel_p)))
        full_r_max = float(np.max(np.abs(r_arr)))
        full_y_max = float(np.max(np.abs(y_rel_arr)))
        post_p_dev = float(np.max(np.abs(rel_p[m_stat])))
        post_r_max = float(np.max(np.abs(r_arr[m_stat])))
        post_y_max = float(np.max(np.abs(y_rel_arr[m_stat])))

        if aborted or int(np.sum(m_post)) < 50:
            progress = max(0.0, min(1.0, abort_time_s / target_time))
            bode_score = 6.0 * (200000.0 + (1.0 - progress) * 50000.0)
            if mode == "APEO":
                xtalk_penalty = 100.0 * (full_r_max ** 2 + full_y_max ** 2)
            elif mode == "LatEC":
                xtalk_penalty = 100.0 * (full_p_dev ** 2 + full_y_max ** 2)
            else:
                xtalk_penalty = 100.0 * (full_p_dev ** 2 + full_r_max ** 2)
        else:
            t_fit = t_arr[m_post]
            t_rel = t_fit - delay
            y_fit_deg = main_arr[m_post]
            cols = [np.ones_like(t_fit), t_fit - eval_start]
            for k in range(K):
                omega_k = 2.0 * math.pi * float(freqs[k])
                psi_k = omega_k * t_rel + float(phases[k])
                cols.append(np.sin(psi_k))
                cols.append(np.cos(psi_k))
            coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit_deg, rcond=None)
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
                tp_tgt = float(tgt_phases[k])
                ph_err = ((raw_phase - tp_tgt + 180.0) % 360.0) - 180.0
                db_err = 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k]))
                g_pts = 100.0 * (db_err ** 2)
                p_pts = 1.0 * (ph_err ** 2)
                gain_loss_pts += g_pts
                phase_loss_pts += p_pts
                key_str = f"{f_k:.3f}" if mode == "APEO" else f"{f_k:.4f}"
                freq_results[key_str] = {
                    "gain_db_diff": round(db_err, 3),
                    "gain_loss_pts": round(g_pts, 1),
                    "phase_err": round(ph_err, 2),
                    "phase_loss_pts": round(p_pts, 1),
                }
            bode_score = gain_loss_pts + phase_loss_pts
            vel_actual = np.gradient(y_fit_deg, t_fit)
            vel_harm = np.gradient(sim_harm_fit, t_fit)
            vel_ref = np.gradient(ref_fit, t_fit)
            hf_noise_rms = float(np.sqrt(np.mean((vel_actual - vel_harm) ** 2)))
            corr_vel = float(np.corrcoef(vel_actual, vel_ref)[0, 1]) if np.std(vel_actual) > 1e-6 else 0.0
            if mode == "APEO":
                xtalk_penalty = 100.0 * (post_r_max ** 2 + post_y_max ** 2)
            elif mode == "LatEC":
                xtalk_penalty = 100.0 * (post_p_dev ** 2 + post_y_max ** 2)
            else:
                xtalk_penalty = 100.0 * (post_p_dev ** 2 + post_r_max ** 2)

            if len(y_fit_deg) >= 500:
                lowfreq_ptp_deg = float(np.ptp(np.convolve(y_fit_deg, np.ones(500) / 500.0, mode="valid")))
            else:
                lowfreq_ptp_deg = float(np.ptp(y_fit_deg))

    stat_mask = m_full if mode == "VOL3" else m_stat
    mah_stats = {
        "PITCH": _axis_fit_stats(true_p[stat_mask], mah_p[stat_mask]),
        "ROLL": _axis_fit_stats(true_r[stat_mask], mah_r[stat_mask]),
        "YAW": _axis_fit_stats(true_y[stat_mask], mah_y[stat_mask]),
    }
    cun_stats = {
        "PITCH": _axis_fit_stats(rel_p[stat_mask], cun_p[stat_mask]),
        "ROLL": _axis_fit_stats(rel_r[stat_mask], cun_r[stat_mask]),
        "YAW": _axis_fit_stats(rel_y[stat_mask], cun_y[stat_mask]),
    }
    est_stats = {
        "PITCH": _axis_fit_stats(rel_p[stat_mask], est_p[stat_mask]),
        "ROLL": _axis_fit_stats(rel_r[stat_mask], est_r[stat_mask]),
        "YAW": _axis_fit_stats(rel_y[stat_mask], est_y[stat_mask]),
    }
    spd_stats = {
        "PITCH": _axis_fit_stats(true_wp[stat_mask], mah_wp[stat_mask]),
        "ROLL": _axis_fit_stats(true_wr[stat_mask], mah_wr[stat_mask]),
        "YAW": _axis_fit_stats(true_wy[stat_mask], mah_wy[stat_mask]),
    }

    vol3_steady = None
    if mode == "VOL3":
        m_ss = t_arr >= 5.0
        vol3_steady = {
            "true_mean_deg": {"P": round(float(np.mean(true_p[m_ss])), 3), "R": round(float(np.mean(true_r[m_ss])), 3), "Y": round(float(np.mean(true_y[m_ss])), 3)},
            "mah_mean_deg": {"P": round(float(np.mean(mah_p[m_ss])), 3), "R": round(float(np.mean(mah_r[m_ss])), 3), "Y": round(float(np.mean(mah_y[m_ss])), 3)},
            "cun_mean_deg": {"P": round(float(np.mean(cun_p[m_ss])), 3), "R": round(float(np.mean(cun_r[m_ss])), 3), "Y": round(float(np.mean(cun_y[m_ss])), 3)},
            "mah_ss_rmse_deg": {
                "P": round(float(np.sqrt(np.mean((mah_p[m_ss] - true_p[m_ss]) ** 2))), 4),
                "R": round(float(np.sqrt(np.mean((mah_r[m_ss] - true_r[m_ss]) ** 2))), 4),
                "Y": round(float(np.sqrt(np.mean((mah_y[m_ss] - true_y[m_ss]) ** 2))), 4),
            },
        }

    rec = {
        "name": name,
        "mode": mode,
        "bool_banner": bool_banner,
        "aborted": aborted,
        "abort_time_s": round(abort_time_s, 3),
        "bode_score": round(bode_score, 2),
        "gain_loss_pts": round(gain_loss_pts, 2),
        "phase_loss_pts": round(phase_loss_pts, 2),
        "hf_noise_rms": round(hf_noise_rms, 3),
        "corr_vel": round(corr_vel, 4),
        "xtalk_penalty": round(xtalk_penalty, 2),
        "lowfreq_ptp_deg": round(lowfreq_ptp_deg, 4),
        "mahony_stats": mah_stats,
        "cuneate_stats": cun_stats,
        "est_neck_stats": est_stats,
        "mahony_speed_stats": spd_stats,
        "vol3_steady": vol3_steady,
        "freq_results": freq_results,
    }
    return idx, rec, t_arr, (-r_arr)


def main():
    parser = argparse.ArgumentParser(description="Validate Corrected 3-Axis Mahony Estimator")
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()

    t0 = time.time()
    g35 = {k: float(v) for k, v in json.load(open(G35_JSON, "r", encoding="utf-8"))["params"].items()}
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}
    w6_data = json.load(open(W6_JSON, "r", encoding="utf-8"))["records"]
    w6_15 = next(r for r in w6_data if r["name"] == "W6_15_G35_SOFTLEAD_KSC46")["params"]
    w6_15 = {k: float(v) for k, v in w6_15.items()}

    g2_eps = dict(g2_clean)
    g2_eps["K_sc_roll"] = float(g2_clean["K_sc_roll"]) * (1.0 + 1e-9)

    vol3_p = dict(g2_clean)
    vol3_p["desired_pitch"] = math.radians(8.0)
    vol3_p["desired_roll"] = math.radians(6.0)
    vol3_p["desired_yaw"] = math.radians(12.0)

    tasks = [
        (0, "APEO_G35", "APEO", g35),
        (1, "YawMA_G35", "YawMA", g35),
        (2, "LatEC_G2_CLEAN", "LatEC", g2_clean),
        (3, "LatEC_G2_EPS1E9", "LatEC", g2_eps),
        (4, "LatEC_W615_CLEAN", "LatEC", w6_15),
        (5, "VOL_3AXIS_TURN", "VOL3", vol3_p),
    ]

    records = [None] * len(tasks)
    r_series = {}
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_eval_validation_job, t) for t in tasks]
        for fut in as_completed(futs):
            idx, rec, t_arr, r_iso = fut.result()
            records[idx] = rec
            r_series[rec["name"]] = (t_arr, r_iso)
            print(f"  [done] {rec['name']:18s} mode={rec['mode']:5s} Bode={rec['bode_score']:8.2f} HF={rec['hf_noise_rms']:6.3f}", flush=True)

    # Check +1e-9 divergence between LatEC_G2_CLEAN and LatEC_G2_EPS1E9
    t_a, r_a = r_series["LatEC_G2_CLEAN"]
    t_b, r_b = r_series["LatEC_G2_EPS1E9"]
    n_c = min(len(r_a), len(r_b))
    dr = np.abs(r_b[:n_c] - r_a[:n_c])
    idx_1e4 = np.where(dr > 1e-4)[0]
    idx_0p05 = np.where(dr > 0.05)[0]
    eps_div = {
        "t_div_1e4_deg_s": round(float(t_a[idx_1e4[0]]), 3) if len(idx_1e4) > 0 else None,
        "t_div_0p05_deg_s": round(float(t_a[idx_0p05[0]]), 3) if len(idx_0p05) > 0 else None,
        "max_abs_dR_deg": round(float(np.max(dr)), 6),
        "bode_diff_pts": round(float(records[3]["bode_score"] - records[2]["bode_score"]), 2),
    }

    dt = time.time() - t0
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(dt, 1), "records": records, "eps_divergence": eps_div}, f, indent=2)

    print(f"\n=== 3-Axis Corrected Mahony Validation ({dt:.1f}s) ===", flush=True)
    for r in records:
        print(
            f"\n[{r['name']}] ({r['mode']}) | Abort={r['aborted']} | Bode={r['bode_score']:.2f} "
            f"(Gain={r['gain_loss_pts']:.1f}, Phs={r['phase_loss_pts']:.1f}) | HF={r['hf_noise_rms']:.3f} | LowPtP={r['lowfreq_ptp_deg']:.3f}"
        )
        for ax in ("PITCH", "ROLL", "YAW"):
            ms = r["mahony_stats"][ax]
            cs = r["cuneate_stats"][ax]
            print(
                f"  {ax:5s}: Mahony slope={ms['slope']:+6.3f} r={ms['r']:+7.4f} RMSE={ms['rmse_deg']:6.4f} (PtP {ms['true_ptp']:.3f}->{ms['est_ptp']:.3f}) | "
                f"Cuneate slope={cs['slope']:+6.3f} r={cs['r']:+7.4f} RMSE={cs['rmse_deg']:6.4f}"
            )
        if r["vol3_steady"]:
            print(f"  VOL3 steady (t=5..10s): {r['vol3_steady']}")

    print(f"\n+1e-9 Lyapunov Divergence Check (LatEC_G2_CLEAN vs EPS1E9): {eps_div}", flush=True)


if __name__ == "__main__":
    main()
