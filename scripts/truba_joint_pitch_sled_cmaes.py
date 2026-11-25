#!/usr/bin/env python3
"""Overlapped Fast-5s-APEO + Zheng 2g/3g Joint 18D Pitch CMA-ES & Exact 15.8s Finalist Verifier.

Speed design for TRUBA orfoz (-n 56):
  - Gen 0 calibrates `base_apeo_fast` on a 5.0 s APEO multisine surrogate (~3.2x faster than 15.8 s)
    with a 420 s per-run wall-clock timeout guard so stiff draws never stall a generation.
  - Each generation evaluates pop=26 candidates by dispatching 26 `apeo_fast` tasks + 26
    `sled_bundle` tasks (2g u=0, 2g u=-0.75, 3g u=0) simultaneously across 56 workers (~3.5 min/gen).
  - After Gen 40, the top 6 distinct archive finalists are re-verified with the exact 15.8 s
    `generate_standard_plot.run_axis_worker(('APEO', ...))` in parallel (~11 min).
"""

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

for _env_k in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "SIMBODY_NUM_THREADS",
):
    os.environ.setdefault(_env_k, "1")

import cma
import numpy as np
import opensim as osim

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cmaes_worker_p22 import CANONICAL_TARGETS, PER_MUSCLE_72_U0, ensure_libraries_loaded
from scripts.generate_standard_plot import (
    MODEL_PATH,
    apply_cli_overrides_and_inspect,
    run_axis_worker,
)
from scripts.run_roll_yaw_baseline import build_c2_multisine_spline
from scripts.run_zheng_2g_3g_urgency_sweep import (
    DT_LOG,
    SPLEN_CAP_NAMES,
    T_SETTLE,
    T_TOTAL,
    build_sled_spline,
)

BASE_CHAMP_JSON = os.path.join(_ROOT, "logs", "se2_c1pos_3axis_champion.json")
CHAMP_JSON = os.path.join(_ROOT, "logs", "truba_joint_pitch_sled_champion.json")
CLEAN_CHAMP_JSON = os.path.join(_ROOT, "logs", "truba_joint_pitch_sled_clean_champion.json")
HIST_JSON = os.path.join(_ROOT, "logs", "truba_joint_pitch_sled_history.json")


def atomic_write_json(path: str, payload: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, path)


def build_pitch18_specs(base_params: dict):
    return [
        ("tau_lead_pole",     base_params["tau_lead_pole"],     0.004,  0.040, 0.0030),
        ("K_sc",              base_params["K_sc"],              1.50,  10.00, 0.55),
        ("z_sc",              base_params["z_sc"],              0.001,  0.045, 0.0035),
        ("p_sc",              base_params["p_sc"],              15.0,  110.0, 8.0),
        ("K_phas",            base_params["K_phas"],            0.0,   20.00, 2.00),
        ("z_phas",            base_params["z_phas"],            0.0,    0.040, 0.0050),
        ("p_phas",            base_params["p_phas"],            0.50,  20.00, 1.50),
        ("K_ton",             base_params["K_ton"],             1.00,  16.00, 1.20),
        ("p_ton",             base_params["p_ton"],             3.00,  40.00, 3.00),
        ("Kp_prop_pitch",     base_params["Kp_prop_pitch"],     0.05,   3.00, 0.20),
        ("z_ccr",             base_params["z_ccr"],             0.001,  0.050, 0.0040),
        ("p_ccr",             base_params["p_ccr"],             20.0,  160.0, 14.0),
        ("Kp_proprioception", base_params["Kp_proprioception"], 0.001,  0.150, 0.010),
        ("Kp_task_pitch",     base_params["Kp_task_pitch"],     1.00,  12.00, 0.80),
        ("Kd_task_pitch",     base_params["Kd_task_pitch"],     0.05,   3.00, 0.25),
        ("Ki_task_pitch",     base_params["Ki_task_pitch"],     0.20,   8.00, 0.60),
        ("Kp_buckle",         base_params["Kp_buckle"],         0.20,   4.50, 0.30),
        ("Kd_buckle",         base_params["Kd_buckle"],         0.02,   0.60, 0.040),
    ]


def params_to_overrides(params: dict) -> list:
    return [
        (k, repr(float(v)))
        for k, v in params.items()
        if k not in ("enable_sensor_noise", "enable_motor_noise")
    ] + [
        ("enable_sensor_noise", "false"),
        ("enable_motor_noise", "false"),
        ("highg_oto_knee_mps2", "7.80"),
    ]


def run_apeo_fast(overrides: list, wall_limit_s: float = 420.0) -> dict:
    """Fast 5.0 s APEO multisine surrogate (delay=0.4s, ramp=0.8s, steady>=1.2s) with wall timeout."""
    t0 = time.time()
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

    bool_state, _, _ = apply_cli_overrides_and_inspect(controller, overrides)
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

    delay = 0.4
    ramp_duration = 0.8
    target_time = 5.0
    eval_start_time = 1.2

    ordered_targets = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
    freqs = np.array([t[0] for t in ordered_targets], dtype=float)
    tgt_gains = np.array([t[1] for t in ordered_targets], dtype=float)
    tgt_phases = np.array([t[2] for t in ordered_targets], dtype=float)
    K = len(freqs)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
    amps_input_unit = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
    ms_spline, _ = build_c2_multisine_spline(freqs, amps_input_unit, phases, delay, ramp_duration, target_time)

    coord_pert = model.updCoordinateSet().get("spine_tx")
    coord_pert.set_locked(False)
    coord_pert.setDefaultIsPrescribed(True)
    coord_pert.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)
    model.realizePosition(state)

    skull = model.getBodySet().get("skull")
    q = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
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

    t_list, p_list, r_list, y_list = [], [], [], []
    aborted = False
    abort_time_s = float(target_time)
    abort_reason = ""

    while current_time < target_time - 1e-9:
        if (time.time() - t0) > wall_limit_s:
            aborted = True
            abort_time_s = float(current_time)
            abort_reason = f"wall_timeout>{wall_limit_s:.0f}s at t={current_time:.2f}s"
            break
        current_time = min(current_time + step, target_time)
        try:
            state = manager.integrate(current_time)
        except Exception as e:
            aborted = True
            abort_time_s = float(current_time)
            abort_reason = str(e).splitlines()[0]
            break

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        r1_deg = math.degrees(coord_r1.getValue(state))
        r2_deg = math.degrees(coord_r2.getValue(state))
        y1_deg = math.degrees(coord_y1.getValue(state))
        y2_deg = math.degrees(coord_y2.getValue(state))
        p_deg = p1_deg + p2_deg
        r_deg = r1_deg + r2_deg
        y_rel_deg = y1_deg + y2_deg

        t_list.append(current_time)
        p_list.append(p_deg)
        r_list.append(r_deg)
        y_list.append(y_rel_deg)

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

    t_arr = np.asarray(t_list, dtype=float)
    p_arr = np.asarray(p_list, dtype=float)
    r_arr = np.asarray(r_list, dtype=float)
    y_arr = np.asarray(y_list, dtype=float)
    m_post = t_arr >= eval_start_time
    n_post = int(np.sum(m_post))

    full_roll_max = float(np.max(np.abs(r_arr))) if len(r_arr) else 0.0
    full_yaw_max = float(np.max(np.abs(y_arr))) if len(y_arr) else 0.0
    post_roll_max = float(np.max(np.abs(r_arr[m_post]))) if np.any(m_post) else full_roll_max
    post_yaw_max = float(np.max(np.abs(y_arr[m_post]))) if np.any(m_post) else full_yaw_max

    if aborted or n_post < 50:
        progress = max(0.0, min(1.0, abort_time_s / target_time))
        classic_score = 6.0 * (200000.0 + (1.0 - progress) * 50000.0)
        xtalk = 100.0 * (full_roll_max ** 2 + full_yaw_max ** 2)
        total_score = classic_score + xtalk
        hf_rms = 0.0
    else:
        t_fit = t_arr[m_post]
        t_rel = t_fit - delay
        y_fit_deg = p_arr[m_post]

        cols = [np.ones_like(t_fit), t_fit - eval_start_time]
        for k in range(K):
            psi_k = 2.0 * math.pi * float(freqs[k]) * t_rel + float(phases[k])
            cols.append(np.sin(psi_k))
            cols.append(np.cos(psi_k))
        coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit_deg, rcond=None)

        sim_harm_fit = np.zeros_like(t_fit)
        gain_loss = 0.0
        phase_loss = 0.0
        for k in range(K):
            f_k = float(freqs[k])
            psi = 2.0 * np.pi * f_k * t_rel + float(phases[k])
            a_sin = float(coeffs[2 + 2 * k])
            b_cos = float(coeffs[2 + 2 * k + 1])
            sim_harm_fit += a_sin * np.sin(psi) + b_cos * np.cos(psi)

            amp_out_deg = math.hypot(a_sin, b_cos)
            sim_gain = amp_out_deg / float(amps_input_unit[k])
            raw_phase = math.degrees(math.atan2(b_cos, a_sin))
            ph_err = ((raw_phase - float(tgt_phases[k]) + 180.0) % 360.0) - 180.0
            db_err = 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k]))
            gain_loss += 100.0 * (db_err ** 2)
            phase_loss += 1.0 * (ph_err ** 2)

        vel_actual = np.gradient(y_fit_deg, t_fit)
        vel_harm = np.gradient(sim_harm_fit, t_fit)
        hf_rms = float(np.sqrt(np.mean((vel_actual - vel_harm) ** 2)))
        classic_score = gain_loss + phase_loss
        xtalk = 100.0 * (post_roll_max ** 2 + post_yaw_max ** 2)
        hf_pen = 5.0 * (hf_rms ** 2)
        total_score = classic_score + xtalk + hf_pen

    return {
        "aborted": bool(aborted),
        "abort_time_s": float(abort_time_s),
        "abort_reason": abort_reason,
        "classic_bode_score": float(classic_score),
        "hf_noise_rms": float(hf_rms),
        "total_score": float(total_score),
        "wall_s": round(time.time() - t0, 2),
    }


def run_sled_eval(params_dict: dict, g_level: float, urgency: float) -> dict:
    """Fast in-memory replica of scripts/run_zheng_2g_3g_urgency_sweep.py::run_single_case."""
    ensure_libraries_loaded()

    init_overrides = dict(params_dict)
    init_overrides.update({
        "w_pos_ap": 0.0,
        "w_pos_lat": 0.0,
        "Kp_trans_roll": 0.0,
        "Kp_trans_pitch": 0.0,
        "Kp_shift_obs_px": 0.0,
        "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0,
        "Kv_shift_obs_vz": 0.0,
        "shift_station_blend": 0.0,
        "trajectory_tau": 0.28,
        "urgency": float(urgency),
        "desired_pitch": 0.0,
        "desired_roll": 0.0,
        "desired_yaw": 0.0,
        "desired_ap_shift": 0.0,
        "desired_lat_shift": 0.0,
        "enable_sensor_noise": False,
        "enable_motor_noise": False,
        "highg_oto_knee_mps2": 7.80,
    })

    model = osim.Model(MODEL_PATH)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.updControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl
    if controller is None:
        raise RuntimeError("HeadNeckNeuralController not found in model")

    override_pairs = [
        (k, ("true" if v else "false") if isinstance(v, bool) else repr(float(v)))
        for k, v in init_overrides.items()
        if controller.hasProperty(k)
    ]
    apply_cli_overrides_and_inspect(controller, override_pairs)

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

    sled_spline = build_sled_spline(g_level, sign_x=-1.0)
    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.setPrescribedFunction(sled_spline)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)
    model.realizePosition(state)

    skull = model.getBodySet().get("skull")
    q0 = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q0.get(idx))
    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral", "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    k_gdyn = float(init_overrides.get("K_gamma_dyn", 0.857744))
    k_gstat = float(init_overrides.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    splen_indices = []
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        if mname in SPLEN_CAP_NAMES:
            splen_indices.append(i)
        u0 = PER_MUSCLE_72_U0.get(mname, 0.030)
        m.setActivation(state, u0)
        gd0 = 100.0 * k_gdyn * (0.25 + 0.75 * u0)
        gs0 = 100.0 * k_gstat * u0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{mname}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{mname}", fstat0)

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

    cs = model.getCoordinateSet()
    c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
    ts_abs, flex_abs, splen_act = [], [], []

    def sample(tv, st):
        model.realizeDynamics(st)
        ts_abs.append(float(tv))
        flex_abs.append(-math.degrees(c_p1.getValue(st) + c_p2.getValue(st)))
        splen_act.append(float(sum(float(muscles.get(idx).getActivation(st)) for idx in splen_indices) / len(splen_indices)))

    sample(0.0, state)
    t_now = 0.0
    stop_reason = None
    while t_now < T_TOTAL - 1e-9:
        t_next = round(min(t_now + DT_LOG, T_TOTAL), 6)
        try:
            state = manager.integrate(t_next)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]
            break
        t_now = t_next
        sample(t_now, state)

    if stop_reason is not None or len(ts_abs) < 200:
        return {
            "aborted": True,
            "stop_reason": stop_reason or "short_run",
            "pk_flex": 0.0,
            "min_ons": -99.0,
            "dip": 1.0,
            "early_excess": 1.0,
            "n_pk": 9,
            "peak_splen": 0.0,
        }

    ts_arr = np.asarray(ts_abs, dtype=float)
    f_arr = np.asarray(flex_abs, dtype=float)
    sp_arr = np.asarray(splen_act, dtype=float)

    imp_mask = ts_arr >= (T_SETTLE - 1e-9)
    idx0 = int(np.where(imp_mask)[0][0])
    t_imp_ms = (ts_arr[imp_mask] - T_SETTLE) * 1000.0
    f_imp = f_arr[imp_mask] - float(f_arr[idx0])
    sp_imp = sp_arr[imp_mask]

    pk_flex = float(np.max(f_imp))
    ons_mask = t_imp_ms <= 150.0 + 1e-9
    min_ons = float(np.min(f_imp[ons_mask])) if np.any(ons_mask) else 0.0
    peak_splen = float(np.max(sp_imp))

    dip = 0.0
    n_sp = len(sp_imp)
    for k in range(1, n_sp - 1):
        if 55.0 <= t_imp_ms[k] <= 115.0:
            if sp_imp[k] > 0.06 and sp_imp[k] > sp_imp[k - 1] and sp_imp[k] >= sp_imp[k + 1]:
                hi = min(n_sp, k + 30)
                j_min = k + int(np.argmin(sp_imp[k:hi]))
                after_mask = (np.arange(n_sp) >= j_min) & (t_imp_ms <= 250.0 + 1e-9)
                if np.any(after_mask):
                    after_max = float(np.max(sp_imp[after_mask]))
                    if after_max > sp_imp[j_min] + 0.002:
                        dip = max(dip, float(sp_imp[k] - sp_imp[j_min]))

    m_early = (t_imp_ms >= 50.0) & (t_imp_ms <= 115.0)
    m_main = (t_imp_ms >= 120.0) & (t_imp_ms <= 250.0)
    max_early = float(np.max(sp_imp[m_early])) if np.any(m_early) else 0.0
    max_main = float(np.max(sp_imp[m_main])) if np.any(m_main) else 0.0
    early_excess = max(0.0, max_early - max_main)

    n_pk = 0
    for k in range(1, n_sp - 1):
        if sp_imp[k] > 0.08 and sp_imp[k] > sp_imp[k - 1] and sp_imp[k] >= sp_imp[k + 1]:
            lo = max(0, k - 25)
            hi = min(n_sp, k + 26)
            prom = float(sp_imp[k] - max(np.min(sp_imp[lo : k + 1]), np.min(sp_imp[k:hi])))
            if prom >= 0.015:
                n_pk += 1

    return {
        "aborted": False,
        "stop_reason": None,
        "pk_flex": pk_flex,
        "min_ons": min_ons,
        "dip": dip,
        "early_excess": early_excess,
        "n_pk": n_pk,
        "peak_splen": peak_splen,
    }


def dispatch_subtask(task: dict) -> dict:
    kind = task["kind"]
    idx = task["idx"]
    params = dict(task["base_params"])
    params.update(task["pitch_params"])
    overrides = params_to_overrides(params)

    if kind == "apeo_fast":
        res = run_apeo_fast(overrides, wall_limit_s=420.0)
        return {"kind": kind, "idx": idx, "res": res}
    if kind == "apeo_exact":
        t0 = time.time()
        ensure_libraries_loaded()
        res = run_axis_worker(("APEO", overrides))
        return {
            "kind": kind,
            "idx": idx,
            "res": {
                "aborted": bool(res["aborted"]),
                "abort_time_s": float(res["abort_time_s"]),
                "classic_bode_score": float(res["classic_bode_score"]),
                "hf_noise_rms": float(res["hf_noise_rms"]),
                "total_score": float(res["total_score"]),
                "wall_s": round(time.time() - t0, 2),
            },
        }
    if kind == "sled_bundle":
        t0 = time.time()
        s2_u0 = run_sled_eval(params, 2.0, 0.00)
        s2_nb = run_sled_eval(params, 2.0, -0.75)
        s3_u0 = run_sled_eval(params, 3.0, 0.00)
        return {
            "kind": kind,
            "idx": idx,
            "res": {
                "s2_u0": s2_u0,
                "s2_nb": s2_nb,
                "s3_u0": s3_u0,
                "wall_s": round(time.time() - t0, 2),
            },
        }
    raise ValueError(f"Unknown task kind: {kind}")


def score_candidate(idx: int, pitch_params: dict, apeo_res: dict, sled_res: dict, bode_hinge: float, is_exact: bool = False) -> dict:
    apeo_tot = float(apeo_res["total_score"])
    apeo_bode = float(apeo_res["classic_bode_score"])
    apeo_hf = float(apeo_res["hf_noise_rms"])
    s2_u0 = sled_res["s2_u0"]
    s2_nb = sled_res["s2_nb"]
    s3_u0 = sled_res["s3_u0"]

    if apeo_res["aborted"] or s2_u0["aborted"] or s2_nb["aborted"] or s3_u0["aborted"]:
        return {
            "idx": idx,
            "pitch_params": pitch_params,
            "aborted": True,
            "is_exact": is_exact,
            "apeo_total": round(apeo_tot, 2),
            "apeo_bode": round(apeo_bode, 2),
            "apeo_hf": round(apeo_hf, 3),
            "loss": round(50000.0 + apeo_tot, 2),
            "is_clean": False,
            "apeo_wall_s": apeo_res["wall_s"],
            "sled_wall_s": sled_res["wall_s"],
        }

    loss_bode = apeo_tot + 6.0 * max(0.0, apeo_tot - bode_hinge)
    loss_onset = 5000.0 * (
        max(0.0, -s2_u0["min_ons"] - 0.15) ** 2 + max(0.0, -s3_u0["min_ons"] - 0.15) ** 2
    )
    loss_bimodal = (
        20000.0 * (s2_u0["dip"] + s3_u0["dip"])
        + 12000.0 * (s2_u0["early_excess"] + s3_u0["early_excess"])
        + 800.0 * (max(0, s2_u0["n_pk"] - 1) + max(0, s3_u0["n_pk"] - 1))
    )
    pk_nbdl = s2_nb["pk_flex"]
    loss_nbdl = (
        40.0 * max(0.0, 33.5 - pk_nbdl) ** 2
        + 40.0 * max(0.0, pk_nbdl - 39.5) ** 2
        + 3.0 * (pk_nbdl - 36.5) ** 2
    )
    pk2 = s2_u0["pk_flex"]
    pk3 = s3_u0["pk_flex"]
    loss_range = (
        400.0 * max(0.0, pk2 + 3.0 - pk3) ** 2
        + 400.0 * max(0.0, 4.0 - pk2) ** 2
        + 400.0 * max(0.0, pk2 - 15.0) ** 2
    )
    total_loss = loss_bode + loss_onset + loss_bimodal + loss_nbdl + loss_range

    clean_bode_cap = 2310.0 if is_exact else (bode_hinge + 25.0)
    is_clean = (
        apeo_tot <= clean_bode_cap
        and s2_u0["n_pk"] <= 1
        and s3_u0["n_pk"] <= 1
        and s2_u0["dip"] <= 0.015
        and s3_u0["dip"] <= 0.015
        and s2_u0["min_ons"] >= -0.40
        and s3_u0["min_ons"] >= -0.40
    )

    return {
        "idx": idx,
        "pitch_params": pitch_params,
        "aborted": False,
        "is_exact": is_exact,
        "apeo_total": round(apeo_tot, 2),
        "apeo_bode": round(apeo_bode, 2),
        "apeo_hf": round(apeo_hf, 3),
        "pk_2g_u0": round(pk2, 2),
        "min_ons_2g": round(s2_u0["min_ons"], 2),
        "dip_2g": round(s2_u0["dip"], 4),
        "early_exc_2g": round(s2_u0["early_excess"], 4),
        "n_pk_2g": int(s2_u0["n_pk"]),
        "spl_2g_u0": round(s2_u0["peak_splen"], 4),
        "pk_2g_nbdl": round(pk_nbdl, 2),
        "pk_3g_u0": round(pk3, 2),
        "min_ons_3g": round(s3_u0["min_ons"], 2),
        "dip_3g": round(s3_u0["dip"], 4),
        "early_exc_3g": round(s3_u0["early_excess"], 4),
        "n_pk_3g": int(s3_u0["n_pk"]),
        "spl_3g_u0": round(s3_u0["peak_splen"], 4),
        "loss": round(float(total_loss), 2),
        "loss_parts": {
            "bode": round(float(loss_bode), 1),
            "onset": round(float(loss_onset), 1),
            "bimodal": round(float(loss_bimodal), 1),
            "nbdl": round(float(loss_nbdl), 1),
            "range": round(float(loss_range), 1),
        },
        "is_clean": bool(is_clean),
        "apeo_wall_s": apeo_res["wall_s"],
        "sled_wall_s": sled_res["wall_s"],
    }


def make_champion_payload(base_obj: dict, best_rec: dict, label: str) -> dict:
    out_params = dict(base_obj["params"])
    out_params.update(best_rec["pitch_params"])
    out_params["highg_oto_knee_mps2"] = 7.80
    return {
        "source": f"{base_obj.get('source', 'C1-Pos')} + Joint Pitch+Sled ({label})",
        "se2_c1pos_params": dict(base_obj.get("se2_c1pos_params", {})),
        "best_sway_ap_config": dict(base_obj.get("best_sway_ap_config", {})),
        "joint_pitch_sled_metrics": {k: v for k, v in best_rec.items() if k != "pitch_params"},
        "params": out_params,
    }


def select_distinct_finalists(archive: list, names: list, stds: list, top_k: int = 6) -> list:
    valid = [r for r in archive if not r.get("aborted")]
    if not valid:
        return archive[:top_k]

    def norm_dist(a: dict, b: dict) -> float:
        return math.sqrt(sum(((a[n] - b[n]) / s) ** 2 for n, s in zip(names, stds)) / len(names))

    # Candidate pools: overall best loss, best clean, lowest dip near good Bode, lowest Bode
    ranked = sorted(valid, key=lambda r: r["loss"])
    clean_ranked = sorted([r for r in valid if r.get("is_clean")], key=lambda r: (r["loss"], r["apeo_total"]))
    low_dip_ranked = sorted(
        [r for r in valid if r["apeo_total"] <= valid[0]["apeo_total"] + 150.0 or r["loss"] < 15000.0],
        key=lambda r: (r["dip_2g"] + r["dip_3g"], r["loss"]),
    )

    chosen = []
    for pool_list in (clean_ranked[:2], ranked, low_dip_ranked):
        for r in pool_list:
            if len(chosen) >= top_k:
                break
            if all(norm_dist(r["pitch_params"], c["pitch_params"]) >= 0.08 for c in chosen):
                chosen.append(r)

    if len(chosen) < top_k:
        for r in ranked:
            if len(chosen) >= top_k:
                break
            if r not in chosen:
                chosen.append(r)
    return chosen[:top_k]


def run_cmaes(pool, base_obj: dict, specs, pop: int, gens: int):
    base_params = dict(base_obj["params"])
    names = [s[0] for s in specs]
    lbs = [float(s[2]) for s in specs]
    ubs = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]
    base_pitch = {n: float(base_params[n]) for n in names}

    # Gen 0 self-calibration on overlapped 5.0 s APEO_fast + sled
    print("[GEN 00] Calibrating base champion on 5.0 s APEO_fast + 2g/3g sled...", flush=True)
    g0_res = list(pool.map(dispatch_subtask, [
        {"kind": "apeo_fast", "idx": 0, "base_params": base_params, "pitch_params": base_pitch},
        {"kind": "sled_bundle", "idx": 0, "base_params": base_params, "pitch_params": base_pitch},
    ]))
    g0_apeo = g0_res[0]["res"]
    g0_sled = g0_res[1]["res"]
    base_apeo_fast = float(g0_apeo["total_score"])
    bode_hinge = base_apeo_fast + 25.0
    base_rec = score_candidate(0, base_pitch, g0_apeo, g0_sled, bode_hinge, is_exact=False)
    base_rec["_sled_res"] = g0_sled
    print(
        f"[GEN 00] Base calibration: apeo_fast={base_apeo_fast:.2f} (hinge={bode_hinge:.2f}, wall={g0_apeo['wall_s']:.1f}s), "
        f"2g={base_rec['pk_2g_u0']:.2f}/dip={base_rec['dip_2g']:.3f}/nPk={base_rec['n_pk_2g']}, "
        f"3g={base_rec['pk_3g_u0']:.2f}/dip={base_rec['dip_3g']:.3f}/nPk={base_rec['n_pk_3g']}, "
        f"NBDL={base_rec['pk_2g_nbdl']:.2f}, loss={base_rec['loss']:.1f}",
        flush=True,
    )

    x0 = [float(np.clip(base_pitch[n], lb, ub)) for n, lb, ub in zip(names, lbs, ubs)]
    es = cma.CMAEvolutionStrategy(
        x0,
        0.40,
        {
            "bounds": [lbs, ubs],
            "CMA_stds": stds,
            "popsize": pop,
            "maxiter": gens,
            "seed": 20261124,
            "verb_disp": 0,
            "verb_log": 0,
        },
    )

    best_overall = base_rec
    best_clean = base_rec if base_rec["is_clean"] else None
    archive = [base_rec]
    history = []
    t_start = time.time()

    atomic_write_json(CHAMP_JSON, make_champion_payload(base_obj, best_overall, "Gen00_BaseFast"))

    for gen in range(1, gens + 1):
        if es.stop():
            print(f"[CMAES] Stopping early at gen {gen}: {es.stop()}", flush=True)
            break
        t_gen = time.time()
        xs = es.ask()
        pitch_maps = [
            {n: float(np.clip(v, lb, ub)) for n, v, lb, ub in zip(names, vec, lbs, ubs)}
            for vec in xs
        ]
        # Submit 26 APEO_fast + 26 sled_bundle tasks simultaneously across 56 cores
        subtasks = []
        for idx, p_map in enumerate(pitch_maps):
            subtasks.append({"kind": "apeo_fast", "idx": idx, "base_params": base_params, "pitch_params": p_map})
            subtasks.append({"kind": "sled_bundle", "idx": idx, "base_params": base_params, "pitch_params": p_map})

        out_list = list(pool.map(dispatch_subtask, subtasks))
        apeo_by_idx = {o["idx"]: o["res"] for o in out_list if o["kind"] == "apeo_fast"}
        sled_by_idx = {o["idx"]: o["res"] for o in out_list if o["kind"] == "sled_bundle"}

        evals = []
        for idx, p_map in enumerate(pitch_maps):
            rec = score_candidate(idx, p_map, apeo_by_idx[idx], sled_by_idx[idx], bode_hinge, is_exact=False)
            rec["_sled_res"] = sled_by_idx[idx]
            rec["gen"] = gen
            evals.append(rec)
            if not rec["aborted"]:
                archive.append(rec)

        losses = [float(e["loss"]) for e in evals]
        es.tell(xs, losses)

        # Bound archive size in memory to top 60 valid candidates
        archive = sorted(archive, key=lambda r: r["loss"])[:60]

        gen_best = min(evals, key=lambda r: r["loss"])
        if gen_best["loss"] < best_overall["loss"]:
            best_overall = gen_best
            atomic_write_json(CHAMP_JSON, make_champion_payload(base_obj, best_overall, f"Gen{gen:02d}_FastBest"))

        clean_in_gen = [e for e in evals if e.get("is_clean")]
        if clean_in_gen:
            clean_g = min(clean_in_gen, key=lambda r: (r["loss"], r["apeo_total"]))
            if best_clean is None or clean_g["loss"] < best_clean["loss"]:
                best_clean = clean_g
                atomic_write_json(CLEAN_CHAMP_JSON, make_champion_payload(base_obj, best_clean, f"Gen{gen:02d}_FastClean"))

        max_aw = max(e["apeo_wall_s"] for e in evals)
        n_ab = sum(1 for e in evals if e["aborted"])
        history.append({
            "gen": gen,
            "gen_wall_s": round(time.time() - t_gen, 1),
            "max_apeo_wall_s": round(max_aw, 1),
            "n_aborted": n_ab,
            "gen_best_loss": gen_best["loss"],
            "gen_best_apeo": gen_best["apeo_total"],
            "best_overall_loss": best_overall["loss"],
            "best_overall_apeo": best_overall["apeo_total"],
            "best_clean_loss": best_clean["loss"] if best_clean else None,
            "best_clean_apeo": best_clean["apeo_total"] if best_clean else None,
        })
        atomic_write_json(HIST_JSON, {
            "elapsed_s": round(time.time() - t_start, 1),
            "base_apeo_fast": round(base_apeo_fast, 2),
            "bode_hinge_fast": round(bode_hinge, 2),
            "best_overall": {k: v for k, v in best_overall.items() if k != "_sled_res"},
            "best_clean": {k: v for k, v in best_clean.items() if k != "_sled_res"} if best_clean else None,
            "history": history,
        })

        bo_s2 = f"{best_overall.get('pk_2g_u0', 0):.2f}/dip={best_overall.get('dip_2g', -1):.3f}/nPk={best_overall.get('n_pk_2g', -1)}"
        bo_s3 = f"{best_overall.get('pk_3g_u0', 0):.2f}/dip={best_overall.get('dip_3g', -1):.3f}/nPk={best_overall.get('n_pk_3g', -1)}"
        bc_tag = (
            f"CLEAN(loss={best_clean['loss']:.1f}, fast={best_clean['apeo_total']:.1f}, 2g={best_clean['pk_2g_u0']:.2f}, 3g={best_clean['pk_3g_u0']:.2f})"
            if best_clean
            else "CLEAN=None"
        )
        print(
            f"[GEN {gen:02d}/{gens:02d}] {time.time() - t_gen:5.1f}s (maxA={max_aw:5.1f}s, ab={n_ab:2d}) | "
            f"GenBest={gen_best['loss']:8.1f} (fast={gen_best['apeo_total']:7.1f}) | "
            f"Best={best_overall['loss']:8.1f} (fast={best_overall['apeo_total']:7.1f}, "
            f"2g={bo_s2}, 3g={bo_s3}, NBDL={best_overall.get('pk_2g_nbdl', 0):.2f}, ons={best_overall.get('min_ons_2g', 0):+.2f}/{best_overall.get('min_ons_3g', 0):+.2f}) | {bc_tag}",
            flush=True,
        )

    # Exact 15.8 s APEO verification on top 6 distinct finalists in parallel
    finalists = select_distinct_finalists(archive, names, stds, top_k=6)
    print(f"\n[FINALIST VERIFY] Running exact 15.8 s APEO in parallel on top {len(finalists)} finalists...", flush=True)
    exact_tasks = [
        {"kind": "apeo_exact", "idx": i, "base_params": base_params, "pitch_params": f["pitch_params"]}
        for i, f in enumerate(finalists)
    ]
    exact_out = list(pool.map(dispatch_subtask, exact_tasks))
    exact_by_idx = {o["idx"]: o["res"] for o in exact_out}

    verified = []
    for i, f in enumerate(finalists):
        v_rec = score_candidate(i, f["pitch_params"], exact_by_idx[i], f["_sled_res"], bode_hinge=2270.0, is_exact=True)
        v_rec["fast_apeo"] = f["apeo_total"]
        v_rec["fast_loss"] = f["loss"]
        v_rec["from_gen"] = f.get("gen", 0)
        verified.append(v_rec)

    verified_sorted = sorted(verified, key=lambda r: r["loss"])
    print(
        "[FINALIST VERIFY] Results:\n"
        f"{'#':>2} | {'Gen':>3} | {'ExactLoss':>9} | {'ExactAPEO':>9} | {'FastAPEO':>8} | {'2g(pk/ons/dip/nPk)':>22} | {'NBDL2g':>6} | {'3g(pk/ons/dip/nPk)':>22} | Clean",
        flush=True,
    )
    for r in verified_sorted:
        if r["aborted"]:
            print(f"{r['idx']:>2} | {r['from_gen']:>3} | ABORTED   | {r['apeo_total']:9.2f} | {r['fast_apeo']:8.2f} | ABORTED                |   ---- | ABORTED                | False", flush=True)
        else:
            s2 = f"{r['pk_2g_u0']:5.2f}/{r['min_ons_2g']:+5.2f}/{r['dip_2g']:.3f}/{r['n_pk_2g']}"
            s3 = f"{r['pk_3g_u0']:5.2f}/{r['min_ons_3g']:+5.2f}/{r['dip_3g']:.3f}/{r['n_pk_3g']}"
            print(f"{r['idx']:>2} | {r['from_gen']:>3} | {r['loss']:9.1f} | {r['apeo_total']:9.2f} | {r['fast_apeo']:8.2f} | {s2:>22} | {r['pk_2g_nbdl']:6.2f} | {s3:>22} | {r['is_clean']}", flush=True)

    valid_verified = [r for r in verified_sorted if not r["aborted"]]
    if valid_verified:
        final_best = valid_verified[0]
        atomic_write_json(CHAMP_JSON, make_champion_payload(base_obj, final_best, "Exact15s_Best"))
        clean_verified = [r for r in valid_verified if r["is_clean"]]
        if clean_verified:
            final_clean = min(clean_verified, key=lambda r: (r["loss"], r["apeo_total"]))
            atomic_write_json(CLEAN_CHAMP_JSON, make_champion_payload(base_obj, final_clean, "Exact15s_Clean"))

    atomic_write_json(HIST_JSON, {
        "elapsed_s": round(time.time() - t_start, 1),
        "base_apeo_fast": round(base_apeo_fast, 2),
        "bode_hinge_fast": round(bode_hinge, 2),
        "finalists_exact": [{k: v for k, v in r.items() if k != "pitch_params"} for r in verified_sorted],
        "history": history,
    })
    print(f"[CMAES] Finished completely in {time.time() - t_start:.1f} s.", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=["cmaes"], default="cmaes")
    parser.add_argument("--pop", type=int, default=26)
    parser.add_argument("--gens", type=int, default=40)
    parser.add_argument("--workers", type=int, default=56)
    args = parser.parse_args()

    base_obj = json.load(open(BASE_CHAMP_JSON, "r", encoding="utf-8"))
    specs = build_pitch18_specs(base_obj["params"])

    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        run_cmaes(pool, base_obj, specs, pop=args.pop, gens=args.gens)


if __name__ == "__main__":
    main()
