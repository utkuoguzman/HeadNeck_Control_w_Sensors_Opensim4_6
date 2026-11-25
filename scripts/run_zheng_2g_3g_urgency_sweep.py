#!/usr/bin/env python3
"""1D Cortical Urgency (u) Sweep on Zheng et al. (2021) / NBDL 2g and 3g Frontal Sled Pulses."""

import json
import math
import os
import shutil
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

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import PER_MUSCLE_72_U0, ensure_libraries_loaded
from scripts.generate_standard_plot import apply_cli_overrides_and_inspect
from validate_5dof_voluntary_and_se2_shift import (
    CHAMP_C1_JSON,
    MODEL_PATH,
)

ART_DIR = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae"
OUT_JSON = os.path.join(_ROOT, "logs", "zheng_2g_3g_urgency_sweep.json")
OUT_PNG = os.path.join(_ROOT, "results", "zheng_2g_3g_urgency_sweep.png")

G_ACC = 9.81
G_LEVELS = [2.0, 3.0]
URGENCY_LIST = [-1.00, -0.75, -0.50, -0.25, 0.00, 0.20, 0.50, 1.00]

T_SETTLE = 0.35
T_IMPACT = 0.35
T_TOTAL = T_SETTLE + T_IMPACT  # 0.70 s
DT_LOG = 0.002  # 500 Hz

# Zheng et al. (2021) C^2-smooth haversine pulse timing relative to impact onset (t_imp = t - T_SETTLE)
T_PULSE_START = 0.0375
T_PULSE_END = 0.2125
T_PULSE_DUR = T_PULSE_END - T_PULSE_START  # 0.175 s (peak at 0.125 s = 125.0 ms)

SPLEN_CAP_NAMES = (
    "splen_cap_sklc6",
    "splen_cap_sklc6_L",
    "splen_cap_sklthx",
    "splen_cap_sklthx_L",
)
SCM_NAMES = ("stern_mast", "stern_mast_L")


def compute_engagement(u: float) -> float:
    """Exact replica of HeadNeckNeuralController.cpp computeVoluntaryEngagement(u)."""
    u_c = min(max(float(u), 0.0), 1.0)
    if u_c <= 0.0:
        return 0.0
    k_norm = math.tanh(1.0 / 0.15)
    return min(max(math.tanh(u_c / 0.15) / k_norm, 0.0), 1.0)


def compute_supraspinal_alertness(u: float) -> float:
    if u >= 0.0:
        return 1.0
    a = min(max(1.0 + float(u), 0.0), 1.0)
    return a * a


def compute_spinal_alertness(u: float) -> float:
    if u >= 0.0:
        return 1.0
    return min(max(1.0 + float(u), 0.0), 1.0)


def eval_haversine_sled_kinematics(t_abs: float, g_level: float, sign_x: float = -1.0):
    """Exact C^2-smooth haversine (sin^2) pulse kinematics (a, v, x) after quiet settle."""
    t_imp = t_abs - T_SETTLE
    a_peak = float(g_level) * G_ACC * float(sign_x)
    t_p = T_PULSE_DUR
    w = 2.0 * math.pi / t_p

    if t_imp <= T_PULSE_START:
        return 0.0, 0.0, 0.0

    if t_imp <= T_PULSE_END:
        tau = t_imp - T_PULSE_START
        a = 0.5 * a_peak * (1.0 - math.cos(w * tau))
        v = 0.5 * a_peak * (tau - math.sin(w * tau) / w)
        x = 0.5 * a_peak * (0.5 * tau * tau + (math.cos(w * tau) - 1.0) / (w * w))
        return a, v, x

    v_end = 0.5 * a_peak * t_p
    x_end_pulse = 0.25 * a_peak * t_p * t_p
    rem = t_imp - T_PULSE_END
    return 0.0, v_end, x_end_pulse + v_end * rem


def build_sled_spline(g_level: float, sign_x: float = -1.0):
    dt_spline = 0.001
    ts = np.arange(0.0, T_TOTAL + 0.02 + 1e-9, dt_spline)
    sp = osim.SimmSpline()
    for t in ts:
        _, _, x = eval_haversine_sled_kinematics(float(t), g_level, sign_x=sign_x)
        sp.addPoint(float(t), float(x))
    _, _, x_end = eval_haversine_sled_kinematics(T_TOTAL, g_level, sign_x=sign_x)
    if sign_x < 0.0:
        if abs(g_level - 2.0) < 1e-6:
            assert abs(x_end + 0.3863) < 0.01, f"2g x_end mismatch: {x_end}"
        elif abs(g_level - 3.0) < 1e-6:
            assert abs(x_end + 0.5794) < 0.01, f"3g x_end mismatch: {x_end}"
    return sp


def run_single_case(spec: dict) -> dict:
    t0 = time.time()
    ensure_libraries_loaded()

    g_level = float(spec["g_level"])
    urgency = float(spec["urgency"])
    engagement = compute_engagement(urgency)
    sign_x = float(spec.get("sign_x", -1.0))

    champ_obj = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))
    init_overrides = dict(champ_obj["params"])
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
        "urgency": urgency,
        "desired_pitch": 0.0,
        "desired_roll": 0.0,
        "desired_yaw": 0.0,
        "desired_ap_shift": 0.0,
        "desired_lat_shift": 0.0,
        "enable_sensor_noise": False,
        "enable_motor_noise": False,
    })
    if "phasic_oto_sat_mps2" in spec and spec["phasic_oto_sat_mps2"] is not None:
        init_overrides["phasic_oto_sat_mps2"] = float(spec["phasic_oto_sat_mps2"])
    if "phasic_torque_sat_nm" in spec and spec["phasic_torque_sat_nm"] is not None:
        init_overrides["phasic_torque_sat_nm"] = float(spec["phasic_torque_sat_nm"])

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

    # Verify urgency property applied cleanly
    u_applied = osim.PropertyHelper.getValueDouble(controller.getPropertyByName("urgency"))

    # Lock eye coordinates to 0
    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    # Prescribe all 6 ground DOFs; drive spine_tx with Zheng sled pulse
    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    sled_spline = build_sled_spline(g_level, sign_x=sign_x)
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
    R_head_to_world = skull.getTransformInGround(state).R()
    q0 = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q0.get(idx))

    for sv in (
        "pitch_error_integral",
        "roll_error_integral",
        "yaw_error_integral",
        "vol_filt_pitch",
        "vol_filt_roll",
        "vol_filt_yaw",
    ):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    k_gdyn = float(init_overrides.get("K_gamma_dyn", 0.857744))
    k_gstat = float(init_overrides.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    n_mus = muscles.getSize()
    neck_muscle_indices = []
    splen_indices = []
    scm_indices = []
    for i in range(n_mus):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        neck_muscle_indices.append(i)
        if mname in SPLEN_CAP_NAMES:
            splen_indices.append(i)
        if mname in SCM_NAMES:
            scm_indices.append(i)
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

    assert len(splen_indices) == 4, f"Expected 4 splenius capitis muscles, got {splen_indices}"
    assert len(scm_indices) == 2, f"Expected 2 SCM muscles, got {scm_indices}"

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
    c_r1, c_r2 = cs.get("roll1"), cs.get("roll2")
    c_y1, c_y2 = cs.get("yaw1"), cs.get("yaw2")
    com_local = skull.getMassCenter()
    nose_vec_local = osim.Vec3(1.0, 0.0, 0.0)

    ts_abs = []
    flex_abs_deg = []
    roll_abs_deg = []
    yaw_abs_deg = []
    flex_vel_dps = []
    nose_y_abs = []
    com_x_rel_t1_abs = []
    splen_cap_act = []
    scm_act = []
    max_neck_act_ts = []
    spine_tx_m = []

    def sample_state(t_val, st):
        model.realizeDynamics(st)
        # In HYOID OpenSim: +pitch is extension (nose UP), -pitch is forward flexion (nose DOWN)
        f_deg = -math.degrees(c_p1.getValue(st) + c_p2.getValue(st))
        r_deg = math.degrees(c_r1.getValue(st) + c_r2.getValue(st))
        y_deg = math.degrees(c_y1.getValue(st) + c_y2.getValue(st))
        vf_dps = -math.degrees(c_p1.getSpeedValue(st) + c_p2.getSpeedValue(st))
        R_hw = skull.getTransformInGround(st).R()
        nose_w = R_hw.multiply(nose_vec_local)
        com_w = skull.findStationLocationInGround(st, com_local)
        tx_val = float(coord_tx.getValue(st))
        sp_vals = [float(muscles.get(idx).getActivation(st)) for idx in splen_indices]
        sc_vals = [float(muscles.get(idx).getActivation(st)) for idx in scm_indices]
        neck_vals = [float(muscles.get(idx).getActivation(st)) for idx in neck_muscle_indices]
        ts_abs.append(float(t_val))
        flex_abs_deg.append(float(f_deg))
        roll_abs_deg.append(float(r_deg))
        yaw_abs_deg.append(float(y_deg))
        flex_vel_dps.append(float(vf_dps))
        nose_y_abs.append(float(nose_w.get(1)))
        com_x_rel_t1_abs.append(float(com_w.get(0) - tx_val))
        splen_cap_act.append(float(sum(sp_vals) / len(sp_vals)))
        scm_act.append(float(sum(sc_vals) / len(sc_vals)))
        max_neck_act_ts.append(float(max(neck_vals)))
        spine_tx_m.append(tx_val)

    sample_state(0.0, state)
    t_now = 0.0
    stop_reason = None
    while t_now < T_TOTAL - 1e-9:
        t_next = round(min(t_now + DT_LOG, T_TOTAL), 6)
        try:
            state = manager.integrate(t_next)
        except Exception as e:
            stop_reason = f"Stopped at t={t_next:.4f}s: {str(e).splitlines()[0]}"
            break
        t_now = t_next
        sample_state(t_now, state)

    ts_arr = np.asarray(ts_abs, dtype=float)
    f_arr = np.asarray(flex_abs_deg, dtype=float)
    vf_arr = np.asarray(flex_vel_dps, dtype=float)
    ny_arr = np.asarray(nose_y_abs, dtype=float)
    cx_arr = np.asarray(com_x_rel_t1_abs, dtype=float)
    splen_arr = np.asarray(splen_cap_act, dtype=float)
    scm_arr = np.asarray(scm_act, dtype=float)
    max_act_arr = np.asarray(max_neck_act_ts, dtype=float)
    tx_arr = np.asarray(spine_tx_m, dtype=float)

    # Impact window relative to t = T_SETTLE
    imp_mask = ts_arr >= (T_SETTLE - 1e-9)
    idx0 = int(np.where(imp_mask)[0][0]) if np.any(imp_mask) else 0
    flex_at_settle = float(f_arr[idx0])
    flex_rel_arr = f_arr - flex_at_settle
    nose_y_rel_arr = ny_arr - float(ny_arr[idx0])
    com_x_rel_arr = cx_arr - float(cx_arr[idx0])

    t_imp_ms = (ts_arr[imp_mask] - T_SETTLE) * 1000.0
    f_imp_rel = flex_rel_arr[imp_mask]
    f_imp_abs = f_arr[imp_mask]
    vf_imp = vf_arr[imp_mask]
    ny_imp = nose_y_rel_arr[imp_mask]
    cx_imp = com_x_rel_arr[imp_mask]
    splen_imp = splen_arr[imp_mask]
    scm_imp = scm_arr[imp_mask]
    max_act_imp = max_act_arr[imp_mask]

    if len(f_imp_rel) > 0:
        idx_max_flex = int(np.argmax(f_imp_rel))
        idx_min_flex = int(np.argmin(f_imp_rel))
        onset_mask = t_imp_ms <= 150.0 + 1e-9
        min_flex_onset_deg = float(np.min(f_imp_rel[onset_mask])) if np.any(onset_mask) else 0.0
        idx_abs_peak = int(np.argmax(np.abs(f_imp_rel)))
        peak_flex_deg = float(f_imp_rel[idx_max_flex])
        min_flex_deg = float(f_imp_rel[idx_min_flex])
        t_peak_ms = float(t_imp_ms[idx_max_flex])
        signed_peak_rel_deg = float(f_imp_rel[idx_abs_peak])
        t_signed_peak_ms = float(t_imp_ms[idx_abs_peak])
        peak_pitch_abs_deg = float(-f_imp_abs[idx_abs_peak])
        peak_vel_dps = float(np.max(np.abs(vf_imp)))
        nose_y_min = float(np.min(ny_imp))
        com_x_max_m = float(np.max(cx_imp))
        splen_env = np.empty_like(splen_imp)
        splen_env[0] = splen_imp[0]
        _alpha_emg = float(DT_LOG) / (0.015 + float(DT_LOG))
        for _k_e in range(1, len(splen_imp)):
            splen_env[_k_e] = splen_env[_k_e - 1] + _alpha_emg * (splen_imp[_k_e] - splen_env[_k_e - 1])
        idx_splen_peak = int(np.argmax(splen_env))
        peak_splen_cap = float(splen_env[idx_splen_peak])
        t_peak_splen_ms = float(t_imp_ms[idx_splen_peak])
        # Prominence-aware peak counter on 15 ms sEMG envelope
        win_half = 25
        n_splen_peaks = 0
        for k in range(1, len(splen_env) - 1):
            if splen_env[k] > 0.08 and splen_env[k] > splen_env[k - 1] and splen_env[k] >= splen_env[k + 1]:
                lo = max(0, k - win_half)
                hi = min(len(splen_env), k + win_half + 1)
                prom = float(splen_env[k] - max(np.min(splen_env[lo : k + 1]), np.min(splen_env[k:hi])))
                if prom >= 0.015:
                    n_splen_peaks += 1
        peak_scm = float(np.max(scm_imp))
        max_neck_act = float(np.max(max_act_imp))
    else:
        peak_flex_deg = min_flex_deg = min_flex_onset_deg = t_peak_ms = signed_peak_rel_deg = t_signed_peak_ms = 0.0
        peak_pitch_abs_deg = peak_vel_dps = nose_y_min = com_x_max_m = peak_splen_cap = t_peak_splen_ms = peak_scm = max_neck_act = 0.0
        n_splen_peaks = 0

    wall_s = time.time() - t0
    return {
        "g_level": g_level,
        "urgency": urgency,
        "urgency_applied": u_applied,
        "engagement": round(engagement, 6),
        "supraspinal_alertness": round(compute_supraspinal_alertness(urgency), 4),
        "spinal_alertness": round(compute_spinal_alertness(urgency), 4),
        "sign_x": sign_x,
        "wall_s": round(wall_s, 2),
        "stop_reason": stop_reason,
        "pitch_at_settle_deg": round(-flex_at_settle, 3),
        "flex_at_settle_deg": round(flex_at_settle, 3),
        "spine_tx_end_m": round(float(tx_arr[-1]), 4),
        "peak_flex_deg": round(peak_flex_deg, 2),
        "min_flex_deg": round(min_flex_deg, 2),
        "min_flex_onset_deg": round(min_flex_onset_deg, 2),
        "signed_peak_rel_deg": round(signed_peak_rel_deg, 2),
        "t_peak_ms": round(t_peak_ms, 1),
        "t_signed_peak_ms": round(t_signed_peak_ms, 1),
        "peak_pitch_abs_deg": round(peak_pitch_abs_deg, 2),
        "peak_vel_dps": round(peak_vel_dps, 1),
        "nose_y_min": round(nose_y_min, 4),
        "com_x_max_m": round(com_x_max_m, 4),
        "peak_splen_cap": round(peak_splen_cap, 4),
        "t_peak_splen_ms": round(t_peak_splen_ms, 1),
        "n_splen_peaks": n_splen_peaks,
        "peak_scm": round(peak_scm, 4),
        "max_neck_act": round(max_neck_act, 4),
        "t_imp_ms": [round(float(v), 2) for v in t_imp_ms],
        "pitch_rel_deg": [round(float(v), 3) for v in f_imp_rel],
        "pitch_abs_deg": [round(float(-v), 3) for v in f_imp_abs],
        "pitch_vel_dps": [round(float(v), 2) for v in vf_imp],
        "splen_cap_act": [round(float(v), 5) for v in splen_imp],
        "scm_act": [round(float(v), 5) for v in scm_imp],
        "spine_tx_imp_m": [round(float(v), 5) for v in tx_arr[imp_mask]],
    }


def make_comparison_plot(results_by_key: dict):
    fig, axes = plt.subplots(2, 2, figsize=(15.0, 9.6), dpi=180, sharex=True)
    color_map = {
        -1.00: "#64748b",
        -0.75: "#ea580c",
        -0.50: "#d97706",
        -0.25: "#0d9488",
         0.00: "#059669",
         0.20: "#2563eb",
         0.50: "#4f46e5",
         1.00: "#7c3aed",
    }

    corridors = {
        2.0: (33.0, 40.0, "NBDL 2g corridor (33–40°)"),
        3.0: (40.0, 47.0, "NBDL 3g corridor (40–47°)"),
    }
    ext_refs = {
        2.0: (0.50, "Zheng Fig. 6 2g splen. cap. peak (~0.50)"),
        3.0: (0.60, "Zheng Fig. 6 3g splen. cap. peak (~0.60)"),
    }

    for col, g_level in enumerate(G_LEVELS):
        ax_ang = axes[0, col]
        ax_act = axes[1, col]

        c_lo, c_hi, c_lbl = corridors[g_level]
        ax_ang.axhspan(c_lo, c_hi, color="#f59e0b", alpha=0.18, label=c_lbl)
        ax_ang.axhline(c_lo, color="#d97706", linestyle="--", linewidth=0.9, alpha=0.8)
        ax_ang.axhline(c_hi, color="#d97706", linestyle="--", linewidth=0.9, alpha=0.8)

        e_ref, e_lbl = ext_refs[g_level]
        ax_act.axhline(e_ref, color="#dc2626", linestyle="--", linewidth=1.2, alpha=0.85, label=e_lbl)

        # Subtle shading of the 175 ms sled acceleration window (37.5 -> 212.5 ms)
        ax_ang.axvspan(37.5, 212.5, color="#cbd5e1", alpha=0.18)
        ax_act.axvspan(37.5, 212.5, color="#cbd5e1", alpha=0.18)

        for idx, u in enumerate(URGENCY_LIST):
            r = results_by_key[(g_level, u)]
            e_val = r["engagement"]
            col_hex = color_map.get(u, "#1f2937")
            lw = 2.6 if u in (-0.75, 0.00, 1.00) else 1.7
            ls = "--" if u == -1.00 else "-"
            tag = " [Passive]" if u == -1.00 else (" [Relaxed NBDL]" if abs(u + 0.75) < 1e-6 else (" [Alert APEO]" if u == 0.0 else (" [Max Braced]" if u == 1.0 else "")))
            lbl = f"u={u:+.2f}{tag} | pk={r['peak_flex_deg']:.1f}°, spl={r['peak_splen_cap']:.2f}"
            ax_ang.plot(r["t_imp_ms"], r["pitch_rel_deg"], color=col_hex, linewidth=lw, linestyle=ls, label=lbl)
            ax_act.plot(r["t_imp_ms"], r["splen_cap_act"], color=col_hex, linewidth=lw, linestyle=ls, label=f"u={u:+.2f} (nPk={r['n_splen_peaks']})")

        ax_ang.set_title(f"{int(g_level)}g Frontal Sled Pulse — Head C.G. Sagittal Rotation vs T1", fontsize=11.5, fontweight="bold")
        ax_ang.set_ylabel("Sagittal Flexion Rel. to Settle (°)", fontsize=10)
        ax_ang.grid(True, linestyle=":", alpha=0.55)
        ax_ang.set_xlim(0.0, 350.0)
        ax_ang.legend(loc="upper left", fontsize=7.2, framealpha=0.92)

        ax_act.set_title(f"{int(g_level)}g Frontal Sled Pulse — Splenius Capitis Activation", fontsize=11.5, fontweight="bold")
        ax_act.set_xlabel("Time Since Impact Onset (ms) [after 0.35 s quiet pre-settle]", fontsize=10)
        ax_act.set_ylabel("Mean Splenius Capitis Activation", fontsize=10)
        ax_act.grid(True, linestyle=":", alpha=0.55)
        ax_act.set_xlim(0.0, 350.0)
        ax_act.set_ylim(0.0, max(0.75, max(results_by_key[(g_level, u)]["peak_splen_cap"] for u in URGENCY_LIST) + 0.08))
        ax_act.legend(loc="upper right", fontsize=7.6, framealpha=0.92)

    fig.suptitle(
        "Unified Alertness/Urgency u ∈ [-1, +1] — Zheng et al. (2021) / NBDL 2g & 3g Sled (C^2 Haversine, High-G Reweighted)",
        fontsize=12.0,
        fontweight="bold",
        y=0.985,
    )
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, dpi=180)
    plt.close(fig)
    if os.path.isdir(ART_DIR):
        shutil.copy2(OUT_PNG, os.path.join(ART_DIR, os.path.basename(OUT_PNG)))


def main():
    t_wall0 = time.time()
    specs = [{"g_level": g, "urgency": u, "sign_x": -1.0} for g in G_LEVELS for u in URGENCY_LIST]
    print(f"Running {len(specs)} simulations across 6 local performance cores...", flush=True)

    results = []
    with ProcessPoolExecutor(max_workers=6) as pool:
        for res in pool.map(run_single_case, specs):
            results.append(res)
            print(
                f"[DONE] {int(res['g_level'])}g | u={res['urgency']:.2f} (e={res['engagement']:.3f}) | "
                f"pk_flex={res['peak_flex_deg']:+6.2f}° (min={res['min_flex_deg']:+6.2f}°) @ {res['t_peak_ms']:5.1f}ms | "
                f"Vmax={res['peak_vel_dps']:5.1f}°/s | splen={res['peak_splen_cap']:.3f} @ {res['t_peak_splen_ms']:5.1f}ms | "
                f"tx_end={res['spine_tx_end_m']:+.4f}m | settle={res['pitch_at_settle_deg']:+.2f}° | "
                f"{res['wall_s']:.1f}s | stop={res['stop_reason']}",
                flush=True,
            )

    results_by_key = {(r["g_level"], r["urgency"]): r for r in results}

    # Check if forward flexion went negative (unexpected sign convention)
    u0_2g = results_by_key[(2.0, 0.0)]
    print(
        f"\nSign Check (2g, u=0): peak_flex={u0_2g['peak_flex_deg']:+.2f}°, "
        f"min_flex={u0_2g['min_flex_deg']:+.2f}°, signed_peak={u0_2g['signed_peak_rel_deg']:+.2f}°",
        flush=True,
    )

    make_comparison_plot(results_by_key)

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    summary_rows = [
        {k: v for k, v in r.items() if k not in ("t_imp_ms", "pitch_rel_deg", "pitch_abs_deg", "pitch_vel_dps", "splen_cap_act", "scm_act", "spine_tx_imp_m")}
        for r in results
    ]
    payload = {
        "meta": {
            "controller": "C11 + e(u)",
            "pulse_shape": "C2_haversine",
            "t_settle_s": T_SETTLE,
            "t_impact_s": T_IMPACT,
            "t_total_s": T_TOTAL,
            "pulse_ms": [37.5, 125.0, 212.5],
            "g_levels": G_LEVELS,
            "urgency_list": URGENCY_LIST,
            "total_wall_s": round(time.time() - t_wall0, 2),
        },
        "summary": summary_rows,
        "cases": results,
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print("\n=== SUMMARY TABLE: Unified Alertness u in [-1, +1] vs. Zheng et al. (2021) / NBDL 2g & 3g ===")
    hdr = f"{'G':>3} | {'u':>6} | {'s_sup':>5} | {'e(u)':>5} | {'Settle':>7} | {'PkFlex':>7} | {'MinOns':>7} | {'MinFlex':>8} | {'t_pk':>7} | {'Vmax':>8} | {'SplenPk':>7} | {'nPk':>3} | {'SCM_Pk':>7} | {'MaxAct':>7}"
    print(hdr)
    print("-" * len(hdr))
    for g in G_LEVELS:
        for u in URGENCY_LIST:
            r = results_by_key[(g, u)]
            print(
                f"{int(g):>2}g | {u:+6.2f} | {r['supraspinal_alertness']:5.3f} | {r['engagement']:5.3f} | {r['pitch_at_settle_deg']:+6.2f}° | "
                f"{r['peak_flex_deg']:+6.2f}° | {r['min_flex_onset_deg']:+6.2f}° | {r['min_flex_deg']:+7.2f}° | {r['t_peak_ms']:5.1f}ms | "
                f"{r['peak_vel_dps']:6.1f}°/s | {r['peak_splen_cap']:7.4f} | {r['n_splen_peaks']:3d} | "
                f"{r['peak_scm']:7.4f} | {r['max_neck_act']:7.4f}"
            )
    print(f"\nSaved JSON: {OUT_JSON}")
    print(f"Saved PNG : {OUT_PNG}")
    print(f"Total wall time: {time.time() - t_wall0:.2f} s")


if __name__ == "__main__":
    main()
