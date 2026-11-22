#!/usr/bin/env python3
"""Step 3: 7-Case 2x2 Weighted IK & SE2(3) C1 Counter-Translation Validation Suite."""

import argparse
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

import matplotlib.pyplot as plt
import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0
from scripts.generate_standard_plot import MODEL_PATH, ART_DIR, apply_cli_overrides_and_inspect

CHAMP_C1_JSON = os.path.join(_ROOT, "logs", "se2_c1pos_3axis_champion.json")
CHAMP_FALLBACK_JSON = os.path.join(_ROOT, "logs", "se2_3axis_champion_final.json")
OUT_JSON = os.path.join(_ROOT, "logs", "validation_5dof_voluntary_shift.json")
OUT_PNG = os.path.join(_ROOT, "results", "validation_5dof_voluntary_shift.png")


def run_validation_case(spec):
    ensure_libraries_loaded()
    name = spec["name"]
    champ_params = spec["base_params"]
    init_overrides = dict(champ_params)
    init_overrides.update(spec.get("init_overrides", {}))
    step_overrides = spec.get("step_overrides", {})
    step_time = float(spec.get("step_time", 0.5))
    duration = float(spec.get("duration", 6.0))
    sway_amp_m = float(spec.get("sway_amp_m", 0.0))
    sway_freq_hz = float(spec.get("sway_freq_hz", 0.5))
    ss_win = float(spec.get("ss_win", 1.0))

    model = osim.Model(MODEL_PATH)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl
    assert controller is not None, "HeadNeckNeuralController not found"

    ov_list = [(k, f"{float(v):.15g}") for k, v in init_overrides.items()]
    bool_state, false_booleans, _ = apply_cli_overrides_and_inspect(controller, ov_list)
    assert not false_booleans, f"{name} false booleans: {false_booleans}"

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

    if sway_amp_m > 0.0:
        dt_s = 0.002
        ts_grid = np.arange(0.0, duration + 2.0 * dt_s, dt_s)
        vals = []
        omega = 2.0 * math.pi * sway_freq_hz
        for t in ts_grid:
            if t <= 0.3:
                ramp = 0.0
            elif t < 1.1:
                u = (t - 0.3) / 0.8
                ramp = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
            else:
                ramp = 1.0
            vals.append(sway_amp_m * ramp * math.sin(omega * max(0.0, t - 0.3)))
        sp = osim.SimmSpline()
        for t, v in zip(ts_grid, vals):
            sp.addPoint(float(t), float(v))
        coord_tx = model.updCoordinateSet().get("spine_tx")
        coord_tx.setPrescribedFunction(sp)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    if sway_amp_m > 0.0:
        model.getCoordinateSet().get("spine_tx").setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    com_local = skull.getMassCenter()
    c1_local = osim.Vec3(0.0, 0.0, 0.0)
    com0 = skull.findStationLocationInGround(state, com_local)
    c1_0 = skull.findStationLocationInGround(state, c1_local)
    com0_arr = np.array([com0.get(0), com0.get(1), com0.get(2)], dtype=float)
    c1_0_arr = np.array([c1_0.get(0), c1_0.get(1), c1_0.get(2)], dtype=float)

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

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(mname, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gdyn_eff * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gstat_eff * a0
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

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")
    coord_tx = model.getCoordinateSet().get("spine_tx")
    coord_tz = model.getCoordinateSet().get("spine_tz")

    out_cun_ap = controller.getOutput("cuneate_ap_shift")
    out_cun_lat = controller.getOutput("cuneate_lat_shift")
    out_obs_px = controller.getOutput("obs_pos_x")

    step = 0.005
    t_now = 0.0
    step_applied = False
    t_arr, p1_arr, p2_arr, r1_arr, r2_arr, y1_arr, y2_arr = [], [], [], [], [], [], []
    dx_c1_rel_cm, dz_c1_rel_cm, dx_c1_glob_cm = [], [], []
    dx_com_rel_cm, dz_com_rel_cm, dx_com_glob_cm = [], [], []
    trunk_x_cm, cun_ap_cm, cun_lat_cm, obs_px_cm = [], [], [], []
    aborted = False

    while t_now < duration:
        t_now = min(t_now + step, duration)
        if not step_applied and step_overrides and t_now >= step_time - 1e-9:
            for k, v in step_overrides.items():
                prop = controller.updPropertyByName(k)
                osim.PropertyHelper.setValueDouble(float(v), prop)
            step_applied = True
        try:
            state = manager.integrate(t_now)
            model.realizeVelocity(state)
        except Exception as e:
            print(f"[WARN] {spec['name']} stopped at t={t_now:.3f}s: {str(e).splitlines()[0]}", flush=True)
            break

        p1_d = math.degrees(coord_p1.getValue(state))
        p2_d = math.degrees(coord_p2.getValue(state))
        r1_d = math.degrees(coord_r1.getValue(state))
        r2_d = math.degrees(coord_r2.getValue(state))
        y1_d = math.degrees(coord_y1.getValue(state))
        y2_d = math.degrees(coord_y2.getValue(state))

        c1_pos = skull.findStationLocationInGround(state, c1_local)
        com_pos = skull.findStationLocationInGround(state, com_local)
        tx = float(coord_tx.getValue(state))
        tz = float(coord_tz.getValue(state))

        dx_c1_g = (float(c1_pos.get(0)) - c1_0_arr[0]) * 100.0
        dz_c1_g = (float(c1_pos.get(2)) - c1_0_arr[2]) * 100.0
        dx_c1_r = dx_c1_g - tx * 100.0
        dz_c1_r = dz_c1_g - tz * 100.0

        dx_cm_g = (float(com_pos.get(0)) - com0_arr[0]) * 100.0
        dz_cm_g = (float(com_pos.get(2)) - com0_arr[2]) * 100.0
        dx_cm_r = dx_cm_g - tx * 100.0
        dz_cm_r = dz_cm_g - tz * 100.0

        c_ap = float(out_cun_ap.getValueAsString(state)) * 100.0
        c_lat = float(out_cun_lat.getValueAsString(state)) * 100.0
        o_px = float(out_obs_px.getValueAsString(state)) * 100.0

        t_arr.append(t_now)
        p1_arr.append(p1_d); p2_arr.append(p2_d)
        r1_arr.append(r1_d); r2_arr.append(r2_d)
        y1_arr.append(y1_d); y2_arr.append(y2_d)
        dx_c1_glob_cm.append(dx_c1_g); dx_c1_rel_cm.append(dx_c1_r); dz_c1_rel_cm.append(dz_c1_r)
        dx_com_glob_cm.append(dx_cm_g); dx_com_rel_cm.append(dx_cm_r); dz_com_rel_cm.append(dz_cm_r)
        trunk_x_cm.append(tx * 100.0)
        cun_ap_cm.append(c_ap)
        cun_lat_cm.append(c_lat)
        obs_px_cm.append(o_px)

        if abs(p1_d + p2_d) > 65.0 or abs(r1_d + r2_d) > 40.0 or abs(y1_d + y2_d) > 65.0:
            aborted = True
            break

    t_np = np.array(t_arr)
    pitch_tot = np.array(p1_arr) + np.array(p2_arr)
    roll_tot = np.array(r1_arr) + np.array(r2_arr)
    yaw_tot = np.array(y1_arr) + np.array(y2_arr)
    c1_dx_r_np = np.array(dx_c1_rel_cm)
    c1_dz_r_np = np.array(dz_c1_rel_cm)
    c1_dx_g_np = np.array(dx_c1_glob_cm)
    com_dx_g_np = np.array(dx_com_glob_cm)

    m_quiet = t_np < step_time - 1e-6 if step_time > 0.05 else (t_np <= 0.05)
    m_ss = t_np >= (duration - ss_win)
    m_sway_ss = t_np >= 4.0

    summary = {
        "name": name,
        "label": spec["label"],
        "aborted": aborted,
        "false_booleans": false_booleans,
        "quiet_max_att_deg": round(float(max(np.max(np.abs(pitch_tot[m_quiet])), np.max(np.abs(roll_tot[m_quiet])), np.max(np.abs(yaw_tot[m_quiet])))), 4) if np.any(m_quiet) else 0.0,
        "quiet_max_c1_cm": round(float(max(np.max(np.abs(c1_dx_r_np[m_quiet])), np.max(np.abs(c1_dz_r_np[m_quiet])))), 4) if np.any(m_quiet) else 0.0,
        "ss_c1_dx_rel_cm": round(float(np.mean(c1_dx_r_np[m_ss])), 3),
        "ss_c1_dz_rel_cm": round(float(np.mean(c1_dz_r_np[m_ss])), 3),
        "ss_com_dx_rel_cm": round(float(np.mean(np.array(dx_com_rel_cm)[m_ss])), 3),
        "ss_com_dz_rel_cm": round(float(np.mean(np.array(dz_com_rel_cm)[m_ss])), 3),
        "ss_cun_ap_cm": round(float(np.mean(np.array(cun_ap_cm)[m_ss])), 3),
        "ss_cun_lat_cm": round(float(np.mean(np.array(cun_lat_cm)[m_ss])), 3),
        "ss_pitch_deg": round(float(np.mean(pitch_tot[m_ss])), 3),
        "ss_roll_deg": round(float(np.mean(roll_tot[m_ss])), 3),
        "ss_yaw_deg": round(float(np.mean(yaw_tot[m_ss])), 3),
        "ss_pitch1_deg": round(float(np.mean(np.array(p1_arr)[m_ss])), 2),
        "ss_pitch2_deg": round(float(np.mean(np.array(p2_arr)[m_ss])), 2),
        "ss_roll1_deg": round(float(np.mean(np.array(r1_arr)[m_ss])), 2),
        "ss_roll2_deg": round(float(np.mean(np.array(r2_arr)[m_ss])), 2),
        "max_abs_pitch_deg": round(float(np.max(np.abs(pitch_tot))), 3),
        "max_abs_roll_deg": round(float(np.max(np.abs(roll_tot))), 3),
        "max_abs_yaw_deg": round(float(np.max(np.abs(yaw_tot))), 3),
        "max_abs_roll1_deg": round(float(np.max(np.abs(r1_arr))), 3),
        "sway_rms_c1_glob_cm": round(float(np.sqrt(np.mean(c1_dx_g_np[m_sway_ss] ** 2))), 3),
        "sway_rms_com_glob_cm": round(float(np.sqrt(np.mean(com_dx_g_np[m_sway_ss] ** 2))), 3),
        "sway_rms_c1_rel_cm": round(float(np.sqrt(np.mean(c1_dx_r_np[m_sway_ss] ** 2))), 3),
        "sway_mean_c1_rel_cm": round(float(np.mean(c1_dx_r_np[m_sway_ss])), 3),
        "sway_rms_pitch_deg": round(float(np.sqrt(np.mean(pitch_tot[m_sway_ss] ** 2))), 3),
        "ss_std_cun_ap_cm": round(float(np.std(np.array(cun_ap_cm)[m_ss])), 4) if np.any(m_ss) else 0.0,
        "ss_std_cun_lat_cm": round(float(np.std(np.array(cun_lat_cm)[m_ss])), 4) if np.any(m_ss) else 0.0,
        "ss_std_pitch_deg": round(float(np.std(pitch_tot[m_ss])), 4) if np.any(m_ss) else 0.0,
    }

    traces = {
        "t": [round(float(x), 4) for x in t_arr],
        "c1_dx_rel_cm": [round(float(x), 4) for x in dx_c1_rel_cm],
        "c1_dz_rel_cm": [round(float(x), 4) for x in dz_c1_rel_cm],
        "c1_dx_glob_cm": [round(float(x), 4) for x in dx_c1_glob_cm],
        "com_dx_rel_cm": [round(float(x), 4) for x in dx_com_rel_cm],
        "com_dx_glob_cm": [round(float(x), 4) for x in dx_com_glob_cm],
        "trunk_x_cm": [round(float(x), 4) for x in trunk_x_cm],
        "cun_ap_cm": [round(float(x), 4) for x in cun_ap_cm],
        "cun_lat_cm": [round(float(x), 4) for x in cun_lat_cm],
        "obs_px_cm": [round(float(x), 4) for x in obs_px_cm],
        "pitch_deg": [round(float(x), 4) for x in pitch_tot],
        "roll_deg": [round(float(x), 4) for x in roll_tot],
        "yaw_deg": [round(float(x), 4) for x in yaw_tot],
        "pitch1_deg": [round(float(x), 4) for x in p1_arr],
        "pitch2_deg": [round(float(x), 4) for x in p2_arr],
        "roll1_deg": [round(float(x), 4) for x in r1_arr],
        "roll2_deg": [round(float(x), 4) for x in r2_arr],
    }
    return summary, traces


def smooth_step_ref(t_arr, target_val, step_t=0.5, tau=0.28):
    """Exact 3rd-order critically damped (jerk-limited, right-skewed bell) trajectory planner response to a step at step_t."""
    t_np = np.asarray(t_arr, dtype=float)
    dt = np.maximum(t_np - step_t, 0.0)
    active = (t_np >= step_t).astype(float)
    tau3 = 0.7535 * float(tau)
    u = dt / tau3
    return active * float(target_val) * (1.0 - (1.0 + u + 0.5 * u * u) * np.exp(-u))


def render_validation_figure(records):
    by_name = {r["summary"]["name"]: r for r in records}
    fig, axes = plt.subplots(2, 3, figsize=(16.8, 9.4), dpi=160)
    fig.suptitle(
        "5-DOF Cortical Control: Closed-Form 2x2 Weighted IK (C1/Attitude) & SE2(3) Counter-Translation\n"
        "Booleans: ALL 28 TRUE | 72-Muscle HYOID Model | Closed-Loop 6-DOF Cuneate & Washout SE2(3)",
        fontsize=12.5,
        fontweight="bold",
        y=0.98,
    )

    # Panel A: Case 1 Protraction (+2.0 cm C1)
    ax = axes[0, 0]
    tr, sm = by_name["Case1_AP_Protraction"]["traces"], by_name["Case1_AP_Protraction"]["summary"]
    t = np.array(tr["t"])
    ax.plot(t, tr["c1_dx_rel_cm"], "#059669", lw=2.2, label=f"True C1 dx ({sm['ss_c1_dx_rel_cm']:+.2f} cm)")
    ax.plot(t, tr["cun_ap_cm"], "#0284c7", ls="--", lw=1.7, label=f"Cuneate Sensor dx ({sm['ss_cun_ap_cm']:+.2f} cm)")
    ax.plot(t, smooth_step_ref(t, 2.0), "#0f172a", ls=":", lw=1.6, label="Smooth Ref (+2.00 cm, tau=0.28s)")
    ax.axvline(0.5, color="#dc2626", ls="--", lw=1.1, alpha=0.7)
    ax.set_title(f"Case 1: +2.0 cm C1 Protraction (Gaze Pitch={sm['ss_pitch_deg']:+.2f} deg)", fontweight="bold", fontsize=10.2)
    ax.set_ylabel("C1 AP Shift [cm]", fontweight="bold")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc="lower right")

    # Panel B: Case 2 Retraction (-1.5 cm C1)
    ax = axes[0, 1]
    tr, sm = by_name["Case2_AP_Retraction"]["traces"], by_name["Case2_AP_Retraction"]["summary"]
    t = np.array(tr["t"])
    ax.plot(t, tr["c1_dx_rel_cm"], "#059669", lw=2.2, label=f"True C1 dx ({sm['ss_c1_dx_rel_cm']:+.2f} cm)")
    ax.plot(t, tr["cun_ap_cm"], "#0284c7", ls="--", lw=1.7, label=f"Cuneate Sensor dx ({sm['ss_cun_ap_cm']:+.2f} cm)")
    ax.plot(t, smooth_step_ref(t, -1.5), "#0f172a", ls=":", lw=1.6, label="Smooth Ref (-1.50 cm, tau=0.28s)")
    ax.axvline(0.5, color="#dc2626", ls="--", lw=1.1, alpha=0.7)
    ax.set_title(f"Case 2: -1.5 cm C1 Retraction (Gaze Pitch={sm['ss_pitch_deg']:+.2f} deg)", fontweight="bold", fontsize=10.2)
    ax.set_ylabel("C1 AP Shift [cm]", fontweight="bold")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc="upper right")

    # Panel C: Case 3 Lateral Glide (+0.5 cm C1)
    ax = axes[0, 2]
    tr, sm = by_name["Case3_Lat_Glide"]["traces"], by_name["Case3_Lat_Glide"]["summary"]
    t = np.array(tr["t"])
    ax.plot(t, tr["c1_dz_rel_cm"], "#2563eb", lw=2.2, label=f"True C1 dz ({sm['ss_c1_dz_rel_cm']:+.2f} cm)")
    ax.plot(t, tr["cun_lat_cm"], "#7c3aed", ls="--", lw=1.7, label=f"Cuneate Sensor dz ({sm['ss_cun_lat_cm']:+.2f} cm)")
    ax.plot(t, smooth_step_ref(t, 0.5), "#0f172a", ls=":", lw=1.6, label="Smooth Ref (+0.50 cm, tau=0.28s)")
    ax.axvline(0.5, color="#dc2626", ls="--", lw=1.1, alpha=0.7)
    ax.set_title(f"Case 3: +0.5 cm C1 Lat Glide (roll1={sm['ss_roll1_deg']:+.1f}, Roll={sm['ss_roll_deg']:+.2f} deg)", fontweight="bold", fontsize=10.0)
    ax.set_ylabel("C1 Lateral Shift [cm]", fontweight="bold")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc="lower right")

    # Panel D: Cases 1-3 Head Attitude Crosstalk
    ax = axes[1, 0]
    for cname, col, lbl in [
        ("Case1_AP_Protraction", "#059669", f"C1 Pitch (ss={by_name['Case1_AP_Protraction']['summary']['ss_pitch_deg']:+.2f} deg)"),
        ("Case2_AP_Retraction", "#d97706", f"C2 Pitch (ss={by_name['Case2_AP_Retraction']['summary']['ss_pitch_deg']:+.2f} deg)"),
        ("Case3_Lat_Glide", "#2563eb", f"C3 Roll  (ss={by_name['Case3_Lat_Glide']['summary']['ss_roll_deg']:+.2f} deg)"),
    ]:
        tr = by_name[cname]["traces"]
        vals = tr["roll_deg"] if "Lat" in cname else tr["pitch_deg"]
        ax.plot(tr["t"], vals, color=col, lw=1.9, label=lbl)
    ax.axhline(0.0, color="#64748b", ls=":", lw=1.0)
    ax.axvline(0.5, color="#dc2626", ls="--", lw=1.1, alpha=0.7)
    ax.set_title("Head Attitude Invariance During Pure C1 Shifts", fontweight="bold", fontsize=10.2)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Head Attitude Error [deg]", fontweight="bold")
    ax.set_ylim(-1.2, 1.2)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8.0, loc="upper right")

    # Panel E: Case 4A Combined Pitch + C1 Shift (2x2 Task-Priority IK, w_pos=1)
    ax = axes[1, 1]
    tr4a, sm4a = by_name["Case4A_Combined_wPos1"]["traces"], by_name["Case4A_Combined_wPos1"]["summary"]
    t4a = np.array(tr4a["t"])
    ax_r = ax.twinx()
    ax.plot(t4a, tr4a["pitch_deg"], "#059669", lw=2.2, label=f"Pitch ({sm4a['ss_pitch_deg']:+.2f} deg)")
    ax.plot(t4a, smooth_step_ref(t4a, 10.0), "#065f46", ls=":", lw=1.4, label="Smooth Pitch Ref (+10 deg)")
    ax_r.plot(t4a, tr4a["c1_dx_rel_cm"], "#7c3aed", lw=2.1, label=f"C1 dx ({sm4a['ss_c1_dx_rel_cm']:+.2f} cm)")
    ax_r.plot(t4a, smooth_step_ref(t4a, 0.8), "#5b21b6", ls=":", lw=1.4, label="Smooth C1 dx Ref (+0.8 cm)")
    ax.axvline(0.5, color="#dc2626", ls="--", lw=1.1, alpha=0.7)
    ax.set_title("Case 4: Combined +10 deg Pitch & +0.8 cm C1 Shift", fontweight="bold", fontsize=10.2)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Head Pitch [deg]", color="#059669", fontweight="bold")
    ax_r.set_ylabel("C1 AP Shift [cm]", color="#7c3aed", fontweight="bold")
    l1, lb1 = ax.get_legend_handles_labels(); l2, lb2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=7.4, loc="lower right")
    ax.grid(True, alpha=0.3)

    # Panel F: Case 5 SE2(3) Counter-Translation under 0.5 Hz Trunk Sway
    ax = axes[1, 2]
    tr_on, sm_on = by_name["Case5B_Sway_SE2On"]["traces"], by_name["Case5B_Sway_SE2On"]["summary"]
    ax.plot(tr_on["t"], tr_on["trunk_x_cm"], "#94a3b8", ls=":", lw=1.5, label="Trunk Sway (1.5 cm)")
    ax.plot(tr_on["t"], tr_on["c1_dx_glob_cm"], "#2563eb", lw=2.1, label=f"Global C1 x SE2 On  (RMS={sm_on['sway_rms_c1_glob_cm']:.2f} cm)")
    ax.plot(tr_on["t"], tr_on["c1_dx_rel_cm"], "#059669", ls="--", lw=1.4, label=f"Counter-Shift C1 (mean={sm_on['sway_mean_c1_rel_cm']:+.2f} cm)")
    ax.set_title("Case 5: SE2(3) C1 Counter-Translation (0.5 Hz)", fontweight="bold", fontsize=10.2)
    ax.set_xlabel("Time [s]", fontweight="bold")
    ax.set_ylabel("Global / C1 AP Displacement [cm]", fontweight="bold")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=7.4, loc="upper right")

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, bbox_inches="tight")
    os.makedirs(ART_DIR, exist_ok=True)
    shutil.copyfile(OUT_PNG, os.path.join(ART_DIR, "validation_5dof_voluntary_shift.png"))
    plt.close(fig)
    print(f"[OK] Saved validation plot: {OUT_PNG}", flush=True)


def build_validation_specs(champ_params, kp_px, kv_vx, ki_ap_sway, pid_ov=None):
    """Build the 7 canonical 5-DOF voluntary IK & SE2(3) counter-translation validation case specs."""
    no_se2_ov = dict(
        pid_ov or {},
        Kp_trans_roll=0.0,
        Kp_trans_pitch=0.0,
        Kp_shift_obs_px=0.0,
        Kv_shift_obs_vx=0.0,
        Kp_shift_obs_pz=0.0,
        Kv_shift_obs_vz=0.0,
        shift_station_blend=0.0,
        trajectory_tau=0.28,
    )
    return [
        {
            "name": "Case1_AP_Protraction",
            "label": "+2.0 cm C1 Protraction (0 deg Gaze)",
            "base_params": champ_params,
            "init_overrides": dict(no_se2_ov, w_pos_ap=1.0),
            "step_overrides": {"desired_ap_shift": 0.020},
            "step_time": 0.5,
            "duration": 6.0,
        },
        {
            "name": "Case2_AP_Retraction",
            "label": "-1.5 cm C1 Retraction (0 deg Gaze)",
            "base_params": champ_params,
            "init_overrides": dict(no_se2_ov, w_pos_ap=1.0),
            "step_overrides": {"desired_ap_shift": -0.015},
            "step_time": 0.5,
            "duration": 6.0,
        },
        {
            "name": "Case3_Lat_Glide",
            "label": "+0.5 cm C1 Lateral Glide (0 deg Gaze)",
            "base_params": champ_params,
            "init_overrides": dict(no_se2_ov, w_pos_lat=1.0),
            "step_overrides": {"desired_lat_shift": 0.005},
            "step_time": 0.5,
            "duration": 6.0,
        },
        {
            "name": "Case4A_Combined_wPos1",
            "label": "+10 deg Pitch + +0.8 cm C1 Protraction (w_pos_ap=1.0)",
            "base_params": champ_params,
            "init_overrides": dict(no_se2_ov, w_pos_ap=1.0),
            "step_overrides": {"desired_pitch": math.radians(10.0), "desired_ap_shift": 0.008},
            "step_time": 0.5,
            "duration": 6.0,
        },
        {
            "name": "Case4B_AttitudeOnly_wPos0",
            "label": "+10 deg Pitch + +0.8 cm cmd Ignored (w_pos_ap=0.0)",
            "base_params": champ_params,
            "init_overrides": dict(no_se2_ov, w_pos_ap=0.0),
            "step_overrides": {"desired_pitch": math.radians(10.0), "desired_ap_shift": 0.008},
            "step_time": 0.5,
            "duration": 6.0,
        },
        {
            "name": "Case5A_Sway_SE2Off",
            "label": "0.5 Hz Trunk AP Sway (w_pos_ap=0.0, SE2 Off)",
            "base_params": champ_params,
            "init_overrides": dict(no_se2_ov, w_pos_ap=0.0),
            "step_overrides": {},
            "step_time": 0.3,
            "duration": 8.0,
            "sway_amp_m": 0.015,
            "sway_freq_hz": 0.5,
        },
        {
            "name": "Case5B_Sway_SE2On",
            "label": f"0.5 Hz Trunk AP Sway (Kp_px={kp_px}, Kv_vx={kv_vx})",
            "base_params": champ_params,
            "init_overrides": dict(
                no_se2_ov,
                w_pos_ap=1.0,
                Kp_shift_obs_px=kp_px,
                Kv_shift_obs_vx=kv_vx,
                Ki_shift_ap=ki_ap_sway,
                tau_obs_washout=2.2,
            ),
            "step_overrides": {},
            "step_time": 0.3,
            "duration": 8.0,
            "sway_amp_m": 0.015,
            "sway_freq_hz": 0.5,
        },
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kp-shift", type=float, default=None)
    parser.add_argument("--kd-shift", type=float, default=None)
    parser.add_argument("--ki-shift", type=float, default=None)
    parser.add_argument("--kp-obs-px", type=float, default=0.0)
    parser.add_argument("--kv-obs-vx", type=float, default=-0.16)
    args = parser.parse_args()

    t0 = time.time()
    src_json = CHAMP_C1_JSON if os.path.exists(CHAMP_C1_JSON) else CHAMP_FALLBACK_JSON
    print(f"[INFO] Loading base params from: {src_json}", flush=True)
    src_obj = json.load(open(src_json, "r", encoding="utf-8"))
    champ_params = src_obj["params"]
    sw_cfg = src_obj.get("best_sway_ap_config", {})
    use_kp_px = float(sw_cfg.get("Kp_shift_obs_px", args.kp_obs_px))
    use_kv_vx = float(sw_cfg.get("Kv_shift_obs_vx", args.kv_obs_vx))
    use_ki_ap = float(sw_cfg.get("Ki_shift_ap", 0.0))
    print(f"[INFO] Case5B SE2(3) counter-shift gains: Kp_px={use_kp_px}, Kv_vx={use_kv_vx}, Ki_ap={use_ki_ap}", flush=True)
    pid_ov = {}
    if args.kp_shift is not None:
        pid_ov["Kp_shift_ap"] = args.kp_shift
        pid_ov["Kp_shift_lat"] = args.kp_shift
    if args.kd_shift is not None:
        pid_ov["Kd_shift_ap"] = args.kd_shift
        pid_ov["Kd_shift_lat"] = args.kd_shift
    if args.ki_shift is not None:
        pid_ov["Ki_shift_ap"] = args.ki_shift
        pid_ov["Ki_shift_lat"] = args.ki_shift

    specs = build_validation_specs(champ_params, use_kp_px, use_kv_vx, use_ki_ap, pid_ov=pid_ov)

    records = [None] * len(specs)
    with ProcessPoolExecutor(max_workers=len(specs)) as ex:
        futs = [(i, ex.submit(run_validation_case, sp)) for i, sp in enumerate(specs)]
        for i, fut in futs:
            sm, tr = fut.result()
            records[i] = {"summary": sm, "traces": tr}
            print(
                f"  [{sm['name']:26s}] C1_dx={sm['ss_c1_dx_rel_cm']:+6.2f} (COM={sm['ss_com_dx_rel_cm']:+6.2f}, cun={sm['ss_cun_ap_cm']:+6.2f}) | "
                f"C1_dz={sm['ss_c1_dz_rel_cm']:+6.2f} (cun={sm['ss_cun_lat_cm']:+6.2f}) | "
                f"att=[p={sm['ss_pitch_deg']:+5.2f}, r={sm['ss_roll_deg']:+5.2f}, y={sm['ss_yaw_deg']:+5.2f}] | "
                f"p1/p2=[{sm['ss_pitch1_deg']:+5.1f}/{sm['ss_pitch2_deg']:+5.1f}] r1max={sm['max_abs_roll1_deg']:4.2f} | "
                f"sway_C1_glob={sm['sway_rms_c1_glob_cm']:.2f}cm | Q={sm['quiet_max_att_deg']:.3f}",
                flush=True,
            )

    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"elapsed_s": round(time.time() - t0, 1), "cases": [r["summary"] for r in records]}, f, indent=2)
    render_validation_figure(records)
    print(f"[OK] Saved validation JSON: {OUT_JSON} in {time.time() - t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
