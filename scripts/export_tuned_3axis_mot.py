"""Record and export OpenSim .mot trajectories for the unified ungated 3-axis reflex controller.

Generates .mot files (radians/meters, degrees/meters, and degrees + 72 neck muscle activations)
for APEO (Pitch, spine_tx), LatEC (Roll, spine_tz), and YawMA (Yaw, gndyaw) using
logs/roll_yaw_baseline/tuned_3axis_params.json, and verifies exact Classic Bode Scores.
"""

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import CANONICAL_TARGETS, PER_MUSCLE_72_U0, ensure_libraries_loaded
from run_roll_yaw_baseline import (
    build_c2_multisine_spline,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    load_aligned_data,
)
from tune_roll_and_yaw import build_c2_zero_offset_spline_yaw

GROUND_TRUNK_COORDS = ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]
EYE_COORDS = ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]

PER_AXIS_PROP_KEYS = [
    "K_ton_roll", "K_ton_yaw", "p_ton_roll", "p_ton_yaw",
    "K_sc_roll", "K_sc_yaw", "z_sc_roll", "z_sc_yaw", "p_sc_roll", "p_sc_yaw",
    "K_phas_roll", "z_phas_roll", "p_phas_roll", "p_ap_roll", "c_ap_roll",
    "Kp_prop_roll", "Kp_prop_yaw", "z_ccr_roll", "z_ccr_yaw", "p_ccr_roll", "p_ccr_yaw",
    "deriv_comp_roll", "deriv_comp_yaw",
]


def _build_apeo_spline(freqs, amps, phases, delay, ramp_duration, target_time):
    """Build exact Solution-A C2 acceleration-windowed multisine spline matching cmaes_worker_multisine.py."""
    ms_spline = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        ms_spline.addPoint(float(t), 0.0)

    spline_dt = 0.0025
    sub = 10
    dt_fine = spline_dt / sub
    n_fine = int(round(ramp_duration / dt_fine))
    t_fine = np.linspace(0.0, ramp_duration, n_fine + 1)
    tau_fine = t_fine / ramp_duration
    w_fine = 10.0 * (tau_fine ** 3) - 15.0 * (tau_fine ** 4) + 6.0 * (tau_fine ** 5)
    phi_v = (tau_fine ** 2) * ((1.0 - tau_fine) ** 2)
    phi_v /= np.sum(0.5 * (phi_v[:-1] + phi_v[1:]) * dt_fine)

    omegas = 2.0 * np.pi * freqs
    psi_fine = np.outer(t_fine, omegas) + phases
    y_fine = np.sin(psi_fine) @ amps
    dy_fine = np.cos(psi_fine) @ (amps * omegas)
    ddy_fine = -np.sin(psi_fine) @ (amps * (omegas ** 2))

    ddx_raw = w_fine * ddy_fine
    v_raw_end = float(np.sum(0.5 * (ddx_raw[:-1] + ddx_raw[1:]) * dt_fine))
    dv_match = float(dy_fine[-1]) - v_raw_end
    ddx_fine = ddx_raw + dv_match * phi_v

    dx_fine = np.zeros_like(t_fine)
    dx_fine[1:] = np.cumsum(0.5 * (ddx_fine[:-1] + ddx_fine[1:]) * dt_fine)
    x_fine = np.zeros_like(t_fine)
    x_fine[1:] = np.cumsum(0.5 * (dx_fine[:-1] + dx_fine[1:]) * dt_fine)
    x_offset = float(x_fine[-1] - y_fine[-1])

    n_ramp_knots = int(round(ramp_duration / spline_dt))
    for i_k in range(n_ramp_knots):
        ms_spline.addPoint(float(delay + i_k * spline_dt), float(x_fine[i_k * sub]))
    for t in np.arange(delay + ramp_duration, target_time + 0.02, spline_dt):
        dt_rel = t - delay
        y = float(np.sum(amps * np.sin(omegas * dt_rel + phases)))
        ms_spline.addPoint(float(t), float(y + x_offset))
    return ms_spline


def _configure_controller(controller, params):
    """Apply all calibrated 3-axis parameters identically to evaluate_and_plot_tuned_3axis.py."""
    double_props = {
        "K_ton": params["K_ton"],
        "p_ton": params["p_ton"],
        "K_sc": params["K_sc"],
        "z_sc": params["z_sc"],
        "p_sc": params["p_sc"],
        "K_phas": params["K_phas"],
        "z_phas": params["z_phas"],
        "p_phas": params["p_phas"],
        "p_phas2": -1.0,
        "k_p": params["kp_prop"],
        "Kp_task_pitch": params["Kp_pitch"],
        "Kd_task_pitch": params["Kd_pitch"],
        "Kp_task_roll": params["Kp_roll"],
        "Kd_task_roll": params["Kd_roll"],
        "Kp_task_yaw": params["Kp_yaw"],
        "Kd_task_yaw": params["Kd_yaw"],
        "k_vigor": 0.0,
        "trajectory_tau": 0.1,
        "K_eso": 0.0,
        "Ki_task": params["Ki_pitch"],
        "Ki_task_pitch": params["Ki_pitch"],
        "Ki_task_roll": params.get("Ki_roll", 0.0),
        "Ki_task_yaw": params.get("Ki_yaw", 0.0),
        "Kp_proprioception": params["kp_prop"],
        "z_ccr": params["z_ccr"],
        "p_ccr": params["p_ccr"],
        "K_gamma_dyn": params["k_gamma_dyn"],
        "K_gamma_stat": params["k_gamma_stat"],
        "delay_vcr": 0.010,
        "delay_ccr": 0.015,
        "delay_smith_proprio": 0.007,
        "delay_afferent_proprio": 0.007,
    }
    for prop_name, val in double_props.items():
        osim.PropertyHelper.setValueDouble(float(val), controller.updPropertyByName(prop_name))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))

    for pk in PER_AXIS_PROP_KEYS:
        if pk in params and params[pk] is not None:
            osim.PropertyHelper.setValueDouble(float(params[pk]), controller.updPropertyByName(pk))


def _write_mot_file(filepath, header_title, in_degrees_str, col_names, data_rows):
    """Write an OpenSim version=1 .mot file."""
    with open(filepath, "w", encoding="utf-8") as f_out:
        f_out.write(f"{header_title}\n")
        f_out.write("version=1\n")
        f_out.write(f"nRows={len(data_rows)}\n")
        f_out.write(f"nColumns={len(col_names)}\n")
        f_out.write(f"inDegrees={in_degrees_str}\n")
        f_out.write("endheader\n")
        f_out.write("\t".join(col_names) + "\n")
        for row in data_rows:
            f_out.write("\t".join(f"{val:.8f}" for val in row) + "\n")


def _build_test_excitation(mode):
    """Return coordinate name, spline, frequencies, amplitudes (in target gain denominator units), phases, targets, and end time."""
    delay = 1.5
    ramp_duration = 1.5

    if mode == "APEO":
        ordered = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
        K = len(ordered)
        freqs = np.array([item[0] for item in ordered], dtype=float)
        tgt_gains = np.array([item[1] for item in ordered], dtype=float)
        tgt_phases = np.array([item[2] for item in ordered], dtype=float)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        base_amps = np.array([0.0215 * ((0.835 / f) ** 1.0) for f in freqs])
        scale_factor = float(str(1.0 / math.sqrt(K)))
        amps_input_unit = np.maximum(base_amps * scale_factor, 0.0)
        target_time = delay + ramp_duration + 6.0 / freqs[0]
        spline = _build_apeo_spline(freqs, amps_input_unit, phases, delay, ramp_duration, target_time)
        return "spine_tx", spline, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration, target_time

    df_lat, df_yaw = load_aligned_data()
    df_target = df_lat if mode == "LatEC" else df_yaw
    freqs = np.asarray(df_target["Frequency_Hz"], dtype=float)
    K = len(freqs)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
    target_time = delay + ramp_duration + 2.0 / freqs[0]

    if mode == "LatEC":
        amps_input_unit = np.asarray(get_latec_amplitudes(freqs), dtype=float)
        tgt_gains = np.asarray(df_target["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.asarray(df_target["Exp_Roll_Phase_deg"], dtype=float)
        spline, _ = build_c2_multisine_spline(freqs, amps_input_unit, phases, delay, ramp_duration, target_time)
        return "spine_tz", spline, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration, target_time

    if mode == "YawMA":
        amps_input_unit = np.asarray(get_yawma_amplitudes_deg(freqs), dtype=float)
        amps_sim_unit = np.radians(amps_input_unit)
        tgt_gains = np.asarray(df_target["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.asarray(df_target["Exp_Yaw_Phase_deg"], dtype=float)
        spline, _ = build_c2_zero_offset_spline_yaw(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
        return "gndyaw", spline, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration, target_time

    raise ValueError(f"Unsupported test mode: {mode}")


def _compute_classic_bode(t_arr, y_fit_deg_full, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration):
    """Demodulate steady-state harmonics and return Classic Bode Error Score (pts)."""
    m_post = t_arr >= (delay + ramp_duration)
    t_fit = t_arr[m_post]
    y_fit_deg = y_fit_deg_full[m_post]
    t_rel = t_fit - delay

    cols = [np.ones_like(t_fit), t_fit - (delay + ramp_duration)]
    k_count = len(freqs)
    for k in range(k_count):
        omega_k = 2.0 * math.pi * freqs[k]
        psi_k = omega_k * t_rel + phases[k]
        cols.append(np.sin(psi_k))
        cols.append(np.cos(psi_k))

    coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit_deg, rcond=None)
    gain_pts = 0.0
    phase_pts = 0.0
    for k in range(k_count):
        a_sin = float(coeffs[2 + 2 * k])
        b_cos = float(coeffs[2 + 2 * k + 1])
        sim_gain = math.hypot(a_sin, b_cos) / float(amps_input_unit[k])
        sim_phase = math.degrees(math.atan2(b_cos, a_sin))
        db_err = 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k]))
        ph_err = ((sim_phase - float(tgt_phases[k]) + 180.0) % 360.0) - 180.0
        gain_pts += 100.0 * (db_err ** 2)
        phase_pts += 1.0 * (ph_err ** 2)
    return round(gain_pts + phase_pts, 2), round(gain_pts, 2), round(phase_pts, 2)


def export_single_test_mot(spec):
    """Simulate one reflex benchmark, write its three .mot variants, and return verification metrics."""
    mode, prefix = spec
    ensure_libraries_loaded()

    params_path = os.path.join(_ROOT, "logs", "roll_yaw_baseline", "tuned_3axis_params.json")
    with open(params_path, "r", encoding="utf-8") as f_p:
        params = json.load(f_p)

    model = osim.Model(os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim"))
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl
    if controller is None:
        raise RuntimeError("HeadNeckNeuralController not found in model!")

    _configure_controller(controller, params)

    (
        pert_coord_name, spline, freqs, amps_input_unit, phases,
        tgt_gains, tgt_phases, delay, ramp_duration, target_time
    ) = _build_test_excitation(mode)

    zero_func = osim.Constant(0.0)
    for coord_name in GROUND_TRUNK_COORDS:
        if coord_name == pert_coord_name:
            continue
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    for eye_c in EYE_COORDS:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    coord_pert = model.updCoordinateSet().get(pert_coord_name)
    coord_pert.set_locked(False)
    coord_pert.setDefaultIsPrescribed(True)
    coord_pert.setPrescribedFunction(spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    r_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(r_head_to_world).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q.get(idx))

    for state_name in [
        "pitch_error_integral", "roll_error_integral", "yaw_error_integral",
        "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw",
    ]:
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{state_name}", 0.0)

    muscles = model.updMuscles()
    active_muscle_names = []
    active_muscle_objs = []
    k_gdyn_eff = min(max(float(params["k_gamma_dyn"]), 0.0), 1.0)
    k_gstat_eff = min(max(float(params["k_gamma_stat"]), 0.0), 1.0)
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        active_muscle_names.append(name)
        active_muscle_objs.append(m)
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

    coord_set = model.getCoordinateSet()
    n_coords = coord_set.getSize()
    coord_objs = [coord_set.get(i) for i in range(n_coords)]
    coord_names = [c.getName() for c in coord_objs]
    # OpenSim SimbodyEngine::convertDegreesToRadians only converts MotionType::Rotational (1);
    # Translational (2) and SimmSpline-based Coupled (3: pitch1/2, roll1/2, yaw1/2) must stay in meters / radians.
    coord_is_rot = [int(c.getMotionType()) == 1 for c in coord_objs]

    c_p1 = coord_set.get("pitch1")
    c_p2 = coord_set.get("pitch2")
    c_r1 = coord_set.get("roll1")
    c_r2 = coord_set.get("roll2")
    c_y1 = coord_set.get("yaw1")
    c_y2 = coord_set.get("yaw2")
    c_gndyaw = coord_set.get("gndyaw")

    rows_rad = []
    rows_deg = []
    rows_deg_mus = []
    t_eval = []
    y_eval = []
    aborted = False

    def record_frame(t_val, st, include_in_eval=True):
        p_deg = math.degrees(c_p1.getValue(st) + c_p2.getValue(st))
        r_deg = math.degrees(c_r1.getValue(st) + c_r2.getValue(st))
        y_rel_deg = math.degrees(c_y1.getValue(st) + c_y2.getValue(st))
        yaw_glob_deg = math.degrees(c_gndyaw.getValue(st)) + y_rel_deg

        row_rad = [t_val]
        row_deg = [t_val]
        for c_obj, is_rot in zip(coord_objs, coord_is_rot):
            v_raw = c_obj.getValue(st)
            row_rad.append(v_raw)
            row_deg.append(math.degrees(v_raw) if is_rot else v_raw)
        rows_rad.append(row_rad)
        rows_deg.append(row_deg)

        row_mus = list(row_deg)
        for m_obj in active_muscle_objs:
            row_mus.append(m_obj.getStateVariableValue(st, "activation"))
        rows_deg_mus.append(row_mus)

        if include_in_eval:
            t_eval.append(t_val)
            if mode == "APEO":
                y_eval.append(p_deg)
            elif mode == "LatEC":
                y_eval.append(-r_deg)
            else:
                y_eval.append(yaw_glob_deg)

        if abs(p_deg) > 45.0 or abs(r_deg) > 35.0 or abs(y_rel_deg) > 48.0:
            return True
        return False

    print(f"[START] {mode} ({pert_coord_name}, duration={target_time:.4f}s)...", flush=True)
    record_frame(0.0, osim.State(state), include_in_eval=False)

    step = 0.005
    current_time = 0.0
    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)
        if record_frame(current_time, state, include_in_eval=True):
            aborted = True
            print(f"[ABORT] {mode} exceeded kinematic safety threshold at t={current_time:.3f}s", file=sys.stderr, flush=True)
            break

    classic_score, gain_pts, phase_pts = _compute_classic_bode(
        np.array(t_eval), np.array(y_eval),
        freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration,
    )

    out_dir = os.path.join(_ROOT, "state_files")
    os.makedirs(out_dir, exist_ok=True)
    mot_rad_path = os.path.join(out_dir, f"{prefix}.mot")
    mot_deg_path = os.path.join(out_dir, f"{prefix}_degrees.mot")
    mot_mus_path = os.path.join(out_dir, f"{prefix}_with_muscles.mot")

    cols_base = ["time"] + coord_names
    cols_mus = ["time"] + coord_names + [f"/forceset/{m}/activation" for m in active_muscle_names]

    _write_mot_file(mot_rad_path, prefix, "no", cols_base, rows_rad)
    _write_mot_file(mot_deg_path, f"{prefix}_degrees", "yes", cols_base, rows_deg)
    _write_mot_file(mot_mus_path, f"{prefix}_with_muscles", "yes", cols_mus, rows_deg_mus)

    print(
        f"[DONE] {mode}: nRows={len(rows_rad)}, t_end={rows_rad[-1][0]:.4f}s, "
        f"aborted={aborted}, Classic Bode={classic_score:.2f} pts (G={gain_pts:.1f}, P={phase_pts:.1f})",
        flush=True,
    )
    return {
        "mode": mode,
        "aborted": aborted,
        "nRows": len(rows_rad),
        "nCoords": len(cols_base),
        "nMuscleCols": len(cols_mus),
        "t_end": round(rows_rad[-1][0], 4),
        "classic_bode_score": classic_score,
        "gain_loss_pts": gain_pts,
        "phase_loss_pts": phase_pts,
        "files": [mot_rad_path, mot_deg_path, mot_mus_path],
    }


def main():
    specs = [
        ("APEO", "tuned_3axis_APEO_pitch_multisine"),
        ("LatEC", "tuned_3axis_LatEC_roll_multisine"),
        ("YawMA", "tuned_3axis_YawMA_yaw_multisine"),
    ]
    with ProcessPoolExecutor(max_workers=3) as ex:
        results = list(ex.map(export_single_test_mot, specs))

    print("\n=== Exported .mot Summary ===", flush=True)
    for res in results:
        print(
            f"{res['mode']:>5s} | rows={res['nRows']:4d} | t_end={res['t_end']:6.3f}s | "
            f"Classic={res['classic_bode_score']:8.2f} pts (G={res['gain_loss_pts']:.1f}, P={res['phase_loss_pts']:.1f}) | "
            f"aborted={res['aborted']}",
            flush=True,
        )
        for p in res["files"]:
            print(f"   {p}", flush=True)


if __name__ == "__main__":
    main()
