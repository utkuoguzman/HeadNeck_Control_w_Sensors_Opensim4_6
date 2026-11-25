#!/usr/bin/env python3
"""Option 1 Step 1: 16-Experiment Parallel Lyapunov, Internal-State & Pathway Diagnostic."""

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
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0
from run_roll_yaw_baseline import load_aligned_data, get_latec_amplitudes, build_c2_multisine_spline
from scripts.generate_standard_plot import MODEL_PATH, apply_cli_overrides_and_inspect

G2_CLEAN_JSON = os.path.join(_ROOT, "logs", "cmaes_latec_only_18p_clean_champion.json")
G35_JSON = os.path.join(_ROOT, "logs", "cmaes_3axis_18p_roll_champion.json")
OUT_JSON = os.path.join(_ROOT, "logs", "roll_drift_chaos_diagnostic.json")

STATE_VARS = (
    "roll_error_integral",
    "vol_filt_roll",
    "ton_state_roll",
    "sc_state_roll",
    "phas_state_roll",
    "phas2_state_yaw",
    "otolith_irr_state_z",
    "luen_otolith_z",
)


def _format_overrides(pdict):
    overrides = []
    for k, v in pdict.items():
        if isinstance(v, str):
            overrides.append((k, v.strip().lower()))
        else:
            overrides.append((k, f"{float(v):.15g}"))
    return overrides


def run_diagnostic_lat_ec(args):
    """Run one diagnostic LatEC simulation with internal state telemetry."""
    idx, name, pdict, sim_cfg = args
    ensure_libraries_loaded()

    int_acc = float(sim_cfg.get("int_acc", 1e-4))
    min_dt = float(sim_cfg.get("min_dt", 1e-4))
    max_dt = float(sim_cfg.get("max_dt", 1e-3))
    zero_pert = bool(sim_cfg.get("zero_pert", False))

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

    overrides = _format_overrides(pdict)
    bool_state, false_booleans, bool_banner = apply_cli_overrides_and_inspect(controller, overrides)

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
    target_time = delay + pert_duration  # 15.8 s

    df_lat, _ = load_aligned_data()
    freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
    tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
    tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
    K = len(freqs)
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
    amps_input_unit = np.array(get_latec_amplitudes(freqs), dtype=float)
    amps_sim_unit = amps_input_unit.copy()

    coord_pert = model.updCoordinateSet().get("spine_tz")
    coord_pert.set_locked(False)
    coord_pert.setDefaultIsPrescribed(True)
    if not zero_pert:
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_sim_unit, phases, delay, ramp_duration, target_time)
        coord_pert.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    for q_idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{q_idx}", q.get(q_idx))

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
    manager.setIntegratorAccuracy(int_acc)
    manager.setIntegratorMinimumStepSize(min_dt)
    manager.setIntegratorMaximumStepSize(max_dt)

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

    out_mahony_roll = osim.OutputDouble.safeDownCast(controller.getOutput("mahony_roll"))
    out_est_neck_roll = osim.OutputDouble.safeDownCast(controller.getOutput("est_neck_roll"))

    t_list = []
    r_iso_list, r_buckle_list, p_list, y_rel_list = [], [], [], []
    mahony_roll_list, est_neck_roll_list = [], []
    sv_lists = {sv: [] for sv in STATE_VARS}

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

        t_list.append(current_time)
        r_iso_list.append(-r_deg)
        r_buckle_list.append(r1_deg - r2_deg)
        p_list.append(p_deg)
        y_rel_list.append(y_rel_deg)
        mahony_roll_list.append(math.degrees(out_mahony_roll.getValue(state)))
        est_neck_roll_list.append(math.degrees(out_est_neck_roll.getValue(state)))

        for sv in STATE_VARS:
            val = float(model.getStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}"))
            sv_lists[sv].append(val)

        if abs(p_deg) > 45.0 or abs(r_deg) > 35.0 or abs(y_rel_deg) > 48.0:
            aborted = True
            abort_time_s = float(current_time)
            abort_reason = f"t={current_time:.2f}s p={p_deg:+.2f} r={r_deg:+.2f} y={y_rel_deg:+.2f}"
            break

    t_arr = np.asarray(t_list, dtype=float)
    r_iso_arr = np.asarray(r_iso_list, dtype=float)
    r_buckle_arr = np.asarray(r_buckle_list, dtype=float)
    p_arr = np.asarray(p_list, dtype=float)
    y_rel_arr = np.asarray(y_rel_list, dtype=float)
    mah_arr = np.asarray(mahony_roll_list, dtype=float)
    est_arr = np.asarray(est_neck_roll_list, dtype=float)
    sv_arrs = {sv: np.asarray(sv_lists[sv], dtype=float) for sv in STATE_VARS}

    eval_start_time = 5.0
    m_post = t_arr >= eval_start_time
    n_post = int(np.sum(m_post))

    full_pitch_dev_max = float(np.max(np.abs(p_arr - p_arr[0])))
    full_roll_max = float(np.max(np.abs(r_iso_arr)))
    full_yaw_rel_max = float(np.max(np.abs(y_rel_arr)))
    post_pitch_dev_max = float(np.max(np.abs(p_arr[m_post] - p_arr[0]))) if np.any(m_post) else full_pitch_dev_max
    post_yaw_rel_max = float(np.max(np.abs(y_rel_arr[m_post]))) if np.any(m_post) else full_yaw_rel_max

    freq_results = {}
    hf_noise_rms = 0.0
    corr_vel = 0.0
    hf_penalty = 0.0
    pitch_crosstalk_penalty = 0.0
    gain_loss_pts = 0.0
    phase_loss_pts = 0.0
    bode_latec = 0.0

    if aborted or n_post < 50:
        progress = max(0.0, min(1.0, abort_time_s / target_time))
        bode_latec = 6.0 * (200000.0 + (1.0 - progress) * 50000.0)
        pitch_crosstalk_penalty = 100.0 * (full_pitch_dev_max ** 2 + full_yaw_rel_max ** 2)
    else:
        t_fit = t_arr[m_post]
        t_rel = t_fit - delay
        y_fit_deg = r_iso_arr[m_post]

        if not zero_pert:
            cols = [np.ones_like(t_fit), t_fit - eval_start_time]
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

                freq_results[f"{f_k:.4f}"] = {
                    "freq_hz": f_k,
                    "gain_db_diff": round(db_err, 4),
                    "gain_loss_pts": round(g_pts, 2),
                    "sim_phase": round(tp + ph_err, 2),
                    "target_phase": round(tp, 2),
                    "phase_err": round(ph_err, 2),
                    "phase_loss_pts": round(p_pts, 2),
                }

            vel_actual = np.gradient(y_fit_deg, t_fit)
            vel_harm = np.gradient(sim_harm_fit, t_fit)
            vel_ref = np.gradient(ref_fit, t_fit)
            hf_noise_rms = float(np.sqrt(np.mean((vel_actual - vel_harm) ** 2)))
            corr_vel = float(np.corrcoef(vel_actual, vel_ref)[0, 1]) if np.std(vel_actual) > 1e-6 else 0.0
            bode_latec = gain_loss_pts + phase_loss_pts
        else:
            vel_actual = np.gradient(y_fit_deg, t_fit)
            hf_noise_rms = float(np.sqrt(np.mean(vel_actual ** 2)))

        pitch_crosstalk_penalty = 100.0 * (post_pitch_dev_max ** 2 + post_yaw_rel_max ** 2)
        hf_penalty = 25.0 * (hf_noise_rms ** 2)

    # Drift & internal state metrics over post-5s (or available points)
    m_stat = m_post if np.any(m_post) else np.ones_like(t_arr, dtype=bool)
    r_post = r_iso_arr[m_stat]
    rb_post = r_buckle_arr[m_stat]
    mah_post = mah_arr[m_stat]
    est_post = est_arr[m_stat]
    ri_post = sv_arrs["roll_error_integral"][m_stat]
    ton_post = sv_arrs["ton_state_roll"][m_stat]
    ph_post = sv_arrs["phas_state_roll"][m_stat]
    sc_post = sv_arrs["sc_state_roll"][m_stat]

    if len(r_post) >= 500:
        r_smooth = np.convolve(r_post, np.ones(500, dtype=float) / 500.0, mode="valid")
        r_iso_lowfreq_ptp = float(np.ptp(r_smooth))
    else:
        r_iso_lowfreq_ptp = float(np.ptp(r_post)) if len(r_post) > 0 else 0.0

    r_slope = float(np.polyfit(t_arr[m_stat], r_post, 1)[0]) if len(r_post) >= 10 else 0.0

    mah_delta_post = mah_post - mah_arr[0]
    mah_plus = mah_delta_post + r_post   # mahony_delta - (r1+r2) if opposite sign to r_iso
    mah_minus = mah_delta_post - r_post  # mahony_delta - (-r_arr) if same sign as r_iso

    sat_frac = float(np.mean(np.abs(ri_post) >= 0.399)) if len(ri_post) > 0 else 0.0

    summary = {
        "name": name,
        "zero_pert": zero_pert,
        "sim_cfg": {"int_acc": int_acc, "min_dt": min_dt, "max_dt": max_dt},
        "bool_banner": bool_banner,
        "aborted": aborted,
        "abort_time_s": round(abort_time_s, 3),
        "abort_reason": abort_reason,
        "bode_latec": round(bode_latec, 2),
        "gain_loss_pts": round(gain_loss_pts, 2),
        "phase_loss_pts": round(phase_loss_pts, 2),
        "hf_noise_rms": round(hf_noise_rms, 3),
        "corr_vel": round(corr_vel, 4),
        "xtalk_latec": round(pitch_crosstalk_penalty, 2),
        "hf_penalty": round(hf_penalty, 2),
        "freq_results": freq_results,
        "post5s_metrics": {
            "r_iso_mean": round(float(np.mean(r_post)), 4),
            "r_iso_std": round(float(np.std(r_post)), 4),
            "r_iso_ptp": round(float(np.ptp(r_post)), 4),
            "r_iso_lowfreq_ptp": round(r_iso_lowfreq_ptp, 4),
            "r_iso_slope_deg_s": round(r_slope, 5),
            "r_buckle_mean": round(float(np.mean(rb_post)), 4),
            "r_buckle_ptp": round(float(np.ptp(rb_post)), 4),
            "mahony_roll_raw_mean": round(float(np.mean(mah_post)), 4),
            "mahony_roll_raw_ptp": round(float(np.ptp(mah_post)), 4),
            "mahony_plus_riso_rms": round(float(np.sqrt(np.mean(mah_plus ** 2))), 4),
            "mahony_plus_riso_mean": round(float(np.mean(mah_plus)), 4),
            "mahony_minus_riso_rms": round(float(np.sqrt(np.mean(mah_minus ** 2))), 4),
            "mahony_minus_riso_mean": round(float(np.mean(mah_minus)), 4),
            "est_neck_roll_mean": round(float(np.mean(est_post)), 4),
            "est_neck_roll_ptp": round(float(np.ptp(est_post)), 4),
            "roll_err_int_min": round(float(np.min(ri_post)), 5),
            "roll_err_int_max": round(float(np.max(ri_post)), 5),
            "roll_err_int_sat_frac": round(sat_frac, 4),
            "ton_state_roll_mean": round(float(np.mean(ton_post)), 5),
            "ton_state_roll_ptp": round(float(np.ptp(ton_post)), 5),
            "phas_state_roll_mean": round(float(np.mean(ph_post)), 5),
            "phas_state_roll_ptp": round(float(np.ptp(ph_post)), 5),
            "sc_state_roll_rms": round(float(np.sqrt(np.mean(sc_post ** 2))), 5),
            "sc_state_roll_ptp": round(float(np.ptp(sc_post)), 5),
        },
    }

    # Keep full 5ms r_iso for exact divergence timestamp + 0.05s downsampled trajectories in memory
    ds_slice = slice(None, None, 10)  # 0.05s = 316 points
    traj_mem = {
        "t_full": t_arr,
        "r_iso_full": r_iso_arr,
        "t": t_arr[ds_slice],
        "r_iso_deg": r_iso_arr[ds_slice],
        "r_buckle_deg": r_buckle_arr[ds_slice],
        "mahony_roll_deg": mah_arr[ds_slice],
        "est_neck_roll_deg": est_arr[ds_slice],
        "roll_error_integral": sv_arrs["roll_error_integral"][ds_slice],
        "phas_state_roll": sv_arrs["phas_state_roll"][ds_slice],
        "sc_state_roll": sv_arrs["sc_state_roll"][ds_slice],
        "ton_state_roll": sv_arrs["ton_state_roll"][ds_slice],
        "phas2_state_yaw": sv_arrs["phas2_state_yaw"][ds_slice],
        "otolith_irr_state_z": sv_arrs["otolith_irr_state_z"][ds_slice],
    }
    return idx, summary, traj_mem


def compute_pair_divergence(name_a, traj_a, name_b, traj_b):
    """Compute pairwise divergence between two diagnostic trajectories."""
    n_full = min(len(traj_a["r_iso_full"]), len(traj_b["r_iso_full"]))
    t_f = traj_a["t_full"][:n_full]
    dr_full = np.abs(traj_b["r_iso_full"][:n_full] - traj_a["r_iso_full"][:n_full])

    idx_1e4 = np.where(dr_full > 1e-4)[0]
    idx_0p05 = np.where(dr_full > 0.05)[0]
    t_div_1e4 = round(float(t_f[idx_1e4[0]]), 3) if len(idx_1e4) > 0 else None
    t_div_0p05 = round(float(t_f[idx_0p05[0]]), 3) if len(idx_0p05) > 0 else None

    n_ds = min(len(traj_a["t"]), len(traj_b["t"]))
    max_diffs = {}
    for ch in (
        "r_iso_deg",
        "r_buckle_deg",
        "mahony_roll_deg",
        "est_neck_roll_deg",
        "roll_error_integral",
        "phas_state_roll",
        "sc_state_roll",
        "ton_state_roll",
        "phas2_state_yaw",
        "otolith_irr_state_z",
    ):
        diff = np.abs(traj_b[ch][:n_ds] - traj_a[ch][:n_ds])
        max_diffs[ch] = float(np.max(diff)) if len(diff) > 0 else 0.0

    max_r_full = float(np.max(dr_full)) if len(dr_full) > 0 else 0.0
    max_diffs["r_iso_deg_full_5ms"] = max_r_full

    return {
        "pair": f"{name_a} vs {name_b}",
        "t_div_1e4_deg_s": t_div_1e4,
        "t_div_0p05_deg_s": t_div_0p05,
        "final_abs_r_iso_diff_deg": round(float(dr_full[-1]), 6) if n_full > 0 else 0.0,
        "max_abs_diffs": max_diffs,
    }


def build_experiments(g2_clean, g35):
    """Build the 16 diagnostic experiments D01..D16."""
    def _clone(base, mods=None):
        p = dict(base)
        if mods:
            p.update(mods)
        return p

    default_cfg = {"int_acc": 1e-4, "min_dt": 1e-4, "max_dt": 1e-3, "zero_pert": False}

    exps = [
        ("D01_G2_BASE", _clone(g2_clean), dict(default_cfg)),
        ("D02_G2_EPS_1E9", _clone(g2_clean, {"K_sc_roll": float(g2_clean["K_sc_roll"]) * (1.0 + 1e-9)}), dict(default_cfg)),
        ("D03_G2_TIGHT_ACC", _clone(g2_clean), {"int_acc": 1e-6, "min_dt": 5e-5, "max_dt": 2.5e-4, "zero_pert": False}),
        ("D04_G2_FIXED_DT", _clone(g2_clean), {"int_acc": 1e-4, "min_dt": 2.5e-4, "max_dt": 2.5e-4, "zero_pert": False}),
        ("D05_G2_FIXED_DT_EPS1E9", _clone(g2_clean, {"K_sc_roll": float(g2_clean["K_sc_roll"]) * (1.0 + 1e-9)}), {"int_acc": 1e-4, "min_dt": 2.5e-4, "max_dt": 2.5e-4, "zero_pert": False}),
        ("D06_G2_QUIET_ZERO_PERT", _clone(g2_clean), {"int_acc": 1e-4, "min_dt": 1e-4, "max_dt": 1e-3, "zero_pert": True}),
        ("D07_G2_PHAS0", _clone(g2_clean, {"K_phas_roll": 0.0}), dict(default_cfg)),
        ("D08_G2_KI0", _clone(g2_clean, {"Ki_task_roll": 0.0}), dict(default_cfg)),
        ("D09_G2_HIGH_BUCKLE", _clone(g2_clean, {"Kp_buckle_roll": 2.5, "Kd_buckle_roll": 0.38}), dict(default_cfg)),
        ("D10_G2_LOW_SPINAL_CCR", _clone(g2_clean, {"Kp_proprioception_roll": 0.003}), dict(default_cfg)),
        ("D11_G2_SOFT_LEAD", _clone(g2_clean, {"tau_lead_pole_roll": 0.025}), dict(default_cfg)),
        ("D12_G2_STIFF_LOWFREQ", _clone(g2_clean, {"p_ton_roll": 9.0, "K_ton_roll": 2.2, "Kp_task_roll": 3.0, "Kd_task_roll": 2.0, "Kp_prop_roll": 3.7}), dict(default_cfg)),
        ("D13_G35_BASE", _clone(g35), dict(default_cfg)),
        ("D14_G35_EPS_1E9", _clone(g35, {"K_sc_roll": float(g35["K_sc_roll"]) * (1.0 + 1e-9)}), dict(default_cfg)),
        ("D15_G2_MAHONY_OFF", _clone(g2_clean, {"enable_mahony": "false"}), dict(default_cfg)),
        ("D16_G2_DECODER_OFF", _clone(g2_clean, {"enable_decoder": "false"}), dict(default_cfg)),
    ]
    return exps


def main():
    parser = argparse.ArgumentParser(description="Option 1 Step 1: Roll Drift & Chaos Diagnostic (16 parallel runs)")
    parser.add_argument("--workers", type=int, default=16, help="Local worker processes")
    args = parser.parse_args()

    t0 = time.time()
    g2_clean = {k: float(v) for k, v in json.load(open(G2_CLEAN_JSON, "r", encoding="utf-8"))["params"].items()}
    g35 = {k: float(v) for k, v in json.load(open(G35_JSON, "r", encoding="utf-8"))["params"].items()}

    exps = build_experiments(g2_clean, g35)
    tasks = [(i, name, pdict, cfg) for i, (name, pdict, cfg) in enumerate(exps)]

    print(f"=== Running {len(tasks)} Roll Drift/Chaos Diagnostic Experiments on {args.workers} workers ===", flush=True)
    summaries = [None] * len(tasks)
    trajs = {}

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(run_diagnostic_lat_ec, t) for t in tasks]
        for fut in as_completed(futs):
            idx, rec, traj_mem = fut.result()
            summaries[idx] = rec
            trajs[rec["name"]] = traj_mem
            pm = rec["post5s_metrics"]
            print(
                f"  [done] {rec['name']:24s} Bode={rec['bode_latec']:9.1f} RollHF={rec['hf_noise_rms']:6.3f} "
                f"LowPtP={pm['r_iso_lowfreq_ptp']:6.3f} BuckPtP={pm['r_buckle_ptp']:5.2f} Abort={rec['aborted']}",
                flush=True,
            )

    pairs = [
        ("D01_G2_BASE", "D02_G2_EPS_1E9"),
        ("D04_G2_FIXED_DT", "D05_G2_FIXED_DT_EPS1E9"),
        ("D13_G35_BASE", "D14_G35_EPS_1E9"),
    ]
    divergences = [compute_pair_divergence(a, trajs[a], b, trajs[b]) for a, b in pairs]

    dt = time.time() - t0
    out_payload = {
        "elapsed_s": round(dt, 1),
        "records": summaries,
        "divergences": divergences,
    }
    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(out_payload, f, indent=2)

    print(f"\n=== Diagnostic Summary ({dt:.1f}s) ===", flush=True)
    hdr = (
        f"{'Name':24s} | {'Bode':>9s} | {'GainL':>7s} | {'PhsL':>7s} | {'HF_rms':>6s} | "
        f"{'LowPtP':>6s} | {'rMean':>6s} | {'BuckPtP':>7s} | {'MahPtP':>6s} | {'EstPtP':>6s} | {'IntSat':>6s}"
    )
    print(hdr)
    print("-" * len(hdr))
    for r in summaries:
        pm = r["post5s_metrics"]
        print(
            f"{r['name']:24s} | {r['bode_latec']:9.1f} | {r['gain_loss_pts']:7.0f} | {r['phase_loss_pts']:7.0f} | "
            f"{r['hf_noise_rms']:6.3f} | {pm['r_iso_lowfreq_ptp']:6.3f} | {pm['r_iso_mean']:+6.3f} | "
            f"{pm['r_buckle_ptp']:7.3f} | {pm['mahony_roll_raw_ptp']:6.3f} | {pm['est_neck_roll_ptp']:6.3f} | {pm['roll_err_int_sat_frac']:6.3f}"
        )

    print("\n=== 1e-9 Perturbation Divergence Analysis ===")
    for d in divergences:
        md = d["max_abs_diffs"]
        print(
            f"  {d['pair']}: t_div(>1e-4 deg)={d['t_div_1e4_deg_s']}s, t_div(>0.05 deg)={d['t_div_0p05_deg_s']}s, "
            f"max_dR={md['r_iso_deg_full_5ms']:.5f} deg, max_dBuckle={md['r_buckle_deg']:.5f}, "
            f"max_dMah={md['mahony_roll_deg']:.5f}, max_dInt={md['roll_error_integral']:.5f}, "
            f"max_dPhas={md['phas_state_roll']:.5f}, max_dSc={md['sc_state_roll']:.5f}"
        )


if __name__ == "__main__":
    main()
