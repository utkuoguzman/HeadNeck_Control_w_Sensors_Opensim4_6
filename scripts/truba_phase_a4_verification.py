#!/usr/bin/env python3
"""Phase A4 Comprehensive 5-Suite Noise-OFF + Multi-Seed Noise-ON Verification (TRUBA).

Runs 63 simulations across 48 cores on HYOID_HeadNeckNeural_Model_KuoLigaments.osim
with the finalized Phase A3 champion (Tier A cleanup + Tier B SRS/GTO/Renshaw/Capsule + Kuo67 B_k50_c30):
  Suite 1 (12 sims): Exact 15.8s 3-Axis Passive Bode (APEO, LatEC, YawMA) Noise-OFF + 3-Seed Noise-ON
  Suite 2 (34 sims): Zheng 2g/3g Sled 16-Case Urgency Sweep Noise-OFF + 3-Seed Noise-ON on key conditions
  Suite 3 ( 2 sims): 18.0s Voluntary 50deg RPY 10-Phase Sweep (Noise-OFF + Seed 1 Noise-ON)
  Suite 4 (10 sims): 5.0s Quiet Stance 2x2 Biological Noise Ablation (OFF + 3 seeds each S/M/Both)
  Suite 5 ( 5 sims): 5-DOF Voluntary AP/Lat Shift (4 cases) + VOL60 (1 case)
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

import numpy as np
import opensim as osim

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cmaes_worker_p22 import CANONICAL_TARGETS, PER_MUSCLE_72_U0, ensure_libraries_loaded
import scripts.truba_phase_a3_joint_cmaes as a3
import scripts.truba_phase_a3_exact_roi_cmaes as a3ex
from scripts.run_roll_yaw_baseline import (
    build_c2_multisine_spline,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    load_aligned_data,
)
from scripts.tune_roll_and_yaw import build_c2_zero_offset_spline_yaw
from scripts.run_zheng_2g_3g_urgency_sweep import (
    DT_LOG,
    SCM_NAMES,
    SPLEN_CAP_NAMES,
    T_SETTLE,
    T_TOTAL,
    build_sled_spline,
)
from scripts.run_rpy_50deg_yaw_sweep import AXIS_INDEX, PHASES, compute_overshoot, compute_t95

OUT_JSON = os.path.join(_ROOT, "logs", "phase_a3", "phase_a4_full_verification.json")


def _build_model_and_ctrl(params: dict):
    ensure_libraries_loaded()
    model = osim.Model(a3.KUO_MODEL_PATH)
    controller, false_bools = a3ex._apply_srs_and_ctrl(model, params)
    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)
    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)
    cy1 = model.updCoordinateSet().get("yaw1")
    cy1.set_locked(False)
    cy1.setDefaultIsPrescribed(False)
    return model, controller, cy1, false_bools


def run_bode_detailed(spec: dict) -> dict:
    t0 = time.time()
    mode = spec["mode"]
    ov = dict(spec["params"])
    ov["enable_sensor_noise"] = bool(spec["sensor_noise"])
    ov["enable_motor_noise"] = bool(spec["motor_noise"])
    ov["noise_seed"] = int(spec["seed"])
    ov["noise_scale"] = float(spec["noise_scale"])

    model, controller, cy1, false_bools = _build_model_and_ctrl(ov)
    delay, ramp_duration, target_time, eval_start = 0.8, 1.5, 15.8, 5.0

    if mode == "APEO":
        ordered = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
        freqs = np.array([t[0] for t in ordered], dtype=float)
        tgt_gains = np.array([t[1] for t in ordered], dtype=float)
        tgt_phases = np.array([t[2] for t in ordered], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_in, phases, delay, ramp_duration, target_time)
        pcoord = "spine_tx"
    elif mode == "LatEC":
        df_lat, _ = load_aligned_data()
        freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(get_latec_amplitudes(freqs), dtype=float)
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_in, phases, delay, ramp_duration, target_time)
        pcoord = "spine_tz"
    else:
        _, df_yaw = load_aligned_data()
        freqs = np.array(df_yaw["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_yaw["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.array(df_yaw["Exp_Yaw_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(get_yawma_amplitudes_deg(freqs), dtype=float)
        ms_spline, _ = build_c2_zero_offset_spline_yaw(freqs, np.radians(amps_in), phases, delay, ramp_duration, target_time)
        pcoord = "gndyaw"

    coord_pert = model.updCoordinateSet().get(pcoord)
    coord_pert.setPrescribedFunction(ms_spline)
    state = model.initSystem()
    cy1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)
    a3._init_neck_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False)
    mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)
    mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4)
    mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    mgr.initialize(state)

    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    cr1, cr2 = cs.get("roll1"), cs.get("roll2")
    cy_1, cy_2 = cs.get("yaw1"), cs.get("yaw2")
    cgndyaw = cs.get("gndyaw")

    t_l, p_l, r_l, yr_l, yg_l = [], [], [], [], []
    aborted = False
    abort_reason = ""
    t_now = 0.0
    while t_now < target_time - 1e-9:
        t_now = min(t_now + 0.005, target_time)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            aborted = True
            abort_reason = str(e).splitlines()[0]
            break
        pd = math.degrees(cp1.getValue(state) + cp2.getValue(state))
        rd = math.degrees(cr1.getValue(state) + cr2.getValue(state))
        yrd = math.degrees(cy_1.getValue(state) + cy_2.getValue(state))
        ygd = math.degrees(cgndyaw.getValue(state)) + yrd
        t_l.append(t_now); p_l.append(pd); r_l.append(rd); yr_l.append(yrd); yg_l.append(ygd)
        if abs(pd) > 54.0 or abs(rd) > 35.0 or abs(yrd) > 49.0:
            aborted = True
            abort_reason = f"ROM@{t_now:.2f}s"
            break

    t_arr = np.asarray(t_l); p_arr = np.asarray(p_l); r_arr = np.asarray(r_l)
    yr_arr = np.asarray(yr_l); yg_arr = np.asarray(yg_l)
    main_arr = p_arr if mode == "APEO" else (-r_arr if mode == "LatEC" else yg_arr)
    m_post = t_arr >= eval_start
    if aborted or int(np.sum(m_post)) < 50:
        return {"suite": "bode", "mode": mode, "noise": spec["sensor_noise"], "seed": spec["seed"],
                "aborted": True, "abort_reason": abort_reason, "total_score": 1200000.0, "wall_s": round(time.time()-t0, 1)}

    t_fit = t_arr[m_post]; t_rel = t_fit - delay; y_fit = main_arr[m_post]
    cols = [np.ones_like(t_fit), t_fit - eval_start]
    for k in range(K):
        psi_k = 2.0 * math.pi * float(freqs[k]) * t_rel + float(phases[k])
        cols.append(np.sin(psi_k)); cols.append(np.cos(psi_k))
    coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit, rcond=None)
    ref_fit = np.zeros_like(t_fit); harm_fit = np.zeros_like(t_fit)
    g_loss = 0.0; p_loss = 0.0; frf = {}
    for k in range(K):
        fk = float(freqs[k]); psi = 2.0 * np.pi * fk * t_rel + float(phases[k])
        ref_fit += float(amps_in[k]) * float(tgt_gains[k]) * np.sin(psi + math.radians(float(tgt_phases[k])))
        ac, bc = float(coeffs[2 + 2*k]), float(coeffs[2 + 2*k + 1])
        harm_fit += ac * np.sin(psi) + bc * np.cos(psi)
        sg = math.hypot(ac, bc) / float(amps_in[k])
        rph = math.degrees(math.atan2(bc, ac))
        tp = float(tgt_phases[k])
        ph_err = ((rph - tp + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(sg, 1e-6) / float(tgt_gains[k]))
        g_loss += 100.0 * db_err * db_err; p_loss += ph_err * ph_err
        frf[f"{fk:.4f}"] = {
            "f_hz": round(fk, 4), "sim_gain": round(sg, 3), "tgt_gain": round(float(tgt_gains[k]), 3),
            "db_err": round(db_err, 2), "sim_phase": round(tp + ph_err, 1), "tgt_phase": round(tp, 1), "ph_err": round(ph_err, 1),
        }

    vel_act = np.gradient(y_fit, t_fit); vel_hrm = np.gradient(harm_fit, t_fit); vel_ref = np.gradient(ref_fit, t_fit)
    hf_rms = float(np.sqrt(np.mean((vel_act - vel_hrm)**2)))
    rv = float(np.corrcoef(vel_act, vel_ref)[0, 1]) if np.std(vel_act) > 1e-6 else 0.0
    p_dev = float(np.max(np.abs(p_arr[m_post] - p_arr[0])))
    r_dev = float(np.max(np.abs(r_arr[m_post])))
    y_dev = float(np.max(np.abs(yr_arr[m_post])))
    xt = 100.0 * ((r_dev**2 + y_dev**2) if mode == "APEO" else ((p_dev**2 + y_dev**2) if mode == "LatEC" else (p_dev**2 + r_dev**2)))
    hf_p = (5.0 if mode == "APEO" else 25.0) * (hf_rms**2)
    cls = g_loss + p_loss; tot = cls + xt + hf_p
    out = {
        "suite": "bode", "mode": mode, "noise": bool(spec["sensor_noise"]), "seed": int(spec["seed"]),
        "aborted": False, "false_bools": false_bools,
        "classic_score": round(cls, 2), "gain_loss": round(g_loss, 2), "phase_loss": round(p_loss, 2),
        "xtalk_pen": round(xt, 2), "hf_pen": round(hf_p, 2), "hf_rms": round(hf_rms, 3),
        "total_score": round(tot, 2), "corr_vel": round(rv, 4), "frf": frf, "wall_s": round(time.time()-t0, 1),
    }
    if not spec["sensor_noise"]:
        # Save downsampled waveform for plotting (every 20 ms)
        out["trace"] = {"t": [round(float(v), 3) for v in t_fit[::4]], "sim": [round(float(v), 3) for v in y_fit[::4]], "ref": [round(float(v), 3) for v in ref_fit[::4]]}
    return out


def run_sled_detailed(spec: dict) -> dict:
    t0 = time.time()
    g_level = float(spec["g"]); urgency = float(spec["u"])
    ov = dict(spec["params"])
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0, "Kp_trans_roll": 0.0, "Kp_trans_pitch": 0.0,
        "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0, "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0,
        "shift_station_blend": 0.0, "trajectory_tau": 0.28, "urgency": urgency,
        "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
        "enable_sensor_noise": bool(spec["sensor_noise"]), "enable_motor_noise": bool(spec["motor_noise"]),
        "noise_seed": int(spec["seed"]), "noise_scale": float(spec["noise_scale"]),
    })
    model, controller, cy1, _ = _build_model_and_ctrl(ov)
    ctx = model.updCoordinateSet().get("spine_tx")
    ctx.setPrescribedFunction(build_sled_spline(g_level, sign_x=-1.0))
    state = model.initSystem()
    cy1.setIsPrescribed(state, False)
    ctx.setSpeedValue(state, 0.0)
    a3._init_neck_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    muscles = model.updMuscles()
    splen_idx = [i for i in range(muscles.getSize()) if muscles.get(i).getName() in SPLEN_CAP_NAMES]
    scm_idx = [i for i in range(muscles.getSize()) if muscles.get(i).getName() in SCM_NAMES]
    neck_idx = [i for i in range(muscles.getSize()) if not muscles.get(i).getName().startswith(("r_", "l_"))]
    cp1, cp2 = model.getCoordinateSet().get("pitch1"), model.getCoordinateSet().get("pitch2")

    ts_l, f_l, sp_l, sc_l, ma_l = [], [], [], [], []
    t_now = 0.0; stop_reason = None
    while t_now < T_TOTAL - 1e-9:
        t_next = round(min(t_now + DT_LOG, T_TOTAL), 6)
        try:
            state = mgr.integrate(t_next)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]; break
        t_now = t_next
        model.realizeDynamics(state)
        ts_l.append(t_now)
        f_l.append(-math.degrees(cp1.getValue(state) + cp2.getValue(state)))
        sp_l.append(sum(float(muscles.get(i).getActivation(state)) for i in splen_idx) / len(splen_idx))
        sc_l.append(sum(float(muscles.get(i).getActivation(state)) for i in scm_idx) / len(scm_idx))
        ma_l.append(max(float(muscles.get(i).getActivation(state)) for i in neck_idx))

    if stop_reason is not None or len(ts_l) < 200:
        return {"suite": "sled", "g": g_level, "u": urgency, "noise": bool(spec["sensor_noise"]), "seed": int(spec["seed"]),
                "aborted": True, "stop_reason": stop_reason}

    ts = np.asarray(ts_l); f_arr = np.asarray(f_l); sp_arr = np.asarray(sp_l); sc_arr = np.asarray(sc_l)
    imp = ts >= (T_SETTLE - 1e-9)
    idx0 = int(np.where(imp)[0][0])
    t_ms = (ts[imp] - T_SETTLE) * 1000.0
    f_imp = f_arr[imp] - float(f_arr[idx0])
    sp_imp = sp_arr[imp]; sc_imp = sc_arr[imp]
    n_pk = 0; pk_ms = []
    for k in range(1, len(sp_imp) - 1):
        if sp_imp[k] > 0.08 and sp_imp[k] > sp_imp[k - 1] and sp_imp[k] >= sp_imp[k + 1]:
            lo = max(0, k - 25); hi = min(len(sp_imp), k + 26)
            prom = float(sp_imp[k] - max(np.min(sp_imp[lo:k + 1]), np.min(sp_imp[k:hi])))
            if prom >= 0.015:
                n_pk += 1; pk_ms.append(round(float(t_ms[k]), 1))
    ons_mask = t_ms <= 150.0 + 1e-9
    out = {
        "suite": "sled", "g": g_level, "u": urgency, "noise": bool(spec["sensor_noise"]), "seed": int(spec["seed"]),
        "aborted": False, "settle_deg": round(float(f_arr[idx0]), 2),
        "peak_flex_deg": round(float(np.max(f_imp)), 2),
        "min_ons_deg": round(float(np.min(f_imp[ons_mask])), 2),
        "peak_splen": round(float(np.max(sp_imp)), 4), "peak_scm": round(float(np.max(sc_imp)), 4),
        "n_pk": n_pk, "pk_ms": pk_ms, "max_act": round(max(ma_l), 4), "wall_s": round(time.time()-t0, 1),
    }
    if not spec["sensor_noise"]:
        out["trace"] = {
            "t_ms": [round(float(v), 1) for v in t_ms[::2]],
            "flex_deg": [round(float(v), 2) for v in f_imp[::2]],
            "splen": [round(float(v), 4) for v in sp_imp[::2]],
            "scm": [round(float(v), 4) for v in sc_imp[::2]],
        }
    return out


def run_rpy_18s(spec: dict) -> dict:
    t0 = time.time()
    ov = dict(spec["params"])
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0, "Kp_trans_roll": 0.0, "Kp_trans_pitch": 0.0,
        "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0, "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0,
        "shift_station_blend": 0.0, "trajectory_tau": 0.28, "urgency": 0.0,
        "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
        "enable_sensor_noise": bool(spec["sensor_noise"]), "enable_motor_noise": bool(spec["motor_noise"]),
        "noise_seed": int(spec["seed"]), "noise_scale": float(spec["noise_scale"]),
    })
    model, controller, cy1, _ = _build_model_and_ctrl(ov)
    state = model.initSystem(); cy1.setIsPrescribed(state, False)
    a3._init_neck_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    p_dp = controller.updPropertyByName("desired_pitch")
    p_dr = controller.updPropertyByName("desired_roll")
    p_dy = controller.updPropertyByName("desired_yaw")
    p_u = controller.updPropertyByName("urgency")
    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    cr1, cr2 = cs.get("roll1"), cs.get("roll2")
    cy_1, cy_2 = cs.get("yaw1"), cs.get("yaw2")

    ts, ps, rs, ys = [], [], [], []
    cur_ph = -1; stop_reason = None
    t_now = 0.0
    while t_now < 18.0 - 1e-9:
        ph = next(p for p in PHASES if p["t_start"] <= t_now + 1e-9 < p["t_end"] or (p["id"] == 9 and t_now >= p["t_start"]))
        if ph["id"] != cur_ph:
            osim.PropertyHelper.setValueDouble(math.radians(ph["pitch_deg"]), p_dp)
            osim.PropertyHelper.setValueDouble(math.radians(ph["roll_deg"]), p_dr)
            osim.PropertyHelper.setValueDouble(math.radians(ph["yaw_deg"]), p_dy)
            osim.PropertyHelper.setValueDouble(float(ph["urgency"]), p_u)
            cur_ph = ph["id"]
        t_now = round(min(t_now + 0.010, 18.0), 4)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]; break
        ts.append(t_now)
        ps.append(math.degrees(cp1.getValue(state) + cp2.getValue(state)))
        rs.append(math.degrees(cr1.getValue(state) + cr2.getValue(state)))
        ys.append(math.degrees(cy_1.getValue(state) + cy_2.getValue(state)))

    t_arr = np.asarray(ts); arrs = [np.asarray(ps), np.asarray(rs), np.asarray(ys)]
    ph_summary = []
    for ph in PHASES:
        msk = (t_arr >= ph["t_start"]) & (t_arr <= ph["t_end"])
        if not np.any(msk): continue
        pa = ph["primary_axis"]
        if pa is None:
            ph_summary.append({"name": ph["name"], "max_drift_deg": round(float(max(np.max(np.abs(a[msk])) for a in arrs)), 2)})
        else:
            ai = AXIS_INDEX[pa]
            ref_v = ph[f"{pa}_deg"]
            vals = arrs[ai][msk]
            ovs, ovs_pct = compute_overshoot(vals, ref_v, float(vals[0]))
            t95 = compute_t95(t_arr[msk], vals, ph["t_start"], ref_v, float(vals[0]))
            ph_summary.append({"name": ph["name"], "target": ref_v, "end": round(float(vals[-1]), 2), "ovs_deg": ovs, "t95_s": t95})

    out = {
        "suite": "rpy_18s", "noise": bool(spec["sensor_noise"]), "seed": int(spec["seed"]),
        "aborted": stop_reason is not None, "stop_reason": stop_reason, "t_end": round(float(t_arr[-1]) if len(t_arr) else 0.0, 2),
        "phases": ph_summary, "wall_s": round(time.time()-t0, 1),
    }
    if not spec["sensor_noise"]:
        out["trace"] = {"t": [round(float(v), 2) for v in t_arr[::2]], "pitch": [round(float(v), 2) for v in arrs[0][::2]],
                        "roll": [round(float(v), 2) for v in arrs[1][::2]], "yaw": [round(float(v), 2) for v in arrs[2][::2]]}
    return out


def run_quiet_stance(spec: dict) -> dict:
    t0 = time.time()
    ov = dict(spec["params"])
    ov.update({
        "urgency": 0.0, "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
        "enable_sensor_noise": bool(spec["sensor_noise"]), "enable_motor_noise": bool(spec["motor_noise"]),
        "noise_seed": int(spec["seed"]), "noise_scale": float(spec["noise_scale"]),
    })
    model, _, cy1, _ = _build_model_and_ctrl(ov)
    state = model.initSystem(); cy1.setIsPrescribed(state, False)
    a3._init_neck_state(model, state, ov)
    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)
    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    cr1, cr2 = cs.get("roll1"), cs.get("roll2")
    cy_1, cy_2 = cs.get("yaw1"), cs.get("yaw2")
    ps, rs, ys = [], [], []
    t_now = 0.0; stop_reason = None
    while t_now < 5.0 - 1e-9:
        t_now = round(min(t_now + 0.010, 5.0), 4)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]; break
        if t_now >= 1.0:
            ps.append(math.degrees(cp1.getValue(state) + cp2.getValue(state)))
            rs.append(math.degrees(cr1.getValue(state) + cr2.getValue(state)))
            ys.append(math.degrees(cy_1.getValue(state) + cy_2.getValue(state)))
    pa, ra, ya = np.asarray(ps), np.asarray(rs), np.asarray(ys)
    return {
        "suite": "quiet", "tag": spec["tag"], "seed": int(spec["seed"]),
        "aborted": stop_reason is not None, "stop_reason": stop_reason,
        "pitch_std_deg": round(float(np.std(pa)), 4) if len(pa) else 0.0,
        "roll_std_deg": round(float(np.std(ra)), 4) if len(ra) else 0.0,
        "yaw_std_deg": round(float(np.std(ya)), 4) if len(ya) else 0.0,
        "max_abs_deg": round(float(max(np.max(np.abs(pa)), np.max(np.abs(ra)), np.max(np.abs(ya)))), 3) if len(pa) else 99.0,
        "wall_s": round(time.time()-t0, 1),
    }


def run_shift_case(spec: dict) -> dict:
    t0 = time.time()
    ov = dict(spec["params"])
    ov.update({
        "urgency": 0.0, "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
        "desired_ap_shift": 0.0, "desired_lat_shift": 0.0,
        "enable_sensor_noise": False, "enable_motor_noise": False,
    })
    model, controller, cy1, _ = _build_model_and_ctrl(ov)
    state = model.initSystem(); cy1.setIsPrescribed(state, False)
    a3._init_neck_state(model, state, ov)
    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    c1_body = model.getBodySet().get("cerv1")
    spine_body = model.getBodySet().get("spine")
    model.realizePosition(state)
    p0 = c1_body.getPositionInGround(state)
    s0 = spine_body.getPositionInGround(state)
    x0_rel = float(p0.get(0) - s0.get(0)); z0_rel = float(p0.get(2) - s0.get(2))

    cp1, cp2 = model.getCoordinateSet().get("pitch1"), model.getCoordinateSet().get("pitch2")
    cr1, cr2 = model.getCoordinateSet().get("roll1"), model.getCoordinateSet().get("roll2")
    step_done = False; t_now = 0.0; stop_reason = None
    xs, zs, ps, rs = [], [], [], []
    while t_now < 4.5 - 1e-9:
        if not step_done and t_now >= 0.499:
            for k, v in spec["step_ov"].items():
                osim.PropertyHelper.setValueDouble(float(v), controller.updPropertyByName(k))
            step_done = True
        t_now = round(min(t_now + 0.010, 4.5), 4)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]; break
        model.realizePosition(state)
        pc = c1_body.getPositionInGround(state); ps_b = spine_body.getPositionInGround(state)
        if t_now >= 3.5:
            xs.append(100.0 * (float(pc.get(0) - ps_b.get(0)) - x0_rel))
            zs.append(100.0 * (float(pc.get(2) - ps_b.get(2)) - z0_rel))
            ps.append(math.degrees(cp1.getValue(state) + cp2.getValue(state)))
            rs.append(math.degrees(cr1.getValue(state) + cr2.getValue(state)))
    return {
        "suite": "shift_5dof", "name": spec["name"], "aborted": stop_reason is not None, "stop_reason": stop_reason,
        "ap_cm": round(float(np.mean(xs)), 3) if xs else 0.0,
        "lat_cm": round(float(np.mean(zs)), 3) if zs else 0.0,
        "pitch_deg": round(float(np.mean(ps)), 2) if ps else 0.0,
        "roll_deg": round(float(np.mean(rs)), 2) if rs else 0.0,
        "wall_s": round(time.time()-t0, 1),
    }


def _dispatch_a4(spec: dict) -> dict:
    try:
        s = spec["suite"]
        if s == "bode": return run_bode_detailed(spec)
        if s == "sled": return run_sled_detailed(spec)
        if s == "rpy_18s": return run_rpy_18s(spec)
        if s == "quiet": return run_quiet_stance(spec)
        if s == "shift_5dof": return run_shift_case(spec)
        if s == "vol60": return {"suite": "vol60", **a3ex.run_vol60_exact(spec["params"])}
        raise ValueError(s)
    except Exception as e:
        return {"suite": spec["suite"], "aborted": True, "error": repr(e)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=48)
    ap.add_argument("--source", choices=["phase_a3", "phase_a3_exact"], default="phase_a3_exact")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    t0 = time.time()
    if args.source == "phase_a3_exact":
        _, base_p = a3ex.load_base_params()
        merged = a3ex.merge_shared_params(base_p, exclude_axis="")
        out_path = args.out or os.path.join(_ROOT, "logs", "phase_a3_exact", "phase_a4_exact_full_verification.json")
    else:
        _, base_p = a3.load_base_params()
        merged = a3.merge_shared_params(base_p, exclude_axis="")
        out_path = args.out or OUT_JSON

    tasks = []
    # Suite 1: 3-axis Bode Noise-OFF + 3-Seed Noise-ON (12 sims)
    for mode in ("APEO", "LatEC", "YawMA"):
        tasks.append({"suite": "bode", "mode": mode, "sensor_noise": False, "motor_noise": False, "seed": 0, "noise_scale": 0.0, "params": merged})
        for seed in (1, 2, 3):
            tasks.append({"suite": "bode", "mode": mode, "sensor_noise": True, "motor_noise": True, "seed": seed, "noise_scale": 1.0, "params": merged})

    # Suite 2: Zheng 2g/3g 16-case Noise-OFF + 18 Noise-ON (34 sims)
    for g in (2.0, 3.0):
        for u in (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0):
            tasks.append({"suite": "sled", "g": g, "u": u, "sensor_noise": False, "motor_noise": False, "seed": 0, "noise_scale": 0.0, "params": merged})
        for u in (-0.75, 0.0, 0.5):
            for seed in (1, 2, 3):
                tasks.append({"suite": "sled", "g": g, "u": u, "sensor_noise": True, "motor_noise": True, "seed": seed, "noise_scale": 1.0, "params": merged})

    # Suite 3: 18s Voluntary 50deg RPY (Noise-OFF + Seed 1 Noise-ON) (2 sims)
    tasks.append({"suite": "rpy_18s", "sensor_noise": False, "motor_noise": False, "seed": 0, "noise_scale": 0.0, "params": merged})
    tasks.append({"suite": "rpy_18s", "sensor_noise": True, "motor_noise": True, "seed": 1, "noise_scale": 1.0, "params": merged})

    # Suite 4: Quiet stance 2x2 ablation (10 sims)
    tasks.append({"suite": "quiet", "tag": "OFF", "sensor_noise": False, "motor_noise": False, "seed": 0, "noise_scale": 0.0, "params": merged})
    for tag, sn, mn in (("Sensor", True, False), ("Motor", False, True), ("Both", True, True)):
        for seed in (1, 2, 3):
            tasks.append({"suite": "quiet", "tag": tag, "sensor_noise": sn, "motor_noise": mn, "seed": seed, "noise_scale": 1.0, "params": merged})

    # Suite 5: 5-DOF Voluntary AP/Lat Shift + VOL60 (5 sims)
    tasks.append({"suite": "shift_5dof", "name": "Case1_AP_Protract_2cm", "step_ov": {"desired_ap_shift": 0.020, "urgency": 0.5}, "params": merged})
    tasks.append({"suite": "shift_5dof", "name": "Case2_AP_Retract_1p5cm", "step_ov": {"desired_ap_shift": -0.015, "urgency": 0.5}, "params": merged})
    tasks.append({"suite": "shift_5dof", "name": "Case3_Lat_Shift_0p5cm", "step_ov": {"desired_lat_shift": 0.005, "urgency": 0.5}, "params": merged})
    tasks.append({"suite": "shift_5dof", "name": "Case4_Combined_AP_Pitch", "step_ov": {"desired_ap_shift": 0.015, "desired_pitch": math.radians(15.0), "urgency": 0.5}, "params": merged})
    tasks.append({"suite": "vol60", "params": merged})

    print(f"[PHASE-A4] Dispatching {len(tasks)} verification simulations across {args.workers} workers...", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        results = list(ex.map(_dispatch_a4, tasks))

    payload = {
        "verified_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "wall_s": round(time.time() - t0, 1),
        "n_total": len(results),
        "n_aborted": sum(1 for r in results if r.get("aborted")),
        "params": merged,
        "results": results,
    }
    a3.atomic_write_json(out_path, payload)
    print(f"[PHASE-A4] COMPLETED {len(results)} sims (aborted={payload['n_aborted']}) in {payload['wall_s']}s -> {out_path}", flush=True)


if __name__ == "__main__":
    main()
