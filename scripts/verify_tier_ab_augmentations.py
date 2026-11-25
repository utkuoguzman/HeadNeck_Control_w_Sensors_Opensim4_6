"""
Step 4 Verification & Ablation Matrix (Plan Cases 0-5) -> writes scratch/tier_ab_checkpoint1.json
for scheduled Planner Review Checkpoint before surgical joint optimization.
"""

import os
import sys
import math
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_k, "1")

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
import scripts.generate_standard_plot as gsp
import scripts.run_zheng_2g_3g_urgency_sweep as rz

DETAILED_OSIM = gsp.MODEL_PATH
LIGAMENTS_OSIM = os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")
CHAMP_JSON = os.path.join(_ROOT, "logs", "se2_c1pos_3axis_champion.json")
OUT_JSON = r"C:\Users\User\.gemini\antigravity\brain\89fb8d56-9d12-4051-a256-02497ef8d6ae\scratch\tier_ab_kuo_checkpoint.json"


def _load_champ():
    with open(CHAMP_JSON, "r", encoding="utf-8") as f:
        doc = json.load(f)
    p = dict(doc["params"])
    p.update(doc.get("se2_c1pos_params", {}))
    return p


def _apply_dict_overrides(controller, ov):
    pairs = []
    for k, v in ov.items():
        if not controller.hasProperty(k):
            continue
        if isinstance(v, bool):
            pairs.append((k, "true" if v else "false"))
        elif isinstance(v, str):
            pairs.append((k, v))
        else:
            pairs.append((k, repr(float(v))))
    return gsp.apply_cli_overrides_and_inspect(controller, pairs)


def _init_muscles_and_controller_state(model, state, ov, enable_gto_reflex=False):
    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    q = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q.get(idx))
    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral", "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    k_gdyn = float(ov.get("K_gamma_dyn", 0.857744))
    k_gstat = float(ov.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        nm = m.getName()
        if nm.startswith("r_") or nm.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = gsp.PER_MUSCLE_72_U0.get(nm, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gdyn * (0.25 + 0.75 * a0)
        gs0 = 100.0 * k_gstat * a0
        fd0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fs0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fd0)
        m.setStateVariableValue(state, "spindle/static_activation", fs0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{nm}", fd0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{nm}", fs0)
    model.equilibrateMuscles(state)


def _run_bode_worker(task):
    gsp.ensure_libraries_loaded()
    model_path = task["model_path"]
    mode = task["mode"]
    ov = dict(_load_champ())
    ov.update(task.get("overrides", {}))

    model = osim.Model(model_path)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            controller = c
    bool_state, false_bools, _ = _apply_dict_overrides(controller, ov)

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0); c.setDefaultLocked(True)

    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False); c.setDefaultIsPrescribed(True); c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False); c_yaw1.setDefaultIsPrescribed(False)

    delay, ramp_duration, pert_duration = 0.8, 1.5, 15.0
    target_time = delay + pert_duration

    if mode == "APEO":
        ordered = sorted(gsp.CANONICAL_TARGETS, key=lambda x: x[0])
        freqs = np.array([t[0] for t in ordered], dtype=float)
        tgt_gains = np.array([t[1] for t in ordered], dtype=float)
        tgt_phases = np.array([t[2] for t in ordered], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
        amps_sim = amps_in.copy()
        prescribed_coord_name = "spine_tx"
        ms_spline, _ = gsp.build_c2_multisine_spline(freqs, amps_sim, phases, delay, ramp_duration, target_time)
    elif mode == "LatEC":
        df_lat, _ = gsp.load_aligned_data()
        freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(gsp.get_latec_amplitudes(freqs), dtype=float)
        amps_sim = amps_in.copy()
        prescribed_coord_name = "spine_tz"
        ms_spline, _ = gsp.build_c2_multisine_spline(freqs, amps_sim, phases, delay, ramp_duration, target_time)
    else:
        _, df_yaw = gsp.load_aligned_data()
        freqs = np.array(df_yaw["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_yaw["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.array(df_yaw["Exp_Yaw_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(gsp.get_yawma_amplitudes_deg(freqs), dtype=float)
        amps_sim = np.radians(amps_in)
        prescribed_coord_name = "gndyaw"
        ms_spline, _ = gsp.build_c2_zero_offset_spline_yaw(freqs, amps_sim, phases, delay, ramp_duration, target_time)

    coord_pert = model.updCoordinateSet().get(prescribed_coord_name)
    coord_pert.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)
    _init_muscles_and_controller_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    cr1, cr2 = cs.get("roll1"), cs.get("roll2")
    cy1, cy2 = cs.get("yaw1"), cs.get("yaw2")
    cgndyaw = cs.get("gndyaw")

    t_list, p_list, r_list, yrel_list, yglob_list = [], [], [], [], []
    aborted = False; abort_time_s = float(target_time); abort_reason = ""
    t_now = 0.0; step = 0.005
    while t_now < target_time:
        t_now = min(t_now + step, target_time)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            aborted = True; abort_time_s = t_now; abort_reason = str(e).splitlines()[0]
            break
        p1d = math.degrees(cp1.getValue(state)); p2d = math.degrees(cp2.getValue(state))
        r1d = math.degrees(cr1.getValue(state)); r2d = math.degrees(cr2.getValue(state))
        y1d = math.degrees(cy1.getValue(state)); y2d = math.degrees(cy2.getValue(state))
        pd = p1d + p2d; rd = r1d + r2d; yrd = y1d + y2d
        ygd = math.degrees(cgndyaw.getValue(state)) + yrd
        t_list.append(t_now); p_list.append(pd); r_list.append(rd); yrel_list.append(yrd); yglob_list.append(ygd)
        if mode == "APEO":
            if (p1d < -12.0 or p1d > 18.0 or p2d < -24.75 or p2d > 36.0 or pd < -36.75 or pd > 54.0
                or abs(r1d) > 4.5 or abs(r2d) > 24.75 or abs(rd) > 29.25
                or abs(y1d) > 28.5 or abs(y2d) > 20.25 or abs(yrd) > 48.75):
                aborted = True; abort_time_s = t_now; abort_reason = f"t={t_now:.2f}s p={pd:+.2f}"
                break
        else:
            if abs(pd) > 45.0 or abs(rd) > 35.0 or abs(yrd) > 48.0:
                aborted = True; abort_time_s = t_now; abort_reason = f"t={t_now:.2f}s r={rd:+.2f}"
                break

    t_arr = np.array(t_list, dtype=float)
    p_arr = np.array(p_list, dtype=float); r_arr = np.array(r_list, dtype=float)
    yr_arr = np.array(yrel_list, dtype=float); yg_arr = np.array(yglob_list, dtype=float)
    main_arr = p_arr if mode == "APEO" else (-r_arr if mode == "LatEC" else yg_arr)

    m_post = t_arr >= 5.0
    n_post = int(np.sum(m_post))
    if aborted or n_post < 50:
        prog = max(0.0, min(1.0, abort_time_s / target_time))
        cls = 6.0 * (200000.0 + (1.0 - prog) * 50000.0)
        return {"case": task["case"], "kind": "bode", "mode": mode, "aborted": True, "abort_reason": abort_reason,
                "total_score": round(cls, 2), "classic_score": round(cls, 2), "false_bools": false_bools}

    t_fit = t_arr[m_post]; t_rel = t_fit - delay; y_fit = main_arr[m_post]
    cols = [np.ones_like(t_fit), t_fit - 5.0]
    for k in range(K):
        psi_k = 2.0 * math.pi * float(freqs[k]) * t_rel + float(phases[k])
        cols.append(np.sin(psi_k)); cols.append(np.cos(psi_k))
    coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit, rcond=None)
    ref_fit = np.zeros_like(t_fit); harm_fit = np.zeros_like(t_fit)
    g_loss = 0.0; p_loss = 0.0; f_summary = {}
    for k in range(K):
        fk = float(freqs[k]); psi = 2.0 * np.pi * fk * t_rel + float(phases[k])
        ref_fit += float(amps_in[k]) * float(tgt_gains[k]) * np.sin(psi + math.radians(float(tgt_phases[k])))
        asin_c, bcos_c = float(coeffs[2 + 2*k]), float(coeffs[2 + 2*k + 1])
        harm_fit += asin_c * np.sin(psi) + bcos_c * np.cos(psi)
        sg = math.hypot(asin_c, bcos_c) / float(amps_in[k])
        rph = math.degrees(math.atan2(bcos_c, asin_c))
        tp = float(tgt_phases[k])
        ph_err = ((rph - tp + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(sg, 1e-6) / float(tgt_gains[k]))
        g_pts = 100.0 * db_err * db_err; p_pts = ph_err * ph_err
        g_loss += g_pts; p_loss += p_pts
        f_summary[f"{fk:.3f}"] = {"db": round(db_err, 2), "ph_err": round(ph_err, 1)}

    vel_act = np.gradient(y_fit, t_fit); vel_hrm = np.gradient(harm_fit, t_fit); vel_ref = np.gradient(ref_fit, t_fit)
    hf_rms = float(np.sqrt(np.mean((vel_act - vel_hrm)**2)))
    rv = float(np.corrcoef(vel_act, vel_ref)[0, 1]) if np.std(vel_act) > 1e-6 else 0.0
    p_dev = float(np.max(np.abs(p_arr[m_post] - p_arr[0])))
    r_dev = float(np.max(np.abs(r_arr[m_post])))
    y_dev = float(np.max(np.abs(yr_arr[m_post])))
    if mode == "APEO":
        xt = 100.0 * (r_dev**2 + y_dev**2); hf_p = 5.0 * (hf_rms**2)
    elif mode == "LatEC":
        xt = 100.0 * (p_dev**2 + y_dev**2); hf_p = 25.0 * (hf_rms**2)
    else:
        xt = 100.0 * (p_dev**2 + r_dev**2); hf_p = 25.0 * (hf_rms**2)
    cls = g_loss + p_loss; tot = cls + xt + hf_p
    return {
        "case": task["case"], "kind": "bode", "mode": mode, "aborted": False,
        "classic_score": round(cls, 2), "gain_loss": round(g_loss, 2), "phase_loss": round(p_loss, 2),
        "xtalk_pen": round(xt, 2), "hf_pen": round(hf_p, 2), "total_score": round(tot, 2),
        "corr_vel": round(rv, 4), "false_bools": false_bools,
        "hf_bins": {k: v for k, v in f_summary.items() if float(k) >= 2.0} if mode == "APEO" else {},
    }


def _run_sled_worker(task):
    gsp.ensure_libraries_loaded()
    model_path = task["model_path"]
    g_level = task["g_level"]; urgency = task["urgency"]
    ov = dict(_load_champ())
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0,
        "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0,
        "shift_station_blend": 0.0, "urgency": urgency,
    })
    ov.update(task.get("overrides", {}))

    model = osim.Model(model_path)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            controller = c
    _apply_dict_overrides(controller, ov)

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0); c.setDefaultLocked(True)

    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False); c.setDefaultIsPrescribed(True); c.setPrescribedFunction(zero_func)
    sp_tx = rz.build_sled_spline(g_level, sign_x=-1.0)
    ctx = model.updCoordinateSet().get("spine_tx"); ctx.setPrescribedFunction(sp_tx)
    c_yaw1 = model.updCoordinateSet().get("yaw1"); c_yaw1.set_locked(False); c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False); ctx.setSpeedValue(state, 0.0)
    _init_muscles_and_controller_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    muscles = model.updMuscles()
    splen_idx = [i for i in range(muscles.getSize()) if muscles.get(i).getName() in rz.SPLEN_CAP_NAMES]
    neck_idx = [i for i in range(muscles.getSize()) if not (muscles.get(i).getName().startswith("r_") or muscles.get(i).getName().startswith("l_"))]
    fset = model.getForceSet()
    lig_forces = [fset.get(i) for i in range(fset.getSize()) if fset.get(i).getConcreteClassName() == "DynamicLigamentForce"]
    lig_out_strain = [lf.getOutput("strain") for lf in lig_forces]
    lig_out_ten = [lf.getOutput("tension") for lf in lig_forces]

    sv_names = model.getStateVariableNames()
    ib_sv_names = []
    if ov.get("enable_gto_reflex", False):
        for idx in neck_idx:
            sn = f"/controllerset/VCR_CCR_Controller/ib_state_{muscles.get(idx).getName()}"
            if sv_names.findIndex(sn) >= 0:
                ib_sv_names.append(sn)

    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    ts_l, f_l, sp_l, mact_l = [], [], [], []
    max_lig_eps, max_lig_ten, max_ib_state = 0.0, 0.0, 0.0
    t_now = 0.0; stop_reason = None
    while t_now < rz.T_TOTAL - 1e-9:
        t_next = round(min(t_now + rz.DT_LOG, rz.T_TOTAL), 6)
        try:
            state = mgr.integrate(t_next)
        except Exception as e:
            stop_reason = f"Stopped at {t_next:.4f}s: {str(e).splitlines()[0]}"
            break
        t_now = t_next
        model.realizeDynamics(state)
        ts_l.append(t_now)
        f_l.append(-math.degrees(cp1.getValue(state) + cp2.getValue(state)))
        sp_l.append(sum(float(muscles.get(idx).getActivation(state)) for idx in splen_idx) / len(splen_idx))
        mact_l.append(max(float(muscles.get(idx).getActivation(state)) for idx in neck_idx))
        if ib_sv_names and t_now >= rz.T_SETTLE:
            for sn in ib_sv_names:
                v_ib = abs(float(model.getStateVariableValue(state, sn)))
                if v_ib > max_ib_state: max_ib_state = v_ib
        if lig_out_strain and t_now >= rz.T_SETTLE:
            for o_eps, o_ten in zip(lig_out_strain, lig_out_ten):
                v_lam = float(o_eps.getValueAsString(state))
                v_eps = max(0.0, v_lam - 1.0)
                v_ten = float(o_ten.getValueAsString(state))
                if v_eps > max_lig_eps: max_lig_eps = v_eps
                if v_ten > max_lig_ten: max_lig_ten = v_ten

    ts = np.asarray(ts_l); f_arr = np.asarray(f_l); sp_arr = np.asarray(sp_l)
    imp = ts >= (rz.T_SETTLE - 1e-9)
    idx0 = int(np.where(imp)[0][0])
    f_settle = float(f_arr[idx0])
    f_rel = f_arr[imp] - f_settle
    t_imp_ms = (ts[imp] - rz.T_SETTLE) * 1000.0
    sp_imp = sp_arr[imp]

    # Exact ground-truth peak counter from run_zheng_2g_3g_urgency_sweep.py L378-386
    win_half = 25
    n_pk = 0; pk_times = []; pk_vals = []
    for k in range(1, len(sp_imp) - 1):
        if sp_imp[k] > 0.08 and sp_imp[k] > sp_imp[k - 1] and sp_imp[k] >= sp_imp[k + 1]:
            lo = max(0, k - win_half); hi = min(len(sp_imp), k + win_half + 1)
            prom = float(sp_imp[k] - max(np.min(sp_imp[lo:k + 1]), np.min(sp_imp[k:hi])))
            if prom >= 0.015:
                n_pk += 1; pk_times.append(round(float(t_imp_ms[k]), 1)); pk_vals.append(round(float(sp_imp[k]), 4))

    ons_mask = t_imp_ms <= 150.0 + 1e-9
    return {
        "case": task["case"], "kind": "sled", "g": g_level, "u": urgency, "stop_reason": stop_reason,
        "settle_deg": round(f_settle, 2),
        "peak_flex_deg": round(float(np.max(f_rel)), 2),
        "min_flex_ons_deg": round(float(np.min(f_rel[ons_mask])), 2),
        "t_peak_ms": round(float(t_imp_ms[int(np.argmax(f_rel))]), 1),
        "peak_splen": round(float(np.max(sp_imp)), 4),
        "t_peak_splen_ms": round(float(t_imp_ms[int(np.argmax(sp_imp))]), 1),
        "n_splen_peaks": n_pk, "pk_times_ms": pk_times, "pk_vals": pk_vals,
        "max_act": round(max(mact_l), 4) if mact_l else 0.0,
        "max_ib_state_ips": round(max_ib_state, 2),
        "max_lig_eff_strain_pct": round(100.0 * max_lig_eps, 2),
        "max_lig_tension_N": round(max_lig_ten, 1),
    }


def _run_voluntary_pitch_worker(task):
    gsp.ensure_libraries_loaded()
    ov = dict(_load_champ())
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0, "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0, "shift_station_blend": 0.0,
        "urgency": 0.0, "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
    })
    ov.update(task.get("overrides", {}))
    model = osim.Model(task["model_path"])
    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController": c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController": controller = c
    _apply_dict_overrides(controller, ov)
    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn); c.set_locked(False); c.setDefaultIsPrescribed(True); c.setPrescribedFunction(zero_func)
    cy1 = model.updCoordinateSet().get("yaw1"); cy1.set_locked(False); cy1.setDefaultIsPrescribed(False)
    state = model.initSystem(); cy1.setIsPrescribed(state, False)
    _init_muscles_and_controller_state(model, state, ov)

    mgr = osim.Manager(model); mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    cp1, cp2 = model.getCoordinateSet().get("pitch1"), model.getCoordinateSet().get("pitch2")
    prop_dp = controller.updPropertyByName("desired_pitch")
    prop_u = controller.updPropertyByName("urgency")
    step_done = False; ts, ps = [], []
    t_now = 0.0; stop_reason = None
    while t_now < 2.8 - 1e-9:
        if not step_done and t_now >= 0.499:
            osim.PropertyHelper.setValueDouble(math.radians(60.0), prop_dp)
            osim.PropertyHelper.setValueDouble(1.0, prop_u)
            step_done = True
        t_now = round(min(t_now + 0.005, 2.8), 4)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]; break
        ts.append(t_now); ps.append(math.degrees(cp1.getValue(state) + cp2.getValue(state)))
    pa = np.asarray(ps)
    return {
        "case": task["case"], "kind": "vol_pitch60", "stop_reason": stop_reason,
        "end_pitch_deg": round(float(pa[-1]), 2), "max_pitch_deg": round(float(np.max(pa)), 2),
        "ovs_deg": round(float(max(0.0, np.max(pa) - 60.0)), 2),
    }


def _dispatch(task):
    try:
        if task["kind"] == "bode": return _run_bode_worker(task)
        if task["kind"] == "sled": return _run_sled_worker(task)
        return _run_voluntary_pitch_worker(task)
    except Exception as e:
        return {
            "case": task["case"], "kind": task["kind"], "mode": task.get("mode", ""),
            "g": task.get("g_level", 0), "u": task.get("urgency", 0.0),
            "aborted": True, "stop_reason": repr(e), "error": repr(e),
        }


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", nargs="*", default=None, help="Optional list of case names to run")
    ap.add_argument("--out", default=OUT_JSON, help="Output JSON path")
    args = ap.parse_args()

    t0 = time.time()
    tasks = []

    # Case 0: Clean C1 regression on untouched Detailed.osim (3-axis Bode + 2g/3g sled)
    for mode in ("APEO", "LatEC", "YawMA"):
        tasks.append({"case": "C0_CleanC1", "kind": "bode", "model_path": DETAILED_OSIM, "mode": mode, "overrides": {}})
    for g in (2, 3):
        for u in (-0.75, 0.0, 0.5):
            tasks.append({"case": "C0_CleanC1", "kind": "sled", "model_path": DETAILED_OSIM, "g_level": g, "urgency": u, "overrides": {}})

    # Case 1: Kuo67 + B_k50_c30 Calibrated Passive Plant Only (3-axis Bode + 2g/3g sled + vol60; all 3 reflex toggles False)
    ov1 = {"enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": False}
    for mode in ("APEO", "LatEC", "YawMA"):
        tasks.append({"case": "C1_KuoCal_PassiveOnly", "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": mode, "overrides": ov1})
    for g in (2, 3):
        for u in (-0.75, 0.0, 0.5):
            tasks.append({"case": "C1_KuoCal_PassiveOnly", "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": u, "overrides": ov1})
    tasks.append({"case": "C1_KuoCal_PassiveOnly", "kind": "vol", "model_path": LIGAMENTS_OSIM, "overrides": ov1})

    # Case 2: GTO Ib Only (with Reference GTO efference cancellation)
    ov2 = {"enable_gto_reflex": True, "enable_renshaw": False, "enable_ligament_reflex": False}
    tasks.append({"case": "C2_GTO_Only", "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": "APEO", "overrides": ov2})
    for g, u in ((2, -0.75), (2, 0.0), (2, 0.5), (3, 0.0)):
        tasks.append({"case": "C2_GTO_Only", "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": u, "overrides": ov2})
    tasks.append({"case": "C2_GTO_Only", "kind": "vol", "model_path": LIGAMENTS_OSIM, "overrides": ov2})

    # Case 3a (un-damped) & 3b (Renshaw Only) with highg canal detuning removed
    ov3a = {"highg_p_sc_floor": 60.0, "highg_tau_lead_add_s": 0.0, "enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": False}
    ov3b = {"highg_p_sc_floor": 60.0, "highg_tau_lead_add_s": 0.0, "enable_gto_reflex": False, "enable_renshaw": True, "enable_ligament_reflex": False}
    for g in (2, 3):
        tasks.append({"case": "C3a_NoCanalFloor_NoRenshaw", "kind": "sled", "model_path": DETAILED_OSIM, "g_level": g, "urgency": 0.0, "overrides": ov3a})
        tasks.append({"case": "C3b_Renshaw_Only", "kind": "sled", "model_path": DETAILED_OSIM, "g_level": g, "urgency": 0.0, "overrides": ov3b})
    tasks.append({"case": "C3b_Renshaw_Only", "kind": "bode", "model_path": DETAILED_OSIM, "mode": "APEO", "overrides": ov3b})

    # Case 3b on calibrated Kuo+SRS model
    for g in (2, 3):
        tasks.append({"case": "C3b_Renshaw_KuoCal", "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": 0.0, "overrides": ov3b})
    tasks.append({"case": "C3b_Renshaw_KuoCal", "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": "APEO", "overrides": ov3b})
    tasks.append({"case": "C3b_Renshaw_KuoCal", "kind": "vol", "model_path": LIGAMENTS_OSIM, "overrides": ov3b})

    # Case 234: All Tier B combined (SRS + Real GTO + Renshaw + Ligament) on calibrated Kuo model, baseline C1 vestibular
    ov234 = {"enable_gto_reflex": True, "enable_renshaw": True, "enable_ligament_reflex": True}
    for mode in ("APEO", "LatEC", "YawMA"):
        tasks.append({"case": "C234_All_TierB", "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": mode, "overrides": ov234})
    for g in (2, 3):
        for u in (-0.75, 0.0, 0.5):
            tasks.append({"case": "C234_All_TierB", "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": u, "overrides": ov234})
    tasks.append({"case": "C234_All_TierB", "kind": "vol", "model_path": LIGAMENTS_OSIM, "overrides": ov234})

    # Case 4: Ligament/Capsule Reflex Only
    ov4 = {"enable_gto_reflex": False, "enable_renshaw": False, "enable_ligament_reflex": True}
    tasks.append({"case": "C4_Ligament_Only", "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": "APEO", "overrides": ov4})
    for g, u in ((2, -0.75), (2, 0.0), (3, -0.75), (3, 0.0)):
        tasks.append({"case": "C4_Ligament_Only", "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": u, "overrides": ov4})
    tasks.append({"case": "C4_Ligament_Only", "kind": "vol", "model_path": LIGAMENTS_OSIM, "overrides": ov4})

    # Case 5: Combined Tier A (z_phas=0, single peripheral LN tanh, no highg cliffs) + Tier B All ON
    for sat in (4.5, 6.0, 8.0):
        cname = f"C5_Comb_tanh{sat}"
        ov5 = {
            "z_phas": 0.0, "phasic_oto_tanh_mps2": sat, "phasic_oto_sat_mps2": 0.0,
            "highg_oto_knee_mps2": 0.0, "phasic_torque_sat_nm": 0.0,
            "highg_p_sc_floor": 60.0, "highg_tau_lead_add_s": 0.0,
            "enable_gto_reflex": True, "enable_renshaw": True, "enable_ligament_reflex": True,
        }
        for mode in ("APEO", "LatEC", "YawMA"):
            tasks.append({"case": cname, "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": mode, "overrides": ov5})
        for g in (2, 3):
            for u in (-0.75, 0.0, 0.5):
                tasks.append({"case": cname, "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": u, "overrides": ov5})
        tasks.append({"case": cname, "kind": "vol", "model_path": LIGAMENTS_OSIM, "overrides": ov5})

    # Bounded physical K_phas=4.0 probe on Combined Tier A+B
    ov5k = {
        "z_phas": 0.0, "K_phas": 4.0, "phasic_oto_tanh_mps2": 6.0, "phasic_oto_sat_mps2": 0.0,
        "highg_oto_knee_mps2": 0.0, "phasic_torque_sat_nm": 0.0,
        "highg_p_sc_floor": 60.0, "highg_tau_lead_add_s": 0.0,
        "enable_gto_reflex": True, "enable_renshaw": True, "enable_ligament_reflex": True,
    }
    tasks.append({"case": "C5_Comb_Kphas4_tanh6", "kind": "bode", "model_path": LIGAMENTS_OSIM, "mode": "APEO", "overrides": ov5k})
    for g, u in ((2, -0.75), (2, 0.0), (3, -0.75), (3, 0.0)):
        tasks.append({"case": "C5_Comb_Kphas4_tanh6", "kind": "sled", "model_path": LIGAMENTS_OSIM, "g_level": g, "urgency": u, "overrides": ov5k})

    if args.cases:
        keep = set(args.cases)
        tasks = [t for t in tasks if t["case"] in keep]
    out_json = args.out

    os.makedirs(os.path.dirname(out_json), exist_ok=True)
    n_workers = min(10, max(1, len(tasks)))
    print(f"Dispatching {len(tasks)} simulations across {n_workers} workers...", flush=True)
    results_by_idx = [None] * len(tasks)
    completed = 0

    def _write_checkpoint():
        done_items = [r for r in results_by_idx if r is not None]
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump({"wall_s": round(time.time() - t0, 2), "completed": len(done_items), "total": len(tasks), "results": done_items}, f, indent=2)

    with ProcessPoolExecutor(max_workers=n_workers) as ex:
        fut_to_idx = {ex.submit(_dispatch, task): idx for idx, task in enumerate(tasks)}
        for fut in as_completed(fut_to_idx):
            idx = fut_to_idx[fut]
            results_by_idx[idx] = fut.result()
            completed += 1
            if completed % 5 == 0 or completed == len(tasks):
                print(f"  progress: {completed}/{len(tasks)} ({time.time() - t0:.1f}s)", flush=True)
                _write_checkpoint()

    results = results_by_idx
    print(f"Completed {len(results)} simulations in {time.time()-t0:.2f}s -> {out_json}", flush=True)
    for r in results:
        if "error" in r:
            print(f"  {r['case']:26s} {r['kind'].upper():5s} ERROR: {r['error']}")
        elif r["kind"] == "bode":
            print(f"  {r['case']:26s} BODE {r['mode']:5s}: tot={r.get('total_score')} (cls={r.get('classic_score')}, rv={r.get('corr_vel')}, abort={r.get('aborted')})")
        elif r["kind"] == "sled":
            print(f"  {r['case']:26s} SLED {r['g']}g u={r['u']:+.2f}: flex={r['peak_flex_deg']:+.2f} ons={r['min_flex_ons_deg']:+.2f} nPk={r['n_splen_peaks']} pks={r['pk_times_ms']} maxAct={r['max_act']:.3f} ib={r.get('max_ib_state_ips', 0.0):.1f} lig={r['max_lig_eff_strain_pct']:.1f}%/{r['max_lig_tension_N']:.0f}N")
        else:
            print(f"  {r['case']:26s} VOL60: end={r['end_pitch_deg']:+.2f} max={r['max_pitch_deg']:+.2f} ovs={r['ovs_deg']:.2f} err={r['stop_reason']}")


if __name__ == "__main__":
    main()
