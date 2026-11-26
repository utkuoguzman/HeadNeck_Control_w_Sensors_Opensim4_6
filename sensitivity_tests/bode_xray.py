"""Full-Model Bode Internal-Probe X-Ray Engine (100% Full Model — all subsystems ON).

At every experimental frequency of APEO, LatEC, or YawMA, decomposes:
  - Output kinematics: total angle, upper joint (q1), lower joint (q2), skull rel translation
  - Internal C++ probes: dbg_vcr_torque, dbg_ccr_torque, dbg_buckle_torque, dbg_qp_delivered_3dof,
    dbg_phasic_oto_input, dbg_canal_irr_head, mahony angle, cuneate angle, SRS thixotropy
  - Per-frequency Gain (dB, pts), Phase (deg, pts), classic_score, total_score, rv, hf_rms.
"""
from __future__ import annotations

import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

for _env_k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_env_k, "1")

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in (_REPO_ROOT, os.path.join(_REPO_ROOT, "scripts"), os.path.join(_REPO_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import opensim as osim
import scripts.truba_phase_a3_exact_roi_cmaes as a3ex


def _parse_vec3(s: str) -> Tuple[float, float, float]:
    # Format: ~[x,y,z]
    s = s.strip().lstrip("~").strip("[]()")
    parts = [float(x) for x in s.split(",")]
    return parts[0], parts[1], parts[2]


def _harmonic_fit(X: np.ndarray, y: np.ndarray, freqs: np.ndarray, amps_in: np.ndarray):
    c, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    gains, phases = [], []
    for k in range(len(freqs)):
        a_c, b_c = float(c[2 + 2 * k]), float(c[2 + 2 * k + 1])
        gains.append(math.hypot(a_c, b_c) / float(amps_in[k]))
        phases.append(math.degrees(math.atan2(b_c, a_c)))
    return c, np.asarray(gains, dtype=float), np.asarray(phases, dtype=float)


def run_bode_probed(
    mode: str,
    overrides: Dict[str, float],
    exact_15s: bool = False,
    bushing_k_scale: float = 1.0,
    bushing_c_scale: float = 1.0,
    wall_limit_s: float = 1800.0,
    collect_xray: bool = True,
) -> dict:
    """Run a 100% FULL-MODEL Bode sim with internal C++ pathway torque & joint-split probing."""
    t0 = time.time()
    a3ex.ensure_libraries_loaded()
    _, bp = a3ex.load_base_params()
    ov = dict(bp)
    ov.update(a3ex.TIER_AB_CLEAN_LOCKED)
    ov.update(overrides)
    # Enforce 100% FULL MODEL: never allow subsystems to be turned off
    for flag in ("enable_vcr", "enable_ccr", "enable_task_controller", "enable_antibuckling",
                 "enable_gto_reflex", "enable_renshaw", "enable_ligament_reflex"):
        ov[flag] = True

    model = osim.Model(a3ex.KUO_MODEL_PATH)
    # Optional in-memory sensitivity scaling of disc bushings (1.0 = untouched model)
    if abs(bushing_k_scale - 1.0) > 1e-6 or abs(bushing_c_scale - 1.0) > 1e-6:
        fset = model.updForceSet()
        for i in range(fset.getSize()):
            f = fset.get(i)
            if f.getConcreteClassName() == "BushingForce":
                bf = osim.BushingForce.safeDownCast(f)
                rk = bf.get_rotational_stiffness()
                rc = bf.get_rotational_damping()
                bf.set_rotational_stiffness(osim.Vec3(rk.get(0) * bushing_k_scale, rk.get(1) * bushing_k_scale, rk.get(2) * bushing_k_scale))
                bf.set_rotational_damping(osim.Vec3(rc.get(0) * bushing_c_scale, rc.get(1) * bushing_c_scale, rc.get(2) * bushing_c_scale))

    a3ex._apply_srs_and_ctrl(model, ov)

    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c); c.setDefaultValue(0.0); c.setDefaultLocked(True)
    zf = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn); c.set_locked(False); c.setDefaultIsPrescribed(True); c.setPrescribedFunction(zf)
    cy1 = model.updCoordinateSet().get("yaw1"); cy1.set_locked(False); cy1.setDefaultIsPrescribed(False)

    delay, ramp_duration = 0.8, 1.5
    # LatEC has f_min = 0.121 Hz (T = 8.26 s), so ALWAYS run exact 15.8 s (10.8 s = 1.31 cycles) to avoid ill-conditioned 0.12 Hz phase!
    use_exact = exact_15s or (mode == "LatEC")
    target_time, eval_start = (15.8, 5.0) if use_exact else (10.0, 4.0)

    df_lat, df_yaw = a3ex.load_aligned_data()
    if mode == "APEO":
        ordered = sorted(a3ex.CANONICAL_TARGETS, key=lambda x: x[0])
        freqs = np.array([t[0] for t in ordered], dtype=float)
        tgt_gains = np.array([t[1] for t in ordered], dtype=float)
        tgt_phases = np.array([t[2] for t in ordered], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array([0.0215 * (0.835 / f) / math.sqrt(K) for f in freqs], dtype=float)
        ms_spline, _ = a3ex.build_c2_multisine_spline(freqs, amps_in, phases, delay, ramp_duration, target_time)
        pname, ax_i = "spine_tx", 2  # Z-axis (sagittal)
    elif mode == "LatEC":
        freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(a3ex.get_latec_amplitudes(freqs), dtype=float)
        ms_spline, _ = a3ex.build_c2_multisine_spline(freqs, amps_in, phases, delay, ramp_duration, target_time)
        pname, ax_i = "spine_tz", 0  # X-axis (coronal)
    else:
        freqs = np.array(df_yaw["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_yaw["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.array(df_yaw["Exp_Yaw_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(a3ex.get_yawma_amplitudes_deg(freqs), dtype=float)
        ms_spline, _ = a3ex.build_c2_zero_offset_spline_yaw(freqs, np.radians(amps_in), phases, delay, ramp_duration, target_time)
        pname, ax_i = "gndyaw", 1  # Y-axis (axial)

    cp = model.updCoordinateSet().get(pname); cp.setPrescribedFunction(ms_spline)
    state = model.initSystem(); cy1.setIsPrescribed(state, False); cp.setSpeedValue(state, 0.0)
    a3ex._init_neck_state(model, state, ov)

    ctrl = None
    for ci in range(model.getControllerSet().getSize()):
        cc = model.getControllerSet().get(ci)
        if cc.getConcreteClassName() == "HeadNeckNeuralController":
            ctrl = cc
            break

    mgr = osim.Manager(model); mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    cr1, cr2 = cs.get("roll1"), cs.get("roll2")
    cy_1, cy_2, cg = cs.get("yaw1"), cs.get("yaw2"), cs.get("gndyaw")
    skull = model.getBodySet().get("skull")

    ts, p_l, r_l, yr_l, yg_l = [], [], [], [], []
    q1_l, q2_l, rel_l = [], [], []
    vcr_l, ccr_l, bkl_l, qp_l, phin_l, mah_l, cun_l, thix_l = [], [], [], [], [], [], [], []

    aborted, abort_reason = False, ""
    t = 0.0
    while t < target_time - 1e-9:
        if (time.time() - t0) > wall_limit_s:
            aborted, abort_reason = True, f"wall>{wall_limit_s:.0f}s@{t:.2f}s"
            break
        t = min(t + 0.005, target_time)
        try:
            state = mgr.integrate(t)
        except Exception as e:
            aborted, abort_reason = True, str(e).splitlines()[0]
            break
        p1d, p2d = math.degrees(cp1.getValue(state)), math.degrees(cp2.getValue(state))
        r1d, r2d = math.degrees(cr1.getValue(state)), math.degrees(cr2.getValue(state))
        y1d, y2d = math.degrees(cy_1.getValue(state)), math.degrees(cy_2.getValue(state))
        pd, rd, yrd = p1d + p2d, r1d + r2d, y1d + y2d
        ygd = math.degrees(cg.getValue(state)) + yrd
        if abs(pd) > 54.0 or abs(rd) > 35.0 or abs(yrd) > 48.75:
            aborted, abort_reason = True, f"ROM@{t:.2f}s p={pd:+.1f} r={rd:+.1f} y={yrd:+.1f}"
            break

        ts.append(t); p_l.append(pd); r_l.append(rd); yr_l.append(yrd); yg_l.append(ygd)
        if mode == "APEO":
            q1_l.append(-p1d); q2_l.append(-p2d)
            rel_l.append(skull.getPositionInGround(state).get(0) - cp.getValue(state))
        elif mode == "LatEC":
            q1_l.append(-r1d); q2_l.append(-r2d)
            rel_l.append(skull.getPositionInGround(state).get(2) - cp.getValue(state))
        else:
            q1_l.append(y1d); q2_l.append(y2d)
            rel_l.append(yrd)

        # Read internal C++ controller probes at post-eval window when X-ray diagnostics are requested
        if collect_xray and t >= eval_start - 0.02 and ctrl is not None:
            model.realizeDynamics(state)
            vcr_l.append(_parse_vec3(ctrl.getOutput("dbg_vcr_torque").getValueAsString(state))[ax_i])
            ccr_l.append(_parse_vec3(ctrl.getOutput("dbg_ccr_torque").getValueAsString(state))[ax_i])
            bkl_l.append(_parse_vec3(ctrl.getOutput("dbg_buckle_torque").getValueAsString(state))[ax_i])
            qp_l.append(_parse_vec3(ctrl.getOutput("dbg_qp_delivered_3dof").getValueAsString(state))[ax_i])
            phin_l.append(_parse_vec3(ctrl.getOutput("dbg_phasic_oto_input").getValueAsString(state))[ax_i])
            mah_l.append(math.degrees(_parse_vec3(ctrl.getOutput("mahony_rotvec").getValueAsString(state))[ax_i]))
            cun_l.append(math.degrees(_parse_vec3(ctrl.getOutput("cuneate_rotvec").getValueAsString(state))[ax_i]))
            try:
                thix_l.append(float(model.getStateVariableValue(state, "/forceset/splen_cap_sklc6/srs_thix")))
            except Exception:
                thix_l.append(1.0)
        else:
            vcr_l.append(0.0); ccr_l.append(0.0); bkl_l.append(0.0); qp_l.append(0.0)
            phin_l.append(0.0); mah_l.append(0.0); cun_l.append(0.0); thix_l.append(1.0)

    if aborted or len(ts) < 100:
        return {"mode": mode, "aborted": True, "abort_reason": abort_reason, "wall_s": round(time.time() - t0, 2), "total_score": 1.2e6}

    t_arr = np.asarray(ts)
    p_arr, r_arr, yr_arr, yg_arr = np.asarray(p_l), np.asarray(r_l), np.asarray(yr_l), np.asarray(yg_l)
    main_arr = -p_arr if mode == "APEO" else (-r_arr if mode == "LatEC" else yg_arr)
    m = t_arr >= eval_start
    t_fit = t_arr[m]; t_rel = t_fit - delay

    cols = [np.ones_like(t_fit), t_fit - eval_start]
    for k in range(K):
        psi_k = 2.0 * math.pi * freqs[k] * t_rel + phases[k]
        cols.append(np.sin(psi_k)); cols.append(np.cos(psi_k))
    X = np.column_stack(cols)

    c_main, sim_g, sim_ph = _harmonic_fit(X, main_arr[m], freqs, amps_in)
    _, q1_g, q1_ph = _harmonic_fit(X, np.asarray(q1_l)[m], freqs, amps_in)
    _, q2_g, q2_ph = _harmonic_fit(X, np.asarray(q2_l)[m], freqs, amps_in)
    _, rel_g, rel_ph = _harmonic_fit(X, np.asarray(rel_l)[m], freqs, amps_in)
    _, vcr_g, vcr_ph = _harmonic_fit(X, np.asarray(vcr_l)[m], freqs, amps_in)
    _, ccr_g, ccr_ph = _harmonic_fit(X, np.asarray(ccr_l)[m], freqs, amps_in)
    _, bkl_g, bkl_ph = _harmonic_fit(X, np.asarray(bkl_l)[m], freqs, amps_in)
    _, qp_g, qp_ph = _harmonic_fit(X, np.asarray(qp_l)[m], freqs, amps_in)
    _, phin_g, _ = _harmonic_fit(X, np.asarray(phin_l)[m], freqs, amps_in)
    _, mah_g, mah_ph = _harmonic_fit(X, np.asarray(mah_l)[m], freqs, amps_in)

    ref_fit = np.zeros_like(t_fit); harm_fit = np.zeros_like(t_fit)
    g_loss, p_loss = 0.0, 0.0
    per_freq = []
    for k in range(K):
        psi = 2.0 * np.pi * freqs[k] * t_rel + phases[k]
        ref_fit += amps_in[k] * tgt_gains[k] * np.sin(psi + math.radians(tgt_phases[k]))
        a_c, b_c = float(c_main[2 + 2 * k]), float(c_main[2 + 2 * k + 1])
        harm_fit += a_c * np.sin(psi) + b_c * np.cos(psi)
        ph_err = ((float(sim_ph[k]) - float(tgt_phases[k]) + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(float(sim_g[k]), 1e-6) / float(tgt_gains[k]))
        g_pts = 100.0 * db_err * db_err
        p_pts = ph_err * ph_err
        g_loss += g_pts; p_loss += p_pts
        per_freq.append({
            "f_hz": round(float(freqs[k]), 3),
            "sim_g": round(float(sim_g[k]), 2), "tgt_g": round(float(tgt_gains[k]), 2),
            "db_err": round(db_err, 2), "g_pts": round(g_pts, 1),
            "sim_ph": round(float(sim_ph[k]), 1), "tgt_ph": round(float(tgt_phases[k]), 1),
            "ph_err": round(ph_err, 1), "p_pts": round(p_pts, 1),
            "q1_g": round(float(q1_g[k]), 1), "q1_ph": round(float(q1_ph[k]), 1),
            "q2_g": round(float(q2_g[k]), 1), "q2_ph": round(float(q2_ph[k]), 1),
            "rel_g": round(float(rel_g[k]), 3), "rel_ph": round(float(rel_ph[k]), 1),
            "tau_vcr_g": round(float(vcr_g[k]), 2), "tau_vcr_ph": round(float(vcr_ph[k]), 1),
            "tau_ccr_g": round(float(ccr_g[k]), 2), "tau_bkl_g": round(float(bkl_g[k]), 2),
            "tau_qp_g": round(float(qp_g[k]), 2), "mah_g": round(float(mah_g[k]), 1),
        })

    vel_act = np.gradient(main_arr[m], t_fit)
    vel_hrm = np.gradient(harm_fit, t_fit)
    vel_ref = np.gradient(ref_fit, t_fit)
    hf_rms = float(np.sqrt(np.mean((vel_act - vel_hrm) ** 2)))
    rv = float(np.corrcoef(vel_act, vel_ref)[0, 1]) if np.std(vel_act) > 1e-6 else 0.0
    p_dev = float(np.max(np.abs(p_arr[m] - p_arr[0])))
    r_dev = float(np.max(np.abs(r_arr[m])))
    y_dev = float(np.max(np.abs(yr_arr[m])))
    if mode == "APEO":
        xt = 100.0 * (r_dev ** 2 + y_dev ** 2); hf_p = 5.0 * hf_rms ** 2
    elif mode == "LatEC":
        xt = 100.0 * (p_dev ** 2 + y_dev ** 2); hf_p = 25.0 * hf_rms ** 2
    else:
        xt = 100.0 * (p_dev ** 2 + r_dev ** 2); hf_p = 25.0 * hf_rms ** 2
    cls = g_loss + p_loss
    tot = cls + xt + hf_p
    thix_arr = np.asarray(thix_l)[m]
    return {
        "mode": mode, "aborted": False, "exact_15s": exact_15s,
        "total_score": round(tot, 1), "classic_score": round(cls, 1),
        "gain_loss": round(g_loss, 1), "phase_loss": round(p_loss, 1),
        "xtalk_pen": round(xt, 1), "hf_pen": round(hf_p, 1),
        "hf_rms": round(hf_rms, 3), "rv": round(rv, 4),
        "thix_mean": round(float(np.mean(thix_arr)), 3),
        "thix_min": round(float(np.min(thix_arr)), 3),
        "wall_s": round(time.time() - t0, 1),
        "per_freq": per_freq,
    }


def print_xray(label: str, res: dict):
    if res.get("aborted"):
        print(f"\n=== {label} [{res['mode']}]: ABORTED ({res.get('abort_reason')}) ===")
        return
    print(
        f"\n=== {label} [{res['mode']}] ({res['wall_s']}s) | TOT={res['total_score']:8.1f} "
        f"(cls={res['classic_score']:7.1f} G={res['gain_loss']:6.1f} P={res['phase_loss']:6.1f} "
        f"xt={res['xtalk_pen']:5.1f} hfP={res['hf_pen']:5.1f}) rv={res['rv']:+.4f} hf={res['hf_rms']:.2f} thix={res['thix_mean']:.2f} ==="
    )
    print("  f(Hz) |  simG( tgtG)   dB  gPts |  simPh( tgtPh)   err  pPts |  q1G( ph)   q2G( ph) |  vcrG  ccrG  bklG   qpG")
    for r in res["per_freq"]:
        print(
            f"  {r['f_hz']:5.2f} | {r['sim_g']:5.1f}({r['tgt_g']:5.1f}) {r['db_err']:+5.1f} {r['g_pts']:5.0f} | "
            f"{r['sim_ph']:+6.1f}({r['tgt_ph']:+6.1f}) {r['ph_err']:+5.1f} {r['p_pts']:5.0f} | "
            f"{r['q1_g']:4.0f}({r['q1_ph']:+4.0f}) {r['q2_g']:4.0f}({r['q2_ph']:+4.0f}) | "
            f"{r['tau_vcr_g']:5.1f} {r['tau_ccr_g']:5.1f} {r['tau_bkl_g']:5.1f} {r['tau_qp_g']:5.1f}"
        )
