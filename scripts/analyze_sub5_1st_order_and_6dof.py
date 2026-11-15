#!/usr/bin/env python3
"""Evaluate, run finite-difference sensitivity, and optimize Sub 5 1st-order controller under USE_6DOF_QP=0 vs 1."""

import argparse
import copy
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

for _env_k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "SIMBODY_NUM_THREADS"):
    os.environ.setdefault(_env_k, "1")

import numpy as np

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import PER_MUSCLE_72_U0, ensure_libraries_loaded
from export_tuned_3axis_mot import (
    EYE_COORDS,
    GROUND_TRUNK_COORDS,
    _build_test_excitation,
    _compute_classic_bode,
    _configure_controller,
)


PARAMS_JSON_PATH = os.path.join(_ROOT, "logs", "roll_yaw_baseline", "tuned_3axis_params.json")
OUT_JSON_DIR = os.path.join(_ROOT, "logs", "roll_yaw_baseline")


def _nearest_val(t_arr, y_arr, t_target):
    idx = int(np.argmin(np.abs(t_arr - t_target)))
    return float(y_arr[idx])


def _compute_bode_with_bins(t_arr, y_fit_deg_full, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration):
    """Return classic Bode summary and per-frequency signed errors."""
    classic_score, gain_pts, phase_pts = _compute_classic_bode(
        t_arr, y_fit_deg_full, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration
    )
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

    bins = []
    ref_fit = np.zeros_like(t_fit)
    harm_fit = np.zeros_like(t_fit)
    for k in range(k_count):
        a_sin = float(coeffs[2 + 2 * k])
        b_cos = float(coeffs[2 + 2 * k + 1])
        sim_gain = math.hypot(a_sin, b_cos) / float(amps_input_unit[k])
        sim_phase = math.degrees(math.atan2(b_cos, a_sin))
        db_err = 20.0 * math.log10(max(sim_gain, 1e-6) / float(tgt_gains[k]))
        ph_err = ((sim_phase - float(tgt_phases[k]) + 180.0) % 360.0) - 180.0
        psi_k = 2.0 * math.pi * freqs[k] * t_rel + phases[k]
        ref_fit += float(amps_input_unit[k]) * float(tgt_gains[k]) * np.sin(psi_k + math.radians(float(tgt_phases[k])))
        harm_fit += a_sin * np.sin(psi_k) + b_cos * np.cos(psi_k)
        bins.append({
            "freq_hz": round(float(freqs[k]), 4),
            "db_err": round(db_err, 2),
            "ph_err_deg": round(ph_err, 1),
            "sim_gain": round(sim_gain, 2),
            "tgt_gain": round(float(tgt_gains[k]), 2),
        })

    vel_act = np.gradient(y_fit_deg, t_fit)
    vel_harm = np.gradient(harm_fit, t_fit)
    vel_ref = np.gradient(ref_fit, t_fit)
    hf_noise_rms = float(np.sqrt(np.mean((vel_act - vel_harm) ** 2)))
    corr_vel = float(np.corrcoef(vel_act, vel_ref)[0, 1]) if np.std(vel_act) > 1e-6 else 0.0

    return {
        "classic_bode_score": classic_score,
        "gain_loss_pts": gain_pts,
        "phase_loss_pts": phase_pts,
        "hf_noise_rms": round(hf_noise_rms, 3),
        "corr_vel": round(corr_vel, 4),
        "bins": bins,
    }


def simulate_trial(spec):
    """Run one APEO, LatEC, or YawMA trial and return Bode + cross-axis pitch/roll/yaw metrics."""
    tag, mode, params = spec
    ensure_libraries_loaded()

    try:
        model = osim.Model(os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim"))
        controller = None
        for i in range(model.getControllerSet().getSize()):
            ctrl = model.getControllerSet().get(i)
            if ctrl.getConcreteClassName() == "FixationController":
                ctrl.set_enabled(False)
            elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
                controller = ctrl
        if controller is None:
            return {"tag": tag, "mode": mode, "ok": False, "error": "Controller not found"}

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
        k_gdyn_eff = min(max(float(params["k_gamma_dyn"]), 0.0), 1.0)
        k_gstat_eff = min(max(float(params["k_gamma_stat"]), 0.0), 1.0)
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

        cs = model.getCoordinateSet()
        c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
        c_r1, c_r2 = cs.get("roll1"), cs.get("roll2")
        c_y1, c_y2 = cs.get("yaw1"), cs.get("yaw2")
        c_gndyaw = cs.get("gndyaw")

        t_l, p1_l, p2_l, pt_l, rt_l, yr_l, yg_l = [], [], [], [], [], [], []
        step = 0.005
        current_time = 0.0
        aborted = False
        abort_reason = None

        while current_time < target_time:
            current_time = min(current_time + step, target_time)
            state = manager.integrate(current_time)
            p1_d = math.degrees(c_p1.getValue(state))
            p2_d = math.degrees(c_p2.getValue(state))
            pt_d = p1_d + p2_d
            rt_d = math.degrees(c_r1.getValue(state) + c_r2.getValue(state))
            yr_d = math.degrees(c_y1.getValue(state) + c_y2.getValue(state))
            yg_d = math.degrees(c_gndyaw.getValue(state)) + yr_d

            t_l.append(current_time)
            p1_l.append(p1_d)
            p2_l.append(p2_d)
            pt_l.append(pt_d)
            rt_l.append(rt_d)
            yr_l.append(yr_d)
            yg_l.append(yg_d)

            if abs(pt_d) > 45.0 or abs(rt_d) > 35.0 or abs(yr_d) > 48.0:
                aborted = True
                abort_reason = f"t={current_time:.3f}s pt={pt_d:+.2f} rt={rt_d:+.2f} yr={yr_d:+.2f}"
                break

        t_arr = np.asarray(t_l)
        p1_arr = np.asarray(p1_l)
        p2_arr = np.asarray(p2_l)
        pt_arr = np.asarray(pt_l)
        rt_arr = np.asarray(rt_l)
        yr_arr = np.asarray(yr_l)
        yg_arr = np.asarray(yg_l)

        if aborted or len(t_arr) < 100 or float(t_arr[-1]) < (delay + ramp_duration + 1.0):
            return {
                "tag": tag,
                "mode": mode,
                "ok": False,
                "aborted": True,
                "error": abort_reason or f"Short run t_end={t_arr[-1] if len(t_arr) else 0.0}",
            }

        if mode == "APEO":
            y_eval = pt_arr
        elif mode == "LatEC":
            y_eval = -rt_arr
        else:
            y_eval = yg_arr

        bode = _compute_bode_with_bins(
            t_arr, y_eval, freqs, amps_input_unit, phases, tgt_gains, tgt_phases, delay, ramp_duration
        )

        # Deviation relative to resting t=0.005s
        dp1 = p1_arr - p1_arr[0]
        dp2 = p2_arr - p2_arr[0]
        dpt = pt_arr - pt_arr[0]
        m_ss = t_arr >= (delay + ramp_duration)

        t12_target = min(12.0, float(t_arr[-1]))
        cross = {
            "p_tot_init": round(float(pt_arr[0]), 2),
            "p1_init": round(float(p1_arr[0]), 2),
            "p2_init": round(float(p2_arr[0]), 2),
            "max_abs_dp_tot": round(float(np.max(np.abs(dpt))), 2),
            "max_abs_dp1": round(float(np.max(np.abs(dp1))), 2),
            "max_abs_dp2": round(float(np.max(np.abs(dp2))), 2),
            "max_abs_p_tot": round(float(np.max(np.abs(pt_arr))), 2),
            "max_abs_p1": round(float(np.max(np.abs(p1_arr))), 2),
            "max_abs_p2": round(float(np.max(np.abs(p2_arr))), 2),
            "p_tot_t3": round(_nearest_val(t_arr, pt_arr, 3.0), 2),
            "p1_t3": round(_nearest_val(t_arr, p1_arr, 3.0), 2),
            "p2_t3": round(_nearest_val(t_arr, p2_arr, 3.0), 2),
            "p_tot_t12": round(_nearest_val(t_arr, pt_arr, t12_target), 2),
            "p1_t12": round(_nearest_val(t_arr, p1_arr, t12_target), 2),
            "p2_t12": round(_nearest_val(t_arr, p2_arr, t12_target), 2),
            "p_tot_end": round(float(pt_arr[-1]), 2),
            "p1_end": round(float(p1_arr[-1]), 2),
            "p2_end": round(float(p2_arr[-1]), 2),
            "s_buckle_end": round(float(p1_arr[-1] - p2_arr[-1]), 2),
            "max_s_buckle_ss": round(float(np.max(np.abs(p1_arr[m_ss] - p2_arr[m_ss]))), 2),
            "rms_dp_tot_ss": round(float(np.sqrt(np.mean(dpt[m_ss] ** 2))), 2),
            "max_abs_roll_ss": round(float(np.max(np.abs(rt_arr[m_ss]))), 2),
            "max_abs_yaw_rel_ss": round(float(np.max(np.abs(yr_arr[m_ss]))), 2),
        }

        return {
            "tag": tag,
            "mode": mode,
            "ok": True,
            "aborted": False,
            "t_end": round(float(t_arr[-1]), 3),
            "use_6dof_qp": os.environ.get("USE_6DOF_QP", "0"),
            **bode,
            "cross": cross,
        }
    except Exception as exc:
        return {"tag": tag, "mode": mode, "ok": False, "aborted": True, "error": str(exc)}


def _run_jobs_in_qp_mode(jobs, qp_mode_str, max_workers=6):
    """Run jobs inside a fresh Python subprocess with USE_6DOF_QP set before DLL/CRT startup."""
    fd_in, in_path = tempfile.mkstemp(prefix="qp_in_", suffix=".json")
    fd_out, out_path = tempfile.mkstemp(prefix="qp_out_", suffix=".json")
    os.close(fd_in)
    os.close(fd_out)
    try:
        with open(in_path, "w", encoding="utf-8") as f:
            json.dump({"jobs": jobs, "max_workers": max_workers}, f)
        env = os.environ.copy()
        env["USE_6DOF_QP"] = qp_mode_str
        subprocess.run(
            [sys.executable, __file__, "--subcmd", "_worker_batch", "--in-json", in_path, "--out-json", out_path],
            env=env,
            check=True,
        )
        with open(out_path, "r", encoding="utf-8") as f:
            return json.load(f)
    finally:
        for p in (in_path, out_path):
            try:
                os.remove(p)
            except OSError:
                pass


def _worker_batch_main(in_json, out_json):
    with open(in_json, "r", encoding="utf-8") as f:
        payload = json.load(f)
    jobs = payload["jobs"]
    max_workers = int(payload.get("max_workers", 6))
    specs = [(j["tag"], j["mode"], j["params"]) for j in jobs]
    results = []
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        for res in ex.map(simulate_trial, specs):
            results.append(res)
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(results, f)


def _load_params():
    with open(PARAMS_JSON_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _fmt_trial(r):
    if not r["ok"]:
        return f"FAILED ({r.get('error')})"
    c = r["cross"]
    return (
        f"Classic={r['classic_bode_score']:8.2f} (G={r['gain_loss_pts']:7.1f}, P={r['phase_loss_pts']:7.1f}) | "
        f"noise={r['hf_noise_rms']:4.2f} r={r['corr_vel']:.3f} | "
        f"p_t3={c['p_tot_t3']:+5.2f} ({c['p1_t3']:+5.2f}/{c['p2_t3']:+5.2f}) | "
        f"p_end={c['p_tot_end']:+5.2f} ({c['p1_end']:+5.2f}/{c['p2_end']:+5.2f}) | "
        f"max|dp|={c['max_abs_dp_tot']:5.2f} (p1:{c['max_abs_dp1']:4.2f}, p2:{c['max_abs_dp2']:4.2f})"
    )


def cmd_baseline():
    base_params = _load_params()
    jobs = [
        {"tag": "base_APEO", "mode": "APEO", "params": base_params},
        {"tag": "base_LatEC", "mode": "LatEC", "params": base_params},
        {"tag": "base_YawMA", "mode": "YawMA", "params": base_params},
    ]
    out = {}
    t0 = time.time()
    for qp_mode in ("0", "1"):
        res_list = _run_jobs_in_qp_mode(jobs, qp_mode, max_workers=3)
        out[f"USE_6DOF_QP={qp_mode}"] = {r["mode"]: r for r in res_list}
        print(f"\n=== BASELINE (Sub 5 1st-Order 15.5/18.0ms) | USE_6DOF_QP={qp_mode} ({time.time()-t0:.1f}s) ===", flush=True)
        for m in ("APEO", "LatEC", "YawMA"):
            print(f"  {m:5s}: {_fmt_trial(out[f'USE_6DOF_QP={qp_mode}'][m])}", flush=True)

    out_path = os.path.join(OUT_JSON_DIR, "sub5_1st_order_baseline_qp0_vs_qp1.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved baseline JSON: {out_path}", flush=True)


def cmd_sensitivity():
    base_params = _load_params()
    base_qp0 = {r["mode"]: r for r in _run_jobs_in_qp_mode(
        [{"tag": f"b0_{m}", "mode": m, "params": base_params} for m in ("APEO", "LatEC", "YawMA")], "0", 3
    )}
    base_qp1 = {r["mode"]: r for r in _run_jobs_in_qp_mode(
        [{"tag": f"b1_{m}", "mode": m, "params": base_params} for m in ("APEO", "LatEC", "YawMA")], "1", 3
    )}

    # Key parameters to perturb (+/- step) across axes
    # Format: (param_key, delta, axes_to_check)
    perturb_specs = [
        # Shared Pitch / Trunk Gains (affect APEO directly & YawMA/LatEC cross-axis stability!)
        ("Kp_pitch",     0.40, ("APEO", "LatEC", "YawMA")),
        ("Ki_pitch",     0.50, ("APEO", "LatEC", "YawMA")),
        ("Kd_pitch",     0.08, ("APEO", "LatEC", "YawMA")),
        ("K_ton",        0.50, ("APEO", "YawMA")),
        ("K_sc",         0.40, ("APEO",)),
        ("K_phas",       1.50, ("APEO",)),
        ("kp_prop",      0.15, ("APEO",)),
        # Roll Parameters (LatEC)
        ("Kp_roll",      0.25, ("LatEC",)),
        ("Kd_roll",      0.15, ("LatEC",)),
        ("Ki_roll",      0.25, ("LatEC",)),
        ("K_ton_roll",   0.20, ("LatEC",)),
        ("K_sc_roll",    0.25, ("LatEC",)),
        ("K_phas_roll",  1.50, ("LatEC",)),
        ("Kp_prop_roll", 0.50, ("LatEC",)),
        ("z_ccr_roll",   0.02, ("LatEC",)),
        # Yaw Parameters (YawMA)
        ("Kp_yaw",       0.05, ("YawMA",)),
        ("Kd_yaw",       0.01, ("YawMA",)),
        ("Ki_yaw",       0.10, ("YawMA",)),
        ("K_sc_yaw",     0.25, ("YawMA",)),
        ("Kp_prop_yaw",  1.00, ("YawMA", "LatEC")),
        ("z_ccr_yaw",    0.015, ("YawMA",)),
        ("deriv_comp_yaw", 0.10, ("YawMA",)),
    ]

    jobs = []
    meta = []
    for pkey, delta, axes in perturb_specs:
        v0 = float(base_params[pkey])
        for sign, sname in ((+1.0, "plus"), (-1.0, "minus")):
            # Keep Ki_yaw >= 0
            v_new = v0 + sign * delta
            if pkey in ("Ki_yaw", "z_ccr_roll", "z_ccr_yaw", "deriv_comp_yaw", "Kd_yaw") and v_new < 0.0:
                v_new = 0.0
            if abs(v_new - v0) < 1e-9:
                continue
            p_mod = copy.deepcopy(base_params)
            p_mod[pkey] = v_new
            for mode in axes:
                tag = f"{pkey}__{sname}__{mode}"
                jobs.append({"tag": tag, "mode": mode, "params": p_mod})
                meta.append({"tag": tag, "pkey": pkey, "v0": v0, "v_new": v_new, "sign": sign, "mode": mode})

    print(f"Running sensitivity sweep ({len(jobs)} sims per QP mode, 6 workers each)...", flush=True)
    sens_report = {"base_qp0": base_qp0, "base_qp1": base_qp1, "sweeps": {}}
    for qp_mode, base_ref in (("0", base_qp0), ("1", base_qp1)):
        t0 = time.time()
        res_list = _run_jobs_in_qp_mode(jobs, qp_mode, max_workers=6)
        res_by_tag = {r["tag"]: r for r in res_list}
        print(f"\n=== SENSITIVITY USE_6DOF_QP={qp_mode} ({len(jobs)} sims in {time.time()-t0:.1f}s) ===", flush=True)
        rows = []
        for m_info in meta:
            r = res_by_tag[m_info["tag"]]
            b = base_ref[m_info["mode"]]
            dv = m_info["v_new"] - m_info["v0"]
            if not r["ok"]:
                print(f"  {m_info['mode']:5s} {m_info['pkey']:15s} -> {m_info['v_new']:7.3f} | FAILED: {r.get('error')}", flush=True)
                rows.append({**m_info, "ok": False, "error": r.get("error")})
                continue
            dJ = r["classic_bode_score"] - b["classic_bode_score"]
            ddp = r["cross"]["max_abs_dp_tot"] - b["cross"]["max_abs_dp_tot"]
            rows.append({
                **m_info,
                "ok": True,
                "classic_bode_score": r["classic_bode_score"],
                "d_bode": round(dJ, 2),
                "max_abs_dp_tot": r["cross"]["max_abs_dp_tot"],
                "d_max_abs_dp_tot": round(ddp, 2),
                "p_tot_t3": r["cross"]["p_tot_t3"],
                "p_tot_end": r["cross"]["p_tot_end"],
                "p1_end": r["cross"]["p1_end"],
                "p2_end": r["cross"]["p2_end"],
            })
        # Print sorted best Bode improvements per axis
        for mode in ("APEO", "LatEC", "YawMA"):
            sub = [x for x in rows if x["mode"] == mode and x["ok"]]
            sub.sort(key=lambda x: x["d_bode"])
            print(f"  -- Top {mode} improvements (USE_6DOF_QP={qp_mode}) --", flush=True)
            for x in sub[:8]:
                print(
                    f"    {x['pkey']:15s} {x['v0']:6.3f}->{x['v_new']:6.3f} | "
                    f"J={x['classic_bode_score']:8.2f} ({x['d_bode']:+8.2f}) | "
                    f"max|dp|={x['max_abs_dp_tot']:5.2f} ({x['d_max_abs_dp_tot']:+5.2f}) | "
                    f"p_end={x['p_tot_end']:+5.2f} ({x['p1_end']:+5.2f}/{x['p2_end']:+5.2f})",
                    flush=True,
                )
        sens_report["sweeps"][f"qp{qp_mode}"] = rows

    out_path = os.path.join(OUT_JSON_DIR, "sub5_1st_order_sensitivity.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(sens_report, f, indent=2)
    print(f"\nSaved sensitivity JSON: {out_path}", flush=True)


def cmd_coopt_batch():
    """Run all 12 baseline + co-optimization candidates in parallel under USE_6DOF_QP=1 + Fixes 1-3."""
    base_params = _load_params()

    specs = [
        # Jobs 1-3: New 6-DOF Baseline with Fixes 1-3
        ("base_APEO",  "APEO",  {}),
        ("base_LatEC", "LatEC", {}),
        ("base_YawMA", "YawMA", {}),
        # Jobs 4-6: APEO Pitch Sensitivity/Tuning
        ("APEO_1", "APEO", {"kp_prop": 0.40, "K_sc": 3.85}),
        ("APEO_2", "APEO", {"kp_prop": 0.38, "K_sc": 3.70, "K_ton": 5.60}),
        ("APEO_3", "APEO", {"kp_prop": 0.41, "K_sc": 3.90, "z_sc": 0.0062}),
        # Jobs 7-9: LatEC Roll Sensitivity/Tuning
        ("LatEC_1", "LatEC", {"K_ton_roll": -1.10, "p_ton_roll": 2.6, "K_sc_roll": 1.85, "Kp_prop_roll": 4.0}),
        ("LatEC_2", "LatEC", {"Kp_roll": 1.25, "Ki_roll": 1.65, "K_ton_roll": -1.00, "K_phas_roll": 17.8, "Kp_prop_roll": 3.8}),
        ("LatEC_3", "LatEC", {"Kp_roll": 1.35, "Kd_roll": 1.05, "K_ton_roll": -0.95, "K_sc_roll": 1.90, "z_ccr_roll": 0.115}),
        # Jobs 10-12: YawMA Yaw Sensitivity/Tuning
        ("YawMA_1", "YawMA", {"K_sc_yaw": 1.45, "Kp_prop_yaw": 9.5, "z_ccr_yaw": 0.075, "deriv_comp_yaw": 0.35}),
        ("YawMA_2", "YawMA", {"K_sc_yaw": 1.35, "p_sc_yaw": 4.2, "Kp_prop_yaw": 9.2, "z_ccr_yaw": 0.070, "deriv_comp_yaw": 0.30}),
        ("YawMA_3", "YawMA", {"Kp_yaw": 0.22, "K_sc_yaw": 1.50, "Kp_prop_yaw": 10.0, "z_ccr_yaw": 0.080, "deriv_comp_yaw": 0.40}),
    ]

    jobs = []
    overrides_by_tag = {}
    for tag, mode, ov in specs:
        p = copy.deepcopy(base_params)
        p.update(ov)
        jobs.append({"tag": tag, "mode": mode, "params": p})
        overrides_by_tag[tag] = ov

    print(f"Launching 12-worker coopt_batch (USE_6DOF_QP=1 + Fixes 1-3)...", flush=True)
    t0 = time.time()
    res_list = _run_jobs_in_qp_mode(jobs, "1", max_workers=12)
    wall = time.time() - t0
    print(f"\n=== COOPT_BATCH COMPLETE ({len(res_list)} jobs in {wall:.1f}s) ===", flush=True)

    for r in res_list:
        r["overrides"] = overrides_by_tag.get(r["tag"], {})
        print(f"  {r['tag']:8s} ({r['mode']:5s}): {_fmt_trial(r)}", flush=True)

    out_path = os.path.join(OUT_JSON_DIR, "sub5_coopt_batch_results.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(
            {"wall_s": round(wall, 1), "base_params": base_params, "results": res_list},
            f,
            indent=2,
        )
    print(f"\nSaved coopt batch JSON: {out_path}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--subcmd", required=True, choices=["baseline", "sensitivity", "coopt_batch", "_worker_batch"])
    parser.add_argument("--in-json")
    parser.add_argument("--out-json")
    args = parser.parse_args()
    if args.subcmd == "_worker_batch":
        _worker_batch_main(args.in_json, args.out_json)
    elif args.subcmd == "baseline":
        cmd_baseline()
    elif args.subcmd == "sensitivity":
        cmd_sensitivity()
    elif args.subcmd == "coopt_batch":
        cmd_coopt_batch()


if __name__ == "__main__":
    main()
