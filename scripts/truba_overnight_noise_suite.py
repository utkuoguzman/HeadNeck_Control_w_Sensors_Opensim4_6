#!/usr/bin/env python3
"""Overnight Multi-Seed Tier-1 Biological Noise Robustness Suite (TRUBA orfoz -n 56).

Runs 3 high-parallelism waves across 56 cores:
  Part A (48 sims): 3-Axis Passive Bode across noise_scale in {0.0, 0.5, 1.0, 1.5} x Seeds 1..5
  Part B (48 sims): Zheng 2g & 3g Sled across u in {-0.75, 0.00, +0.50, +1.00} x (OFF + 5 Seeds ON)
  Part C (14 sims): 5.0 s Quiet Stance 2x2 Ablation (10 sims) + 18.0 s Voluntary RPY Sweep (OFF + 3 Seeds ON)
"""

import cmath
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
import opensim as osim

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cmaes_worker_p22 import PER_MUSCLE_72_U0, ensure_libraries_loaded
from scripts.generate_standard_plot import (
    ART_DIR,
    MODEL_PATH,
    apply_cli_overrides_and_inspect,
    run_axis_worker,
)
from scripts.run_rpy_50deg_yaw_sweep import (
    AXIS_INDEX,
    PHASES,
    compute_overshoot,
    compute_t95,
)
from scripts.run_zheng_2g_3g_urgency_sweep import (
    DT_LOG,
    SCM_NAMES,
    SPLEN_CAP_NAMES,
    T_SETTLE,
    T_TOTAL,
    build_sled_spline,
)
from scripts.validate_5dof_voluntary_and_se2_shift import CHAMP_C1_JSON
from scripts.validate_biological_noise import run_quiet_stance_worker

OUT_JSON = os.path.join(_ROOT, "logs", "truba_overnight_noise_suite.json")
OUT_PNG = os.path.join(_ROOT, "results", "truba_overnight_noise_suite.png")

BODE_MODES = ["APEO", "LatEC", "YawMA"]
BODE_SCALES = [0.0, 0.5, 1.0, 1.5]
SEEDS_5 = [1, 2, 3, 4, 5]
SEEDS_3 = [1, 2, 3]
SLED_G_LEVELS = [2.0, 3.0]
SLED_URGENCIES = [-0.75, 0.00, 0.50, 1.00]


def run_bode_noise_worker(spec: dict) -> dict:
    pdict = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))["params"]
    s_on = bool(spec["sensor_noise"])
    m_on = bool(spec["motor_noise"])
    scale = float(spec["noise_scale"])
    seed = int(spec["seed"])

    overrides = [
        (k, f"{float(v):.15g}")
        for k, v in pdict.items()
        if k not in ("enable_sensor_noise", "enable_motor_noise", "noise_seed", "noise_scale")
    ] + [
        ("enable_sensor_noise", "true" if s_on else "false"),
        ("enable_motor_noise", "true" if m_on else "false"),
        ("noise_seed", str(seed)),
        ("noise_scale", f"{scale:.6g}"),
    ]
    res = run_axis_worker((spec["mode"], overrides))
    return {
        "kind": "bode",
        "mode": spec["mode"],
        "noise_scale": scale,
        "sensor_noise": s_on,
        "motor_noise": m_on,
        "seed": seed,
        "classic_bode_score": float(res["classic_bode_score"]),
        "total_score": float(res["total_score"]),
        "gain_loss_pts": float(res["gain_loss_pts"]),
        "phase_loss_pts": float(res["phase_loss_pts"]),
        "pitch_crosstalk_penalty": float(res["pitch_crosstalk_penalty"]),
        "hf_penalty": float(res["hf_penalty"]),
        "hf_noise_rms": float(res["hf_noise_rms"]),
        "corr_vel": float(res["corr_vel"]),
        "quiet_main_max_deg": float(res["quiet_main_max_deg"]),
        "aborted": bool(res["aborted"]),
        "freq_frf": {
            k: {
                "sim_gain": float(v["sim_gain"]),
                "sim_phase": float(v["sim_phase"]),
                "target_gain": float(v["target_gain"]),
                "target_phase": float(v["target_phase"]),
            }
            for k, v in res.get("freq_results", {}).items()
        },
    }


def run_sled_noise_worker(spec: dict) -> dict:
    t0 = time.time()
    ensure_libraries_loaded()

    g_level = float(spec["g_level"])
    urgency = float(spec["urgency"])
    s_on = bool(spec["sensor_noise"])
    m_on = bool(spec["motor_noise"])
    seed = int(spec["seed"])
    scale = float(spec["noise_scale"])

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
        "enable_sensor_noise": s_on,
        "enable_motor_noise": m_on,
        "noise_seed": seed,
        "noise_scale": scale,
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
        raise RuntimeError("HeadNeckNeuralController not found")

    override_pairs = [
        (k, ("true" if v else "false") if isinstance(v, bool) else (str(int(v)) if k == "noise_seed" else repr(float(v))))
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

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.setPrescribedFunction(build_sled_spline(g_level, sign_x=-1.0))

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
    splen_idx, scm_idx = [], []
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        if mname in SPLEN_CAP_NAMES:
            splen_idx.append(i)
        if mname in SCM_NAMES:
            scm_idx.append(i)
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
    c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
    ts_abs, flex_abs, vel_dps, spl_ts, scm_ts = [], [], [], [], []

    def sample(tv, st):
        model.realizeDynamics(st)
        ts_abs.append(float(tv))
        flex_abs.append(-math.degrees(c_p1.getValue(st) + c_p2.getValue(st)))
        vel_dps.append(-math.degrees(c_p1.getSpeedValue(st) + c_p2.getSpeedValue(st)))
        spl_ts.append(float(sum(float(muscles.get(idx).getActivation(st)) for idx in splen_idx) / len(splen_idx)))
        scm_ts.append(float(sum(float(muscles.get(idx).getActivation(st)) for idx in scm_idx) / len(scm_idx)))

    sample(0.0, state)
    t_now = 0.0
    stop_reason = None
    while t_now < T_TOTAL - 1e-9:
        t_next = round(min(t_now + DT_LOG, T_TOTAL), 6)
        try:
            state = mgr.integrate(t_next)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]
            break
        t_now = t_next
        sample(t_now, state)

    ts_arr = np.asarray(ts_abs, dtype=float)
    f_arr = np.asarray(flex_abs, dtype=float)
    vf_arr = np.asarray(vel_dps, dtype=float)
    spl_arr = np.asarray(spl_ts, dtype=float)
    scm_arr = np.asarray(scm_ts, dtype=float)

    imp_mask = ts_arr >= (T_SETTLE - 1e-9)
    idx0 = int(np.where(imp_mask)[0][0]) if np.any(imp_mask) else 0
    t_imp_ms = (ts_arr[imp_mask] - T_SETTLE) * 1000.0
    f_imp = f_arr[imp_mask] - float(f_arr[idx0])
    vf_imp = vf_arr[imp_mask]
    spl_imp = spl_arr[imp_mask]
    scm_imp = scm_arr[imp_mask]

    if len(f_imp) > 0:
        idx_pk = int(np.argmax(f_imp))
        peak_flex = float(f_imp[idx_pk])
        t_pk_ms = float(t_imp_ms[idx_pk])
        ons_mask = t_imp_ms <= 150.0 + 1e-9
        min_ons = float(np.min(f_imp[ons_mask])) if np.any(ons_mask) else 0.0
        pk_vel = float(np.max(np.abs(vf_imp)))
        pk_spl = float(np.max(spl_imp))
        pk_scm = float(np.max(scm_imp))
        n_pk = 0
        for k in range(1, len(spl_imp) - 1):
            if spl_imp[k] > 0.08 and spl_imp[k] > spl_imp[k - 1] and spl_imp[k] >= spl_imp[k + 1]:
                lo = max(0, k - 25)
                hi = min(len(spl_imp), k + 26)
                prom = float(spl_imp[k] - max(np.min(spl_imp[lo : k + 1]), np.min(spl_imp[k:hi])))
                if prom >= 0.015:
                    n_pk += 1
    else:
        peak_flex = t_pk_ms = min_ons = pk_vel = pk_spl = pk_scm = 0.0
        n_pk = 0

    return {
        "kind": "sled",
        "g_level": g_level,
        "urgency": urgency,
        "tag": "OFF" if not (s_on or m_on) else f"ON_s{seed}",
        "sensor_noise": s_on,
        "motor_noise": m_on,
        "seed": seed,
        "noise_scale": scale,
        "stop_reason": stop_reason,
        "peak_flex_deg": round(peak_flex, 2),
        "min_flex_ons_deg": round(min_ons, 2),
        "t_peak_ms": round(t_pk_ms, 1),
        "peak_vel_dps": round(pk_vel, 1),
        "peak_splen_cap": round(pk_spl, 4),
        "n_splen_peaks": int(n_pk),
        "peak_scm": round(pk_scm, 4),
        "wall_s": round(time.time() - t0, 2),
        "t_imp_ms": [round(float(v), 1) for v in t_imp_ms],
        "pitch_rel_deg": [round(float(v), 3) for v in f_imp],
        "splen_cap_act": [round(float(v), 4) for v in spl_imp],
    }


def run_rpy_noise_worker(spec: dict) -> dict:
    t0 = time.time()
    ensure_libraries_loaded()
    s_on = bool(spec["sensor_noise"])
    m_on = bool(spec["motor_noise"])
    seed = int(spec["seed"])
    scale = float(spec["noise_scale"])

    champ_obj = json.load(open(CHAMP_C1_JSON, "r", encoding="utf-8"))
    base_params = dict(champ_obj["params"])
    base_params.update({
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
        "urgency": 0.0,
        "desired_pitch": 0.0,
        "desired_roll": 0.0,
        "desired_yaw": 0.0,
        "desired_ap_shift": 0.0,
        "desired_lat_shift": 0.0,
        "enable_sensor_noise": s_on,
        "enable_motor_noise": m_on,
        "noise_seed": seed,
        "noise_scale": scale,
    })

    model = osim.Model(MODEL_PATH)
    controller = model.updControllerSet().get("VCR_CCR_Controller")
    override_pairs = [
        (k, ("true" if v else "false") if isinstance(v, bool) else (str(int(v)) if k == "noise_seed" else repr(float(v))))
        for k, v in base_params.items()
        if controller.hasProperty(k)
    ]
    apply_cli_overrides_and_inspect(controller, override_pairs)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    model.realizePosition(state)

    skull = model.getBodySet().get("skull")
    q0 = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q0.get(idx))
    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral", "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    k_gdyn = float(base_params.get("K_gamma_dyn", 0.857744))
    k_gstat = float(base_params.get("K_gamma_stat", 0.69312344))
    muscles = model.updMuscles()
    n_mus = muscles.getSize()
    for i in range(n_mus):
        m = muscles.get(i)
        mname = m.getName()
        if mname.startswith("r_") or mname.startswith("l_"):
            continue
        u0 = 0.03
        gd0 = 100.0 * k_gdyn * (0.25 + 0.75 * u0)
        gs0 = 100.0 * k_gstat * u0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{mname}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{mname}", fstat0)
    model.equilibrateMuscles(state)

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
    c_p1, c_p2 = cs.get("pitch1"), cs.get("pitch2")
    c_r1, c_r2 = cs.get("roll1"), cs.get("roll2")
    c_y1, c_y2 = cs.get("yaw1"), cs.get("yaw2")

    dt = 0.01
    duration = 18.0
    t_now = 0.0
    curr_phase_id = -1
    tr = {k: [] for k in ("t", "pitch", "roll", "yaw", "ref_pitch", "ref_roll", "ref_yaw", "vel_pitch", "vel_roll", "vel_yaw", "ref_vel_pitch", "ref_vel_roll", "ref_vel_yaw", "max_act")}
    stop_reason = None

    while t_now < duration - 1e-9:
        for ph in PHASES:
            if ph["t_start"] - 1e-9 <= t_now < ph["t_end"] - 1e-9:
                if ph["id"] != curr_phase_id:
                    curr_phase_id = ph["id"]
                    osim.PropertyHelper.setValueDouble(float(ph["urgency"]), controller.updPropertyByName("urgency"))
                    osim.PropertyHelper.setValueDouble(float(math.radians(ph["pitch_deg"])), controller.updPropertyByName("desired_pitch"))
                    osim.PropertyHelper.setValueDouble(float(math.radians(ph["roll_deg"])), controller.updPropertyByName("desired_roll"))
                    osim.PropertyHelper.setValueDouble(float(math.radians(ph["yaw_deg"])), controller.updPropertyByName("desired_yaw"))
                break
        t_next = min(t_now + dt, duration)
        try:
            state = mgr.integrate(t_next)
            model.realizeVelocity(state)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]
            break
        t_now = t_next

        tr["t"].append(t_now)
        tr["pitch"].append(math.degrees(c_p1.getValue(state) + c_p2.getValue(state)))
        tr["roll"].append(math.degrees(c_r1.getValue(state) + c_r2.getValue(state)))
        tr["yaw"].append(math.degrees(c_y1.getValue(state) + c_y2.getValue(state)))
        tr["ref_pitch"].append(math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_pos_pitch")))
        tr["ref_roll"].append(math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_pos_roll")))
        tr["ref_yaw"].append(math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_pos_yaw")))
        tr["vel_pitch"].append(math.degrees(c_p1.getSpeedValue(state) + c_p2.getSpeedValue(state)))
        tr["vel_roll"].append(math.degrees(c_r1.getSpeedValue(state) + c_r2.getSpeedValue(state)))
        tr["vel_yaw"].append(math.degrees(c_y1.getSpeedValue(state) + c_y2.getSpeedValue(state)))
        tr["ref_vel_pitch"].append(math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_vel_pitch")))
        tr["ref_vel_roll"].append(math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_vel_roll")))
        tr["ref_vel_yaw"].append(math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/traj_vel_yaw")))
        m_act = max(float(muscles.get(mi).getActivation(state)) for mi in range(n_mus) if not (muscles.get(mi).getName().startswith("r_") or muscles.get(mi).getName().startswith("l_")))
        tr["max_act"].append(m_act)

    t_np = np.asarray(tr["t"], dtype=float)
    phase_summaries = []
    prev_end = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    for ph in PHASES:
        mask = (t_np >= ph["t_start"]) & (t_np <= ph["t_end"])
        if not np.any(mask):
            continue
        t_ph = t_np[mask]
        end_mask = (t_ph >= max(ph["t_start"], t_ph[-1] - 0.25)) & (t_ph <= t_ph[-1])
        idx_end = np.where(mask)[0][-len(t_ph[end_mask]):]
        arr_ph = {ax: np.asarray(tr[ax])[mask] for ax in ("pitch", "roll", "yaw")}
        end_map = {ax: float(np.mean(np.asarray(tr[ax])[idx_end])) for ax in ("pitch", "roll", "yaw")}
        target_map = {"pitch": ph["pitch_deg"], "roll": ph["roll_deg"], "yaw": ph["yaw_deg"]}
        pax = ph["primary_axis"]
        if pax is not None:
            ovs_tgt_deg, ovs_tgt_pct = compute_overshoot(arr_ph[pax], target_map[pax], prev_end[pax])
            ovs_end_deg, ovs_end_pct = compute_overshoot(arr_ph[pax], end_map[pax], prev_end[pax])
            t95_s = compute_t95(t_ph, arr_ph[pax], ph["t_start"], end_map[pax], prev_end[pax])
            flagged = bool(ovs_tgt_deg > 2.0 or ovs_tgt_pct > 5.0 or ovs_end_deg > 2.0 or ovs_end_pct > 5.0)
            max_xtalk = round(float(max(np.max(np.abs(arr_ph[ax])) for ax in ("pitch", "roll", "yaw") if ax != pax)), 2)
        else:
            ovs_tgt_deg = ovs_tgt_pct = ovs_end_deg = ovs_end_pct = t95_s = 0.0
            flagged = False
            max_xtalk = round(float(max(np.max(np.abs(arr_ph[ax])) for ax in ("pitch", "roll", "yaw"))), 2)
        phase_summaries.append({
            "id": ph["id"],
            "name": ph["name"],
            "primary_axis": pax,
            "end_deg": [round(end_map[ax], 2) for ax in ("pitch", "roll", "yaw")],
            "overshoot_vs_target_deg": ovs_tgt_deg,
            "overshoot_vs_end_deg": ovs_end_deg,
            "overshoot_flagged": flagged,
            "t95_settling_s": t95_s,
            "max_off_axis_crosstalk_deg": max_xtalk,
        })
        prev_end = dict(end_map)

    rms_err = {
        ax: round(float(np.sqrt(np.mean((np.asarray(tr[ax]) - np.asarray(tr[f"ref_{ax}"])) ** 2))), 3)
        for ax in ("pitch", "roll", "yaw")
    }
    max_ovs = max((ps["overshoot_vs_target_deg"] for ps in phase_summaries), default=0.0)
    n_flagged = sum(1 for ps in phase_summaries if ps["overshoot_flagged"])
    return {
        "kind": "rpy",
        "tag": "OFF" if not (s_on or m_on) else f"ON_s{seed}",
        "sensor_noise": s_on,
        "motor_noise": m_on,
        "seed": seed,
        "noise_scale": scale,
        "t_end_s": round(float(t_now), 3),
        "stop_reason": stop_reason,
        "wall_s": round(time.time() - t0, 2),
        "rms_err_vs_traj_deg": rms_err,
        "max_ovs_tgt_deg": round(float(max_ovs), 2),
        "n_flagged": int(n_flagged),
        "phases": phase_summaries,
    }


def dispatch_part_c(spec: dict) -> dict:
    if spec["kind"] == "quiet":
        return run_quiet_stance_worker(spec)
    return run_rpy_noise_worker(spec)


def compute_bode_stats(bode_res: list) -> tuple:
    stats = {}
    ens_frf = {}
    for sc in BODE_SCALES:
        stats[sc] = {}
        ens_frf[sc] = {}
        for m in BODE_MODES:
            rows = [r for r in bode_res if abs(r["noise_scale"] - sc) < 1e-6 and r["mode"] == m]
            tots = [r["total_score"] for r in rows]
            clss = [r["classic_bode_score"] for r in rows]
            hfs = [r["hf_noise_rms"] for r in rows]
            corrs = [r["corr_vel"] for r in rows]
            stats[sc][m] = {
                "total_mean": round(float(np.mean(tots)), 2),
                "total_sd": round(float(np.std(tots, ddof=1)), 2) if len(tots) > 1 else 0.0,
                "classic_mean": round(float(np.mean(clss)), 2),
                "classic_sd": round(float(np.std(clss, ddof=1)), 2) if len(clss) > 1 else 0.0,
                "hf_mean": round(float(np.mean(hfs)), 3),
                "hf_sd": round(float(np.std(hfs, ddof=1)), 3) if len(hfs) > 1 else 0.0,
                "corr_mean": round(float(np.mean(corrs)), 4),
                "n_aborted": sum(1 for r in rows if r["aborted"]),
            }
            if rows and rows[0]["freq_frf"]:
                ens_c = 0.0
                for fk, fv in rows[0]["freq_frf"].items():
                    H_mean = np.mean([r["freq_frf"][fk]["sim_gain"] * cmath.exp(1j * math.radians(r["freq_frf"][fk]["sim_phase"])) for r in rows])
                    db_e = 20.0 * math.log10(max(abs(H_mean), 1e-6) / fv["target_gain"])
                    ph_e = ((math.degrees(cmath.phase(H_mean)) - fv["target_phase"] + 180.0) % 360.0) - 180.0
                    ens_c += 100.0 * (db_e ** 2) + (ph_e ** 2)
                ens_frf[sc][m] = round(float(ens_c), 2)
            else:
                ens_frf[sc][m] = None
        stats[sc]["CombinedTotal"] = {
            "mean": round(sum(stats[sc][m]["total_mean"] for m in BODE_MODES), 2),
            "ens_classic": round(sum(ens_frf[sc][m] or 0.0 for m in BODE_MODES), 2),
        }
    return stats, ens_frf


def make_figure(bode_stats, ens_frf, bode_res, sled_res, quiet_res, rpy_res):
    fig, axes = plt.subplots(2, 3, figsize=(17.5, 9.8), dpi=180)

    # Panel A: 3-Axis Bode Total Score vs noise_scale
    ax = axes[0, 0]
    xs = np.array(BODE_SCALES, dtype=float)
    for m, col, mk in [("APEO", "#059669", "o"), ("LatEC", "#2563eb", "s"), ("YawMA", "#7c3aed", "^")]:
        means = [bode_stats[sc][m]["total_mean"] for sc in BODE_SCALES]
        sds = [bode_stats[sc][m]["total_sd"] for sc in BODE_SCALES]
        ax.errorbar(xs, means, yerr=sds, marker=mk, lw=2.0, capsize=4, color=col, label=f"{m} (0→{means[-1]:.0f})")
    ax.set_xlabel("noise_scale (0=OFF, 1=Biological Tier-1)", fontweight="bold")
    ax.set_ylabel("Bode Total Score [pts]", fontweight="bold")
    ax.set_title("A. 3-Axis Bode Noise Scaling (5 Seeds/Level)", fontweight="bold")
    ax.set_xticks(BODE_SCALES)
    ax.grid(True, linestyle=":", alpha=0.55)
    ax.legend(fontsize=8.2)

    # Panel B: Bode Velocity Correlation & Ensemble FRF Classic vs noise_scale
    ax = axes[0, 1]
    ax_r = ax.twinx()
    for m, col, mk in [("APEO", "#059669", "o"), ("LatEC", "#2563eb", "s"), ("YawMA", "#7c3aed", "^")]:
        corrs = [bode_stats[sc][m]["corr_mean"] for sc in BODE_SCALES]
        ax.plot(xs, corrs, marker=mk, lw=2.0, color=col, label=f"{m} r_v")
    ens_c = [bode_stats[sc]["CombinedTotal"]["ens_classic"] for sc in BODE_SCALES]
    ax_r.plot(xs, ens_c, marker="D", ls="--", lw=2.0, color="#ea580c", label=f"EnsFRF Classic ({ens_c[0]:.0f}→{ens_c[-1]:.0f})")
    ax.set_xlabel("noise_scale", fontweight="bold")
    ax.set_ylabel("Mean Velocity Correlation r_v", fontweight="bold")
    ax_r.set_ylabel("Ensemble FRF Classic [pts]", color="#ea580c", fontweight="bold")
    ax.set_ylim(0.75, 1.01)
    ax.set_xticks(BODE_SCALES)
    ax.set_title("B. Bode Linearity & Coherent Averaging Under Noise", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.55)
    l1, lb1 = ax.get_legend_handles_labels()
    l2, lb2 = ax_r.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=7.6, loc="lower left")

    # Panel C: 2g & 3g Sled Peak Flexion OFF vs 5-Seed ON
    ax = axes[0, 2]
    ux = np.arange(len(SLED_URGENCIES))
    ax.axhspan(33.0, 40.0, color="#f59e0b", alpha=0.14, label="NBDL 2g (33–40°)")
    ax.axhspan(40.0, 47.0, color="#ef4444", alpha=0.10, label="NBDL 3g (40–47°)")
    for g_val, col, ls_off in [(2.0, "#2563eb", "--"), (3.0, "#059669", "--")]:
        off_y = [next(r["peak_flex_deg"] for r in sled_res if r["g_level"] == g_val and r["urgency"] == u and r["tag"] == "OFF") for u in SLED_URGENCIES]
        on_m, on_s = [], []
        for u in SLED_URGENCIES:
            vs = [r["peak_flex_deg"] for r in sled_res if r["g_level"] == g_val and r["urgency"] == u and r["tag"] != "OFF"]
            on_m.append(np.mean(vs))
            on_s.append(np.std(vs, ddof=1))
        ax.plot(ux, off_y, ls=ls_off, marker="o", lw=1.6, color=col, alpha=0.65, label=f"{int(g_val)}g OFF")
        ax.errorbar(ux, on_m, yerr=on_s, marker="s", lw=2.2, capsize=4, color=col, label=f"{int(g_val)}g ON (5 seeds)")
    ax.set_xticks(ux)
    ax.set_xticklabels([f"u={u:+.2f}" for u in SLED_URGENCIES], fontsize=8.5)
    ax.set_ylabel("Peak Forward Flexion [deg]", fontweight="bold")
    ax.set_title("C. Zheng 2g/3g Peak Flexion: OFF vs 5-Seed Noise ON", fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.55)
    ax.legend(fontsize=7.3, loc="upper right", ncol=2)

    # Panel D: 2g Splenius Capitis Noise-OFF vs 5-Seed Noise-ON Envelope
    ax = axes[1, 0]
    for u_val, col, lbl_u in [(0.00, "#059669", "u=0.00 [Alert]"), (1.00, "#7c3aed", "u=+1.00 [Braced]")]:
        off_r = next(r for r in sled_res if r["g_level"] == 2.0 and r["urgency"] == u_val and r["tag"] == "OFF")
        on_rs = [r for r in sled_res if r["g_level"] == 2.0 and r["urgency"] == u_val and r["tag"] != "OFF"]
        t_ms = np.asarray(off_r["t_imp_ms"])
        mat = np.vstack([np.asarray(r["splen_cap_act"]) for r in on_rs])
        m_act = np.mean(mat, axis=0)
        sd_act = np.std(mat, axis=0, ddof=1)
        ax.plot(t_ms, off_r["splen_cap_act"], ls="--", lw=1.8, color=col, label=f"{lbl_u} OFF")
        ax.plot(t_ms, m_act, lw=2.0, color=col, label=f"{lbl_u} ON mean±1SD")
        ax.fill_between(t_ms, m_act - sd_act, m_act + sd_act, color=col, alpha=0.22)
    ax.set_xlabel("Time Since Impact Onset [ms]", fontweight="bold")
    ax.set_ylabel("Mean Splenius Capitis Activation", fontweight="bold")
    ax.set_title("D. 2g Splenius Capitis Envelope (OFF vs 5-Seed Noise ON)", fontweight="bold")
    ax.set_xlim(0.0, 350.0)
    ax.grid(True, linestyle=":", alpha=0.55)
    ax.legend(fontsize=7.8, loc="upper right")

    # Panel E: Quiet Stance 2x2 Ablation
    ax = axes[1, 1]
    q_labels = ["Off (0,0)", "Sensor (1,0)", "Motor (0,1)", "Both (1,1)"]
    qx = np.arange(len(q_labels))
    bw = 0.25
    for idx, (k, nm, col) in enumerate([("pitch_rms_deg", "Pitch", "#059669"), ("roll_rms_deg", "Roll", "#2563eb"), ("yaw_rms_deg", "Yaw", "#7c3aed")]):
        qm, qs = [], []
        for lbl in q_labels:
            vs = [r[k] for r in quiet_res if r["label"] == lbl]
            qm.append(np.mean(vs))
            qs.append(np.std(vs, ddof=1) if len(vs) > 1 else 0.0)
        ax.bar(qx + (idx - 1) * bw, qm, bw, yerr=qs, capsize=3, color=col, alpha=0.88, label=nm)
    ax.set_xticks(qx)
    ax.set_xticklabels(q_labels, fontsize=8.5)
    ax.set_ylabel("Demeaned RMS Sway (1–5 s) [deg]", fontweight="bold")
    ax.set_title("E. Quiet Upright Stance 2x2 Ablation (3 Seeds)", fontweight="bold")
    ax.grid(True, axis="y", linestyle=":", alpha=0.55)
    ax.legend(fontsize=8.2)

    # Panel F: 18.0 s Voluntary RPY Tracking RMS Error & Max Overshoot
    ax = axes[1, 2]
    rpy_off = next(r for r in rpy_res if r["tag"] == "OFF")
    rpy_on = [r for r in rpy_res if r["tag"] != "OFF"]
    cats = ["Pitch RMS", "Roll RMS", "Yaw RMS", "Max OvsTgt"]
    off_bars = [
        rpy_off["rms_err_vs_traj_deg"]["pitch"],
        rpy_off["rms_err_vs_traj_deg"]["roll"],
        rpy_off["rms_err_vs_traj_deg"]["yaw"],
        rpy_off["max_ovs_tgt_deg"],
    ]
    on_bars_m, on_bars_s = [], []
    for key in ("pitch", "roll", "yaw"):
        vs = [r["rms_err_vs_traj_deg"][key] for r in rpy_on]
        on_bars_m.append(np.mean(vs))
        on_bars_s.append(np.std(vs, ddof=1))
    vs_ovs = [r["max_ovs_tgt_deg"] for r in rpy_on]
    on_bars_m.append(np.mean(vs_ovs))
    on_bars_s.append(np.std(vs_ovs, ddof=1))
    rx = np.arange(len(cats))
    ax.bar(rx - 0.18, off_bars, 0.34, color="#64748b", label="Noise OFF")
    ax.bar(rx + 0.18, on_bars_m, 0.34, yerr=on_bars_s, capsize=4, color="#059669", label="Noise ON (3 seeds)")
    ax.set_xticks(rx)
    ax.set_xticklabels(cats, fontsize=8.5)
    ax.set_ylabel("Error / Overshoot [deg]", fontweight="bold")
    ax.set_title("F. 18.0 s Voluntary RPY Sweep (OFF vs 3-Seed ON)", fontweight="bold")
    ax.grid(True, axis="y", linestyle=":", alpha=0.55)
    ax.legend(fontsize=8.2)

    fig.suptitle(
        "Overnight Multi-Seed Biological Noise Robustness Suite (5-Seed Bode & Sled, 2x2 Quiet Ablation, 3-Seed 18s RPY)",
        fontsize=12.0,
        fontweight="bold",
        y=0.985,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.955])
    os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
    fig.savefig(OUT_PNG, dpi=180)
    plt.close(fig)
    if os.path.isdir(ART_DIR):
        shutil.copy2(OUT_PNG, os.path.join(ART_DIR, os.path.basename(OUT_PNG)))


def main():
    t0 = time.time()
    print(f"[NOISE SUITE] Using champion JSON: {CHAMP_C1_JSON}", flush=True)

    # Part A: 48 Bode sims
    bode_specs = [{"mode": m, "noise_scale": 0.0, "sensor_noise": False, "motor_noise": False, "seed": 42} for m in BODE_MODES]
    for sc in [0.5, 1.0, 1.5]:
        for sd in SEEDS_5:
            for m in BODE_MODES:
                bode_specs.append({"mode": m, "noise_scale": sc, "sensor_noise": True, "motor_noise": True, "seed": sd})

    # Part B: 48 Sled sims
    sled_specs = []
    for g_val in SLED_G_LEVELS:
        for u_val in SLED_URGENCIES:
            sled_specs.append({"g_level": g_val, "urgency": u_val, "sensor_noise": False, "motor_noise": False, "seed": 42, "noise_scale": 0.0})
            for sd in SEEDS_5:
                sled_specs.append({"g_level": g_val, "urgency": u_val, "sensor_noise": True, "motor_noise": True, "seed": sd, "noise_scale": 1.0})

    # Part C: 10 Quiet + 4 RPY = 14 sims
    part_c_specs = [{"kind": "quiet", "label": "Off (0,0)", "sensor_noise": False, "motor_noise": False, "seed": 42}]
    for lbl, sn, mn in [("Sensor (1,0)", True, False), ("Motor (0,1)", False, True), ("Both (1,1)", True, True)]:
        for sd in SEEDS_3:
            part_c_specs.append({"kind": "quiet", "label": lbl, "sensor_noise": sn, "motor_noise": mn, "seed": sd})
    part_c_specs.append({"kind": "rpy", "sensor_noise": False, "motor_noise": False, "seed": 42, "noise_scale": 0.0})
    for sd in SEEDS_3:
        part_c_specs.append({"kind": "rpy", "sensor_noise": True, "motor_noise": True, "seed": sd, "noise_scale": 1.0})

    with ProcessPoolExecutor(max_workers=56) as pool:
        t_a = time.time()
        print(f"[WAVE 1/3] Running {len(bode_specs)} 3-axis Passive Bode simulations across 56 cores...", flush=True)
        bode_res = list(pool.map(run_bode_noise_worker, bode_specs))
        print(f"[WAVE 1/3] Finished in {time.time() - t_a:.1f}s.", flush=True)

        t_b = time.time()
        print(f"[WAVE 2/3] Running {len(sled_specs)} Zheng 2g/3g Sled simulations across 56 cores...", flush=True)
        sled_res = list(pool.map(run_sled_noise_worker, sled_specs))
        print(f"[WAVE 2/3] Finished in {time.time() - t_b:.1f}s.", flush=True)

        t_c = time.time()
        print(f"[WAVE 3/3] Running {len(part_c_specs)} Quiet Stance + RPY Sweep simulations across 56 cores...", flush=True)
        part_c_res = list(pool.map(dispatch_part_c, part_c_specs))
        print(f"[WAVE 3/3] Finished in {time.time() - t_c:.1f}s.", flush=True)

    quiet_res = [r for r in part_c_res if r["kind"] == "quiet"]
    rpy_res = [r for r in part_c_res if r["kind"] == "rpy"]
    bode_stats, ens_frf = compute_bode_stats(bode_res)

    print("\n=== PART A: 3-AXIS PASSIVE BODE vs NOISE SCALE (5 SEEDS/LEVEL) ===", flush=True)
    for sc in BODE_SCALES:
        st = bode_stats[sc]
        print(
            f"  scale={sc:3.1f} | Combined={st['CombinedTotal']['mean']:7.2f} (EnsC={st['CombinedTotal']['ens_classic']:7.2f}) | "
            f"APEO={st['APEO']['total_mean']:7.2f}±{st['APEO']['total_sd']:5.2f} | "
            f"LatEC={st['LatEC']['total_mean']:7.2f}±{st['LatEC']['total_sd']:5.2f} | "
            f"YawMA={st['YawMA']['total_mean']:7.2f}±{st['YawMA']['total_sd']:5.2f}",
            flush=True,
        )

    print("\n=== PART B: ZHENG 2g/3g SLED NOISE ROBUSTNESS (OFF vs 5 SEEDS ON) ===", flush=True)
    sled_summary = []
    for g_val in SLED_G_LEVELS:
        for u_val in SLED_URGENCIES:
            off_r = next(r for r in sled_res if r["g_level"] == g_val and r["urgency"] == u_val and r["tag"] == "OFF")
            on_rs = [r for r in sled_res if r["g_level"] == g_val and r["urgency"] == u_val and r["tag"] != "OFF"]
            row = {
                "g_level": g_val,
                "urgency": u_val,
                "off_peak_flex_deg": off_r["peak_flex_deg"],
                "on_peak_flex_mean_deg": round(float(np.mean([r["peak_flex_deg"] for r in on_rs])), 2),
                "on_peak_flex_sd_deg": round(float(np.std([r["peak_flex_deg"] for r in on_rs], ddof=1)), 2),
                "off_splen_peak": off_r["peak_splen_cap"],
                "on_splen_peak_mean": round(float(np.mean([r["peak_splen_cap"] for r in on_rs])), 4),
                "on_splen_peak_sd": round(float(np.std([r["peak_splen_cap"] for r in on_rs], ddof=1)), 4),
                "n_aborted": sum(1 for r in on_rs if r["stop_reason"] is not None),
            }
            sled_summary.append(row)
            print(
                f"  {int(g_val)}g u={u_val:+.2f} | Flex OFF={row['off_peak_flex_deg']:+6.2f}° ON={row['on_peak_flex_mean_deg']:+6.2f}±{row['on_peak_flex_sd_deg']:4.2f}° | "
                f"Splen OFF={row['off_splen_peak']:.3f} ON={row['on_splen_peak_mean']:.3f}±{row['on_splen_peak_sd']:.3f}",
                flush=True,
            )

    print("\n=== PART C1: QUIET STANCE 2x2 ABLATION ===", flush=True)
    for lbl in ["Off (0,0)", "Sensor (1,0)", "Motor (0,1)", "Both (1,1)"]:
        sub = [r for r in quiet_res if r["label"] == lbl]
        print(
            f"  {lbl:12s} | Pitch={np.mean([r['pitch_rms_deg'] for r in sub]):.4f}° | "
            f"Roll={np.mean([r['roll_rms_deg'] for r in sub]):.4f}° | Yaw={np.mean([r['yaw_rms_deg'] for r in sub]):.4f}°",
            flush=True,
        )

    print("\n=== PART C2: 18.0s RPY SWEEP (OFF vs 3 SEEDS ON) ===", flush=True)
    for r in rpy_res:
        print(
            f"  [{r['tag']:5s}] t_end={r['t_end_s']}s flag={r['n_flagged']} | maxOvs={r['max_ovs_tgt_deg']:.2f}° | "
            f"rms={r['rms_err_vs_traj_deg']} | stop={r['stop_reason']}",
            flush=True,
        )

    make_figure(bode_stats, ens_frf, bode_res, sled_res, quiet_res, rpy_res)

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    payload = {
        "meta": {
            "champion_json": CHAMP_C1_JSON,
            "total_wall_s": round(time.time() - t0, 1),
            "bode_scales": BODE_SCALES,
            "seeds_5": SEEDS_5,
            "seeds_3": SEEDS_3,
        },
        "bode_stats": {str(k): v for k, v in bode_stats.items()},
        "bode_runs": [{k: v for k, v in r.items() if k != "freq_frf"} for r in bode_res],
        "sled_summary": sled_summary,
        "sled_runs": [{k: v for k, v in r.items() if k not in ("t_imp_ms", "pitch_rel_deg", "splen_cap_act")} for r in sled_res],
        "quiet_stance": [{k: v for k, v in r.items() if k not in ("t", "pitch_deg", "roll_deg", "yaw_deg")} for r in quiet_res],
        "rpy_runs": rpy_res,
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"\n[OK] Saved {OUT_JSON} and {OUT_PNG} in {time.time() - t0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
