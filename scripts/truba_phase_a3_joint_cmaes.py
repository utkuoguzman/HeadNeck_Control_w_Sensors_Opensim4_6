#!/usr/bin/env python3
"""Phase A3 Distributed Cooperative Multi-Axis + Sled CMA-ES (TRUBA ARF).

Optimizes controller gains on the finalized neurobiomechanical plant:
  - osim_files/HYOID_HeadNeckNeural_Model_KuoLigaments.osim
    * 67 Kuo et al. (2019) non-disc DynamicLigamentForce ligaments
    * B_k50_c30 calibrated disc-only bushings
    * 72 neck muscles with tendon-equilibrated Plan 2 SRS (k_srs=23.4, thixotropy)
    * 72 neck muscles with real Lin & Crago (2002) GTO (Layer 2 voluntary suppression)
  - Tier A vestibular cleanup LOCKED ON:
    * z_phas = 0.0, z_phas_roll = 0.0 (no central double-differentiation)
    * single peripheral tanh LN otolith saturation (phasic_oto_tanh_mps2 in [2.5, 10.0])
    * all ad-hoc high-g cliffs OFF (highg_oto_knee_mps2=0, phasic_torque_sat_nm=0,
      phasic_lead_sat_nm=0, phasic_oto_sat_mps2=0, highg_p_sc_floor=60, highg_tau_lead_add_s=0)
  - Tier B spinal/articular loops LOCKED ON:
    * enable_gto_reflex=true (K_gto_ib bounded [0.0002, 0.0008])
    * enable_renshaw=true (K_renshaw [0.05, 0.35], tau_renshaw [0.015, 0.035])
    * enable_ligament_reflex=true (K_lig_protective=0.15, p_lig_reflex=40.0)

Modes (sharing atomic cooperative state in logs/phase_a3/ via POSIX flock):
  - pitch  : 21D Pitch + Sled + Shared Tier A/B (multi-island capable)
  - roll   : 17D Roll LatEC exact 15.8s (z_phas_roll=0.0)
  - yaw    : 15D Yaw YawMA exact 15.8s
  - verify : Exact 15.8s 3-axis Bode + full 2g/3g sled + VOL60 finalist verifier
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

try:
    import fcntl
except ImportError:
    fcntl = None

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_SCRIPT_DIR)
for _p in (_ROOT, _SCRIPT_DIR, os.path.join(_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cmaes_worker_p22 import CANONICAL_TARGETS, PER_MUSCLE_72_U0, ensure_libraries_loaded
import scripts.generate_standard_plot as gsp
from scripts.run_roll_yaw_baseline import (
    build_c2_multisine_spline,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
    load_aligned_data,
)
from scripts.tune_roll_and_yaw import build_c2_zero_offset_spline_yaw
from scripts.run_zheng_2g_3g_urgency_sweep import (
    DT_LOG,
    SPLEN_CAP_NAMES,
    T_SETTLE,
    T_TOTAL,
    build_sled_spline,
)

KUO_MODEL_PATH = os.path.join(_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")
BASE_CHAMP_JSON = os.path.join(_ROOT, "logs", "se2_c1pos_3axis_champion.json")
STATE_DIR = os.path.join(_ROOT, "logs", "phase_a3")
LOCK_PATH = os.path.join(STATE_DIR, "phase_a3.lock")
BEST_PITCH_JSON = os.path.join(STATE_DIR, "phase_a3_best_pitch.json")
BEST_PITCH_ZFREE_JSON = os.path.join(STATE_DIR, "phase_a3_best_pitch_zfree.json")
BEST_ROLL_JSON = os.path.join(STATE_DIR, "phase_a3_best_roll.json")
BEST_YAW_JSON = os.path.join(STATE_DIR, "phase_a3_best_yaw.json")
FINAL_CHAMP_JSON = os.path.join(STATE_DIR, "phase_a3_champion.json")

# Locked Tier A vestibular cleanup + Tier B reflex architecture
TIER_AB_LOCKED = {
    "z_phas": 0.0,
    "z_phas_roll": 0.0,
    "phasic_oto_sat_mps2": 0.0,
    "highg_oto_knee_mps2": 0.0,
    "phasic_torque_sat_nm": 0.0,
    "phasic_lead_sat_nm": 0.0,
    "highg_p_sc_floor": 60.0,
    "highg_tau_lead_add_s": 0.0,
    "highg_sc_gain_boost": 0.0,
    "highg_phas_pole_boost": 0.0,
    "highg_phas_sust_floor": 0.0,
    "enable_gto_reflex": True,
    "enable_renshaw": True,
    "enable_ligament_reflex": True,
    "K_lig_protective": 0.15,
    "p_lig_reflex": 40.0,
    "K_gto_vol_suppress": 0.85,
    "enable_sensor_noise": False,
    "enable_motor_noise": False,
}


class _SharedLock:
    def __init__(self, path):
        self.path = path
        self.fp = None

    def __enter__(self):
        if fcntl is not None:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            self.fp = open(self.path, "a+", encoding="utf-8")
            fcntl.flock(self.fp.fileno(), fcntl.LOCK_EX)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.fp is not None:
            try:
                fcntl.flock(self.fp.fileno(), fcntl.LOCK_UN)
            finally:
                self.fp.close()
                self.fp = None


def atomic_write_json(path: str, payload: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.tmp.{os.getpid()}"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, path)


def safe_load_json(path: str, default=None):
    for attempt in range(4):
        if not os.path.exists(path):
            return default
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            if attempt == 3:
                return default
            time.sleep(0.05 * (attempt + 1))
    return default


def load_base_params():
    doc = json.load(open(BASE_CHAMP_JSON, "r", encoding="utf-8"))
    p = dict(doc["params"])
    p.update(doc.get("se2_c1pos_params", {}))
    p.update(TIER_AB_LOCKED)
    p.setdefault("phasic_oto_tanh_mps2", 8.0)
    p.setdefault("K_gto_ib", 0.0005)
    p.setdefault("K_renshaw", 0.18)
    p.setdefault("tau_renshaw", 0.022)
    return doc, p


def merge_shared_params(base_p: dict, exclude_axis: str = "") -> dict:
    p = dict(base_p)
    if exclude_axis != "pitch":
        bp = safe_load_json(BEST_PITCH_JSON)
        if bp and "axis_params" in bp:
            p.update(bp["axis_params"])
    if exclude_axis != "roll":
        br = safe_load_json(BEST_ROLL_JSON)
        if br and "axis_params" in br:
            p.update(br["axis_params"])
    if exclude_axis != "yaw":
        by = safe_load_json(BEST_YAW_JSON)
        if by and "axis_params" in by:
            p.update(by["axis_params"])
    p.update(TIER_AB_LOCKED)
    return p


def build_pitch21_specs(base_p: dict, preset: str = "default"):
    # 21D: 17 Pitch (z_phas locked at 0.0) + phasic_oto_tanh_mps2 + K_gto_ib + K_renshaw + tau_renshaw
    ov = {}
    if preset == "default":
        ov = {"K_phas": 13.0, "p_phas": 12.0, "phasic_oto_tanh_mps2": 8.0, "K_gto_ib": 0.0005, "K_renshaw": 0.18, "tau_renshaw": 0.022}
    elif preset == "lowphas":
        # Physiological LVN/MVN bandwidth (p_phas ~ 45 rad/s, moderate K_phas + tight tanh LN saturation)
        ov = {"K_phas": 7.5, "p_phas": 45.0, "K_sc": 4.8, "z_sc": 0.014, "p_sc": 55.0, "Kp_prop_pitch": 0.50, "Kp_task_pitch": 3.5, "phasic_oto_tanh_mps2": 4.2, "K_gto_ib": 0.00045, "K_renshaw": 0.20}
    elif preset == "stiffcanal":
        ov = {"K_phas": 9.5, "p_phas": 75.0, "K_sc": 5.5, "z_sc": 0.018, "p_sc": 68.0, "Kp_prop_pitch": 0.35, "Kp_task_pitch": 4.2, "phasic_oto_tanh_mps2": 3.5, "K_gto_ib": 0.00045, "K_renshaw": 0.22}
    elif preset == "highccr":
        ov = {"K_phas": 4.0, "p_phas": 45.0, "Kp_prop_pitch": 1.20, "z_ccr": 0.022, "p_ccr": 90.0, "Kp_proprioception": 0.045, "phasic_oto_tanh_mps2": 5.0, "K_gto_ib": 0.00045, "K_renshaw": 0.16}

    def x0(k, fallback=None):
        return float(ov.get(k, base_p.get(k, fallback)))

    return [
        ("tau_lead_pole",        x0("tau_lead_pole", 0.014),       0.004,  0.038,  0.0030),
        ("K_sc",                 x0("K_sc", 4.0),                  1.50,  11.00,   0.75),
        ("z_sc",                 x0("z_sc", 0.012),                0.002,  0.048,  0.0045),
        ("p_sc",                 x0("p_sc", 55.0),                15.0,  115.0,   10.0),
        ("K_phas",               x0("K_phas", 13.0),               0.50,  22.00,   1.80),
        ("p_phas",               x0("p_phas", 12.0),               1.00, 160.00,  12.00),
        ("phasic_oto_tanh_mps2", x0("phasic_oto_tanh_mps2", 8.0),  1.50,  16.00,   1.50),
        ("K_ton",                x0("K_ton", 4.0),                 1.00,  16.00,   1.30),
        ("p_ton",                x0("p_ton", 14.0),                3.00,  40.00,   3.50),
        ("Kp_prop_pitch",        x0("Kp_prop_pitch", 0.65),        0.05,   3.50,   0.25),
        ("z_ccr",                x0("z_ccr", 0.015),               0.001,  0.050,  0.0045),
        ("p_ccr",                x0("p_ccr", 70.0),               20.0,  160.0,   15.0),
        ("Kp_proprioception",    x0("Kp_proprioception", 0.035),   0.001,  0.150,  0.012),
        ("Kp_task_pitch",        x0("Kp_task_pitch", 3.8),         0.80,  12.00,   0.90),
        ("Kd_task_pitch",        x0("Kd_task_pitch", 0.55),        0.05,   3.00,   0.25),
        ("Ki_task_pitch",        x0("Ki_task_pitch", 3.5),         0.20,   8.00,   0.65),
        ("Kp_buckle",            x0("Kp_buckle", 1.5),             0.20,   4.50,   0.35),
        ("Kd_buckle",            x0("Kd_buckle", 0.15),            0.02,   0.60,   0.045),
        ("K_gto_ib",             x0("K_gto_ib", 0.0005),           0.0002, 0.0008, 0.00010),
        ("K_renshaw",            x0("K_renshaw", 0.18),            0.05,   0.35,   0.040),
        ("tau_renshaw",          x0("tau_renshaw", 0.022),         0.015,  0.035,  0.0035),
    ]


def build_roll17_specs(base_p: dict):
    # 17D Roll (z_phas_roll locked at 0.0)
    def x0(k, fb):
        return float(base_p.get(k, fb))

    return [
        ("K_sc_roll",              x0("K_sc_roll", 2.3),              0.80,  10.00, 0.75),
        ("z_sc_roll",              x0("z_sc_roll", 0.015),            0.002,  0.055, 0.0045),
        ("p_sc_roll",              x0("p_sc_roll", 30.5),            12.0,  160.0, 14.0),
        ("K_phas_roll",            x0("K_phas_roll", 4.5),            0.50,  16.00, 1.40),
        ("p_phas_roll",            x0("p_phas_roll", 9.4),            0.80, 120.00, 8.00),
        ("K_ton_roll",             x0("K_ton_roll", 0.40),            0.10,  14.00, 1.20),
        ("p_ton_roll",             x0("p_ton_roll", 10.7),            0.40,  25.00, 2.20),
        ("Kp_prop_roll",           x0("Kp_prop_roll", 2.1),           0.20,  10.00, 0.80),
        ("z_ccr_roll",             x0("z_ccr_roll", 0.007),           0.001,  0.045, 0.0040),
        ("p_ccr_roll",             x0("p_ccr_roll", 90.0),           20.0,  160.0, 15.0),
        ("Kp_proprioception_roll", x0("Kp_proprioception_roll", 0.038), 0.003, 0.120, 0.010),
        ("Kp_task_roll",           x0("Kp_task_roll", 0.40),          0.10,  10.00, 0.70),
        ("Kd_task_roll",           x0("Kd_task_roll", 2.8),           0.10,   4.50, 0.40),
        ("Ki_task_roll",           x0("Ki_task_roll", 0.52),          0.02,   4.00, 0.30),
        ("tau_lead_pole_roll",     x0("tau_lead_pole_roll", 0.018),   0.004,  0.045, 0.0040),
        ("Kp_buckle_roll",         x0("Kp_buckle_roll", 1.3),         0.20,   4.00, 0.35),
        ("Kd_buckle_roll",         x0("Kd_buckle_roll", 0.29),        0.02,   0.55, 0.045),
    ]


def build_yaw15_specs(base_p: dict):
    def x0(k, fb):
        return float(base_p.get(k, fb))

    return [
        ("tau_lead_pole_yaw",     x0("tau_lead_pole_yaw", 0.012),    0.004,  0.040, 0.0025),
        ("K_sc_yaw",              x0("K_sc_yaw", 1.3),               0.20,   6.00, 0.35),
        ("z_sc_yaw",              x0("z_sc_yaw", 0.016),             0.001,  0.060, 0.0040),
        ("p_sc_yaw",              x0("p_sc_yaw", 3.5),               0.20,  25.00, 0.80),
        ("K_ton_yaw",             x0("K_ton_yaw", 6.2),              1.00,  20.00, 1.30),
        ("p_ton_yaw",             x0("p_ton_yaw", 16.2),             2.00,  40.00, 2.50),
        ("Kp_prop_yaw",           x0("Kp_prop_yaw", 14.0),           1.00,  22.00, 1.50),
        ("z_ccr_yaw",             x0("z_ccr_yaw", 0.050),            0.001,  0.060, 0.0040),
        ("p_ccr_yaw",             x0("p_ccr_yaw", 80.0),            20.0,  180.0, 14.0),
        ("Kp_proprioception_yaw", x0("Kp_proprioception_yaw", 0.04), 0.005,  0.120, 0.009),
        ("Kp_task_yaw",           x0("Kp_task_yaw", 0.55),           0.05,   3.00, 0.15),
        ("Kd_task_yaw",           x0("Kd_task_yaw", 0.05),           0.01,   1.00, 0.040),
        ("Ki_task_yaw",           x0("Ki_task_yaw", 0.05),           0.005,  1.00, 0.030),
        ("Kp_buckle_yaw",         x0("Kp_buckle_yaw", 1.5),          0.20,   5.00, 0.35),
        ("Kd_buckle_yaw",         x0("Kd_buckle_yaw", 0.15),         0.02,   0.80, 0.045),
    ]


def _apply_dict_overrides(controller, ov: dict):
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


def _init_neck_state(model, state, ov: dict):
    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    q = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q.get(idx))
    for sv in ("pitch_error_integral", "roll_error_integral", "yaw_error_integral",
               "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"):
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
        a0 = PER_MUSCLE_72_U0.get(nm, 0.030)
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


def run_bode_kuo(params_dict: dict, mode: str, fast_5s: bool = False, wall_limit_s: float = 480.0, allow_zphas: bool = False) -> dict:
    t0 = time.time()
    ensure_libraries_loaded()
    ov = dict(params_dict)
    locked = dict(TIER_AB_LOCKED)
    if allow_zphas:
        locked.pop("z_phas", None)
    ov.update(locked)

    model = osim.Model(KUO_MODEL_PATH)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            controller = c
    _, false_bools, _ = _apply_dict_overrides(controller, ov)

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

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    if fast_5s and mode == "APEO":
        delay, ramp_duration, target_time, eval_start = 0.4, 0.8, 5.0, 1.2
    elif fast_5s and mode in ("LatEC", "YawMA"):
        # 8.3 s steady window covers >1 full cycle of 0.1209 Hz (~1.6x faster than 15.8 s)
        delay, ramp_duration, target_time, eval_start = 0.5, 1.0, 9.8, 1.5
    else:
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
        prescribed_coord_name = "spine_tx"
    elif mode == "LatEC":
        df_lat, _ = load_aligned_data()
        freqs = np.array(df_lat["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_lat["Exp_Roll_Gain_deg_per_m"], dtype=float)
        tgt_phases = np.array(df_lat["Exp_Roll_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(get_latec_amplitudes(freqs), dtype=float)
        ms_spline, _ = build_c2_multisine_spline(freqs, amps_in, phases, delay, ramp_duration, target_time)
        prescribed_coord_name = "spine_tz"
    else:
        _, df_yaw = load_aligned_data()
        freqs = np.array(df_yaw["Frequency_Hz"], dtype=float)
        tgt_gains = np.array(df_yaw["Exp_Yaw_Gain_deg_per_deg"], dtype=float)
        tgt_phases = np.array(df_yaw["Exp_Yaw_Phase_deg"], dtype=float)
        K = len(freqs)
        phases = np.array([math.pi * k * (k - 1) / K for k in range(K)], dtype=float)
        amps_in = np.array(get_yawma_amplitudes_deg(freqs), dtype=float)
        amps_sim = np.radians(amps_in)
        ms_spline, _ = build_c2_zero_offset_spline_yaw(freqs, amps_sim, phases, delay, ramp_duration, target_time)
        prescribed_coord_name = "gndyaw"

    coord_pert = model.updCoordinateSet().get(prescribed_coord_name)
    coord_pert.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_pert.setSpeedValue(state, 0.0)
    _init_neck_state(model, state, ov)

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
    cy1, cy2 = cs.get("yaw1"), cs.get("yaw2")
    cgndyaw = cs.get("gndyaw")

    t_list, p_list, r_list, yr_list, yg_list = [], [], [], [], []
    aborted = False
    abort_time_s = float(target_time)
    abort_reason = ""
    t_now = 0.0
    step = 0.005
    while t_now < target_time - 1e-9:
        if (time.time() - t0) > wall_limit_s:
            aborted = True
            abort_time_s = t_now
            abort_reason = f"wall>{wall_limit_s:.0f}s@{t_now:.2f}s"
            break
        t_now = min(t_now + step, target_time)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            aborted = True
            abort_time_s = t_now
            abort_reason = str(e).splitlines()[0]
            break
        p1d = math.degrees(cp1.getValue(state))
        p2d = math.degrees(cp2.getValue(state))
        r1d = math.degrees(cr1.getValue(state))
        r2d = math.degrees(cr2.getValue(state))
        y1d = math.degrees(cy1.getValue(state))
        y2d = math.degrees(cy2.getValue(state))
        pd = p1d + p2d
        rd = r1d + r2d
        yrd = y1d + y2d
        ygd = math.degrees(cgndyaw.getValue(state)) + yrd
        t_list.append(t_now)
        p_list.append(pd)
        r_list.append(rd)
        yr_list.append(yrd)
        yg_list.append(ygd)
        if mode == "APEO":
            if (p1d < -12.0 or p1d > 18.0 or p2d < -24.75 or p2d > 36.0 or pd < -36.75 or pd > 54.0
                or abs(r1d) > 4.5 or abs(r2d) > 24.75 or abs(rd) > 29.25
                or abs(y1d) > 28.5 or abs(y2d) > 20.25 or abs(yrd) > 48.75):
                aborted = True
                abort_time_s = t_now
                abort_reason = f"t={t_now:.2f}s p={pd:+.2f}"
                break
        else:
            if abs(pd) > 45.0 or abs(rd) > 35.0 or abs(yrd) > 48.0:
                aborted = True
                abort_time_s = t_now
                abort_reason = f"t={t_now:.2f}s r={rd:+.2f} y={yrd:+.2f}"
                break

    t_arr = np.asarray(t_list, dtype=float)
    p_arr = np.asarray(p_list, dtype=float)
    r_arr = np.asarray(r_list, dtype=float)
    yr_arr = np.asarray(yr_list, dtype=float)
    yg_arr = np.asarray(yg_list, dtype=float)
    main_arr = p_arr if mode == "APEO" else (-r_arr if mode == "LatEC" else yg_arr)
    m_post = t_arr >= eval_start
    n_post = int(np.sum(m_post))

    if aborted or n_post < 50:
        prog = max(0.0, min(1.0, abort_time_s / target_time))
        cls = 6.0 * (200000.0 + (1.0 - prog) * 50000.0)
        return {
            "mode": mode, "aborted": True, "abort_reason": abort_reason,
            "classic_score": round(cls, 2), "total_score": round(cls, 2),
            "hf_rms": 99.0, "corr_vel": 0.0, "false_bools": false_bools,
            "wall_s": round(time.time() - t0, 2),
        }

    t_fit = t_arr[m_post]
    t_rel = t_fit - delay
    y_fit = main_arr[m_post]
    cols = [np.ones_like(t_fit), t_fit - eval_start]
    for k in range(K):
        psi_k = 2.0 * math.pi * float(freqs[k]) * t_rel + float(phases[k])
        cols.append(np.sin(psi_k))
        cols.append(np.cos(psi_k))
    coeffs, _, _, _ = np.linalg.lstsq(np.column_stack(cols), y_fit, rcond=None)
    ref_fit = np.zeros_like(t_fit)
    harm_fit = np.zeros_like(t_fit)
    g_loss = 0.0
    p_loss = 0.0
    for k in range(K):
        fk = float(freqs[k])
        psi = 2.0 * np.pi * fk * t_rel + float(phases[k])
        ref_fit += float(amps_in[k]) * float(tgt_gains[k]) * np.sin(psi + math.radians(float(tgt_phases[k])))
        a_c, b_c = float(coeffs[2 + 2 * k]), float(coeffs[2 + 2 * k + 1])
        harm_fit += a_c * np.sin(psi) + b_c * np.cos(psi)
        sg = math.hypot(a_c, b_c) / float(amps_in[k])
        rph = math.degrees(math.atan2(b_c, a_c))
        ph_err = ((rph - float(tgt_phases[k]) + 180.0) % 360.0) - 180.0
        db_err = 20.0 * math.log10(max(sg, 1e-6) / float(tgt_gains[k]))
        g_loss += 100.0 * db_err * db_err
        p_loss += ph_err * ph_err

    vel_act = np.gradient(y_fit, t_fit)
    vel_hrm = np.gradient(harm_fit, t_fit)
    vel_ref = np.gradient(ref_fit, t_fit)
    hf_rms = float(np.sqrt(np.mean((vel_act - vel_hrm) ** 2)))
    rv = float(np.corrcoef(vel_act, vel_ref)[0, 1]) if np.std(vel_act) > 1e-6 else 0.0
    p_dev = float(np.max(np.abs(p_arr[m_post] - p_arr[0])))
    r_dev = float(np.max(np.abs(r_arr[m_post])))
    y_dev = float(np.max(np.abs(yr_arr[m_post])))
    if mode == "APEO":
        xt = 100.0 * (r_dev ** 2 + y_dev ** 2)
        hf_p = 5.0 * (hf_rms ** 2)
    elif mode == "LatEC":
        xt = 100.0 * (p_dev ** 2 + y_dev ** 2)
        hf_p = 25.0 * (hf_rms ** 2)
    else:
        xt = 100.0 * (p_dev ** 2 + r_dev ** 2)
        hf_p = 25.0 * (hf_rms ** 2)
    cls = g_loss + p_loss
    tot = cls + xt + hf_p
    return {
        "mode": mode, "aborted": False,
        "classic_score": round(cls, 2), "gain_loss": round(g_loss, 2), "phase_loss": round(p_loss, 2),
        "xtalk_pen": round(xt, 2), "hf_pen": round(hf_p, 2), "hf_rms": round(hf_rms, 3),
        "total_score": round(tot, 2), "corr_vel": round(rv, 4), "false_bools": false_bools,
        "wall_s": round(time.time() - t0, 2),
    }


def run_sled_kuo(params_dict: dict, g_level: float, urgency: float, allow_zphas: bool = False) -> dict:
    ensure_libraries_loaded()
    ov = dict(params_dict)
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0,
        "Kp_trans_roll": 0.0, "Kp_trans_pitch": 0.0,
        "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0,
        "shift_station_blend": 0.0, "trajectory_tau": 0.28,
        "urgency": float(urgency),
        "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
        "desired_ap_shift": 0.0, "desired_lat_shift": 0.0,
    })
    locked = dict(TIER_AB_LOCKED)
    if allow_zphas:
        locked.pop("z_phas", None)
    ov.update(locked)

    model = osim.Model(KUO_MODEL_PATH)
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
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)
    sp_tx = build_sled_spline(g_level, sign_x=-1.0)
    ctx = model.updCoordinateSet().get("spine_tx")
    ctx.setPrescribedFunction(sp_tx)
    cy1 = model.updCoordinateSet().get("yaw1")
    cy1.set_locked(False)
    cy1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    cy1.setIsPrescribed(state, False)
    ctx.setSpeedValue(state, 0.0)
    _init_neck_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False)
    mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)
    mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4)
    mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    mgr.initialize(state)

    muscles = model.updMuscles()
    splen_idx = [i for i in range(muscles.getSize()) if muscles.get(i).getName() in SPLEN_CAP_NAMES]
    neck_idx = [i for i in range(muscles.getSize()) if not muscles.get(i).getName().startswith(("r_", "l_"))]
    cs = model.getCoordinateSet()
    cp1, cp2 = cs.get("pitch1"), cs.get("pitch2")
    ts_l, f_l, sp_l, mact_l = [], [], [], []
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
        model.realizeDynamics(state)
        ts_l.append(t_now)
        f_l.append(-math.degrees(cp1.getValue(state) + cp2.getValue(state)))
        sp_l.append(sum(float(muscles.get(idx).getActivation(state)) for idx in splen_idx) / len(splen_idx))
        mact_l.append(max(float(muscles.get(idx).getActivation(state)) for idx in neck_idx))

    if stop_reason is not None or len(ts_l) < 200:
        return {"aborted": True, "stop_reason": stop_reason or "short", "pk_flex": 0.0,
                "min_ons": -99.0, "dip": 1.0, "early_excess": 1.0, "n_pk": 9, "peak_splen": 0.0, "max_act": 1.0}

    ts = np.asarray(ts_l, dtype=float)
    f_arr = np.asarray(f_l, dtype=float)
    sp_arr = np.asarray(sp_l, dtype=float)
    imp = ts >= (T_SETTLE - 1e-9)
    idx0 = int(np.where(imp)[0][0])
    t_imp_ms = (ts[imp] - T_SETTLE) * 1000.0
    f_imp = f_arr[imp] - float(f_arr[idx0])
    sp_imp = sp_arr[imp]

    pk_flex = float(np.max(f_imp))
    ons_mask = t_imp_ms <= 150.0 + 1e-9
    min_ons = float(np.min(f_imp[ons_mask])) if np.any(ons_mask) else 0.0
    peak_splen = float(np.max(sp_imp))

    dip = 0.0
    n_sp = len(sp_imp)
    for k in range(1, n_sp - 1):
        if 55.0 <= t_imp_ms[k] <= 125.0:
            if sp_imp[k] > 0.06 and sp_imp[k] > sp_imp[k - 1] and sp_imp[k] >= sp_imp[k + 1]:
                hi = min(n_sp, k + 30)
                j_min = k + int(np.argmin(sp_imp[k:hi]))
                after_mask = (np.arange(n_sp) >= j_min) & (t_imp_ms <= 260.0 + 1e-9)
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
            prom = float(sp_imp[k] - max(np.min(sp_imp[lo:k + 1]), np.min(sp_imp[k:hi])))
            if prom >= 0.015:
                n_pk += 1

    return {
        "aborted": False, "stop_reason": None,
        "settle_deg": round(float(f_arr[idx0]), 2),
        "pk_flex": round(pk_flex, 2), "min_ons": round(min_ons, 2),
        "dip": round(dip, 4), "early_excess": round(early_excess, 4),
        "n_pk": int(n_pk), "peak_splen": round(peak_splen, 4),
        "max_act": round(max(mact_l), 4) if mact_l else 0.0,
    }


def run_vol60_kuo(params_dict: dict, allow_zphas: bool = False) -> dict:
    ensure_libraries_loaded()
    ov = dict(params_dict)
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0, "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0, "shift_station_blend": 0.0,
        "urgency": 0.0, "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
    })
    locked = dict(TIER_AB_LOCKED)
    if allow_zphas:
        locked.pop("z_phas", None)
    ov.update(locked)
    model = osim.Model(KUO_MODEL_PATH)
    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            controller = c
    _apply_dict_overrides(controller, ov)
    zero_func = osim.Constant(0.0)
    for cn in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)
    cy1 = model.updCoordinateSet().get("yaw1")
    cy1.set_locked(False)
    cy1.setDefaultIsPrescribed(False)
    state = model.initSystem()
    cy1.setIsPrescribed(state, False)
    _init_neck_state(model, state, ov)

    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False)
    mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)
    mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4)
    mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    mgr.initialize(state)

    cp1, cp2 = model.getCoordinateSet().get("pitch1"), model.getCoordinateSet().get("pitch2")
    prop_dp = controller.updPropertyByName("desired_pitch")
    prop_u = controller.updPropertyByName("urgency")
    step_done = False
    ps = []
    t_now = 0.0
    stop_reason = None
    while t_now < 2.8 - 1e-9:
        if not step_done and t_now >= 0.499:
            osim.PropertyHelper.setValueDouble(math.radians(60.0), prop_dp)
            osim.PropertyHelper.setValueDouble(1.0, prop_u)
            step_done = True
        t_now = round(min(t_now + 0.005, 2.8), 4)
        try:
            state = mgr.integrate(t_now)
        except Exception as e:
            stop_reason = str(e).splitlines()[0]
            break
        ps.append(math.degrees(cp1.getValue(state) + cp2.getValue(state)))
    pa = np.asarray(ps) if ps else np.array([0.0])
    return {
        "aborted": stop_reason is not None,
        "stop_reason": stop_reason,
        "end_pitch_deg": round(float(pa[-1]), 2),
        "max_pitch_deg": round(float(np.max(pa)), 2),
        "ovs_deg": round(float(max(0.0, np.max(pa) - 60.0)), 2),
    }


def _dispatch(task: dict) -> dict:
    kind = task["kind"]
    idx = task["idx"]
    params = dict(task["params"])
    az = bool(task.get("allow_zphas", False))
    if kind == "apeo_fast":
        return {"kind": kind, "idx": idx, "res": run_bode_kuo(params, "APEO", fast_5s=True, wall_limit_s=900.0, allow_zphas=az)}
    if kind == "bode_fast":
        return {"kind": kind, "idx": idx, "mode": task["mode"], "res": run_bode_kuo(params, task["mode"], fast_5s=True, wall_limit_s=1500.0, allow_zphas=az)}
    if kind == "bode_exact":
        return {"kind": kind, "idx": idx, "mode": task["mode"], "res": run_bode_kuo(params, task["mode"], fast_5s=False, wall_limit_s=2400.0, allow_zphas=az)}
    if kind == "sled_bundle":
        t0 = time.time()
        res = {
            "s2_nb": run_sled_kuo(params, 2.0, -0.75, allow_zphas=az),
            "s2_u0": run_sled_kuo(params, 2.0, 0.00, allow_zphas=az),
            "s2_br": run_sled_kuo(params, 2.0, 0.50, allow_zphas=az),
            "s3_nb": run_sled_kuo(params, 3.0, -0.75, allow_zphas=az),
            "s3_u0": run_sled_kuo(params, 3.0, 0.00, allow_zphas=az),
        }
        res["wall_s"] = round(time.time() - t0, 2)
        return {"kind": kind, "idx": idx, "res": res}
    if kind == "sled_single":
        return {"kind": kind, "idx": idx, "g": task["g"], "u": task["u"], "res": run_sled_kuo(params, task["g"], task["u"], allow_zphas=az)}
    if kind == "vol60":
        return {"kind": kind, "idx": idx, "res": run_vol60_kuo(params, allow_zphas=az)}
    raise ValueError(f"Unknown kind: {kind}")


def score_pitch_candidate(idx: int, axis_params: dict, apeo_res: dict, sled_res: dict, is_exact: bool = False) -> dict:
    apeo_tot = float(apeo_res["total_score"])
    s2_nb = sled_res["s2_nb"]
    s2_u0 = sled_res["s2_u0"]
    s2_br = sled_res["s2_br"]
    s3_nb = sled_res["s3_nb"]
    s3_u0 = sled_res["s3_u0"]

    if any(r["aborted"] for r in (apeo_res, s2_nb, s2_u0, s2_br, s3_nb, s3_u0)):
        return {
            "idx": idx, "axis_params": axis_params, "aborted": True, "is_exact": is_exact,
            "apeo_total": round(apeo_tot, 2), "loss": round(80000.0 + apeo_tot, 2),
            "is_clean": False, "apeo_wall_s": apeo_res.get("wall_s", 0.0), "sled_wall_s": sled_res.get("wall_s", 0.0),
        }

    # 1. Bode fidelity (primary driver + soft HF penalty)
    hf_rms = float(apeo_res.get("hf_rms", 0.0))
    loss_bode = apeo_tot + 30.0 * max(0.0, hf_rms - 4.0) ** 2

    # 2. Unimodal Splenius Capitis EMG on all active impacts (2g alert, 2g brace, 3g alert)
    loss_bimodal = (
        18000.0 * (s2_u0["dip"] + s2_br["dip"] + s3_u0["dip"])
        + 10000.0 * (s2_u0["early_excess"] + s2_br["early_excess"] + s3_u0["early_excess"])
        + 1200.0 * (max(0, s2_u0["n_pk"] - 1) + max(0, s2_br["n_pk"] - 1) + max(0, s3_u0["n_pk"] - 1))
    )

    # 3. Onset backward whip guard
    loss_onset = 4000.0 * (
        max(0.0, -s2_u0["min_ons"] - 0.25) ** 2
        + max(0.0, -s2_br["min_ons"] - 0.25) ** 2
        + max(0.0, -s3_u0["min_ons"] - 0.25) ** 2
    )

    # 4. Sled kinematic monotonicity & physiological Zheng/NBDL corridors
    pk2_nb, pk2_u0, pk2_br = s2_nb["pk_flex"], s2_u0["pk_flex"], s2_br["pk_flex"]
    pk3_nb, pk3_u0 = s3_nb["pk_flex"], s3_u0["pk_flex"]
    loss_kin = (
        500.0 * max(0.0, pk2_br - pk2_u0 + 0.1) ** 2
        + 500.0 * max(0.0, pk2_u0 + 2.5 - pk3_u0) ** 2
        + 300.0 * max(0.0, 4.5 - pk2_u0) ** 2
        + 300.0 * max(0.0, pk2_u0 - 13.5) ** 2
        + 300.0 * max(0.0, 8.5 - pk3_u0) ** 2
        + 300.0 * max(0.0, pk3_u0 - 21.0) ** 2
        + 30.0 * max(0.0, 26.0 - pk2_nb) ** 2
        + 30.0 * max(0.0, pk2_nb - 40.0) ** 2
        + 30.0 * max(0.0, 36.0 - pk3_nb) ** 2
        + 30.0 * max(0.0, pk3_nb - 48.0) ** 2
        + 500.0 * max(0.0, max(s2_u0["max_act"], s3_u0["max_act"]) - 0.88) ** 2
    )

    total_loss = loss_bode + loss_bimodal + loss_onset + loss_kin
    is_clean = (
        apeo_tot <= (3000.0 if is_exact else 3400.0)
        and s2_u0["n_pk"] <= 1 and s2_br["n_pk"] <= 1 and s3_u0["n_pk"] <= 1
        and s2_u0["dip"] <= 0.015 and s3_u0["dip"] <= 0.015
        and s2_u0["min_ons"] >= -0.40 and s3_u0["min_ons"] >= -0.40
        and pk2_br <= pk2_u0 < pk2_nb and pk2_u0 + 2.0 <= pk3_u0 < pk3_nb
    )

    return {
        "idx": idx, "axis_params": axis_params, "aborted": False, "is_exact": is_exact,
        "apeo_total": round(apeo_tot, 2), "apeo_classic": apeo_res["classic_score"],
        "apeo_rv": apeo_res["corr_vel"], "apeo_hf": round(hf_rms, 3),
        "pk_2g_nb": pk2_nb, "pk_2g_u0": pk2_u0, "pk_2g_br": pk2_br,
        "pk_3g_nb": pk3_nb, "pk_3g_u0": pk3_u0,
        "n_pk": [s2_u0["n_pk"], s2_br["n_pk"], s3_u0["n_pk"]],
        "dips": [s2_u0["dip"], s2_br["dip"], s3_u0["dip"]],
        "ons": [s2_u0["min_ons"], s3_u0["min_ons"]],
        "max_act": round(max(s2_u0["max_act"], s2_br["max_act"], s3_u0["max_act"]), 3),
        "loss": round(float(total_loss), 2),
        "loss_parts": {
            "bode": round(loss_bode, 1), "bimodal": round(loss_bimodal, 1),
            "onset": round(loss_onset, 1), "kin": round(loss_kin, 1),
        },
        "is_clean": bool(is_clean),
        "apeo_wall_s": apeo_res.get("wall_s", 0.0), "sled_wall_s": sled_res.get("wall_s", 0.0),
    }


def score_roll_or_yaw(idx: int, axis_params: dict, bode_res: dict, mode: str) -> dict:
    tot = float(bode_res["total_score"])
    hf_rms = float(bode_res.get("hf_rms", 0.0))
    if bode_res["aborted"]:
        return {"idx": idx, "axis_params": axis_params, "aborted": True, "total_score": round(tot, 2),
                "loss": round(100000.0 + tot, 2), "wall_s": bode_res.get("wall_s", 0.0)}
    hf_cap = 3.5 if mode == "LatEC" else 3.0
    wall = 0.0 if hf_rms <= hf_cap else (15000.0 + 2500.0 * (hf_rms - hf_cap) ** 2)
    return {
        "idx": idx, "axis_params": axis_params, "aborted": False,
        "total_score": round(tot, 2), "classic_score": bode_res["classic_score"],
        "corr_vel": bode_res["corr_vel"], "hf_rms": round(hf_rms, 3),
        "loss": round(tot + wall, 2), "wall_s": bode_res.get("wall_s", 0.0),
    }


def publish_axis_best(json_path: str, rec: dict, island: str, gen: int, score_key: str = "loss"):
    if rec.get("aborted", False):
        return False
    with _SharedLock(LOCK_PATH):
        cur = safe_load_json(json_path)
        if cur is None or float(rec[score_key]) < float(cur.get(score_key, 1e18)):
            payload = {
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "island": island, "gen": gen,
                score_key: float(rec[score_key]),
                "metrics": {k: v for k, v in rec.items() if k != "axis_params"},
                "axis_params": dict(rec["axis_params"]),
            }
            atomic_write_json(json_path, payload)
            return True
    return False


def run_pitch_mode(pool, args):
    _, base_p = load_base_params()
    specs = build_pitch21_specs(base_p, preset=args.init_preset)
    allow_zphas = bool(args.allow_zphas)
    if allow_zphas:
        # 22D Pareto ablation: unlock central z_phas in [0.0, 0.035] alongside single tanh LN + zero high-g cliffs
        specs = list(specs) + [("z_phas", 0.020, 0.0, 0.035, 0.0050)]
    target_best_json = BEST_PITCH_ZFREE_JSON if allow_zphas else BEST_PITCH_JSON
    names = [s[0] for s in specs]
    x0 = [float(np.clip(s[1], s[2], s[3])) for s in specs]
    lbs = [float(s[2]) for s in specs]
    ubs = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]
    sigma_init = args.sigma

    # Auto-warm-start default Wave-2 island from shared Wave-1 best; keep lowphas fresh to explore high-p_phas LVN basin!
    existing_best = safe_load_json(target_best_json)
    if args.init_preset == "default" and existing_best and "axis_params" in existing_best and float(existing_best.get("loss", 1e18)) < 80000.0:
        ap_b = existing_best["axis_params"]
        x0 = [float(np.clip(ap_b.get(n, x), lb, ub)) for n, x, lb, ub in zip(names, x0, lbs, ubs)]
        sigma_init = min(args.sigma, 0.30)
        print(f"[PHASE-A3 PITCH:{args.island}] Warm-starting from shared best (loss={existing_best['loss']:.1f}, sigma={sigma_init})", flush=True)

    es = cma.CMAEvolutionStrategy(
        x0, sigma_init,
        {"bounds": [lbs, ubs], "CMA_stds": stds, "popsize": args.pop,
         "maxiter": args.gens, "seed": args.seed, "verb_disp": 0, "verb_log": 0},
    )
    archive = []
    hist_path = os.path.join(STATE_DIR, f"history_pitch_{args.island}.json")
    hist = []
    best_local = None
    t_start = time.time()

    print(f"[PHASE-A3 PITCH:{args.island}] pop={args.pop} gens={args.gens} workers={args.workers} preset={args.init_preset} seed={args.seed}", flush=True)

    for gen in range(1, args.gens + 1):
        if args.stop_epoch > 0 and time.time() >= args.stop_epoch:
            print(f"[PHASE-A3 PITCH:{args.island}] Reached stop deadline at gen {gen}; moving to exact verification.", flush=True)
            break
        if es.stop():
            print(f"[PHASE-A3 PITCH:{args.island}] CMA-ES converged at gen {gen}: {es.stop()}; restarting with fresh sigma around shared best.", flush=True)
            shared_now = merge_shared_params(base_p, exclude_axis="")
            x_re = [float(np.clip(shared_now.get(n, x), lb, ub)) for n, x, lb, ub in zip(names, x0, lbs, ubs)]
            es = cma.CMAEvolutionStrategy(
                x_re, 0.35,
                {"bounds": [lbs, ubs], "CMA_stds": stds, "popsize": args.pop,
                 "maxiter": args.gens - gen + 1, "seed": args.seed + gen * 17, "verb_disp": 0, "verb_log": 0},
            )

        t_g = time.time()
        partner_p = merge_shared_params(base_p, exclude_axis="pitch")
        xs = es.ask()
        cand_maps = [{n: float(np.clip(v, lb, ub)) for n, v, lb, ub in zip(names, vec, lbs, ubs)} for vec in xs]

        subtasks = []
        for idx, cmap in enumerate(cand_maps):
            full_p = dict(partner_p)
            full_p.update(cmap)
            subtasks.append({"kind": "apeo_fast", "idx": idx, "params": full_p, "allow_zphas": allow_zphas})
            subtasks.append({"kind": "sled_bundle", "idx": idx, "params": full_p, "allow_zphas": allow_zphas})

        outs = list(pool.map(_dispatch, subtasks))
        apeo_map = {o["idx"]: o["res"] for o in outs if o["kind"] == "apeo_fast"}
        sled_map = {o["idx"]: o["res"] for o in outs if o["kind"] == "sled_bundle"}

        evals = []
        for idx, cmap in enumerate(cand_maps):
            rec = score_pitch_candidate(idx, cmap, apeo_map[idx], sled_map[idx], is_exact=False)
            rec["gen"] = gen
            rec["island"] = args.island
            evals.append(rec)
            if not rec["aborted"]:
                archive.append(rec)

        es.tell(xs, [float(e["loss"]) for e in evals])
        archive = sorted(archive, key=lambda r: r["loss"])[:40]
        gen_best = min(evals, key=lambda r: r["loss"])
        if best_local is None or gen_best["loss"] < best_local["loss"]:
            best_local = gen_best
        published = publish_axis_best(target_best_json, gen_best, args.island, gen, "loss")

        n_ab = sum(1 for e in evals if e["aborted"])
        hist.append({
            "gen": gen, "wall_s": round(time.time() - t_g, 1), "n_ab": n_ab,
            "gen_best_loss": gen_best["loss"], "gen_best_apeo": gen_best["apeo_total"],
            "best_loss": best_local["loss"], "best_apeo": best_local["apeo_total"],
            "published": published,
        })
        atomic_write_json(hist_path, {
            "island": args.island, "elapsed_s": round(time.time() - t_start, 1),
            "best_local": best_local, "archive_top10": archive[:10], "history": hist,
        })

        print(
            f"[PITCH:{args.island} G{gen:02d}] {time.time()-t_g:5.1f}s ab={n_ab:2d} | "
            f"GenBest={gen_best['loss']:8.1f} (APEO={gen_best['apeo_total']:7.1f} rv={gen_best.get('apeo_rv',0):.3f} "
            f"2g={gen_best.get('pk_2g_u0',0):.1f}/{gen_best.get('pk_2g_nb',0):.1f} 3g={gen_best.get('pk_3g_u0',0):.1f} "
            f"nPk={gen_best.get('n_pk')} gto={gen_best['axis_params']['K_gto_ib']:.5f} kph={gen_best['axis_params']['K_phas']:.2f} "
            f"sat={gen_best['axis_params']['phasic_oto_tanh_mps2']:.2f}) | Best={best_local['loss']:8.1f} {'*SHARED*' if published else ''}",
            flush=True,
        )

    # Exact 15.8s APEO check on top 4 distinct finalists from this island
    if archive:
        partner_p = merge_shared_params(base_p, exclude_axis="pitch")
        top_cands = archive[:4]
        tasks = []
        for i, rec in enumerate(top_cands):
            fp = dict(partner_p)
            fp.update(rec["axis_params"])
            tasks.append({"kind": "bode_exact", "idx": i, "mode": "APEO", "params": fp, "allow_zphas": allow_zphas})
            tasks.append({"kind": "vol60", "idx": i, "params": fp, "allow_zphas": allow_zphas})
        ex_outs = list(pool.map(_dispatch, tasks))
        ex_bode = {o["idx"]: o["res"] for o in ex_outs if o["kind"] == "bode_exact"}
        ex_vol = {o["idx"]: o["res"] for o in ex_outs if o["kind"] == "vol60"}
        exact_summary = []
        for i, rec in enumerate(top_cands):
            exact_summary.append({
                "island": args.island, "rank": i, "fast_loss": rec["loss"], "fast_apeo": rec["apeo_total"],
                "exact_apeo": ex_bode[i], "vol60": ex_vol[i],
                "sled_summary": {k: rec.get(k) for k in ("pk_2g_nb", "pk_2g_u0", "pk_2g_br", "pk_3g_nb", "pk_3g_u0", "n_pk", "dips", "ons", "max_act")},
                "axis_params": rec["axis_params"],
            })
            print(f"  [EXACT PITCH:{args.island} #{i}] ExactAPEO={ex_bode[i]['total_score']} (rv={ex_bode[i]['corr_vel']}) VOL60={ex_vol[i]['end_pitch_deg']} ovs={ex_vol[i]['ovs_deg']}", flush=True)
        atomic_write_json(os.path.join(STATE_DIR, f"exact_finalists_pitch_{args.island}.json"), {"finalists": exact_summary})


def run_roll_or_yaw_mode(pool, args, mode: str):
    _, base_p = load_base_params()
    if mode == "LatEC":
        specs = build_roll17_specs(base_p)
        best_path = BEST_ROLL_JSON
        axis_tag = "roll"
    else:
        specs = build_yaw15_specs(base_p)
        best_path = BEST_YAW_JSON
        axis_tag = "yaw"

    names = [s[0] for s in specs]
    x0 = [float(np.clip(s[1], s[2], s[3])) for s in specs]
    lbs = [float(s[2]) for s in specs]
    ubs = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]
    sigma_init = args.sigma

    existing_best = safe_load_json(best_path)
    if existing_best and "axis_params" in existing_best and float(existing_best.get("loss", 1e18)) < 50000.0:
        ap_b = existing_best["axis_params"]
        x0 = [float(np.clip(ap_b.get(n, x), lb, ub)) for n, x, lb, ub in zip(names, x0, lbs, ubs)]
        sigma_init = min(args.sigma, 0.22)
        print(f"[PHASE-A3 {axis_tag.upper()}:{args.island}] Warm-starting from shared best (loss={existing_best['loss']:.1f}, sigma={sigma_init})", flush=True)

    es = cma.CMAEvolutionStrategy(
        x0, sigma_init,
        {"bounds": [lbs, ubs], "CMA_stds": stds, "popsize": args.pop,
         "maxiter": args.gens, "seed": args.seed, "verb_disp": 0, "verb_log": 0},
    )
    hist_path = os.path.join(STATE_DIR, f"history_{axis_tag}_{args.island}.json")
    hist = []
    best_local = None
    t_start = time.time()
    print(f"[PHASE-A3 {axis_tag.upper()}:{args.island}] pop={args.pop} gens={args.gens} workers={args.workers} seed={args.seed}", flush=True)

    for gen in range(1, args.gens + 1):
        if args.stop_epoch > 0 and time.time() >= args.stop_epoch:
            print(f"[PHASE-A3 {axis_tag.upper()}:{args.island}] Reached stop deadline at gen {gen}.", flush=True)
            break
        if es.stop():
            shared_now = merge_shared_params(base_p, exclude_axis="")
            x_re = [float(np.clip(shared_now.get(n, x), lb, ub)) for n, x, lb, ub in zip(names, x0, lbs, ubs)]
            es = cma.CMAEvolutionStrategy(
                x_re, 0.30,
                {"bounds": [lbs, ubs], "CMA_stds": stds, "popsize": args.pop,
                 "maxiter": args.gens - gen + 1, "seed": args.seed + gen * 19, "verb_disp": 0, "verb_log": 0},
            )

        t_g = time.time()
        partner_p = merge_shared_params(base_p, exclude_axis=axis_tag)
        xs = es.ask()
        cand_maps = [{n: float(np.clip(v, lb, ub)) for n, v, lb, ub in zip(names, vec, lbs, ubs)} for vec in xs]

        tasks = []
        for idx, cmap in enumerate(cand_maps):
            fp = dict(partner_p)
            fp.update(cmap)
            tasks.append({"kind": "bode_fast", "idx": idx, "mode": mode, "params": fp})
        outs = list(pool.map(_dispatch, tasks))
        bode_map = {o["idx"]: o["res"] for o in outs}

        evals = [score_roll_or_yaw(idx, cmap, bode_map[idx], mode) for idx, cmap in enumerate(cand_maps)]
        es.tell(xs, [float(e["loss"]) for e in evals])
        gen_best = min(evals, key=lambda r: r["loss"])
        if best_local is None or gen_best["loss"] < best_local["loss"]:
            best_local = gen_best
        published = publish_axis_best(best_path, gen_best, args.island, gen, "loss")

        n_ab = sum(1 for e in evals if e["aborted"])
        hist.append({
            "gen": gen, "wall_s": round(time.time() - t_g, 1), "n_ab": n_ab,
            "gen_best_tot": gen_best["total_score"], "best_tot": best_local["total_score"],
            "published": published,
        })
        atomic_write_json(hist_path, {
            "axis": axis_tag, "island": args.island, "elapsed_s": round(time.time() - t_start, 1),
            "best_local": best_local, "history": hist,
        })
        print(
            f"[{axis_tag.upper()}:{args.island} G{gen:02d}] {time.time()-t_g:5.1f}s ab={n_ab:2d} | "
            f"GenBest={gen_best['total_score']:7.2f} (cls={gen_best.get('classic_score',0):7.2f} rv={gen_best.get('corr_vel',0):.4f} hf={gen_best.get('hf_rms',0):.2f}) | "
            f"Best={best_local['total_score']:7.2f} {'*SHARED*' if published else ''}",
            flush=True,
        )


def run_verify_mode(pool, args):
    doc, base_p = load_base_params()
    merged = merge_shared_params(base_p, exclude_axis="")
    print("[PHASE-A3 VERIFY] Running exact 15.8s 3-axis Bode + full 16-case 2g/3g sled + VOL60 on merged champion...", flush=True)
    tasks = [
        {"kind": "bode_exact", "idx": 0, "mode": "APEO", "params": merged},
        {"kind": "bode_exact", "idx": 1, "mode": "LatEC", "params": merged},
        {"kind": "bode_exact", "idx": 2, "mode": "YawMA", "params": merged},
        {"kind": "vol60", "idx": 3, "params": merged},
    ]
    u_list = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0]
    tid = 4
    for g in (2.0, 3.0):
        for u in u_list:
            tasks.append({"kind": "sled_single", "idx": tid, "g": g, "u": u, "params": merged})
            tid += 1

    outs = list(pool.map(_dispatch, tasks))
    bode_res = {o["mode"]: o["res"] for o in outs if o["kind"] == "bode_exact"}
    vol_res = next(o["res"] for o in outs if o["kind"] == "vol60")
    sled_res = sorted(
        [{"g": o["g"], "u": o["u"], **o["res"]} for o in outs if o["kind"] == "sled_single"],
        key=lambda r: (r["g"], r["u"]),
    )

    comb_tot = sum(bode_res[m]["total_score"] for m in ("APEO", "LatEC", "YawMA"))
    comb_cls = sum(bode_res[m]["classic_score"] for m in ("APEO", "LatEC", "YawMA"))
    payload = {
        "source": "Phase A3 Cooperative Multi-Axis+Sled (Tier A Cleanup + Tier B SRS/GTO/Renshaw/Capsule + Kuo67 B_k50_c30)",
        "model_path": "osim_files/HYOID_HeadNeckNeural_Model_KuoLigaments.osim",
        "verified_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "se2_c1pos_params": dict(doc.get("se2_c1pos_params", {})),
        "best_sway_ap_config": dict(doc.get("best_sway_ap_config", {})),
        "tier_ab_locked": dict(TIER_AB_LOCKED),
        "verified_3axis": {
            "combined_total_score": round(comb_tot, 2),
            "combined_classic_score": round(comb_cls, 2),
            "axes": bode_res,
        },
        "verified_sled_16": sled_res,
        "verified_vol60": vol_res,
        "params": merged,
    }
    atomic_write_json(FINAL_CHAMP_JSON, payload)
    print(f"[PHASE-A3 VERIFY] CombinedTotal={comb_tot:.2f} (APEO={bode_res['APEO']['total_score']}, LatEC={bode_res['LatEC']['total_score']}, YawMA={bode_res['YawMA']['total_score']}) | VOL60={vol_res['end_pitch_deg']} (ovs={vol_res['ovs_deg']})", flush=True)
    for r in sled_res:
        print(f"  SLED {r['g']:.0f}g u={r['u']:+.2f}: flex={r['pk_flex']:+.2f} ons={r['min_ons']:+.2f} nPk={r['n_pk']} dip={r['dip']:.4f} maxAct={r['max_act']:.3f}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["pitch", "roll", "yaw", "verify"], required=True)
    ap.add_argument("--island", default="main")
    ap.add_argument("--init-preset", default="default", choices=["default", "lowphas", "stiffcanal", "highccr"])
    ap.add_argument("--pop", type=int, default=26)
    ap.add_argument("--gens", type=int, default=80)
    ap.add_argument("--sigma", type=float, default=0.45)
    ap.add_argument("--seed", type=int, default=20261125)
    ap.add_argument("--workers", type=int, default=56)
    ap.add_argument("--stop-epoch", type=int, default=0, help="Unix timestamp cutoff for clean graceful stop")
    ap.add_argument("--allow-zphas", action="store_true", help="Unlock central z_phas in [0, 0.035] for Pareto comparison (writes separate JSON)")
    args = ap.parse_args()

    os.makedirs(STATE_DIR, exist_ok=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        if args.mode == "pitch":
            run_pitch_mode(pool, args)
        elif args.mode == "roll":
            run_roll_or_yaw_mode(pool, args, "LatEC")
        elif args.mode == "yaw":
            run_roll_or_yaw_mode(pool, args, "YawMA")
        else:
            run_verify_mode(pool, args)


if __name__ == "__main__":
    main()
