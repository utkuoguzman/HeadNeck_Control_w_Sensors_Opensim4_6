#!/usr/bin/env python3
"""Exact-15.8s High-ROI Distributed Cooperative CMA-ES (TRUBA ARF — ZERO SURROGATES).

Every Bode evaluation (APEO, LatEC, YawMA) runs the EXACT full 15.8 s OpenSim simulation
(delay=0.8s, ramp=1.5s, eval_start=5.0s..15.8s). No shortened-window or ML surrogates.

Neurobiomechanical Plant (100% Biophysical, ZERO ad-hoc high-G cliffs/knees):
  - osim_files/HYOID_HeadNeckNeural_Model_KuoLigaments.osim
    * 67 Kuo et al. (2019) non-disc DynamicLigamentForce ligaments + B_k50_c30 disc bushings
    * 72 neck muscles: tendon-equilibrated Plan 2 SRS (mixed-fiber human neck bounds)
    * 72 neck muscles: Lin & Crago (2002) GTO (Layer 2 voluntary PAD suppression)
  - Tier A Clean Vestibular Nonlinearity:
    * Smooth C^infty post-integrator vestibulospinal LVN/MVN tanh ceiling (phasic_torque_tanh_nm)
    * Optional high-threshold peripheral utricular hair-cell tanh ceiling (phasic_oto_tanh_mps2 >= 12 m/s^2)
    * Biological MVN/LVN dynamic phase-lead zero z_phas in [0.0, 0.045] s (and z_phas_roll in [0.0, 0.025] s)
    * ALL ad-hoc high-G knees/cliffs LOCKED OFF (highg_oto_knee_mps2=0, phasic_torque_sat_nm=0,
      phasic_lead_sat_nm=0, phasic_oto_sat_mps2=0, highg_p_sc_floor=60, highg_tau_lead_add_s=0)
  - Tier B Spinal & Articular Loops LOCKED ON:
    * Group Ib GTO autogenic inhibition, Renshaw recurrent inhibition, cervical facet capsule reflex

Modes (sharing atomic cooperative state in logs/phase_a3_exact/ via POSIX flock):
  - pitch    : 34D high-ROI Pitch + Sled + Shared Biophysics (SRS + GTO + Renshaw + Capsule + Fusimotor + fc_vol)
  - rollyaw  : Simultaneous 18D Roll (LatEC) + 15D Yaw (YawMA) exact-15.8s wave on one 56-core node!
  - verify   : Exact 15.8s 3-axis Bode + full 16-case 2g/3g sled + VOL60 finalist verifier
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
PRIOR_A3_JSON = os.path.join(_ROOT, "logs", "phase_a3", "phase_a3_champion.json")
PRIOR_EXACT_DIR = os.path.join(_ROOT, "logs", "phase_a3_exact")
STATE_DIR = os.path.join(_ROOT, "logs", "phase_a3_v3")
LOCK_PATH = os.path.join(STATE_DIR, "phase_a3_v3.lock")
BEST_PITCH_JSON = os.path.join(STATE_DIR, "best_pitch_24d.json")
BEST_ROLL_JSON = os.path.join(STATE_DIR, "best_roll_10d.json")
BEST_YAW_JSON = os.path.join(STATE_DIR, "best_yaw_8d.json")
FINAL_CHAMP_JSON = os.path.join(STATE_DIR, "phase_a3_v3_champion.json")

# Locked clean biophysical architecture + verified Low-ROI V3 physiological constants
# (ALL ad-hoc high-G cliffs OFF, ALL Tier B loops ON, low-ROI params pinned to P26/R10/Y4 verified values)
TIER_AB_CLEAN_LOCKED = {
    "phasic_oto_sat_mps2": 0.0,
    "highg_oto_knee_mps2": 0.0,
    "phasic_torque_sat_nm": 0.0,
    "phasic_torque_tanh_nm": 0.0,
    "phasic_lead_sat_nm": 0.0,
    "highg_p_sc_floor": 60.0,
    "highg_tau_lead_add_s": 0.0,
    "highg_sc_gain_boost": 0.0,
    "highg_phas_pole_boost": 0.0,
    "highg_phas_sust_floor": 0.0,
    "highg_brace_phas_suppress": 0.0,
    "enable_gto_reflex": True,
    "enable_renshaw": True,
    "enable_ligament_reflex": True,
    "p_lig_reflex": 40.0,
    "K_gto_vol_suppress": 0.85,
    "enable_sensor_noise": False,
    "enable_motor_noise": False,
    # Locked Roll/Yaw voluntary PID defaults (u=0 during LatEC/YawMA)
    "Kp_task_roll": 11.0,
    "Kd_task_roll": 2.80,
    "Ki_task_roll": 13.0,
    "Kp_task_yaw": 5.00,
    "Kd_task_yaw": 0.80,
    "Ki_task_yaw": 5.00,
    "Ki_ton_yaw": 0.0,
    # Locked Low-ROI Pitch & Shared Biophysics (14 params -> drops Pitch 38D -> 24D so pop=28 > D!)
    "vcr_upper_share_pitch": 0.44,
    "tau_ton_int": 4.0,
    "z_ccr": 0.020,
    "p_ccr": 80.0,
    "fc_voluntary": 2.15,
    "K_subocc_shear_damp": 0.03,
    "K_gamma_dyn": 0.90,
    "K_gamma_stat": 0.70,
    "K_gto_ib": 0.00045,
    "p_gto_ib": 150.0,
    "K_renshaw": 0.16,
    "tau_renshaw": 0.022,
    "K_lig_protective": 0.09,
    "srs_tau_srs_reset": 0.11,
    # Locked Low-ROI Roll (6 params -> drops Roll 16D -> 10D, anchored on R10=3,480.7)
    "z_phas_roll": 0.0,
    "z_ccr_roll": 0.004,
    "p_ccr_roll": 85.0,
    "Kp_proprioception_roll": 0.030,
    "tau_lead_pole_roll": 0.014,
    "Kd_buckle_roll": 0.34,
    # Locked Low-ROI Yaw (4 params -> drops Yaw 12D -> 8D, anchored on Y4=1,058.7)
    "tau_lead_pole_yaw": 0.0093,
    "z_ccr_yaw": 0.020,
    "p_ccr_yaw": 93.0,
    "Kd_buckle_yaw": 0.48,
}

SRS_KEY_MAP = {
    "srs_k_srs": "k_srs",
    "srs_delta_srs": "delta_srs",
    "srs_tau_srs_reset": "tau_srs_reset",
    "srs_thix_min": "srs_thix_min",
    "srs_delta_thix_break": "delta_srs_thix_break",
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
    # Overlay verified V3 <5000 local baseline (P26 APEO=4,109 + R10 LatEC=3,481 + Y4 YawMA=1,059)
    p.update({
        # Pitch P26 (<5000 winner)
        "K_sc": 2.60, "z_sc": 0.011, "p_sc": 58.0,
        "K_phas": 9.20, "z_phas": 0.022, "p_phas": 3.30,
        "oto_reweight_a0_mps2": 10.0, "oto_reweight_tau_s": 0.200, "phasic_oto_tanh_mps2": 18.0,
        "K_ton": 4.20, "p_ton": 15.0, "Ki_ton": 2.60,
        "Kp_prop_pitch": 0.55, "Kp_proprioception": 0.016, "tau_lead_pole": 0.014,
        "Kp_task_pitch": 12.0, "Kd_task_pitch": 3.0, "Ki_task_pitch": 28.0,
        "Kp_buckle": 0.95, "Kd_buckle": 0.22,
        "srs_k_srs": 9.0, "srs_delta_srs": 0.019, "srs_thix_min": 0.16, "srs_delta_thix_break": 0.014,
        # Roll R10 (<5000 winner)
        "K_sc_roll": 4.90, "z_sc_roll": 0.014, "p_sc_roll": 11.0,
        "K_phas_roll": 4.30, "p_phas_roll": 9.5,
        "K_ton_roll": 1.80, "p_ton_roll": 14.0, "Ki_ton_roll": 0.50,
        "Kp_prop_roll": 0.55, "Kp_buckle_roll": 1.35,
        # Yaw Y4 (<5000 winner)
        "K_sc_yaw": 1.55, "z_sc_yaw": 0.022, "p_sc_yaw": 1.25,
        "K_ton_yaw": 15.7, "p_ton_yaw": 19.5,
        "Kp_prop_yaw": 12.9, "Kp_proprioception_yaw": 0.042, "Kp_buckle_yaw": 1.23,
    })
    p.update(TIER_AB_CLEAN_LOCKED)
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
    p.update(TIER_AB_CLEAN_LOCKED)
    return p


def build_pitch34_specs(base_p: dict, preset: str = "default"):
    """24D High-ROI Pitch + Sled + VOL60 + Shared SRS (V3 pruned from 38D -> 24D so pop=28 > D; anchored verbatim on <5000 winner P26=4,109.3)."""
    ov = {}
    if preset == "default":
        # Verbatim Wave 7 P26 (<5000 winner: APEO=4,109.3, rv=0.9179, hf=4.81, pMean=+1.77, VOL60=58.9)
        ov = {
            "K_sc": 2.60, "z_sc": 0.011, "p_sc": 58.0,
            "K_phas": 9.20, "z_phas": 0.022, "p_phas": 3.30,
            "oto_reweight_a0_mps2": 10.0, "oto_reweight_tau_s": 0.200, "phasic_oto_tanh_mps2": 18.0,
            "K_ton": 4.20, "p_ton": 15.0, "Ki_ton": 2.60,
            "Kp_prop_pitch": 0.55, "Kp_proprioception": 0.016, "tau_lead_pole": 0.014,
            "Kp_task_pitch": 12.0, "Kd_task_pitch": 3.0, "Ki_task_pitch": 28.0,
            "Kp_buckle": 0.95, "Kd_buckle": 0.22,
            "srs_k_srs": 9.0, "srs_delta_srs": 0.019, "srs_thix_min": 0.16, "srs_delta_thix_break": 0.014,
        }
    elif preset == "soft_srs":
        # Verbatim Wave 7 P25 (all-unimodal sleds nPk=[1,1,1,1], 2g=9.4/7.0, 3g=17.9/12.7, APEO=7,861)
        ov = {
            "K_sc": 2.00, "z_sc": 0.011, "p_sc": 54.0,
            "K_phas": 9.00, "z_phas": 0.024, "p_phas": 2.80,
            "oto_reweight_a0_mps2": 8.0, "oto_reweight_tau_s": 0.200, "phasic_oto_tanh_mps2": 11.0,
            "K_ton": 4.00, "p_ton": 14.0, "Ki_ton": 2.60,
            "Kp_prop_pitch": 0.55, "Kp_proprioception": 0.016, "tau_lead_pole": 0.014,
            "Kp_task_pitch": 12.0, "Kd_task_pitch": 3.0, "Ki_task_pitch": 28.0,
            "Kp_buckle": 0.90, "Kd_buckle": 0.21,
            "srs_k_srs": 8.0, "srs_delta_srs": 0.019, "srs_thix_min": 0.16, "srs_delta_thix_break": 0.014,
        }

    def x0(k, fb):
        return float(ov.get(k, base_p.get(k, fb)))

    return [
        # --- Brainstem Semicircular Canals (3) ---
        ("K_sc",                 x0("K_sc", 2.60),                0.40,   6.00,  0.45),
        ("z_sc",                 x0("z_sc", 0.011),               0.002,  0.040, 0.0035),
        ("p_sc",                 x0("p_sc", 58.0),               15.0,  120.0,  10.0),
        # --- Brainstem Phasic Otolith + Hill-4 Reweight + Pre-Lead Hair Tanh (6) ---
        ("K_phas",               x0("K_phas", 9.20),              1.00,  18.00,  1.20),
        ("z_phas",               x0("z_phas", 0.022),             0.000,  0.045, 0.0040),
        ("p_phas",               x0("p_phas", 3.30),              1.50,  25.00,  0.80),
        ("oto_reweight_a0_mps2", x0("oto_reweight_a0_mps2", 10.0),3.50,  14.00,  1.40),
        ("oto_reweight_tau_s",   x0("oto_reweight_tau_s", 0.200), 0.060,  0.400, 0.0450),
        ("phasic_oto_tanh_mps2", x0("phasic_oto_tanh_mps2", 18.0),4.50,  25.00,  3.00),
        # --- Brainstem Tonic Graviceptive + INC/MVN Leaky Verticality Integrator (3) ---
        ("K_ton",                x0("K_ton", 4.20),               1.50,  12.00,  0.75),
        ("p_ton",                x0("p_ton", 15.0),               3.00,  35.00,  3.00),
        ("Ki_ton",               x0("Ki_ton", 2.60),              0.50,   6.00,  0.55),
        # --- Spinal CCR + Synaptic Delay Lead Pole (3) ---
        ("Kp_prop_pitch",        x0("Kp_prop_pitch", 0.55),       0.10,   4.00,  0.22),
        ("Kp_proprioception",    x0("Kp_proprioception", 0.016),  0.003,  0.080, 0.0045),
        ("tau_lead_pole",        x0("tau_lead_pole", 0.014),      0.004,  0.030, 0.0025),
        # --- Cortical Voluntary / Bracing PID (3; strictly u>0, constrained by braced sled + VOL60) ---
        ("Kp_task_pitch",        x0("Kp_task_pitch", 12.0),       4.00,  35.00,  3.00),
        ("Kd_task_pitch",        x0("Kd_task_pitch", 3.00),       0.50,  10.00,  0.80),
        ("Ki_task_pitch",        x0("Ki_task_pitch", 28.0),       5.00,  60.00,  6.00),
        # --- Direct Passive Intersegmental Anti-Buckling (2) ---
        ("Kp_buckle",            x0("Kp_buckle", 0.95),           0.15,   3.50,  0.22),
        ("Kd_buckle",            x0("Kd_buckle", 0.22),           0.04,   0.55,  0.045),
        # --- Mixed-Fiber Neck Cross-Bridge SRS Core (4) ---
        ("srs_k_srs",            x0("srs_k_srs", 9.0),            4.0,   24.0,   1.60),
        ("srs_delta_srs",        x0("srs_delta_srs", 0.019),      0.010,  0.030, 0.0025),
        ("srs_thix_min",         x0("srs_thix_min", 0.16),        0.10,   0.45,  0.040),
        ("srs_delta_thix_break", x0("srs_delta_thix_break", 0.014),0.005, 0.040, 0.0035),
    ]


def build_roll18_specs(base_p: dict):
    """10D High-ROI Subcortical/Spinal Roll space anchored verbatim on <5000 winner R10 (LatEC=3,480.7, rv=0.9287, hf=1.89)."""
    ov = {
        "K_sc_roll": 4.90, "z_sc_roll": 0.014, "p_sc_roll": 11.0,
        "K_phas_roll": 4.30, "p_phas_roll": 9.5,
        "K_ton_roll": 1.80, "p_ton_roll": 14.0, "Ki_ton_roll": 0.50,
        "Kp_prop_roll": 0.55, "Kp_buckle_roll": 1.35,
    }
    def x0(k, fb):
        return float(ov.get(k, base_p.get(k, fb)))

    return [
        ("K_sc_roll",      x0("K_sc_roll", 4.90),      0.80,  8.00, 0.55),
        ("z_sc_roll",      x0("z_sc_roll", 0.014),     0.002, 0.045, 0.0035),
        ("p_sc_roll",      x0("p_sc_roll", 11.0),      5.0,  50.0,  2.50),
        ("K_phas_roll",    x0("K_phas_roll", 4.30),    0.50, 12.00, 0.75),
        ("p_phas_roll",    x0("p_phas_roll", 9.5),     1.50, 35.00, 2.20),
        ("K_ton_roll",     x0("K_ton_roll", 1.80),     0.30,  8.00, 0.40),
        ("p_ton_roll",     x0("p_ton_roll", 14.0),     3.00, 30.00, 2.50),
        ("Ki_ton_roll",    x0("Ki_ton_roll", 0.50),    0.00,  3.00, 0.25),
        ("Kp_prop_roll",   x0("Kp_prop_roll", 0.55),   0.10,  6.00, 0.25),
        ("Kp_buckle_roll", x0("Kp_buckle_roll", 1.35), 0.20,  3.50, 0.25),
    ]


def build_yaw15_specs(base_p: dict):
    """8D High-ROI Subcortical/Spinal Yaw space anchored verbatim on <5000 winner Y4 (YawMA=1,058.7, rv=0.9831, hf=1.95)."""
    ov = {
        "K_sc_yaw": 1.55, "z_sc_yaw": 0.022, "p_sc_yaw": 1.25,
        "K_ton_yaw": 15.7, "p_ton_yaw": 19.5,
        "Kp_prop_yaw": 12.9, "Kp_proprioception_yaw": 0.042,
        "Kp_buckle_yaw": 1.23,
    }
    def x0(k, fb):
        return float(ov.get(k, base_p.get(k, fb)))

    return [
        ("K_sc_yaw",              x0("K_sc_yaw", 1.55),              0.30,  5.00, 0.25),
        ("z_sc_yaw",              x0("z_sc_yaw", 0.022),             0.001, 0.050, 0.0035),
        ("p_sc_yaw",              x0("p_sc_yaw", 1.25),              0.30, 15.00, 0.40),
        ("K_ton_yaw",             x0("K_ton_yaw", 15.7),             2.00, 22.00, 1.00),
        ("p_ton_yaw",             x0("p_ton_yaw", 19.5),             3.00, 35.00, 2.50),
        ("Kp_prop_yaw",           x0("Kp_prop_yaw", 12.9),           1.00, 20.00, 1.00),
        ("Kp_proprioception_yaw", x0("Kp_proprioception_yaw", 0.042),0.005, 0.100, 0.007),
        ("Kp_buckle_yaw",         x0("Kp_buckle_yaw", 1.23),         0.20,  4.00, 0.25),
    ]


def _apply_srs_and_ctrl(model, ov: dict):
    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        if m.getName().startswith(("r_", "l_")):
            continue
        osim.PropertyHelper.setValueBool(True, m.updPropertyByName("enable_SRS"))
        for ok, mprop in SRS_KEY_MAP.items():
            if ok in ov and m.hasProperty(mprop):
                osim.PropertyHelper.setValueDouble(float(ov[ok]), m.updPropertyByName(mprop))
        gto_sub = m.updComponent("GTO")
        osim.PropertyHelper.setValueBool(True, gto_sub.updPropertyByName("enable_GTO"))

    controller = None
    for i in range(model.getControllerSet().getSize()):
        c = model.updControllerSet().get(i)
        if c.getConcreteClassName() == "FixationController":
            c.set_enabled(False)
        elif c.getConcreteClassName() == "HeadNeckNeuralController":
            controller = c

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
    _, false_bools, _ = gsp.apply_cli_overrides_and_inspect(controller, pairs)
    return controller, false_bools


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
        if nm.startswith(("r_", "l_")):
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


def run_bode_exact(params_dict: dict, mode: str, wall_limit_s: float = 2400.0) -> dict:
    """Strictly EXACT 15.8 s Bode evaluation (delay=0.8s, ramp=1.5s, eval_start=5.0s..15.8s)."""
    t0 = time.time()
    ensure_libraries_loaded()
    ov = dict(params_dict)
    ov.update(TIER_AB_CLEAN_LOCKED)

    model = osim.Model(KUO_MODEL_PATH)
    _, false_bools = _apply_srs_and_ctrl(model, ov)

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
    while t_now < target_time - 1e-9:
        if (time.time() - t0) > wall_limit_s:
            aborted = True
            abort_time_s = t_now
            abort_reason = f"wall>{wall_limit_s:.0f}s@{t_now:.2f}s"
            break
        t_now = min(t_now + 0.005, target_time)
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
                aborted = True; abort_time_s = t_now; abort_reason = f"ROM@{t_now:.2f}s p={pd:+.1f}"; break
        else:
            if abs(pd) > 45.0 or abs(rd) > 35.0 or abs(yrd) > 48.0:
                aborted = True; abort_time_s = t_now; abort_reason = f"ROM@{t_now:.2f}s"; break

    t_arr = np.asarray(t_list, dtype=float)
    p_arr = np.asarray(p_list, dtype=float)
    r_arr = np.asarray(r_list, dtype=float)
    yr_arr = np.asarray(yr_list, dtype=float)
    yg_arr = np.asarray(yg_list, dtype=float)
    main_arr = p_arr if mode == "APEO" else (-r_arr if mode == "LatEC" else yg_arr)
    m_post = t_arr >= eval_start

    if aborted or int(np.sum(m_post)) < 50:
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


def run_sled_exact(params_dict: dict, g_level: float, urgency: float) -> dict:
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
    ov.update(TIER_AB_CLEAN_LOCKED)

    model = osim.Model(KUO_MODEL_PATH)
    _apply_srs_and_ctrl(model, ov)

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

    # Apply 15 ms causal exponential sEMG envelope filter (tau_emg = 0.015 s) to splenius activation
    sp_env = np.empty_like(sp_imp)
    if len(sp_imp) > 0:
        sp_env[0] = sp_imp[0]
        dt_emg = float(DT_LOG)
        alpha_emg = dt_emg / (0.015 + dt_emg)
        for k_e in range(1, len(sp_imp)):
            sp_env[k_e] = sp_env[k_e - 1] + alpha_emg * (sp_imp[k_e] - sp_env[k_e - 1])
    pk_flex = float(np.max(f_imp))
    ons_mask = t_imp_ms <= 150.0 + 1e-9
    min_ons = float(np.min(f_imp[ons_mask])) if np.any(ons_mask) else 0.0
    peak_splen = float(np.max(sp_env))

    dip = 0.0
    n_sp = len(sp_env)
    for k in range(1, n_sp - 1):
        if 55.0 <= t_imp_ms[k] <= 125.0:
            if sp_env[k] > 0.06 and sp_env[k] > sp_env[k - 1] and sp_env[k] >= sp_env[k + 1]:
                hi = min(n_sp, k + 30)
                j_min = k + int(np.argmin(sp_env[k:hi]))
                after_mask = (np.arange(n_sp) >= j_min) & (t_imp_ms <= 260.0 + 1e-9)
                if np.any(after_mask):
                    after_max = float(np.max(sp_env[after_mask]))
                    if after_max > sp_env[j_min] + 0.002:
                        dip = max(dip, float(sp_env[k] - sp_env[j_min]))

    m_early = (t_imp_ms >= 50.0) & (t_imp_ms <= 115.0)
    m_main = (t_imp_ms >= 120.0) & (t_imp_ms <= 250.0)
    max_early = float(np.max(sp_env[m_early])) if np.any(m_early) else 0.0
    max_main = float(np.max(sp_env[m_main])) if np.any(m_main) else 0.0
    early_excess = max(0.0, max_early - max_main)

    n_pk = 0
    for k in range(1, n_sp - 1):
        if sp_env[k] > 0.08 and sp_env[k] > sp_env[k - 1] and sp_env[k] >= sp_env[k + 1]:
            lo = max(0, k - 25)
            hi = min(n_sp, k + 26)
            prom = float(sp_env[k] - max(np.min(sp_env[lo:k + 1]), np.min(sp_env[k:hi])))
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


def run_vol60_exact(params_dict: dict) -> dict:
    ensure_libraries_loaded()
    ov = dict(params_dict)
    ov.update({
        "w_pos_ap": 0.0, "w_pos_lat": 0.0, "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0, "shift_station_blend": 0.0,
        "urgency": 0.0, "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
    })
    ov.update(TIER_AB_CLEAN_LOCKED)
    model = osim.Model(KUO_MODEL_PATH)
    controller, _ = _apply_srs_and_ctrl(model, ov)
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
    mgr.setWriteToStorage(False); mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5); mgr.setIntegratorAccuracy(1e-4)
    mgr.setIntegratorMinimumStepSize(1e-4); mgr.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0); mgr.initialize(state)

    cp1, cp2 = model.getCoordinateSet().get("pitch1"), model.getCoordinateSet().get("pitch2")
    prop_dp = controller.updPropertyByName("desired_pitch")
    prop_u = controller.updPropertyByName("urgency")
    step_done = False; ps = []; t_now = 0.0; stop_reason = None
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
        ps.append(math.degrees(cp1.getValue(state) + cp2.getValue(state)))
    pa = np.asarray(ps) if ps else np.array([0.0])
    return {
        "aborted": stop_reason is not None, "stop_reason": stop_reason,
        "end_pitch_deg": round(float(pa[-1]), 2), "max_pitch_deg": round(float(np.max(pa)), 2),
        "ovs_deg": round(float(max(0.0, np.max(pa) - 60.0)), 2),
    }


def _dispatch(task: dict) -> dict:
    kind = task["kind"]
    idx = task["idx"]
    params = dict(task["params"])
    if kind == "bode_exact":
        return {"kind": kind, "idx": idx, "mode": task["mode"], "res": run_bode_exact(params, task["mode"], wall_limit_s=2400.0)}
    if kind == "sled_bundle":
        t0 = time.time()
        res = {
            "s2_u0": run_sled_exact(params, 2.0, 0.00),
            "s2_br": run_sled_exact(params, 2.0, 0.50),
            "s3_u0": run_sled_exact(params, 3.0, 0.00),
            "s3_br": run_sled_exact(params, 3.0, 0.50),
            "vol60": run_vol60_exact(params),
        }
        res["wall_s"] = round(time.time() - t0, 2)
        return {"kind": kind, "idx": idx, "res": res}
    if kind == "sled_single":
        return {"kind": kind, "idx": idx, "g": task["g"], "u": task["u"], "res": run_sled_exact(params, task["g"], task["u"])}
    if kind == "vol60":
        return {"kind": kind, "idx": idx, "res": run_vol60_exact(params)}
    raise ValueError(f"Unknown kind: {kind}")


def score_pitch_candidate(idx: int, axis_params: dict, apeo_res: dict, sled_res: dict) -> dict:
    apeo_tot = float(apeo_res["total_score"])
    s2_u0, s2_br = sled_res["s2_u0"], sled_res["s2_br"]
    s3_u0, s3_br = sled_res["s3_u0"], sled_res["s3_br"]
    vol60 = sled_res["vol60"]

    if any(r["aborted"] for r in (apeo_res, s2_u0, s2_br, s3_u0, s3_br, vol60)):
        return {
            "idx": idx, "axis_params": axis_params, "aborted": True,
            "apeo_total": round(apeo_tot, 2), "loss": round(80000.0 + apeo_tot, 2),
            "is_clean": False, "apeo_wall_s": apeo_res.get("wall_s", 0.0), "sled_wall_s": sled_res.get("wall_s", 0.0),
        }

    # 1. Exact 15.8s APEO Bode fidelity + soft HF distortion guard
    hf_rms = float(apeo_res.get("hf_rms", 0.0))
    loss_bode = apeo_tot + 40.0 * max(0.0, hf_rms - 4.5) ** 2

    # 2. Unimodal Splenius Capitis sEMG across ALL 4 awake u >= 0 sled conditions
    loss_bimodal = (
        18000.0 * (s2_u0["dip"] + s2_br["dip"] + s3_u0["dip"] + s3_br["dip"])
        + 10000.0 * (s2_u0["early_excess"] + s2_br["early_excess"] + s3_u0["early_excess"] + s3_br["early_excess"])
        + 1200.0 * (max(0, s2_u0["n_pk"] - 1) + max(0, s2_br["n_pk"] - 1) + max(0, s3_u0["n_pk"] - 1) + max(0, s3_br["n_pk"] - 1))
    )

    # 3. Zero backward onset whip guard
    loss_onset = 4000.0 * (
        max(0.0, -s2_u0["min_ons"] - 0.25) ** 2
        + max(0.0, -s2_br["min_ons"] - 0.25) ** 2
        + max(0.0, -s3_u0["min_ons"] - 0.25) ** 2
        + max(0.0, -s3_br["min_ons"] - 0.25) ** 2
    )

    # 4. Physiological Zheng/NBDL u in [0, 1] sled flexion corridors, strict monotonicity & VOL60 voluntary tracking
    pk2_u0, pk2_br = s2_u0["pk_flex"], s2_br["pk_flex"]
    pk3_u0, pk3_br = s3_u0["pk_flex"], s3_br["pk_flex"]
    v60_end, v60_ovs = float(vol60["end_pitch_deg"]), float(vol60["ovs_deg"])
    loss_vol60 = (
        200.0 * max(0.0, 58.0 - v60_end) ** 2
        + 150.0 * max(0.0, v60_end - 62.0) ** 2
        + 150.0 * max(0.0, v60_ovs - 2.0) ** 2
    )
    loss_kin = (
        500.0 * max(0.0, pk2_br - pk2_u0 + 4.0) ** 2
        + 500.0 * max(0.0, pk3_br - pk3_u0 + 6.0) ** 2
        + 500.0 * max(0.0, pk2_u0 + 4.0 - pk3_u0) ** 2
        + 350.0 * max(0.0, 20.0 - pk2_u0) ** 2
        + 350.0 * max(0.0, pk2_u0 - 38.0) ** 2
        + 350.0 * max(0.0, 28.0 - pk3_u0) ** 2
        + 350.0 * max(0.0, pk3_u0 - 46.0) ** 2
        + 250.0 * max(0.0, 3.5 - pk2_br) ** 2
        + 250.0 * max(0.0, pk2_br - 12.5) ** 2
        + 250.0 * max(0.0, 6.5 - pk3_br) ** 2
        + 250.0 * max(0.0, pk3_br - 19.0) ** 2
        + 500.0 * max(0.0, max(s2_u0["max_act"], s2_br["max_act"], s3_u0["max_act"], s3_br["max_act"]) - 0.88) ** 2
        + loss_vol60
    )

    total_loss = loss_bode + loss_bimodal + loss_onset + loss_kin
    is_clean = (
        apeo_tot <= 3000.0
        and s2_u0["n_pk"] <= 1 and s2_br["n_pk"] <= 1 and s3_u0["n_pk"] <= 1 and s3_br["n_pk"] <= 1
        and s2_u0["dip"] <= 0.015 and s3_u0["dip"] <= 0.015 and s2_br["dip"] <= 0.015 and s3_br["dip"] <= 0.015
        and s2_u0["min_ons"] >= -0.40 and s3_u0["min_ons"] >= -0.40
        and 20.0 <= pk2_u0 <= 38.0 and 28.0 <= pk3_u0 <= 46.0
        and 3.5 <= pk2_br <= 12.5 and 6.5 <= pk3_br <= 19.0
        and pk2_br + 4.0 <= pk2_u0 and pk3_br + 6.0 <= pk3_u0 and pk2_u0 + 4.0 <= pk3_u0
        and v60_end >= 57.5 and v60_ovs <= 3.0
    )

    return {
        "idx": idx, "axis_params": axis_params, "aborted": False,
        "apeo_total": round(apeo_tot, 2), "apeo_classic": apeo_res["classic_score"],
        "apeo_gain_loss": apeo_res.get("gain_loss", 0.0), "apeo_phase_loss": apeo_res.get("phase_loss", 0.0),
        "apeo_rv": apeo_res["corr_vel"], "apeo_hf": round(hf_rms, 3),
        "pk_2g_u0": pk2_u0, "pk_2g_br": pk2_br,
        "pk_3g_u0": pk3_u0, "pk_3g_br": pk3_br,
        "vol60_end": v60_end, "vol60_ovs": v60_ovs,
        "n_pk": [s2_u0["n_pk"], s2_br["n_pk"], s3_u0["n_pk"], s3_br["n_pk"]],
        "dips": [s2_u0["dip"], s2_br["dip"], s3_u0["dip"], s3_br["dip"]],
        "ons": [s2_u0["min_ons"], s3_u0["min_ons"]],
        "max_act": round(max(s2_u0["max_act"], s2_br["max_act"], s3_u0["max_act"], s3_br["max_act"]), 3),
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
    specs = build_pitch34_specs(base_p, preset=args.init_preset)
    names = [s[0] for s in specs]
    x0 = [float(np.clip(s[1], s[2], s[3])) for s in specs]
    lbs = [float(s[2]) for s in specs]
    ubs = [float(s[3]) for s in specs]
    stds = [float(s[4]) for s in specs]
    sigma_init = args.sigma

    existing_best = safe_load_json(BEST_PITCH_JSON)
    if args.init_preset in ("default", "low_tanh") and existing_best and "axis_params" in existing_best and float(existing_best.get("loss", 1e18)) < 80000.0:
        ap_b = dict(existing_best["axis_params"])
        if args.init_preset == "low_tanh":
            ap_b["phasic_torque_tanh_nm"] = 4.8
            ap_b["K_phas"] = max(15.5, float(ap_b.get("K_phas", 15.5)))
            ap_b["K_lig_protective"] = min(0.08, float(ap_b.get("K_lig_protective", 0.08)))
        x0 = [float(np.clip(ap_b.get(n, x), lb, ub)) for n, x, lb, ub in zip(names, x0, lbs, ubs)]
        sigma_init = min(args.sigma, 0.42 if args.init_preset == "low_tanh" else 0.35)
        print(f"[EXACT-34D PITCH:{args.island}] Warm-starting ({args.init_preset}) from shared best (loss={existing_best['loss']:.1f}, sigma={sigma_init})", flush=True)

    es = cma.CMAEvolutionStrategy(
        x0, sigma_init,
        {"bounds": [lbs, ubs], "CMA_stds": stds, "popsize": args.pop,
         "maxiter": args.gens, "seed": args.seed, "verb_disp": 0, "verb_log": 0},
    )
    hist_path = os.path.join(STATE_DIR, f"history_pitch34_{args.island}.json")
    hist = []
    best_local = None
    t_start = time.time()
    print(f"[EXACT-34D PITCH:{args.island}] 34D EXACT 15.8s | pop={args.pop} gens={args.gens} workers={args.workers} preset={args.init_preset} seed={args.seed}", flush=True)

    for gen in range(1, args.gens + 1):
        if args.stop_epoch > 0 and time.time() >= args.stop_epoch:
            print(f"[EXACT-34D PITCH:{args.island}] Reached stop deadline at gen {gen}.", flush=True)
            break
        if es.stop():
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
            subtasks.append({"kind": "bode_exact", "idx": idx, "mode": "APEO", "params": full_p})
            subtasks.append({"kind": "sled_bundle", "idx": idx, "params": full_p})

        outs = list(pool.map(_dispatch, subtasks))
        apeo_map = {o["idx"]: o["res"] for o in outs if o["kind"] == "bode_exact"}
        sled_map = {o["idx"]: o["res"] for o in outs if o["kind"] == "sled_bundle"}

        evals = []
        for idx, cmap in enumerate(cand_maps):
            rec = score_pitch_candidate(idx, cmap, apeo_map[idx], sled_map[idx])
            rec["gen"] = gen
            rec["island"] = args.island
            evals.append(rec)

        es.tell(xs, [float(e["loss"]) for e in evals])
        gen_best = min(evals, key=lambda r: r["loss"])
        if best_local is None or gen_best["loss"] < best_local["loss"]:
            best_local = gen_best
        published = publish_axis_best(BEST_PITCH_JSON, gen_best, args.island, gen, "loss")

        n_ab = sum(1 for e in evals if e["aborted"])
        valid_losses = [float(e["loss"]) for e in evals if not e["aborted"]]
        pop_mean = float(np.mean(valid_losses)) if valid_losses else float(gen_best["loss"])
        pop_med = float(np.median(valid_losses)) if valid_losses else float(gen_best["loss"])
        hist.append({
            "gen": gen, "wall_s": round(time.time() - t_g, 1), "n_ab": n_ab,
            "gen_best_loss": gen_best["loss"], "gen_best_apeo": gen_best["apeo_total"],
            "pop_mean": round(pop_mean, 2), "pop_median": round(pop_med, 2),
            "best_loss": best_local["loss"], "best_apeo": best_local["apeo_total"],
            "published": published,
        })
        atomic_write_json(hist_path, {
            "island": args.island, "elapsed_s": round(time.time() - t_start, 1),
            "best_local": best_local, "history": hist,
        })
        ap = gen_best["axis_params"]
        print(
            f"[EXACT36:{args.island} G{gen:02d}] {time.time()-t_g:5.1f}s ab={n_ab:2d} | "
            f"Loss={gen_best['loss']:7.1f} mean={pop_mean:7.1f} med={pop_med:7.1f} (APEO={gen_best['apeo_total']:6.1f} G={gen_best.get('apeo_gain_loss',0):5.0f} P={gen_best.get('apeo_phase_loss',0):5.0f} rv={gen_best.get('apeo_rv',0):.3f} "
            f"2g={gen_best.get('pk_2g_u0',0):.1f}/{gen_best.get('pk_2g_br',0):.1f} 3g={gen_best.get('pk_3g_u0',0):.1f}/{gen_best.get('pk_3g_br',0):.1f} v60={gen_best.get('vol60_end',0):.1f} nPk={gen_best.get('n_pk')} "
            f"a0={ap['oto_reweight_a0_mps2']:.1f} uSh={ap['vcr_upper_share_pitch']:.2f} hT={ap['phasic_oto_tanh_mps2']:.0f} ksrs={ap['srs_k_srs']:.1f}) "
            f"| Best={best_local['loss']:7.1f} {'*SHARED*' if published else ''}",
            flush=True,
        )


def run_rollyaw_mode(pool, args):
    """Runs 18D Roll (LatEC) + 15D Yaw (YawMA) SIMULTANEOUSLY on exact 15.8 s in every generation!"""
    _, base_p = load_base_params()
    r_specs = build_roll18_specs(base_p)
    y_specs = build_yaw15_specs(base_p)

    def make_es(specs, best_path, seed_off):
        names = [s[0] for s in specs]
        x0 = [float(np.clip(s[1], s[2], s[3])) for s in specs]
        lbs = [float(s[2]) for s in specs]
        ubs = [float(s[3]) for s in specs]
        stds = [float(s[4]) for s in specs]
        sig = args.sigma
        ex = safe_load_json(best_path)
        if ex and "axis_params" in ex and float(ex.get("loss", 1e18)) < 50000.0:
            x0 = [float(np.clip(ex["axis_params"].get(n, x), lb, ub)) for n, x, lb, ub in zip(names, x0, lbs, ubs)]
            sig = min(args.sigma, 0.25)
        es = cma.CMAEvolutionStrategy(
            x0, sig,
            {"bounds": [lbs, ubs], "CMA_stds": stds, "popsize": args.pop,
             "maxiter": args.gens, "seed": args.seed + seed_off, "verb_disp": 0, "verb_log": 0},
        )
        return names, lbs, ubs, stds, es

    r_names, r_lbs, r_ubs, r_stds, r_es = make_es(r_specs, BEST_ROLL_JSON, 101)
    y_names, y_lbs, y_ubs, y_stds, y_es = make_es(y_specs, BEST_YAW_JSON, 202)
    best_r = None
    best_y = None
    print(f"[EXACT-ROLLYAW:{args.island}] 18D Roll + 15D Yaw EXACT 15.8s | pop={args.pop}+pop={args.pop} workers={args.workers}", flush=True)

    for gen in range(1, args.gens + 1):
        if args.stop_epoch > 0 and time.time() >= args.stop_epoch:
            print(f"[EXACT-ROLLYAW:{args.island}] Reached stop deadline at gen {gen}.", flush=True)
            break
        t_g = time.time()
        shared_p = merge_shared_params(base_p, exclude_axis="")
        r_xs = r_es.ask()
        y_xs = y_es.ask()
        r_maps = [{n: float(np.clip(v, lb, ub)) for n, v, lb, ub in zip(r_names, vec, r_lbs, r_ubs)} for vec in r_xs]
        y_maps = [{n: float(np.clip(v, lb, ub)) for n, v, lb, ub in zip(y_names, vec, y_lbs, y_ubs)} for vec in y_xs]

        tasks = []
        for idx, cmap in enumerate(r_maps):
            fp = dict(shared_p); fp.update(cmap)
            tasks.append({"kind": "bode_exact", "idx": idx, "mode": "LatEC", "params": fp})
        for idx, cmap in enumerate(y_maps):
            fp = dict(shared_p); fp.update(cmap)
            tasks.append({"kind": "bode_exact", "idx": idx, "mode": "YawMA", "params": fp})

        outs = list(pool.map(_dispatch, tasks))
        r_res = {o["idx"]: o["res"] for o in outs if o["mode"] == "LatEC"}
        y_res = {o["idx"]: o["res"] for o in outs if o["mode"] == "YawMA"}

        r_evals = [score_roll_or_yaw(i, c, r_res[i], "LatEC") for i, c in enumerate(r_maps)]
        y_evals = [score_roll_or_yaw(i, c, y_res[i], "YawMA") for i, c in enumerate(y_maps)]
        r_es.tell(r_xs, [float(e["loss"]) for e in r_evals])
        y_es.tell(y_xs, [float(e["loss"]) for e in y_evals])

        rb = min(r_evals, key=lambda r: r["loss"])
        yb = min(y_evals, key=lambda r: r["loss"])
        if best_r is None or rb["loss"] < best_r["loss"]:
            best_r = rb
        if best_y is None or yb["loss"] < best_y["loss"]:
            best_y = yb
        pr = publish_axis_best(BEST_ROLL_JSON, rb, args.island, gen, "loss")
        py = publish_axis_best(BEST_YAW_JSON, yb, args.island, gen, "loss")

        r_valid = [float(e["loss"]) for e in r_evals if not e["aborted"]]
        y_valid = [float(e["loss"]) for e in y_evals if not e["aborted"]]
        r_mean = float(np.mean(r_valid)) if r_valid else float(rb["total_score"])
        r_med = float(np.median(r_valid)) if r_valid else float(rb["total_score"])
        y_mean = float(np.mean(y_valid)) if y_valid else float(yb["total_score"])
        y_med = float(np.median(y_valid)) if y_valid else float(yb["total_score"])
        print(
            f"[ROLLYAW:{args.island} G{gen:02d}] {time.time()-t_g:5.1f}s | "
            f"Roll={rb['total_score']:7.1f} mean={r_mean:7.1f} med={r_med:7.1f} (rv={rb.get('corr_vel',0):.4f} best={best_r['total_score']:7.1f}{'*' if pr else ''}) | "
            f"Yaw={yb['total_score']:7.1f} mean={y_mean:7.1f} med={y_med:7.1f} (rv={yb.get('corr_vel',0):.4f} best={best_y['total_score']:7.1f}{'*' if py else ''})",
            flush=True,
        )


def run_verify_mode(pool, args):
    doc, base_p = load_base_params()
    merged = merge_shared_params(base_p, exclude_axis="")
    print("[EXACT-VERIFY] Running exact 15.8s 3-axis Bode + full 16-case 2g/3g sled + VOL60 on merged champion...", flush=True)
    tasks = [
        {"kind": "bode_exact", "idx": 0, "mode": "APEO", "params": merged},
        {"kind": "bode_exact", "idx": 1, "mode": "LatEC", "params": merged},
        {"kind": "bode_exact", "idx": 2, "mode": "YawMA", "params": merged},
        {"kind": "vol60", "idx": 3, "params": merged},
    ]
    u_list = [0.0, 0.25, 0.5, 0.75, 1.0]
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
    srs_params = {SRS_KEY_MAP[k]: merged[k] for k in SRS_KEY_MAP if k in merged}
    payload = {
        "source": "Phase A3-Exact 34D+18D+15D Cooperative CMA-ES (Zero Surrogates, Post-Integrator LVN Tanh + Mixed-Fiber SRS + Tier B)",
        "model_path": "osim_files/HYOID_HeadNeckNeural_Model_KuoLigaments.osim",
        "verified_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "se2_c1pos_params": dict(doc.get("se2_c1pos_params", {})),
        "best_sway_ap_config": dict(doc.get("best_sway_ap_config", {})),
        "tier_ab_locked": dict(TIER_AB_CLEAN_LOCKED),
        "srs_muscle_params": srs_params,
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
    print(f"[EXACT-VERIFY] CombinedTotal={comb_tot:.2f} (APEO={bode_res['APEO']['total_score']}, LatEC={bode_res['LatEC']['total_score']}, YawMA={bode_res['YawMA']['total_score']}) | VOL60={vol_res['end_pitch_deg']} (ovs={vol_res['ovs_deg']})", flush=True)
    for r in sled_res:
        print(f"  SLED {r['g']:.0f}g u={r['u']:+.2f}: flex={r['pk_flex']:+.2f} ons={r['min_ons']:+.2f} nPk={r['n_pk']} dip={r['dip']:.4f} maxAct={r['max_act']:.3f}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["pitch", "rollyaw", "verify"], required=True)
    ap.add_argument("--island", default="main")
    ap.add_argument("--init-preset", default="default", choices=["default", "soft_srs", "high_bw", "low_tanh"])
    ap.add_argument("--pop", type=int, default=28)
    ap.add_argument("--gens", type=int, default=40)
    ap.add_argument("--sigma", type=float, default=0.50)
    ap.add_argument("--seed", type=int, default=20261125)
    ap.add_argument("--workers", type=int, default=56)
    ap.add_argument("--stop-epoch", type=int, default=0)
    args = ap.parse_args()

    os.makedirs(STATE_DIR, exist_ok=True)
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        if args.mode == "pitch":
            run_pitch_mode(pool, args)
        elif args.mode == "rollyaw":
            run_rollyaw_mode(pool, args)
        else:
            run_verify_mode(pool, args)


if __name__ == "__main__":
    main()
