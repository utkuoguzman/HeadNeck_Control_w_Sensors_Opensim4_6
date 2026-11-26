"""Shared infrastructure for the 21-Block Head-Neck Neurobiomechanical Functional Unit Test Suite.

Design Philosophy:
  Every functional block has:
    1. An isolated test scenario (prescribed inputs / selective enable_* switches).
    2. Pre-specified analytical / biophysical expected outcomes BEFORE probing.
    3. Direct readout of internal block outputs (via OpenSim Output<T> & StateVariables).
    4. Structured CheckResult records (name, expected, actual, passed, detail).
"""
from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import opensim as osim

UNIT_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(UNIT_TEST_DIR)
for _p in (REPO_ROOT, os.path.join(REPO_ROOT, "scripts"), os.path.join(REPO_ROOT, "util")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from cmaes_worker_p22 import PER_MUSCLE_72_U0, ensure_libraries_loaded
import scripts.truba_phase_a3_exact_roi_cmaes as a3ex

KUO_MODEL_PATH = os.path.join(REPO_ROOT, "osim_files", "HYOID_HeadNeckNeural_Model_KuoLigaments.osim")
CTRL_PATH = "/controllerset/VCR_CCR_Controller"
NECK_COORDS = ("pitch1", "pitch2", "roll1", "roll2", "yaw1", "yaw2")
TRUNK_COORDS = ("gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz")
EYE_COORDS = ("r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc")


@dataclass
class CheckResult:
    check_id: str
    expected: str
    actual: str
    passed: bool
    margin: str = ""


@dataclass
class BlockReport:
    block_id: str
    layer: str
    title: str
    scenario: str
    checks: List[CheckResult] = field(default_factory=list)
    wall_s: float = 0.0
    error: str = ""
    plot_data: Dict[str, Any] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return (not self.error) and bool(self.checks) and all(c.passed for c in self.checks)

    def add(self, check_id: str, expected: str, actual: str, passed: bool, margin: str = "") -> None:
        self.checks.append(CheckResult(check_id, expected, actual, bool(passed), margin))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "block_id": self.block_id,
            "layer": self.layer,
            "title": self.title,
            "scenario": self.scenario,
            "passed": self.passed,
            "wall_s": round(self.wall_s, 2),
            "error": self.error,
            "checks": [asdict(c) for c in self.checks],
        }


# ---------------------------------------------------------------------------
# Read-only OpenSim Output & State Probe Helpers
# ---------------------------------------------------------------------------
def read_out_double(component, name: str, state) -> float:
    out = osim.OutputDouble.safeDownCast(component.getOutput(name))
    if out is None:
        raise RuntimeError(f"OutputDouble '{name}' not found on {component.getName()}")
    return float(out.getValue(state))


def read_out_vec3(component, name: str, state) -> np.ndarray:
    out = osim.OutputVec3.safeDownCast(component.getOutput(name))
    if out is None:
        raise RuntimeError(f"OutputVec3 '{name}' not found on {component.getName()}")
    v = out.getValue(state)
    return np.array([float(v.get(0)), float(v.get(1)), float(v.get(2))], dtype=float)


def read_ctrl_sv(model, state, sv_name: str) -> float:
    return float(model.getStateVariableValue(state, f"{CTRL_PATH}/{sv_name}"))


def read_mahony_pry_deg(model, state) -> Tuple[float, float, float]:
    q0 = read_ctrl_sv(model, state, "obs_q0")
    q1 = read_ctrl_sv(model, state, "obs_q1")
    q2 = read_ctrl_sv(model, state, "obs_q2")
    q3 = read_ctrl_sv(model, state, "obs_q3")
    qn = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    if qn > 1e-8:
        q0 /= qn; q1 /= qn; q2 /= qn; q3 /= qn
    p = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))
    r = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))
    y = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))))
    return p, r, y


def extract_skull_zxy_deg(skull, state, R_base: Optional[osim.Rotation] = None) -> Tuple[float, float, float]:
    def _q2pry(R):
        qq = R.convertRotationToQuaternion()
        q0, q1, q2, q3 = qq.get(0), qq.get(1), qq.get(2), qq.get(3)
        p = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))
        r = math.degrees(math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))
        y = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (q0 * q2 - q3 * q1)))))
        return p, r, y

    p, r, y = _q2pry(skull.getTransformInGround(state).R())
    if R_base is not None:
        pb, rb, yb = _q2pry(R_base)
        return p - pb, r - rb, y - yb
    return p, r, y


def make_spline(ts: np.ndarray, ys: np.ndarray) -> osim.SimmSpline:
    sp = osim.SimmSpline()
    for t, y in zip(ts, ys):
        sp.addPoint(float(t), float(y))
    return sp


def smooth_step_ramp(ts: np.ndarray, t0: float, dur: float, amp: float) -> np.ndarray:
    """C^2 smoothstep cosine ramp from 0 at t0 to amp at t0+dur."""
    u = np.clip((ts - t0) / max(dur, 1e-6), 0.0, 1.0)
    return amp * 0.5 * (1.0 - np.cos(np.pi * u))


def smooth_vel_pulse_pos(ts: np.ndarray, t0: float, t_rise: float, t_hold: float, v_peak: float) -> np.ndarray:
    """Position trajectory whose derivative ramps smoothly to v_peak, holds for t_hold, then ramps to 0."""
    dt = ts[1] - ts[0]
    vel = np.zeros_like(ts)
    for i, t in enumerate(ts):
        if t < t0:
            vel[i] = 0.0
        elif t < t0 + t_rise:
            u = (t - t0) / t_rise
            vel[i] = v_peak * 0.5 * (1.0 - math.cos(math.pi * u))
        elif t < t0 + t_rise + t_hold:
            vel[i] = v_peak
        elif t < t0 + 2.0 * t_rise + t_hold:
            u = (t - (t0 + t_rise + t_hold)) / t_rise
            vel[i] = v_peak * 0.5 * (1.0 + math.cos(math.pi * u))
        else:
            vel[i] = 0.0
    pos = np.cumsum(vel) * dt
    pos -= pos[0]
    return pos


def smooth_accel_pulse_pos(ts: np.ndarray, t0: float, t_rise: float, t_hold: float, a_peak: float) -> np.ndarray:
    """Position trajectory whose 2nd derivative ramps smoothly to a_peak and holds for t_hold."""
    dt = ts[1] - ts[0]
    acc = np.zeros_like(ts)
    for i, t in enumerate(ts):
        if t < t0:
            acc[i] = 0.0
        elif t < t0 + t_rise:
            u = (t - t0) / t_rise
            acc[i] = a_peak * 0.5 * (1.0 - math.cos(math.pi * u))
        elif t < t0 + t_rise + t_hold:
            acc[i] = a_peak
        elif t < t0 + 2.0 * t_rise + t_hold:
            u = (t - (t0 + t_rise + t_hold)) / t_rise
            acc[i] = a_peak * 0.5 * (1.0 + math.cos(math.pi * u))
        else:
            acc[i] = 0.0
    vel = np.cumsum(acc) * dt
    pos = np.cumsum(vel) * dt
    pos -= pos[0]
    return pos


def build_isolated_model(
    ctrl_overrides: Optional[Dict[str, Any]] = None,
    prescribed_coords: Optional[Dict[str, osim.Function]] = None,
    lock_neck_except: Optional[Tuple[str, ...]] = None,
    enable_srs: bool = True,
    enable_gto: bool = True,
    external_torques: Optional[Dict[str, Tuple[float, float, float]]] = None,
    spring_targets: Optional[Dict[str, Tuple[float, float, float]]] = None,
) -> Tuple[osim.Model, Any, Any]:
    """Build KUO model with clean baseline parameters, custom controller overrides, and optional prescribed motions.

    external_torques: optional dict {coord_name: (t0, t_ramp, tau_Nm)} attaching pure CoordinateActuator loads
      to the untouched free-neck production topology (avoids OpenSim 4.6 PrescribedMotion MomentArmSolver distortion).
    spring_targets: optional dict {coord_name: (q_rest_rad, stiffness, viscosity)} attaching damped SpringGeneralizedForce
      holds on the untouched free-neck topology so coordinates settle smoothly to static equilibrium (qdot -> 0).
    """
    ensure_libraries_loaded()
    _, base_p = a3ex.load_base_params()
    ov = dict(base_p)
    ov.update({
        "urgency": 0.0,
        "desired_pitch": 0.0, "desired_roll": 0.0, "desired_yaw": 0.0,
        "desired_ap_shift": 0.0, "desired_lat_shift": 0.0,
        "w_pos_ap": 0.0, "w_pos_lat": 0.0,
        "Kp_shift_obs_px": 0.0, "Kv_shift_obs_vx": 0.0,
        "Kp_shift_obs_pz": 0.0, "Kv_shift_obs_vz": 0.0,
        "enable_sensor_noise": False, "enable_motor_noise": False,
    })
    if ctrl_overrides:
        ov.update(ctrl_overrides)

    model = osim.Model(KUO_MODEL_PATH)
    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        if m.getName().startswith(("r_", "l_")):
            continue
        osim.PropertyHelper.setValueBool(bool(enable_srs), m.updPropertyByName("enable_SRS"))
        for ok, mprop in a3ex.SRS_KEY_MAP.items():
            if ok in ov and m.hasProperty(mprop):
                osim.PropertyHelper.setValueDouble(float(ov[ok]), m.updPropertyByName(mprop))
        gto_sub = m.updComponent("GTO")
        osim.PropertyHelper.setValueBool(bool(enable_gto), gto_sub.updPropertyByName("enable_GTO"))

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
    import scripts.generate_standard_plot as gsp
    gsp.apply_cli_overrides_and_inspect(controller, pairs)

    for eye_c in EYE_COORDS:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    zero_fn = osim.Constant(0.0)
    for cn in TRUNK_COORDS:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_fn)

    for cn in NECK_COORDS:
        c = model.updCoordinateSet().get(cn)
        c.set_locked(False)
        c.setDefaultLocked(False)
        c.setDefaultIsPrescribed(False)

    if lock_neck_except is not None:
        for cn in NECK_COORDS:
            if cn not in lock_neck_except and not (prescribed_coords and cn in prescribed_coords):
                c = model.updCoordinateSet().get(cn)
                c.setDefaultLocked(True)

    neck_prescribed_post_init = []
    if prescribed_coords:
        for cn, fn in prescribed_coords.items():
            c = model.updCoordinateSet().get(cn)
            c.set_locked(False)
            c.setDefaultLocked(False)
            c.setPrescribedFunction(fn)
            if cn in NECK_COORDS:
                # pitch1/roll1 trigger a Simbody Stage::Dynamics cache mismatch if DefaultIsPrescribed=True
                # before initSystem(); activating setIsPrescribed(state, True) post-init avoids it cleanly.
                c.setDefaultIsPrescribed(False)
                neck_prescribed_post_init.append(cn)
            else:
                c.setDefaultIsPrescribed(True)

    if external_torques:
        pc = osim.PrescribedController()
        pc.setName("UnitTestExternalLoad")
        for cname, (t_start, t_ramp, tau_nm) in external_torques.items():
            act = osim.CoordinateActuator(cname)
            act.setName("ut_ext_" + cname)
            act.setOptimalForce(1.0)
            model.addForce(act)
            pc.addActuator(act)
            ts_r = np.linspace(0.0, 2.5, 251)
            vals = smooth_step_ramp(ts_r, t_start, t_ramp, tau_nm)
            pc.prescribeControlForActuator("ut_ext_" + cname, make_spline(ts_r, vals))
        model.addController(pc)

    if spring_targets:
        for cname, (q_rest_rad, stiff, visc) in spring_targets.items():
            sg = osim.SpringGeneralizedForce(cname)
            sg.setName("ut_spr_" + cname)
            sg.setRestLength(float(q_rest_rad))
            sg.setStiffness(float(stiff))
            sg.setViscosity(float(visc))
            model.addForce(sg)

    state = model.initSystem()
    # Ensure non-prescribed yaw1 is free (Kuo model XML defaults yaw1 prescribed)
    if not (prescribed_coords and "yaw1" in prescribed_coords) and (lock_neck_except is None or "yaw1" in lock_neck_except):
        model.updCoordinateSet().get("yaw1").setIsPrescribed(state, False)
    for cn in neck_prescribed_post_init:
        model.updCoordinateSet().get(cn).setIsPrescribed(state, True)
    a3ex._init_neck_state(model, state, ov)
    return model, controller, state


def make_manager(model, state, acc: float = 1e-4, max_step: float = 0.001) -> osim.Manager:
    mgr = osim.Manager(model)
    mgr.setWriteToStorage(False)
    mgr.setPerformAnalyses(False)
    mgr.setIntegratorMethod(5)
    mgr.setIntegratorAccuracy(acc)
    mgr.setIntegratorMinimumStepSize(1e-4)
    mgr.setIntegratorMaximumStepSize(max_step)
    state.setTime(0.0)
    mgr.initialize(state)
    return mgr


def integrate_and_sample(
    model,
    state,
    t_end: float,
    dt: float,
    probe_fn: Callable[[float, Any], Dict[str, Any]],
    step_hook: Optional[Callable[[float, Any], None]] = None,
    max_step: float = 0.001,
) -> List[Dict[str, Any]]:
    mgr = make_manager(model, state, max_step=max_step)
    samples = []
    t_now = 0.0
    model.realizeDynamics(state)
    samples.append(probe_fn(0.0, state))
    while t_now < t_end - 1e-9:
        if step_hook is not None:
            step_hook(t_now, state)
        t_now = round(min(t_now + dt, t_end), 6)
        state = mgr.integrate(t_now)
        model.realizeDynamics(state)
        samples.append(probe_fn(t_now, state))
    return samples
