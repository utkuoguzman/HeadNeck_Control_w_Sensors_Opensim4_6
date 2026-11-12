import os
import sys
import math
import json
import csv
import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "util"), os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import opensim as osim
from cmaes_worker_p22 import ensure_libraries_loaded, PER_MUSCLE_72_U0
from run_roll_yaw_baseline import (
    P15_RUN4,
    load_aligned_data,
    get_latec_amplitudes,
    get_yawma_amplitudes_deg,
)


def build_c2_zero_offset_multisine_spline(freqs, amps, phases, delay, ramp_duration, target_time, spline_dt=0.0025):
    """
    Constructs a C^2 quintic-windowed multisine spline x(t) = w(tau) * y(t)
    so that for t >= delay + ramp_duration, x_offset == 0.000000 and velocity drift == 0.000000,
    with C^2 continuity at t = delay and t = delay + ramp_duration.
    """
    ms_spline = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        ms_spline.addPoint(float(t), 0.0)

    omegas = 2.0 * np.pi * freqs
    for t in np.arange(delay, target_time + 0.02, spline_dt):
        dt_rel = t - delay
        if dt_rel < ramp_duration:
            tau = dt_rel / ramp_duration
            w = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
        else:
            w = 1.0
        y = float(np.sum(amps * np.sin(omegas * dt_rel + phases)))
        ms_spline.addPoint(float(t), float(w * y))

    return ms_spline, 0.0


def find_static_equilibrium_pitch_trim(params=P15_RUN4):
    """
    Finds the exact static equilibrium pitch_error_integral and vol_filt_pitch
    so that t=0 starts with zero sagittal pitch nod.
    """
    ensure_libraries_loaded()
    model = osim.Model(os.path.join(_ROOT, "osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim"))
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    osim.PropertyHelper.setValueDouble(params["K_ton"], controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(params["p_ton"], controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(params["K_sc"], controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(params["z_sc"], controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(params["p_sc"], controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(params["K_phas"], controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(params["z_phas"], controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(params["p_phas"], controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(params["kp_prop"], controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(params["Kp_pitch"], controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Kd_pitch"], controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["Ki_pitch"], controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(params["Ki_pitch"], controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(params["kp_prop"], controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(params["z_ccr"], controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(params["p_ccr"], controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(params["k_gamma_dyn"], controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(params["k_gamma_stat"], controller.updPropertyByName("K_gamma_stat"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_tx", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)
    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)
    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    q = osim.Rotation(skull.getTransformInGround(state).R()).convertRotationToQuaternion()
    for idx in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx}", q.get(idx))

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * min(max(params["k_gamma_dyn"], 0.0), 1.0) * (0.25 + 0.75 * a0)
        gs0 = 100.0 * min(max(params["k_gamma_stat"], 0.0), 1.0) * a0
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
    state = manager.integrate(8.0)

    p_int = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral")
    v_flt = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch")
    p1 = model.getCoordinateSet().get("pitch1").getValue(state)
    p2 = model.getCoordinateSet().get("pitch2").getValue(state)
    print(f"[TRIM] Static settled at t=8s: pitch1={math.degrees(p1):.3f} deg, pitch2={math.degrees(p2):.3f} deg, pitch_error_integral={p_int:.6f}, vol_filt_pitch={v_flt:.6f}")
    return p_int, v_flt, p1, p2


if __name__ == "__main__":
    find_static_equilibrium_pitch_trim()
