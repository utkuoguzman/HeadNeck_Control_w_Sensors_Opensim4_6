"""
surrogacy/measure_passive_plant.py
Step 1b of the Upgraded 6-DOF, 72-Muscle MIMO Differentiable Neuromotor Surrogate.

Measures the exact empirical 6-DOF co-contracted passive plant frequency response
G_passive_6dof(j*omega) = [Theta_p1, Theta_p2, Theta_r1, Theta_r2, Theta_y1, Theta_y2]^T / X_sled(j*omega)
from the full non-linear HYOID OpenSim 4.6 model across the 6 canonical Happee (2017) frequencies
with the 72-muscle literature-centered 6-DOF equilibrium co-contraction (PER_MUSCLE_72_U0)
and zero cortical integral preload (init_pitch_integral = 0.0, grav_trim_torque = 0.0).

Outputs:
    surrogacy/opensim_passive_plant.json: Exact 6-DOF + total pitch G_passive complex table.
"""

import opensim as osim
import os
import math
import numpy as np
import json
import sys
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.getcwd())
from cmaes_worker_p22 import PER_MUSCLE_72_U0, extract_bode

osim.Logger.setLevelString("Error")
osim.LoadOpenSimLibrary("osimMillard12EqWithAff.dll")
osim.LoadOpenSimLibrary("osimHeadNeckNeural.dll")
osim.LoadOpenSimLibrary("FixationController_plugin/build/Release/osimFixation.dll")

FREQS = [0.324, 0.783, 1.099, 2.446, 5.583, 7.927]

def measure_frequency(freq):
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    # Set active sensory reflexes and cortical task pitch gains to zero
    # so only the 72-muscle 6-DOF equilibrium co-contraction (u_min,i) + passive bushings act
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(78.0, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(14.4, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(80.0, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(-1.0, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(20.0, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(-1.0, controller.updPropertyByName("p_phas2"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(10.175, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(185.0, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(0.70, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(0.95, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(10.21, controller.updPropertyByName("Kp_task_roll"))
    osim.PropertyHelper.setValueDouble(1.29, controller.updPropertyByName("Kd_task_roll"))
    osim.PropertyHelper.setValueDouble(0.24, controller.updPropertyByName("Kp_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.12, controller.updPropertyByName("Kd_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_yaw"))

    for i in range(model.getCoordinateSet().getSize()):
        c = model.updCoordinateSet().get(i)
        if "eye" in c.getName():
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    amplitude = 0.0215 * (0.835 / freq) ** 2
    delay = 1.5
    period = 1.0 / freq
    ramp_duration = period
    min_phase3_time = 2.0 / 1.8
    n_eval = max(1, math.ceil(min_phase3_time / period))
    target_time = delay + ramp_duration + (n_eval * period)

    sine_function = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        sine_function.addPoint(t, 0.0)
    spline_dt = min(0.01, period / 40.0)
    for t in np.arange(delay, target_time + 0.05, spline_dt):
        dt = t - delay
        w = 10.0 * ((dt / ramp_duration) ** 3) - 15.0 * ((dt / ramp_duration) ** 4) + 6.0 * ((dt / ramp_duration) ** 5) if dt < ramp_duration else 1.0
        sine_function.addPoint(t, w * amplitude * math.sin(2.0 * math.pi * freq * dt))

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    for idx_q in range(4):
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/obs_q{idx_q}", q.get(idx_q))

    for sv in ["pitch_error_integral", "roll_error_integral", "yaw_error_integral", "vol_filt_pitch", "vol_filt_roll", "vol_filt_yaw"]:
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/{sv}", 0.0)

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * 0.70 * a0
        gs0 = 100.0 * 0.95 * a0
        fdyn0 = (gd0 * gd0) / (gd0 * gd0 + 3600.0)
        fstat0 = (gs0 * gs0) / (gs0 * gs0 + 3600.0)
        m.setStateVariableValue(state, "spindle/dynamic_activation", fdyn0)
        m.setStateVariableValue(state, "spindle/static_activation", fstat0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fdyn_{name}", fdyn0)
        model.setStateVariableValue(state, f"/controllerset/VCR_CCR_Controller/sp_fstat_{name}", fstat0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5)
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.001)
    state.setTime(0.0)
    manager.initialize(state)

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")

    t_hist, p_hist, p1_hist, p2_hist, r_hist, y_hist = [], [], [], [], [], []
    step = min(0.01, period / 40.0)
    curr = 0.0
    while curr < target_time:
        curr = min(curr + step, target_time)
        state = manager.integrate(curr)
        p1 = math.degrees(coord_p1.getValue(state))
        p2 = math.degrees(coord_p2.getValue(state))
        r = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        y = math.degrees(coord_y1.getValue(state) + coord_y2.getValue(state))
        t_hist.append(curr)
        p1_hist.append(p1)
        p2_hist.append(p2)
        p_hist.append(p1 + p2)
        r_hist.append(r)
        y_hist.append(y)

    discard = delay + ramp_duration
    omega = 2.0 * math.pi * freq

    def get_complex_bode(arr):
        amp_d, ph_r = extract_bode(t_hist, arr, freq, discard, delay)
        g_dm = amp_d / amplitude
        ph_d = math.degrees(ph_r)
        while ph_d > 90.0: ph_d -= 360.0
        while ph_d < -270.0: ph_d += 360.0
        return g_dm, ph_d

    gain_deg_m, phase_deg = get_complex_bode(p_hist)
    g_p1, ph_p1 = get_complex_bode(p1_hist)
    g_p2, ph_p2 = get_complex_bode(p2_hist)

    print(
        f"[{freq:5.3f} Hz] Co-Contracted 6-DOF Passive Plant: "
        f"Total Pitch = {gain_deg_m:7.2f} deg/m @ {phase_deg:+6.1f}° | "
        f"p1 = {g_p1:6.2f} @ {ph_p1:+6.1f}° | p2 = {g_p2:6.2f} @ {ph_p2:+6.1f}° | "
        f"max|roll|={np.max(np.abs(r_hist)):.3f}° max|yaw|={np.max(np.abs(y_hist)):.3f}°",
        flush=True
    )
    return {
        "freq": freq,
        "omega": omega,
        "amplitude": amplitude,
        "gain_deg_m": gain_deg_m,
        "phase_deg": phase_deg,
        "p1_gain_deg_m": g_p1,
        "p1_phase_deg": ph_p1,
        "p2_gain_deg_m": g_p2,
        "p2_phase_deg": ph_p2,
        "max_abs_roll_deg": float(np.max(np.abs(r_hist))),
        "max_abs_yaw_deg": float(np.max(np.abs(y_hist))),
        "mean_pitch": float(np.mean(p_hist))
    }

if __name__ == "__main__":
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    print("Measuring Exact 6-DOF Co-Contracted Passive OpenSim Plant across 6 Frequencies in parallel...")
    results = {}
    with ProcessPoolExecutor(max_workers=6) as executor:
        futures = {executor.submit(measure_frequency, f): f for f in FREQS}
        for fut in futures:
            res = fut.result()
            results[str(res["freq"])] = res

    os.makedirs("surrogacy", exist_ok=True)
    out_file = "surrogacy/opensim_passive_plant.json"
    with open(out_file, "w", encoding="utf-8") as fp:
        json.dump(results, fp, indent=2)
    print(f"Saved exact 6-DOF OpenSim passive plant to {out_file}")
