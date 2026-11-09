import os, sys, ctypes
_UTIL_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.dirname(_UTIL_DIR)
for _p in (_UTIL_DIR, _ROOT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Optional 6-P-Core-Only Hardware Affinity Pinning (i5-13600K: P-cores are logical CPUs 0..11; even CPUs 0,2,4,6,8,10 are the 6 physical P-cores):
if os.name == "nt" and os.environ.get("PCORE_ONLY", "0") == "1":
    try:
        _slot = os.environ.get("PCORE_SLOT", "")
        if _slot != "":
            _pcore_idx = int(_slot) % 6
            # Pin to the physical P-core pair (2*k and 2*k+1) or primary thread (1 << (2*k)) so each worker gets a dedicated physical P-core:
            _mask = 1 << (2 * _pcore_idx)
        else:
            # All 6 physical P-core primary threads (0, 2, 4, 6, 8, 10 -> 0x0555):
            _mask = 0x0555
        ctypes.windll.kernel32.SetProcessAffinityMask(ctypes.windll.kernel32.GetCurrentProcess(), ctypes.c_size_t(_mask))
    except Exception:
        pass

import opensim as osim
import numpy as np
import math
import os
import sys
import json
from cmaes_worker_p22 import (
    ensure_libraries_loaded,
    PER_MUSCLE_72_U0,
    CANONICAL_TARGETS,
)


# Fixed Neurophysiological Literature Poles & Plant/Muscle Delay Compensators (1 + tau*s + 0.5*tau^2*s^2):
LIT_P_TON     = 15.0    # Tonic otolith tilt reflex pole (2.39 Hz, Happee et al. 2017 / Peterka 2002; paired with p_fast = 250 rad/s)
LIT_TAU_PLANT = 0.0150  # Millard (2012) extrafusal muscle activation delay compensator tau_act = 15.0 ms
LIT_TAU_5     = 0.0025  # Path 5 (Sub 5 -> Sub 7 MVST) axonal delay compensator tau_5 = 2.5 ms
LIT_P_SC      = 50.0    # Semicircular canal reflex pole (7.96 Hz, Goldberg & Fernandez 1971 / Keshner 1995; paired with p_fast = 250 rad/s)
LIT_P_PHAS    = 4.0     # Phasic otolith velocity-storage / leaky-integrator pole (0.64 Hz, Angelaki 2004 / Raphan & Cohen 2002; paired with p_fast = 250 rad/s)
LIT_TAU_6     = 0.0040  # Path 6 (Sub 6 -> Sub 7 Propriospinal) axonal delay compensator tau_6 = 4.0 ms
LIT_P_CCR     = 100.0   # Spinal Ia-motoneuron EPSP pole (15.92 Hz, Burke 1981 / Matthews 1972; paired with p_fast = 250 rad/s)


def run_multisine_simulation(
    kp, ki, kd,
    K_ton, p_ton=LIT_P_TON, K_sc=49.3723, z_sc=LIT_TAU_PLANT, p_sc=LIT_P_SC, K_phas=4.9396, p_phas=LIT_P_PHAS,
    kp_prop=2.5030, z_ccr=LIT_TAU_PLANT, p_ccr=LIT_P_CCR, k_gamma_dyn=0.9964, k_gamma_stat=0.9951,
    worker_name="M-?", p_phas2=-1.0, z_phas=LIT_TAU_PLANT
):
    ensure_libraries_loaded()

    # Respect caller/env poles & plant/muscle delay compensators (falling back to literature defaults):
    p_ton  = float(p_ton)  if (p_ton  is not None and float(p_ton)  > 0.5) else LIT_P_TON
    z_sc   = LIT_TAU_PLANT if (z_sc   is None or float(z_sc)   > 0.25 or float(z_sc)   < 0.0) else float(z_sc)
    p_sc   = float(p_sc)   if (p_sc   is not None and float(p_sc)   > 0.5) else LIT_P_SC
    p_phas = float(p_phas) if (p_phas is not None and float(p_phas) > 0.5) else LIT_P_PHAS
    z_phas = LIT_TAU_PLANT if (z_phas is None or float(z_phas) > 0.25 or float(z_phas) < 0.0) else float(z_phas)
    z_ccr  = LIT_TAU_PLANT if (z_ccr  is None or float(z_ccr)  > 0.25 or float(z_ccr)  < 0.0) else float(z_ccr)
    p_ccr  = float(p_ccr)  if (p_ccr  is not None and float(p_ccr)  > 0.5) else LIT_P_CCR
    k_gamma_dyn  = min(max(float(k_gamma_dyn),  0.0), 1.0)
    k_gamma_stat = min(max(float(k_gamma_stat), 0.0), 1.0)

    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")

    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    if controller is None:
        raise RuntimeError("HeadNeckNeuralController not found!")

    osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(z_phas, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    osim.PropertyHelper.setValueDouble(p_phas2, controller.updPropertyByName("p_phas2"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("k_p"))

    kp_task_pitch = float(os.environ.get("KP_TASK_PITCH", str(kp)))
    kd_task_pitch = float(os.environ.get("KD_TASK_PITCH", str(kd)))
    kp_task_roll  = float(os.environ.get("KP_TASK_ROLL",  "10.21"))
    kd_task_roll  = float(os.environ.get("KD_TASK_ROLL",  "1.29"))
    kp_task_yaw   = float(os.environ.get("KP_TASK_YAW",   "0.24"))
    kd_task_yaw   = float(os.environ.get("KD_TASK_YAW",   "0.12"))
    osim.PropertyHelper.setValueDouble(kp_task_pitch, controller.updPropertyByName("Kp_task_pitch"))
    osim.PropertyHelper.setValueDouble(kd_task_pitch, controller.updPropertyByName("Kd_task_pitch"))
    osim.PropertyHelper.setValueDouble(kp_task_roll,  controller.updPropertyByName("Kp_task_roll"))
    osim.PropertyHelper.setValueDouble(kd_task_roll,  controller.updPropertyByName("Kd_task_roll"))
    osim.PropertyHelper.setValueDouble(kp_task_yaw,   controller.updPropertyByName("Kp_task_yaw"))
    osim.PropertyHelper.setValueDouble(kd_task_yaw,   controller.updPropertyByName("Kd_task_yaw"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_roll"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_yaw"))

    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))
    osim.PropertyHelper.setValueBool(True, controller.updPropertyByName("use_smith_predictor"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
    osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    eye_list = (
        ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]
        if os.environ.get("LOCK_EYES", "1") != "0"
        else ["gaze_r_x", "gaze_r_y", "gaze_r_z", "gaze_l_x", "gaze_l_y", "gaze_l_z"]
    )
    for eye_c in eye_list:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    # 6-Frequency Schroeder-Phase Multisine (Ascending Frequency Order)
    ordered_targets = sorted(CANONICAL_TARGETS, key=lambda x: x[0])
    K = len(ordered_targets)
    freqs = np.array([t[0] for t in ordered_targets])
    phases = np.array([math.pi * k * (k - 1) / K for k in range(K)])
    ms_exp = float(os.environ.get("MULTISINE_EXP", "1.0"))
    min_amp_m = float(os.environ.get("MULTISINE_MIN_AMP_M", "0.0"))
    base_amps = np.array([0.0215 * ((0.835 / f) ** ms_exp) for f in freqs])
    scale_factor = float(os.environ.get("MULTISINE_SCALE", str(1.0 / math.sqrt(K))))
    amps = np.maximum(base_amps * scale_factor, min_amp_m)

    delay = 1.5
    ramp_duration = 1.5
    eval_duration = 2.0 / freqs[0]  # 2 full cycles of 0.324 Hz = 6.1728 s
    target_time = delay + ramp_duration + eval_duration  # 9.1728 s total (vs 25.4 s for 6 single sines)

    ms_spline = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.02):
        ms_spline.addPoint(float(t), 0.0)
    spline_dt = 0.0025  # 400 Hz cubic spline knot spacing (>50 knots/cycle at 7.927 Hz)

    ramp_mode = os.environ.get("MULTISINE_RAMP_MODE", "A").upper()
    if ramp_mode == "B":
        for t in np.arange(delay, target_time + 0.02, spline_dt):
            dt_rel = t - delay
            if dt_rel < ramp_duration:
                tau = dt_rel / ramp_duration
                w = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
            else:
                w = 1.0
            y = 0.0
            for k in range(K):
                y += amps[k] * math.sin(2.0 * math.pi * freqs[k] * dt_rel + phases[k])
            ms_spline.addPoint(float(t), float(w * y))
    else:
        # Solution A: Window in Acceleration Space (a_T1(t) = w(tau)*y''(t) + dv_match*phi_v(tau))
        # Eliminates the Leibniz product-rule acceleration parasite on 0.324 Hz while guaranteeing
        # exact C^2 continuity and zero velocity drift for t >= delay + ramp_duration.
        omegas = 2.0 * np.pi * freqs
        sub = 10
        dt_fine = spline_dt / sub
        N_fine = int(round(ramp_duration / dt_fine))
        t_fine = np.linspace(0.0, ramp_duration, N_fine + 1)
        tau_fine = t_fine / ramp_duration
        w_fine = 10.0 * (tau_fine ** 3) - 15.0 * (tau_fine ** 4) + 6.0 * (tau_fine ** 5)
        phi_v = (tau_fine ** 2) * ((1.0 - tau_fine) ** 2)
        phi_v /= np.sum(0.5 * (phi_v[:-1] + phi_v[1:]) * dt_fine)

        psi_fine = np.outer(t_fine, omegas) + phases
        y_fine = np.sin(psi_fine) @ amps
        dy_fine = np.cos(psi_fine) @ (amps * omegas)
        ddy_fine = -np.sin(psi_fine) @ (amps * (omegas ** 2))

        ddx_raw = w_fine * ddy_fine
        v_raw_end = float(np.sum(0.5 * (ddx_raw[:-1] + ddx_raw[1:]) * dt_fine))
        dv_match = float(dy_fine[-1]) - v_raw_end
        ddx_fine = ddx_raw + dv_match * phi_v

        dx_fine = np.zeros_like(t_fine)
        dx_fine[1:] = np.cumsum(0.5 * (ddx_fine[:-1] + ddx_fine[1:]) * dt_fine)
        x_fine = np.zeros_like(t_fine)
        x_fine[1:] = np.cumsum(0.5 * (dx_fine[:-1] + dx_fine[1:]) * dt_fine)
        x_offset = float(x_fine[-1] - y_fine[-1])

        N_ramp_knots = int(round(ramp_duration / spline_dt))
        for i_k in range(N_ramp_knots):
            t = delay + i_k * spline_dt
            ms_spline.addPoint(float(t), float(x_fine[i_k * sub]))
        for t in np.arange(delay + ramp_duration, target_time + 0.02, spline_dt):
            dt_rel = t - delay
            y = float(np.sum(amps * np.sin(omegas * dt_rel + phases)))
            ms_spline.addPoint(float(t), float(y + x_offset))

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(ms_spline)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    coord_tx.setSpeedValue(state, 0.0)

    model.realizePosition(state)
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        k_gdyn_eff = min(max(k_gamma_dyn, 0.0), 1.0)
        k_gstat_eff = min(max(k_gamma_stat, 0.0), 1.0)
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
    manager.setIntegratorMethod(int(os.environ.get("OSIM_INT_METHOD", "5")))  # 5 = SemiExplicitEuler2
    manager.setIntegratorAccuracy(float(os.environ.get("OSIM_INT_ACC", "1e-4")))
    manager.setIntegratorMinimumStepSize(float(os.environ.get("OSIM_MIN_STEP", "1e-4")))
    manager.setIntegratorMaximumStepSize(float(os.environ.get("OSIM_MAX_STEP", "0.001")))

    state.setTime(0.0)
    manager.initialize(state)

    step = 0.005  # 200 Hz sampling (>25 samples/cycle at 7.927 Hz, 1,235 samples in steady-state window)
    current_time = 0.0

    coord_p1 = model.getCoordinateSet().get("pitch1")
    coord_p2 = model.getCoordinateSet().get("pitch2")
    coord_r1 = model.getCoordinateSet().get("roll1")
    coord_r2 = model.getCoordinateSet().get("roll2")
    coord_y1 = model.getCoordinateSet().get("yaw1")
    coord_y2 = model.getCoordinateSet().get("yaw2")

    t_history = []
    pitch_history = []
    p1_history = []
    p2_history = []
    r1_history = []
    r2_history = []
    roll_history = []
    y1_history = []
    y2_history = []
    yaw_history = []
    mahony_pitch_history = []
    true_omega_pitch_history = []
    luen_omega_pitch_history = []
    sp_dL_history = []
    pitch_int_history = []
    vol_pitch_history = []

    aborted = False
    abort_time_s = target_time
    abort_reason = ""

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        p1_deg = math.degrees(coord_p1.getValue(state))
        p2_deg = math.degrees(coord_p2.getValue(state))
        r1_deg = math.degrees(coord_r1.getValue(state))
        r2_deg = math.degrees(coord_r2.getValue(state))
        y1_deg = math.degrees(coord_y1.getValue(state))
        y2_deg = math.degrees(coord_y2.getValue(state))

        head_pitch_deg = p1_deg + p2_deg
        head_roll_deg  = r1_deg + r2_deg
        head_yaw_deg   = y1_deg + y2_deg

        try:
            q0 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0")
            q1 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1")
            q2 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2")
            q3 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3")
            qn = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
            if qn > 1e-6:
                q0, q1, q2, q3 = q0 / qn, q1 / qn, q2 / qn, q3 / qn
            mah_p_deg = math.degrees(math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))
            luen_w_deg = math.degrees(model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/luen_omega_pitch"))
            true_w_deg = math.degrees(coord_p1.getSpeedValue(state) + coord_p2.getSpeedValue(state))
            sp_dL_mm = 1000.0 * model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/sp_dL_semi_cap_sklthx")
            p_int = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral")
            v_pit = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch")
        except Exception:
            mah_p_deg = head_pitch_deg
            luen_w_deg = 0.0
            true_w_deg = 0.0
            sp_dL_mm = 0.0
            p_int = 0.0
            v_pit = 0.0

        t_history.append(current_time)
        pitch_history.append(head_pitch_deg)
        p1_history.append(p1_deg)
        p2_history.append(p2_deg)
        r1_history.append(r1_deg)
        r2_history.append(r2_deg)
        roll_history.append(head_roll_deg)
        y1_history.append(y1_deg)
        y2_history.append(y2_deg)
        yaw_history.append(head_yaw_deg)
        mahony_pitch_history.append(mah_p_deg)
        true_omega_pitch_history.append(true_w_deg)
        luen_omega_pitch_history.append(luen_w_deg)
        sp_dL_history.append(sp_dL_mm)
        pitch_int_history.append(p_int)
        vol_pitch_history.append(v_pit)

        if (p1_deg < -12.0 or p1_deg > 18.0 or
            p2_deg < -24.75 or p2_deg > 36.0 or
            head_pitch_deg < -36.75 or head_pitch_deg > 54.0 or
            abs(r1_deg) > 4.5 or abs(r2_deg) > 24.75 or abs(head_roll_deg) > 29.25 or
            abs(y1_deg) > 28.5 or abs(y2_deg) > 20.25 or abs(head_yaw_deg) > 48.75):
            aborted = True
            abort_time_s = float(current_time)
            abort_reason = (
                f"t={current_time:.2f}s: p1={p1_deg:+.2f}°, p2={p2_deg:+.2f}°, pitch={head_pitch_deg:+.2f}°, "
                f"roll={head_roll_deg:+.2f}° (r1={r1_deg:+.2f}°,r2={r2_deg:+.2f}°), "
                f"yaw={head_yaw_deg:+.2f}° (y1={y1_deg:+.2f}°,y2={y2_deg:+.2f}°)"
            )
            print(f"[ABORT TRANSITION] {worker_name} entered transition zone ({abort_reason})", file=sys.stderr)
            break

    t_arr = np.array(t_history) if t_history else np.array([0.0])
    p_arr = np.array(pitch_history) if pitch_history else np.array([2.801])
    p1_arr = np.array(p1_history) if p1_history else np.array([2.063])
    p2_arr = np.array(p2_history) if p2_history else np.array([0.738])
    r1_arr = np.array(r1_history) if r1_history else np.array([0.0])
    r2_arr = np.array(r2_history) if r2_history else np.array([0.0])
    r_arr = np.array(roll_history) if roll_history else np.array([0.0])
    y1_arr = np.array(y1_history) if y1_history else np.array([0.0])
    y2_arr = np.array(y2_history) if y2_history else np.array([0.0])
    y_arr = np.array(yaw_history) if yaw_history else np.array([0.0])
    mah_arr = np.array(mahony_pitch_history) if mahony_pitch_history else p_arr
    tw_arr = np.array(true_omega_pitch_history) if true_omega_pitch_history else np.array([0.0])
    lw_arr = np.array(luen_omega_pitch_history) if luen_omega_pitch_history else np.array([0.0])
    sp_arr = np.array(sp_dL_history) if sp_dL_history else np.array([0.0])
    pi_arr = np.array(pitch_int_history) if pitch_int_history else np.array([0.0])
    vp_arr = np.array(vol_pitch_history) if vol_pitch_history else np.array([0.0])

    m_pre = t_arr <= delay
    m_post = t_arr >= (delay + ramp_duration)
    if not np.any(m_post):
        m_post = np.ones_like(t_arr, dtype=bool)

    pre_pitch_drift = float(np.max(p_arr[m_pre]) - np.min(p_arr[m_pre])) if np.any(m_pre) else 0.0
    mah_err = (mah_arr[m_post] - mah_arr[0]) - (p_arr[m_post] - p_arr[0])
    mah_rmse = float(np.sqrt(np.mean(mah_err ** 2)))
    mah_corr = float(np.corrcoef(p_arr[m_post], mah_arr[m_post])[0, 1]) if (len(p_arr[m_post]) > 3 and np.std(p_arr[m_post]) > 1e-5 and np.std(mah_arr[m_post]) > 1e-5) else 0.0
    luen_rmse = float(np.sqrt(np.mean((lw_arr[m_post] - tw_arr[m_post]) ** 2)))
    luen_corr = float(np.corrcoef(tw_arr[m_post], lw_arr[m_post])[0, 1]) if (len(tw_arr[m_post]) > 3 and np.std(tw_arr[m_post]) > 1e-5 and np.std(lw_arr[m_post]) > 1e-5) else 0.0

    telemetry = {
        "aborted": aborted,
        "abort_time_s": round(abort_time_s, 3),
        "abort_reason": abort_reason,
        "pre_pitch_drift_deg": round(pre_pitch_drift, 4),
        "post_pitch_min_deg": round(float(np.min(p_arr[m_post])), 3),
        "post_pitch_max_deg": round(float(np.max(p_arr[m_post])), 3),
        "post_pitch_mean_deg": round(float(np.mean(p_arr[m_post])), 3),
        "p1_min_deg": round(float(np.min(p1_arr[m_post])), 2),
        "p1_max_deg": round(float(np.max(p1_arr[m_post])), 2),
        "p2_min_deg": round(float(np.min(p2_arr[m_post])), 2),
        "p2_max_deg": round(float(np.max(p2_arr[m_post])), 2),
        "max_s_buckling_deg": round(float(np.max(np.abs(p1_arr[m_post] - p2_arr[m_post]))), 2),
        "max_abs_roll_deg": round(float(np.max(np.abs(r_arr[m_post]))), 3),
        "max_abs_roll1_deg": round(float(np.max(np.abs(r1_arr[m_post]))), 3),
        "max_abs_roll2_deg": round(float(np.max(np.abs(r2_arr[m_post]))), 3),
        "max_abs_yaw_deg": round(float(np.max(np.abs(y_arr[m_post]))), 3),
        "max_abs_yaw1_deg": round(float(np.max(np.abs(y1_arr[m_post]))), 3),
        "max_abs_yaw2_deg": round(float(np.max(np.abs(y2_arr[m_post]))), 3),
        "mahony_pitch_rmse_deg": round(mah_rmse, 3),
        "mahony_pitch_corr": round(mah_corr, 4),
        "canal_luen_rmse_deg_s": round(luen_rmse, 2),
        "canal_luen_corr": round(luen_corr, 4),
        "spindle_dL_max_mm": round(float(np.max(np.abs(sp_arr[m_post]))), 3),
        "pitch_integral_max": round(float(np.max(np.abs(pi_arr[m_post]))), 4),
        "vol_pitch_torque_max_nm": round(float(np.max(np.abs(vp_arr[m_post]))), 3),
    }

    if aborted or len(t_arr[m_post]) < 50:
        progress_ratio = max(0.0, min(1.0, abort_time_s / target_time))
        abort_score = 6.0 * (200000.0 + (1.0 - progress_ratio) * 50000.0)
        return abort_score, {}, telemetry

    # Simultaneous 6-Harmonic Least-Squares Demodulation over steady-state window t >= delay + ramp
    t_fit = t_arr[m_post]
    y_fit_deg = p_arr[m_post]
    t_rel = t_fit - delay

    cols = [np.ones_like(t_fit), t_fit - (delay + ramp_duration)]
    for k in range(K):
        omega_k = 2.0 * math.pi * freqs[k]
        psi_k = omega_k * t_rel + phases[k]
        cols.append(np.sin(psi_k))
        cols.append(np.cos(psi_k))

    A_mat = np.column_stack(cols)
    coeffs, _, _, _ = np.linalg.lstsq(A_mat, y_fit_deg, rcond=None)

    freq_results = {}
    total_score = 0.0

    NOISE_FLOOR_7927_DEG = float(os.environ.get("NOISE_FLOOR_7927_DEG", "0.0"))
    AMP_GUARD_7927_DEG = float(os.environ.get("AMP_GUARD_7927_DEG", "0.12"))

    for k in range(K):
        f_val, tgt_gain, tgt_phase = ordered_targets[k]
        S_k = float(coeffs[2 + 2 * k])
        C_k = float(coeffs[2 + 2 * k + 1])
        amp_deg = math.sqrt(S_k * S_k + C_k * C_k)
        sim_gain = amp_deg / amps[k]
        sim_phase = math.degrees(math.atan2(C_k, S_k))

        gain_err_pct = abs(sim_gain - tgt_gain) / tgt_gain * 100.0
        gain_db_diff = 20.0 * math.log10(max(sim_gain, 1e-6) / tgt_gain)
        gain_loss = 100.0 * (gain_db_diff ** 2)

        phase_diff = abs(sim_phase - tgt_phase) % 360.0
        if phase_diff > 180.0:
            phase_diff = 360.0 - phase_diff
        phase_loss = 1.0 * (phase_diff ** 2)
        f_score = gain_loss + phase_loss

        # 7.927 Hz sub-0.1 deg integrator noise-floor deadband:
        # Target pitch amplitude at 7.927 Hz is 0.0558 deg (55.8 millidegrees), while SemiExplicitEuler2
        # variable-step integration across the t=1.81s cervical snap has a ~0.04-0.08 deg complex phasor noise floor.
        # Apply a C1-smooth complex phasor deadband of NOISE_FLOOR_7927_DEG with a hard tremor guard above AMP_GUARD_7927_DEG.
        if f_val > 7.0 and NOISE_FLOOR_7927_DEG > 0.0:
            tgt_amp_deg = tgt_gain * amps[k]
            tgt_p_rad = math.radians(tgt_phase)
            S_t = tgt_amp_deg * math.cos(tgt_p_rad)
            C_t = tgt_amp_deg * math.sin(tgt_p_rad)
            dS = S_k - S_t
            dC = C_k - C_t
            e_deg = math.hypot(dS, dC)
            if e_deg <= NOISE_FLOOR_7927_DEG:
                gain_loss = 0.0
                phase_loss = 0.0
                f_score = 0.0
            else:
                scale = (e_deg - NOISE_FLOOR_7927_DEG) / e_deg
                S_eff = S_t + scale * dS
                C_eff = C_t + scale * dC
                amp_eff = math.hypot(S_eff, C_eff)
                g_eff = amp_eff / amps[k]
                p_eff = math.degrees(math.atan2(C_eff, S_eff))
                db_eff = 20.0 * math.log10(max(g_eff, 1e-6) / tgt_gain)
                pd_eff = abs(p_eff - tgt_phase) % 360.0
                if pd_eff > 180.0:
                    pd_eff = 360.0 - pd_eff
                gain_loss = 100.0 * (db_eff ** 2)
                phase_loss = 1.0 * (pd_eff ** 2)
                f_score = gain_loss + phase_loss
            if amp_deg > AMP_GUARD_7927_DEG:
                tremor_7927 = 50000.0 * ((amp_deg - AMP_GUARD_7927_DEG) ** 2)
                gain_loss += tremor_7927
                f_score += tremor_7927

        total_score += f_score

        freq_results[f"{f_val:.3f}"] = {
            "freq": f_val,
            "target_gain": tgt_gain,
            "sim_gain": sim_gain,
            "gain_err_pct": gain_err_pct,
            "gain_db_diff": gain_db_diff,
            "gain_loss": gain_loss,
            "target_phase": tgt_phase,
            "sim_phase": sim_phase,
            "phase_err_deg": phase_diff,
            "phase_loss": phase_loss,
            "score": f_score,
            "telemetry": telemetry,
        }

    # High-frequency / Roll-Yaw tremor penalty with 1.0 deg deadband:
    # - Zero penalty when both Max |Roll| <= 1.0 deg and Max |Yaw| <= 1.0 deg (and pitch >=10 Hz RMS <= 0.10 deg).
    # - Above 1.0 deg in Roll or Yaw, quadratic penalty prevents out-of-plane instability while keeping <1.0 deg 100% pure Bode.
    HF_CUTOFF_HZ = 10.0
    ROLL_YAW_DEADBAND_DEG = float(os.environ.get("ROLL_YAW_DEADBAND_DEG", "1.00"))
    ROLL_YAW_EXCESS_WEIGHT = float(os.environ.get("ROLL_YAW_EXCESS_WEIGHT", "6400.0"))
    PITCH_HF_DEADBAND_DEG = float(os.environ.get("PITCH_HF_DEADBAND_DEG", "0.10"))
    PITCH_HF_EXCESS_WEIGHT = float(os.environ.get("PITCH_HF_EXCESS_WEIGHT", "10000.0"))
    try:
        t_u = np.arange(t_fit[0], t_fit[-1], 0.005)
        def _hf_rms(sig):
            s_u = np.interp(t_u, t_fit, sig)
            s_u = (s_u - np.mean(s_u)) * np.hanning(len(s_u))
            mag = np.abs(np.fft.rfft(s_u)) * 2.0 / np.sum(np.hanning(len(s_u)))
            fr = np.fft.rfftfreq(len(s_u), d=0.005)
            return float(np.sqrt(np.sum(mag[fr >= HF_CUTOFF_HZ] ** 2) / 2.0))
        hf_p1 = _hf_rms(p1_arr[m_post])
        hf_pt = _hf_rms(p_arr[m_post])
        hf_rl = _hf_rms(r_arr[m_post])
        hf_comb = math.sqrt(hf_p1 ** 2 + hf_pt ** 2 + hf_rl ** 2)
    except Exception:
        hf_p1 = hf_pt = hf_rl = hf_comb = 0.0
    roll_excess = max(0.0, float(telemetry.get("max_abs_roll_deg", 0.0)) - ROLL_YAW_DEADBAND_DEG)
    yaw_excess = max(0.0, float(telemetry.get("max_abs_yaw_deg", 0.0)) - ROLL_YAW_DEADBAND_DEG)
    pitch_hf_excess = max(0.0, float(hf_pt) - PITCH_HF_DEADBAND_DEG)
    hf_penalty = (
        ROLL_YAW_EXCESS_WEIGHT * (roll_excess ** 2 + yaw_excess ** 2)
        + PITCH_HF_EXCESS_WEIGHT * pitch_hf_excess
    )
    # DC Offset & Linear Drift telemetry (penalty disabled: DC_DRIFT_WEIGHT = 0.0)
    DC_DRIFT_WEIGHT = float(os.environ.get("DC_DRIFT_WEIGHT", "0.0"))
    c0_rel_deg = float(coeffs[0] - p_arr[0])
    c1_deg_per_s = float(coeffs[1])
    trend_curve_deg = c0_rel_deg + c1_deg_per_s * (t_fit - (delay + ramp_duration))
    trend_rms_deg = float(np.sqrt(np.mean(trend_curve_deg ** 2)))
    dc_drift_penalty = DC_DRIFT_WEIGHT * (trend_rms_deg ** 2)

    telemetry["hf_rms_pitch1_deg"] = round(hf_p1, 4)
    telemetry["hf_rms_pitch_deg"] = round(hf_pt, 4)
    telemetry["hf_rms_roll_deg"] = round(hf_rl, 4)
    telemetry["hf_rms_combined_deg"] = round(hf_comb, 4)
    telemetry["hf_penalty"] = round(hf_penalty, 2)
    telemetry["dc_offset_rel_deg"] = round(float(np.mean(p_arr[m_post] - p_arr[0])), 3)
    telemetry["drift_slope_deg_s"] = round(c1_deg_per_s, 4)
    telemetry["trend_rms_deg"] = round(trend_rms_deg, 3)
    telemetry["dc_drift_penalty"] = round(dc_drift_penalty, 2)
    telemetry["bode_score_without_hf"] = round(total_score, 2)
    total_score += hf_penalty + dc_drift_penalty

    if os.environ.get("SAVE_TS", "0") == "1":
        os.makedirs("logs/candidates_multisine", exist_ok=True)
        np.savez_compressed(
            os.path.join("logs", "candidates_multisine", f"{worker_name}_ts.npz"),
            t=t_arr,
            pitch=p_arr,
            p1=p1_arr,
            p2=p2_arr,
            roll=r_arr,
            r1=r1_arr,
            r2=r2_arr,
            yaw=y_arr,
            y1=y1_arr,
            y2=y2_arr,
            mahony_pitch=mah_arr,
            coeffs=coeffs,
        )

    return total_score, freq_results, telemetry


if __name__ == "__main__":
    p_ton  = float(os.environ.get("P_TON",  str(LIT_P_TON)))
    z_sc   = float(os.environ.get("Z_SC",   str(LIT_TAU_PLANT)))
    p_sc   = float(os.environ.get("P_SC",   str(LIT_P_SC)))
    p_phas = float(os.environ.get("P_PHAS", str(LIT_P_PHAS)))
    z_phas = float(os.environ.get("Z_PHAS", str(LIT_TAU_PLANT)))
    z_ccr  = float(os.environ.get("Z_CCR",  str(LIT_TAU_PLANT)))
    p_ccr  = float(os.environ.get("P_CCR",  str(LIT_P_CCR)))

    if len(sys.argv) == 13:
        # 11-Parameter Gains + Poles + Ki_task_pitch Staged CMA-ES Mode:
        # [K_ton, K_sc, K_phas, kp_prop, Kp_task_pitch, Kd_task_pitch, p_ton, p_sc, p_phas, p_ccr, Ki_task_pitch, worker_label]
        K_ton         = float(sys.argv[1])
        K_sc          = float(sys.argv[2])
        K_phas        = float(sys.argv[3])
        kp_p          = float(sys.argv[4])
        kp_task_pitch = float(sys.argv[5])
        kd_task_pitch = float(sys.argv[6])
        p_ton         = float(sys.argv[7])
        p_sc          = float(sys.argv[8])
        p_phas        = float(sys.argv[9])
        p_ccr         = float(sys.argv[10])
        ki_task_pitch = float(sys.argv[11])
        worker_label  = sys.argv[12]
        k_dyn         = float(os.environ.get("K_GAMMA_DYN",  "1.0"))
        k_stat        = float(os.environ.get("K_GAMMA_STAT", "1.0"))
        z_sc          = 0.0
        z_phas        = 0.0
        z_ccr         = 0.0
    elif len(sys.argv) == 12:
        # 10-Parameter Gains + Poles Staged CMA-ES Mode:
        # [K_ton, K_sc, K_phas, kp_prop, Kp_task_pitch, Kd_task_pitch, p_ton, p_sc, p_phas, p_ccr, worker_label]
        K_ton         = float(sys.argv[1])
        K_sc          = float(sys.argv[2])
        K_phas        = float(sys.argv[3])
        kp_p          = float(sys.argv[4])
        kp_task_pitch = float(sys.argv[5])
        kd_task_pitch = float(sys.argv[6])
        p_ton         = float(sys.argv[7])
        p_sc          = float(sys.argv[8])
        p_phas        = float(sys.argv[9])
        p_ccr         = float(sys.argv[10])
        worker_label  = sys.argv[11]
        ki_task_pitch = float(os.environ.get("KI_TASK_PITCH", "0.0"))
        k_dyn         = float(os.environ.get("K_GAMMA_DYN",  "1.0"))
        k_stat        = float(os.environ.get("K_GAMMA_STAT", "1.0"))
        z_sc          = 0.0
        z_phas        = 0.0
        z_ccr         = 0.0
    elif len(sys.argv) == 11:
        # 9-Parameter Gain-Only CMA-ES Mode:
        # [K_ton, K_sc, K_phas, kp_p, k_dyn, k_stat, kp_task_pitch, kd_task_pitch, ki_task_pitch, worker_label]
        K_ton         = float(sys.argv[1])
        K_sc          = float(sys.argv[2])
        K_phas        = float(sys.argv[3])
        kp_p          = float(sys.argv[4])
        k_dyn         = min(max(float(sys.argv[5]), 0.0), 1.0)
        k_stat        = min(max(float(sys.argv[6]), 0.0), 1.0)
        kp_task_pitch = float(sys.argv[7])
        kd_task_pitch = float(sys.argv[8])
        ki_task_pitch = float(sys.argv[9])
        worker_label  = sys.argv[10]
    else:
        w_n    = float(sys.argv[1])
        zeta   = float(sys.argv[2])
        ki     = float(sys.argv[3])
        K_ton  = float(sys.argv[4])
        p_ton  = float(sys.argv[5])
        K_sc   = float(sys.argv[6])
        z_sc   = float(sys.argv[7])
        p_sc   = float(sys.argv[8])
        K_phas = float(sys.argv[9])
        p_phas = float(sys.argv[10])
        kp_p   = float(sys.argv[11])
        z_ccr  = float(sys.argv[12])
        p_ccr  = float(sys.argv[13])
        k_dyn  = min(max(float(sys.argv[14]), 0.0), 1.0)
        k_stat = min(max(float(sys.argv[15]), 0.0), 1.0)

        if len(sys.argv) >= 20:
            kp_task_pitch = float(sys.argv[16])
            kd_task_pitch = float(sys.argv[17])
            ki_task_pitch = float(sys.argv[18])
            worker_label  = sys.argv[19]
        else:
            kp_task_pitch = 0.0
            kd_task_pitch = 0.0
            ki_task_pitch = ki
            worker_label  = sys.argv[-1]

    os.environ["KP_TASK_PITCH"] = str(kp_task_pitch)
    os.environ["KD_TASK_PITCH"] = str(kd_task_pitch)

    params_list = [
        K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas,
        kp_p, z_ccr, p_ccr, k_dyn, k_stat,
        kp_task_pitch, kd_task_pitch, ki_task_pitch,
    ]
    params_10_staged = [
        K_ton, K_sc, K_phas, kp_p, kp_task_pitch, kd_task_pitch,
        p_ton, p_sc, p_phas, p_ccr,
    ]
    params_11_staged = [
        K_ton, K_sc, K_phas, kp_p, kp_task_pitch, kd_task_pitch,
        p_ton, p_sc, p_phas, p_ccr, ki_task_pitch,
    ]

    try:
        total_score, freq_results, telemetry = run_multisine_simulation(
            kp_task_pitch, ki_task_pitch, kd_task_pitch,
            K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas,
            kp_p, z_ccr, p_ccr, k_dyn, k_stat,
            worker_name=worker_label, z_phas=z_phas,
        )
        os.makedirs("logs/candidates_multisine", exist_ok=True)
        out_path = os.path.join("logs", "candidates_multisine", f"{worker_label}.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "worker": worker_label,
                    "total_score": total_score,
                    "params": params_list,
                    "params_10": params_10_staged,
                    "params_11": params_11_staged,
                    "frequencies": freq_results,
                    "telemetry": telemetry,
                },
                f,
                indent=2,
            )
        print(f"{total_score:.6f}")
    except Exception as e:
        import traceback
        traceback.print_exc(file=sys.stderr)
        print("6000000.0")
        sys.exit(1)

