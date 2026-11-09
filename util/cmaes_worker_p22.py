import os, sys
_UTIL_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT_DIR = os.path.dirname(_UTIL_DIR)
for _p in (_UTIL_DIR, _ROOT_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import opensim as osim
import numpy as np
import math
import os
import sys
import json


def extract_bode(t_array, pitch_array, freq, discard_time, delay):
    valid_indices = [i for i, t in enumerate(t_array) if t >= discard_time]
    t_steady = np.array([t_array[i] for i in valid_indices])
    pitch_steady = np.array([pitch_array[i] for i in valid_indices])

    if len(t_steady) < 10:
        return 0.0, 0.0

    omega = 2.0 * math.pi * freq
    t_shifted = t_steady - delay

    coeffs = np.polyfit(t_steady, pitch_steady, 1)
    pitch_detrended = pitch_steady - (coeffs[0] * t_steady + coeffs[1])

    Is = 2.0 * np.mean(pitch_detrended * np.sin(omega * t_shifted))
    Ic = 2.0 * np.mean(pitch_detrended * np.cos(omega * t_shifted))

    amplitude_rad = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)

    return amplitude_rad, phase_rad


_LIBRARIES_LOADED = False

CANONICAL_TARGETS = [
    (2.446, 423.34, -161.34),
    (5.583, 503.30, -201.38),
    (7.927, 572.69, -247.62),
    (1.099, 319.74, -109.23),
    (0.783, 236.93,  -96.96),
    (0.324,  86.55,   -7.03)
]

PER_MUSCLE_72_U0 = {
    "digastric_post": 0.0257708683,
    "digastric_ant": 0.0351123801,
    "Geniohyoid": 0.0358435442,
    "Mylohyoid_Post": 0.0353440759,
    "Mylohyoid_Ant": 0.0357049120,
    "Stylohyoid_Lat": 0.0326655729,
    "Stylohyoid_Med": 0.0327072420,
    "Sterno_hyoid": 0.0139074664,
    "SternoThyroid": 0.0139580126,
    "Omo_hyoid": 0.0140401442,
    "digastric_post_L": 0.0260257723,
    "digastric_ant_L": 0.0352215535,
    "Geniohyoid_L": 0.0357879142,
    "Mylohyoid_Post_L": 0.0354227939,
    "Mylohyoid_Ant_L": 0.0359162302,
    "Stylohyoid_Lat_L": 0.0327162539,
    "Stylohyoid_Med_L": 0.0327579685,
    "Sterno_hyoid_L": 0.0138869448,
    "SternoThyroid_L": 0.0139146060,
    "Omo_hyoid_L": 0.0140511883,
    "stern_mast": 0.0137600533,
    "stern_mast_L": 0.0138882584,
    "cleid_mast": 0.0140218931,
    "cleid_mast_L": 0.0140604434,
    "cleid_occ": 0.0141125391,
    "cleid_occ_L": 0.0141761987,
    "scalenus_ant": 0.0145401926,
    "scalenus_ant_L": 0.0144270113,
    "scalenus_med": 0.0326803693,
    "scalenus_med_L": 0.0325170091,
    "scalenus_post": 0.0317368734,
    "scalenus_post_L": 0.0316464182,
    "long_cap_sklc4": 0.0353953794,
    "long_cap_sklc4_L": 0.0353956169,
    "long_col_c1thx": 0.0295473898,
    "long_col_c1thx_L": 0.0295025776,
    "long_col_c1c5": 0.0334943909,
    "long_col_c1c5_L": 0.0334361024,
    "long_col_c5thx": 0.0298660580,
    "long_col_c5thx_L": 0.0298181117,
    "trap_cl": 0.0222030957,
    "trap_cl_L": 0.0221611600,
    "trap_acr": 0.0391430837,
    "trap_acr_L": 0.0389694904,
    "splen_cap_sklc6": 0.0232840744,
    "splen_cap_sklc6_L": 0.0233107645,
    "splen_cap_sklthx": 0.0256466659,
    "splen_cap_sklthx_L": 0.0256818910,
    "splen_cerv_c3thx": 0.0487024411,
    "splen_cerv_c3thx_L": 0.0486376139,
    "semi_cap_sklc5": 0.0221057900,
    "semi_cap_sklc5_L": 0.0222267488,
    "semi_cap_sklthx": 0.0230974719,
    "semi_cap_sklthx_L": 0.0231614554,
    "semi_cerv_c3thx": 0.0694999818,
    "semi_cerv_c3thx_L": 0.0694159181,
    "levator_scap": 0.0494874183,
    "levator_scap_L": 0.0494206141,
    "longissi_cap_sklc6": 0.0305173104,
    "longissi_cap_sklc6_L": 0.0305802837,
    "longissi_cerv_c4thx": 0.0687748785,
    "longissi_cerv_c4thx_L": 0.0686880523,
    "iliocost_cerv_c5rib": 0.0665554157,
    "iliocost_cerv_c5rib_L": 0.0664675579,
    "rectcap_post_maj": 0.0305236751,
    "rectcap_post_maj_L": 0.0305448742,
    "rectcap_post_min": 0.0307383191,
    "rectcap_post_min_L": 0.0308234885,
    "obl_cap_sup": 0.0320057410,
    "obl_cap_sup_L": 0.0321181604,
    "obl_cap_inf": 0.0358454823,
    "obl_cap_inf_L": 0.0358149574,
}


def _load_plugin_cross_platform(plugin_dir, base_name):
    if sys.platform.startswith("win"):
        candidates = [
            os.path.join(os.getcwd(), plugin_dir, "build", "Release", f"{base_name}.dll"),
            os.path.join(os.getcwd(), plugin_dir, "build", f"{base_name}.dll"),
        ]
    else:
        candidates = [
            os.path.join(os.getcwd(), plugin_dir, "build", f"lib{base_name}.so"),
            os.path.join(os.getcwd(), plugin_dir, "build", "Release", f"lib{base_name}.so"),
        ]
    for p in candidates:
        if os.path.exists(p):
            osim.LoadOpenSimLibrary(p)
            return
    osim.LoadOpenSimLibrary(candidates[0])


def ensure_libraries_loaded():
    global _LIBRARIES_LOADED
    if not _LIBRARIES_LOADED:
        os.environ["SIMB_NUM_THREADS"] = "1"
        os.environ["OMP_NUM_THREADS"] = "1"
        osim.Logger.setLevelString("Warn")
        if not sys.platform.startswith("win"):
            _load_plugin_cross_platform("vestibular_plugin", "osimVestibular")
        _load_plugin_cross_platform("proprioception_plugin", "osimMillard12EqWithAff")
        _load_plugin_cross_platform("HeadNeckNeuralController_plugin", "osimHeadNeckNeural")
        _load_plugin_cross_platform("FixationController_plugin", "osimFixation")
        _LIBRARIES_LOADED = True


def run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name="W-?", p_phas2=-1.0, z_phas=-1.0):
    ensure_libraries_loaded()

    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")

    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "FixationController":
            ctrl.set_enabled(False)
        elif ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl

    if controller is None:
        raise Exception("HeadNeckNeuralController not found!")

    # Pass the Phase 2.4 optimizer parameters (reflex gains, zeros, poles, phasic lead zero z_phas, and pitch task PID)
    # Literature-fixed poles & 2nd-order delay compensator zeros (tau_5 = 2.5 ms MVST, tau_6 = 4.0 ms Propriospinal)
    p_ton  = 15.0
    z_sc   = 0.0025
    p_sc   = 50.0
    p_phas = 20.0
    z_ccr  = 0.0040
    p_ccr  = 100.0
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

    # Literature-fixed Mahony SE_2(3) Observer relative weights (K_oto = 0.20 rad/s, K_prop = 2.50 rad/s)
    osim.PropertyHelper.setValueDouble(0.20, controller.updPropertyByName("Kp_otolith_attitude"))
    osim.PropertyHelper.setValueDouble(2.50, controller.updPropertyByName("K_proprioceptive_attitude_pitch"))
    osim.PropertyHelper.setValueDouble(2.50, controller.updPropertyByName("K_proprioceptive_attitude_roll"))
    osim.PropertyHelper.setValueDouble(2.50, controller.updPropertyByName("K_proprioceptive_attitude_yaw"))

    # Direct task space PID parameters (Pitch optimized by CMA-ES; Roll/Yaw fixed at analytical values)
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

    # Ki_task_pitch optimized by CMA-ES (initial state scaled below to preserve exact t=0 static gravity trim)
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

    # Lock extraocular coordinates during neck perturbation evaluation
    for eye_c in ["r_eye_add_abd", "r_eye_sup_inf", "r_eye_inc_exc", "l_eye_add_abd", "l_eye_sup_inf", "l_eye_inc_exc"]:
        if model.getCoordinateSet().contains(eye_c):
            c = model.updCoordinateSet().get(eye_c)
            c.setDefaultValue(0.0)
            c.setDefaultLocked(True)

    amplitude = 0.0215 * (0.835 / freq)**2
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
        if dt < ramp_duration:
            tau = dt / ramp_duration
            w = 10.0 * (tau**3) - 15.0 * (tau**4) + 6.0 * (tau**5)
        else:
            w = 1.0
        y = w * amplitude * math.sin(2.0 * math.pi * freq * dt)
        sine_function.addPoint(t, y)

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
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    # Zero trim at t=0 (72-muscle co-contraction balanced for exact 6-DOF upright static equilibrium)
    grav_trim_torque = 0.0
    init_pitch_integral = 0.0
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", init_pitch_integral)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", grav_trim_torque)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)

    # Isolate 72 neck muscles and initialize exact 6-DOF static equilibrium co-contraction & spindle states
    muscles = model.updMuscles()
    for i in range(muscles.getSize()):
        m = muscles.get(i)
        name = m.getName()
        if name.startswith("r_") or name.startswith("l_"):
            m.setAppliesForce(state, False)
            continue
        a0 = PER_MUSCLE_72_U0.get(name, 0.030)
        m.setActivation(state, a0)
        gd0 = 100.0 * k_gamma_dyn * a0
        gs0 = 100.0 * k_gamma_stat * a0
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
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    # Fastest sensor pole is Irregular Semicircular Canal tau_2,irr = 0.0006 s (0.6 ms, lambda = -1666.7 s^-1, h_crit = 1.2 ms).
    # min_step = 1e-4 s (0.1 ms, |h*lambda_max| = 0.167 << 1) avoids |Lp_dot|^0.3 micro-step throttling while max_step = 1e-3 s (1.0 ms < 1.2 ms) guarantees stability.
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.001)

    state.setTime(0.0)
    manager.initialize(state)

    step = min(0.01, (1.0/freq) / 40.0)
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

        # Read CNS internal observer, proprioceptive & PID states
        try:
            q0 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0")
            q1 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1")
            q2 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2")
            q3 = model.getStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3")
            qn = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
            if qn > 1e-6:
                q0, q1, q2, q3 = q0/qn, q1/qn, q2/qn, q3/qn
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
            abort_reason = (f"t={current_time:.2f}s: p1={p1_deg:+.2f}°, p2={p2_deg:+.2f}°, pitch={head_pitch_deg:+.2f}°, "
                            f"roll={head_roll_deg:+.2f}° (r1={r1_deg:+.2f}°,r2={r2_deg:+.2f}°), "
                            f"yaw={head_yaw_deg:+.2f}° (y1={y1_deg:+.2f}°,y2={y2_deg:+.2f}°)")
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

    m_pre = t_arr <= 1.5
    discard_time = delay + ramp_duration
    m_post = t_arr >= discard_time if np.any(t_arr >= discard_time) else np.ones_like(t_arr, dtype=bool)

    p0_ref = float(p_arr[0])
    mah0_ref = float(mah_arr[0])
    pre_min = float(np.min(p_arr[m_pre]) - p0_ref) if np.any(m_pre) else 0.0
    pre_max = float(np.max(p_arr[m_pre]) - p0_ref) if np.any(m_pre) else 0.0
    pre_drift = float(max(abs(pre_min), abs(pre_max)))

    mah_rel = mah_arr - mah0_ref
    p_rel = p_arr - p0_ref
    mah_rmse = float(np.sqrt(np.mean((mah_rel[m_post] - p_rel[m_post])**2))) if len(p_rel[m_post]) > 2 else 0.0
    mah_corr = float(np.corrcoef(mah_rel[m_post], p_rel[m_post])[0, 1]) if (len(p_rel[m_post]) > 2 and np.std(p_rel[m_post]) > 1e-5 and np.std(mah_rel[m_post]) > 1e-5) else 1.0
    scc_rmse = float(np.sqrt(np.mean((lw_arr[m_post] - tw_arr[m_post])**2))) if len(tw_arr[m_post]) > 2 else 0.0
    scc_corr = float(np.corrcoef(lw_arr[m_post], tw_arr[m_post])[0, 1]) if (len(tw_arr[m_post]) > 2 and np.std(tw_arr[m_post]) > 1e-5 and np.std(lw_arr[m_post]) > 1e-5) else 1.0

    telemetry = {
        "aborted": aborted,
        "abort_time_s": round(abort_time_s, 3),
        "abort_reason": abort_reason,
        "pre_pitch_drift_deg": round(pre_drift, 4),
        "post_pitch_min_deg": round(float(np.min(p_rel[m_post])), 3),
        "post_pitch_max_deg": round(float(np.max(p_rel[m_post])), 3),
        "post_pitch_mean_deg": round(float(np.mean(p_rel[m_post])), 3),
        "p1_min_deg": round(float(np.min(p1_arr[m_post])), 2),
        "p1_max_deg": round(float(np.max(p1_arr[m_post])), 2),
        "p2_min_deg": round(float(np.min(p2_arr[m_post])), 2),
        "p2_max_deg": round(float(np.max(p2_arr[m_post])), 2),
        "max_s_buckling_deg": round(float(np.max(np.abs((p1_arr - 2.063) - (p2_arr - 0.738)))), 2),
        "max_abs_roll_deg": round(float(np.max(np.abs(r_arr))), 3),
        "max_abs_roll1_deg": round(float(np.max(np.abs(r1_arr))), 3),
        "max_abs_roll2_deg": round(float(np.max(np.abs(r2_arr))), 3),
        "max_abs_yaw_deg": round(float(np.max(np.abs(y_arr))), 3),
        "max_abs_yaw1_deg": round(float(np.max(np.abs(y1_arr))), 3),
        "max_abs_yaw2_deg": round(float(np.max(np.abs(y2_arr))), 3),
        "mahony_pitch_rmse_deg": round(mah_rmse, 3),
        "mahony_pitch_corr": round(mah_corr, 4),
        "canal_luen_rmse_deg_s": round(scc_rmse, 2),
        "canal_luen_corr": round(scc_corr, 4),
        "spindle_dL_max_mm": round(float(np.max(np.abs(sp_arr))), 3),
        "pitch_integral_max": round(float(np.max(np.abs(pi_arr))), 4),
        "vol_pitch_torque_max_nm": round(float(np.max(np.abs(vp_arr))), 3)
    }

    if aborted:
        return 50000.0, 0.0, telemetry

    amp_deg, phase_rad = extract_bode(t_history, pitch_history, freq, discard_time, delay)

    gain_deg_m = amp_deg / amplitude
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0

    return gain_deg_m, phase_deg, telemetry


if __name__ == "__main__":
    if len(sys.argv) < 19:
        print(f"Usage: {sys.argv[0]} wn zeta ki K_ton p_ton K_sc z_sc p_sc K_phas p_phas kp_prop z_ccr p_ccr k_gamma_dyn k_gamma_stat [p_phas2 kp_task_pitch kd_task_pitch ki_task_pitch] freq target_gain target_phase [worker_name]", file=sys.stderr)
        sys.exit(1)

    omega_n = float(sys.argv[1])
    zeta = float(sys.argv[2])
    ki = float(sys.argv[3])

    raw_p0 = float(sys.argv[4])
    p_ton = float(sys.argv[5])
    K_sc = float(sys.argv[6])
    raw_z_sc = float(sys.argv[7])
    p_sc = float(sys.argv[8])

    K_ton = raw_p0
    r_lead_sc = raw_z_sc
    z_sc = r_lead_sc * p_sc

    K_phas = float(sys.argv[9])
    p_phas = float(sys.argv[10])
    kp_prop = float(sys.argv[11])
    raw_z_ccr = float(sys.argv[12])
    p_ccr = float(sys.argv[13])

    r_lead_ccr = raw_z_ccr
    z_ccr = r_lead_ccr * p_ccr

    k_gamma_dyn = float(sys.argv[14])
    k_gamma_stat = float(sys.argv[15])

    if len(sys.argv) >= 20 and sys.argv[19].upper() == "ALL":
        r_lead_phas = -1.0
        z_phas = -1.0
        p_phas2 = -1.0
        kp = float(sys.argv[16])
        kd = float(sys.argv[17])
        ki = float(sys.argv[18])
        freq = None
        target_gain = None
        target_phase = None
        worker_name = sys.argv[20] if len(sys.argv) >= 21 else f"W-{os.getpid()}"
        saved_params = [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, kp, kd, ki]
    elif len(sys.argv) >= 21 and sys.argv[20].upper() == "ALL":
        raw_p12 = float(sys.argv[16])
        if 0.0 < raw_p12 <= 1.0:
            r_lead_phas = raw_p12
            z_phas = r_lead_phas * p_phas
            p_phas2 = -1.0
        else:
            r_lead_phas = -1.0
            z_phas = -1.0
            p_phas2 = raw_p12
        kp = float(sys.argv[17])
        kd = float(sys.argv[18])
        ki = float(sys.argv[19])
        freq = None
        target_gain = None
        target_phase = None
        worker_name = sys.argv[21] if len(sys.argv) >= 22 else f"W-{os.getpid()}"
        saved_params = [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, raw_p12, kp, kd, ki]
    elif len(sys.argv) == 22 or (len(sys.argv) == 23 and not sys.argv[22].replace('.', '', 1).replace('-', '', 1).isdigit()):
        r_lead_phas = -1.0
        z_phas = -1.0
        p_phas2 = -1.0
        kp = float(sys.argv[16])
        kd = float(sys.argv[17])
        ki = float(sys.argv[18])
        freq = float(sys.argv[19])
        target_gain = float(sys.argv[20])
        target_phase = float(sys.argv[21])
        worker_name = sys.argv[22] if len(sys.argv) >= 23 else f"W-{os.getpid()}"
        saved_params = [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, kp, kd, ki]
    elif len(sys.argv) >= 23:
        raw_p12 = float(sys.argv[16])
        if 0.0 < raw_p12 <= 1.0:
            r_lead_phas = raw_p12
            z_phas = r_lead_phas * p_phas
            p_phas2 = -1.0
        else:
            r_lead_phas = -1.0
            z_phas = -1.0
            p_phas2 = raw_p12
        kp = float(sys.argv[17])
        kd = float(sys.argv[18])
        ki = float(sys.argv[19])
        freq = float(sys.argv[20])
        target_gain = float(sys.argv[21])
        target_phase = float(sys.argv[22])
        worker_name = sys.argv[23] if len(sys.argv) >= 24 else f"W-{os.getpid()}"
        saved_params = [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, raw_p12, kp, kd, ki]
    else:
        r_lead_phas = -1.0
        z_phas = -1.0
        p_phas2 = -1.0
        kp = 0.0
        kd = 0.0
        freq = float(sys.argv[16])
        target_gain = float(sys.argv[17])
        target_phase = float(sys.argv[18])
        worker_name = sys.argv[19] if len(sys.argv) >= 20 else f"W-{os.getpid()}"
        saved_params = [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat]

    try:
        try:
            cand_dir = os.path.join("logs", "candidates")
            os.makedirs(cand_dir, exist_ok=True)
            cand_file = os.path.join(cand_dir, f"{worker_name}.json")
            with open(cand_file, "w", encoding="utf-8") as f_cand:
                json.dump({
                    "worker": worker_name,
                    "params": saved_params
                }, f_cand)
        except Exception:
            pass

        if freq is None:
            total_score = 0.0
            for t_idx, (f_val, t_gain, t_phase) in enumerate(CANONICAL_TARGETS):
                try:
                    sim_gain, sim_phase, telemetry = run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, f_val, worker_name, p_phas2, z_phas)
                    sim_gain_safe = max(1e-6, sim_gain)
                    gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
                    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
                    phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
                    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
                    error = gain_loss + phase_loss
                    penalty = 0.0
                    if kp > 500: penalty += (kp - 500) * 10
                    if ki > 500: penalty += (ki - 500) * 10
                    if telemetry.get("aborted", False):
                        f_score = 200000.0 + max(0.0, 5.0 - telemetry.get("abort_time_s", 0.0)) * 10000.0
                    else:
                        f_score = error + penalty
                    try:
                        cand_dir = os.path.join("logs", "candidates")
                        os.makedirs(cand_dir, exist_ok=True)
                        res_file = os.path.join(cand_dir, f"{worker_name}_{f_val:.2f}Hz.json")
                        with open(res_file, "w", encoding="utf-8") as f_res:
                            json.dump({
                                "worker": worker_name,
                                "freq": f_val,
                                "target_gain": t_gain,
                                "sim_gain": sim_gain,
                                "gain_err_pct": abs(sim_gain - t_gain) / t_gain * 100.0,
                                "gain_db_diff": gain_db_diff,
                                "gain_loss": gain_loss,
                                "target_phase": t_phase,
                                "sim_phase": sim_phase,
                                "phase_err_deg": abs(phase_diff),
                                "phase_loss": phase_loss,
                                "score": f_score,
                                "params": saved_params,
                                "telemetry": telemetry
                            }, f_res)
                    except Exception:
                        pass
                except Exception:
                    import traceback
                    traceback.print_exc(file=sys.stderr)
                    f_score = 1050000.0

                if f_score < 150000.0:
                    total_score += f_score
                    print(f"[PASS] [{worker_name}] passed {f_val:.3f}Hz ({t_idx+1}/{len(CANONICAL_TARGETS)}) (Score: {f_score:.1f} | Cum: {total_score:.1f})", flush=True)
                else:
                    early_stop_penalty = (len(CANONICAL_TARGETS) - t_idx) * 200000.0
                    total_score += (f_score + early_stop_penalty)
                    print(f"[EARLY STOPPING] [{worker_name}] aborted on {f_val:.3f}Hz (Score: {f_score:.1f}). Penalty added.", flush=True)
                    break
            else:
                print(f"[COMPLETE] [{worker_name}] finished all 6 frequencies! Total Score: {total_score:.2f}", flush=True)
            print(f"{total_score}")
            sys.exit(0)

        sim_gain, sim_phase, telemetry = run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name, p_phas2, z_phas)

        sim_gain_safe = max(1e-6, sim_gain)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / target_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0

        phase_diff = (sim_phase - target_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0

        error = gain_loss + phase_loss

        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10

        if telemetry.get("aborted", False):
            final_score = 200000.0 + max(0.0, 5.0 - telemetry.get("abort_time_s", 0.0)) * 10000.0
        else:
            final_score = error + penalty
        try:
            cand_dir = os.path.join("logs", "candidates")
            os.makedirs(cand_dir, exist_ok=True)
            res_file = os.path.join(cand_dir, f"{worker_name}_{freq:.2f}Hz.json")
            with open(res_file, "w", encoding="utf-8") as f_res:
                json.dump({
                    "worker": worker_name,
                    "freq": freq,
                    "target_gain": target_gain,
                    "sim_gain": sim_gain,
                    "gain_err_pct": abs(sim_gain - target_gain) / target_gain * 100.0,
                    "gain_db_diff": gain_db_diff,
                    "gain_loss": gain_loss,
                    "target_phase": target_phase,
                    "sim_phase": sim_phase,
                    "phase_err_deg": abs(phase_diff),
                    "phase_loss": phase_loss,
                    "score": final_score,
                    "params": saved_params,
                    "telemetry": telemetry
                }, f_res)
        except Exception:
            pass
        print(f"{final_score}")
        sys.exit(0)
    except Exception as e:
        import traceback
        traceback.print_exc(file=sys.stderr)
        survived = current_time if 'current_time' in locals() else 0.0
        survival_penalty = 1000000.0 + (5.0 - survived) * 10000.0
        print(f"{survival_penalty}")
        sys.exit(0)
