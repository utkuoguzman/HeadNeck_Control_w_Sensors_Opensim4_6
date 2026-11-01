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


def ensure_libraries_loaded():
    global _LIBRARIES_LOADED
    if not _LIBRARIES_LOADED:
        os.environ["SIMB_NUM_THREADS"] = "1"
        os.environ["OMP_NUM_THREADS"] = "1"
        osim.Logger.setLevelString("Warn")
        osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
        osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
        osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
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
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    # Preserve exact t=0 static gravity equilibrium torque (40.2786 * 0.10528 = 4.24053 Nm) across all Ki_task_pitch values
    grav_trim_torque = 40.2786 * 0.10528
    init_pitch_integral = (grav_trim_torque / max(1.0, ki)) if ki > 0.0 else 0.10528
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", init_pitch_integral)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", grav_trim_torque)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt2_pitch", grav_trim_torque)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt2_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt2_yaw", 0.0)
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)

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

        if (p1_deg < -12.0 or p1_deg > 18.0 or
            p2_deg < -24.75 or p2_deg > 36.0 or
            head_pitch_deg < -36.75 or head_pitch_deg > 54.0 or
            abs(r1_deg) > 4.5 or abs(r2_deg) > 24.75 or abs(head_roll_deg) > 29.25 or
            abs(y1_deg) > 28.5 or abs(y2_deg) > 20.25 or abs(head_yaw_deg) > 48.75):
            print(f"[ABORT TRANSITION] {worker_name} entered transition zone at t={current_time:.2f}s "
                  f"(p1={p1_deg:.1f}°, p2={p2_deg:.1f}°, pitch={head_pitch_deg:.1f}°, "
                  f"r1={r1_deg:.1f}°, r2={r2_deg:.1f}°, roll={head_roll_deg:.1f}°, "
                  f"y1={y1_deg:.1f}°, y2={y2_deg:.1f}°, yaw={head_yaw_deg:.1f}°)", file=sys.stderr)
            return 50000.0, 0.0

        t_history.append(current_time)
        pitch_history.append(head_pitch_deg)

    discard_time = delay + ramp_duration
    amp_deg, phase_rad = extract_bode(t_history, pitch_history, freq, discard_time, delay)

    gain_deg_m = amp_deg / amplitude
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0

    return gain_deg_m, phase_deg


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

    if len(sys.argv) >= 21 and sys.argv[20].upper() == "ALL":
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
                    sim_gain, sim_phase = run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, f_val, worker_name, p_phas2, z_phas)
                    sim_gain_safe = max(1e-6, sim_gain)
                    gain_db_diff = 20.0 * math.log10(sim_gain_safe / t_gain)
                    gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0
                    phase_diff = (sim_phase - t_phase + 180.0) % 360.0 - 180.0
                    phase_loss = ((phase_diff / 10.0) ** 2) * 100.0
                    error = gain_loss + phase_loss
                    penalty = 0.0
                    if kp > 500: penalty += (kp - 500) * 10
                    if ki > 500: penalty += (ki - 500) * 10
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
                                "params": saved_params
                            }, f_res)
                    except Exception:
                        pass
                except Exception:
                    import traceback
                    traceback.print_exc(file=sys.stderr)
                    f_score = 1050000.0

                if f_score < 250000.0:
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

        sim_gain, sim_phase = run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name, p_phas2, z_phas)

        sim_gain_safe = max(1e-6, sim_gain)
        gain_db_diff = 20.0 * math.log10(sim_gain_safe / target_gain)
        gain_loss = ((gain_db_diff / 1.0) ** 2) * 100.0

        phase_diff = (sim_phase - target_phase + 180.0) % 360.0 - 180.0
        phase_loss = ((phase_diff / 10.0) ** 2) * 100.0

        error = gain_loss + phase_loss

        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10

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
                    "params": saved_params
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
