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

def run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name="W-?"):
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

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

    # Pass the Phase 2.2 optimizer parameters (12 total: 2 active zeros, 4 poles, 4 gains, 2 gamma drives)
    osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("k_p"))

    # Base LQR costs
    for ax in ['pitch', 'roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_vcr"))
    osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

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
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_pitch", ki * 0.10528)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_roll", 0.0)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/vol_filt_yaw", 0.0)
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

    t_history = []
    pitch_history = []

    while current_time < target_time:
        current_time = min(current_time + step, target_time)
        state = manager.integrate(current_time)

        head_pitch_deg = math.degrees(coord_p1.getValue(state) + coord_p2.getValue(state))
        head_roll_deg = math.degrees(coord_r1.getValue(state) + coord_r2.getValue(state))
        head_yaw_deg = math.degrees(coord_y1.getValue(state))

        # Cross-talk stability guard: Prune candidates that wobble laterally beyond 30 deg
        if abs(head_pitch_deg) > 45.0 or abs(head_roll_deg) > 30.0 or abs(head_yaw_deg) > 30.0:
            penalty = 1000000.0 + (target_time - current_time) * 10000.0
            print(f"[ABORT] {worker_name} lateral/pitch divergence at t={current_time:.2f}s (P={head_pitch_deg:.1f}, R={head_roll_deg:.1f}, Y={head_yaw_deg:.1f})", file=sys.stderr)
            return 10000.0, 0.0

        t_history.append(current_time)
        pitch_history.append(head_pitch_deg)

    discard_time = delay + ramp_duration
    amp_deg, phase_rad = extract_bode(t_history, pitch_history, freq, discard_time, delay)
    
    accel_amplitude = amplitude * (2.0 * math.pi * freq)**2
    gain_deg_ms2 = amp_deg / accel_amplitude
    phase_deg = math.degrees(phase_rad)
    while phase_deg > 90.0: phase_deg -= 360.0
    while phase_deg < -270.0: phase_deg += 360.0
    
    return gain_deg_ms2, phase_deg

if __name__ == "__main__":
    if len(sys.argv) < 19:
        print(f"Usage: {sys.argv[0]} wn zeta ki K_ton p_ton K_sc z_sc p_sc K_phas p_phas kp_prop z_ccr p_ccr k_gamma_dyn k_gamma_stat freq target_gain target_phase [worker_name]", file=sys.stderr)
        sys.exit(1)
        
    omega_n = float(sys.argv[1])
    zeta = float(sys.argv[2])
    ki = float(sys.argv[3])
    
    # 12 TF Parameters
    raw_p0 = float(sys.argv[4])
    p_ton = float(sys.argv[5])
    
    K_sc = float(sys.argv[6])
    raw_z_sc = float(sys.argv[7])
    p_sc = float(sys.argv[8])
    
    # Biological Ratio Coupling: r_ton = K_ton / K_sc
    if raw_p0 <= 5.0:
        r_ton = raw_p0
        K_ton = r_ton * K_sc
    else:
        K_ton = raw_p0
        r_ton = K_ton / K_sc
        
    # Biological Lead Ratio Coupling: z_sc = r_lead_sc * p_sc
    if raw_z_sc <= 1.0:
        r_lead_sc = raw_z_sc
        z_sc = r_lead_sc * p_sc
    else:
        z_sc = raw_z_sc
        r_lead_sc = z_sc / p_sc
    
    K_phas = float(sys.argv[9])
    p_phas = float(sys.argv[10])
    
    kp_prop = float(sys.argv[11])
    raw_z_ccr = float(sys.argv[12])
    p_ccr = float(sys.argv[13])
    
    # Biological Lead Ratio Coupling: z_ccr = r_lead_ccr * p_ccr
    if raw_z_ccr <= 1.0:
        r_lead_ccr = raw_z_ccr
        z_ccr = r_lead_ccr * p_ccr
    else:
        z_ccr = raw_z_ccr
        r_lead_ccr = z_ccr / p_ccr
    
    k_gamma_dyn = float(sys.argv[14])
    k_gamma_stat = float(sys.argv[15])
    
    freq = float(sys.argv[16])
    target_gain = float(sys.argv[17])
    target_phase = float(sys.argv[18])
    
    I_eff = 1.0
    kp = I_eff * (omega_n ** 2)
    kd = 0.0
    
    try:
        worker_name = sys.argv[19] if len(sys.argv) >= 20 else f"W-{os.getpid()}"
        try:
            cand_dir = os.path.join("logs", "candidates")
            os.makedirs(cand_dir, exist_ok=True)
            cand_file = os.path.join(cand_dir, f"{worker_name}.json")
            if not os.path.exists(cand_file):
                with open(cand_file, "w", encoding="utf-8") as f_cand:
                    json.dump({
                        "worker": worker_name,
                        "params": [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat]
                    }, f_cand)
        except Exception:
            pass
        sim_gain, sim_phase = run_simulation(kp, ki, kd, K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name)
        
        # Phase 2.2: Isotropic Equal-Corridor Bode Loss
        # 1.0 dB Gain Error == 10.0 deg Phase Error == 100 pts
        # Exact vertical distance difference on standard Bode magnitude (dB) and phase (deg) axes
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
                    "params": [K_ton, p_ton, K_sc, z_sc, p_sc, K_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat]
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

