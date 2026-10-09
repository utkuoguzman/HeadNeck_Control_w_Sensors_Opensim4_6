import opensim as osim
import numpy as np
import math
import os
import sys

def extract_bode(t_array, pitch_array, freq, discard_time, delay):
    valid_indices = [i for i, t in enumerate(t_array) if t >= discard_time]
    t_steady = np.array([t_array[i] for i in valid_indices])
    pitch_steady = np.array([pitch_array[i] for i in valid_indices])
    
    if len(t_steady) < 10:
        return 0.0, 0.0
        
    omega = 2.0 * math.pi * freq
    
    # We must phase shift the sine and cosine because our sine wave didn't start at t=0, it started at t=delay.
    # The perturbation is A * sin(omega * (t - delay)).
    # So we correlate against sin(omega * (t - delay)) and cos(omega * (t - delay)).
    # We can handle this elegantly by extracting relative to the actual sled position function!
    # Or just mathematically subtract the delay from t_steady!
    t_shifted = t_steady - delay # We hardcoded delay=0.5
    
    # NEW: Detrend the data! If the head is slowly drooping over 6 seconds, 
    # it ruins the Fourier extraction. We must remove the linear drift!
    coeffs = np.polyfit(t_steady, pitch_steady, 1) # fit a line (slope and intercept)
    pitch_detrended = pitch_steady - (coeffs[0] * t_steady + coeffs[1])
    
    Is = 2.0 * np.mean(pitch_detrended * np.sin(omega * t_shifted))
    Ic = 2.0 * np.mean(pitch_detrended * np.cos(omega * t_shifted))
    
    amplitude_rad = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    
    return amplitude_rad, phase_rad

def run_simulation(kp, ki, kd, K_ton, z_ton, p_ton, K_sc, z_sc, p_sc, K_phas, z_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name="W-?"):
    # Prevent Simbody from secretly spawning hundreds of internal threads and thrashing the CPU!
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    osim.Logger.setLevelString("Warn")
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
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

    # Pass the optimizer's parameters to the controller
    osim.PropertyHelper.setValueDouble(K_ton, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(z_ton, controller.updPropertyByName("z_ton"))
    osim.PropertyHelper.setValueDouble(p_ton, controller.updPropertyByName("p_ton"))
    osim.PropertyHelper.setValueDouble(K_sc, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(z_sc, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(p_sc, controller.updPropertyByName("p_sc"))
    osim.PropertyHelper.setValueDouble(K_phas, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(z_phas, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(p_phas, controller.updPropertyByName("p_phas"))
    
    osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.13, controller.updPropertyByName("k_v"))

    osim.PropertyHelper.setValueDouble(kp, controller.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(kd, controller.updPropertyByName("Kd_task"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(z_ccr, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(p_ccr, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))


    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    amplitude = 0.0215 * (0.835 / freq)**2
    
    # User requested: Wait 0.5s for PID I-term to settle, THEN apply Sine wave for 2 periods
    delay = 0.5
    period = 1.0 / freq
    target_time = delay + 2.0 * period # 2 full periods
    
    import numpy as np
    sine_function = osim.SimmSpline()
    # Flat zero for the first 0.5 seconds
    for t in np.arange(0.0, delay, 0.05):
        sine_function.addPoint(t, 0.0)
    # Sine wave starting from t=0.5
    for t in np.arange(delay, target_time + 0.1, 0.01):
        y = amplitude * math.sin(2 * math.pi * freq * (t - delay))
        sine_function.addPoint(t, y)

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()
    
    # We no longer need an initial velocity spike because the spline smoothly starts at 0.0
    coord_tx.setSpeedValue(state, 0.0)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)
    
    state.setTime(0.0)
    manager.initialize(state)
        
    # Perfect SE(3) Observer Initialization
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    # NEW OPTIMIZATION: Pre-load the PID integral terms so we don't have to wait 2.5s to settle!
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", -0.01906)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", -0.01184)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.06309)


    step = min(0.01, (1.0/freq) / 40.0) # Always get at least 40 points per period! 
    current_t = 0.0
    
    t_history = []
    pitch_history = []
    
    worker_id = worker_name
    
    while current_t < target_time:
        next_t = current_t + step
        if next_t > target_time:
            next_t = target_time
        manager.integrate(next_t)
        current_t = next_t
        
        st = manager.getState()
        p1 = model.getCoordinateSet().get("pitch1").getValue(st)
        p2 = model.getCoordinateSet().get("pitch2").getValue(st)
        r1 = model.getCoordinateSet().get("roll1").getValue(st) if model.getCoordinateSet().contains("roll1") else 0.0
        r2 = model.getCoordinateSet().get("roll2").getValue(st) if model.getCoordinateSet().contains("roll2") else 0.0
        y1 = model.getCoordinateSet().get("yaw1").getValue(st) if model.getCoordinateSet().contains("yaw1") else 0.0
        y2 = model.getCoordinateSet().get("yaw2").getValue(st) if model.getCoordinateSet().contains("yaw2") else 0.0
        
        head_pitch_deg = math.degrees(p1 + p2)
        head_roll_deg = math.degrees(r1 + r2)
        head_yaw_deg = math.degrees(y1 + y2)
        t_history.append(current_t)
        pitch_history.append(p1 + p2)
        
        # EARLY ABORT: If the head collapses or explodes, kill the simulation instantly!
        if abs(head_pitch_deg) > 45.0 or abs(head_roll_deg) > 45.0 or abs(head_yaw_deg) > 45.0:
            print(f"❌ [{worker_id} | {freq:.2f}Hz] ❌ KILLED: Head collapsed! P:{head_pitch_deg:.1f} R:{head_roll_deg:.1f} Y:{head_yaw_deg:.1f}", file=sys.stderr, flush=True)
            # Give a graded penalty based on how early it failed so CMA-ES has a gradient
            survival_penalty = 1000000.0 + (5.0 - current_t) * 10000.0
            print(f"{survival_penalty}")
            sys.exit(0)

        
        # Only print occasionally to avoid spamming the console too hard
        if round(current_t * 100) % 20 == 0: 
            print(f"⏳ [{worker_id} | {freq:.2f}Hz] t={current_t:.2f}s | P: {head_pitch_deg:5.1f} | R: {head_roll_deg:5.1f} | Y: {head_yaw_deg:5.1f}", file=sys.stderr, flush=True)
        
    # Discard the first period to allow transient to settle, measure the second period.
    discard_time = delay + period
    amp_rad, phase_rad = extract_bode(t_history, pitch_history, freq, discard_time, delay)
    
    amp_deg = amp_rad * (180.0 / math.pi)
    
    # Calculate acceleration amplitude of the sled
    accel_amplitude = amplitude * (2.0 * math.pi * freq)**2
    gain_deg_ms2 = amp_deg / accel_amplitude
    phase_deg = phase_rad * (180.0 / math.pi)
    if phase_deg > 90: phase_deg -= 360
    
    return gain_deg_ms2, phase_deg

if __name__ == "__main__":
    import sys
    import os
    if len(sys.argv) < 21:
        sys.exit(1)
        
    omega_n = float(sys.argv[1])
    zeta = float(sys.argv[2])
    ki = float(sys.argv[3])
    
    # TF Parameters (14 total)
    K_ton = float(sys.argv[4])
    z_ton = float(sys.argv[5])
    p_ton = float(sys.argv[6])
    
    K_sc = float(sys.argv[7])
    z_sc = float(sys.argv[8])
    p_sc = float(sys.argv[9])
    
    K_phas = float(sys.argv[10])
    z_phas = float(sys.argv[11])
    p_phas = float(sys.argv[12])
    
    kp_prop = float(sys.argv[13])
    z_ccr = float(sys.argv[14])
    p_ccr = float(sys.argv[15])
    k_gamma_dyn = float(sys.argv[16])
    k_gamma_stat = float(sys.argv[17])
    
    freq = float(sys.argv[18])
    target_gain = float(sys.argv[19])
    target_phase = float(sys.argv[20])
    
    
    I_eff = 1.0
    kp = I_eff * (omega_n ** 2)
    kd = 0.0 # Force no instantaneous derivative feedback!
    
    try:
        worker_name = sys.argv[21] if len(sys.argv) >= 22 else f"W-{os.getpid()}"
        sim_gain, sim_phase = run_simulation(kp, ki, kd, K_ton, z_ton, p_ton, K_sc, z_sc, p_sc, K_phas, z_phas, p_phas, kp_prop, z_ccr, p_ccr, k_gamma_dyn, k_gamma_stat, freq, worker_name)
        # Frequency-Weighted Cost Function
        # Low Freq (< 1 Hz): Phase is highly critical for postural stability.
        # High Freq (> 3 Hz): System acts as a shock absorber. Gain (stiffness) is critical, but exact phase lag is dominated by biological delay and matters less.
        if freq < 1.0:
            w_gain = 500.0
            w_phase = 1500.0
        elif freq < 3.0:
            w_gain = 1000.0
            w_phase = 1000.0
        else:
            w_gain = 1500.0
            w_phase = 200.0  # Relax phase constraint at high frequencies
            
        error = ((sim_gain - target_gain)/target_gain)**2 * w_gain + ((sim_phase - target_phase)/180.0)**2 * w_phase
        
        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10
        
        final_score = error + penalty
        print(f"{final_score}")
        sys.exit(0)
    except Exception as e:
        import traceback
        traceback.print_exc(file=sys.stderr)
        survived = current_t if 'current_t' in locals() else 0.0
        survival_penalty = 1000000.0 + (5.0 - survived) * 10000.0
        print(f"{survival_penalty}")
        sys.exit(0)
