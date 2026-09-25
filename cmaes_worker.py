import opensim as osim
import numpy as np
import math
import os
import sys

def extract_bode(t_array, pitch_array, frequency, discard_time):
    valid_indices = [i for i, t in enumerate(t_array) if t >= discard_time]
    t_steady = np.array([t_array[i] for i in valid_indices])
    pitch_steady = np.array([pitch_array[i] for i in valid_indices])
    
    if len(t_steady) < 10:
        return 0.0, 0.0
        
    omega = 2.0 * math.pi * frequency
    
    # We must phase shift the sine and cosine because our sine wave didn't start at t=0, it started at t=delay.
    # The perturbation is A * sin(omega * (t - delay)).
    # So we correlate against sin(omega * (t - delay)) and cos(omega * (t - delay)).
    # We can handle this elegantly by extracting relative to the actual sled position function!
    # Or just mathematically subtract the delay from t_steady!
    t_shifted = t_steady - 0.5 # We hardcoded delay=0.5
    
    Is = 2.0 * np.mean(pitch_steady * np.sin(omega * t_shifted))
    Ic = 2.0 * np.mean(pitch_steady * np.cos(omega * t_shifted))
    
    amplitude_rad = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    
    return amplitude_rad, phase_rad

def run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, frequency, worker_name="W-?"):
    # Prevent Simbody from secretly spawning hundreds of internal threads and thrashing the CPU!
    os.environ["SIMB_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "3"
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
    osim.PropertyHelper.setValueDouble(g_ton, controller.updPropertyByName("G_ton"))
    osim.PropertyHelper.setValueDouble(g_sc, controller.updPropertyByName("G_sc"))
    osim.PropertyHelper.setValueDouble(g_phas, controller.updPropertyByName("G_phas"))
    
    osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.13, controller.updPropertyByName("k_v"))

    osim.PropertyHelper.setValueDouble(kp, controller.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(kd, controller.updPropertyByName("Kd_task"))
    osim.PropertyHelper.setValueDouble(kp_prop, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(k_gamma_dyn, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(k_gamma_stat, controller.updPropertyByName("K_gamma_stat"))


    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    amplitude = 0.0215 * (0.835 / frequency)**2
    
    # User requested: Wait 0.5s for PID I-term to settle, THEN apply Sine wave for 2 periods
    delay = 0.5
    period = 1.0 / frequency
    target_time = delay + 2.0 * period # 2 full periods
    
    import numpy as np
    sine_function = osim.SimmSpline()
    # Flat zero for the first 0.5 seconds
    for t in np.arange(0.0, delay, 0.05):
        sine_function.addPoint(t, 0.0)
    # Sine wave starting from t=0.5
    for t in np.arange(delay, target_time + 0.1, 0.01):
        y = amplitude * math.sin(2 * math.pi * frequency * (t - delay))
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
    manager.setIntegratorMinimumStepSize(1e-8)
    manager.setIntegratorMaximumStepSize(0.01)
    
    state.setTime(0.0)
    manager.initialize(state)

    step = 0.05 
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
        
        # Only print occasionally to avoid spamming the console too hard
        if round(current_t * 100) % 20 == 0: 
            print(f"[{worker_id} | {frequency:.2f}Hz] t={current_t:.2f}s | P: {head_pitch_deg:5.1f} | R: {head_roll_deg:5.1f} | Y: {head_yaw_deg:5.1f}", file=sys.stderr, flush=True)
        
    # Discard the first period to allow transient to settle, measure the second period.
    discard_time = delay + period
    amp_rad, phase_rad = extract_bode(t_history, pitch_history, frequency, discard_time)
    
    amp_deg = amp_rad * (180.0 / math.pi)
    gain_deg_m = amp_deg / amplitude
    phase_deg = phase_rad * (180.0 / math.pi)
    if phase_deg > 90: phase_deg -= 360
    
    return gain_deg_m, phase_deg

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 13:
        sys.exit(1)
        
    kp = float(sys.argv[1])
    ki = float(sys.argv[2])
    kd = float(sys.argv[3])
    g_ton = float(sys.argv[4])
    g_sc = float(sys.argv[5])
    g_phas = float(sys.argv[6])
    kp_prop = float(sys.argv[7])
    k_gamma_dyn = float(sys.argv[8])
    k_gamma_stat = float(sys.argv[9])
    
    freq = float(sys.argv[10])
    target_gain = float(sys.argv[11])
    target_phase = float(sys.argv[12])
    
    try:
        worker_name = sys.argv[13] if len(sys.argv) >= 14 else f"W-{os.getpid()}"
        sim_gain, sim_phase = run_simulation(kp, ki, kd, g_ton, g_sc, g_phas, kp_prop, k_gamma_dyn, k_gamma_stat, freq, worker_name)
        error = (sim_gain - target_gain)**2 + 0.5*(sim_phase - target_phase)**2
        
        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10
        
        final_score = error + penalty
        print(f"{final_score}")
        sys.exit(0)
    except Exception as e:
        import traceback
        traceback.print_exc(file=sys.stderr)
        print("100000.0")
        sys.exit(1)

