import opensim as osim
import numpy as np
import math
import os
import cma

print("Loading OpenSim libraries...")
osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

def extract_bode(t_array, pitch_array, frequency):
    # Ignore the first 1.5 seconds to let the initial droop transient settle
    valid_indices = [i for i, t in enumerate(t_array) if t > 1.5]
    t_steady = np.array([t_array[i] for i in valid_indices])
    pitch_steady = np.array([pitch_array[i] for i in valid_indices])
    
    if len(t_steady) < 10:
        return 0.0, 0.0
        
    omega = 2.0 * math.pi * frequency
    
    # Simple Fourier extraction (inner product)
    # B * sin(w*t + phi) = B*cos(phi)*sin(w*t) + B*sin(phi)*cos(w*t)
    # Is = B*cos(phi), Ic = B*sin(phi)
    Is = 2.0 * np.mean(pitch_steady * np.sin(omega * t_steady))
    Ic = 2.0 * np.mean(pitch_steady * np.cos(omega * t_steady))
    
    amplitude_rad = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    
    return amplitude_rad, phase_rad

def run_simulation(kp, ki, kd, frequency):
    # Reload model to avoid state corruption
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

    # DO NOT FORGET BASELINE REFLEXES
    osim.PropertyHelper.setValueDouble(2.0, controller.updPropertyByName("G_ton"))
    osim.PropertyHelper.setValueDouble(1.73, controller.updPropertyByName("G_sc"))
    osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.13, controller.updPropertyByName("k_v"))

    # Set PID parameters from CMA-ES
    osim.PropertyHelper.setValueDouble(kp, controller.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(kd, controller.updPropertyByName("Kd_task"))

    # Setup sled prescribed motion
    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    # Scale amplitude inversely with frequency squared to maintain constant peak acceleration
    amplitude = 0.0215 * (0.835 / frequency)**2
    sine_function = osim.Sine(amplitude, 2 * math.pi * frequency, 0)
    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()

    # Apply exact initial velocity for sled
    v_initial = amplitude * 2 * math.pi * frequency * math.cos(0)
    coord_tx.setSpeedValue(state, v_initial)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-8)
    manager.setIntegratorMaximumStepSize(0.01)

    state.setTime(0.0)
    manager.initialize(state)

    target_time = 5.0 # Extended to capture full cycle of 0.32Hz
    step = 0.05 # Back to 0.05 for speed
    current_t = 0.0
    
    t_history = []
    pitch_history = []
    
    while current_t < target_time:
        next_t = current_t + step
        if next_t > target_time:
            next_t = target_time
        
        manager.integrate(next_t)
        current_t = next_t
        
        p1 = model.getCoordinateSet().get("pitch1").getValue(manager.getState())
        p2 = model.getCoordinateSet().get("pitch2").getValue(manager.getState())
        t_history.append(current_t)
        pitch_history.append(p1 + p2)
        
    amp_rad, phase_rad = extract_bode(t_history, pitch_history, frequency)
    
    # Convert to Happee's Units: Gain [deg / m], Phase [deg]
    amp_deg = amp_rad * (180.0 / math.pi)
    gain_deg_m = amp_deg / amplitude
    phase_deg = phase_rad * (180.0 / math.pi)
    
    # Normalize phase to be around -90 to 0 (sometimes atan2 wraps to +270)
    if phase_deg > 90: phase_deg -= 360
    
    return gain_deg_m, phase_deg

def objective_wrapper(x):
    kp, ki, kd = x
    
    if kp < 2.0 or ki < 0.0 or kd < 0.0:
        return 100000.0
        
    targets = [
        (0.323929, 50.868070, -77.524752),
        (0.442127, 63.821646, -82.694072),
        (0.603456, 81.029187, -86.129080),
        (0.823652, 106.268348, -85.537188),
        (1.124195, 149.430277, -81.089109)
    ]
    
    total_error = 0.0
    
    try:
        for freq, target_gain, target_phase in targets:
            sim_gain, sim_phase = run_simulation(kp, ki, kd, freq)
            
            # Weighted error (Gain and Phase are roughly same magnitude, so direct sum of squares is okay)
            gain_error = (sim_gain - target_gain)**2
            phase_error = (sim_phase - target_phase)**2
            
            total_error += gain_error + phase_error
            
        # Penalize biologically extreme gains
        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10
        
        final_score = total_error + penalty
        print(f"Eval: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} => Bode Error: {final_score:.1f}")
        return final_score
    except Exception as e:
        print(f"Eval FAILED: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} => {e}")
        return 100000.0

if __name__ == "__main__":
    print("Starting Bode Plot CMA-ES Optimization (Serial Mode)...")
    
    # Initial guess based on your previous tuning
    x0 = [50.0, 40.0, 5.0]
    sigma0 = 10.0 
    
    es = cma.CMAEvolutionStrategy(x0, sigma0, {'bounds': [[2.0, 0.0, 0.0], [500.0, 500.0, 50.0]], 'popsize': 16})
    
    iteration = 0
    while not es.stop() and iteration < 500: # Run overnight until full mathematical convergence
        iteration += 1
        print(f"\n--- Generation {iteration} ---")
        solutions = es.ask()
        fitnesses = [objective_wrapper(x) for x in solutions]
        es.tell(solutions, fitnesses)
        es.disp()
        
    res = es.result
    print("\n===============================")
    print("Optimization Finished!")
    print(f"Best Kp: {res.xbest[0]:.2f}")
    print(f"Best Ki: {res.xbest[1]:.2f}")
    print(f"Best Kd: {res.xbest[2]:.2f}")
    print(f"Best Bode SSE: {res.fbest:.1f}")

