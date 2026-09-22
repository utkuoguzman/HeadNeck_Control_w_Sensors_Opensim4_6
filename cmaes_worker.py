import opensim as osim
import numpy as np
import math
import os
import sys

def extract_bode(t_array, pitch_array, frequency):
    valid_indices = [i for i, t in enumerate(t_array) if t > 1.5]
    t_steady = np.array([t_array[i] for i in valid_indices])
    pitch_steady = np.array([pitch_array[i] for i in valid_indices])
    
    if len(t_steady) < 10:
        return 0.0, 0.0
        
    omega = 2.0 * math.pi * frequency
    Is = 2.0 * np.mean(pitch_steady * np.sin(omega * t_steady))
    Ic = 2.0 * np.mean(pitch_steady * np.cos(omega * t_steady))
    
    amplitude_rad = np.sqrt(Is**2 + Ic**2)
    phase_rad = math.atan2(Ic, Is)
    
    return amplitude_rad, phase_rad

def run_simulation(kp, ki, kd, frequency):
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

    osim.PropertyHelper.setValueDouble(kp, controller.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(kd, controller.updPropertyByName("Kd_task"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    amplitude = 0.0215 * (0.835 / frequency)**2
    sine_function = osim.Sine(amplitude, 2 * math.pi * frequency, 0)
    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()

    v_initial = amplitude * 2 * math.pi * frequency * math.cos(0)
    coord_tx.setSpeedValue(state, v_initial)

    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.initialize(state)

    target_time = 4.0 
    step = 0.02 
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
    
    amp_deg = amp_rad * (180.0 / math.pi)
    gain_deg_m = amp_deg / amplitude
    phase_deg = phase_rad * (180.0 / math.pi)
    if phase_deg > 90: phase_deg -= 360
    
    return gain_deg_m, phase_deg

if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(1)
        
    kp = float(sys.argv[1])
    ki = float(sys.argv[2])
    kd = float(sys.argv[3])
    
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
            total_error += (sim_gain - target_gain)**2 + (sim_phase - target_phase)**2
            
        penalty = 0.0
        if kp > 500: penalty += (kp - 500) * 10
        if ki > 500: penalty += (ki - 500) * 10
        
        final_score = total_error + penalty
        print(f"{final_score}")
        sys.exit(0)
    except Exception as e:
        print("100000.0")
        sys.exit(1)

