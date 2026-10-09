import opensim as osim
import numpy as np
import math
import os
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def min_jerk(t, T, A):
    if t < 0:
        return 0.0, 0.0, 0.0
    if t > T:
        return A, 0.0, 0.0
        
    tau = t / T
    pos = A * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    vel = (A / T) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    acc = (A / T**2) * (60 * tau - 180 * tau**2 + 120 * tau**3)
    return pos, vel, acc

def run_peng_simulation():
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

    # Set task parameters to defaults (will be optimized in CMA-ES)
    osim.PropertyHelper.setValueDouble(4.47, controller.updPropertyByName("omega_n_pitch"))
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("zeta_pitch"))
    osim.PropertyHelper.setValueDouble(4.47, controller.updPropertyByName("omega_n_roll"))
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("zeta_roll"))
    osim.PropertyHelper.setValueDouble(4.47, controller.updPropertyByName("omega_n_yaw"))
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("zeta_yaw"))

    # Set 14 Reflex parameters to the exact best values found in Phase 0 (Gen 4)
    osim.PropertyHelper.setValueDouble(15.4172, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(34.0019, controller.updPropertyByName("z_ton"))
    osim.PropertyHelper.setValueDouble(3.6907, controller.updPropertyByName("p_ton"))
    
    osim.PropertyHelper.setValueDouble(15.7436, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(9.5737, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(58.4690, controller.updPropertyByName("p_sc"))
    
    osim.PropertyHelper.setValueDouble(1.1765, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(12.9448, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(9.0170, controller.updPropertyByName("p_phas"))
    
    osim.PropertyHelper.setValueDouble(1.2452, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(23.1813, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(55.5098, controller.updPropertyByName("p_ccr"))
    
    osim.PropertyHelper.setValueDouble(3.7080, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(2.6681, controller.updPropertyByName("K_gamma_stat"))

    # Initial desired states
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("desired_pitch"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("desired_pitch_v"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("desired_pitch_a"))

    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz", "spine_tx"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    state = model.initSystem()
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)
    
    state.setTime(0.0)
    manager.initialize(state)
        
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    step = 0.005
    current_t = 0.0
    
    # Peng 1999 voluntary movement: 40 deg (0.698 rad) over 0.3s
    delay = 0.5
    movement_duration = 0.3
    amplitude_deg = 40.0 # flexion
    amplitude_rad = math.radians(amplitude_deg)
    target_time = 1.2
    
    t_history = []
    pitch_history = []
    target_history = []
    
    splenius_exc = []
    scm_exc = []

    print("Running Peng voluntary step tracking simulation...")
    
    muscle_splen = model.getMuscles().get("splen_cap_sklthx")
    muscle_scm = model.getMuscles().get("stern_mast")
    
    while current_t < target_time - 1e-6:
        # Get target trajectory at current time
        pos, vel, acc = min_jerk(current_t - delay, movement_duration, amplitude_rad)
        
        # Inject target into the C++ controller via properties!
        osim.PropertyHelper.setValueDouble(pos, controller.updPropertyByName("desired_pitch"))
        osim.PropertyHelper.setValueDouble(vel, controller.updPropertyByName("desired_pitch_v"))
        osim.PropertyHelper.setValueDouble(acc, controller.updPropertyByName("desired_pitch_a"))

        next_t = current_t + step
        if next_t > target_time:
            next_t = target_time
            
        manager.integrate(next_t)
        current_t = next_t
        
        st = manager.getState()
        p1 = model.getCoordinateSet().get("pitch1").getValue(st)
        p2 = model.getCoordinateSet().get("pitch2").getValue(st)
        
        t_history.append(current_t)
        pitch_history.append(math.degrees(p1 + p2))
        target_history.append(math.degrees(pos))
        
        # Excitation / Activation
        # Controller computes excitation, muscle state has activation. Let's record activation.
        splenius_exc.append(muscle_splen.getActivation(st))
        scm_exc.append(muscle_scm.getActivation(st))
        
    print("Simulation complete! Generating plots...")
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    ax1.plot(t_history, target_history, 'k--', label='Target (Min Jerk)')
    ax1.plot(t_history, pitch_history, 'b-', label='Head Pitch')
    ax1.set_ylabel('Pitch (deg)')
    ax1.set_title('Peng 1999 Voluntary Step Tracking (40°)')
    ax1.legend()
    ax1.grid(True)
    
    ax2.plot(t_history, scm_exc, 'r-', label='Agonist (SCM) - Flexor')
    ax2.plot(t_history, splenius_exc, 'g-', label='Antagonist (Splenius) - Extensor')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Activation (0-1)')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('peng_voluntary_tracking.png')
    print("Saved plot to peng_voluntary_tracking.png")

if __name__ == "__main__":
    run_peng_simulation()
