import opensim as osim
import numpy as np
import math
import os
import sys

# ==========================================
# TARGET KINEMATICS (From Peng/Zheng Data)
# To be filled by the user tomorrow
# ==========================================
TARGET_RISE_TIME_S = 0.15      # Example value
TARGET_OVERSHOOT_DEG = 2.0     # Example value
TARGET_SETTLING_TIME_S = 0.40  # Example value
# ==========================================


def run_simulation(Wp_base, Wv_base, k_vigor, tau_traj, worker_name="W-?"):
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
        return 1e6
        
    osim.PropertyHelper.setValueDouble(Wp_base, controller.updPropertyByName("Wp_base_pitch"))
    osim.PropertyHelper.setValueDouble(Wv_base, controller.updPropertyByName("Wv_base_pitch"))
    osim.PropertyHelper.setValueDouble(Wp_base, controller.updPropertyByName("Wp_base_roll"))
    osim.PropertyHelper.setValueDouble(Wv_base, controller.updPropertyByName("Wv_base_roll"))
    osim.PropertyHelper.setValueDouble(Wp_base, controller.updPropertyByName("Wp_base_yaw"))
    osim.PropertyHelper.setValueDouble(Wv_base, controller.updPropertyByName("Wv_base_yaw"))
    osim.PropertyHelper.setValueDouble(k_vigor, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(tau_traj, controller.updPropertyByName("trajectory_tau"))
    
    delay = 0.2

    # Gen 4 reflex parameters (frozen)
    osim.PropertyHelper.setValueDouble(15.4172, controller.updPropertyByName("K_ton"))
    osim.PropertyHelper.setValueDouble(34.0019, controller.updPropertyByName("z_ton"))
    osim.PropertyHelper.setValueDouble(3.6907, controller.updPropertyByName("p_ton"))
    
    osim.PropertyHelper.setValueDouble(15.7436, controller.updPropertyByName("K_sc"))
    osim.PropertyHelper.setValueDouble(9.5737, controller.updPropertyByName("z_sc"))
    osim.PropertyHelper.setValueDouble(58.4690, controller.updPropertyByName("p_sc"))
    
    osim.PropertyHelper.setValueDouble(1.1818, controller.updPropertyByName("K_phas"))
    osim.PropertyHelper.setValueDouble(12.9370, controller.updPropertyByName("z_phas"))
    osim.PropertyHelper.setValueDouble(9.0202, controller.updPropertyByName("p_phas"))
    
    osim.PropertyHelper.setValueDouble(1.2476, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(23.1819, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(55.5080, controller.updPropertyByName("p_ccr"))
    
    osim.PropertyHelper.setValueDouble(3.7051, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(2.6738, controller.updPropertyByName("K_gamma_stat"))

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

    step = 0.01
    current_t = 0.0
    
    # Simulating a 40-degree pitch step (typical Peng/Zheng experiment)
    delay = 0.2
    target_time = 1.0
    amp_p = math.radians(40.0)
    
    act_history = []
    t_history = []
    
    while current_t < target_time - 1e-6:
        # Send a single step command at t = delay
        p_pos = amp_p if current_t >= delay else 0.0
        osim.PropertyHelper.setValueDouble(p_pos, controller.updPropertyByName("desired_pitch"))

        next_t = current_t + step
        if next_t > target_time: next_t = target_time
            
        manager.integrate(next_t)
        current_t = next_t
        
        st = manager.getState()
        cset = model.getCoordinateSet()
        act_p = cset.get("pitch1").getValue(st) + cset.get("pitch2").getValue(st)
        
        act_history.append(math.degrees(act_p))
        t_history.append(current_t)
        
    # Analyze Kinematics
    peak_val = max(act_history)
    steady_state = act_history[-1]
    
    # Calculate Overshoot
    overshoot = peak_val - steady_state
    if overshoot < 0: overshoot = 0
    
    # Calculate Rise Time (10% to 90%)
    t_10 = None
    t_90 = None
    for i, a in enumerate(act_history):
        if t_10 is None and a >= 0.1 * steady_state:
            t_10 = t_history[i]
        if t_90 is None and a >= 0.9 * steady_state:
            t_90 = t_history[i]
            break
            
    rise_time = (t_90 - t_10) if (t_90 is not None and t_10 is not None) else 1e6
    
    # Calculate Settling Time (time to reach and stay within 2% of steady state)
    settling_time = 1e6
    for i in range(len(act_history)-1, -1, -1):
        if abs(act_history[i] - steady_state) > 0.02 * steady_state:
            settling_time = t_history[i] - delay
            break

    # We drop C entirely. The cost is purely matching human kinematics.
    cost = (rise_time - TARGET_RISE_TIME_S)**2 + \
           (overshoot - TARGET_OVERSHOOT_DEG)**2 + \
           (settling_time - TARGET_SETTLING_TIME_S)**2
           
    return cost

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) != 5:
        sys.exit(1)
        
    wname = args[0]
    vals = [float(x) for x in args[1:]]
    score = run_simulation(vals[0], vals[1], vals[2], vals[3], wname)
    print(score)
