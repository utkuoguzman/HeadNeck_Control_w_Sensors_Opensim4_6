import opensim as osim
import numpy as np
import math
import os
import sys

def min_jerk(t, T, A):
    if t < 0: return 0.0, 0.0, 0.0
    if t > T: return A, 0.0, 0.0
    tau = t / T
    pos = A * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    vel = (A / T) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4)
    acc = (A / T**2) * (60 * tau - 180 * tau**2 + 120 * tau**3)
    return pos, vel, acc

def run_simulation(Wp_p, Wv_p, Wp_r, Wv_r, Wp_y, Wv_y, worker_name="W-?"):
    # Calculate Natural Frequencies
    wn_p = Wp_p**0.25
    wn_r = Wp_r**0.25
    wn_y = Wp_y**0.25
    
    # Calculate Damping Ratios (mathematically guaranteed >= 0.707)
    z_p = math.sqrt((Wv_p / (4.0 * math.sqrt(Wp_p))) + 0.5) if Wp_p > 0 else 0.707
    z_r = math.sqrt((Wv_r / (4.0 * math.sqrt(Wp_r))) + 0.5) if Wp_r > 0 else 0.707
    z_y = math.sqrt((Wv_y / (4.0 * math.sqrt(Wp_y))) + 0.5) if Wp_y > 0 else 0.707

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
        
    osim.PropertyHelper.setValueDouble(wn_p, controller.updPropertyByName("omega_n_pitch"))
    osim.PropertyHelper.setValueDouble(z_p, controller.updPropertyByName("zeta_pitch"))
    osim.PropertyHelper.setValueDouble(wn_r, controller.updPropertyByName("omega_n_roll"))
    osim.PropertyHelper.setValueDouble(z_r, controller.updPropertyByName("zeta_roll"))
    osim.PropertyHelper.setValueDouble(wn_y, controller.updPropertyByName("omega_n_yaw"))
    osim.PropertyHelper.setValueDouble(z_y, controller.updPropertyByName("zeta_yaw"))

    # Gen 4 best parameters
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
    
    # 3-axis Diagonal movement
    delay = 0.2
    duration = 0.3
    amp_p = math.radians(40.0)
    amp_r = math.radians(25.0)
    amp_y = math.radians(40.0)
    target_time = 1.0
    
    err_sq_sum = 0.0
    effort_sum = 0.0
    samples = 0
    
    muscles = model.getMuscles()
    num_muscles = muscles.getSize()
    
    while current_t < target_time - 1e-6:
        p_pos, p_vel, p_acc = min_jerk(current_t - delay, duration, amp_p)
        r_pos, r_vel, r_acc = min_jerk(current_t - delay, duration, amp_r)
        y_pos, y_vel, y_acc = min_jerk(current_t - delay, duration, amp_y)
        
        osim.PropertyHelper.setValueDouble(p_pos, controller.updPropertyByName("desired_pitch"))
        osim.PropertyHelper.setValueDouble(p_vel, controller.updPropertyByName("desired_pitch_v"))
        osim.PropertyHelper.setValueDouble(p_acc, controller.updPropertyByName("desired_pitch_a"))
        
        osim.PropertyHelper.setValueDouble(r_pos, controller.updPropertyByName("desired_roll"))
        osim.PropertyHelper.setValueDouble(r_vel, controller.updPropertyByName("desired_roll_v"))
        osim.PropertyHelper.setValueDouble(r_acc, controller.updPropertyByName("desired_roll_a"))
        
        osim.PropertyHelper.setValueDouble(y_pos, controller.updPropertyByName("desired_yaw"))
        osim.PropertyHelper.setValueDouble(y_vel, controller.updPropertyByName("desired_yaw_v"))
        osim.PropertyHelper.setValueDouble(y_acc, controller.updPropertyByName("desired_yaw_a"))

        next_t = current_t + step
        if next_t > target_time: next_t = target_time
            
        manager.integrate(next_t)
        current_t = next_t
        
        st = manager.getState()
        cset = model.getCoordinateSet()
        act_p = cset.get("pitch1").getValue(st) + cset.get("pitch2").getValue(st)
        act_r = cset.get("roll1").getValue(st) + cset.get("roll2").getValue(st)
        act_y = cset.get("yaw1").getValue(st) + cset.get("yaw2").getValue(st)
        
        err_sq_sum += (act_p - p_pos)**2 + (act_r - r_pos)**2 + (act_y - y_pos)**2
        
        # Calculate metabolic effort (sum of squared activations)
        step_effort = 0.0
        for m_idx in range(num_muscles):
            a = muscles.get(m_idx).getActivation(st)
            step_effort += a * a
        effort_sum += step_effort
            
        samples += 1
        
    rmse = math.sqrt(err_sq_sum / samples) if samples > 0 else 1e6
    avg_effort = effort_sum / samples if samples > 0 else 1e6
    
    # Cost Function: Tracking Error (rad) + C * Metabolic Effort
    # If C = 0.01, an avg activation of 0.5 across 20 muscles adds ~0.05 to the score, balancing with RMSE.
    total_cost = rmse + 0.01 * avg_effort
    return total_cost

if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) != 7:
        sys.exit(1)
        
    wname = args[0]
    vals = [float(x) for x in args[1:]]
    score = run_simulation(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], wname)
    print(score)
