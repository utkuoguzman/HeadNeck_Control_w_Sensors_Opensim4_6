import os, sys, math
import opensim as osim
import numpy as np
from scipy.optimize import minimize_scalar

def evaluate_Ki(ki_candidate, verbose=False):
    ki = float(ki_candidate)
    
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

    # Baseline task costs
    for ax in ['pitch', 'roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("K_eso"))

    # Baseline reflex parameters
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
    osim.PropertyHelper.setValueDouble(1.2452, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(23.1813, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(55.5098, controller.updPropertyByName("p_ccr"))
    osim.PropertyHelper.setValueDouble(3.7080, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(2.6681, controller.updPropertyByName("K_gamma_stat"))

    # Set Candidate Ki
    osim.PropertyHelper.setValueDouble(ki, controller.updPropertyByName("Ki_task"))

    # Stationary trunk/sled
    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz", "spine_tx"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)
    
    # Observer initialization
    skull = model.getBodySet().get("skull")
    R_head_to_world = skull.getTransformInGround(state).R()
    q = osim.Rotation(R_head_to_world).convertRotationToQuaternion()
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q0", q.get(0))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q1", q.get(1))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q2", q.get(2))
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/obs_q3", q.get(3))

    # Anti-gravity Pitch Support Torque: tau_preload_pitch = 3.5743 N*m
    # Pure symmetric posture: Roll and Yaw gravitational moments are strictly zero!
    tau_preload_pitch = 3.5743
    init_int_pitch = tau_preload_pitch / ki
    init_int_roll  = 0.0
    init_int_yaw   = 0.0

    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", init_int_pitch)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", init_int_roll)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", init_int_yaw)

    # Initialize manager AFTER setting all state variables so SimTK captures them!
    state.setTime(0.0)
    manager.initialize(state)

    current_t = 0.0
    final_t = 3.0
    step = 0.02
    
    t_hist = []
    pitch_hist = []
    dpitch_hist = []
    
    while current_t < final_t - 1e-6:
        next_t = min(current_t + step, final_t)
        manager.integrate(next_t)
        current_t = next_t
        
        st = manager.getState()
        p1 = model.getCoordinateSet().get("pitch1").getValue(st)
        p2 = model.getCoordinateSet().get("pitch2").getValue(st)
        dp1 = model.getCoordinateSet().get("pitch1").getSpeedValue(st)
        dp2 = model.getCoordinateSet().get("pitch2").getSpeedValue(st)
        
        pitch_deg = math.degrees(p1 + p2)
        dpitch_deg = math.degrees(dp1 + dp2)
        
        t_hist.append(current_t)
        pitch_hist.append(pitch_deg)
        dpitch_hist.append(dpitch_deg)
        
        if abs(pitch_deg) > 30.0:
            # Diverged / collapsed
            return 100000.0

    t_arr = np.array(t_hist)
    p_arr = np.array(pitch_hist)
    dp_arr = np.array(dpitch_hist)

    # Metrics:
    final_pitch_error = abs(p_arr[-1] - 0.0)
    final_velocity = abs(dp_arr[-1])
    
    # Settle quality (between 1.5s and 3.0s)
    late_mask = t_arr >= 1.5
    late_p_err = np.mean(np.abs(p_arr[late_mask]))
    late_oscillation = np.std(p_arr[late_mask])
    
    # Cost function:
    # We want final pitch close to 0 deg, zero oscillation, and low settling error
    cost = 100.0 * final_pitch_error + 50.0 * late_p_err + 200.0 * late_oscillation + 20.0 * final_velocity
    
    final_int_p = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/pitch_error_integral")
    final_int_r = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/roll_error_integral")
    final_int_y = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/yaw_error_integral")
    
    if verbose:
        print(f"Ki = {ki:6.2f} | Cost: {cost:8.2f} | Final Pitch: {p_arr[-1]:6.2f} deg | Late Std: {late_oscillation:6.3f} | I_final: {final_int_p:.5f}", flush=True)
        
    return cost, p_arr[-1], final_int_p, final_int_r, final_int_y

def main():
    print("=" * 70)
    print("OPTION 2: DEDICATED STATIC POSTURAL OPTIMIZATION FOR Ki")
    print("=" * 70)
    print("Goal: Find optimal Ki that eliminates gravity droop with zero oscillation")
    print("Using Invariant Torque Preload Formula: I_0(Ki) = tau_preload / Ki\n")
    
    # 1. Grid scan over Ki to inspect the landscape (25 to 45)
    test_kis = [25.0, 30.0, 35.0, 40.0, 45.0]
    print("--- Coarse Grid Evaluation (25.0 to 45.0) ---", flush=True)
    results = []
    for ki in test_kis:
        c, p_final, ip, ir, iy = evaluate_Ki(ki, verbose=True)
        results.append((ki, c, p_final, ip, ir, iy))
        
    # Sort by cost
    results.sort(key=lambda x: x[1])
    best_grid_ki = results[0][0]
    print(f"\nBest coarse Ki: {best_grid_ki:.2f} (Cost: {results[0][1]:.2f})", flush=True)
    
    # 2. Local bounded optimization in [25.0, 45.0]
    print("\n--- Fine Bounded Optimization in [25.0, 45.0] (Brent's Method) ---", flush=True)
    bounds = (25.0, 45.0)
    
    res = minimize_scalar(lambda k: evaluate_Ki(k)[0], bounds=bounds, method='bounded', options={'xatol': 0.1, 'maxiter': 12})
    
    opt_ki = res.x
    opt_cost, opt_pitch, opt_ip, opt_ir, opt_iy = evaluate_Ki(opt_ki, verbose=False)
    
    print("\n" + "=" * 70)
    print("OPTIMIZATION RESULTS:")
    print("=" * 70)
    print(f"Optimal Ki_task:              {opt_ki:.4f}")
    print(f"Minimum Cost:                 {opt_cost:.2f}")
    print(f"Steady-State Pitch Error:     {opt_pitch:.3f} deg")
    print(f"Preload Integral (Pitch):     {opt_ip:.5f}")
    print(f"Preload Integral (Roll):      {opt_ir:.5f}")
    print(f"Preload Integral (Yaw):       {opt_iy:.5f}")
    print("=" * 70)
    
    # Save to file
    os.makedirs("logs", exist_ok=True)
    with open("logs/optimal_ki_static.txt", "w") as f:
        f.write(f"Optimal Ki_task: {opt_ki:.4f}\n")
        f.write(f"Steady-State Pitch Error: {opt_pitch:.3f} deg\n")
        f.write(f"Preload Integral Pitch: {opt_ip:.5f}\n")
        f.write(f"Preload Integral Roll: {opt_ir:.5f}\n")
        f.write(f"Preload Integral Yaw: {opt_iy:.5f}\n")
    print("Saved to logs/optimal_ki_static.txt")

if __name__ == "__main__":
    main()

