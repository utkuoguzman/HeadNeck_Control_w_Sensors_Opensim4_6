import os, sys, math
import opensim as osim
import numpy as np
from scipy.optimize import minimize_scalar

FIXED_KI = 34.3459
FIXED_I_ROLL = -0.00477
FIXED_I_YAW = 0.00253

def evaluate_I0(i_pitch_candidate, verbose=False):
    i_pitch = float(i_pitch_candidate)
    
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

    # Baseline reflex parameters (Best from Gen 4, Score: 2438.42)
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

    # Set Fixed Optimal Ki
    osim.PropertyHelper.setValueDouble(FIXED_KI, controller.updPropertyByName("Ki_task"))

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

    # Preload Candidate Integrals
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", i_pitch)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", FIXED_I_ROLL)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", FIXED_I_YAW)

    # Initialize manager AFTER setting all state variables so SimTK integrator captures them!
    state.setTime(0.0)
    manager.initialize(state)

    current_t = 0.0
    final_t = 2.0 # 2.0s is plenty to evaluate initial transient droop and settling
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
            return 100000.0, 0.0, 0.0

    t_arr = np.array(t_hist)
    p_arr = np.array(pitch_hist)
    dp_arr = np.array(dpitch_hist)

    # Metrics focused on eliminating initial transient and settling to 0 deg:
    max_transient_droop = np.max(np.abs(p_arr))
    final_pitch_error = abs(p_arr[-1])
    overall_std = np.std(p_arr)
    
    # Cost: penalize any deviation from 0 degrees across the entire 2s duration
    cost = 100.0 * final_pitch_error + 80.0 * max_transient_droop + 150.0 * overall_std
    
    final_int_p = model.getStateVariableValue(st, "/controllerset/VCR_CCR_Controller/pitch_error_integral")
    
    if verbose:
        print(f"I_pitch = {i_pitch:8.5f} | Cost: {cost:7.2f} | Max Droop: {max_transient_droop:5.2f} deg | Final P: {p_arr[-1]:5.2f} deg | Std: {overall_std:5.3f}", flush=True)
        
    return cost, p_arr[-1], final_int_p

def main():
    print("=" * 70, flush=True)
    print("STATIC OPTIMIZATION: FINE TUNING PRELOADED INTEGRAL I_pitch*")
    print(f"Fixed Ki* = {FIXED_KI:.4f} | Roll Preload = {FIXED_I_ROLL:.5f} | Yaw Preload = {FIXED_I_YAW:.5f}", flush=True)
    print("=" * 70, flush=True)
    
    # Coarse scan testing 0.0 vs preloaded values
    test_i = [0.0, 0.04, 0.07287, 0.10]
    print("\n--- Coarse Grid Evaluation ---", flush=True)
    results = []
    for ip in test_i:
        c, p_fin, ip_fin = evaluate_I0(ip, verbose=True)
        results.append((ip, c, p_fin, ip_fin))
        
    results.sort(key=lambda x: x[1])
    best_grid_ip = results[0][0]
    print(f"\nBest coarse I_pitch: {best_grid_ip:.5f} (Cost: {results[0][1]:.2f})", flush=True)
    
    # Fine bounded search around best grid point
    b_low = max(0.02, best_grid_ip - 0.015)
    b_high = min(0.15, best_grid_ip + 0.015)
    print(f"\n--- Fine Brent Optimization in [{b_low:.5f}, {b_high:.5f}] ---", flush=True)
    
    res = minimize_scalar(lambda ip: evaluate_I0(ip)[0], bounds=(b_low, b_high), method='bounded', options={'xatol': 0.001, 'maxiter': 8})
    
    opt_ip = res.x
    opt_cost, opt_p, _ = evaluate_I0(opt_ip, verbose=False)
    
    print("\n" + "=" * 70, flush=True)
    print("FINAL OPTIMIZED PRELOAD RESULTS:", flush=True)
    print("=" * 70, flush=True)
    print(f"Fixed Ki_task:             {FIXED_KI:.4f}", flush=True)
    print(f"Optimal I_pitch preload:   {opt_ip:.5f}", flush=True)
    print(f"Minimum Transient Cost:    {opt_cost:.2f}", flush=True)
    print(f"Final Pitch Angle:         {opt_p:.3f} deg", flush=True)
    print(f"I_roll preload:            {FIXED_I_ROLL:.5f}", flush=True)
    print(f"I_yaw preload:             {FIXED_I_YAW:.5f}", flush=True)
    print("=" * 70, flush=True)
    
    with open("logs/optimal_preload_integral.txt", "w") as f:
        f.write(f"Fixed Ki_task: {FIXED_KI:.4f}\n")
        f.write(f"Optimal I_pitch: {opt_ip:.5f}\n")
        f.write(f"I_roll: {FIXED_I_ROLL:.5f}\n")
        f.write(f"I_yaw: {FIXED_I_YAW:.5f}\n")
        f.write(f"Final Pitch: {opt_p:.3f} deg\n")
        f.write(f"Cost: {opt_cost:.2f}\n")
    print("Saved to logs/optimal_preload_integral.txt", flush=True)

if __name__ == "__main__":
    main()

