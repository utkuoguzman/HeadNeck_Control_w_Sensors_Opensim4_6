import opensim as osim
import numpy as np
import math
import os
import sys

def generate_mot_for_freq(freq):
    print(f"Generating MOT for {freq:.2f} Hz...")
    
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
            
    # Base LQR costs
    for ax in ['pitch', 'roll', 'yaw']:
        osim.PropertyHelper.setValueDouble(400.0, controller.updPropertyByName(f"Wp_base_{ax}"))
        osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName(f"Wv_base_{ax}"))
    osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("k_vigor"))
    osim.PropertyHelper.setValueDouble(0.1, controller.updPropertyByName("trajectory_tau"))

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
    osim.PropertyHelper.setValueDouble(1.2476, controller.updPropertyByName("Kp_proprioception"))
    osim.PropertyHelper.setValueDouble(6.54, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(23.1819, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(55.5080, controller.updPropertyByName("p_ccr"))
    
    osim.PropertyHelper.setValueDouble(3.7051, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(2.6738, controller.updPropertyByName("K_gamma_stat"))

    # Exactly lock base coordinates to ground
    zero_func = osim.Constant(0.0)
    for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
        c = model.updCoordinateSet().get(coord_name)
        c.set_locked(False)
        c.setDefaultIsPrescribed(True)
        c.setPrescribedFunction(zero_func)

    # Ensure yaw1 is dynamic (not prescribed)
    c_yaw1 = model.updCoordinateSet().get("yaw1")
    c_yaw1.set_locked(False)
    c_yaw1.setDefaultIsPrescribed(False)

    # Real Happee acceleration scaling: 2.15 cm at 0.835 Hz
    amplitude = 0.0215 * (0.835 / freq)**2
    delay = 0.5
    period = 1.0 / freq
    target_time = delay + 2.0 * period
    
    sine_function = osim.SimmSpline()
    for t in np.arange(0.0, delay, 0.05):
        sine_function.addPoint(t, 0.0)
    for t in np.arange(delay, target_time + 0.1, 0.01):
        y = amplitude * math.sin(2 * math.pi * freq * (t - delay))
        sine_function.addPoint(t, y)

    coord_tx = model.updCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(sine_function)

    state = model.initSystem()
    c_yaw1.setIsPrescribed(state, False)
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

    # Pre-load PID gravity compensation integral terms
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/pitch_error_integral", -0.01906)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/roll_error_integral", -0.01184)
    model.setStateVariableValue(state, "/controllerset/VCR_CCR_Controller/yaw_error_integral", 0.06309)

    cset = model.getCoordinateSet()
    coord_names = [cset.get(i).getName() for i in range(cset.getSize())]
    is_rot = [cset.get(i).getMotionType() != 2 for i in range(cset.getSize())]

    step = min(0.01, (1.0/freq) / 40.0)
    dt_sample = 0.01
    current_t = 0.0
    last_sample_t = 0.0
    rows = []
    pitch_degs = []

    st = manager.getState()
    vals = [cset.get(i).getValue(st) for i in range(cset.getSize())]
    rows.append([0.0] + vals)

    while current_t < target_time - 1e-6:
        next_t = min(current_t + step, target_time)
        manager.integrate(next_t)
        current_t = next_t
        
        st = manager.getState()
        p1 = cset.get("pitch1").getValue(st)
        p2 = cset.get("pitch2").getValue(st)
        pitch_degs.append(math.degrees(p1 + p2))
        
        if current_t - last_sample_t >= dt_sample - 1e-8:
            vals = [cset.get(i).getValue(st) for i in range(cset.getSize())]
            rows.append([current_t] + vals)
            last_sample_t = current_t

    os.makedirs("state_files/happee", exist_ok=True)
    out_file = f"state_files/happee/happee_motion_{freq}Hz.mot"
    with open(out_file, 'w') as f:
        f.write(f"happee_motion_{freq}Hz\n")
        f.write("version=1\n")
        f.write(f"nRows={len(rows)}\n")
        f.write(f"nColumns={len(coord_names) + 1}\n")
        f.write("inDegrees=no\n")
        f.write("endheader\n")
        f.write("\t".join(["time"] + coord_names) + "\n")
        for r in rows:
            f.write("\t".join(f"{x:.8f}" for x in r) + "\n")
            
    print(f"Saved: {out_file} ({len(rows)} rows, pitch range: [{min(pitch_degs):.1f}°, {max(pitch_degs):.1f}°])")

if __name__ == '__main__':
    frequencies = [0.78, 1.11, 2.48, 5.52, 7.95, 0.31]
    for f in frequencies:
        generate_mot_for_freq(f)
