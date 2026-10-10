import opensim as osim
import numpy as np
import math
import os
import sys

def generate_sto_for_freq(freq):
    print(f"Generating STO for {freq} Hz...")
    
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
            
    # Set Task properties to arbitrary holding values
    osim.PropertyHelper.setValueDouble(4.47, controller.updPropertyByName("omega_n_pitch"))
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("zeta_pitch"))
    osim.PropertyHelper.setValueDouble(4.47, controller.updPropertyByName("omega_n_roll"))
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("zeta_roll"))
    osim.PropertyHelper.setValueDouble(4.47, controller.updPropertyByName("omega_n_yaw"))
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("zeta_yaw"))

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

    delay = 0.5
    A = 0.1
    if freq < 0.5:
        target_time = 6.95
        discard_time = 0.5
    elif freq < 1.0:
        target_time = 3.1
        discard_time = 0.5
    elif freq < 2.0:
        target_time = 1.6
        discard_time = 0.7
    elif freq < 4.0:
        target_time = 1.0
        discard_time = 0.6
    else:
        target_time = 0.7
        discard_time = 0.5
        
    num_points = 500
    times = osim.Vector(num_points, 0.0)
    values = osim.Vector(num_points, 0.0)
    
    for i in range(num_points):
        t = i * (target_time / (num_points - 1))
        times.set(i, t)
        
        if t <= delay:
            values.set(i, 0.0)
        else:
            t_eff = t - delay
            envelope = 1.0
            if t_eff < 0.2:
                envelope = 0.5 - 0.5 * math.cos(math.pi * (t_eff / 0.2))
            values.set(i, A * envelope * math.sin(2.0 * math.pi * freq * t_eff))
            
    spline = osim.GCVSpline()
    spline.setDegree(3)
    
    for i in range(num_points):
        spline.addPoint(times.get(i), values.get(i))

    coord_tx = model.getCoordinateSet().get("spine_tx")
    coord_tx.set_locked(False)
    coord_tx.setDefaultIsPrescribed(True)
    coord_tx.setPrescribedFunction(spline)

    state = model.initSystem()
    coord_tx.setSpeedValue(state, 0.0)
    model.equilibrateMuscles(state)

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5)
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

    step = min(0.01, (1.0/freq) / 40.0)
    current_t = 0.0
    
    while current_t < target_time - 1e-6:
        next_t = current_t + step
        if next_t > target_time: next_t = target_time
        manager.integrate(next_t)
        current_t = next_t
        
    states_table = manager.getStatesTable()
    os.makedirs("state_files/happee", exist_ok=True)
    out_file = f"state_files/happee/happee_test_{freq}Hz.sto"
    osim.STOFileAdapter.write(states_table, out_file)
    print(f"Saved: {out_file}")

if __name__ == '__main__':
    frequencies = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]
    for f in frequencies:
        generate_sto_for_freq(f)
