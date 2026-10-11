import opensim as osim
import numpy as np
import math
import os
import sys

def generate_mot_for_freq(freq):
    print(f"Generating MOT for {freq} Hz...")
    
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
    osim.PropertyHelper.setValueDouble(23.1819, controller.updPropertyByName("z_ccr"))
    osim.PropertyHelper.setValueDouble(55.5080, controller.updPropertyByName("p_ccr"))
    
    osim.PropertyHelper.setValueDouble(3.7051, controller.updPropertyByName("K_gamma_dyn"))
    osim.PropertyHelper.setValueDouble(2.6738, controller.updPropertyByName("K_gamma_stat"))

    delay = 0.5
    A = 0.1
    if freq < 0.5:
        target_time = 6.95
    elif freq < 1.0:
        target_time = 3.1
    elif freq < 2.0:
        target_time = 1.6
    elif freq < 4.0:
        target_time = 1.0
    else:
        target_time = 0.7
        
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

    cset = model.getCoordinateSet()
    coord_names = [cset.get(i).getName() for i in range(cset.getSize())]
    is_rot = [cset.get(i).getMotionType() != 2 for i in range(cset.getSize())]

    manager = osim.Manager(model)
    manager.setIntegratorMethod(5) # SemiExplicitEuler2
    manager.setIntegratorAccuracy(1e-4)
    manager.setIntegratorMinimumStepSize(1e-4)
    manager.setIntegratorMaximumStepSize(0.01)
    
    state.setTime(0.0)
    manager.initialize(state)

    dt_sample = 0.01 # 100 Hz sampling for smooth, fast OpenSim GUI playback
    current_t = 0.0
    rows = []
    
    # Initial sample at t=0
    st = manager.getState()
    vals = [math.degrees(cset.get(i).getValue(st)) if is_rot[i] else cset.get(i).getValue(st) for i in range(cset.getSize())]
    rows.append([0.0] + vals)

    step = min(0.01, (1.0/freq) / 40.0)
    last_sample_t = 0.0
    
    while current_t < target_time - 1e-6:
        next_t = min(current_t + step, target_time)
        manager.integrate(next_t)
        current_t = next_t
        
        if current_t - last_sample_t >= dt_sample - 1e-8:
            st = manager.getState()
            vals = [math.degrees(cset.get(i).getValue(st)) if is_rot[i] else cset.get(i).getValue(st) for i in range(cset.getSize())]
            rows.append([current_t] + vals)
            last_sample_t = current_t
        
    os.makedirs("state_files/happee", exist_ok=True)
    out_file = f"state_files/happee/happee_motion_{freq}Hz.mot"
    with open(out_file, 'w') as f:
        f.write("Coordinates\n")
        f.write("version=1\n")
        f.write(f"nRows={len(rows)}\n")
        f.write(f"nColumns={len(coord_names) + 1}\n")
        f.write("inDegrees=yes\n")
        f.write("endheader\n")
        f.write("\t".join(["time"] + coord_names) + "\n")
        for r in rows:
            f.write("\t".join(f"{x:.6f}" for x in r) + "\n")
            
    print(f"Saved: {out_file} ({len(rows)} rows, {os.path.getsize(out_file)/1024:.1f} KB)")

if __name__ == '__main__':
    frequencies = [7.95, 5.52, 2.48, 1.11, 0.78, 0.31]
    for f in frequencies:
        generate_mot_for_freq(f)
