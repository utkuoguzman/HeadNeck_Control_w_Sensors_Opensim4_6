import opensim as osim
import numpy as np
import os
import sys

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))

def evaluate_params(Kp, Ki, Kd):
    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    model.setUseVisualizer(False)

    ctrl = None
    for i in range(model.getControllerSet().getSize()):
        c = model.getControllerSet().get(i)
        if c.getConcreteClassName() == "HeadNeckNeuralController":
            ctrl = c
            
    osim.PropertyHelper.setValueDouble(Kp, ctrl.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(Ki, ctrl.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(Kd, ctrl.updPropertyByName("Kd_task"))
    osim.PropertyHelper.setValueDouble(12.15, ctrl.updPropertyByName("G_ton"))
    
    state = model.initSystem()
    model.equilibrateMuscles(state)
    
    manager = osim.Manager(model)
    manager.setIntegratorMethod(5)
    manager.setIntegratorAccuracy(1e-3)
    manager.initialize(state)
    
    cset = model.getCoordinateSet()
    dt = 0.05
    total_error = 0.0
    
    try:
        for step in range(1, int(6.0 / dt) + 1):
            t = step * dt
            dp, dr, dy = 0.0, 0.0, 0.0
            
            if t < 1.0:
                pass
            elif t < 2.0:
                dp = np.radians(45.0) # Pitch step
            elif t < 3.0:
                pass
            elif t < 4.0:
                dr = np.radians(35.0) # Roll step
            elif t < 5.0:
                pass
            elif t < 6.0:
                dy = np.radians(50.0) # Yaw step
                
            osim.PropertyHelper.setValueDouble(dp, ctrl.updPropertyByName("desired_pitch"))
            osim.PropertyHelper.setValueDouble(dr, ctrl.updPropertyByName("desired_roll"))
            osim.PropertyHelper.setValueDouble(dy, ctrl.updPropertyByName("desired_yaw"))
            osim.PropertyHelper.setValueDouble(0.0, ctrl.updPropertyByName("desired_pitch_v"))
            osim.PropertyHelper.setValueDouble(0.0, ctrl.updPropertyByName("desired_roll_v"))
            osim.PropertyHelper.setValueDouble(0.0, ctrl.updPropertyByName("desired_yaw_v"))
            
            manager.integrate(t)
            state = manager.getState()
            
            act_p = cset.get("pitch1").getValue(state) + cset.get("pitch2").getValue(state)
            act_r = cset.get("roll1").getValue(state) + cset.get("roll2").getValue(state)
            act_y = cset.get("yaw1").getValue(state) + cset.get("yaw2").getValue(state)
            
            total_error += abs(dp - act_p) + abs(dr - act_r) + abs(dy - act_y)
    except Exception as e:
        return 1e6 # Heavily penalize crashes
        
    return total_error

if __name__ == "__main__":
    if len(sys.argv) == 4:
        Kp = float(sys.argv[1])
        Ki = float(sys.argv[2])
        Kd = float(sys.argv[3])
        err = evaluate_params(Kp, Ki, Kd)
        print(err)
    else:
        print("Usage: python cmaes_step_worker.py Kp Ki Kd")

