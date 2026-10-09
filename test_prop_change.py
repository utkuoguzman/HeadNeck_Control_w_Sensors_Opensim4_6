import opensim as osim
import math
import os
import sys

def run_test():
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
    osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

    model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
    controller = None
    for i in range(model.getControllerSet().getSize()):
        ctrl = model.getControllerSet().get(i)
        if ctrl.getConcreteClassName() == "HeadNeckNeuralController":
            controller = ctrl
    
    state = model.initSystem()
    manager = osim.Manager(model)
    manager.initialize(state)
    
    manager.integrate(0.01)
    
    # Try setting property
    osim.PropertyHelper.setValueDouble(1.0, controller.updPropertyByName("desired_pitch"))
    
    try:
        manager.integrate(0.02)
        print("Success! Integrated with new property.")
    except Exception as e:
        print("Failed to integrate after setting property:", e)

if __name__ == "__main__":
    run_test()

