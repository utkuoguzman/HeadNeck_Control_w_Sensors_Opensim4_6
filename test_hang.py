import sys, os
print("Starting...", flush=True)
import opensim as osim
print("Imported opensim", flush=True)
osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))
print("Libraries loaded", flush=True)
model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
print("Model loaded", flush=True)

# ADD THE CONTROLLER SETTINGS SO IT DOESNT COLLAPSE
controller = None
for i in range(model.getControllerSet().getSize()):
    c = model.getControllerSet().get(i)
    if c.getName() == "head_neck_neural_controller":
        controller = c
        break
if controller:
    osim.PropertyHelper.setValueDouble(2.0, controller.updPropertyByName("G_ton"))
    osim.PropertyHelper.setValueDouble(1.73, controller.updPropertyByName("G_sc"))
    osim.PropertyHelper.setValueDouble(0.45, controller.updPropertyByName("k_p"))
    osim.PropertyHelper.setValueDouble(0.13, controller.updPropertyByName("k_v"))
    osim.PropertyHelper.setValueDouble(50.0, controller.updPropertyByName("Kp_task"))
    osim.PropertyHelper.setValueDouble(40.0, controller.updPropertyByName("Ki_task"))
    osim.PropertyHelper.setValueDouble(5.0, controller.updPropertyByName("Kd_task"))
    
state = model.initSystem()
print("System initialized", flush=True)
model.equilibrateMuscles(state)
print("Muscles equilibrated", flush=True)
manager = osim.Manager(model)
manager.setIntegratorMethod(5)
manager.setIntegratorAccuracy(1e-4)
manager.setIntegratorMinimumStepSize(1e-8)
manager.setIntegratorMaximumStepSize(0.01)
state.setTime(0.0)
manager.initialize(state)
print("Manager initialized", flush=True)
for i in range(1, 10):
    manager.integrate(i * 0.1)
    print(f"Integrated {i*0.1}s", flush=True)
print("Done!")
