import time
import os
import opensim as osim

print("Starting CPodes test...", flush=True)
osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'vestibular_plugin/build/Release/osimVestibular.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")

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

zero_func = osim.Constant(0.0)
for coord_name in ["gndpitch", "gndroll", "gndyaw", "spine_ty", "spine_tz"]:
    c = model.updCoordinateSet().get(coord_name)
    c.set_locked(False)
    c.setDefaultIsPrescribed(True)
    c.setPrescribedFunction(zero_func)

coord_tx = model.updCoordinateSet().get("spine_tx")
coord_tx.set_locked(False)
coord_tx.setDefaultIsPrescribed(True)
coord_tx.setPrescribedFunction(zero_func)

state = model.initSystem()
model.equilibrateMuscles(state)

manager = osim.Manager(model)
manager.setIntegratorMethod(1)
manager.setIntegratorAccuracy(1e-4)

state.setTime(0.0)
manager.initialize(state)

start = time.time()
manager.integrate(1.0)
end = time.time()
print(f"Time for 1.0s with CPodes: {end - start:.2f} seconds", flush=True)

