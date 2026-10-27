import opensim as osim
import os

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
ctrl_fix = model.getControllerSet().get("Binocular_Fixation_Controller")
print("FixationController properties:")
for i in range(ctrl_fix.getNumProperties()):
    p = ctrl_fix.getPropertyByIndex(i)
    print(f"  {p.getName()}: {p.toString()}")

act_names = ctrl_fix.get_actuator_list()
print("Actuators in FixationController:", [act_names.get(i) for i in range(act_names.size())])
