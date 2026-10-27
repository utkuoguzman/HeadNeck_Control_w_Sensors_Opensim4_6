import opensim as osim
import os

osim.Logger.setLevelString("Warn")
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
ctrl = model.getControllerSet().get("VCR_CCR_Controller")
print("VCR_CCR_Controller socket_actuators count:")
p = ctrl.getPropertyByName("socket_actuators")
print(f"Num actuators: {p.size()}")
act_list = [p.getValueString(i) for i in range(p.size())]
print(f"First 5: {act_list[:5]}")
print(f"Last 5: {act_list[-5:]}")
# Check if any eye muscles or non-neck actuators are included
eye_muscles = [a for a in act_list if 'r_' in a or 'l_' in a]
print("Are any eye muscles in socket_actuators?", eye_muscles)

