import os
import sys
import opensim as osim

fix_dll = os.path.abspath("FixationController_plugin/build/Release/osimFixation.dll")
vest_dll = os.path.abspath("vestibular_plugin/build/Release/osimVestibular.dll")
prop_dll = os.path.abspath("proprioception_plugin/build/Release/osimMillard12EqWithAff.dll")
dll_path = os.path.abspath("HeadNeckNeuralControllerAdaptive_plugin/build/Release/osimHeadNeckNeuralAdaptive.dll")

print("Loading plugin DLLs...")
osim.LoadOpenSimLibraryExact(fix_dll)
osim.LoadOpenSimLibraryExact(vest_dll)
osim.LoadOpenSimLibraryExact(prop_dll)
osim.LoadOpenSimLibraryExact(dll_path)

model_path = "osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim"
print(f"Loading model: {model_path}...")
model = osim.Model(model_path)
state = model.initSystem()

cset = model.getControllerSet()
c = cset.get("VCR_CCR_Controller")
print(f"Controller Concrete Class: {c.getConcreteClassName()}")

# Check state variables
names = [
    "/controllerset/VCR_CCR_Controller/adapt_K_sc",
    "/controllerset/VCR_CCR_Controller/adapt_K_ton",
    "/controllerset/VCR_CCR_Controller/adapt_K_phas",
    "/controllerset/VCR_CCR_Controller/adapt_kp_prop",
    "/controllerset/VCR_CCR_Controller/psif_x1_0",
    "/controllerset/VCR_CCR_Controller/psif_x2_0"
]

for name in names:
    val = model.getStateVariableValue(state, name)
    print(f"  {name} = {val}")

print("\nSUCCESS: All state variables properly instantiated and loaded!")

