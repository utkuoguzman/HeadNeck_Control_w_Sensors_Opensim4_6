import os
import opensim as osim

fix_dll = os.path.abspath("FixationController_plugin/build/Release/osimFixation.dll")
vest_dll = os.path.abspath("vestibular_plugin/build/Release/osimVestibular.dll")
prop_dll = os.path.abspath("proprioception_plugin/build/Release/osimMillard12EqWithAff.dll")
dll_path = os.path.abspath("HeadNeckNeuralControllerAdaptive_plugin/build/Release/osimHeadNeckNeuralAdaptive.dll")

osim.LoadOpenSimLibraryExact(fix_dll)
osim.LoadOpenSimLibraryExact(vest_dll)
osim.LoadOpenSimLibraryExact(prop_dll)
osim.LoadOpenSimLibraryExact(dll_path)

model_path = "osim_files/HYOID_HeadNeckNeural_Model_Adaptive.osim"
model = osim.Model(model_path)
state = model.initSystem()

c = model.getControllerSet().get("VCR_CCR_Controller")
print("Controller Class:", c.getConcreteClassName())

ADAPT_12_NAMES = [
    "adapt_K_ton", "adapt_p_ton", "adapt_K_sc", "adapt_z_sc", "adapt_p_sc",
    "adapt_K_phas", "adapt_p_phas", "adapt_kp_prop", "adapt_z_ccr", "adapt_p_ccr",
    "adapt_K_gamma_dyn", "adapt_K_gamma_stat"
]

print("\n--- Verifying all 12 continuous state variables ---")
for name in ADAPT_12_NAMES:
    path = f"/controllerset/VCR_CCR_Controller/{name}"
    val = model.getStateVariableValue(state, path)
    print(f"  {name:20s} = {val:8.4f}")

print("\nSUCCESS: All 12 continuous state variables verified!")

