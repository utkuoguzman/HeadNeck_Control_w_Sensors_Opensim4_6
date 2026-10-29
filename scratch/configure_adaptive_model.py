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

controller = None
for i in range(model.getControllerSet().getSize()):
    c = model.getControllerSet().get(i)
    if c.getConcreteClassName() == "HeadNeckNeuralControllerAdaptive":
        controller = c
        break

assert controller is not None, "HeadNeckNeuralControllerAdaptive not found!"

# Set center-frequency-normalized baseline values (matching data/prescreen_rational_champion.json)
osim.PropertyHelper.setValueDouble(0.5800,   controller.updPropertyByName("K_ton"))
osim.PropertyHelper.setValueDouble(78.0001,  controller.updPropertyByName("p_ton"))
osim.PropertyHelper.setValueDouble(0.0974,   controller.updPropertyByName("K_sc"))
osim.PropertyHelper.setValueDouble(21.5985,  controller.updPropertyByName("z_sc"))
osim.PropertyHelper.setValueDouble(90.0050,  controller.updPropertyByName("p_sc"))
osim.PropertyHelper.setValueDouble(4.5320,   controller.updPropertyByName("K_phas"))
osim.PropertyHelper.setValueDouble(15.0046,  controller.updPropertyByName("p_phas"))
osim.PropertyHelper.setValueDouble(0.3082,   controller.updPropertyByName("k_p"))
osim.PropertyHelper.setValueDouble(0.3082,   controller.updPropertyByName("Kp_proprioception"))
osim.PropertyHelper.setValueDouble(8.4129,   controller.updPropertyByName("z_ccr"))
osim.PropertyHelper.setValueDouble(170.6536, controller.updPropertyByName("p_ccr"))
osim.PropertyHelper.setValueDouble(0.7339,   controller.updPropertyByName("K_gamma_dyn"))
osim.PropertyHelper.setValueDouble(1.1482,   controller.updPropertyByName("K_gamma_stat"))

osim.PropertyHelper.setValueDouble(40.2786, controller.updPropertyByName("Ki_task"))
osim.PropertyHelper.setValueDouble(40.2786, controller.updPropertyByName("Ki_task_pitch"))
osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_roll"))
osim.PropertyHelper.setValueDouble(0.0, controller.updPropertyByName("Ki_task_yaw"))

# Zero voluntary pitch PD clamping for relaxed perturbation task
osim.PropertyHelper.setValueDouble(0.0,   controller.updPropertyByName("Kp_task_pitch"))
osim.PropertyHelper.setValueDouble(0.0,   controller.updPropertyByName("Kd_task_pitch"))
osim.PropertyHelper.setValueDouble(10.21, controller.updPropertyByName("Kp_task_roll"))
osim.PropertyHelper.setValueDouble(1.29,  controller.updPropertyByName("Kd_task_roll"))
osim.PropertyHelper.setValueDouble(0.24,  controller.updPropertyByName("Kp_task_yaw"))
osim.PropertyHelper.setValueDouble(0.12,  controller.updPropertyByName("Kd_task_yaw"))
osim.PropertyHelper.setValueDouble(0.0,   controller.updPropertyByName("k_vigor"))

osim.PropertyHelper.setValueBool(True,    controller.updPropertyByName("use_smith_predictor"))
osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_smith_proprio"))
osim.PropertyHelper.setValueDouble(0.007, controller.updPropertyByName("delay_afferent_proprio"))
osim.PropertyHelper.setValueDouble(0.002, controller.updPropertyByName("delay_afferent_vestibular"))
osim.PropertyHelper.setValueDouble(0.010, controller.updPropertyByName("delay_vcr"))
osim.PropertyHelper.setValueDouble(0.015, controller.updPropertyByName("delay_ccr"))

# Adaptive properties (Option 1: Harmonic Bode-Sensitivity Adaptive Control)
osim.PropertyHelper.setValueBool(True,       controller.updPropertyByName("enable_adaptation"))
osim.PropertyHelper.setValueDouble(1.0,      controller.updPropertyByName("adaptation_rate_multiplier"))
osim.PropertyHelper.setValueDouble(0.01,     controller.updPropertyByName("sigma_leakage"))
osim.PropertyHelper.setValueDouble(2.5,      controller.updPropertyByName("adaptation_start_time"))
osim.PropertyHelper.setValueDouble(0.142887, controller.updPropertyByName("multisine_scale"))

model.printToXML(model_path)
print(f"Updated {model_path} with upgraded center-frequency-normalized HBAC parameters.")
