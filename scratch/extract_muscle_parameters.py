import os
import opensim as osim
import json

osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'proprioception_plugin/build/Release/osimMillard12EqWithAff.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll'))
osim.LoadOpenSimLibrary(os.path.join(os.getcwd(), 'FixationController_plugin/build/Release/osimFixation.dll'))

model = osim.Model("osim_files/HYOID_HeadNeckNeural_Model_Detailed.osim")
muscles = model.getMuscles()
print(f"Total muscles in model: {muscles.getSize()}")

muscle_dict = {}
for i in range(muscles.getSize()):
    m = muscles.get(i)
    name = m.getName()
    muscle_dict[name] = {
        "F0": m.getMaxIsometricForce(),
        "l0m": m.getOptimalFiberLength(),
        "lst": m.getTendonSlackLength(),
        "alpha0": m.getPennationAngleAtOptimalFiberLength(),
        "vmax": 10.0 * m.getOptimalFiberLength()
    }

output_path = "logs/cervical_72_muscle_parameters.json"
with open(output_path, "w", encoding="utf-8") as f:
    json.dump(muscle_dict, f, indent=2)

print(f"Successfully extracted {len(muscle_dict)} muscles to {output_path}!")
for i in range(min(5, len(muscle_dict))):
    k = list(muscle_dict.keys())[i]
    print(f"  {k}: F0={muscle_dict[k]['F0']} N, l0m={muscle_dict[k]['l0m']*100:.1f} cm, lst={muscle_dict[k]['lst']*100:.1f} cm")

