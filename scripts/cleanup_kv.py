import os

def clean_file(filepath, target_lines):
    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()
    new_lines = [l for l in lines if not any(t in l for t in target_lines)]
    with open(filepath, "w", encoding="utf-8") as f:
        f.writelines(new_lines)
    print(f"Cleaned {filepath}: removed {len(lines) - len(new_lines)} line(s).")

clean_file("HeadNeckNeuralController_plugin/HeadNeckNeuralController.h", ["OpenSim_DECLARE_PROPERTY(k_v,"])
clean_file("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", ["constructProperty_k_v("])
clean_file("cmaes_worker_p15.py", ['controller.updPropertyByName("k_v")'])

