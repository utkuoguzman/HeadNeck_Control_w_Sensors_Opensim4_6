
import os, glob

for file in glob.glob("run_*_*.py"):
    with open(file, "r") as f:
        content = f.read()
    
    # Check if already fixed
    if "os.path.join(script_dir" not in content:
        content = content.replace("log_file = 'logs/", "log_file = os.path.join(script_dir, 'logs', '")
        content = content.replace(".txt'", ".txt')")
        
        content = content.replace("model_file = 'osim_files/", "model_file = os.path.join(script_dir, 'osim_files', '")
        content = content.replace(".osim'", ".osim')")
        
        content = content.replace("save_path = 'state_files/", "save_path = os.path.join(script_dir, 'state_files', '")
        content = content.replace(".sto'", ".sto')")
        
        content = content.replace("LoadOpenSimLibrary('proprioception_plugin", "LoadOpenSimLibrary(os.path.join(script_dir, 'proprioception_plugin")
        content = content.replace("LoadOpenSimLibrary('vestibular_plugin", "LoadOpenSimLibrary(os.path.join(script_dir, 'vestibular_plugin")
        content = content.replace("LoadOpenSimLibrary('HeadNeckNeuralController_plugin", "LoadOpenSimLibrary(os.path.join(script_dir, 'HeadNeckNeuralController_plugin")
        content = content.replace("LoadOpenSimLibrary('FixationController_plugin", "LoadOpenSimLibrary(os.path.join(script_dir, 'FixationController_plugin")
        content = content.replace(".dll')", ".dll'))")
        
        with open(file, "w") as f:
            f.write(content)

