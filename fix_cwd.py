
import glob

header = """import os
import sys
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)
"""

files = glob.glob("run_*.py") + ["simulate_full_motion.py", "test_sto_save.py"]
for file in files:
    with open(file, "r") as f:
        content = f.read()
    if "os.chdir(script_dir)" not in content:
        with open(file, "w") as f:
            f.write(header + "\n" + content)

