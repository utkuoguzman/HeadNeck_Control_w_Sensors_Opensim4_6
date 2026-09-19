import os
import sys
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import org.opensim.modeling as modeling
import traceback
import sys

out_file = open(r"logs\simulation_progress.txt", "w")

def log(msg):
    out_file.write(msg + "\n")
    out_file.flush()

log("Testing STO File Save...")

try:
    model = getCurrentModel()
    if model == None:
        raise Exception("No model loaded in the GUI!")

    state = model.initSystem()
    manager = modeling.Manager(model)
    manager.setIntegratorAccuracy(1e-3)
    manager.initialize(state)

    log("Integrating for 0.2s...")
    manager.integrate(0.2)
    log("Integration complete.")

    log("Getting states table...")
    states_table = manager.getStatesTable()
    output_file = r"state_files\test_save.sto"
    
    log("Writing to disk at: " + output_file)
    modeling.STOFileAdapter.write(states_table, output_file)
    
    log("Success! Saved to: " + output_file)

except Exception, e:
    log("ERROR OCCURRED: " + str(e))
    # Capture the full traceback to string and write it to the log file
    tb_str = traceback.format_exc()
    log("TRACEBACK:")
    log(tb_str)

out_file.close()
