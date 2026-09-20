import os
import sys
import time
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import org.opensim.modeling as modeling

def log(msg, filename):
    f = open(filename, "a")
    f.write(msg + "\n")
    f.close()
    print msg

# Load the plugin
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "proprioception_plugin/build/Release/osimMillard12EqWithAff.dll"))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "vestibular_plugin/build/Release/osimVestibular.dll"))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll"))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "FixationController_plugin/build/Release/osimFixation.dll"))

model_path = os.path.join(script_dir, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
model = modeling.Model(model_path)

# Add reporter for vestibular outputs
# No set_report_time_interval - it will record every integrator step just like Manager



state = model.initSystem()
manager = modeling.Manager(model)
log_file = os.path.join(script_dir, 'logs/log_vestibular_detailed_explicit.txt')
f = open(log_file, 'w')
f.write('--- SIMULATION START ---\n')
f.close()

manager.setIntegratorMethod(3) # RK-Merson
manager.setIntegratorAccuracy(0.00008571)
manager.setIntegratorMaximumStepSize(1.0)
manager.setIntegratorMinimumStepSize(0.00008571)
manager.setIntegratorInternalStepLimit(11667)

manager.initialize(state)

target_time = 1.0
step = 0.25
current_t = 0.0

log("Starting integration...", log_file)
start_time = time.clock()

while current_t < target_time:
    next_t = current_t + step
    if next_t > target_time:
        next_t = target_time
    
    manager.integrate(next_t)
    current_t = next_t
    
    elapsed = time.clock() - start_time
    log("Reached time: " + str(current_t) + "s (Wall time elapsed: " + str(elapsed) + "s)", log_file)

end_time = time.clock()
execution_time = end_time - start_time

log("Simulation finished at " + str(execution_time) + " seconds!", log_file)

statesTable = manager.getStatesTable()
modeling.STOFileAdapter.write(statesTable, os.path.join(script_dir, 'state_files/vestibular_detailed_explicit.sto'))
log("Results saved to state_files/vestibular_detailed_explicit.sto", log_file)
