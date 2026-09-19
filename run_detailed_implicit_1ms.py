
import os
import sys
import time

try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import org.opensim.modeling as modeling

log_file = os.path.join(script_dir, "logs", "log_detailed_implicit_1ms.txt")
model_file = os.path.join(script_dir, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
save_path = os.path.join(script_dir, "state_files", "detailed_implicit_1ms.sto")

def log(message):
    f = open(log_file, "a")
    f.write(message + "\n")
    f.close()
    print message

f = open(log_file, "w")
f.write("--- SIMULATION START ---\n")
f.close()

log("Loading libraries...")
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "proprioception_plugin/build/Release/osimMillard12EqWithAff.dll"))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "vestibular_plugin/build/Release/osimVestibular.dll"))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll"))
modeling.Model.LoadOpenSimLibrary(os.path.join(script_dir, "FixationController_plugin/build/Release/osimFixation.dll"))

log("Loading model " + model_file)
model = modeling.Model(model_file)
state = model.initSystem()
manager = modeling.Manager(model)

manager.setIntegratorMethod(5) # SemiExplicitEuler2
manager.setIntegratorAccuracy(1e-4)
manager.setIntegratorMinimumStepSize(1e-8)

target_time = 1.0
dt = 0.001
num_steps = int(target_time / dt) + 1

log("Preparing custom STO file writer...")
f_sto = open(save_path, "w")
f_sto.write("detailed_implicit_1ms\n")
f_sto.write("version=1\n")

names_array = model.getStateVariableNames()
num_states = names_array.getSize()
f_sto.write("nRows=" + str(num_steps) + "\n")
f_sto.write("nColumns=" + str(num_states + 1) + "\n")
f_sto.write("inDegrees=no\n")
f_sto.write("endheader\n")

f_sto.write("time\t")
for i in range(num_states):
    f_sto.write(names_array.get(i) + ("\t" if i < num_states-1 else "\n"))

manager.initialize(state)

log("Starting integration at 1ms intervals...")
start_time = time.clock()

current_t = 0.0

for step in range(num_steps):
    if current_t > 0:
        manager.integrate(current_t)
        state = manager.getState()
    
    f_sto.write(str(current_t) + "\t")
    state_vals = model.getStateVariableValues(state)
    for i in range(num_states):
        f_sto.write(str(state_vals.get(i)) + ("\t" if i < num_states-1 else "\n"))
        
    current_t += dt
    
    if step % 250 == 0 and step > 0:
        elapsed = time.clock() - start_time
        log("Reached time: " + str(current_t) + "s (Wall time elapsed: " + str(elapsed) + "s)")

f_sto.close()
end_time = time.clock()
execution_time = end_time - start_time

log("Simulation finished at " + str(execution_time) + " seconds!")
log("Results manually saved to " + save_path)

