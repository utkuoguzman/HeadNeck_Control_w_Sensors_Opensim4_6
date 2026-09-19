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
reporter = modeling.TableReporterVec3()
reporter.setName("vestibular_reporter")
reporter.set_report_time_interval(0.01)
# No set_report_time_interval - it will record every integrator step just like Manager

vestibular = model.getComponent("componentset/vestibular_sensor")
reporter.updInput("inputs").connect(vestibular.getOutput("canal_firing_rate"), "canal")
reporter.updInput("inputs").connect(vestibular.getOutput("otolith_firing_rate"), "otolith")
model.addComponent(reporter)

state = model.initSystem()
manager = modeling.Manager(model)
log_file = os.path.join(script_dir, 'logs/log_vestibular_detailed_implicit.txt')
f = open(log_file, 'w')
f.write('--- SIMULATION START ---\n')
f.close()

manager.setIntegratorMethod(5) # SemiExplicitEuler2
manager.setIntegratorAccuracy(1e-4)
manager.setIntegratorMinimumStepSize(1e-8)

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

table = reporter.getTable()
modeling.STOFileAdapterVec3.write(table, os.path.join(script_dir, 'state_files/vestibular_detailed_implicit.sto'))
log("Results saved to state_files/vestibular_detailed_implicit.sto", log_file)
