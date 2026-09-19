import os
import sys
import time
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except:
    script_dir = r"D:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6"
os.chdir(script_dir)

import org.opensim.modeling as osim

def log(msg, filename):
    with open(filename, "a") as f:
        f.write(msg + "\n")
    print msg

# Load the plugin
osim.Model.LoadOpenSimLibrary(os.path.join(script_dir, "proprioception_plugin/build/Release/osimMillard12EqWithAff.dll"))
osim.Model.LoadOpenSimLibrary(os.path.join(script_dir, "vestibular_plugin/build/Release/osimVestibular.dll"))
osim.Model.LoadOpenSimLibrary(os.path.join(script_dir, "HeadNeckNeuralController_plugin/build/Release/osimHeadNeckNeural.dll"))
osim.Model.LoadOpenSimLibrary(os.path.join(script_dir, "FixationController_plugin/build/Release/osimFixation.dll"))

# Load the model
model_path = os.path.join(script_dir, "osim_files", "HYOID_HeadNeckNeural_Model_Detailed.osim")
model = osim.Model(model_path)

# Add reporter
reporter = osim.TableReporterVec3()
reporter.setName("vestibular_reporter")
reporter.set_report_time_interval(0.001)

# Get sensor
vestibular = model.getComponent("vestibular_sensor")
reporter.updInput("inputs").connect(vestibular.getOutput("canal_firing_rate"), "canal")
reporter.updInput("inputs").connect(vestibular.getOutput("otolith_firing_rate"), "otolith")
model.addComponent(reporter)

state = model.initSystem()
manager = osim.Manager(model)

manager.setIntegratorMethod(3) # RK-Merson
manager.setIntegratorAccuracy(0.00008571)
manager.setIntegratorMaximumStepSize(1.0)
manager.setIntegratorMinimumStepSize(0.00008571)
manager.setIntegratorInternalStepLimit(11667)

manager.initialize(state)
log("Integrating Detailed Vestibular Explicit for 1.0s...", "logs/log_vestibular_explicit_1ms.txt")
start = time.clock()
manager.integrate(1.0)
end = time.clock()

log("Simulation finished at " + str(end - start) + " seconds!", "logs/log_vestibular_explicit_1ms.txt")

table = reporter.getTable()
osim.STOFileAdapterVec3.write(table, "state_files/vestibular_detailed_explicit_1ms.sto")
log("Saved to vestibular_detailed_explicit_1ms.sto", "logs/log_vestibular_explicit_1ms.txt")
