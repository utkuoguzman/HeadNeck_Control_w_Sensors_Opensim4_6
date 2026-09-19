import opensim as osim
import os

# Load the plugin
plugin_path = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\vestibular_plugin\build\Release\osimVestibular.dll"
osim.LoadOpenSimLibrary(plugin_path)

# Load the model
model_path = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\osim_files\HYOID_ScaledStrength_v1.2andEye_v1_IndependentEyes_Afferents.osim"
model = osim.Model(model_path)

# Create the Vestibular Afferent sensor
vestibular = osim.Schneider15VestibularAfferent()
vestibular.setName("vestibular_sensor")

# Try to find the head body (usually named "skull" or "head")
# Let's check for "skull" first, then "head"
head_frame = None
if model.getBodySet().contains("skull"):
    head_frame = model.getBodySet().get("skull")
elif model.getBodySet().contains("head"):
    head_frame = model.getBodySet().get("head")
else:
    # Print available bodies to debug
    print("Available bodies:")
    for i in range(model.getBodySet().getSize()):
        print(model.getBodySet().get(i).getName())
    raise RuntimeError("Could not find 'skull' or 'head' body in the model!")

# Connect the sensor to the head frame
vestibular.updSocket("head_frame").connect(head_frame)

# Add the sensor to the model
model.addComponent(vestibular)

# Create a TableReporter to record the firing rates
reporter = osim.TableReporterVec3()
reporter.setName("vestibular_reporter")
reporter.set_report_time_interval(0.01)
reporter.updInput("inputs").connect(vestibular.getOutput("canal_firing_rate"), "canal")
reporter.updInput("inputs").connect(vestibular.getOutput("otolith_firing_rate"), "otolith")
model.addComponent(reporter)

# Initialize the system
state = model.initSystem()

# Run a quick simulation (e.g. 0 to 0.5s) to test
manager = osim.Manager(model)
manager.initialize(state)
print("Integrating from 0 to 0.5s...")
manager.integrate(0.5)

# Print the results
table = reporter.getTable()
osim.STOFileAdapterVec3.write(table, "vestibular_outputs.sto")
print("Successfully generated vestibular_outputs.sto!")

