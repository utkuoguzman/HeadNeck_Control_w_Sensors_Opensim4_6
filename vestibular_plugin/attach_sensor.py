import os
import sys

if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(r"C:\OpenSim 4.6\bin")

import opensim as osim

# Load the plugin
plugin_path = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\vestibular_plugin\build\Release\osimVestibular.dll"
osim.LoadOpenSimLibrary(plugin_path)

# Load the model
model_path = r"D:\Akademik\PhD_Thesis\OpenSim\Proprioception_Plugin_Opensim4_6_Explicit\osim_files\HYOID_ScaledStrength_v1.2andEye_v1_IndependentEyes_Afferents.osim"
model = osim.Model(model_path)

# Create the Vestibular Afferent sensor
vestibular = osim.Schneider15VestibularAfferent()
vestibular.setName("vestibular_sensor")

# Try to find the head body
head_frame = None
if model.getBodySet().contains("skull"):
    head_frame = model.getBodySet().get("skull")
elif model.getBodySet().contains("head"):
    head_frame = model.getBodySet().get("head")
else:
    raise RuntimeError("Could not find 'skull' or 'head' body in the model!")

# Connect the sensor to the head frame
vestibular.updSocket("head_frame").connect(head_frame)

# Add the sensor to the model
model.addComponent(vestibular)

# Save the model
model.printToXML(model_path)
print(f"Successfully attached vestibular_sensor to '{head_frame.getName()}' and saved {model_path}!")

