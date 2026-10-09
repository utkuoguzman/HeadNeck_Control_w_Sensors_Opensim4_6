import re

filepath = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp"

with open(filepath, "r") as f:
    content = f.read()

insert_str = """
    addStateVariable("ton_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("ton_state_roll", SimTK::Stage::Velocity);
    addStateVariable("ton_state_yaw", SimTK::Stage::Velocity);
    addStateVariable("sc_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("sc_state_roll", SimTK::Stage::Velocity);
    addStateVariable("sc_state_yaw", SimTK::Stage::Velocity);
    addStateVariable("phas_state_pitch", SimTK::Stage::Velocity);
    addStateVariable("phas_state_roll", SimTK::Stage::Velocity);
    addStateVariable("phas_state_yaw", SimTK::Stage::Velocity);
"""

# Find the end of constructor
pos = content.find("addStateVariable(\"eso_tau_dist_yaw\", SimTK::Stage::Velocity);")
if pos != -1:
    end_pos = content.find("\n", pos) + 1
    content = content[:end_pos] + insert_str + content[end_pos:]

with open(filepath, "w") as f:
    f.write(content)

print("Added missing state variables to constructor")

