import os
with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "r") as f:
    code = f.read()

state_vars_str = '''addStateVariable("yaw_error_integral", Stage::Dynamics);
    
    // Neural Filter States (Pitch, Roll, Yaw)
    addStateVariable("ton_state_pitch", Stage::Dynamics);
    addStateVariable("ton_state_roll", Stage::Dynamics);
    addStateVariable("ton_state_yaw", Stage::Dynamics);
    addStateVariable("sc_state_pitch", Stage::Dynamics);
    addStateVariable("sc_state_roll", Stage::Dynamics);
    addStateVariable("sc_state_yaw", Stage::Dynamics);
    addStateVariable("phas_state_pitch", Stage::Dynamics);
    addStateVariable("phas_state_roll", Stage::Dynamics);
    addStateVariable("phas_state_yaw", Stage::Dynamics);'''

code = code.replace('addStateVariable("yaw_error_integral", Stage::Dynamics);', state_vars_str)

with open("HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp", "w") as f:
    f.write(code)

print("States injected!")

