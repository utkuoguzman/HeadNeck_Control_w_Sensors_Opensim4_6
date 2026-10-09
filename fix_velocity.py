import re

filepath = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp"

with open(filepath, "r") as f:
    content = f.read()

# 1. Revert s.getY() to getStateVariableValue
content = re.sub(r's\.getY\(\)\[getStateVariableSystemIndex\(([^)]+)\)\]', r'getStateVariableValue(s, \1)', content)

# Also revert spindle fix:
content = re.sub(r's\.getY\(\)\[spindle->getStateVariableSystemIndex\(([^)]+)\)\]', r'spindle->getStateVariableValue(s, \1)', content)

# 2. Change Stage::Dynamics to SimTK::Stage::Velocity in addStateVariable
content = re.sub(r'addStateVariable\(([^,]+),\s*Stage::Dynamics\)', r'addStateVariable(\1, SimTK::Stage::Velocity)', content)

with open(filepath, "w") as f:
    f.write(content)

print("Applied Stage::Velocity fix to HeadNeckNeuralController.cpp")

