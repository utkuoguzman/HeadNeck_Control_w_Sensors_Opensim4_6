import re

filepath = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp"

with open(filepath, "r") as f:
    content = f.read()

# 1. Change SimTK::Stage::Time back to Stage::Dynamics
content = content.replace("SimTK::Stage::Time", "Stage::Dynamics")

# 2. Replace getStateVariableValue(s, "xyz") with s.getY()[getStateVariableSystemIndex("xyz")]
# Regex to match getStateVariableValue(s, ...)
# It can be a string literal or a string expression.
# For simplicity, we just find all occurrences of:
# getStateVariableValue(s, EXPRESSION)
# And replace with:
# s.getY()[getStateVariableSystemIndex(EXPRESSION)]

content = re.sub(r'getStateVariableValue\(s,\s*([^)]+)\)', r's.getY()[getStateVariableSystemIndex(\1)]', content)

# 3. Handle spindle->getStateVariableValue(s, "tension_bag1") -> s.getY()[spindle->getStateVariableSystemIndex("tension_bag1")]
content = re.sub(r'spindle->s\.getY\(\)\[getStateVariableSystemIndex\(([^)]+)\)\]', r's.getY()[spindle->getStateVariableSystemIndex(\1)]', content)

with open(filepath, "w") as f:
    f.write(content)

print("Applied bypass cache fix to HeadNeckNeuralController.cpp")

