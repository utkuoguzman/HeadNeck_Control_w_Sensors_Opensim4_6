filepath = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp"

with open(filepath, "r") as f:
    content = f.read()

# Fix spindle reads
import re
content = re.sub(r's\.getY\(\)\[spindle->getStateVariableSystemIndex\(([^)]+)\)\]', r'spindle->getStateVariableValue(s, \1)', content)

with open(filepath, "w") as f:
    f.write(content)

