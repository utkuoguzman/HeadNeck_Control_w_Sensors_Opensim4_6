import os
path = r'HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp'
with open(path, 'r') as f:
    content = f.read()

content = content.replace("    }\n\nvoid HeadNeckNeuralController::computeControls", "    }\n}\n\nvoid HeadNeckNeuralController::computeControls")

with open(path, 'w') as f:
    f.write(content)
print("Brace fixed!")
