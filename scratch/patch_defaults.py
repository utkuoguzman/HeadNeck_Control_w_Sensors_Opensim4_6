# Update default values in scratch/HeadNeckNeuralController_plugin_split/HeadNeckNeuralController.cpp
path = "scratch/HeadNeckNeuralController_plugin_split/HeadNeckNeuralController.cpp"
with open(path, "r", encoding="utf-8") as f:
    c = f.read()

target = """    constructProperty_use_smith_predictor(false);
    constructProperty_delay_smith_proprio(0.015); // Disabled by default during neck study; set true for eye-movement study
    constructProperty_delay_afferent_proprio(0.0);"""

replacement = """    constructProperty_use_smith_predictor(true);
    constructProperty_delay_smith_proprio(0.007);
    constructProperty_delay_afferent_proprio(0.007);"""

if target in c:
    c = c.replace(target, replacement)
    with open(path, "w", encoding="utf-8") as f:
        f.write(c)
    print("Default properties updated to anatomically correct Smith-augmented model!")
else:
    print("Error: target not found in cpp!")

