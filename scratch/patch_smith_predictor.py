import os

# 1. Patch HeadNeckNeuralController.h
header_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.h"
with open(header_path, "r", encoding="utf-8") as f:
    header_content = f.read()

target_h = 'OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "If true, includes extraocular eye muscles (r_*, l_*) in controller state allocation (default: false)");'
replacement_h = '''OpenSim_DECLARE_PROPERTY(enable_eye_afferents, bool, "If true, includes extraocular eye muscles (r_*, l_*) in controller state allocation (default: false)");
    OpenSim_DECLARE_PROPERTY(use_smith_predictor, bool, "If true, utilizes Smith Predictor delay compensation for Mahony proprioceptive fusion (default: false)");
    OpenSim_DECLARE_PROPERTY(delay_smith_proprio, double, "Proprioceptive transport delay in seconds for Smith compensation (default: 0.015)");'''

if target_h in header_content:
    header_content = header_content.replace(target_h, replacement_h)
    with open(header_path, "w", encoding="utf-8") as f:
        f.write(header_content)
    print("Patched HeadNeckNeuralController.h successfully!")
else:
    print("Warning: target_h not found in HeadNeckNeuralController.h")

# 2. Patch HeadNeckNeuralController.cpp
cpp_path = "HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp"
with open(cpp_path, "r", encoding="utf-8") as f:
    cpp_content = f.read()

# Add constructProperty in constructor
target_constructor = 'constructProperty_enable_eye_afferents(false);'
replacement_constructor = '''constructProperty_enable_eye_afferents(false);
    constructProperty_use_smith_predictor(false);
    constructProperty_delay_smith_proprio(0.015);'''

if target_constructor in cpp_content:
    cpp_content = cpp_content.replace(target_constructor, replacement_constructor)
    print("Patched constructor in HeadNeckNeuralController.cpp successfully!")
else:
    print("Warning: target_constructor not found in HeadNeckNeuralController.cpp")

# Patch extendComputeStateVariableDerivatives around line 850
target_eprop = '''    // Error vector in World frame [Roll=0, Yaw=1, Pitch=2]
    SimTK::Vec3 e_prop_world(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);'''

replacement_eprop = '''    // Error vector in World frame [Roll=0, Yaw=1, Pitch=2]
    SimTK::Vec3 e_prop_world;
    if (get_use_smith_predictor() && !history.empty()) {
        DelayedData prop_delayed = getDelayedData(s.getTime(), get_delay_smith_proprio());
        if (prop_delayed.record.valid) {
            double pred_roll  = prop_delayed.record.theta[0];
            double pred_yaw   = prop_delayed.record.theta[1];
            double pred_pitch = prop_delayed.record.theta[2];
            e_prop_world = SimTK::Vec3(prop_gnd_roll - pred_roll, prop_gnd_yaw - pred_yaw, prop_gnd_pitch - pred_pitch);
        } else {
            e_prop_world = SimTK::Vec3(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
        }
    } else {
        e_prop_world = SimTK::Vec3(prop_gnd_roll - est_roll, prop_gnd_yaw - est_yaw, prop_gnd_pitch - est_pitch);
    }'''

if target_eprop in cpp_content:
    cpp_content = cpp_content.replace(target_eprop, replacement_eprop)
    with open(cpp_path, "w", encoding="utf-8") as f:
        f.write(cpp_content)
    print("Patched e_prop in HeadNeckNeuralController.cpp successfully!")
else:
    print("Warning: target_eprop not found in HeadNeckNeuralController.cpp")

