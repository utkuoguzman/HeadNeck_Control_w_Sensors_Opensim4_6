import os
import re

# 1. Modify Header File
header_path = r'HeadNeckNeuralController_plugin\HeadNeckNeuralController.h'
with open(header_path, 'r') as f:
    h_content = f.read()

if "baseline_spatial_orientation" not in h_content:
    h_content = h_content.replace(
        "mutable double last_jacobian_update_time;", 
        "mutable double last_jacobian_update_time;\n    mutable SimTK::Vec3 baseline_spatial_orientation; // roll, yaw, pitch"
    )
    with open(header_path, 'w') as f:
        f.write(h_content)

# 2. Modify CPP File
cpp_path = r'HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp'
with open(cpp_path, 'r') as f:
    cpp_content = f.read()

# 2a. initializeGeometryAndBaseline
init_pattern = r"(void HeadNeckNeuralController::initializeGeometryAndBaseline\(const SimTK::State& s\) const \{\n    const Model& model = getModel\(\);)"
init_replacement = """\\1
    
    // Capture anatomical resting orientation as Vestibular Baseline!
    const Body& skull = model.getBodySet().get("skull");
    SimTK::Rotation R_head_to_world = skull.getMobilizedBody().getBodyTransform(s).R();
    SimTK::Quaternion base_q = R_head_to_world.convertRotationToQuaternion();
    double bq0 = base_q[0], bq1 = base_q[1], bq2 = base_q[2], bq3 = base_q[3];
    baseline_spatial_orientation[0] = std::atan2(2.0*(bq0*bq1 + bq2*bq3), 1.0 - 2.0*(bq1*bq1 + bq2*bq2)); // roll
    baseline_spatial_orientation[1] = std::asin(std::clamp(2.0*(bq0*bq2 - bq3*bq1), -1.0, 1.0)); // yaw
    baseline_spatial_orientation[2] = std::atan2(2.0*(bq0*bq3 + bq1*bq2), 1.0 - 2.0*(bq2*bq2 + bq3*bq3)); // pitch
"""
if "baseline_spatial_orientation" not in cpp_content:
    cpp_content = re.sub(init_pattern, init_replacement, cpp_content)

# 2b. computeControls G_ton block
g_ton_pattern = r"tau_vcr\[0\] -= get_G_ton\(\) \* \(estimated_pitch - get_desired_pitch\(\)\); // pitch\s+tau_vcr\[1\] -= get_G_ton\(\) \* \(estimated_roll - get_desired_roll\(\)\);\s+// roll\s+tau_vcr\[2\] -= get_G_ton\(\) \* \(estimated_yaw - get_desired_yaw\(\)\);\s+// yaw"
g_ton_replacement = """tau_vcr[0] -= get_G_ton() * ((estimated_pitch - baseline_spatial_orientation[2]) - get_desired_pitch()); // pitch
        tau_vcr[1] -= get_G_ton() * ((estimated_roll - baseline_spatial_orientation[0]) - get_desired_roll());  // roll
        tau_vcr[2] -= get_G_ton() * ((estimated_yaw - baseline_spatial_orientation[1]) - get_desired_yaw());   // yaw"""
cpp_content = re.sub(g_ton_pattern, g_ton_replacement, cpp_content)

with open(cpp_path, 'w') as f:
    f.write(cpp_content)

print("Added anatomical baseline offset to Vestibular reflex!")
