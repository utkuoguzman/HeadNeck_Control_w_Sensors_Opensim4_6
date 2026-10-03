import os
import re

path = r'HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp'
with open(path, 'r') as f:
    content = f.read()

# 4. computeControls - Mahony variable extraction
old_mahony = """    // Extract Estimated Angles from Mahony Quaternion
    double q0 = getStateVariableValue(s, "mahony_q0");
    double q1 = getStateVariableValue(s, "mahony_q1");
    double q2 = getStateVariableValue(s, "mahony_q2");
    double q3 = getStateVariableValue(s, "mahony_q3");"""
new_mahony = """    // Extract Estimated Angles from SE_2(3) Observer
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    
    double v_est_x = getStateVariableValue(s, "obs_vx");
    double v_est_y = getStateVariableValue(s, "obs_vy");
    double v_est_z = getStateVariableValue(s, "obs_vz");"""
content = content.replace(old_mahony, new_mahony)

# 5. computeControls - G_phas
pattern_gphas = r"// G_phas \(Otolith Phasic\) dampens linear acceleration.*?ah_z = .*?;"
replacement_gphas = """// G_phas (Otolith Phasic) dampens linear acceleration
        // BIOLOGICAL REALISM: The controller now derives pure acceleration from the otoliths
        // using the SE_2(3) spatial observer. It no longer relies on ground-truth physics!
        
        // Simulate raw otolith specific force
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world); 
        
        // Derive pure biological acceleration using our internal orientation belief
        SimTK::Rotation R_est(SimTK::Quaternion(q0, q1, q2, q3)); 
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world); // R_est is World->Head, so R_est * g_world is g in head frame
        
        double ah_x = a_lin_head[0];
        double ah_y = a_lin_head[1];
        double ah_z = a_lin_head[2];"""
content = re.sub(pattern_gphas, replacement_gphas, content, flags=re.MULTILINE|re.DOTALL)

with open(path, 'w') as f:
    f.write(content)
print("Updated computeControls!")
