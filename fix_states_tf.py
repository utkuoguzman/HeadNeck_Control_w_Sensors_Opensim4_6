import re
import sys

filepath = r"d:\Akademik\PhD_Thesis\OpenSim\HeadNeck_Control_w_Sensors_Opensim4_6\HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp"

with open(filepath, "r") as f:
    content = f.read()

# 1. Change addStateVariable("...", Stage::Dynamics) to addStateVariable("...", SimTK::Stage::Time)
content = re.sub(r'addStateVariable\(([^,]+),\s*Stage::Dynamics\)', r'addStateVariable(\1, SimTK::Stage::Time)', content)

# 2. Add the missing derivatives for ton, sc, phas to computeStateVariableDerivatives
# We need to find the end of computeStateVariableDerivatives
end_of_derivatives = content.find("void HeadNeckNeuralController::computeControls")
if end_of_derivatives == -1:
    print("Could not find computeControls")
    sys.exit(1)

# Find where the SE_2(3) spatial observer ends in computeStateVariableDerivatives
obs_end = content.find("setStateVariableDerivativeValue(s, \"obs_pz\", 0.0);", 0, end_of_derivatives)
if obs_end == -1:
    print("Could not find obs_pz in computeStateVariableDerivatives")
    sys.exit(1)

insert_pos = content.find("}", obs_end) + 1 # end of the else block

new_derivatives = """
    // --- NEURAL TRANSFER FUNCTION DERIVATIVES ---
    if (vcr_data.record.omega_recon.size() == 3) {
        double est_roll = vcr_data.record.theta[0];
        double est_yaw = vcr_data.record.theta[1];
        double est_pitch = vcr_data.record.theta[2];
        
        double u_ton_pitch = (est_pitch - baseline_spatial_orientation[2]) - get_desired_pitch();
        double u_ton_roll  = (est_roll - baseline_spatial_orientation[0]) - get_desired_roll();
        double u_ton_yaw   = (est_yaw - baseline_spatial_orientation[1]) - get_desired_yaw();
        
        double u_sc_pitch = vcr_data.record.omega_recon[2] - get_desired_pitch_v();
        double u_sc_roll  = vcr_data.record.omega_recon[0] - get_desired_roll_v();
        double u_sc_yaw   = vcr_data.record.omega_recon[1] - get_desired_yaw_v();
        
        const Body& skull = model.getBodySet().get("skull");
        SimTK::Rotation R_world_to_head = skull.getMobilizedBody().getBodyTransform(s).R().invert();
        SimTK::Vec3 g_world(0.0, -9.81, 0.0);
        SimTK::Vec3 f_oto = R_world_to_head * (vcr_data.a_lin - g_world);
        
        SimTK::Rotation R_est;
        R_est.setRotationToBodyFixedXYZ(SimTK::Vec3(est_roll, est_yaw, est_pitch));
        SimTK::Vec3 a_lin_head = f_oto + (R_est * g_world);
        
        double r_head = 0.15; 
        double expected_ah_x = get_desired_pitch_a() * r_head; 
        double expected_ah_z = get_desired_roll_a() * r_head;
        
        double u_phas_pitch = a_lin_head[0] - expected_ah_x;
        double u_phas_roll  = a_lin_head[2] - expected_ah_z;
        double u_phas_yaw   = 0.0;

        double x_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
        double x_ton_roll  = getStateVariableValue(s, "ton_state_roll");
        double x_ton_yaw   = getStateVariableValue(s, "ton_state_yaw");
        
        double x_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
        double x_sc_roll  = getStateVariableValue(s, "sc_state_roll");
        double x_sc_yaw   = getStateVariableValue(s, "sc_state_yaw");
        
        double x_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
        double x_phas_roll  = getStateVariableValue(s, "phas_state_roll");
        double x_phas_yaw   = getStateVariableValue(s, "phas_state_yaw");

        setStateVariableDerivativeValue(s, "ton_state_pitch", -get_p_ton() * x_ton_pitch + u_ton_pitch);
        setStateVariableDerivativeValue(s, "ton_state_roll", -get_p_ton() * x_ton_roll + u_ton_roll);
        setStateVariableDerivativeValue(s, "ton_state_yaw", -get_p_ton() * x_ton_yaw + u_ton_yaw);

        setStateVariableDerivativeValue(s, "sc_state_pitch", -get_p_sc() * x_sc_pitch + u_sc_pitch);
        setStateVariableDerivativeValue(s, "sc_state_roll", -get_p_sc() * x_sc_roll + u_sc_roll);
        setStateVariableDerivativeValue(s, "sc_state_yaw", -get_p_sc() * x_sc_yaw + u_sc_yaw);

        setStateVariableDerivativeValue(s, "phas_state_pitch", -get_p_phas() * x_phas_pitch + u_phas_pitch);
        setStateVariableDerivativeValue(s, "phas_state_roll", -get_p_phas() * x_phas_roll + u_phas_roll);
        setStateVariableDerivativeValue(s, "phas_state_yaw", -get_p_phas() * x_phas_yaw + u_phas_yaw);
    } else {
        setStateVariableDerivativeValue(s, "ton_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "ton_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "sc_state_yaw", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_pitch", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_roll", 0.0);
        setStateVariableDerivativeValue(s, "phas_state_yaw", 0.0);
    }
"""

content = content[:insert_pos] + new_derivatives + content[insert_pos:]

# 3. We must also add the reads of x_ton_pitch etc to the TOP of computeStateVariableDerivatives
reads = """
    double local_ton_pitch = getStateVariableValue(s, "ton_state_pitch");
    double local_ton_roll = getStateVariableValue(s, "ton_state_roll");
    double local_ton_yaw = getStateVariableValue(s, "ton_state_yaw");
    double local_sc_pitch = getStateVariableValue(s, "sc_state_pitch");
    double local_sc_roll = getStateVariableValue(s, "sc_state_roll");
    double local_sc_yaw = getStateVariableValue(s, "sc_state_yaw");
    double local_phas_pitch = getStateVariableValue(s, "phas_state_pitch");
    double local_phas_roll = getStateVariableValue(s, "phas_state_roll");
    double local_phas_yaw = getStateVariableValue(s, "phas_state_yaw");
"""
reads_pos = content.find("double local_eff_tau_pitch = getStateVariableValue(s, \"eff_tau_pitch\");")
content = content[:reads_pos] + reads + content[reads_pos:]

# Update the new_derivatives block to use the local variables
content = content.replace("double x_ton_pitch = getStateVariableValue(s, \"ton_state_pitch\");", "double x_ton_pitch = local_ton_pitch;")
content = content.replace("double x_ton_roll  = getStateVariableValue(s, \"ton_state_roll\");", "double x_ton_roll = local_ton_roll;")
content = content.replace("double x_ton_yaw   = getStateVariableValue(s, \"ton_state_yaw\");", "double x_ton_yaw = local_ton_yaw;")
content = content.replace("double x_sc_pitch = getStateVariableValue(s, \"sc_state_pitch\");", "double x_sc_pitch = local_sc_pitch;")
content = content.replace("double x_sc_roll  = getStateVariableValue(s, \"sc_state_roll\");", "double x_sc_roll = local_sc_roll;")
content = content.replace("double x_sc_yaw   = getStateVariableValue(s, \"sc_state_yaw\");", "double x_sc_yaw = local_sc_yaw;")
content = content.replace("double x_phas_pitch = getStateVariableValue(s, \"phas_state_pitch\");", "double x_phas_pitch = local_phas_pitch;")
content = content.replace("double x_phas_roll  = getStateVariableValue(s, \"phas_state_roll\");", "double x_phas_roll = local_phas_roll;")
content = content.replace("double x_phas_yaw   = getStateVariableValue(s, \"phas_state_yaw\");", "double x_phas_yaw = local_phas_yaw;")


with open(filepath, "w") as f:
    f.write(content)

print("Applied fixes to HeadNeckNeuralController.cpp")

