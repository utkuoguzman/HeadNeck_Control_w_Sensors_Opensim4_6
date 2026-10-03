import os
import re

path = r'HeadNeckNeuralController_plugin\HeadNeckNeuralController.cpp'
with open(path, 'r') as f:
    content = f.read()

# 1. computeStateVariableDerivatives - Restore Integral Error States to Proprioception
old_integral = """    // Get biologically estimated orientation from SE_2(3) Observer
    double q0 = getStateVariableValue(s, "obs_q0");
    double q1 = getStateVariableValue(s, "obs_q1");
    double q2 = getStateVariableValue(s, "obs_q2");
    double q3 = getStateVariableValue(s, "obs_q3");
    double norm_q = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm_q > 1e-4) { q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q; }
    
    double estimated_roll = std::atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2));
    double estimated_yaw = std::asin(std::clamp(2.0*(q0*q2 - q3*q1), -1.0, 1.0));
    double estimated_pitch = std::atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3));

    double p_err = get_desired_pitch() - estimated_pitch;"""

new_integral = """    double head_pitch = q_p1 + q_p2;
    double head_roll = q_r1 + q_r2;
    double head_yaw = q_y1 + q_y2;
    
    double p_err = get_desired_pitch() - head_pitch;"""
content = content.replace(old_integral, new_integral)

content = content.replace("double r_err = get_desired_roll() - estimated_roll;", "double r_err = get_desired_roll() - head_roll;")
content = content.replace("double y_err = get_desired_yaw() - estimated_yaw;", "double y_err = get_desired_yaw() - head_yaw;")

# 3. computeControls - Restore tau_vol calculation to Proprioception
old_tau_vol = """    // Pitch uses the dynamically tuned Kp_task. Tracking is now 100% biologically driven by the Observer!
    tau_vol[0] = get_Kp_task() * (get_desired_pitch() - estimated_pitch) 
               + get_Kd_task() * (get_desired_pitch_v() - (vcr_record.omega_recon.size()==3 ? vcr_record.omega_recon[2] : 0.0)) 
               + get_Ki_task() * pitch_int 
               + I_head_pitch * get_desired_pitch_a();
               
    tau_vol[1] = get_Kp_task() * (get_desired_roll() - estimated_roll) 
               + get_Kd_task() * (get_desired_roll_v() - (vcr_record.omega_recon.size()==3 ? vcr_record.omega_recon[0] : 0.0)) 
               + get_Ki_task() * roll_int 
               + I_head_roll * get_desired_roll_a();
               
    tau_vol[2] = get_Kp_task() * (get_desired_yaw() - estimated_yaw) 
               + get_Kd_task() * (get_desired_yaw_v() - (vcr_record.omega_recon.size()==3 ? vcr_record.omega_recon[1] : 0.0)) 
               + get_Ki_task() * yaw_int 
               + I_head_yaw * get_desired_yaw_a();"""

new_tau_vol = """    // Pitch uses the dynamically tuned Kp_task. Now tracking velocity and accel properly (Proprioceptive Tracking).
    tau_vol[0] = get_Kp_task() * (get_desired_pitch() - head_pitch) 
               + get_Kd_task() * (get_desired_pitch_v() - omega[2]) 
               + get_Ki_task() * pitch_int 
               + I_head_pitch * get_desired_pitch_a();
               
    tau_vol[1] = get_Kp_task() * (get_desired_roll() - head_roll) + get_Kd_task() * (get_desired_roll_v() - omega[0]) + get_Ki_task() * roll_int + I_head_roll * get_desired_roll_a();
               
    tau_vol[2] = get_Kp_task() * (get_desired_yaw() - head_yaw) + get_Kd_task() * (get_desired_yaw_v() - omega[1]) + get_Ki_task() * yaw_int + I_head_yaw * get_desired_yaw_a();"""
content = content.replace(old_tau_vol, new_tau_vol)

with open(path, 'w') as f:
    f.write(content)
print("Proprioceptive tracking restored!")
