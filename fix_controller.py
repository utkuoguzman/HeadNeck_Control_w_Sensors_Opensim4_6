import sys
import re

def fix_cpp(filename):
    with open(filename, 'r') as f:
        content = f.read()
        
    # Replace the constructor properties
    content = content.replace("constructProperty_omega_n_pitch(4.47);", "constructProperty_Wp_base_pitch(400.0);\n    constructProperty_Wv_base_pitch(40.0);")
    content = content.replace("constructProperty_zeta_pitch(1.0);", "constructProperty_k_vigor(0.0);")
    content = content.replace("constructProperty_omega_n_roll(4.47);", "constructProperty_Wp_base_roll(400.0);\n    constructProperty_Wv_base_roll(40.0);")
    content = content.replace("constructProperty_zeta_roll(1.0);", "")
    content = content.replace("constructProperty_omega_n_yaw(4.47);", "constructProperty_Wp_base_yaw(400.0);\n    constructProperty_Wv_base_yaw(40.0);")
    content = content.replace("constructProperty_zeta_yaw(1.0);", "constructProperty_trajectory_tau(0.1);")

    # Add states
    add_states = """
    addStateVariable("traj_pos_pitch", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_pitch", SimTK::Stage::Velocity);
    addStateVariable("traj_pos_roll", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_roll", SimTK::Stage::Velocity);
    addStateVariable("traj_pos_yaw", SimTK::Stage::Velocity);
    addStateVariable("traj_vel_yaw", SimTK::Stage::Velocity);
"""
    content = content.replace('addStateVariable("pitch_error_integral", SimTK::Stage::Velocity);', add_states + '    addStateVariable("pitch_error_integral", SimTK::Stage::Velocity);')

    # Init states
    init_states = """
    setStateVariableValue(s, "traj_pos_pitch", 0.0);
    setStateVariableValue(s, "traj_vel_pitch", 0.0);
    setStateVariableValue(s, "traj_pos_roll", 0.0);
    setStateVariableValue(s, "traj_vel_roll", 0.0);
    setStateVariableValue(s, "traj_pos_yaw", 0.0);
    setStateVariableValue(s, "traj_vel_yaw", 0.0);
"""
    content = content.replace('setStateVariableValue(s, "obs_q0", 1.0);', init_states + '    setStateVariableValue(s, "obs_q0", 1.0);')

    injection = """
    double tau_traj = get_trajectory_tau();
    if (tau_traj < 1e-4) tau_traj = 1e-4;
    double traj_pos_pitch = getStateVariableValue(s, "traj_pos_pitch");
    double traj_vel_pitch = getStateVariableValue(s, "traj_vel_pitch");
    double traj_acc_pitch = (1.0 / (tau_traj * tau_traj)) * (get_desired_pitch() - traj_pos_pitch) - (2.0 / tau_traj) * traj_vel_pitch;
    
    double traj_pos_roll = getStateVariableValue(s, "traj_pos_roll");
    double traj_vel_roll = getStateVariableValue(s, "traj_vel_roll");
    double traj_acc_roll = (1.0 / (tau_traj * tau_traj)) * (get_desired_roll() - traj_pos_roll) - (2.0 / tau_traj) * traj_vel_roll;

    double traj_pos_yaw = getStateVariableValue(s, "traj_pos_yaw");
    double traj_vel_yaw = getStateVariableValue(s, "traj_vel_yaw");
    double traj_acc_yaw = (1.0 / (tau_traj * tau_traj)) * (get_desired_yaw() - traj_pos_yaw) - (2.0 / tau_traj) * traj_vel_yaw;
"""

    idx1 = content.find('void HeadNeckNeuralController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const {')
    insert_pos1 = content.find('{', idx1) + 1
    content = content[:insert_pos1] + injection + content[insert_pos1:]

    # For computeStateVariableDerivatives, we do the same
    idx2 = content.find('void HeadNeckNeuralController::computeStateVariableDerivatives(const SimTK::State& s) const {')
    insert_pos2 = content.find('{', idx2) + 1
    content = content[:insert_pos2] + injection + content[insert_pos2:]

    # Replace kp/kd computation and tau_vol calculation
    
    old_tau_vol = """      // Pitch uses the dynamically tuned omega_n and zeta.
      double kp_pitch = I_head_pitch * get_omega_n_pitch() * get_omega_n_pitch();
      double kd_pitch = I_head_pitch * 2.0 * get_zeta_pitch() * get_omega_n_pitch();
      tau_vol[0] = kp_pitch * (get_desired_pitch() - head_pitch) 
                 + kd_pitch * (get_desired_pitch_v() - omega[2]) 
                 + I_head_pitch * get_desired_pitch_a();
                 
      double kp_roll = I_head_roll * get_omega_n_roll() * get_omega_n_roll();
      double kd_roll = I_head_roll * 2.0 * get_zeta_roll() * get_omega_n_roll();
      tau_vol[1] = kp_roll * (get_desired_roll() - head_roll) 
                 + kd_roll * (get_desired_roll_v() - omega[0]) 
                 + I_head_roll * get_desired_roll_a();
                 
      double kp_yaw = I_head_yaw * get_omega_n_yaw() * get_omega_n_yaw();
      double kd_yaw = I_head_yaw * 2.0 * get_zeta_yaw() * get_omega_n_yaw();
      tau_vol[2] = kp_yaw * (get_desired_yaw() - head_yaw) 
                 + kd_yaw * (get_desired_yaw_v() - omega[1]) 
                 + I_head_yaw * get_desired_yaw_a();"""

    new_tau_vol = """      
      double vigor_pitch = 1.0 + get_k_vigor() * std::abs(traj_vel_pitch);
      double kp_pitch_lqr = std::sqrt(get_Wp_base_pitch() * vigor_pitch);
      double kd_pitch_lqr = std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch_lqr);
      double kp_pitch = I_head_pitch * kp_pitch_lqr;
      double kd_pitch = I_head_pitch * kd_pitch_lqr;
      tau_vol[0] = kp_pitch * (traj_pos_pitch - head_pitch) 
                 + kd_pitch * (traj_vel_pitch - omega[2]) 
                 + I_head_pitch * traj_acc_pitch;
                 
      double vigor_roll = 1.0 + get_k_vigor() * std::abs(traj_vel_roll);
      double kp_roll_lqr = std::sqrt(get_Wp_base_roll() * vigor_roll);
      double kd_roll_lqr = std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll_lqr);
      double kp_roll = I_head_roll * kp_roll_lqr;
      double kd_roll = I_head_roll * kd_roll_lqr;
      tau_vol[1] = kp_roll * (traj_pos_roll - head_roll) 
                 + kd_roll * (traj_vel_roll - omega[0]) 
                 + I_head_roll * traj_acc_roll;
                 
      double vigor_yaw = 1.0 + get_k_vigor() * std::abs(traj_vel_yaw);
      double kp_yaw_lqr = std::sqrt(get_Wp_base_yaw() * vigor_yaw);
      double kd_yaw_lqr = std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw_lqr);
      double kp_yaw = I_head_yaw * kp_yaw_lqr;
      double kd_yaw = I_head_yaw * kd_yaw_lqr;
      tau_vol[2] = kp_yaw * (traj_pos_yaw - head_yaw) 
                 + kd_yaw * (traj_vel_yaw - omega[1]) 
                 + I_head_yaw * traj_acc_yaw;"""
    
    content = content.replace(old_tau_vol, new_tau_vol)
    
    # We also need to fix ton and sc usages of get_desired_pitch etc!
    content = content.replace("get_desired_pitch()", "traj_pos_pitch")
    content = content.replace("get_desired_pitch_v()", "traj_vel_pitch")
    content = content.replace("get_desired_pitch_a()", "traj_acc_pitch")

    content = content.replace("get_desired_roll()", "traj_pos_roll")
    content = content.replace("get_desired_roll_v()", "traj_vel_roll")
    content = content.replace("get_desired_roll_a()", "traj_acc_roll")

    content = content.replace("get_desired_yaw()", "traj_pos_yaw")
    content = content.replace("get_desired_yaw_v()", "traj_vel_yaw")
    content = content.replace("get_desired_yaw_a()", "traj_acc_yaw")
    
    # But wait! I replaced `get_desired_pitch()` everywhere, including the injection! 
    # Let's fix the injection back to use getProperty_desired_pitch() or get_desired_pitch().
    # Actually, let's just do:
    content = content.replace("getProperty_desired_pitch()", "get_desired_pitch()")
    content = content.replace("getProperty_desired_roll()", "get_desired_roll()")
    content = content.replace("getProperty_desired_yaw()", "get_desired_yaw()")
    # Wait, the injection in the python script used get_desired_pitch(). So it became traj_pos_pitch - traj_pos_pitch.
    # Let's fix it properly.
    content = content.replace("traj_pos_pitch - traj_pos_pitch", "get_desired_pitch() - traj_pos_pitch")
    content = content.replace("traj_pos_roll - traj_pos_roll", "get_desired_roll() - traj_pos_roll")
    content = content.replace("traj_pos_yaw - traj_pos_yaw", "get_desired_yaw() - traj_pos_yaw")

    # Add derivatives for the trajectory planner
    derivatives = """
    setStateVariableDerivativeValue(s, "traj_pos_pitch", traj_vel_pitch);
    setStateVariableDerivativeValue(s, "traj_vel_pitch", traj_acc_pitch);
    setStateVariableDerivativeValue(s, "traj_pos_roll", traj_vel_roll);
    setStateVariableDerivativeValue(s, "traj_vel_roll", traj_acc_roll);
    setStateVariableDerivativeValue(s, "traj_pos_yaw", traj_vel_yaw);
    setStateVariableDerivativeValue(s, "traj_vel_yaw", traj_acc_yaw);
"""
    # put it right before `// Compute derivatives for PID error integrals` in computeStateVariableDerivatives
    idx3 = content.find('// Compute derivatives for PID error integrals')
    if idx3 != -1:
        content = content[:idx3] + derivatives + content[idx3:]

    with open(filename, 'w') as f:
        f.write(content)

if __name__ == '__main__':
    fix_cpp(sys.argv[1])

