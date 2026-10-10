import re

def fix():
    with open('HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp', 'r') as f:
        content = f.read()

    # robust replacement
    
    # Pitch
    content = re.sub(
        r'double kp_pitch = I_head_pitch \* get_omega_n_pitch\(\) \* get_omega_n_pitch\(\);.*?I_head_pitch \* traj_acc_pitch;',
        r'''double vigor_pitch = 1.0 + get_k_vigor() * std::abs(traj_vel_pitch);
      double kp_pitch_lqr = std::sqrt(get_Wp_base_pitch() * vigor_pitch);
      double kd_pitch_lqr = std::sqrt(get_Wv_base_pitch() * vigor_pitch + 2.0 * kp_pitch_lqr);
      double kp_pitch = I_head_pitch * kp_pitch_lqr;
      double kd_pitch = I_head_pitch * kd_pitch_lqr;
      tau_vol[0] = kp_pitch * (get_desired_pitch() - head_pitch) 
                 + kd_pitch * (traj_vel_pitch - omega[2]) 
                 + I_head_pitch * traj_acc_pitch;''',
        content, flags=re.DOTALL
    )

    # Roll
    content = re.sub(
        r'double kp_roll = I_head_roll \* get_omega_n_roll\(\) \* get_omega_n_roll\(\);.*?I_head_roll \* traj_acc_roll;',
        r'''double vigor_roll = 1.0 + get_k_vigor() * std::abs(traj_vel_roll);
      double kp_roll_lqr = std::sqrt(get_Wp_base_roll() * vigor_roll);
      double kd_roll_lqr = std::sqrt(get_Wv_base_roll() * vigor_roll + 2.0 * kp_roll_lqr);
      double kp_roll = I_head_roll * kp_roll_lqr;
      double kd_roll = I_head_roll * kd_roll_lqr;
      tau_vol[1] = kp_roll * (get_desired_roll() - head_roll) 
                 + kd_roll * (traj_vel_roll - omega[0]) 
                 + I_head_roll * traj_acc_roll;''',
        content, flags=re.DOTALL
    )

    # Yaw
    content = re.sub(
        r'double kp_yaw = I_head_yaw \* get_omega_n_yaw\(\) \* get_omega_n_yaw\(\);.*?I_head_yaw \* traj_acc_yaw;',
        r'''double vigor_yaw = 1.0 + get_k_vigor() * std::abs(traj_vel_yaw);
      double kp_yaw_lqr = std::sqrt(get_Wp_base_yaw() * vigor_yaw);
      double kd_yaw_lqr = std::sqrt(get_Wv_base_yaw() * vigor_yaw + 2.0 * kp_yaw_lqr);
      double kp_yaw = I_head_yaw * kp_yaw_lqr;
      double kd_yaw = I_head_yaw * kd_yaw_lqr;
      tau_vol[2] = kp_yaw * (get_desired_yaw() - head_yaw) 
                 + kd_yaw * (traj_vel_yaw - omega[1]) 
                 + I_head_yaw * traj_acc_yaw;''',
        content, flags=re.DOTALL
    )
    
    with open('HeadNeckNeuralController_plugin/HeadNeckNeuralController.cpp', 'w') as f:
        f.write(content)

if __name__ == '__main__':
    fix()

